/**
 * CAN Edge Gateway Master — Phase 2 (FreeRTOS)
 *
 * 6-task architecture:
 *   vTask_CAN_Rx (prio 5):    ISR→semaphore→FIFO
 *   vTask_CAN_Monitor (prio 4): bus load, error state, escalation
 *   vTask_CAN_Tx (prio 3):    FIFO→CAN transmit
 *   vTask_Protocol (prio 2):  CAN↔Modbus sync, cache, batch upload
 *   vTask_W5500 (prio 1):     TCP server, Modbus processing
 *   vTask_Housekeep (prio 0): watchdog, stack monitor, logs
 */

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "oled.h"
#include "CAN_User.h"
#include "W5500.h"
#include "ModbusTCP.h"
#include "fifo.h"
#include "stm32f10x_iwdg.h"
#include <stdio.h>
#include <string.h>

/* ---- Semihosting ---- */
#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }

/* ---- Task handles ---- */
static TaskHandle_t hTask_CAN_Rx     = NULL;
static TaskHandle_t hTask_CAN_Monitor = NULL;
static TaskHandle_t hTask_CAN_Tx     = NULL;
static TaskHandle_t hTask_Protocol   = NULL;
static TaskHandle_t hTask_W5500      = NULL;
static TaskHandle_t hTask_Housekeep  = NULL;

/* ---- Task stack sizes ---- */
#define STACK_CAN_RX        256
#define STACK_CAN_MONITOR   256
#define STACK_CAN_TX        256
#define STACK_PROTOCOL      512
#define STACK_W5500         512
#define STACK_HOUSEKEEP     256

/* ---- OLED ---- */
static char g_oled_line[4][17];

/* ---- W5500 state ---- */
static uint8_t g_eth_was_connected = 0;

/* ==================== OLED helper ==================== */

static const char *CAN_StateStr(void)
{
    switch (g_can_error.error_level) {
    case 0:  return "OK ";
    case 1:  return "WRN";
    case 2:  return "ERR";
    case 3:  return "BOF";
    default: return "???";
    }
}

static const char *ETH_StateStr(void)
{
    if (!W5500_IsOnline())   return "FAIL";
    if (!W5500_LinkUp())     return "NOLK";
    if (W5500_IsConnected()) return "CON ";
    return "LSN ";
}

static void OLED_UpdateDisplay(void)
{
    uint8_t online1 = g_slave_nodes[0].online;
    uint8_t fault1  = g_slave_nodes[0].fault_flag;

    /* Line 1: CAN state + Ethernet state */
    sprintf(g_oled_line[0], "CAN:%-3s ETH:%-4s", CAN_StateStr(), ETH_StateStr());

    /* Line 2: Slave 1 temperature + bus load */
    if (online1) {
        uint16_t load = g_can_error.bus_load;
        if (load < 1000) {
            sprintf(g_oled_line[1], "S1:%d.%dC L:%u.%02u%%",
                    g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec,
                    load / 100, load % 100);
        } else {
            uint16_t r = (load + 5) / 10;
            if (r >= 1000)
                sprintf(g_oled_line[1], "S1:%d.%dC L:100%%",
                        g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec);
            else
                sprintf(g_oled_line[1], "S1:%d.%dC L:%u.%u%%",
                        g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec,
                        r / 10, r % 10);
        }
    } else
        sprintf(g_oled_line[1], "S1:--.-C L:--%% ");

    /* Line 3: humidity + heartbeat */
    sprintf(g_oled_line[2], "H:%d.%d%% HB:%-5u",
            g_slave_nodes[0].humi_int, g_slave_nodes[0].humi_dec,
            g_slave_nodes[0].heartbeat_count);

    /* Line 4: CAN frame count + FIFO status + node state */
    {
        char r_str[5];
        const char *status;
        uint32_t rx = g_can_rx_int_count;
        uint8_t  hc = FIFO_High_Count();
        uint8_t  nc = FIFO_Normal_Count();

        if (rx >= 1000000)      sprintf(r_str, "%luM", rx/1000000);
        else if (rx >= 10000)   sprintf(r_str, "%luK", rx/1000);
        else if (rx >= 1000)    sprintf(r_str, "%lu.%luK", rx/1000, (rx%1000)/100);
        else                    sprintf(r_str, "%lu", (unsigned long)rx);

        if (!online1)                status = "OFF";
        else if (fault1)             status = "ALM";
        else if (g_slave_nodes[0].blacklist) status = "BLK";
        else                         status = "OK ";

        sprintf(g_oled_line[3], "R%-4s H:%-2u N:%-2u%s", r_str, hc, nc, status);
    }

    for (uint8_t i = 0; i < 4; i++) {
        g_oled_line[i][16] = '\0';
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

/* ==================== Task: CAN_Rx (priority 5) ==================== */

static void Task_CAN_Rx(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        /* Block until ISR signals new frame */
        if (xSemaphoreTake(g_can_rx_sem, portMAX_DELAY) == pdPASS) {
            CAN_ProcessFrame(&g_isr_rx_frame);
        }
    }
}

/* ==================== Task: CAN_Monitor (priority 4, 100ms) ==================== */

static void Task_CAN_Monitor(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    for (;;) {
        CAN_HeartBeatCheck();
        CAN_ErrorMonitor();
        CAN_CalcBusLoad();
        CAN_CheckEscalation();
        CAN_CheckDeescalation();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

/* ==================== Task: CAN_Tx (priority 3) ==================== */

static void Task_CAN_Tx(void *pvParameters)
{
    CanRxFrame frame;
    (void)pvParameters;

    for (;;) {
        /* Wait for FIFO not empty */
        if (xSemaphoreTake(g_can_fifo_not_empty, pdMS_TO_TICKS(10)) != pdPASS)
            continue;

        /* Drain FIFO: HIGH first, then NORMAL */
        while (FIFO_High_Pop(&frame) == 0) {
            /* Throttle check: in emergency mode, only allow priority-0 and heartbeat */
            if (g_system_throttle_level >= 2) {
                uint8_t func = frame.Data[CAN_DATA_FUNC_IDX];
                uint8_t prio = frame.Data[CAN_DATA_TYPE_IDX];
                if (prio != 0 && func != CAN_FUNC_HEARTBEAT)
                    continue;  /* Drop non-emergency, non-heartbeat frames */
            }
            CAN_SendFrame(frame.StdId, frame.Data, frame.DLC);
        }
        while (FIFO_Normal_Pop(&frame) == 0) {
            if (g_system_throttle_level >= 2) {
                uint8_t func = frame.Data[CAN_DATA_FUNC_IDX];
                uint8_t prio = frame.Data[CAN_DATA_TYPE_IDX];
                if (prio != 0 && func != CAN_FUNC_HEARTBEAT)
                    continue;
            }
            CAN_SendFrame(frame.StdId, frame.Data, frame.DLC);
        }
    }
}

/* ==================== Task: Protocol (priority 2, 50ms) ==================== */

static void Task_Protocol(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    for (;;) {
        ModbusTCP_SyncFromCAN();

        /* Check link state for connect/disconnect events */
        if (W5500_IsConnected()) {
            if (!g_eth_was_connected) {
                ModbusTCP_OnReconnect();
                g_eth_was_connected = 1;
            }
        } else {
            if (g_eth_was_connected) {
                ModbusTCP_OnDisconnect();
                g_eth_was_connected = 0;
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

/* ==================== Task: W5500 (priority 1) ==================== */

static void Task_W5500(void *pvParameters)
{
    (void)pvParameters;

    /* Wait for network init before starting the TCP server */
    vTaskDelay(pdMS_TO_TICKS(100));

    W5500_TCPServer_Start(MODBUS_PORT);

    for (;;) {
        /* Critical section for SPI integrity */
        taskENTER_CRITICAL();
        W5500_TCPServer_Run();
        taskEXIT_CRITICAL();

        taskENTER_CRITICAL();
        ModbusTCP_Process();
        taskEXIT_CRITICAL();

        vTaskDelay(pdMS_TO_TICKS(5));  /* ~200 polls/sec */
    }
}

/* ==================== Task: Housekeep (priority 0, 1s) ==================== */

static void Task_Housekeep(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    /* Initialize IWDG: ~1s timeout */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);     /* 40kHz / 64 = 625Hz */
    IWDG_SetReload(625);                       /* 625 / 625 = 1s */
    IWDG_ReloadCounter();
    IWDG_Enable();

    for (;;) {
        IWDG_ReloadCounter();

        /* Stack high-water checks (log via USART1 if available) */
        {
            UBaseType_t stacks[6];
            stacks[0] = uxTaskGetStackHighWaterMark(hTask_CAN_Rx);
            stacks[1] = uxTaskGetStackHighWaterMark(hTask_CAN_Monitor);
            stacks[2] = uxTaskGetStackHighWaterMark(hTask_CAN_Tx);
            stacks[3] = uxTaskGetStackHighWaterMark(hTask_Protocol);
            stacks[4] = uxTaskGetStackHighWaterMark(hTask_W5500);
            stacks[5] = uxTaskGetStackHighWaterMark(hTask_Housekeep);

            /* If any task has less than 20% stack remaining, log warning */
            for (uint8_t i = 0; i < 6; i++) {
                if (stacks[i] < 50) {
                    /* USART1 log: task stack low */
                    (void)stacks[i];  /* TODO: log to USART1 */
                }
            }
        }

        /* Update OLED */
        OLED_UpdateDisplay();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/* ==================== Stack overflow hook ==================== */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    /* Critical: task stack overflow detected */
    taskDISABLE_INTERRUPTS();
    OLED_Clear();
    OLED_ShowString(1, 1, "STACK OVERFLOW!");
    OLED_ShowString(2, 1, pcTaskName ? pcTaskName : "???");
    OLED_ShowString(3, 1, "System Halted");
    for (;;) { }
}

/* ==================== malloc failed hook ==================== */

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    OLED_Clear();
    OLED_ShowString(1, 1, "MALLOC FAILED!");
    for (;;) { }
}

/* ==================== Main ==================== */

int main(void)
{
    int8_t ret;

    /* Hardware init */
    Delay_InitTick();
    Delay_ms(200);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "M: Init...");

    CAN_User_Init();
    OLED_ShowString(2, 1, "   CAN OK");

    /* W5500 init */
    ret = W5500_Init();
    if (ret != W5500_OK) {
        OLED_ShowString(3, 1, "   W5500 FAIL!");
        char diag[17];
        sprintf(diag, "   Ver:0x%02X", g_w5500_version);
        OLED_ShowString(4, 1, diag);
        while (1);
    }

    ret = W5500_ConfigNetwork();
    if (ret != W5500_OK) {
        OLED_ShowString(3, 1, "   NET CFG ERR");
    } else {
        OLED_ShowString(3, 1, "   ETH OK");
        OLED_ShowString(4, 1, "FreeRTOS...");
    }

    ModbusTCP_Init();
    g_eth_was_connected = 0;

    /* Create FreeRTOS tasks (see README priority table) */
    xTaskCreate(Task_CAN_Rx,      "CAN_Rx",   STACK_CAN_RX,    NULL, 5, &hTask_CAN_Rx);
    xTaskCreate(Task_CAN_Monitor, "CAN_Mon",  STACK_CAN_MONITOR, NULL, 4, &hTask_CAN_Monitor);
    xTaskCreate(Task_CAN_Tx,      "CAN_Tx",   STACK_CAN_TX,    NULL, 3, &hTask_CAN_Tx);
    xTaskCreate(Task_Protocol,    "Protocol", STACK_PROTOCOL,  NULL, 2, &hTask_Protocol);
    xTaskCreate(Task_W5500,       "W5500",    STACK_W5500,     NULL, 1, &hTask_W5500);
    xTaskCreate(Task_Housekeep,   "HouseKp",  STACK_HOUSEKEEP, NULL, 0, &hTask_Housekeep);

    OLED_Clear();
    OLED_ShowString(1, 1, "M: Starting...");
    OLED_ShowString(2, 1, "FreeRTOS v10.4");

    /* Start the scheduler — never returns */
    vTaskStartScheduler();

    /* Should never reach here */
    while (1) { }
}
