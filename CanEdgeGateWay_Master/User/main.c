/**
 * CAN Edge Gateway Master — Phase 2 FULL
 *
 * 6-task architecture with all fixes applied:
 *   vTask_CAN_Rx (prio 5):    ISR→semaphore→FIFO
 *   vTask_CAN_Monitor (prio 4): bus load, error state, escalation
 *   vTask_CAN_Tx (prio 3):    FIFO→CAN transmit
 *   vTask_Protocol (prio 2):  CAN↔Modbus sync, cache, batch upload
 *   vTask_W5500 (prio 1):     TCP server, Modbus processing
 *   vTask_Housekeep (prio 0): OLED display, stack monitor
 */

#include "stm32f10x.h"
#include "FreeRTOSConfig.h"
#include "delay.h"
#include "oled.h"
#include "CAN_User.h"
#include "W5500.h"
#include "ModbusTCP.h"
#include "fifo.h"
#include <stdio.h>
#include <string.h>

#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }

static TaskHandle_t hTask_CAN_Rx     = NULL;
static TaskHandle_t hTask_CAN_Monitor = NULL;
static TaskHandle_t hTask_CAN_Tx     = NULL;
static TaskHandle_t hTask_Protocol   = NULL;
static TaskHandle_t hTask_W5500      = NULL;
static TaskHandle_t hTask_Housekeep  = NULL;

#define STACK_CAN_RX        320
#define STACK_CAN_MONITOR   320
#define STACK_CAN_TX        320
#define STACK_PROTOCOL      448
#define STACK_W5500         448
#define STACK_HOUSEKEEP     384

static char g_oled_line[4][17];
static uint8_t g_eth_was_connected = 0;

extern size_t xPortGetFreeHeapSize(void);

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
    if (!W5500_IsLinkUpCached()) return "NOLK";  /* cached, no SPI */
    if (W5500_IsConnected()) return "CON ";
    return "LSN ";
}

static void OLED_UpdateDisplay(void)
{
    uint8_t online1 = g_slave_nodes[0].online;
    uint8_t fault1  = g_slave_nodes[0].fault_flag;

    sprintf(g_oled_line[0], "CAN:%-3s ETH:%-4s", CAN_StateStr(), ETH_StateStr());
    if (online1) {
        uint16_t load = g_can_error.bus_load;
        if (load < 1000)
            sprintf(g_oled_line[1], "S1:%d.%dC L:%u.%02u%%",
                    g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec, load/100, load%100);
        else {
            uint16_t r = (load+5)/10;
            if (r >= 1000)
                sprintf(g_oled_line[1], "S1:%d.%dC L:100%%",
                        g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec);
            else
                sprintf(g_oled_line[1], "S1:%d.%dC L:%u.%u%%",
                        g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec, r/10, r%10);
        }
    } else
        sprintf(g_oled_line[1], "S1:--.-C L:--%% ");
    sprintf(g_oled_line[2], "H:%d.%d%% HB:%-5u",
            g_slave_nodes[0].humi_int, g_slave_nodes[0].humi_dec, g_slave_nodes[0].heartbeat_count);
    {
        char r_str[5]; const char *status;
        uint32_t rx = g_can_rx_int_count;
        uint8_t hc = FIFO_High_Count(), nc = FIFO_Normal_Count();
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
    for (uint8_t i = 0; i < 4; i++) { g_oled_line[i][16] = '\0'; OLED_ShowString(i+1, 1, g_oled_line[i]); }
}

/* ==================== Tasks ==================== */

static void Task_CAN_Rx(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        /* Use VERY long timeout (≈24 days) instead of portMAX_DELAY.
         * This goes to the delayed list (not suspended list),
         * which was already validated with pdMS_TO_TICKS(100). */
        if (xSemaphoreTake(g_can_rx_sem, pdMS_TO_TICKS(0x7FFFFFFF)) == pdPASS)
            CAN_ProcessFrame(&g_isr_rx_frame);
    }
}

static void Task_CAN_Monitor(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;
    for (;;) {
        CAN_HeartBeatCheck(); CAN_ErrorMonitor(); CAN_CalcBusLoad();
        CAN_CheckEscalation(); CAN_CheckDeescalation();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

static void Task_CAN_Tx(void *pvParameters)
{
    CanRxFrame frame;
    (void)pvParameters;
    for (;;) {
        if (xSemaphoreTake(g_can_fifo_not_empty, pdMS_TO_TICKS(10)) != pdPASS) continue;
        while (FIFO_High_Pop(&frame) == 0) {
            if (g_system_throttle_level >= 2) {
                uint8_t f=frame.Data[CAN_DATA_FUNC_IDX], p=frame.Data[CAN_DATA_TYPE_IDX];
                if (p != 0 && f != CAN_FUNC_HEARTBEAT) continue;
            }
            CAN_SendFrame(frame.StdId, frame.Data, frame.DLC);
        }
        while (FIFO_Normal_Pop(&frame) == 0) {
            if (g_system_throttle_level >= 2) {
                uint8_t f=frame.Data[CAN_DATA_FUNC_IDX], p=frame.Data[CAN_DATA_TYPE_IDX];
                if (p != 0 && f != CAN_FUNC_HEARTBEAT) continue;
            }
            CAN_SendFrame(frame.StdId, frame.Data, frame.DLC);
        }
    }
}

static void Task_Protocol(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;
    for (;;) {
        ModbusTCP_SyncFromCAN();
        if (W5500_IsConnected()) {
            if (!g_eth_was_connected) { ModbusTCP_OnReconnect(); g_eth_was_connected = 1; }
        } else {
            if (g_eth_was_connected) { ModbusTCP_OnDisconnect(); g_eth_was_connected = 0; }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

static void Task_W5500(void *pvParameters)
{
    (void)pvParameters;
    vTaskDelay(pdMS_TO_TICKS(100));
    W5500_TCPServer_Start(MODBUS_PORT);
    for (;;) {
        W5500_TCPServer_Run();
        ModbusTCP_Process();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void Task_Housekeep(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;
    for (;;) {
        UBaseType_t s[6];
        s[0]=uxTaskGetStackHighWaterMark(hTask_CAN_Rx);
        s[1]=uxTaskGetStackHighWaterMark(hTask_CAN_Monitor);
        s[2]=uxTaskGetStackHighWaterMark(hTask_CAN_Tx);
        s[3]=uxTaskGetStackHighWaterMark(hTask_Protocol);
        s[4]=uxTaskGetStackHighWaterMark(hTask_W5500);
        s[5]=uxTaskGetStackHighWaterMark(hTask_Housekeep);
        for (uint8_t i=0;i<6;i++) if (s[i]<50) (void)s[i];
        OLED_UpdateDisplay();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    portDISABLE_INTERRUPTS(); OLED_Clear();
    OLED_ShowString(1, 1, "STACK OVERFLOW!");
    OLED_ShowString(2, 1, pcTaskName ? pcTaskName : "???");
    OLED_ShowString(3, 1, "System Halted");
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    portDISABLE_INTERRUPTS(); OLED_Clear();
    OLED_ShowString(1, 1, "MALLOC FAILED!");
    for (;;) { }
}

int main(void)
{
    int8_t ret;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    Delay_InitTick(); Delay_ms(200);
    OLED_Init(); OLED_Clear();
    OLED_ShowString(1, 1, "M: Init...");

    CAN_User_Init();
    OLED_ShowString(2, 1, "   CAN OK");

    ret = W5500_Init();
    if (ret != W5500_OK) {
        OLED_ShowString(3, 1, "   W5500 FAIL!");
        { char d[17]; sprintf(d,"   Ver:0x%02X",g_w5500_version); d[16]=0; OLED_ShowString(4,1,d); }
        while (1);
    }
    ret = W5500_ConfigNetwork();
    if (ret != W5500_OK) OLED_ShowString(3, 1, "   NET CFG ERR");
    else { OLED_ShowString(3, 1, "   ETH OK"); OLED_ShowString(4, 1, "FreeRTOS..."); }

    ModbusTCP_Init();
    g_eth_was_connected = 0;

    {
        BaseType_t t;

        /* CAN_Rx — use very long finite timeout instead of portMAX_DELAY
         * (portMAX_DELAY hits xSuspendedTaskList path which has an unresolved bug) */
        t = xTaskCreate(Task_CAN_Rx,     "CAN_Rx",  STACK_CAN_RX,     NULL, 5, &hTask_CAN_Rx);
        if (t!=pdPASS){OLED_ShowString(4,1,"CRxFAIL ");while(1);}
        t = xTaskCreate(Task_CAN_Monitor,"CAN_Mon", STACK_CAN_MONITOR, NULL, 4, &hTask_CAN_Monitor);
        if (t!=pdPASS){OLED_ShowString(4,1,"CMonFAIL");while(1);}
        t = xTaskCreate(Task_CAN_Tx,     "CAN_Tx",  STACK_CAN_TX,      NULL, 3, &hTask_CAN_Tx);
        if (t!=pdPASS){OLED_ShowString(4,1,"CTxFAIL ");while(1);}
        t = xTaskCreate(Task_Protocol,   "Proto",   STACK_PROTOCOL,    NULL, 2, &hTask_Protocol);
        if (t!=pdPASS){OLED_ShowString(4,1,"ProtoFAIL");while(1);}
        t = xTaskCreate(Task_W5500,      "W5500",   STACK_W5500,       NULL, 1, &hTask_W5500);
        if (t!=pdPASS){OLED_ShowString(4,1,"W5500FAIL");while(1);}
        t = xTaskCreate(Task_Housekeep,  "HouseKp", STACK_HOUSEKEEP,   NULL, 0, &hTask_Housekeep);
        if (t!=pdPASS){OLED_ShowString(4,1,"HKpFAIL ");while(1);}
    }

    {
        BaseType_t sem;
        do { sem = xSemaphoreTake(g_can_monitor_sem, 0); } while (sem == pdPASS);
        do { sem = xSemaphoreTake(g_can_rx_sem, 0); } while (sem == pdPASS);
        do { sem = xSemaphoreTake(g_can_fifo_not_empty, 0); } while (sem == pdPASS);
    }

    OLED_Clear();
    OLED_ShowString(1, 1, "M: Starting...");
    OLED_ShowString(2, 1, "FreeRTOS v10.4");

    vTaskStartScheduler();

    OLED_Clear();
    OLED_ShowString(1, 1, "SCHED FAILED!");
    OLED_ShowString(2, 1, "vTaskStartSched");
    OLED_ShowString(3, 1, "returned!!!");
    while (1) { }
}
