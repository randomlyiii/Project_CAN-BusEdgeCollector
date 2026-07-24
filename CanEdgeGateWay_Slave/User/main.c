/**
 * CAN Edge Gateway Slave — Phase 2 (FreeRTOS)
 *
 * 4-task architecture:
 *   vTask_Sensor (prio 3):   DHT11 + LM393 2s each, cache data
 *   vTask_CAN_Slave (prio 2): heartbeat 500ms, TX frames, replay cache
 *   vTask_Key (prio 1):      20ms key scan
 *   vTask_Housekeep (prio 0): 1s watchdog, cache cleanup, OLED
 */

#include "stm32f10x.h"
#include "FreeRTOSConfig.h"
#include "delay.h"
#include "oled.h"
#include "dht11.h"
#include "CAN_User.h"
#include "KEY.h"
#include "lm393.h"
#include "local_cache.h"
#include "stm32f10x_iwdg.h"
#include <stdio.h>
#include <string.h>

/* ---- Semihosting ---- */
#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }

/* ---- Task handles ---- */
static TaskHandle_t hTask_Sensor     = NULL;
static TaskHandle_t hTask_CAN_Slave  = NULL;
static TaskHandle_t hTask_Key        = NULL;
static TaskHandle_t hTask_Housekeep  = NULL;

/* ---- Task stacks ---- */
#define STACK_SENSOR    384
#define STACK_CAN_SLV   320
#define STACK_KEY       192
#define STACK_HOUSEKEEP 192

/* ---- Sensor data cache ---- */
static uint8_t  g_temp_int  = 0, g_temp_dec  = 0;
static uint8_t  g_humi_int  = 0, g_humi_dec  = 0;
static uint8_t  g_dht11_ok  = 0;
static uint8_t  g_dht11_fail_cnt = 0;
static uint16_t g_light_lux = 0;
static uint8_t  g_lm393_ok = 0;

/* ---- CAN TX count ---- */
static uint32_t g_can_tx_oled_count = 0;

/* ---- OLED ---- */
static char g_oled_line[4][17];

static void OLED_UpdateDisplay(void)
{
    /* Line 1: Node ID + status */
    sprintf(g_oled_line[0], "S:Node#%02u %s",
            SLAVE_NODE_ID, g_dht11_ok ? "Ready" : "FAIL");

    /* Line 2: Temperature + Light (ADC 0-4095 → 0-100%) */
    if (g_dht11_ok && g_lm393_ok) {
        uint16_t light_pct = (uint16_t)((uint32_t)g_light_lux * 100 / 4095);
        sprintf(g_oled_line[1], "T:%u.%uC L:%3u%%",
                g_temp_int, g_temp_dec, (unsigned int)light_pct);
    } else if (g_dht11_ok) {
        sprintf(g_oled_line[1], "T:%u.%uC L:---%%",
                g_temp_int, g_temp_dec);
    } else {
        sprintf(g_oled_line[1], "T:--.-C L:---%%");
    }

    /* Line 3: Humidity + CAN status */
    {
        const char *can = LocalCache_IsOffline(&g_local_cache) ? "OFF" : "OK ";
        if (g_dht11_ok)
            sprintf(g_oled_line[2], "H:%u.%u%% C:%-6s", g_humi_int, g_humi_dec, can);
        else
            sprintf(g_oled_line[2], "H:--.-%% C:%-6s", can);
    }

    /* Line 4: TX count + cache usage */
    {
        uint8_t cache_count = LocalCache_Count(&g_local_cache);
        uint32_t tx = g_can_tx_oled_count;
        if (tx > 99999) tx = 99999;
        sprintf(g_oled_line[3], "TX:%05lu C:%02u/%-2u",
                (unsigned long)tx, cache_count, LOCAL_CACHE_MAX);
    }

    for (uint8_t i = 0; i < 4; i++) {
        g_oled_line[i][16] = '\0';
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

/* ==================== Task: Sensor (priority 3) ==================== */

static void Task_Sensor(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    /* Update DHT11 every wake, LM393 every other wake (both ~2s) */
    uint8_t cycle = 0;

    for (;;) {
        /* DHT11 */
        g_dht11_ok = (DHT11_Read_Data(&g_humi_int, &g_humi_dec,
                                       &g_temp_int, &g_temp_dec) == 0);
        if (!g_dht11_ok) {
            g_dht11_fail_cnt++;
        } else {
            g_dht11_fail_cnt = 0;
        }

        /* LM393 (every 2 cycles = ~4s during throttle, but 2s base) */
        if (cycle == 0) {
            g_light_lux = Lm393_ReadAnalog();
            g_lm393_ok = 1;
        }
        cycle = (cycle + 1) & 1;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
    }
}

/* ==================== Task: CAN Slave (priority 2) ==================== */

static void Task_CAN_Slave(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t   id;
    uint8_t    data[CAN_DATA_LEN];
    (void)pvParameters;

    for (;;) {
        /* Heartbeat + temp/humi (every 500ms) */
        CAN_SendHeartBeat();
        CAN_SendTempHumi(g_temp_int, g_temp_dec, g_humi_int, g_humi_dec);
        g_can_tx_oled_count++;

        /* Light sensor (every 2s: 4th wake) */
        {
            static uint8_t light_div = 0;
            light_div++;
            if (light_div >= 4 && g_lm393_ok) {
                CAN_SendLight(g_light_lux);
                light_div = 0;
                g_can_tx_oled_count++;
            }
        }

        /* Replay cached data if CAN recovered */
        if (LocalCache_ShouldReplay(&g_local_cache)) {
            if (LocalCache_Pop(&g_local_cache, &id, data) == 0) {
                CAN_SendFrame(id, data, 8);
                g_can_tx_oled_count++;
            }
        }

        /* Trigger escalation if DHT11 failed 3 consecutive times */
        if (g_dht11_fail_cnt >= 3) {
            CAN_SendAlarm();
            g_can_tx_oled_count++;
            g_dht11_fail_cnt = 0;  /* Reset after alarm */
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

/* ==================== Task: Key (priority 1, 20ms) ==================== */

static void Task_Key(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    for (;;) {
        uint8_t key_evt = KEY_Scan();
        if (key_evt == KEY_EVT_KEY1_SHRT) {
            CAN_SendAlarm();
            g_can_tx_oled_count++;
        } else if (key_evt == KEY_EVT_KEY2_SHRT) {
            CAN_SendRecover();
            g_can_tx_oled_count++;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

/* ==================== Task: Housekeep (priority 0, 1s) ==================== */

static void Task_Housekeep(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    /* Init IWDG: ~1s timeout */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(625);
    IWDG_ReloadCounter();
    IWDG_Enable();

    for (;;) {
        IWDG_ReloadCounter();

        /* Stack high-water checks */
        {
            UBaseType_t stacks[4];
            stacks[0] = uxTaskGetStackHighWaterMark(hTask_Sensor);
            stacks[1] = uxTaskGetStackHighWaterMark(hTask_CAN_Slave);
            stacks[2] = uxTaskGetStackHighWaterMark(hTask_Key);
            stacks[3] = uxTaskGetStackHighWaterMark(hTask_Housekeep);
            (void)stacks;  /* TODO: USART1 log if < 20% */
        }

        /* Update OLED */
        OLED_UpdateDisplay();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/* ==================== Malloc failed hook ==================== */

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    OLED_Clear();
    OLED_ShowString(1, 1, "MALLOC FAILED!");
    for (;;) { }
}

/* ==================== Stack overflow hook ==================== */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    OLED_Clear();
    OLED_ShowString(1, 1, "STACK OVERFLOW!");
    for (;;) { }
}

/* ==================== Main ==================== */

int main(void)
{
    /* Hardware init */

    /* CRITICAL: 4-bit preemption priority (no sub-priority).
       FreeRTOS BASEPRI mechanism requires ALL priority bits to be
       preemption-priority bits. Default reset value is 0-bit preemption
       which silently breaks all FreeRTOS critical sections. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    Delay_InitTick();
    Delay_ms(200);

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "S: Init...");

    DHT11_GPIO_Init();
    OLED_ShowString(2, 1, "   DHT11 OK");

    Lm393_Init();
    OLED_ShowString(2, 1, "DHT+LM393 OK");

    CAN_User_Init();
    OLED_ShowString(3, 1, "   CAN OK");

    KEY_Init();
    OLED_ShowString(4, 1, "   Key OK");

    Delay_ms(1000);
    OLED_Clear();

    /* Create FreeRTOS tasks */
    xTaskCreate(Task_Sensor,    "Sensor",  STACK_SENSOR,    NULL, 3, &hTask_Sensor);
    xTaskCreate(Task_CAN_Slave, "CAN_Slv", STACK_CAN_SLV,   NULL, 2, &hTask_CAN_Slave);
    xTaskCreate(Task_Key,       "Key",     STACK_KEY,       NULL, 1, &hTask_Key);
    xTaskCreate(Task_Housekeep, "HouseKp", STACK_HOUSEKEEP, NULL, 0, &hTask_Housekeep);

    OLED_ShowString(1, 1, "S: Starting...");
    OLED_ShowString(2, 1, "FreeRTOS v10.4");

    vTaskStartScheduler();

    /* DIAGNOSTIC: reached only if scheduler failed to start */
    OLED_Clear();
    OLED_ShowString(1, 1, "SCHED FAILED!");
    OLED_ShowString(2, 1, "vTaskStartSched");
    OLED_ShowString(3, 1, "returned!!!");
    while (1) { }
}
