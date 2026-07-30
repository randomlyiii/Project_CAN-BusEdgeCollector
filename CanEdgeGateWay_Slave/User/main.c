/**
 * CAN Edge Gateway Slave — Final Phase (Config-Driven Multi-Variant)
 *
 * Architecture:
 *   Config layer  (Config/slave_config.h)    — one-line variant switch
 *   Sensor layer  (Hardware/sensor_manager)   — table-driven, conditional compile
 *   CAN layer     (Hardware/CAN_User)         — generic CAN_SendSensorData()
 *   OLED layer    (inline renderers)          — table-driven, sensor-count adaptive
 *
 * 4-task FreeRTOS:
 *   vTask_Sensor   (prio 3, 200ms): sensor_manager_read_all()
 *   vTask_CAN_Slave(prio 2, 500ms): heartbeat + sensor data + alarm/recover + cache replay
 *   vTask_Key      (prio 1,  20ms): key scan -> alarm/recover
 *   vTask_Housekeep(prio 0,   1s): IWDG, stack watermarks, OLED update
 */

#include "stm32f10x.h"
#include "../FreeRTOS/inc/FreeRTOS.h"
#include "../FreeRTOS/inc/task.h"
#include "../Config/slave_config.h"
#include "delay.h"
#include "oled.h"
#include "CAN_User.h"
#include "KEY.h"
#include "local_cache.h"
#include "sensor_manager.h"
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

/* ---- Task stacks (Phase-2-verified values) ---- */
#define STACK_SENSOR    384
#define STACK_CAN_SLV   320
#define STACK_KEY       192
#define STACK_HOUSEKEEP 384

/* ---- CAN TX count (shared: CAN task increments, Housekeep reads for OLED) ---- */
static uint32_t g_can_tx_oled_count = 0;

/* ---- OLED line buffer (16 chars + null) ---- */
static char g_oled_line[4][17];

/* ================================================================
 * OLED Line Renderers — table-driven, sensor-count adaptive
 *
 * Layout:
 *   Line 1: Node info (always)
 *   Line 2: Sensor[0] data (if count >= 1)
 *   Line 3: Sensor[1] data (if count >= 2) or "No Sensor!"
 *   Line 4: CAN + cache status (always)
 *
 * Each renderer writes <=16 chars to buf[17]. The caller pads to 16.
 * ================================================================ */

/* Line renderer function pointer type */
typedef void (*line_renderer_t)(char *buf, uint16_t buf_size);

/* ---- Line 1: Node identity + global status ---- */
static void oled_render_node_info(char *buf, uint16_t size)
{
    const char *status = sensor_manager_has_alarm() ? "ALARM" : "Ready";
    snprintf(buf, size, "S:%s %s", slave_node_name_str, status);
}

/* ---- Line 2/3: Per-sensor data (dispatched by type_id) ---- */
static void oled_render_sensor_data(char *buf, uint16_t size, uint8_t idx)
{
    sensor_t *s = sensor_manager_get_by_index(idx);
    if (!s) {
        snprintf(buf, size, "?              ");
        return;
    }
    if (!s->online) {
        snprintf(buf, size, "%s:OFFLINE", s->name);
        return;
    }

    switch (s->type_id) {
    case SENSOR_TYPE_DHT11:
        /* Max: "T:50.9C H:90.9%" = 15 chars, safe within 16 */
        snprintf(buf, size, "T:%u.%uC H:%u.%u%%",
                 (unsigned int)s->last_data.dht11.temp_int,
                 (unsigned int)s->last_data.dht11.temp_dec,
                 (unsigned int)s->last_data.dht11.humi_int,
                 (unsigned int)s->last_data.dht11.humi_dec);
        break;

    case SENSOR_TYPE_BH1750: {
        uint16_t lux = s->last_data.bh1750.lux;
        if (lux < 1000UL) {
            snprintf(buf, size, "Lux:%-4u      ", (unsigned int)lux);
        } else {
            uint16_t k = lux / 1000U;
            uint8_t  d = (lux % 1000U) / 100U;
            snprintf(buf, size, "Lux:%u.%uk      ", (unsigned int)k, (unsigned int)d);
        }
        break;
    }

    case SENSOR_TYPE_LM393_AO:
        snprintf(buf, size, "Lm DO:%s A:%-4u",
                 s->last_data.lm393.digital ? "DK" : "BR",
                 (unsigned int)s->last_data.lm393.analog);
        break;

    case SENSOR_TYPE_RESERVED:
        snprintf(buf, size, "CH1:----       ");
        break;

    default:
        snprintf(buf, size, "%-16s", s->name);
        break;
    }
}

/* ---- Line 4: CAN TX count + cache occupancy ---- */
static void oled_render_can_status(char *buf, uint16_t size)
{
    const char *can_state = LocalCache_IsOffline(&g_local_cache) ? "OFF" : "OK ";
    uint8_t cache_cnt = LocalCache_Count(&g_local_cache);
    uint32_t tx = g_can_tx_oled_count;
    if (tx > 99999) tx = 99999;

    snprintf(buf, size, "TX:%05lu C:%02u/%-2u",
             (unsigned long)tx, cache_cnt, LOCAL_CACHE_MAX);
}

/* ---- Master OLED update: assemble 4-line display ---- */
static void OLED_UpdateDisplay(void)
{
    char line_buf[17];
    uint8_t sensor_count = sensor_manager_get_count();
    uint8_t line = 0;
    uint8_t j;

    /* Line 1: Node info (always present) */
    oled_render_node_info(line_buf, sizeof(line_buf));
    for (j = strlen(line_buf); j < 16; j++) line_buf[j] = ' ';
    line_buf[16] = '\0';
    OLED_ShowString(++line, 1, line_buf);

    /* Line 2-3: Sensor data -- populated by sensor count */
    if (sensor_count >= 1) {
        oled_render_sensor_data(line_buf, sizeof(line_buf), 0);
        for (j = strlen(line_buf); j < 16; j++) line_buf[j] = ' ';
        line_buf[16] = '\0';
        OLED_ShowString(++line, 1, line_buf);
    }
    if (sensor_count >= 2) {
        oled_render_sensor_data(line_buf, sizeof(line_buf), 1);
        for (j = strlen(line_buf); j < 16; j++) line_buf[j] = ' ';
        line_buf[16] = '\0';
        OLED_ShowString(++line, 1, line_buf);
    }
    if (sensor_count == 0) {
        snprintf(line_buf, sizeof(line_buf), " No Sensor!    ");
        line_buf[16] = '\0';
        OLED_ShowString(++line, 1, line_buf);
    }
    /* Pad remaining sensor lines to clear stale pixels */
    while (line < 3) {
        snprintf(line_buf, sizeof(line_buf), "                ");
        line_buf[16] = '\0';
        OLED_ShowString(++line, 1, line_buf);
    }

    /* Line 4: CAN + cache status (always at bottom) */
    oled_render_can_status(line_buf, sizeof(line_buf));
    for (j = strlen(line_buf); j < 16; j++) line_buf[j] = ' ';
    line_buf[16] = '\0';
    OLED_ShowString(4, 1, line_buf);
}

/* ================================================================
 * Task: Sensor (priority 3, 200ms scan)
 *
 * sensor_manager_read_all() handles all sensors in one call.
 * Each sensor's actual sample interval is gated internally by
 * last_read_tick -- the 200ms scan rate just ensures timely polling.
 * ================================================================ */

static void Task_Sensor(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    for (;;) {
        sensor_manager_read_all();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
    }
}

/* ================================================================
 * Task: CAN Slave (priority 2, 500ms)
 *
 * Sends heartbeat, iterates sensor table to publish data, handles
 * alarm/recover escalation, and replays cached frames on CAN recovery.
 * ================================================================ */

static void Task_CAN_Slave(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t   id;
    uint8_t    data[CAN_DATA_LEN];
    uint8_t    i;
    (void)pvParameters;

    /* CAN init MUST be in task context — scheduler running = ISR safe */
    CAN_DeInit(CAN1);
    Delay_ms(10);
    CAN_User_Init();

    /* Startup stagger: each slave waits before first TX.
       Slave #1 (node_id=0x01) → 0ms, Slave #2 (node_id=0x02) → 2000ms */
    vTaskDelay(pdMS_TO_TICKS((slave_node_id - 1) * 2000));

    for (;;) {
        /* Drain RX FIFO (max 16 frames/cycle — prevents IWDG starvation
           on busy bus). Polling-based, no ISR needed. */
        {
            CanRxMsg rx_msg;
            uint8_t drain = 0;
            while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0 && drain < 16) {
                CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);
                drain++;
                (void)rx_msg;  /* Slave doesn't process frames, just drains */
            }
        }

        /* TX 预算: 每周期最多发 3 帧 */
        uint8_t tx_budget = 3;

        /* 1. Heartbeat (always) */
        if (tx_budget) { CAN_SendHeartBeat(); g_can_tx_oled_count++; tx_budget--; }

        /* 2. 告警/恢复 (emergency, 不计预算 — 掉线传感器也必须能发 ALARM) */
        for (i = 0; i < sensor_manager_get_count(); i++) {
            sensor_t *s = sensor_manager_get_by_index(i);
            if (!s || !s->enabled) continue;
            if (s->alarm_active) {
                CAN_SendAlarm();
                g_can_tx_oled_count++;
            }
            if (s->recover_pending) {
                CAN_SendRecover();
                g_can_tx_oled_count++;
                s->recover_pending = 0;
            }
        }

        /* 3. 在线传感器数据 (预算剩余内尽量发) */
        for (i = 0; i < sensor_manager_get_count() && tx_budget > 0; i++) {
            sensor_t *s = sensor_manager_get_by_index(i);
            if (!s || !s->online || !s->enabled) continue;
            if (!s->read_fn) continue;
            CAN_SendSensorData(s->type_id, &s->last_data);
            g_can_tx_oled_count++; tx_budget--;
        }

        /* 清理过期缓存帧 > 10s, 防恢复后总线流量风暴 */
        LocalCache_Cleanup(&g_local_cache);

        /* Cache replay: 有预算时补传 */
        while (tx_budget && LocalCache_ShouldReplay(&g_local_cache)) {
            if (LocalCache_Pop(&g_local_cache, &id, data) == 0) {
                CAN_SendFrame(id, data, 8);
                g_can_tx_oled_count++; tx_budget--;
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

/* ================================================================
 * Task: Key (priority 1, 20ms)
 *
 * KEY1 short press -> CAN alarm
 * KEY2 short press -> CAN recover
 * ================================================================ */

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

/* ================================================================
 * Task: Housekeep (priority 0, 1s)
 *
 * IWDG feeding, stack high-water monitoring, OLED refresh.
 * ================================================================ */

static void Task_Housekeep(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    (void)pvParameters;

    /* Init IWDG: ~4s timeout (LSI=30~60kHz, margin for false reset) */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    IWDG_SetReload(1250);
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
            (void)stacks;
        }

        /* Update OLED — protect I2C bus shared with BH1750 */
        if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50));
        OLED_UpdateDisplay();
        if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);

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

/* ================================================================
 * Main
 *
 * Init flow:
 *   1. NVIC priority group (4-bit preemption -- mandatory for FreeRTOS BASEPRI)
 *   2. Delay init + OLED
 *   3. sensor_manager_init() -- config-driven, inits all enabled sensors
 *   4. CAN + KEY init
 *   5. Create FreeRTOS tasks
 *   6. vTaskStartScheduler()
 * ================================================================ */

int main(void)
{
    /* CRITICAL: 4-bit preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    Delay_InitTick();
    Delay_ms(200);

    /* DIAG: PA1 output -- HIGH = scheduler starting */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    {
        GPIO_InitTypeDef g;
        g.GPIO_Pin   = GPIO_Pin_1;
        g.GPIO_Mode  = GPIO_Mode_Out_PP;
        g.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &g);
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    }

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "S: Init...     ");

    /* Init sensors (conditional compile in sensor_manager -- only inits
       what slave_has_xxx enables). WARNING: PB11(DHT11) is shorted to
       I2C bus(PB8/PB9) on some boards -- set slave_has_dht11=0 in config. */
    sensor_manager_init();
    OLED_ShowString(2, 1, " Sensors OK    ");

    /* CAN init in Task_CAN_Slave — slave is TX-only, no RX interrupt needed */

    KEY_Init();
    OLED_ShowString(4, 1, "   Key OK      ");

    Delay_ms(500);
    OLED_Clear();

    xTaskCreate(Task_Sensor,    "Sensor",  STACK_SENSOR,    NULL, 3, &hTask_Sensor);
    xTaskCreate(Task_CAN_Slave, "CAN_Slv", STACK_CAN_SLV,   NULL, 2, &hTask_CAN_Slave);
    xTaskCreate(Task_Key,       "Key",     STACK_KEY,       NULL, 1, &hTask_Key);
    xTaskCreate(Task_Housekeep, "HouseKp", STACK_HOUSEKEEP, NULL, 0, &hTask_Housekeep);

    OLED_ShowString(1, 1, "S: Starting... ");
    OLED_ShowString(2, 1, "FreeRTOS v10.4 ");

    GPIO_SetBits(GPIOA, GPIO_Pin_1);  /* HIGH = scheduler starting */

    vTaskStartScheduler();

    /* Should never reach here */
    OLED_Clear();
    OLED_ShowString(1, 1, "SCHED FAILED!  ");
    while (1) { }
}
