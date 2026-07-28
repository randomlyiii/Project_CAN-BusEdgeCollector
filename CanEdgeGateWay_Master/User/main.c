/**
 * CAN Edge Gateway Master — Sequential Architecture
 *
 * Phase 1 证明 CAN+W5500 必须顺序执行。Phase 2 的 FreeRTOS
 * 并发架构无法在物理层隔离 SPI 和 CAN RX 的串扰。
 * 改为单任务轮询——保留 FreeRTOS 的 tick/内存管理/OLED 刷新，
 * CAN/W5500 合并为一个顺序任务。
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
#include "stm32f10x_iwdg.h"

#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; };
FILE __stdout = {0};
void _sys_exit(int x) { x = x; }

static TaskHandle_t hTask_Unified   = NULL;
static TaskHandle_t hTask_Housekeep = NULL;

#define STACK_UNIFIED     768
#define STACK_HOUSEKEEP   384

static char g_oled_line[4][17];
static uint8_t g_eth_was_connected = 0;

/* ---- Page-switch key (PA0, GND=press) ---- */
#define KEY_PAGE_PORT    GPIOA
#define KEY_PAGE_PIN     GPIO_Pin_0
static uint8_t g_display_page = 0;  /* 0=sensors+ETH, 1=CAN bus+alarm */

static void KEY_Page_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin   = KEY_PAGE_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY_PAGE_PORT, &gpio);
}

static uint8_t KEY_Page_Scan(void)
{
    static uint8_t last = 1;
    uint8_t cur = GPIO_ReadInputDataBit(KEY_PAGE_PORT, KEY_PAGE_PIN);
    if (last == 1 && cur == 0) { last = 0; return 1; }
    if (cur == 1) last = 1;
    return 0;
}

/* ==================== OLED helper ==================== */
static const char *CAN_StateStr(void)
{
    switch (g_can_error.error_level) {
    case 1:  return "WRN";
    case 2:  return "ERR";
    case 3:  return "BOF";
    default:
        /* 启动后 3 秒未收到任何帧 → 总线异常（CAN 线未接/从站掉电） */
        if (g_can_rx_int_count == 0 && Delay_GetTick() > 3000)
            return "WRN";
        /* 已知节点曾在线但现在离线 → 总线异常 */
        for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
            if (g_slave_nodes[i].last_heartbeat_tick > 0 && !g_slave_nodes[i].online)
                return "WRN";
        }
        return "OK ";
    }
}

static const char *ETH_StateStr(void)
{
    if (!W5500_IsOnline())   return "FAIL";
    if (!W5500_IsLinkUpCached()) return "NOLK";
    if (W5500_IsConnected()) return "CON ";
    return "LSN ";
}

static void OLED_BuildPage1(void)
{
    /* ---- Page 1: Sensor + ETH (original display) ---- */
    uint8_t online1 = g_slave_nodes[0].online;
    uint8_t fault1  = g_slave_nodes[0].fault_flag;

    sprintf(g_oled_line[0], "CAN:%-3s ETH:%-4s", CAN_StateStr(), ETH_StateStr());
    if (online1) {
        uint16_t lux = g_slave_nodes[0].light_lux;
        if (lux < 1000)
            sprintf(g_oled_line[1], "S1:%d.%dC L:%3u",
                    g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec, lux);
        else {
            uint16_t k = lux / 1000;
            uint8_t  d = (lux % 1000) / 100;
            sprintf(g_oled_line[1], "S1:%d.%dC L:%u.%uk",
                    g_slave_nodes[0].temp_int, g_slave_nodes[0].temp_dec, k, d);
        }
    } else
        sprintf(g_oled_line[1], "S1:--.-C L:---");
    sprintf(g_oled_line[2], "H:%d.%d%% HB:%-5u",
            g_slave_nodes[0].humi_int, g_slave_nodes[0].humi_dec,
            (g_slave_nodes[0].heartbeat_count > 99999) ? 99999UL : g_slave_nodes[0].heartbeat_count);
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
        if (hc > 99) hc = 99; if (nc > 99) nc = 99;
        sprintf(g_oled_line[3], "R%-4s %s %u/%u",
                r_str, status, hc, nc);
    }
}

static void OLED_BuildPage2(void)
{
    /* ---- Page 2: CAN bus status + Alarms ---- */
    const char *state = CAN_StateStr();
    uint16_t load = g_can_error.bus_load;
    uint8_t  thr  = g_can_error.throttle_level;
    uint8_t  alm  = 0, blk = 0;
    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;
        if (g_slave_nodes[i].fault_flag ||
            (g_slave_nodes[i].last_heartbeat_tick > 0 && !g_slave_nodes[i].online))  alm = 1;
        if (g_slave_nodes[i].blacklist)   blk++;
    }

    /* Line 1: CAN error level + bus load */
    sprintf(g_oled_line[0], "CAN:%-3s L:%u.%02u%%",
            state, load / 100, load % 100);

    /* Line 2: Alarm flag + throttle + recovery count */
    sprintf(g_oled_line[1], "ALM:%c THR:%u RC:%u",
            alm ? 'Y' : 'N', thr,
            (unsigned int)g_bus_off_recovery_cnt);

    /* Line 3: FIFO pending + blacklist */
    {
        uint8_t hc = FIFO_High_Count(), nc = FIFO_Normal_Count();
        if (hc > 99) hc = 99;
        if (nc > 99) nc = 99;
        sprintf(g_oled_line[2], "H:%u N:%u BLK:%u",
                hc, nc, blk);
    }

    /* Line 4: RX count + FIFO overflow */
    {
        char rx_str[7];
        uint32_t rx = g_can_rx_int_count;
        if (rx >= 1000000)      sprintf(rx_str, "%luM", rx/1000000);
        else if (rx >= 10000)   sprintf(rx_str, "%luK", rx/1000);
        else if (rx >= 1000)    sprintf(rx_str, "%lu.%luK", rx/1000, (rx%1000)/100);
        else                    sprintf(rx_str, "%lu", (unsigned long)rx);
        sprintf(g_oled_line[3], "RX:%-5s OV:%u",
                rx_str, (unsigned int)g_fifo_high_overflow_cnt);
    }
}

static void OLED_UpdateDisplay(void)
{
    if (g_display_page == 0)
        OLED_BuildPage1();
    else
        OLED_BuildPage2();

    /* Pad to 16 chars to clear stale pixels, then draw */
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = strlen(g_oled_line[i]); j < 16; j++)
            g_oled_line[i][j] = ' ';
        g_oled_line[i][16] = '\0';
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

/* ========= 统一任务: 顺序处理 CAN + W5500 (Phase 1 架构) ========= */

static void Task_Unified(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake;

    CAN_DeInit(CAN1); Delay_ms(10); CAN_User_Init();
    vTaskDelay(pdMS_TO_TICKS(100));
    W5500_TCPServer_Start(MODBUS_PORT);
    xLastWake = xTaskGetTickCount();

    for (;;) {
        /* 1. CAN Rx (紧轮询) */
        CanRxMsg msg;
        while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0) {
            CAN_Receive(CAN1, CAN_FIFO0, &msg);
            g_isr_rx_frame.StdId = msg.StdId;
            g_isr_rx_frame.IDE   = msg.IDE;
            g_isr_rx_frame.RTR   = msg.RTR;
            g_isr_rx_frame.DLC   = msg.DLC;
            g_isr_rx_frame.FMI   = msg.FMI;
            memcpy(g_isr_rx_frame.Data, msg.Data, 8);
            CAN_ProcessFrame(&g_isr_rx_frame);
        }
        /* 2. CAN Tx (FIFO 中待发帧) */
        CanRxFrame frame;
        while (FIFO_High_Pop(&frame) == 0)
            CAN_SendFrame(frame.StdId, frame.Data, frame.DLC);
        while (FIFO_Normal_Pop(&frame) == 0)
            CAN_SendFrame(frame.StdId, frame.Data, frame.DLC);
        /* 3. CAN 监控 */
        CAN_HeartBeatCheck(); CAN_ErrorMonitor(); CAN_CalcBusLoad();
        CAN_CheckEscalation(); CAN_CheckDeescalation();
        /* 4. W5500 + ModbusTCP (顺序在 CAN 之后, 不并发) */
        W5500_TCPServer_Run();
        ModbusTCP_Process();
        ModbusTCP_SyncFromCAN();
        if (W5500_IsConnected()) {
            if (!g_eth_was_connected) { ModbusTCP_OnReconnect(); g_eth_was_connected = 1; }
        } else {
            if (g_eth_was_connected) { ModbusTCP_OnDisconnect(); g_eth_was_connected = 0; }
        }
        /* 5. Key scan (page toggle) */
        if (KEY_Page_Scan()) {
            g_display_page = !g_display_page;
            OLED_Clear();
        }
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));
    }
}

/* ==================== Housekeep (priority 0, 1s) ==================== */

static void Task_Housekeep(void *pvParameters)
{
    TickType_t xLastWake = xTaskGetTickCount();
    (void)pvParameters;

    /* IWDG: ~4s timeout (LSI 30~60kHz, prescaler 128, reload 1250) */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    IWDG_SetReload(1250);
    IWDG_ReloadCounter();
    IWDG_Enable();

    for (;;) {
			  // Delay_ms(6000);             /* ← 看门狗验证临时加，等 6 秒 > IWDG 超时 -> 等不到下一行，看门狗就复位了*/
        IWDG_ReloadCounter();
        UBaseType_t s;
        s = uxTaskGetStackHighWaterMark(hTask_Unified);
        if (s < 100) (void)s;
        s = uxTaskGetStackHighWaterMark(hTask_Housekeep);
        if (s < 100) (void)s;
        OLED_UpdateDisplay();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(1000));
    }
}

/* ==================== Hooks ==================== */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    portDISABLE_INTERRUPTS(); OLED_Clear();
    OLED_ShowString(1, 1, "STACK OVERFLOW!");
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    portDISABLE_INTERRUPTS(); OLED_Clear();
    OLED_ShowString(1, 1, "MALLOC FAILED!");
    for (;;) { }
}

/* ==================== Main ==================== */

int main(void)
{
    int8_t ret;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    Delay_InitTick(); Delay_ms(200);
    OLED_Init(); OLED_Clear();
    OLED_ShowString(1, 1, "M: Init...");
    KEY_Page_Init();

    ret = W5500_Init();
    if (ret != W5500_OK) {
        OLED_ShowString(2, 1, "   W5500 FAIL!");
        { char d[17]; sprintf(d,"   Ver:0x%02X",g_w5500_version); d[16]=0; OLED_ShowString(3,1,d); }
        while (1);
    }
    OLED_ShowString(2, 1, "   W5500 OK");

    ret = W5500_ConfigNetwork();
    if (ret != W5500_OK) OLED_ShowString(3, 1, "   NET CFG ERR");
    else                 OLED_ShowString(3, 1, "   ETH OK");

    ModbusTCP_Init();
    g_eth_was_connected = 0;

    xTaskCreate(Task_Unified,  "Unified", STACK_UNIFIED,   NULL, 1, &hTask_Unified);
    xTaskCreate(Task_Housekeep,"HouseKp", STACK_HOUSEKEEP, NULL, 0, &hTask_Housekeep);
    CAN_EnableInterrupts();

    OLED_Clear();
    OLED_ShowString(1, 1, "M: Sequential");
    OLED_ShowString(2, 1, "FreeRTOS v10.4");
    OLED_ShowString(3, 1, "1-task unified");
    OLED_ShowString(4, 1, "500kbps CAN+ETH");

    vTaskStartScheduler();

    OLED_Clear();
    OLED_ShowString(1, 1, "SCHED FAILED!");
    while (1) { }
}
