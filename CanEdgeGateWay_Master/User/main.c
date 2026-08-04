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
static TaskHandle_t hTask_CAN_Drain = NULL;

#define STACK_UNIFIED     768
#define STACK_HOUSEKEEP   384
#define STACK_CAN_DRAIN   256

static char g_oled_line[4][17];
static uint8_t g_eth_was_connected = 0;
volatile uint8_t  g_can_ready = 0;           /* CAN+FIFO 初始化完成标志 (FIFO mutex 已创建) */

/* ---- Page-switch key (PA0, GND=press) ---- */
#define KEY_PAGE_PORT    GPIOA
#define KEY_PAGE_PIN     GPIO_Pin_0
static uint8_t  g_display_page = 0;
static uint32_t g_last_page_switch_tick = 0;

/* ---- Alarm jump detection ---- */
static uint8_t g_prev_alarm_state = 0;

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
    case 0:  return "OK ";
    case 1:  return "WRN";
    case 2:  return "EPS";
    default: return "BOF";
    }
}

static const char *ETH_StateStr(void)
{
    if (!W5500_IsOnline())   return "FAIL";
    if (!W5500_IsLinkUpCached()) return "NOLK";
    if (W5500_IsConnected()) return "CON ";
    return "LSN ";
}

/* ---- Node status helper for OLED (6 态: --- < ??? < ON < EXP < OFF < ALM) ---- */
static const char *OLED_NodeStatusStr(uint8_t i)
{
    if (g_slave_nodes[i].node_id == 0) return "???";
    if (!g_slave_nodes[i].online)      return "---";  /* 预分配槽位, 从未收到帧 */
    if (g_slave_nodes[i].fault_flag)   return "ALM";  /* 硬件告警 */
    if (g_slave_nodes[i].offline)      return "OFF";  /* >30s 收不到帧 */
    if (g_slave_nodes[i].stale)        return "EXP";  /* >3s 收不到帧 */
    return "ON ";                                      /* 正常 */
}

/* ---- RX count compact formatter (safe: max 4 chars) ---- */
static const char *OLED_RxStr(char *buf)
{
    uint32_t rx = g_can_rx_int_count;
    if      (rx >= 1000000UL) sprintf(buf, "%luM", (unsigned long)(rx / 1000000UL));
    else if (rx >= 10000UL)   sprintf(buf, "%luK", (unsigned long)(rx / 1000UL));
    else if (rx >= 1000UL)    sprintf(buf, "%lu.%luK", (unsigned long)(rx / 1000UL), (unsigned long)((rx % 1000UL) / 100UL));
    else                      sprintf(buf, "%lu",   (unsigned long)rx);
    return buf;
}

/* Compact uint32 formatter: returns "999", "1.2K", "12K", "1.2M" (max 4 chars) */
static const char *OLED_CompactNum(char *buf, uint32_t val)
{
    if      (val >= 1000000UL) sprintf(buf, "%luM", (unsigned long)(val / 1000000UL));
    else if (val >= 10000UL)   sprintf(buf, "%luK", (unsigned long)(val / 1000UL));
    else if (val >= 1000UL)    sprintf(buf, "%lu.%luK", (unsigned long)(val / 1000UL), (unsigned long)((val % 1000UL) / 100UL));
    else                       sprintf(buf, "%lu",   (unsigned long)val);
    return buf;
}

/* ================================================================
 * Page 0 — 仪表盘 (Dashboard)   [all lines ≤16 chars verified]
 *
 * CAN:OK ETH:CON         ← CAN + ETH
 * S1:ON  HB:12345        ← 节点1 状态 + 心跳
 * S2:ON  HB:999          ← 节点2 状态 + 心跳
 * ALM:N E:0 R:1.5K       ← 告警 + 过期节点 + 帧数
 * ================================================================ */
static void OLED_BuildPage0(void)
{
    char num_buf[8];
    uint8_t alarm = 0;

    uint8_t expired = 0;

    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;
        if (g_slave_nodes[i].fault_flag) alarm = 1;
        if (g_slave_nodes[i].stale)      expired++;
    }

    /* "CAN:BOF ETH:FAIL" = 16 chars */
    sprintf(g_oled_line[0], "CAN:%-3s ETH:%-4s", CAN_StateStr(), ETH_StateStr());

    /* "S1:ON  HB:65535" = 16 chars (hb clamped to 99999) */
    sprintf(g_oled_line[1], "S1:%-3s HB:%-5u",
            OLED_NodeStatusStr(0),
            (unsigned int)(g_slave_nodes[0].heartbeat_count > 99999U ? 99999U : g_slave_nodes[0].heartbeat_count));

    sprintf(g_oled_line[2], "S2:%-3s HB:%-5u",
            OLED_NodeStatusStr(1),
            (unsigned int)(g_slave_nodes[1].heartbeat_count > 99999U ? 99999U : g_slave_nodes[1].heartbeat_count));

    /* "ALM:N E:0 R:999K" = 16 chars (E=expired nodes count) */
    sprintf(g_oled_line[3], "ALM:%c E:%u R:%s",
            alarm ? 'Y' : 'N',
            (unsigned int)expired,
            OLED_RxStr(num_buf));
}

/* ================================================================
 * Page 1 — 节点1 详细 (Node#01: DHT11 + BH1750)
 *
 * S1 Node#01             ← 12 chars
 * T:50.9C H:90.9%        ← max 16 chars (DHT11 spec limit)
 * L:65.5k HB:65535       ← max 16 chars
 * ALM:N RC:99            ← 12 chars
 * ================================================================ */
static void OLED_BuildPage1(void)
{
    uint8_t online = g_slave_nodes[0].online;
    /* Clamp sensor values within display-safe range */
    uint8_t ti = g_slave_nodes[0].temp_int;
    uint8_t td = g_slave_nodes[0].temp_dec;
    uint8_t hi = g_slave_nodes[0].humi_int;
    uint8_t hd = g_slave_nodes[0].humi_dec;
    if (ti > 99U) ti = 99U;  if (td > 9U) td = 9U;
    if (hi > 99U) hi = 99U;  if (hd > 9U) hd = 9U;

    sprintf(g_oled_line[0], "S1 Node#01");

    /* "T:99.9C H:99.9%" = 16 chars exactly */
    sprintf(g_oled_line[1], "T:%u.%uC H:%u.%u%%",
            (unsigned int)ti, (unsigned int)td,
            (unsigned int)hi, (unsigned int)hd);

    if (online) {
        uint16_t lux = g_slave_nodes[0].light_lux;
        uint16_t hb  = g_slave_nodes[0].heartbeat_count;
        if (hb > 65535U) hb = 65535U;
        if (lux >= 1000U) {
            uint16_t k = lux / 1000U;
            uint8_t  d = (lux % 1000U) / 100U;
            if (k > 99U) { k = 99U; d = 9U; }
            /* "L:99.9k HB:65535" = 16 chars */
            sprintf(g_oled_line[2], "L:%u.%uk HB:%u",
                    (unsigned int)k, (unsigned int)d, (unsigned int)hb);
        } else {
            /* "L:999 HB:65535" = 15 chars */
            sprintf(g_oled_line[2], "L:%u HB:%u",
                    (unsigned int)lux, (unsigned int)hb);
        }
    } else {
        sprintf(g_oled_line[2], "L:--- HB:----");
    }
    /* RC capped at 99: "ALM:N RC:99" = 12 chars */
    sprintf(g_oled_line[3], "ALM:%c RC:%u",
            g_slave_nodes[0].fault_flag ? 'Y' : 'N',
            (unsigned int)(g_bus_off_recovery_cnt > 99U ? 99U : g_bus_off_recovery_cnt));
}

/* ================================================================
 * Page 2 — 节点2 详细 (Node#02: LM393 + RESERVED)
 *
 * S2 Node#02             ← 12 chars
 * LM:BR A:4095           ← 12 chars
 * CH1:---- HB:999        ← 15 chars
 * ALM:N RC:99            ← 12 chars
 * ================================================================ */
static void OLED_BuildPage2(void)
{
    uint16_t hb = g_slave_nodes[1].heartbeat_count;
    uint16_t ao = g_slave_nodes[1].lm393_analog;
    if (hb > 999U)  hb = 999U;
    if (ao > 9999U) ao = 9999U;
    const char *dig = g_slave_nodes[1].lm393_digital ? "DK" : "BR";

    sprintf(g_oled_line[0], "S2 Node#02");
    /* "LM:BR A:4095" = 12 chars (pad loop fills to 16) */
    sprintf(g_oled_line[1], "LM:%s A:%-4u",
            dig, (unsigned int)ao);

    sprintf(g_oled_line[2], "CH1:---- HB:%u",
            (unsigned int)hb);

    sprintf(g_oled_line[3], "ALM:%c RC:%u",
            g_slave_nodes[1].fault_flag ? 'Y' : 'N',
            (unsigned int)(g_bus_off_recovery_cnt > 99U ? 99U : g_bus_off_recovery_cnt));
}

/* ================================================================
 * Page 3 — 总线诊断 (Bus Diagnostics)
 *
 * L:100.00% THR:2        ← max 15 chars (load=10000, thr=2)
 * H:99 N:99 OV:99K       ← max 16 chars (OV compact-formatted)
 * ERR:BOF RC:99K E:8     ← max 16 chars (RC compact-formatted)
 * RX:999K FIFO:OK        ← max 16 chars
 * ================================================================ */
static void OLED_BuildPage3(void)
{
    char nb1[8], nb2[8], nb3[8];
    uint16_t load   = g_can_error.bus_load;
    uint8_t  thr    = g_can_error.throttle_level;
    uint8_t  hc     = FIFO_High_Count();
    uint8_t  nc     = FIFO_Normal_Count();
    uint16_t ov_cnt = (uint16_t)g_fifo_high_overflow_cnt;
    uint16_t rc_cnt = (uint16_t)g_bus_off_recovery_cnt;
    uint8_t  expired = 0;
    const char *err_lvl;

    if (hc > 99U) hc = 99U;
    if (nc > 99U) nc = 99U;
    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;
        if (g_slave_nodes[i].stale) expired++;
    }
    switch (g_can_error.error_level) {
    case 0:  err_lvl = "OK "; break;
    case 1:  err_lvl = "WRN"; break;
    case 2:  err_lvl = "EPS"; break;
    default: err_lvl = "BOF"; break;
    }

    /* "L:100.00% THR:2" = 15 chars */
    sprintf(g_oled_line[0], "L:%u.%02u%% THR:%u",
            (unsigned int)(load / 100U), (unsigned int)(load % 100U),
            (unsigned int)thr);

    /* "H:99 N:99 O:65K" = 15 chars (O=overflow, compact format) */
    sprintf(g_oled_line[1], "H:%u N:%u O:%s",
            (unsigned int)hc, (unsigned int)nc,
            OLED_CompactNum(nb1, ov_cnt));

    /* "E:BOF R:65K E:8" = 16 chars (E=error, R=recovery, E=expired) */
    sprintf(g_oled_line[2], "E:%s R:%s E:%u",
            err_lvl, OLED_CompactNum(nb2, rc_cnt),
            (unsigned int)expired);

    /* "RX:65.5K F:OK" = 11 chars (F=ring 溢出, 主站 RX 丢帧指示;
     *   Modbus FIFO 溢出仍看上行 O: 字段) */
    sprintf(g_oled_line[3], "RX:%s F:%s",
            OLED_RxStr(nb3),
            g_rx_ring_high.overflow_cnt > 0U ? "OV" : "OK");
}

/**
 * OLED Update — page dispatch with auto-return and alarm jump
 **/
static void OLED_UpdateDisplay(void)
{
    uint32_t now = Delay_GetTick();

    /* Auto-return to page 0 after inactivity.
     * 超时来自 g_regs[REG_OLED_AUTO_RETURN_MS] (可从 ModbusPoll 写入 0x002C 动态调节).
     * 0=永不自动回主页. 出厂默认 10000ms (ModbusTCP_Init 写入). */
    if (g_display_page != 0) {
        uint32_t t = g_regs[REG_OLED_AUTO_RETURN_MS];
        if (t && now - g_last_page_switch_tick > t) {
            g_display_page = 0;
            OLED_Clear();
        }
    }

    /* Alarm jump: new alarm → force page 0 */
    {
        uint8_t cur_alarm = 0;
        for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
            if (g_slave_nodes[i].fault_flag) { cur_alarm = 1; break; }
        }
        if (cur_alarm && !g_prev_alarm_state && g_display_page != 0) {
            g_display_page = 0;
            OLED_Clear();
        }
        g_prev_alarm_state = cur_alarm;
    }

    /* Build selected page */
    switch (g_display_page) {
    case 1: OLED_BuildPage1(); break;
    case 2: OLED_BuildPage2(); break;
    case 3: OLED_BuildPage3(); break;
    default: OLED_BuildPage0(); break;
    }

    /* Pad to 16 chars to clear stale pixels, then draw */
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = strlen(g_oled_line[i]); j < 16; j++)
            g_oled_line[i][j] = ' ';
        g_oled_line[i][16] = '\0';
        OLED_ShowString(i + 1, 1, g_oled_line[i]);
    }
}

/* ========= 统一任务: 顺序处理 CAN + W5500 (Phase 1 架构) =========
 *
 * CAN RX: ISR 将帧从硬件 FIFO 搬到软件环形缓冲 (ring_push),
 *         任务在此 drain 软件缓冲 (ring_pop)，不碰硬件 FIFO。
 *         紧急帧 (g_rx_ring_high) 优先处理。
 *
 * CAN TX: 非阻塞，发前检查空闲邮箱，无邮箱则留到下一周期。
 * ================================================================ */

/* ========= 独立 CAN drain 任务: 解耦 W5500 阻塞 =========
 *
 * 背景: Task_Unified 内 W5500/Modbus 可阻塞 100~700ms (SocketCmd 超时 /
 *       SendData 超时), 若 ring drain 在同任务, 阻塞期 ring(64深,~14ms容量)
 *       必然溢出丢帧 (实测 T:700 + F:OV)。
 *
 * 本任务 prio 3 > Task_Unified(1): 每 2ms 抢占, 强制 drain ring_high/norm。
 * 只读内存 ring + 调 CAN_ProcessFrame (不碰 CAN 硬件, 不碰 SPI),
 * 不产生 SPI 边沿 → 物理上与顺序架构等价, 不违反 [[spi-can-concurrency]]。
 * ================================================================ */

static void Task_CAN_Drain(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        if (!g_can_ready) {
            /* CAN 未初始化: FIFO mutex 尚未创建 (在 Task_Unified), 此时
             * 处理帧会 xSemaphoreTake(NULL) → HardFault. 只清 ring 不处理. */
            RingFrame_t rf;
            while (RxRingHigh_pop(&g_rx_ring_high, &rf) == 0) { }
            while (RxRingNorm_pop(&g_rx_ring_norm, &rf) == 0) { }
            vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));  /* 1 tick; 不可用 2ms (pdMS_TO_TICKS(2)=0 忙转) */
            continue;
        }
        g_dbg_step = 1;

        /* 主路径: drain ring_high (RX0 ISR 填充, 全通滤波所有帧) */
        {   RingFrame_t rf;
            while (RxRingHigh_pop(&g_rx_ring_high, &rf) == 0) {
                g_isr_rx_frame.StdId = rf.StdId;
                g_isr_rx_frame.IDE   = 0;
                g_isr_rx_frame.RTR   = 0;
                g_isr_rx_frame.DLC   = rf.DLC;
                g_isr_rx_frame.FMI   = 0;
                memcpy(g_isr_rx_frame.Data, rf.Data, 8);
                CAN_ProcessFrame(&g_isr_rx_frame);
            }
        }

        g_dbg_step = 2;
        /* Fallback: drain ring_norm (FIFO1, FMP1 禁用, 正常无帧) */
        {   RingFrame_t rf;
            while (RxRingNorm_pop(&g_rx_ring_norm, &rf) == 0) {
                g_isr_rx_frame.StdId = rf.StdId;
                g_isr_rx_frame.IDE   = 0;
                g_isr_rx_frame.RTR   = 0;
                g_isr_rx_frame.DLC   = rf.DLC;
                g_isr_rx_frame.FMI   = 0;
                memcpy(g_isr_rx_frame.Data, rf.Data, 8);
                CAN_ProcessFrame(&g_isr_rx_frame);
            }
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));  /* 1 tick=10ms; 5000fps→50帧/10ms < ring64 */
    }
}

static void Task_Unified(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWake;

    CAN_DeInit(CAN1); Delay_ms(10); CAN_User_Init();
    CAN_EnableInterrupts();     /* re-enable after DeInit killed them */
    vTaskDelay(pdMS_TO_TICKS(100));
    W5500_TCPServer_Start(MODBUS_PORT);

    /* 丢弃启动期堆积 + 清零溢出计数: 上面 100ms 延迟期间 ISR 已把 ring
     * 灌满(5000fps×100ms=500帧 > 64深), 不清零则 F:OV 永久置位,
     * 掩盖稳态真实丢帧. 残留 64 帧随后正常 drain, 无影响. */
    g_rx_ring_high.overflow_cnt = 0;
    g_rx_ring_norm.overflow_cnt = 0;
    g_can_ready = 1;   /* CAN+FIFO 就绪, Task_CAN_Drain 可开始处理帧 */

    xLastWake = xTaskGetTickCount();

    for (;;) {
        IWDG_ReloadCounter();   /* 双保险喂狗 — 主循环也喂，防止 Housekeep 被饿死 */
        GPIO_SetBits(GPIOA, GPIO_Pin_1);        /* DBG: 高电平 = 循环开始 */

        /* ================================================================
         * 1. CAN Rx — 已剥离到 Task_CAN_Drain (prio 3, 每 2ms 抢占 drain)
         *
         * 原因: W5500/Modbus 在本任务可阻塞 100~700ms (SocketCmd/SendData
         *       超时), ring drain 留在本任务会被饿死 → ring(64深)溢出丢帧
         *       (实测 T:700 + F:OV)。Task_CAN_Drain 只读内存 ring, 不碰
         *       CAN 硬件/SPI, 不违反顺序架构。
         * ================================================================ */
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);      /* DBG: 低 = 本任务循环体开始 */

        g_dbg_step = 3;
        /* 2. CAN Tx — 主站只接收, 不主动发帧.
         *    从站心跳/数据由 CAN 控制器硬件自动应答, 不需主站主动参与.
         *    去掉测试帧: 避免无 ACK 时自动重传推 TEC 至 BusOff. */
        /* (CAN Tx intentionally omitted — master is receive-only) */

        g_dbg_step = 4;
        /* 3. CAN 监控 */
        CAN_ErrorMonitor(); CAN_HeartBeatCheck(); CAN_CalcBusLoad();
        CAN_CheckEscalation(); CAN_CheckDeescalation();

        g_dbg_step = 5;
        /* 4. W5500 + ModbusTCP */
        GPIO_SetBits(GPIOA, GPIO_Pin_1);        /* DBG: 高 = W5500 开始 */
        W5500_TCPServer_Run();
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);       /* DBG: 低 = W5500 结束 */
        g_dbg_step = 6;
        ModbusTCP_Process();
        ModbusTCP_SyncFromCAN();
        if (W5500_IsConnected()) {
            if (!g_eth_was_connected) { ModbusTCP_OnReconnect(); g_eth_was_connected = 1; }
        } else {
            if (g_eth_was_connected) { ModbusTCP_OnDisconnect(); g_eth_was_connected = 0; }
        }

        /* 5. Key scan (4-page cycle, 10s auto-return) */
        if (KEY_Page_Scan()) {
            g_display_page = (g_display_page + 1) % 4;
            g_last_page_switch_tick = Delay_GetTick();
            OLED_Clear();
        }

        IWDG_ReloadCounter();                   /* 喂狗紧贴 vTaskDelayUntil */
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

/* ---- 复位原因检测 ---- */
static const char *ResetCauseStr(void)
{
    uint32_t csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;                  /* 清除标志 */
    if (csr & RCC_CSR_IWDGRSTF) return "IWDG"; /* 看门狗超时 */
    if (csr & RCC_CSR_SFTRSTF)  return "SW";   /* 软件复位 */
    if (csr & RCC_CSR_PORRSTF)  return "POR";  /* 上电复位 */
    if (csr & RCC_CSR_PINRSTF)  return "NRST"; /* 外部复位 */
    return "UNKN";
}

int main(void)
{
    int8_t ret;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    Delay_InitTick(); Delay_ms(200);
    OLED_Init(); OLED_Clear();

    /* DBG: PA1 输出 — 万用表量电压定位卡死点 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    {   GPIO_InitTypeDef g;
        g.GPIO_Pin = GPIO_Pin_1; g.GPIO_Mode = GPIO_Mode_Out_PP;
        g.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &g);
    }

    /* 显示上次复位原因 */
    {   char dbg[17];
        sprintf(dbg, "RST:%s", ResetCauseStr());
        OLED_ShowString(1, 1, dbg);
        Delay_ms(1500);
    }

    OLED_Clear();
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

    xTaskCreate(Task_CAN_Drain,"CANDrain", STACK_CAN_DRAIN, NULL, 3, &hTask_CAN_Drain);
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
