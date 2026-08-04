/**
 * CAN User Layer — Final Phase (ISR-only-copy + sequential task drain)
 *
 * Architecture (dual-FIFO + dual lock-free ring buffer):
 *
 *   [HW FIFO0: 紧急 0x100-0x1FF]  →  USB_LP_CAN1_RX0_IRQHandler
 *   [HW FIFO1: 常态 0x200-0x3FF]  →  CAN1_RX1_IRQHandler
 *        ↓ (ISR 只搬运，不解析，各写各的 ring)
 *   ring_push → g_rx_ring_high / g_rx_ring_norm (锁无关 SPSC)
 *        ↓ (Task 10ms 周期 drain)
 *   ring_pop → CAN_ProcessFrame (业务逻辑在这里)
 *
 * ISR 只做三件事: CAN_Receive → ring_push → 清 IT 标志 → 退出
 * Task 绝对不碰硬件 FIFO，ISR 绝对不做协议解析
 */

#include "CAN_User.h"
#include "delay.h"
#include <string.h>

/* ---- Globals ---- */
SlaveNode_t         g_slave_nodes[MAX_SLAVE_NODES];
CAN_ErrorStatus_t   g_can_error = {0};
PriorityOverride_t  g_priority_override[MAX_SLAVE_NODES];
volatile uint32_t   g_can_rx_int_count  = 0;
volatile uint32_t   g_can_rx_temp_count = 0;
volatile uint16_t   g_bus_off_recovery_cnt = 0;
volatile uint8_t    g_system_throttle_level = 0;  /* 0=norm, 1=low, 2=emerg */
uint8_t             g_boff_consec = 0;     /* 连续 BusOff 计数, 收到帧时清零 */

/* ---- FreeRTOS sync objects ---- */
/* DBG: 运行计数 — 挂死前最后值定位卡死段 */
volatile uint8_t g_dbg_step = 0;

/* NOTE: RX path uses lock-free ring buffers (g_rx_ring_high/norm).
 *       Semaphore-free periodic drain — see main.c Task_Unified.
 *       TX FIFO 不再推入，不再需要信号量。 */

/* ---- RX Ring Buffers (ISR → Task, lock-free SPSC, each ring own ISR) ---- */
RxRingHigh          g_rx_ring_high;
RxRingNorm          g_rx_ring_norm;

/* ---- ISR → task shared frame buffer ---- */
CanRxFrame          g_isr_rx_frame;

/* ---- Load calculation ---- */
static uint32_t g_tx_bit_count   = 0;
static uint32_t g_rx_bit_count   = 0;
static uint32_t g_last_bus_activity = 0;  /* 最近帧收发时间(ms) */
static uint32_t g_window_start   = 0;
static uint32_t g_throttle_stable_cnt = 0;  /* hysteresis counter */
#define LOAD_WINDOW_MS          100
#define THROTTLE_HYST_CNT       5     /* 500ms stable before recovery */
#define LOAD_THRESHOLD_LOW      7000  /* 70.00% */
#define LOAD_THRESHOLD_HIGH     9000  /* 90.00% */
#define LOAD_RECOVER_MID        8500  /* <85% exit emergency */
#define LOAD_RECOVER_LOW        6500  /* <65% exit low-freq throttle */

/* ---- 负载折算位数: 8字节标准帧实际 ~122 位 (0x55 低填充实测, 500k/4100fps)。
 *     原 108 是无填充最小值, 折算低估真实总线占用 (4100fps 显示 88.56% 实为 ~100%)。
 *     节流阈值随之移动: 90% → 500k×0.9/122 ≈ 3690fps 真实帧。 ---- */
#define CAN_FRAME_BITS          122

/* ---- Weak callbacks ---- */
__weak void CAN_User_OnError(uint8_t level)      { (void)level; }
__weak void CAN_User_OnAlarm(uint8_t node_id, uint8_t func) { (void)node_id; (void)func; }
__weak void CAN_User_OnNodeUpdate(uint8_t node_id) { (void)node_id; }

/* ==================== GPIO + NVIC ==================== */

static void CAN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_11;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;          /* 匹配厂家参考: 内部上拉防浮空噪声 */
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

static void CAN_NVIC_Init(void)
{
    NVIC_InitTypeDef nvic;

    /* CAN RX0 interrupt — priority 5 (≥ configMAX_SYSCALL_INTERRUPT_PRIORITY).
     * Must be ≥5 to safely call FreeRTOS ISR functions.
     * Priorities 0-4 are NOT masked by FreeRTOS BASEPRI critical sections. */
    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 8;     /* < configMAX_SYSCALL(5), 不干扰调度 */
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* CAN RX1 interrupt (FIFO1 — normal/low-freq frames) */
    nvic.NVIC_IRQChannel                   = CAN1_RX1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 8;     /* 同 FIFO0 优先级 */
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* CAN SCE (error) interrupt — priority 6 */
    nvic.NVIC_IRQChannel                   = CAN1_SCE_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/* ==================== CAN Hardware Init (不碰 g_slave_nodes) ====================
 *
 * 返回 0=成功, 1=CAN_Init 失败(总线持续显性/时钟异常).
 * 调用方不应死等, 应由 ErrorMonitor 限频重试.
 * ================================================================ */

static uint8_t CAN_Hardware_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;

    /* 注意: 不在此清除 error_level — 只有调用方知道这次初始化是首启还是 BusOff 恢复,
     *       若调用方在恢复期间失败, error_level 应保持 3 以便 ErrorMonitor 下次再试.
     *       清零时机在 CAN_Init 成功后由调用方处理. */

    CAN_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    can.CAN_TTCM       = DISABLE;
    can.CAN_ABOM       = DISABLE;  /* ErrorMonitor 软件恢复(1s退避), 避免 ABOM 2.8ms 快速恢复
                                      导致 2 节点下 BusOff↔Normal 高频震荡, 永远收不到帧.
                                      3 节点 OK, 2 节点 WRN R=0 的观测证实此问题. */
    can.CAN_AWUM       = DISABLE;
    can.CAN_NART       = DISABLE;  /* 原始: 自动重传 */
    can.CAN_RFLM       = DISABLE;
    can.CAN_TXFP       = DISABLE;
    can.CAN_Mode       = CAN_Mode_Normal;
    /* 500kbps: 36MHz/(6×12tq)=500kbps. 采样点从 50%(5,6) 改为 75%(8,3).
       75% 采样点更适合 2 节点下的信号完整性, 降低 CRC 错误率.
       阶段二原始时序为 5,6 (50%), 3 节点正常但 2 节点频繁 CRC 错. */
    can.CAN_SJW        = CAN_SJW_2tq;
    can.CAN_BS1        = CAN_BS1_8tq;
    can.CAN_BS2        = CAN_BS2_3tq;
    can.CAN_Prescaler  = 6;
    if (CAN_Init(CAN1, &can) == CANINITFAILED)
        return 1;  /* 调用方重试, 不死循环 */

    /* CAN_Init 成功 → 安全清除错误计数器 */
    g_can_error.error_level   = 0;
    g_can_error.tec           = 0;
    g_can_error.rec           = 0;
    g_can_error.bus_load      = 0;
    g_can_error.throttle_level = 0;
    g_system_throttle_level   = 0;
    g_bus_off_recovery_cnt    = 0;

    /* ================================================================
     * 全通 Filter — 所有帧接收, 不分优先级
     *   (之前双 Filter 可能 ID 范围未覆盖从站帧导致 RX=0)
     * ================================================================ */
    filter.CAN_FilterNumber           = 0;
    filter.CAN_FilterMode             = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale            = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh           = 0x0000;
    filter.CAN_FilterIdLow            = 0x0000;
    filter.CAN_FilterMaskIdHigh       = 0x0000;   /* 全 0 = 全部接受 */
    filter.CAN_FilterMaskIdLow        = 0x0000;
    filter.CAN_FilterFIFOAssignment   = CAN_FIFO0; /* 所有帧走 FIFO0 */
    filter.CAN_FilterActivation       = ENABLE;
    CAN_FilterInit(&filter);

    /* Init RX ring buffers */
    memset(&g_rx_ring_high, 0, sizeof(g_rx_ring_high));
    memset(&g_rx_ring_norm, 0, sizeof(g_rx_ring_norm));

    /* FIFO init (creates mutexes — must also be done before interrupts) */
    FIFO_Init();

    g_window_start = Delay_GetTick();
    g_last_bus_activity = Delay_GetTick();
    return 0;
}

/* ==================== CAN Full Init (硬件 + 节点记录) ==================== */

void CAN_User_Init(void)
{
    uint8_t i;

    memset(g_slave_nodes, 0, sizeof(g_slave_nodes));
    memset(g_priority_override, 0, sizeof(g_priority_override));

    /* 首次初始化: 可以重试, 但不应死等 */
    {
        uint8_t retry = 5;
        while (retry--) {
            if (CAN_Hardware_Init() == 0) break;
            Delay_ms(100);
        }
        /* CAN_Hardware_Init 成功后已在内部清零 error_level.
         * 若全部重试失败, 保持 error_level 为之前状态 (3=BusOff 或 0). */
    }

    /* 预填 2 个已知从站 ID, 其余留空(slot=0)支持动态注册 */
    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        g_slave_nodes[i].online = 0;
        if (i < 2)
            g_slave_nodes[i].node_id = i + 1;   /* slave 1, 2 */
    }
}

/* ==================== CAN Enable Interrupts (delayed) ==================== */

void CAN_EnableInterrupts(void)
{
    /* FMP0 中断: RX0 ISR 把 FIFO0 帧搬入 ring_high (无锁 SPSC push).
       FMP1 保持禁用 — 全通滤波所有帧走 FIFO0, 且空 FIFO1 检查曾致异常.
       ERR/BOF 中断禁用 — SCE ISR 风暴曾抢占 Task 导致心跳超时. */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN_NVIC_Init();
}

/* ==================== CAN BusOff 恢复 (INRQ 退出初始化模式) ====================
 *
 * 不再使用 CAN_DeInit + CAN_Hardware_Init (软件全复位违反 CAN 协议).
 * 改为 INRQ 进入/退出初始化模式 → 硬件自动完成 128 隐性位检测 → 恢复.
 * 时序寄存器保持不变, 无需重配.
 *
 * 参考: RM0008 §25.7.4 / 工程实操笔记 §5
 * ================================================================ */

void CAN_ResetBus(void)
{
    uint32_t timeout;
    uint8_t  ok = 0;
    CanRxMsg dummy;

    /* Step 1: 请求进入初始化模式 (INRQ=1) */
    CAN1->MCR |= CAN_MCR_INRQ;

    /* Step 2: 等待 INAK 确认 (最长 ~1ms, 开 5000 循环保险) */
    timeout = 5000;
    while (!(CAN1->MSR & CAN_MSR_INAK)) {
        if (--timeout == 0) { ok = 0; goto finish; }
    }

    /* Step 3: 清 LEC 记录 (不写 ESR, LEC 自动清除) */

    /* Step 4: 退出初始化模式 (INRQ=0), 硬件走 128 隐性位恢复 */
    CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;

    /* Step 5: 等待 INAK 清除, 确认回到正常模式 */
    timeout = 5000;
    while ((CAN1->MSR & CAN_MSR_INAK)) {
        if (--timeout == 0) { ok = 0; goto finish; }
    }
    ok = 1;

    /* 等待硬件完成 128 隐性位检测 (ESR.BOFF 自动清零) */
    {
        uint32_t poll;
        for (poll = 0; poll < 20000; poll++) {   /* ~20ms 最大等待 */
            if (!(CAN1->ESR & ((uint32_t)0x00000004)))  /* BOFF bit */
                break;
            Delay_us(1);
        }
    }

    /* Drain FIFO0 脏帧 (恢复期间可能堆积错误帧) */
    for (uint8_t i = 0; i < 6; i++) {
        if (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
            CAN_Receive(CAN1, CAN_FIFO0, &dummy);
    }

    g_can_error.tec = 0;
    g_can_error.rec = 0;
    g_can_error.error_level = 0;
    g_bus_off_recovery_cnt++;

    return;

finish:
    if (!ok) {
        /* INRQ 超时: 总线持续显性导致无法同步?
         * 留 error_level=3, 让 ErrorMonitor 下次冷却到期再试. */
        CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;  /* 无论如何退出 init */
        g_can_error.error_level = 3;
        return;
    }
}

/* ==================== Frame Send ==================== */

uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len)
{
    CanTxMsg tx_msg;
    uint8_t  i, mailbox;
    uint32_t timeout;

    if (len > 8) len = 8;
    tx_msg.ExtId = 0;
    tx_msg.IDE   = CAN_Id_Standard;
    tx_msg.RTR   = CAN_RTR_Data;
    tx_msg.StdId = id;
    tx_msg.DLC   = len;
    for (i = 0; i < len; i++) tx_msg.Data[i] = data[i];

    mailbox = CAN_Transmit(CAN1, &tx_msg);
    if (mailbox == CAN_TxStatus_NoMailBox)
        return 1;                                   /* 无空闲邮箱 */

    timeout = 1000;
    while (timeout--) {
        if (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok)
            goto send_ok;
        Delay_us(1);
    }
    return 1;

send_ok:
    g_tx_bit_count += CAN_FRAME_BITS;
    g_last_bus_activity = Delay_GetTick();
    return 0;
}

/* ================================================================
 * ISR: CAN FIFO0 Receive — 主接收路径
 *
 * 职责 ONLY: CAN_Receive → pack 紧凑帧 → 无锁 push ring_high → 退出
 * 禁止: FreeRTOS API (prio 8 < syscall 阈值 5), 阻塞, 协议解析。
 * 上一版 ISR 崩溃根因: ISR 内调 RTOS API + 空 FIFO1 检查 (本版已规避)。
 * ================================================================ */

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    while (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CanRxMsg    msg;
        RingFrame_t rf;
        CAN_Receive(CAN1, CAN_FIFO0, &msg);
        rf.StdId = (uint16_t)msg.StdId;
        rf.DLC   = msg.DLC;
        memcpy(rf.Data, msg.Data, 8);
        RxRingHigh_push(&g_rx_ring_high, &rf);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/* ================================================================
 * ISR: CAN FIFO1 Receive (常态+低频帧 0x200-0x3FF)
 *
 * 职责 ONLY: 读硬件 → 推环形缓冲 → 退出
 * ================================================================ */

void CAN1_RX1_IRQHandler(void)
{
    while (CAN_GetITStatus(CAN1, CAN_IT_FMP1) != RESET) {
        CanRxMsg    msg;
        RingFrame_t rf;
        CAN_Receive(CAN1, CAN_FIFO1, &msg);
        rf.StdId = (uint16_t)msg.StdId;
        rf.DLC   = msg.DLC;
        memcpy(rf.Data, msg.Data, 8);
        RxRingNorm_push(&g_rx_ring_norm, &rf);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
    }
}

/* ================================================================
 * ISR: CAN Status Change / Error — 禁用 (ERR/BOF 中断未使能)
 *
 * 拔线时反射产生连续位错误 → SCE ISR 风暴抢占 Task CPU →
 * Task 不能 drain ring → 心跳超时 → 全部掉线。
 * 现 ErrorMonitor 已不调 CAN_ResetBus, SCE 数据仅用于诊断,
 * 关闭 SCE 中断消除 ISR 风暴, 保住正常帧接收路径。
 * ================================================================ */

void CAN1_SCE_IRQHandler(void)
{
    /* ERR/BOF 中断已禁用, 此函数不应被执行 */
}

/* ==================== Process one frame (called from vTask_CAN_Rx) ==================== */

void CAN_ProcessFrame(CanRxFrame *p)
{
    uint8_t  src_id = p->Data[CAN_DATA_SRC_IDX];
    uint8_t  func   = p->Data[CAN_DATA_FUNC_IDX];
    uint8_t  prio   = p->Data[CAN_DATA_TYPE_IDX];
    uint32_t now    = Delay_GetTick();
    uint8_t  i, found;

    /* DLC 校验: 协议约定 8 字节, 非法长度直接丢弃 */
    if (p->DLC != 8) return;

    /* Checksum 校验: byte7 = XOR(byte0~6), 防总线干扰帧污染节点状态 */
    {
        uint8_t chk = 0;
        for (uint8_t ci = 0; ci < 7; ci++) chk ^= p->Data[ci];
        if (chk != p->Data[CAN_DATA_CHKSUM_IDX]) return;
    }

    g_can_rx_int_count++;
    g_rx_bit_count += CAN_FRAME_BITS;
    g_last_bus_activity = Delay_GetTick();

    /* Find or register slave node */
    found = 0;
    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == src_id) { found = 1; break; }
    }
    if (!found) {
        for (i = 0; i < MAX_SLAVE_NODES; i++) {
            if (g_slave_nodes[i].node_id == 0) {
                g_slave_nodes[i].node_id = src_id;
                found = 1;
                break;
            }
        }
    }
    if (!found) return;
    if (i >= MAX_SLAVE_NODES) return;   /* 安全边界校验 */

    /* 任何来自该节点的帧都证明它存活 — 重置所有离线判定 */
    g_slave_nodes[i].online = 1;
    g_slave_nodes[i].stale = 0;
    g_slave_nodes[i].offline = 0;
    g_slave_nodes[i].expire_start_tick = 0;
    g_slave_nodes[i].last_heartbeat_tick = now;
    g_boff_consec = 0;  /* 收到帧 → 总线正常, 清零连续 BusOff 计数 */

    /* Check priority override */
    if (g_priority_override[i].overridden && prio != 0) {
        prio = 0;
        p->Data[CAN_DATA_TYPE_IDX] = 0;
    }

    /* Process by function code */
    switch (func) {
    case CAN_FUNC_HEARTBEAT:
        g_slave_nodes[i].heartbeat_count++;
        break;

    case CAN_FUNC_TEMP_HUMI:
        g_can_rx_temp_count++;
        g_slave_nodes[i].temp_int = p->Data[CAN_DATA_PAYLOAD_IDX];
        g_slave_nodes[i].temp_dec = p->Data[CAN_DATA_PAYLOAD_IDX + 1];
        g_slave_nodes[i].humi_int = p->Data[CAN_DATA_PAYLOAD_IDX + 2];
        g_slave_nodes[i].humi_dec = p->Data[CAN_DATA_PAYLOAD_IDX + 3];
        break;

    case CAN_FUNC_ALARM: {
        static uint32_t last_alarm[MAX_SLAVE_NODES] = {0};
        uint32_t now_a = Delay_GetTick();
        if (now_a - last_alarm[src_id] < 200) break;
        last_alarm[src_id] = now_a;
        g_slave_nodes[i].fault_flag = 1;
        CAN_User_OnAlarm(src_id, func);
        break;
    }

    case CAN_FUNC_RECOVER:
        g_slave_nodes[i].fault_flag = 0;
        break;

    case CAN_FUNC_LIGHT:
        /* Dual-use: BH1750 lux OR LM393 AO analog (same payload format) */
        g_slave_nodes[i].light_lux =
            ((uint16_t)p->Data[CAN_DATA_PAYLOAD_IDX] << 8)
            | p->Data[CAN_DATA_PAYLOAD_IDX + 1];
        g_slave_nodes[i].lm393_analog = g_slave_nodes[i].light_lux;
        g_slave_nodes[i].lm393_digital = p->Data[CAN_DATA_PAYLOAD_IDX + 2];
        break;

    case CAN_FUNC_LM393_DO:
        g_slave_nodes[i].lm393_digital = p->Data[CAN_DATA_PAYLOAD_IDX];
        break;

    case CAN_FUNC_RESERVED:
        g_slave_nodes[i].reserved_ch1 =
            ((uint16_t)p->Data[CAN_DATA_PAYLOAD_IDX] << 8)
            | p->Data[CAN_DATA_PAYLOAD_IDX + 1];
        break;
    }

    /* 推入 FIFO (按优先级分流).
     * 主站 receive-only, 不自发帧, 不存在自接收回环问题.
     * FIFO 数据由 ModbusTCP_SyncFromCAN 消费 → 历史缓存 → 断网恢复后批量上传. */
    if (prio == 0)
        FIFO_High_Push(p);
    else
        FIFO_Normal_Push(p);
}

/* ==================== Heartbeat check (called from vTask_CAN_Monitor) ==================== */

void CAN_HeartBeatCheck(void)
{
    uint32_t now;
    uint8_t i;

    /* 每节点独立刷新(checksum已验证), 单节点断线不牵连其他节点.
     * 去掉 error_level 门禁 — 避免 error_passive 周期全节点假离线. */
    now = Delay_GetTick();

    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;
        if (!g_slave_nodes[i].online) continue;

        uint32_t delta = now - g_slave_nodes[i].last_heartbeat_tick;

        if (delta > STALE_TIMEOUT_MS) {
            /* 超过 3s → 进入过期 */
            g_slave_nodes[i].stale = 1;

            /* 刚进入过期, 记录起始时间戳 (只在 stale 首次置位时打) */
            if (g_slave_nodes[i].expire_start_tick == 0)
                g_slave_nodes[i].expire_start_tick = now;

            /* 持续过期超过 30s → 判长期离线 */
            if (now - g_slave_nodes[i].expire_start_tick > OFFLINE_CONFIRM_MS)
                g_slave_nodes[i].offline = 1;
        } else {
            /* 帧在 3s 内收到 → 清除过期标记 */
            g_slave_nodes[i].stale = 0;
            g_slave_nodes[i].offline = 0;
            g_slave_nodes[i].expire_start_tick = 0;
        }
    }
}

/* ==================== Error monitor + Bus-Off recovery ====================
 *
 * ABOM=DISABLE: 2 节点信号完整性差(终端阻抗匹配), 恢复后收 1-2 帧又 BusOff.
 * 固定 200ms 恢复: 确保 STALE_TIMEOUT(3s) 内完成多次恢复, 总有帧能刷新 timestamp.
 * 3 节点完美运行, 2 节点有 WRN/BOF 循环.
 *
 * 恢复机制:
 *   - 正常 BusOff → 200ms 恢复 (2 节点每恢复收 1-2 帧又 BusOff)
 *   - 连续 5 次 BusOff → 拉长到 3s, 避免无效高频重置
 *   - CAN_ProcessFrame 收到帧时清零连续计数 (见 CAN_ProcessFrame 顶部)
 * ================================================================ */
#define BOFF_COOLDOWN_MS        200
#define BOFF_COOLDOWN_MAX_MS   3000
#define BOFF_CONSEC_THRESH     5
#define BOFF_STABLE_RESET_MS   30000

void CAN_ErrorMonitor(void)
{
    static uint32_t last_recover_tick = 0;
    static uint32_t ok_since = 0;
    static uint8_t  boff_counted = 0;   /* 边沿检测: 每次 BusOff 事件仅累加一次 */

    uint32_t esr = CAN1->ESR;
    uint8_t  tec = (uint8_t)((esr >> 16) & 0xFF);
    uint8_t  rec = (uint8_t)((esr >> 24) & 0xFF);

    g_can_error.tec = tec;
    g_can_error.rec = rec;

    /* 若 error_level==3 是被 CAN_ResetBus 强制设置的(硬件 init 失败但 ESR 已被 DeInit 清空),
     * 不要被 ESR 覆盖掉, 否则 ErrorMonitor 以为总线正常, 永不触发恢复. */
    if (g_can_error.error_level != 3) {
        if (esr & ((uint32_t)0x00000004)) {
            g_can_error.error_level = 3;
        } else if (tec > 96 || rec > 96) {
            g_can_error.error_level = 1;
        } else {
            g_can_error.error_level = 0;
        }
    }

    /* ---- BusOff 恢复: 连续 N 次后拉长间隔 ----
     * 边沿检测: 用 boff_counted 标记是否已为本次 BusOff 事件累加.
     * 避免 10ms 周期内在等待冷却期间重复累加导致过早拉长到 3s.
     * boff_counted 在退出 error_level==3 时清零. */
    if (g_can_error.error_level == 3) {
        uint32_t now = Delay_GetTick();
        uint32_t cooldown = BOFF_COOLDOWN_MS;

        if (!boff_counted) {
            g_boff_consec++;
            boff_counted = 1;
        }

        if (g_boff_consec > BOFF_CONSEC_THRESH)
            cooldown = BOFF_COOLDOWN_MAX_MS;

        if (now - last_recover_tick >= cooldown) {
            CAN_ResetBus();
            last_recover_tick = now;
        }
        ok_since = 0;
    } else if (g_can_error.error_level == 0) {
        uint32_t now = Delay_GetTick();
        boff_counted = 0;            /* 退出 BusOff, 下次事件重新计数 */
        if (ok_since == 0)
            ok_since = now;
        else if (now - ok_since > BOFF_STABLE_RESET_MS)
            ok_since = 0;
    } else {
        boff_counted = 0;            /* error_level 1/2 也清标记 */
        ok_since = 0;
    }

}

/* ==================== Bus load calculation + throttle ==================== */

void CAN_CalcBusLoad(void)
{
    uint32_t now     = Delay_GetTick();
    uint32_t elapsed = now - g_window_start;
    if (elapsed < LOAD_WINDOW_MS) return;

    /* Total bits in window */
    uint32_t total_frames  = g_tx_bit_count / CAN_FRAME_BITS
                           + g_rx_bit_count / CAN_FRAME_BITS;
    uint32_t bits_per_sec  = total_frames * CAN_FRAME_BITS * 1000UL / elapsed;
    uint32_t max_bps       = 500000UL;   /* 500kbps */
    uint32_t load           = (uint32_t)((uint64_t)bits_per_sec * 10000 / max_bps);
    if (load > 10000) load = 10000;
    /* 1s 内有过活动 → 最小 0.01% */
    if (load == 0 && g_last_bus_activity > 0 && now - g_last_bus_activity < 1000) load = 1;
    g_can_error.bus_load = (uint16_t)load;

    /* ---- Throttle decision with hysteresis ---- */
    uint16_t L = (uint16_t)load;

    if (L >= LOAD_THRESHOLD_HIGH) {
        /* Emergency mode: only level-0 + heartbeat allowed */
        g_system_throttle_level = 2;
        g_throttle_stable_cnt   = 0;
    } else if (L >= LOAD_THRESHOLD_LOW && g_system_throttle_level < 2) {
        /* Low-frequency throttle: double light sensor period */
        g_system_throttle_level = 1;
        g_throttle_stable_cnt   = 0;
    } else if (g_system_throttle_level == 2 && L < LOAD_RECOVER_MID) {
        g_throttle_stable_cnt++;
        if (g_throttle_stable_cnt >= THROTTLE_HYST_CNT) {
            g_system_throttle_level = 1;
            g_throttle_stable_cnt   = 0;
        }
    } else if (g_system_throttle_level == 1 && L < LOAD_RECOVER_LOW) {
        g_throttle_stable_cnt++;
        if (g_throttle_stable_cnt >= THROTTLE_HYST_CNT) {
            g_system_throttle_level = 0;
            g_throttle_stable_cnt   = 0;
        }
    } else if (L < LOAD_THRESHOLD_LOW) {
        g_throttle_stable_cnt++;
        if (g_throttle_stable_cnt >= THROTTLE_HYST_CNT) {
            g_system_throttle_level = 0;
            g_throttle_stable_cnt   = 0;
        }
    }

    g_can_error.throttle_level = g_system_throttle_level;

    /* Reset window */
    g_tx_bit_count = 0;
    g_rx_bit_count = 0;
    g_window_start = now;
}

/* ==================== Escalation / De-escalation logic ==================== */

void CAN_CheckEscalation(void)
{
    uint8_t i;

    /* Skip if bus is overloaded (protect emergency channel) */
    if (g_system_throttle_level >= 2) return;

    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;
        if (g_priority_override[i].overridden) continue;

        /* Check sensor fault conditions (from vTask_Protocol visibility) */
        if (g_slave_nodes[i].fault_flag) {
            g_priority_override[i].overridden       = 1;
            g_priority_override[i].original_prio    = 1;
            g_priority_override[i].escalate_tick    = Delay_GetTick();
            g_priority_override[i].escalation_reason = 2;
        }

        /* CAN passive error or worse → elevate this node */
        if (g_can_error.error_level >= 2) {
            g_priority_override[i].overridden       = 1;
            g_priority_override[i].original_prio    = 1;
            g_priority_override[i].escalate_tick    = Delay_GetTick();
            g_priority_override[i].escalation_reason = 3;
        }
    }
}

void CAN_CheckDeescalation(void)
{
    uint8_t  i;
    uint32_t now = Delay_GetTick();
    uint32_t timeout_ms = 500;

    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        if (!g_priority_override[i].overridden) continue;

        /* Check 500ms timeout */
        if (now - g_priority_override[i].escalate_tick < timeout_ms) continue;

        /* Check if fault condition cleared */
        if (g_priority_override[i].escalation_reason == 2) {
            /* Sensor fault — check if node recovered */
            if (g_slave_nodes[i].fault_flag == 0 && g_slave_nodes[i].online) {
                g_priority_override[i].overridden    = 0;
                g_priority_override[i].original_prio = 0;
                g_priority_override[i].escalation_reason = 0;
            }
        } else if (g_priority_override[i].escalation_reason == 3) {
            /* CAN error — check if error cleared */
            if (g_can_error.error_level < 2) {
                g_priority_override[i].overridden    = 0;
                g_priority_override[i].original_prio = 0;
                g_priority_override[i].escalation_reason = 0;
            }
        }
    }
}

/* Manual de-escalation (KEY2 from slave) */
void CAN_ManualDeescalate(uint8_t node_id)
{
    uint8_t i;
    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == node_id && g_priority_override[i].overridden) {
            g_priority_override[i].overridden       = 0;
            g_priority_override[i].original_prio    = 0;
            g_priority_override[i].escalation_reason = 0;
            g_slave_nodes[i].fault_flag = 0;
            break;
        }
    }
}
