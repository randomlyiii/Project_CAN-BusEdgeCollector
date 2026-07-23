#include "CAN_User.h"
#include "delay.h"
#include <string.h>

/* ------------------------- 全局变量定义 ------------------------- */
SlaveNode_t      g_slave_nodes[MAX_SLAVE_NODES];
CAN_ErrorStatus_t g_can_error = {0};
volatile uint8_t  g_can_rx_flag = 0;

/* 局部变量 */
static CanRxMsg   g_rx_msg;                  // 接收缓冲区(中断中填充)
static uint32_t   g_last_bus_calc_tick = 0;  // 上次总线计算tick
static uint32_t   g_tx_count_per_window = 0; // 窗口内发送帧数
static uint32_t   g_rx_count_per_window = 0; // 窗口内接收帧数
static uint32_t   g_window_start_tick = 0;   // 窗口起始tick
static uint8_t    g_node_count = 0;          // 发现的从站数量

/* ------------------------- CAN 错误处理回调 ------------------------- */
/* Weak -- 由 main.c 实现, 用于通知 OLED 更新 */
__weak void CAN_User_OnError(uint8_t level) { (void)level; }
__weak void CAN_User_OnAlarm(uint8_t node_id, uint8_t func) { (void)node_id; (void)func; }
__weak void CAN_User_OnNodeUpdate(uint8_t node_id) { (void)node_id; }

/* ------------------------- 硬件初始化 ------------------------- */
static void CAN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA11 CAN1_RX (浮空输入) */
    gpio.GPIO_Pin   = GPIO_Pin_11;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* PA12 CAN1_TX (复用推挽) */
    gpio.GPIO_Pin   = GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

static void CAN_NVIC_Init(void)
{
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;  // CAN RX0 中断
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                   = CAN1_SCE_IRQn;  // CAN 错误中断
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/* ------------------------- CAN 初始化(500kbps) ------------------------- */
void CAN_User_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;
    uint8_t i;

    /* 清零从站表 */
    memset(g_slave_nodes, 0, sizeof(g_slave_nodes));
    g_can_error.error_level = 0;
    g_can_error.tec = 0;
    g_can_error.rec = 0;
    g_can_error.bus_load = 0;

    CAN_GPIO_Init();

    /* 使能 CAN1 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* CAN 单元初始化 */
    can.CAN_TTCM       = DISABLE;  // 非时间触发
    can.CAN_ABOM       = ENABLE;   // 自动离线管理
    can.CAN_AWUM       = ENABLE;   // 自动唤醒
    can.CAN_NART       = DISABLE;  // 自动重传
    can.CAN_RFLM       = DISABLE;  // 非锁定接收FIFO
    can.CAN_TXFP       = DISABLE;  // 优先级:标识符
    can.CAN_Mode       = CAN_Mode_Normal;
    can.CAN_SJW        = CAN_SJW_1tq;
    can.CAN_BS1        = CAN_BS1_5tq;
    can.CAN_BS2        = CAN_BS2_6tq;
    can.CAN_Prescaler  = 4;        // 500kbps @ 36MHz APB1
    CAN_Init(CAN1, &can);

    /* 过滤器配置 —— 32位列表模式，接收所有帧(阶段一不过滤) */
    filter.CAN_FilterNumber           = 0;
    filter.CAN_FilterMode             = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale            = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh           = 0x0000;
    filter.CAN_FilterIdLow            = 0x0000;
    filter.CAN_FilterMaskIdHigh       = 0x0000;
    filter.CAN_FilterMaskIdLow        = 0x0000;
    filter.CAN_FilterFIFOAssignment   = CAN_FIFO0;
    filter.CAN_FilterActivation       = ENABLE;
    CAN_FilterInit(&filter);

    /* 使能 FIFO0 消息挂起中断 */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    /* 使能错误状态中断 */
    CAN_ITConfig(CAN1, CAN_IT_ERR, ENABLE);
    /* 使能总线错误中断 */
    CAN_ITConfig(CAN1, CAN_IT_BOF, ENABLE);

    CAN_NVIC_Init();

    /* 初始化总线负载计算窗口 */
    g_window_start_tick = Delay_GetTick();
    g_tx_count_per_window = 0;
    g_rx_count_per_window = 0;

    /* 填充默认从站ID(预配节点1~N) */
    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        g_slave_nodes[i].node_id = i + 1;
        g_slave_nodes[i].online  = 0;
    }
}

/* ------------------------- 发送CAN帧 ------------------------- */
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len)
{
    CanTxMsg tx_msg;
    uint8_t  i;
    uint8_t  mailbox;
    uint32_t timeout;

    if (len > 8) len = 8;

    tx_msg.ExtId = 0;
    tx_msg.IDE   = CAN_Id_Standard;
    tx_msg.RTR   = CAN_RTR_Data;
    tx_msg.StdId = id;
    tx_msg.DLC   = len;
    for (i = 0; i < len; i++)
        tx_msg.Data[i] = data[i];

    timeout = 1000; // ~1ms 超时
    mailbox = CAN_Transmit(CAN1, &tx_msg);
    while (CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok && timeout--)
        Delay_us(1);

    if (timeout == 0) return 1; // 发送超时
    g_tx_count_per_window++;    // 计入总线负载
    return 0; // 成功
}

/* ------------------------- 发送校验和计算 ------------------------- */
uint8_t CAN_CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++)
        chk ^= data[i];
    return chk;
}

/* ------------------------- 接收帧处理 ------------------------- */
void CAN_ProcessRxFrame(void)
{
    uint8_t  type, src_id, func;
    uint8_t  chk_calc, temp_int, temp_dec, humi_int, humi_dec;
    uint32_t now;
    uint8_t  i, found;

    if (!g_can_rx_flag) return;
    g_can_rx_flag = 0;  // 由中断置位，这里清掉

    /* 检查校验和 */
    chk_calc = CAN_CalcChecksum((uint8_t*)&g_rx_msg.Data[CAN_DATA_TYPE_IDX], 7);
    if (chk_calc != g_rx_msg.Data[CAN_DATA_CHKSUM_IDX])
        return;  // 校验失败，丢弃

    type   = g_rx_msg.Data[CAN_DATA_TYPE_IDX];
    src_id = g_rx_msg.Data[CAN_DATA_SRC_IDX];
    func   = g_rx_msg.Data[CAN_DATA_FUNC_IDX];
    now    = Delay_GetTick();

    /* 查找/注册从站节点 */
    found = 0;
    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == src_id) {
            found = 1;
            break;
        }
    }
    if (!found) {
        /* 新节点: 注册到空闲槽 */
        for (i = 0; i < MAX_SLAVE_NODES; i++) {
            if (g_slave_nodes[i].node_id == 0) {
                g_slave_nodes[i].node_id = src_id;
                found = 1;
                break;
            }
        }
    }
    if (!found) return; // 从站表满

    /* 黑名单中的节点只接收恢复帧 */
    if (g_slave_nodes[i].blacklist && func != CAN_FUNC_RECOVER)
        return;

    /* 按功能码分发 */
    switch (func) {
    case CAN_FUNC_HEARTBEAT:
        g_slave_nodes[i].last_heartbeat_tick = now;
        g_slave_nodes[i].heartbeat_count++;
        g_slave_nodes[i].online = 1;
        break;

    case CAN_FUNC_TEMP_HUMI:
        g_slave_nodes[i].last_heartbeat_tick = now;
        g_slave_nodes[i].online = 1;
        /* 数据载荷: Byte3=温度整数 Byte4=温度小数 Byte5=湿度整数 Byte6=湿度小数 */
        temp_int = g_rx_msg.Data[CAN_DATA_PAYLOAD_IDX];
        temp_dec = g_rx_msg.Data[CAN_DATA_PAYLOAD_IDX + 1];
        humi_int = g_rx_msg.Data[CAN_DATA_PAYLOAD_IDX + 2];
        humi_dec = g_rx_msg.Data[CAN_DATA_PAYLOAD_IDX + 3];
        g_slave_nodes[i].temp_int = temp_int;
        g_slave_nodes[i].temp_dec = temp_dec;
        g_slave_nodes[i].humi_int = humi_int;
        g_slave_nodes[i].humi_dec = humi_dec;
        CAN_User_OnNodeUpdate(src_id);
        break;

    case CAN_FUNC_ALARM:
        g_slave_nodes[i].fault_flag = 1;
        g_slave_nodes[i].online = 1;
        g_slave_nodes[i].last_heartbeat_tick = now;
        CAN_User_OnAlarm(src_id, func);
        break;

    case CAN_FUNC_RECOVER:
        g_slave_nodes[i].fault_flag = 0;
        g_slave_nodes[i].blacklist = 0; // 解除黑名单
        g_slave_nodes[i].online = 1;
        g_slave_nodes[i].last_heartbeat_tick = now;
        break;

    default:
        break;
    }

    g_rx_count_per_window++;
}

/* ------------------------- CAN RX0 中断 ------------------------- */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CAN_Receive(CAN1, CAN_FIFO0, &g_rx_msg);
        g_can_rx_flag = 1;
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/* ------------------------- CAN SCE 错误中断 ------------------------- */
void CAN1_SCE_IRQHandler(void)
{
    uint8_t tec, rec;
    uint32_t flags;

    flags = CAN_GetITStatus(CAN1, CAN_IT_ERR);
    if (flags != RESET) {
        /* 读取错误计数 */
        tec = (CAN1->ESR >> 16) & 0xFF;
        rec = (CAN1->ESR >> 24) & 0xFF;
        g_can_error.tec = tec;
        g_can_error.rec = rec;

        if (CAN1->ESR & CAN_ESR_BOF) {
            g_can_error.error_level = 3;
            CAN_User_OnError(3);
        } else if (tec > 127 || rec > 127) {
            g_can_error.error_level = 2;
            CAN_User_OnError(2);
        } else if (tec > 96 || rec > 96) {
            g_can_error.error_level = 1;
            CAN_User_OnError(1);
        } else {
            g_can_error.error_level = 0;
        }

        CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
    }

    /* 总线关闭中断 */
    if (CAN_GetITStatus(CAN1, CAN_IT_BOF) != RESET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
    }
}

/* ------------------------- 心跳超时检测 ------------------------- */
void CAN_HeartBeatCheck(void)
{
    uint32_t now = Delay_GetTick();

    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;

        /* 在线但超时 → 离线 */
        if (g_slave_nodes[i].online) {
            if (now - g_slave_nodes[i].last_heartbeat_tick > HEARTBEAT_TIMEOUT_MS) {
                g_slave_nodes[i].online = 0;
                g_slave_nodes[i].heartbeat_count = 0;
                CAN_User_OnNodeUpdate(g_slave_nodes[i].node_id);
            }
        }

        /* 离线节点加入黑名单 */
        if (!g_slave_nodes[i].online && !g_slave_nodes[i].blacklist && g_slave_nodes[i].node_id != 0) {
            g_slave_nodes[i].blacklist = 1;
            g_slave_nodes[i].blacklist_start_tick = now;
        }

        /* 黑名单到期自动解除 */
        if (g_slave_nodes[i].blacklist) {
            if (now - g_slave_nodes[i].blacklist_start_tick > NODE_BLACKLIST_TIMEOUT_MS) {
                g_slave_nodes[i].blacklist = 0;
                /* 节点仍在发送? 发送握手帧尝试恢复 */
            }
        }
    }
}

/* ------------------------- CAN 错误监测 ------------------------- */
void CAN_ErrorMonitor(void)
{
    /* 总线关闭自动恢复由 ABOM 硬件完成 */
    /* 这里只是更新状态给上层使用 */
    if (g_can_error.error_level == 3) {
        /* 软件辅助: 如果硬件 ABOM 没恢复，手动复位 */
        CAN_ResetBus();
    }
}

/* ------------------------- 总线负载率计算(100ms滑动窗口) ------------------------- */
void CAN_CalcBusLoad(void)
{
    uint32_t now = Delay_GetTick();
    uint32_t elapsed;
    uint32_t total_frames;

    elapsed = now - g_window_start_tick;
    if (elapsed < 100) return; // 每100ms计算一次

    total_frames = g_tx_count_per_window + g_rx_count_per_window;
    /* 理论最大值: 500kbps / 128us/帧 ≈ 62.5帧/ms, 100ms ≈ 6250帧 */
    /* 负载率 = 实际帧数 / 理论最大帧数 × 1000 */
    if (total_frames > 6250) total_frames = 6250;
    g_can_error.bus_load = (uint16_t)((uint32_t)total_frames * 1000 / 6250);

    /* 复位窗口 */
    g_tx_count_per_window = 0;
    g_rx_count_per_window = 0;
    g_window_start_tick = now;
}

/* ------------------------- 总线复位(软件辅助) ------------------------- */
void CAN_ResetBus(void)
{
    CAN_DeInit(CAN1);
    Delay_ms(10);
    CAN_User_Init();
}
