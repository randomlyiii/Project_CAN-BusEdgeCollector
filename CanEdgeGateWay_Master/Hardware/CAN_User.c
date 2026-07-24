#include "CAN_User.h"
#include "delay.h"
#include <string.h>

SlaveNode_t      g_slave_nodes[MAX_SLAVE_NODES];
CAN_ErrorStatus_t g_can_error = {0};
volatile uint8_t  g_can_rx_flag = 0;

static CanRxMsg   g_rx_msg;
static uint32_t   g_tx_count_per_window = 0;
static uint32_t   g_rx_count_per_window = 0;
static uint32_t   g_window_start_tick = 0;

uint32_t g_can_rx_int_count  = 0;
uint32_t g_can_rx_temp_count = 0;

__weak void CAN_User_OnError(uint8_t level) { (void)level; }
__weak void CAN_User_OnAlarm(uint8_t node_id, uint8_t func) { (void)node_id; (void)func; }
__weak void CAN_User_OnNodeUpdate(uint8_t node_id) { (void)node_id; }

/* CAN GPIO: PA11=RX, PA12=TX */
static void CAN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_11;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

static void CAN_NVIC_Init(void)
{
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                   = CAN1_SCE_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/* CAN初始化: 500kbps */
void CAN_User_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;
    uint8_t i;

    memset(g_slave_nodes, 0, sizeof(g_slave_nodes));
    g_can_error.error_level = 0;
    g_can_error.tec = 0;
    g_can_error.rec = 0;
    g_can_error.bus_load = 0;

    CAN_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    can.CAN_TTCM       = DISABLE;
    can.CAN_ABOM       = ENABLE;
    can.CAN_AWUM       = ENABLE;
    can.CAN_NART       = DISABLE;
    can.CAN_RFLM       = DISABLE;
    can.CAN_TXFP       = DISABLE;
    can.CAN_Mode       = CAN_Mode_Normal;
    can.CAN_SJW        = CAN_SJW_1tq;
    can.CAN_BS1        = CAN_BS1_5tq;
    can.CAN_BS2        = CAN_BS2_6tq;
    can.CAN_Prescaler  = 5;    // 500kbps @ 36MHz APB1
    CAN_Init(CAN1, &can);

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

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_ERR, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_BOF, ENABLE);
    CAN_NVIC_Init();

    g_window_start_tick = Delay_GetTick();

    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        g_slave_nodes[i].node_id = i + 1;
        g_slave_nodes[i].online  = 0;
    }
}

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

    timeout = 1000;
    mailbox = CAN_Transmit(CAN1, &tx_msg);
    while (CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok && timeout--)
        Delay_us(1);
    if (timeout == 0) return 1;
    g_tx_count_per_window++;
    return 0;
}

/* 在ISR内直接处理一帧, 避免多帧覆盖 */
static void ProcessOneFrame(CanRxMsg *p)
{
    uint8_t  src_id = p->Data[CAN_DATA_SRC_IDX];
    uint8_t  func   = p->Data[CAN_DATA_FUNC_IDX];
    uint32_t now    = Delay_GetTick();
    uint8_t  i, found;

    g_can_rx_int_count++;

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

    /* 黑名单只拦截温湿度上报, 心跳/报警/恢复照常通过 */
    if (g_slave_nodes[i].blacklist
        && func != CAN_FUNC_RECOVER
        && func != CAN_FUNC_HEARTBEAT
        && func != CAN_FUNC_ALARM)
        return;

    /* 心跳/报警到达即视为节点恢复, 自动解除黑名单 */
    if (g_slave_nodes[i].blacklist
        && (func == CAN_FUNC_HEARTBEAT || func == CAN_FUNC_ALARM))
        g_slave_nodes[i].blacklist = 0;

    switch (func) {
    case CAN_FUNC_HEARTBEAT:
        g_slave_nodes[i].last_heartbeat_tick = now;
        g_slave_nodes[i].heartbeat_count++;
        g_slave_nodes[i].online = 1;
        break;

    case CAN_FUNC_TEMP_HUMI:
        g_can_rx_temp_count++;
        g_slave_nodes[i].last_heartbeat_tick = now;
        g_slave_nodes[i].online = 1;
        g_slave_nodes[i].temp_int = p->Data[CAN_DATA_PAYLOAD_IDX];
        g_slave_nodes[i].temp_dec = p->Data[CAN_DATA_PAYLOAD_IDX + 1];
        g_slave_nodes[i].humi_int = p->Data[CAN_DATA_PAYLOAD_IDX + 2];
        g_slave_nodes[i].humi_dec = p->Data[CAN_DATA_PAYLOAD_IDX + 3];
        break;

    case CAN_FUNC_ALARM:
        g_slave_nodes[i].fault_flag = 1;
        g_slave_nodes[i].online = 1;
        g_slave_nodes[i].last_heartbeat_tick = now;
        break;

    case CAN_FUNC_RECOVER:
        g_slave_nodes[i].fault_flag = 0;
        g_slave_nodes[i].blacklist = 0;
        g_slave_nodes[i].online = 1;
        g_slave_nodes[i].last_heartbeat_tick = now;
        break;
    }
    g_rx_count_per_window++;
}

/* ISR中直接收+处理, 每帧独立不覆盖 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg msg;

    while (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CAN_Receive(CAN1, CAN_FIFO0, &msg);
        ProcessOneFrame(&msg);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/* 主循环不用调CAN_ProcessRxFrame, ISR已处理 */
void CAN_ProcessRxFrame(void)
{
}

void CAN1_SCE_IRQHandler(void)
{
    uint8_t tec, rec;

    if (CAN_GetITStatus(CAN1, CAN_IT_ERR) != RESET) {
        tec = (CAN1->ESR >> 16) & 0xFF;
        rec = (CAN1->ESR >> 24) & 0xFF;
        g_can_error.tec = tec;
        g_can_error.rec = rec;

        if (CAN1->ESR & ((uint32_t)0x00000004))
            g_can_error.error_level = 3;
        else if (tec > 127 || rec > 127)
            g_can_error.error_level = 2;
        else if (tec > 96 || rec > 96)
            g_can_error.error_level = 1;
        else
            g_can_error.error_level = 0;

        CAN_User_OnError(g_can_error.error_level);
        CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
    }
    if (CAN_GetITStatus(CAN1, CAN_IT_BOF) != RESET)
        CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
}

void CAN_HeartBeatCheck(void)
{
    uint32_t now = Delay_GetTick();
    for (uint8_t i = 0; i < MAX_SLAVE_NODES; i++) {
        if (g_slave_nodes[i].node_id == 0) continue;
        if (g_slave_nodes[i].online) {
            if (now - g_slave_nodes[i].last_heartbeat_tick > HEARTBEAT_TIMEOUT_MS) {
                g_slave_nodes[i].online = 0;
                g_slave_nodes[i].heartbeat_count = 0;
                CAN_User_OnNodeUpdate(g_slave_nodes[i].node_id);
            }
        }
        if (!g_slave_nodes[i].online && !g_slave_nodes[i].blacklist && g_slave_nodes[i].node_id != 0) {
            g_slave_nodes[i].blacklist = 1;
            g_slave_nodes[i].blacklist_start_tick = now;
        }
        if (g_slave_nodes[i].blacklist) {
            if (now - g_slave_nodes[i].blacklist_start_tick > NODE_BLACKLIST_TIMEOUT_MS)
                g_slave_nodes[i].blacklist = 0;
        }
    }
}

void CAN_ErrorMonitor(void)
{
    if (g_can_error.error_level == 3)
        CAN_ResetBus();
}

void CAN_CalcBusLoad(void)
{
    uint32_t now = Delay_GetTick();
    uint32_t elapsed = now - g_window_start_tick;
    if (elapsed < 100) return;

    uint32_t total = g_tx_count_per_window + g_rx_count_per_window;
    /* 归一化到每秒帧数, 再除以500kbps理论最大帧率(~4500fps), 得到千分比 */
    uint32_t fps = total * 1000 / elapsed;
    uint32_t max_fps = 4500;  /* 500kbps 标准帧8字节数据理论最大值 */
    if (fps > max_fps) fps = max_fps;
    g_can_error.bus_load = (uint16_t)(fps * 10000 / max_fps);

    g_tx_count_per_window = 0;
    g_rx_count_per_window = 0;
    g_window_start_tick = now;
}

void CAN_ResetBus(void)
{
    CAN_DeInit(CAN1);
    Delay_ms(10);
    CAN_User_Init();
}
