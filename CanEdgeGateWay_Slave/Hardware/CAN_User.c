#include "CAN_User.h"
#include "delay.h"
#include <string.h>

/* ========== 全局变量 ========== */
volatile uint8_t g_can_tx_done = 0;
volatile uint8_t g_can_rx_flag = 0;

/* 接收缓冲区(中断中填充) */
static CanRxMsg g_rx_msg;

/* ========== GPIO 初始化 ========== */
static void CAN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* PA11 CAN1_RX */
    gpio.GPIO_Pin   = GPIO_Pin_11;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    /* PA12 CAN1_TX */
    gpio.GPIO_Pin   = GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

/* ========== NVIC 初始化 ========== */
static void CAN_NVIC_Init(void)
{
    NVIC_InitTypeDef nvic;

    /* CAN TX 中断 */
    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/* ========== CAN 初始化 (500kbps) ========== */
void CAN_User_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;

    CAN_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* CAN 单元初始化 */
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
    can.CAN_Prescaler  = 4;        // 500kbps @ 36MHz APB1
    CAN_Init(CAN1, &can);

    /* 过滤器: 接收所有帧 */
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

    /* 中断: FIFO0 消息挂起 */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    /* 中断: 发送邮箱空 */
    CAN_ITConfig(CAN1, CAN_IT_TME,  ENABLE);

    CAN_NVIC_Init();
}

/* ========== 发送 CAN 帧 ========== */
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len)
{
    CanTxMsg tx_msg;
    uint8_t  mailbox;
    uint32_t timeout;

    if (len > 8) len = 8;

    tx_msg.ExtId = 0;
    tx_msg.IDE   = CAN_Id_Standard;
    tx_msg.RTR   = CAN_RTR_Data;
    tx_msg.StdId = id;
    tx_msg.DLC   = len;
    memcpy(tx_msg.Data, data, len);

    timeout = 2000;
    mailbox = CAN_Transmit(CAN1, &tx_msg);
    while (CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok && timeout--)
        Delay_us(1);

    return (timeout == 0) ? 1 : 0;
}

/* ========== 校验和计算 ========== */
static uint8_t CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++) chk ^= data[i];
    return chk;
}

/* ========== 封装并发送 CAN 帧 ========== */
static void SendCANData(uint32_t id, uint8_t func,
                        uint8_t *payload, uint8_t payload_len)
{
    uint8_t data[8];
    uint8_t i;

    data[CAN_DATA_TYPE_IDX] = (id >> 8) & 0x0F;   // 从 ID 取优先级
    data[CAN_DATA_SRC_IDX]  = SLAVE_NODE_ID;
    data[CAN_DATA_FUNC_IDX] = func;

    /* 载荷 */
    for (i = 0; i < payload_len && i < 4; i++)
        data[CAN_DATA_PAYLOAD_IDX + i] = payload[i];

    /* 填充剩余载荷字节 */
    for (; i < 4; i++)
        data[CAN_DATA_PAYLOAD_IDX + i] = 0;

    /* 校验和 (Byte0~Byte6) */
    data[CAN_DATA_CHKSUM_IDX] = CalcChecksum(data, 7);

    CAN_SendFrame(id, data, 8);
}

/* ========== 发送心跳 ========== */
void CAN_SendHeartBeat(void)
{
    uint8_t payload[4] = {SLAVE_NODE_ID, 0, 0, 0};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_HEARTBEAT, payload, 4);
}

/* ========== 发送温湿度 ========== */
void CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                      uint8_t humi_int, uint8_t humi_dec)
{
    uint8_t payload[4] = {temp_int, temp_dec, humi_int, humi_dec};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_TEMP_HUMI, payload, 4);
}

/* ========== 发送紧急报警 ========== */
void CAN_SendAlarm(void)
{
    uint8_t payload[4] = {SLAVE_NODE_ID, 0x01, 0x00, 0x00};  // 故障码 0x01
    SendCANData(CAN_ID_EMERGENCY_BASE, CAN_FUNC_ALARM, payload, 4);
}

/* ========== 发送故障恢复 ========== */
void CAN_SendRecover(void)
{
    uint8_t payload[4] = {SLAVE_NODE_ID, 0x00, 0x00, 0x00};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_RECOVER, payload, 4);
}

/* ========== CAN RX0 中断 ========== */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CAN_Receive(CAN1, CAN_FIFO0, &g_rx_msg);
        g_can_rx_flag = 1;
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/* ========== CAN TX 中断 (发送完成) ========== */
void USB_HP_CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET) {
        g_can_tx_done = 1;
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
    }
}
