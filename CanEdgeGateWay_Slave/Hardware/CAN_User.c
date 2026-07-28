/**
 * CAN User Layer — Slave Phase 2
 *
 * ISR → semaphore → task pipeline.
 * Added BH1750 light sensor frame support + local cache integration.
 */

#include "CAN_User.h"
#include "delay.h"
#include <string.h>

/* ---- Globals ---- */
volatile uint8_t   g_can_rx_flag = 0;
SemaphoreHandle_t  g_can_rx_sem = NULL;
LocalCache         g_local_cache;
volatile uint32_t  g_can_tx_success_count = 0;
volatile uint32_t  g_can_tx_fail_count = 0;

/* ISR-to-task frame buffer */
static CanRxMsg      g_isr_rx_msg;

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
    /* CAN RX0 — priority 5 (≥ configMAX_SYSCALL_INTERRUPT_PRIORITY).
     * Must be ≥5 to safely call xSemaphoreGiveFromISR / portYIELD_FROM_ISR. */
    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/* ==================== Init ==================== */

void CAN_User_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;

    CAN_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    can.CAN_TTCM       = DISABLE;
    can.CAN_ABOM       = DISABLE; /* 匹配厂家参考: 关自动 Bus-Off 恢复 */
    can.CAN_AWUM       = DISABLE; /* 匹配厂家参考: 关自动唤醒 */
    can.CAN_NART       = DISABLE;
    can.CAN_RFLM       = DISABLE;
    can.CAN_TXFP       = DISABLE;
    can.CAN_Mode       = CAN_Mode_Normal;
    /* 500kbps: 36MHz/(6×12tq)=500kbps, SJW=2tq 容忍晶振偏差 ~1500ppm */
    can.CAN_SJW        = CAN_SJW_2tq;
    can.CAN_BS1        = CAN_BS1_5tq;
    can.CAN_BS2        = CAN_BS2_6tq;
    can.CAN_Prescaler  = 6;
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

    /* Create semaphore FIRST — before enabling CAN interrupt */
    g_can_rx_sem = xSemaphoreCreateBinary();

    /* Then enable CAN interrupt (ISR will find valid semaphore handle) */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN_NVIC_Init();

    /* Init local cache */
    LocalCache_Init(&g_local_cache);
}

/* ==================== Send Frame ==================== */

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
    while (timeout--) {
        if (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok)
            goto send_ok;
        Delay_us(1);
    }
    goto tx_fail;

send_ok:
    g_can_tx_success_count++;
    LocalCache_OnTxSuccess(&g_local_cache);
    return 0;

tx_fail:
    g_can_tx_fail_count++;
    LocalCache_OnTxFail(&g_local_cache);
    return 1;
}

/* ==================== Checksum ==================== */

static uint8_t CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++) chk ^= data[i];
    return chk;
}

/* ==================== Frame builders ==================== */

static void SendCANData(uint32_t id, uint8_t func,
                        uint8_t *payload, uint8_t payload_len)
{
    uint8_t data[8], i;

    data[CAN_DATA_TYPE_IDX] = (id >= CAN_ID_EMERGENCY_BASE && id < CAN_ID_NORMAL_BASE) ? 0 :
                               (id >= CAN_ID_LOWFREQ_BASE) ? 2 : 1;
    data[CAN_DATA_SRC_IDX]  = SLAVE_NODE_ID;
    data[CAN_DATA_FUNC_IDX] = func;

    for (i = 0; i < payload_len && i < 4; i++)
        data[CAN_DATA_PAYLOAD_IDX + i] = payload[i];
    for (; i < 4; i++)
        data[CAN_DATA_PAYLOAD_IDX + i] = 0;

    data[CAN_DATA_CHKSUM_IDX] = CalcChecksum(data, 7);


    if (CAN_SendFrame(id, data, 8) != 0) {
        /* TX failed — cache for replay */
        LocalCache_Push(&g_local_cache, id, data);
    }
}

void CAN_SendHeartBeat(void)
{
    uint8_t payload[4] = {SLAVE_NODE_ID, 0, 0, 0};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_HEARTBEAT, payload, 4);
}

void CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                      uint8_t humi_int, uint8_t humi_dec)
{
    uint8_t payload[4] = {temp_int, temp_dec, humi_int, humi_dec};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_TEMP_HUMI, payload, 4);
}

void CAN_SendLight(uint16_t lux)
{
    uint8_t payload[4];
    payload[0] = (uint8_t)(lux >> 8);
    payload[1] = (uint8_t)(lux & 0xFF);
    payload[2] = 0;
    payload[3] = 0;
    SendCANData(CAN_ID_LOWFREQ_BASE + SLAVE_NODE_ID, CAN_FUNC_LIGHT, payload, 4);
}

void CAN_SendAlarm(void)
{
    uint8_t payload[4] = {SLAVE_NODE_ID, 0x01, 0x00, 0x00};
    SendCANData(CAN_ID_EMERGENCY_BASE, CAN_FUNC_ALARM, payload, 4);
}

void CAN_SendRecover(void)
{
    uint8_t payload[4] = {SLAVE_NODE_ID, 0x00, 0x00, 0x00};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_RECOVER, payload, 4);
}

/* ==================== ISR (fast — just read + signal) ==================== */

void CAN1_RX0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CAN_Receive(CAN1, CAN_FIFO0, &g_isr_rx_msg);
        g_can_rx_flag = 1;
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);

        xSemaphoreGiveFromISR(g_can_rx_sem, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
