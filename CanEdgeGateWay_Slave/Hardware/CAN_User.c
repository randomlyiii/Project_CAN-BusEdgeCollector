/**
 * CAN User Layer — Slave Final Phase (INTERRUPT-FREE)
 *
 * Slave is TX-primary but also drains RX FIFO to prevent overflow.
 * ZERO CAN interrupts = immune to bus traffic disrupting FreeRTOS scheduler.
 *
 * TX: CAN_Transmit() with polling timeout (no ISR)
 * RX: CAN_MessagePending() polling in CAN task loop (no ISR)
 */

#include "CAN_User.h"
#include "delay.h"
#include <string.h>

/* ---- Globals ---- */
volatile uint8_t   g_can_rx_flag = 0;
LocalCache         g_local_cache;
volatile uint32_t  g_can_tx_success_count = 0;
volatile uint32_t  g_can_tx_fail_count = 0;

/* ==================== GPIO Init (PA11=RX, PA12=TX) ==================== */

static void CAN_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_11;        /* CAN RX */
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_12;        /* CAN TX */
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

/* ==================== Init (NO interrupts) ==================== */

void CAN_User_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;

    CAN_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    can.CAN_TTCM       = DISABLE;
    can.CAN_ABOM       = ENABLE;   /* 兜底防护: 极端持续干扰导致 BusOff 时硬件自动恢复 */
    can.CAN_AWUM       = DISABLE;
    can.CAN_NART       = ENABLE;   /* 禁止硬件自动重传: 单节点掉线时, 避免其他从站因
                                      ACK 缺失而快速 TEC=255 → BusOff, 连坐全网.
                                      权衡: 单次丢帧需等 500ms 下一周期重发.
                                      主站心跳超时 1500ms(3×周期), 裕量充足. */
    can.CAN_RFLM       = DISABLE;
    can.CAN_TXFP       = DISABLE;
    can.CAN_Mode       = CAN_Mode_Normal;
    /* 500kbps: 36MHz/(6×12tq)=500kbps */
    can.CAN_SJW        = CAN_SJW_2tq;
    can.CAN_BS1        = CAN_BS1_5tq;
    can.CAN_BS2        = CAN_BS2_6tq;
    can.CAN_Prescaler  = 6;
    CAN_Init(CAN1, &can);

    /* Accept ALL frames (pass-through filter) */
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

    /* NOTE: NO CAN interrupts enabled. NO NVIC init. NO semaphore.
       Slave drains FIFO0 by polling CAN_MessagePending() in task loop.
       This is immune to CAN bus traffic disrupting the scheduler. */

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

/* ==================== Internal: build + send CAN frame ==================== */

static void SendCANData(uint32_t id, uint8_t func,
                        uint8_t *payload, uint8_t payload_len)
{
    uint8_t data[8], i;

    /* Frame type byte: 0=emergency, 1=normal, 2=lowfreq */
    data[CAN_DATA_TYPE_IDX] = (id >= CAN_ID_EMERGENCY_BASE && id < CAN_ID_NORMAL_BASE) ? 0 :
                               (id >= CAN_ID_LOWFREQ_BASE) ? 2 : 1;
    data[CAN_DATA_SRC_IDX]  = slave_node_id;   /* Config-driven node ID */
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

/* ==================== Phase 2 frame builders (backward compat) ==================== */

void CAN_SendHeartBeat(void)
{
    uint8_t payload[4] = {slave_node_id, 0, 0, 0};
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
    SendCANData(CAN_ID_LOWFREQ_BASE + slave_node_id, CAN_FUNC_LIGHT, payload, 4);
}

void CAN_SendAlarm(void)
{
    uint8_t payload[4] = {slave_node_id, 0x01, 0x00, 0x00};
    SendCANData(CAN_ID_EMERGENCY_BASE, CAN_FUNC_ALARM, payload, 4);
}

void CAN_SendRecover(void)
{
    uint8_t payload[4] = {slave_node_id, 0x00, 0x00, 0x00};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_RECOVER, payload, 4);
}

/* ================================================================
 * Final Phase — Generic Sensor Data Sender
 *
 * Routes sensor data to the correct CAN ID / function code based on
 * sensor type_id. New sensors only need:
 *   1. A SENSOR_TYPE_* entry in sensor_type.h
 *   2. A CAN_FUNC_* entry if new function code needed
 *   3. A case branch in SensorTypeToCanId() + SensorTypeToFunc()
 *   4. A payload packing case below
 * ================================================================ */

/* CAN ID base → priority-level routing */
static uint32_t SensorTypeToCanId(uint8_t type_id)
{
    switch (type_id) {
    case SENSOR_TYPE_DHT11:
        return CAN_ID_NORMAL_BASE + slave_node_id + slave_can_id_offset;
    case SENSOR_TYPE_BH1750:
    case SENSOR_TYPE_LM393_AO:
    case SENSOR_TYPE_LM393_DO:
    case SENSOR_TYPE_RESERVED:
        return CAN_ID_LOWFREQ_BASE + slave_node_id + slave_can_id_offset;
    default:
        return CAN_ID_NORMAL_BASE + slave_node_id + slave_can_id_offset;
    }
}

/* Sensor type → CAN function code mapping */
static uint8_t SensorTypeToFunc(uint8_t type_id)
{
    switch (type_id) {
    case SENSOR_TYPE_DHT11:     return CAN_FUNC_TEMP_HUMI;
    case SENSOR_TYPE_BH1750:    return CAN_FUNC_LIGHT;
    case SENSOR_TYPE_LM393_AO:  return CAN_FUNC_LIGHT;
    case SENSOR_TYPE_LM393_DO:  return CAN_FUNC_LM393_DO;
    case SENSOR_TYPE_RESERVED:  return CAN_FUNC_RESERVED;
    default:                    return 0xFF;
    }
}

/**
 * Generic sensor data frame builder.
 *
 * @param sensor_type_id  SENSOR_TYPE_* from sensor_type.h
 * @param data            Pointer to sensor's last_data union
 *
 * Payload format (4 bytes, per sensor type):
 *   DHT11:           [temp_int, temp_dec, humi_int, humi_dec]
 *   BH1750:          [lux_hi, lux_lo, 0, 0]
 *   LM393_AO:        [analog_hi, analog_lo, digital, 0]
 *   RESERVED:        [ch1_hi, ch1_lo, 0, 0]
 */
void CAN_SendSensorData(uint8_t sensor_type_id, sensor_data_t *data)
{
    uint32_t id   = SensorTypeToCanId(sensor_type_id);
    uint8_t  func = SensorTypeToFunc(sensor_type_id);
    uint8_t  payload_len = 4;
    uint8_t  payload[4] = {0, 0, 0, 0};

    switch (sensor_type_id) {
    case SENSOR_TYPE_DHT11:
        payload[0] = data->dht11.temp_int;
        payload[1] = data->dht11.temp_dec;
        payload[2] = data->dht11.humi_int;
        payload[3] = data->dht11.humi_dec;
        break;
    case SENSOR_TYPE_BH1750:
        payload[0] = (uint8_t)(data->bh1750.lux >> 8);
        payload[1] = (uint8_t)(data->bh1750.lux & 0xFF);
        break;
    case SENSOR_TYPE_LM393_AO:
        payload[0] = (uint8_t)(data->lm393.analog >> 8);
        payload[1] = (uint8_t)(data->lm393.analog & 0xFF);
        payload[2] = data->lm393.digital;
        break;
    case SENSOR_TYPE_LM393_DO:
        payload[0] = data->lm393.digital;
        break;
    case SENSOR_TYPE_RESERVED:
        payload[0] = (uint8_t)(data->reserved.ch1 >> 8);
        payload[1] = (uint8_t)(data->reserved.ch1 & 0xFF);
        break;
    default:
        break;
    }

    SendCANData(id, func, payload, payload_len);
}

/* ==================== ISR (DISABLED — slave uses polling) ==================== */
#if 0
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
#endif
