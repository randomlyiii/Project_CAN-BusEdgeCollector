/**
 * CAN User Layer — 从站1 协议层, 裸机重构
 *
 * 核心逻辑 (初始化/帧构造/校验/发送) 与从站1 CAN_User.c 一致 —
 * 该代码在测试板上已验证主站可正常接收.
 * 仅剥离 FreeRTOS / LocalCache / sensor_manager 依赖, 加入测试工具统计.
 *
 * 位时序 (从站1 原版, 不可改): 500kbps, BS1=5tq, BS2=6tq, SJW=2tq.
 * 注意 CAN_BSx_tq 宏本身就是寄存器编码, 勿用裸数字覆盖.
 */

#include "CAN_User.h"
#include "test_config.h"
#include "delay.h"
#include <string.h>

/* ---- 统计 ---- */
volatile uint32_t g_can_tx_success_count = 0;
volatile uint32_t g_can_tx_fail_count = 0;
volatile uint16_t g_alarm_drop_cnt = 0;          /* 告警限流丢弃计数 */
volatile uint8_t  g_last_tx[8] = {0};
volatile uint32_t g_rx_count = 0;
volatile uint8_t  g_can_tec = 0;
volatile uint8_t  g_can_rec = 0;

/* ==================== GPIO Init (PA11=RX, PA12=TX) — 从站1 原版 ==================== */

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

/* ==================== Init (NO interrupts) — 从站1 原版 ==================== */

void CAN_User_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;

    CAN_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    can.CAN_TTCM       = DISABLE;
    can.CAN_ABOM       = ENABLE;   /* 兜底防护: BusOff 硬件自动恢复 */
    can.CAN_AWUM       = DISABLE;
    can.CAN_NART       = ENABLE;   /* 禁止自动重传: 单节点无 ACK 时避免 TEC 快速到 255 */
    can.CAN_RFLM       = DISABLE;
    can.CAN_TXFP       = DISABLE;
    can.CAN_Mode       = CAN_Mode_Normal;
    /* 500kbps: 36MHz/(6×12tq)=500kbps. 采样点 50%. */
    can.CAN_SJW        = CAN_SJW_2tq;    /* =1 */
    can.CAN_BS1        = CAN_BS1_5tq;    /* =4 */
    can.CAN_BS2        = CAN_BS2_6tq;    /* =5 */
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

    g_can_tx_success_count = 0;
    g_can_tx_fail_count    = 0;
}

/* ==================== Send Frame — 从站1 原版 ==================== */

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
    return 0;

tx_fail:
    g_can_tx_fail_count++;
    return 1;
}

/* ==================== Checksum (XOR) — 从站1 原版 ==================== */

static uint8_t CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++) chk ^= data[i];
    return chk;
}

/* ==================== Internal: build + send — 从站1 原版 + 快照 ==================== */

static uint8_t SendCANData(uint32_t id, uint8_t func,
                           uint8_t *payload, uint8_t payload_len)
{
    uint8_t data[8], i;

    /* Frame type byte: 0=emergency, 1=normal, 2=lowfreq */
    data[CAN_DATA_TYPE_IDX] = (id >= CAN_ID_EMERGENCY_BASE && id < CAN_ID_NORMAL_BASE) ? 0 :
                               (id >= CAN_ID_LOWFREQ_BASE) ? 2 : 1;
    data[CAN_DATA_SRC_IDX]  = slave_node_id;
    data[CAN_DATA_FUNC_IDX] = func;

    for (i = 0; i < payload_len && i < 4; i++)
        data[CAN_DATA_PAYLOAD_IDX + i] = payload[i];
    for (; i < 4; i++)
        data[CAN_DATA_PAYLOAD_IDX + i] = 0;

    data[CAN_DATA_CHKSUM_IDX] = CalcChecksum(data, 7);

    /* 快照最近帧供 OLED 实时显示 */
    for (i = 0; i < 8; i++) g_last_tx[i] = data[i];

    return CAN_SendFrame(id, data, 8);
}

/* ==================== Phase 2 frame builders — 从站1 原版 ==================== */

uint8_t CAN_SendHeartBeat(void)
{
    uint8_t payload[4] = {slave_node_id, 0, 0, 0};
    return SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_HEARTBEAT, payload, 4);
}

uint8_t CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                         uint8_t humi_int, uint8_t humi_dec)
{
    uint8_t payload[4] = {temp_int, temp_dec, humi_int, humi_dec};
    return SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_TEMP_HUMI, payload, 4);
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
    static uint32_t last_tick = 0;
    static uint8_t  consec = 0;
    uint32_t now = Delay_GetTick();
    uint32_t interval;

    if (now - last_tick > 5000) consec = 0;
    interval = (consec >= 5) ? 2000 : 1000;

    if (now - last_tick < interval) {
        if (g_alarm_drop_cnt < 65535) g_alarm_drop_cnt++;
        return;
    }
    last_tick = now;
    if (consec < 250) consec++;

    uint8_t payload[4] = {slave_node_id, 0x01, 0x00, 0x00};
    SendCANData(CAN_ID_EMERGENCY_BASE, CAN_FUNC_ALARM, payload, 4);
}

void CAN_SendRecover(void)
{
    uint8_t payload[4] = {slave_node_id, 0x00, 0x00, 0x00};
    SendCANData(CAN_ID_NORMAL_BASE, CAN_FUNC_RECOVER, payload, 4);
}

/* ==================== 压测帧发送 (main 驱动节拍) ====================
 * 走 SendCANData (与从站同源帧构造), ID 按从站映射:
 *   HB/TEMP/RECOVER → 0x200   ALARM → 0x100   LIGHT → 0x300+node
 */

uint8_t CAN_TxSendOne(void)
{
    static uint32_t seq = 0;
    uint8_t  func = TEST_FRAME_FUNC;
    uint8_t  payload[4] = {0, 0, 0, 0};
    uint32_t id;

    switch (func) {
    case CAN_FUNC_ALARM:    id = CAN_ID_EMERGENCY_BASE; break;
    case CAN_FUNC_LIGHT:
    case CAN_FUNC_LM393_DO:
    case CAN_FUNC_RESERVED: id = CAN_ID_LOWFREQ_BASE + slave_node_id; break;
    default:                id = CAN_ID_NORMAL_BASE;    break;
    }

    switch (TEST_PAYLOAD_MODE) {
    case 0:  /* 固定字节 — 4 字节全同, 用 0x55 降低位填充, 帧最短, 负载最高 */
        payload[0] = TEST_FIXED_PAYLOAD_B0;
        payload[1] = TEST_FIXED_PAYLOAD_B0;
        payload[2] = TEST_FIXED_PAYLOAD_B0;
        payload[3] = TEST_FIXED_PAYLOAD_B0;
        break;
    case 1:  /* 32位递增计数 — 主站可交叉校验帧完整性 */
        payload[0] = (uint8_t)(seq >>  0);
        payload[1] = (uint8_t)(seq >>  8);
        payload[2] = (uint8_t)(seq >> 16);
        payload[3] = (uint8_t)(seq >> 24);
        break;
    case 2:  /* 模拟温湿度: 26~33°C, 45~54% */
        payload[0] = 26 + (uint8_t)((seq / 500) % 8);
        payload[1] = (uint8_t)(seq % 10);
        payload[2] = 45 + (uint8_t)((seq / 300) % 10);
        payload[3] = (uint8_t)(seq % 10);
        break;
    case 3:  /* 功能码轮转: 心跳/温湿度 交替 */
    default:
        func = (seq & 1) ? CAN_FUNC_HEARTBEAT : CAN_FUNC_TEMP_HUMI;
        id   = CAN_ID_NORMAL_BASE;
        payload[0] = (uint8_t)(seq >> 0);
        payload[1] = (uint8_t)(seq >> 8);
        break;
    }
    seq++;

    return SendCANData(id, func, payload, 4);
}

/* ==================== RX 轮询 + 错误状态 (诊断) ==================== */

void CAN_RxPoll(void)
{
    CanRxMsg msg;

    while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0) {
        CAN_Receive(CAN1, CAN_FIFO0, &msg);
        g_rx_count++;
    }
    while (CAN_MessagePending(CAN1, CAN_FIFO1) > 0) {
        CAN_Receive(CAN1, CAN_FIFO1, &msg);
        g_rx_count++;
    }

    g_can_tec = (uint8_t)((CAN1->ESR >> 16) & 0xFF);
    g_can_rec = (uint8_t)((CAN1->ESR >> 8) & 0xFF);
}
