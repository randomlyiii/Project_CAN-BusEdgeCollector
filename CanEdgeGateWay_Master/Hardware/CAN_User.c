/**
 * CAN User Layer — Phase 2 (FreeRTOS task-driven)
 *
 * ISR responsibility: read frame + give semaphore (sub-50us)
 * vTask_CAN_Rx:   consume semaphore, classify frame, push to FIFO
 * vTask_CAN_Monitor: bus load, error state machine, escalation/de-escalation
 * vTask_CAN_Tx:   pop from FIFO, transmit with priority ordering
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

/* ---- FreeRTOS sync objects ---- */
SemaphoreHandle_t   g_can_rx_sem          = NULL;
SemaphoreHandle_t   g_can_fifo_not_empty  = NULL;
SemaphoreHandle_t   g_can_monitor_sem     = NULL;

/* ---- ISR → task shared frame buffer ---- */
CanRxFrame          g_isr_rx_frame;
static volatile BaseType_t g_rx_higher_prio_woken = pdFALSE;

/* ---- Load calculation ---- */
static uint32_t g_tx_bit_count   = 0;
static uint32_t g_rx_bit_count   = 0;
static uint32_t g_window_start   = 0;
static uint32_t g_throttle_stable_cnt = 0;  /* hysteresis counter */
#define LOAD_WINDOW_MS          100
#define THROTTLE_HYST_CNT       5     /* 500ms stable before recovery */
#define LOAD_THRESHOLD_LOW      7000  /* 70.00% */
#define LOAD_THRESHOLD_HIGH     9000  /* 90.00% */
#define LOAD_RECOVER_MID        8500  /* <85% exit emergency */
#define LOAD_RECOVER_LOW        6500  /* <65% exit low-freq throttle */

/* ---- CAN bit timing: 500kbps standard frame ≈ 108 bits (worst case with stuffing) ---- */
#define CAN_FRAME_BITS          108

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

    /* CAN RX0 interrupt — highest HW priority */
    nvic.NVIC_IRQChannel                   = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* CAN SCE (error) interrupt */
    nvic.NVIC_IRQChannel                   = CAN1_SCE_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

/* ==================== CAN Init ==================== */

void CAN_User_Init(void)
{
    CAN_InitTypeDef       can;
    CAN_FilterInitTypeDef filter;
    uint8_t i;

    memset(g_slave_nodes, 0, sizeof(g_slave_nodes));
    memset(g_priority_override, 0, sizeof(g_priority_override));
    g_can_error.error_level   = 0;
    g_can_error.tec           = 0;
    g_can_error.rec           = 0;
    g_can_error.bus_load      = 0;
    g_can_error.throttle_level = 0;
    g_system_throttle_level   = 0;
    g_bus_off_recovery_cnt    = 0;

    CAN_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    can.CAN_TTCM       = DISABLE;
    can.CAN_ABOM       = ENABLE;  /* Auto Bus-Off recovery after 128×11 recessive bits */
    can.CAN_AWUM       = ENABLE;
    can.CAN_NART       = DISABLE;
    can.CAN_RFLM       = DISABLE;
    can.CAN_TXFP       = DISABLE;
    can.CAN_Mode       = CAN_Mode_Normal;
    can.CAN_SJW        = CAN_SJW_1tq;
    can.CAN_BS1        = CAN_BS1_5tq;
    can.CAN_BS2        = CAN_BS2_6tq;
    can.CAN_Prescaler  = 5;    /* 500kbps @ 36MHz APB1 */
    CAN_Init(CAN1, &can);

    /* Accept ALL frames */
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

    /* Create FreeRTOS sync objects FIRST — before enabling ANY CAN interrupt.
     * If an ISR fires before its semaphore is created, xSemaphoreGiveFromISR
     * receives NULL → configASSERT → infinite loop.  (freertos_standard.md §13) */
    g_can_rx_sem         = xSemaphoreCreateBinary();
    g_can_fifo_not_empty = xSemaphoreCreateBinary();
    g_can_monitor_sem    = xSemaphoreCreateBinary();

    /* FIFO init (creates mutexes — must also be done before interrupts) */
    FIFO_Init();

    /* THEN enable CAN interrupts (ISRs will find valid semaphore handles) */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_ERR, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_BOF, ENABLE);
    CAN_NVIC_Init();

    /* Init slave node records */
    for (i = 0; i < MAX_SLAVE_NODES; i++) {
        g_slave_nodes[i].node_id = i + 1;
        g_slave_nodes[i].online  = 0;
    }

    g_window_start = Delay_GetTick();
}

/* ==================== CAN Reset (enhanced Bus-Off recovery) ==================== */

void CAN_ResetBus(void)
{
    CAN_DeInit(CAN1);
    Delay_ms(10);
    CAN_User_Init();

    g_bus_off_recovery_cnt++;

    /* Send emergency Bus-Off recovery frame */
    uint8_t data[8] = {0};
    data[CAN_DATA_TYPE_IDX]  = 0;  /* priority 0 */
    data[CAN_DATA_SRC_IDX]   = 0xFE;  /* Master node ID */
    data[CAN_DATA_FUNC_IDX]  = CAN_FUNC_BUSOFF_RECOVERY;
    CAN_SendFrame(CAN_ID_EMERGENCY_BASE, data, 8);
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

    timeout = 1000;
    mailbox = CAN_Transmit(CAN1, &tx_msg);
    while (CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok && timeout--)
        Delay_us(1);
    if (timeout == 0) return 1;
    g_tx_bit_count += CAN_FRAME_BITS;
    return 0;
}

/* ==================== ISR: CAN RX (ultra-fast — just read + signal) ==================== */

void CAN1_RX0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CanRxMsg msg;
        CAN_Receive(CAN1, CAN_FIFO0, &msg);

        /* Copy to static buffer for task consumption */
        g_isr_rx_frame.StdId = msg.StdId;
        g_isr_rx_frame.IDE   = msg.IDE;
        g_isr_rx_frame.RTR   = msg.RTR;
        g_isr_rx_frame.DLC   = msg.DLC;
        g_isr_rx_frame.FMI   = msg.FMI;
        memcpy(g_isr_rx_frame.Data, msg.Data, 8);

        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);

        /* Signal the CAN Rx task */
        xSemaphoreGiveFromISR(g_can_rx_sem, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ==================== ISR: CAN Error ==================== */

void CAN1_SCE_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t tec, rec;

    if (CAN_GetITStatus(CAN1, CAN_IT_ERR) != RESET) {
        tec = (uint8_t)((CAN1->ESR >> 16) & 0xFF);
        rec = (uint8_t)((CAN1->ESR >> 24) & 0xFF);
        g_can_error.tec = tec;
        g_can_error.rec = rec;

        if (CAN1->ESR & ((uint32_t)0x00000004))  /* BOFF bit */
            g_can_error.error_level = 3;
        else if (tec > 127 || rec > 127)
            g_can_error.error_level = 2;
        else if (tec > 96 || rec > 96)
            g_can_error.error_level = 1;
        else
            g_can_error.error_level = 0;

        CAN_User_OnError(g_can_error.error_level);
        CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);

        /* Signal monitor task */
        xSemaphoreGiveFromISR(g_can_monitor_sem, &xHigherPriorityTaskWoken);
    }

    if (CAN_GetITStatus(CAN1, CAN_IT_BOF) != RESET) {
        CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
        g_can_error.error_level = 3;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ==================== Process one frame (called from vTask_CAN_Rx) ==================== */

void CAN_ProcessFrame(CanRxFrame *p)
{
    uint8_t  src_id = p->Data[CAN_DATA_SRC_IDX];
    uint8_t  func   = p->Data[CAN_DATA_FUNC_IDX];
    uint8_t  prio   = p->Data[CAN_DATA_TYPE_IDX];
    uint32_t now    = Delay_GetTick();
    uint8_t  i, found;

    g_can_rx_int_count++;
    g_rx_bit_count += CAN_FRAME_BITS;

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

    /* Check priority override */
    if (g_priority_override[i].overridden && prio != 0) {
        prio = 0;
        p->Data[CAN_DATA_TYPE_IDX] = 0;
    }

    /* Blacklist check: block temp/humi from blacklisted nodes */
    if (g_slave_nodes[i].blacklist
        && func != CAN_FUNC_RECOVER
        && func != CAN_FUNC_HEARTBEAT
        && func != CAN_FUNC_ALARM)
        return;

    /* Heartbeat/alarm from blacklisted node → auto unblock */
    if (g_slave_nodes[i].blacklist
        && (func == CAN_FUNC_HEARTBEAT || func == CAN_FUNC_ALARM))
        g_slave_nodes[i].blacklist = 0;

    /* Process by function code */
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
        CAN_User_OnAlarm(src_id, func);
        break;

    case CAN_FUNC_RECOVER:
        g_slave_nodes[i].fault_flag = 0;
        g_slave_nodes[i].blacklist = 0;
        g_slave_nodes[i].online = 1;
        g_slave_nodes[i].last_heartbeat_tick = now;
        break;
    }

    /* Push to dual FIFO based on priority */
    if (prio == 0) {
        FIFO_High_Push(p);
    } else {
        FIFO_Normal_Push(p);
    }

    /* Wake CAN Tx task — NOTE: CAN_ProcessFrame runs in TASK context (Task_CAN_Rx).
     * Use xSemaphoreGive(), NOT xSemaphoreGiveFromISR().
     * FromISR writes to scheduler event lists without proper locking in task context,
     * corrupting the scheduler state and causing system hang.  (freertos_standard.md §15) */
    xSemaphoreGive(g_can_fifo_not_empty);
}

/* ==================== Heartbeat check (called from vTask_CAN_Monitor) ==================== */

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
        if (!g_slave_nodes[i].online && !g_slave_nodes[i].blacklist
            && g_slave_nodes[i].node_id != 0) {
            g_slave_nodes[i].blacklist = 1;
            g_slave_nodes[i].blacklist_start_tick = now;
        }
        if (g_slave_nodes[i].blacklist) {
            if (now - g_slave_nodes[i].blacklist_start_tick > NODE_BLACKLIST_TIMEOUT_MS)
                g_slave_nodes[i].blacklist = 0;
        }
    }
}

/* ==================== Error monitor + Bus-Off recovery ==================== */

void CAN_ErrorMonitor(void)
{
    uint8_t i;

    if (g_can_error.error_level == 3) {
        /* Bus-Off: stop Tx task, clear FIFOs, reset CAN, send recovery frame */
        CAN_ResetBus();

        /* Elevate ALL frames to priority 0 */
        for (i = 0; i < MAX_SLAVE_NODES; i++) {
            if (g_slave_nodes[i].node_id != 0 && !g_priority_override[i].overridden) {
                g_priority_override[i].overridden       = 1;
                g_priority_override[i].original_prio    = 1;
                g_priority_override[i].escalate_tick    = Delay_GetTick();
                g_priority_override[i].escalation_reason = 3;
            }
        }
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
