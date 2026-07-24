/**
 * CAN application layer — Phase 2: FreeRTOS task-driven architecture
 *
 * 3-level priority frame ID allocation:
 *   0 (emergency): 0x100~0x1FF   alarm/fault/BusOff recovery
 *   1 (normal):    0x200~0x2FF   heartbeat, temp/humidity
 *   2 (low-freq):  0x300~0x3FF   light sensor, config/query
 *
 * Frame format (8 bytes):
 *   [Byte0=priority][Byte1=srcID][Byte2=func][Byte3-6=payload][Byte7=checksum]
 */

#ifndef __CAN_USER_H
#define __CAN_USER_H

#include "stm32f10x.h"
#include <stdint.h>
#include "../FreeRTOS/inc/FreeRTOS.h"
#include "../FreeRTOS/inc/semphr.h"
#include "fifo.h"

/* ---- CAN ID bases ---- */
#define CAN_ID_EMERGENCY_BASE     0x100
#define CAN_ID_NORMAL_BASE        0x200
#define CAN_ID_LOWFREQ_BASE       0x300

/* ---- Function codes ---- */
#define CAN_FUNC_HEARTBEAT        0x01
#define CAN_FUNC_TEMP_HUMI        0x02
#define CAN_FUNC_ALARM            0x03
#define CAN_FUNC_RECOVER          0x04
#define CAN_FUNC_LIGHT            0x05   /* BH1750 light sensor */
#define CAN_FUNC_BUSOFF_RECOVERY  0xF0   /* Bus-Off recovery notification */

/* ---- Frame byte index ---- */
#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

/* ---- Node management ---- */
#define MAX_SLAVE_NODES           8
#define HEARTBEAT_TIMEOUT_MS      1500
#define NODE_BLACKLIST_TIMEOUT_MS 10000

/* ---- Slave node record ---- */
typedef struct {
    uint8_t  node_id;
    uint8_t  online;
    uint32_t last_heartbeat_tick;
    uint16_t heartbeat_count;
    uint8_t  fault_flag;
    uint8_t  blacklist;
    uint32_t blacklist_start_tick;
    uint8_t  temp_int;
    uint8_t  temp_dec;
    uint8_t  humi_int;
    uint8_t  humi_dec;
} SlaveNode_t;

/* ---- CAN error status ---- */
typedef struct {
    uint8_t  error_level;       /* 0=OK, 1=active-err, 2=passive-err, 3=BusOff */
    uint8_t  tec;
    uint8_t  rec;
    uint16_t bus_load;          /* 0~10000 = 0.00%~100.00% */
    uint8_t  throttle_level;    /* 0=normal, 1=low-freq limited, 2=emergency mode */
} CAN_ErrorStatus_t;

/* ---- Priority override per node ---- */
typedef struct {
    uint8_t  overridden;        /* 1 = priority elevated */
    uint8_t  original_prio;
    uint32_t escalate_tick;     /* when escalated */
    uint8_t  escalation_reason; /* 0=none, 1=temp-fault, 2=sensor-fail, 3=CAN-error */
} PriorityOverride_t;

/* ---- Extern globals ---- */
extern SlaveNode_t        g_slave_nodes[MAX_SLAVE_NODES];
extern CAN_ErrorStatus_t  g_can_error;
extern PriorityOverride_t g_priority_override[MAX_SLAVE_NODES];
extern volatile uint32_t  g_can_rx_int_count;
extern volatile uint32_t  g_can_rx_temp_count;
extern volatile uint16_t  g_bus_off_recovery_cnt;
extern volatile uint8_t   g_system_throttle_level;

/* ---- FreeRTOS sync objects ---- */
extern SemaphoreHandle_t  g_can_rx_sem;
extern SemaphoreHandle_t  g_can_fifo_not_empty;
extern SemaphoreHandle_t  g_can_monitor_sem;

/* ---- CAN init ---- */
void CAN_User_Init(void);
void CAN_ResetBus(void);

/* ---- Frame send ---- */
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);

/* ---- ISR handlers ---- */
void CAN1_RX0_IRQHandler(void);
void CAN1_SCE_IRQHandler(void);

/* ---- Frame processing (called from vTask_CAN_Rx) ---- */
extern CanRxFrame g_isr_rx_frame;
void CAN_ProcessFrame(CanRxFrame *frame);

/* ---- Heartbeat / error / load (called from vTask_CAN_Monitor) ---- */
void CAN_HeartBeatCheck(void);
void CAN_ErrorMonitor(void);
void CAN_CalcBusLoad(void);

/* ---- Escalation / de-escalation (called from vTask_CAN_Monitor) ---- */
void CAN_CheckEscalation(void);
void CAN_CheckDeescalation(void);
void CAN_ManualDeescalate(uint8_t node_id);

/* ---- Callbacks (weak) ---- */
void CAN_User_OnError(uint8_t level);
void CAN_User_OnAlarm(uint8_t node_id, uint8_t func);
void CAN_User_OnNodeUpdate(uint8_t node_id);

#endif /* __CAN_USER_H */
