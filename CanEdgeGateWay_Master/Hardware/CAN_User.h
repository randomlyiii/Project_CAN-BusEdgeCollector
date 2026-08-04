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

/* ================================================================
 * RX Ring Buffer — ISR→Task 帧传递
 *
 * 单生产者(ISR) + 单消费者(Task)，无需互斥锁。
 * 深度必须为 2 的幂，uint8_t 索引保证原子操作。
 * 满判据: (head+1) & MASK == tail  → 留空一格区分空/满
 *
 * RAM 约束 (STM32F103C8T6 20KB): 64 深 CanRxMsg(20B) 会 L6406E
 * 溢出。紧凑帧 RingFrame_t=12B (去 ExtId/IDE/RTR/FMI，协议只用
 * 标准数据帧 StdId≤0x3FF)，ring_high 64 + ring_norm 4 = 816B。
 * ================================================================ */

/* 紧凑接收帧 — sizeof = 12 (对齐 2) */
typedef struct {
    uint16_t StdId;
    uint8_t  DLC;
    uint8_t  Data[8];
} RingFrame_t;

/* 宏生成指定深度的 SPSC ring + 内联 push/pop。
 * ISR 侧只调 push (无锁，不调 FreeRTOS API)，
 * Task 侧只调 pop。uint8_t head/tail 在 32 位 MCU 上原子。 */
#define RX_RING_DEFINE(NAME, DEPTH)                                   \
    typedef struct {                                                  \
        RingFrame_t      frames[DEPTH];                               \
        volatile uint8_t head;    /* ISR 写入 (生产者) */              \
        volatile uint8_t tail;    /* Task 读取 (消费者) */             \
        volatile uint16_t overflow_cnt;  /* 满=丢帧计数 */             \
    } NAME;                                                           \
    static inline uint8_t NAME##_push(NAME *rb, RingFrame_t *f)       \
    {                                                                 \
        uint8_t next = (uint8_t)((rb->head + 1) & (DEPTH - 1));       \
        if (next == rb->tail) { rb->overflow_cnt++; return 1; }       \
        rb->frames[rb->head] = *f;                                    \
        rb->head = next;                                              \
        return 0;                                                     \
    }                                                                 \
    static inline uint8_t NAME##_pop(NAME *rb, RingFrame_t *f)        \
    {                                                                 \
        if (rb->tail == rb->head) return 1;                           \
        *f = rb->frames[rb->tail];                                    \
        rb->tail = (uint8_t)((rb->tail + 1) & (DEPTH - 1));           \
        return 0;                                                     \
    }

/* 主路径: FIFO0 帧流 (全通滤波 → 所有帧走这里), 64 深 */
RX_RING_DEFINE(RxRingHigh, 64)
/* Fallback: FIFO1 帧流 (正常无帧), 4 深 */
RX_RING_DEFINE(RxRingNorm, 4)

/* ---- CAN ID bases ---- */
#define CAN_ID_EMERGENCY_BASE     0x100
#define CAN_ID_NORMAL_BASE        0x200
#define CAN_ID_LOWFREQ_BASE       0x300

/* ---- Function codes ---- */
#define CAN_FUNC_HEARTBEAT        0x01
#define CAN_FUNC_TEMP_HUMI        0x02
#define CAN_FUNC_ALARM            0x03
#define CAN_FUNC_RECOVER          0x04
#define CAN_FUNC_LIGHT            0x05   /* BH1750 light sensor / LM393 AO */
#define CAN_FUNC_LM393_DO         0x06   /* LM393 digital output (new) */
#define CAN_FUNC_RESERVED         0x07   /* Reserved channel (new) */
#define CAN_FUNC_BUSOFF_RECOVERY  0xF0   /* Bus-Off recovery notification */

/* ---- Frame byte index ---- */
#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

/* ---- Node management ---- */
#define MAX_SLAVE_NODES           8
#define STALE_TIMEOUT_MS          3000U   /* 3× 心跳周期, 数据过期 */
#define OFFLINE_CONFIRM_MS        30000U  /* 持续 EXP 30s 判永久离线 */

/* ---- Slave node record ---- */
typedef struct {
    uint8_t  node_id;
    uint8_t  online;              /* 1=收到过帧 (CAN_ProcessFrame 置, 永远不清 0) */
    uint8_t  stale;               /* 1=>3s 未收帧, 短期过期标记 */
    uint8_t  offline;             /* 1=持续 EXP 超过 30s, 长期确认离线 */
    uint32_t last_heartbeat_tick;
    uint32_t expire_start_tick;   /* 进入 EXP 瞬间打时间戳, 避免持续差值计算 */
    uint16_t heartbeat_count;
    uint8_t  fault_flag;
    /* 无 blacklist / hb_lost / 防抖计数器 — 极简设计 */
    uint8_t  temp_int;
    uint8_t  temp_dec;
    uint8_t  humi_int;
    uint8_t  humi_dec;
    uint16_t light_lux;         /* BH1750 lux / LM393 AO analog (CAN_FUNC_LIGHT) */
    uint16_t lm393_analog;      /* LM393 AO raw ADC 0~4095 (same payload as light_lux) */
    uint8_t  lm393_digital;     /* LM393 DO 0=Bright / 1=Dark */
    uint16_t reserved_ch1;      /* Reserved channel value */
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

/* ---- RX Ring Buffers (ISR→Task, 锁无关 SPSC, 各 ISR 写各自的) ---- */
extern RxRingHigh         g_rx_ring_high;
extern RxRingNorm         g_rx_ring_norm;

/* DBG: 调试步进计数 */
extern volatile uint8_t  g_dbg_step;

/* FIFO0 帧由 RX0 ISR 搬入 ring_high (FMP0 中断使能) */

/* ---- CAN init ---- */
void CAN_User_Init(void);
void CAN_EnableInterrupts(void);
void CAN_ResetBus(void);

/* ---- Frame send ---- */
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);

/* ---- ISR handlers ---- */
void USB_LP_CAN1_RX0_IRQHandler(void);  /* 注意: F103 md 启动文件向量名是 USB_LP_CAN1_RX0 */
void CAN1_RX1_IRQHandler(void);
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
