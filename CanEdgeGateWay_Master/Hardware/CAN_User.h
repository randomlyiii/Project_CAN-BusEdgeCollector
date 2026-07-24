#ifndef __CAN_USER_H
#define __CAN_USER_H

#include "stm32f10x.h"
#include <stdint.h>

/*
 * CAN 应用层协议 — 三级优先级帧 ID 分配:
 *   紧急帧 0x100~0x1FF  (最高优先, 仲裁胜出)
 *   常规帧 0x200~0x2FF  (心跳/温湿度)
 *   低频帧 0x300~0x3FF  (配置/查询)
 *
 * 每帧 8 字节: [类型][源ID][功能码][载荷4B][校验和]
 *   校验和 = payload[0..3] 异或, 暂时未启用
 */
#define CAN_ID_EMERGENCY_BASE     0x100   /* 紧急: 报警/故障上报 */
#define CAN_ID_NORMAL_BASE        0x200   /* 常规: 心跳, 温湿度 */
#define CAN_ID_LOWFREQ_BASE       0x300   /* 低频: 预留扩展 */

#define CAN_FUNC_HEARTBEAT        0x01   /* 心跳 (500ms) */
#define CAN_FUNC_TEMP_HUMI        0x02   /* 温湿度上报 (2s) */
#define CAN_FUNC_ALARM            0x03   /* 故障报警 */
#define CAN_FUNC_RECOVER          0x04   /* 故障恢复 */

/* Byte[0]~Byte[7] 帧格式 */
#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

#define MAX_SLAVE_NODES           8
#define HEARTBEAT_TIMEOUT_MS      1500
#define NODE_BLACKLIST_TIMEOUT_MS 10000

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

typedef struct {
    uint8_t  error_level;       // 0正常 1主动错误 2被动错误 3总线关闭
    uint8_t  tec;
    uint8_t  rec;
    uint16_t bus_load;          // 0~10000 (0.00%~100.00%)
} CAN_ErrorStatus_t;

extern SlaveNode_t      g_slave_nodes[MAX_SLAVE_NODES];
extern CAN_ErrorStatus_t g_can_error;
extern volatile uint8_t  g_can_rx_flag;
extern uint32_t          g_can_rx_int_count;
extern uint32_t          g_can_rx_temp_count;   /* TEMP_HUMI帧计数 */

void CAN_User_Init(void);
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);
void CAN_ProcessRxFrame(void);
void CAN_HeartBeatCheck(void);
void CAN_ErrorMonitor(void);
void CAN_CalcBusLoad(void);
void CAN_ResetBus(void);

#endif
