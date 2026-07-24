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

#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

#define SLAVE_NODE_ID             0x01

extern volatile uint8_t  g_can_tx_done;
extern volatile uint8_t  g_can_rx_flag;

void CAN_User_Init(void);
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);
void CAN_SendHeartBeat(void);
void CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                      uint8_t humi_int, uint8_t humi_dec);
void CAN_SendAlarm(void);
void CAN_SendRecover(void);

#endif
