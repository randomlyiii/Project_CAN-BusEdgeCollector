#ifndef __CAN_USER_H
#define __CAN_USER_H

#include "stm32f10x.h"
#include <stdint.h>

/* ========== CAN 协议定义 (与主站一致) ========== */
#define CAN_ID_EMERGENCY_BASE     0x100
#define CAN_ID_NORMAL_BASE        0x200
#define CAN_ID_LOWFREQ_BASE       0x300

#define CAN_FUNC_HEARTBEAT        0x01
#define CAN_FUNC_TEMP_HUMI        0x02
#define CAN_FUNC_ALARM            0x03
#define CAN_FUNC_RECOVER          0x04

#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

/* ========== 从站节点配置 ========== */
#define SLAVE_NODE_ID             0x01     // 本从站节点ID (1~254)

/* ========== 全局变量 ========== */
extern volatile uint8_t  g_can_tx_done;    // 发送完成标志
extern volatile uint8_t  g_can_rx_flag;    // 接收中断标志

/* ========== 函数声明 ========== */
void CAN_User_Init(void);
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);

/* 从站业务帧 */
void CAN_SendHeartBeat(void);
void CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                      uint8_t humi_int, uint8_t humi_dec);
void CAN_SendAlarm(void);
void CAN_SendRecover(void);

#endif
