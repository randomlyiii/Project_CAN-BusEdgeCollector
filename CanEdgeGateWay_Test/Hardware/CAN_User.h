/**
 * CAN User Layer — 从站1 协议层 (裸机重构)
 *
 * 帧格式与从站1 完全一致, 主站已验证可接收:
 *   [0]=type(0紧急/1正常/2低频) [1]=src [2]=func [3-6]=payload [7]=XOR校验
 *
 * 位时序为从站1 原版: 500kbps, BS1=5tq, BS2=6tq, SJW=2tq.
 */

#ifndef __CAN_USER_H
#define __CAN_USER_H

#include "stm32f10x.h"
#include <stdint.h>

/* ---- CAN ID 基址 (aligned with README.md frame ID design) ---- */
#define CAN_ID_EMERGENCY_BASE     0x100   /* 0级: alarm/fault */
#define CAN_ID_NORMAL_BASE        0x200   /* 1级: normal-periodic */
#define CAN_ID_LOWFREQ_BASE       0x300   /* 2级: low-frequency */

/* ---- Function codes ---- */
#define CAN_FUNC_HEARTBEAT        0x01
#define CAN_FUNC_TEMP_HUMI        0x02
#define CAN_FUNC_ALARM            0x03
#define CAN_FUNC_RECOVER          0x04
#define CAN_FUNC_LIGHT            0x05   /* BH1750 / LM393 AO (光照) */
#define CAN_FUNC_LM393_DO         0x06   /* LM393 数字量 */
#define CAN_FUNC_RESERVED         0x07   /* 预留通道 */

/* ---- Frame byte index ---- */
#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

/* slave_node_id 由 test_config.h 提供 (= TEST_NODE_ID_BASE) */

/* ---- Init / Send ---- */
void     CAN_User_Init(void);
uint8_t  CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);

/* ---- 从站帧 builders (与从站 CAN_User.c 一致, 返回发送结果供计数) ---- */
uint8_t  CAN_SendHeartBeat(void);
uint8_t  CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                          uint8_t humi_int, uint8_t humi_dec);
void     CAN_SendLight(uint16_t lux);
void     CAN_SendAlarm(void);
void     CAN_SendRecover(void);

/* ---- 压测帧发送 (main 循环调用, 内部自增序号) ---- */
uint8_t  CAN_TxSendOne(void);

/* ---- 诊断: RX 轮询 + 错误状态 ---- */
void     CAN_RxPoll(void);

/* ---- 统计 ---- */
extern volatile uint32_t g_can_tx_success_count;
extern volatile uint32_t g_can_tx_fail_count;
extern volatile uint8_t  g_last_tx[8];      /* 最近一帧快照 (OLED 显示) */
extern volatile uint32_t g_rx_count;        /* 测试站自收帧数 */
extern volatile uint8_t  g_can_tec;         /* 发送错误计数 */
extern volatile uint8_t  g_can_rec;         /* 接收错误计数 */

#endif /* __CAN_USER_H */
