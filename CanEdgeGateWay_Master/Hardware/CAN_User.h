#ifndef __CAN_USER_H
#define __CAN_USER_H

#include "stm32f10x.h"
#include <stdint.h>

/* ------------------------- CAN 协议定义 ------------------------- */

/* 帧ID分配（阶段一） */
#define CAN_ID_EMERGENCY_BASE     0x100   // 0级紧急任务帧基址
#define CAN_ID_NORMAL_BASE        0x200   // 1级常态任务帧基址
#define CAN_ID_LOWFREQ_BASE       0x300   // 2级低频任务帧基址

/* 功能码 */
#define CAN_FUNC_HEARTBEAT        0x01    // 心跳
#define CAN_FUNC_TEMP_HUMI        0x02    // 温湿度数据
#define CAN_FUNC_ALARM            0x03    // 故障上报
#define CAN_FUNC_RECOVER          0x04    // 故障恢复

/* 帧数据格式：Byte[0]~Byte[7] */
#define CAN_DATA_TYPE_IDX         0       // 报文类型(0紧急/1常态/2低频)
#define CAN_DATA_SRC_IDX          1       // 源节点ID
#define CAN_DATA_FUNC_IDX         2       // 功能码
#define CAN_DATA_PAYLOAD_IDX      3       // 数据载荷起始(4字节: Byte3~Byte6)
#define CAN_DATA_CHKSUM_IDX       7       // 校验和(Byte0~Byte6异或)

/* ------------------------- CAN 回调事件 ------------------------- */
#define CAN_EVT_HEARTBEAT         0x01
#define CAN_EVT_TEMP_HUMI         0x02
#define CAN_EVT_ALARM             0x03
#define CAN_EVT_RECOVER           0x04
#define CAN_EVT_UNKNOWN           0xFF

/* ------------------------- 从站管理 ------------------------- */
#define MAX_SLAVE_NODES           8       // 最大管理从站数
#define HEARTBEAT_TIMEOUT_MS      1500    // 心跳超时(3 × 500ms)
#define NODE_BLACKLIST_TIMEOUT_MS 10000   // 黑名单屏蔽时间

/* 从站节点状态 */
typedef struct {
    uint8_t  node_id;                      // 节点ID
    uint8_t  online;                       // 在线标志 0=离线 1=在线
    uint32_t last_heartbeat_tick;          // 最后一次心跳tick
    uint16_t heartbeat_count;              // 心跳计数
    uint8_t  fault_flag;                   // 故障标志 0=正常 1=故障
    uint8_t  blacklist;                    // 黑名单标志 0=正常 1=屏蔽
    uint32_t blacklist_start_tick;         // 黑名单开始时间
    uint8_t  temp_int;                     // 温度整数
    uint8_t  temp_dec;                     // 温度小数
    uint8_t  humi_int;                     // 湿度整数
    uint8_t  humi_dec;                     // 湿度小数
} SlaveNode_t;

/* ------------------------- CAN 错误状态 ------------------------- */
typedef struct {
    uint8_t  error_level;                  // 0=正常 1=主动错误 2=被动错误 3=总线关闭
    uint8_t  tec;                          // 发送错误计数
    uint8_t  rec;                          // 接收错误计数
    uint16_t bus_load;                     // 总线负载率(0~1000 表示0.0%~100.0%)
} CAN_ErrorStatus_t;

/* ------------------------- 全局变量 ------------------------- */
extern SlaveNode_t      g_slave_nodes[MAX_SLAVE_NODES];
extern CAN_ErrorStatus_t g_can_error;
extern volatile uint8_t  g_can_rx_flag;    // CAN接收中断标志

/* ------------------------- 函数声明 ------------------------- */
void CAN_User_Init(void);                              // CAN外设初始化(500kbps)
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);  // 发送CAN帧
void CAN_ProcessRxFrame(void);                         // 处理接收到的CAN帧

void CAN_HeartBeatCheck(void);                         // 心跳超时检测
void CAN_ErrorMonitor(void);                           // CAN错误状态监视
void CAN_CalcBusLoad(void);                            // 总线负载率计算
void CAN_ResetBus(void);                               // 总线关闭恢复

#endif
