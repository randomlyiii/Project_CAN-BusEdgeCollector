#ifndef __MODBUSTCP_H
#define __MODBUSTCP_H

#include "stm32f10x.h"
#include <stdint.h>

/* ======================== Modbus TCP 寄存器映射 ======================== */
/* 保持寄存器 (Holding Register, 起始地址 0x0000) — 阶段一 */
/* 偏移量均为字偏移 */

/* 从站数据区 (0x0000 ~ 0x000F) */
#define REG_SLAVE1_TEMP          0x0000   // 从站1 温度(×10)     R
#define REG_SLAVE1_HUMI          0x0001   // 从站1 湿度(×10)     R
#define REG_SLAVE1_HB_CNT        0x0002   // 从站1 心跳计数       R
#define REG_SLAVE1_FAULT         0x0003   // 从站1 故障状态       R
#define REG_SLAVE1_ONLINE        0x0004   // 从站1 在线状态       R

#define REG_SLAVE2_TEMP          0x0005   // 从站2 温度(×10)     R
#define REG_SLAVE2_HUMI          0x0006   // 从站2 湿度(×10)     R
#define REG_SLAVE2_HB_CNT        0x0007   // 从站2 心跳计数       R
#define REG_SLAVE2_FAULT         0x0008   // 从站2 故障状态       R
#define REG_SLAVE2_ONLINE        0x0009   // 从站2 在线状态       R

/* 主站状态区 (0x0010 ~ 0x001F) */
#define REG_CAN_BUS_LOAD         0x0010   // CAN 总线负载率(0~1000)  R
#define REG_CAN_TEC              0x0011   // CAN 发送错误计数       R
#define REG_CAN_REC              0x0012   // CAN 接收错误计数       R
#define REG_CAN_ERR_LEVEL        0x0013   // CAN 错误等级           R
#define REG_ONLINE_NODE_MASK     0x0014   // 在线从站位掩码         R

/* 控制区 (0x0020 ~ 0x002F) */
#define REG_CTRL_RESET           0x0020   // 写 0x55 复位总线      W
#define REG_CTRL_UNBLACKLIST     0x0021   // 写节点ID 解除黑名单    W

/* 寄存器总数 */
#define MODBUS_REG_COUNT         0x0030   // 最大 48 个寄存器

/* ======================== Modbus TCP 帧格式 ======================== */
/* MBAP 头 (7 字节) + PDU (≥2 字节) */
#define MBAP_TID_IDX             0        // 事务标识 (2B)
#define MBAP_PID_IDX             2        // 协议标识 (2B, Modbus=0x0000)
#define MBAP_LEN_IDX             4        // 后续长度 (2B)
#define MBAP_UID_IDX             6        // 单元标识 (1B)
#define MBAP_HEADER_LEN          7

#define PDU_FC_IDX               7        // 功能码 (MBAP + 0)
#define PDU_ADDR_IDX             8        // 起始地址高 (2B)
#define PDU_DATA_IDX             10       // 数据域起始

/* Modbus 功能码 (阶段一) */
#define FC_READ_HOLDING_REGS     0x03     // 读保持寄存器
#define FC_WRITE_SINGLE_REG      0x06     // 写单个寄存器
#define FC_WRITE_MULTI_REGS      0x10     // 写多个寄存器 (预留)

/* 异常码 */
#define EX_ILLEGAL_FUNCTION      0x01
#define EX_ILLEGAL_ADDRESS       0x02
#define EX_SLAVE_FAILURE         0x04

/* ======================== 函数声明 ======================== */
void     ModbusTCP_Init(void);             // 初始化寄存器表
void     ModbusTCP_Process(void);          // 处理接收到的 Modbus 请求
void     ModbusTCP_UpdateReg(uint16_t addr, uint16_t value); // 更新寄存器值

#endif
