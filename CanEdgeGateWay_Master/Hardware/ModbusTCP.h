#ifndef __MODBUSTCP_H
#define __MODBUSTCP_H

#include "stm32f10x.h"
#include <stdint.h>

/* ===================== 寄存器映射 ===================== */
#define REG_SLAVE1_TEMP          0x0000
#define REG_SLAVE1_HUMI          0x0001
#define REG_SLAVE1_HB_CNT        0x0002
#define REG_SLAVE1_FAULT         0x0003
#define REG_SLAVE1_ONLINE        0x0004
#define REG_SLAVE2_TEMP          0x0005
#define REG_SLAVE2_HUMI          0x0006
#define REG_SLAVE2_HB_CNT        0x0007
#define REG_SLAVE2_FAULT         0x0008
#define REG_SLAVE2_ONLINE        0x0009

#define REG_CAN_BUS_LOAD         0x0010
#define REG_CAN_TEC              0x0011
#define REG_CAN_REC              0x0012
#define REG_CAN_ERR_LEVEL        0x0013
#define REG_ONLINE_NODE_MASK     0x0014
#define REG_NET_ERR_CNT          0x0015

#define REG_CTRL_RESET           0x0020
#define REG_CTRL_UNBLACKLIST     0x0021

#define MODBUS_REG_COUNT         0x0030

/* ===================== Modbus功能码 ===================== */
#define FC_READ_HOLDING_REGS     0x03
#define FC_WRITE_SINGLE_REG      0x06

#define EX_ILLEGAL_FUNCTION      0x01
#define EX_ILLEGAL_ADDRESS       0x02
#define EX_SLAVE_FAILURE         0x04

/* ===================== MBAP头 ===================== */
#define MBAP_TID_IDX             0
#define MBAP_PID_IDX             2
#define MBAP_LEN_IDX             4
#define MBAP_UID_IDX             6
#define MBAP_HEADER_LEN          7
#define PDU_FC_IDX               7
#define PDU_ADDR_IDX             8
#define PDU_DATA_IDX             10
#define RESP_DATA_IDX            9    /* 响应数据起始 (MBAP(7)+FC(1)+ByteCnt(1)=9) */

/* ===================== 状态 ===================== */
extern volatile uint16_t g_modbus_rx_errs;   /* 通信故障计数 */
extern volatile uint8_t  g_modbus_offline;   /* 离线标记 */

/* ===================== API ===================== */
void     ModbusTCP_Init(void);
void     ModbusTCP_Process(void);
void     ModbusTCP_SyncFromCAN(void);

#endif
