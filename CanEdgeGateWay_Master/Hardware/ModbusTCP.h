/**
 * Modbus TCP Server — Phase 2 enhanced
 *
 * Changes from Phase 1:
 *   - Dual-zone cache: realtime (overwrite) + history (FIFO, 100 entries)
 *   - Disconnect grading: <3s keep all, ≥3s purge level-2 data
 *   - Extended registers 0x0020-0x0029 for diagnostics
 *   - Batch upload on reconnect (20 entries per TCP connection)
 */

#ifndef __MODBUSTCP_H
#define __MODBUSTCP_H

#include "stm32f10x.h"
#include <stdint.h>

/* ---- Modbus TCP port ---- */
#define MODBUS_PORT              502

/* ---- Register map ---- */
/* Slave 1 data (0x00–0x04) */
#define REG_SLAVE1_TEMP          0x0000
#define REG_SLAVE1_HUMI          0x0001
#define REG_SLAVE1_HB_CNT        0x0002
#define REG_SLAVE1_FAULT         0x0003
#define REG_SLAVE1_ONLINE        0x0004

/* Slave 2 data (0x05–0x0B) */
#define REG_SLAVE2_TEMP          0x0005
#define REG_SLAVE2_HUMI          0x0006
#define REG_SLAVE2_HB_CNT        0x0007
#define REG_SLAVE2_FAULT         0x0008
#define REG_SLAVE2_ONLINE        0x0009
#define REG_SLAVE2_LM393_AO      0x000A   /* LM393 analog 0~4095 */
#define REG_SLAVE2_LM393_DO      0x000B   /* LM393 digital 0/1 */

/* CAN diagnostics (0x10–0x15) */
#define REG_CAN_BUS_LOAD         0x0010
#define REG_CAN_TEC              0x0011
#define REG_CAN_REC              0x0012
#define REG_CAN_ERR_LEVEL        0x0013
#define REG_ONLINE_NODE_MASK     0x0014
#define REG_NET_ERR_CNT          0x0015

/* Control registers (0x20–0x21) */
#define REG_CTRL_RESET           0x0020   /* Write 0x55 to reset CAN */
#define REG_CTRL_UNBLACKLIST     0x0021   /* Write node_id to unblock */

/* ---- Extended diagnostic registers (Phase 2) ---- */
#define REG_LIGHT_SENSOR         0x0022   /* BH1750 lux value (from slave) */
#define REG_FIFO_HIGH_COUNT      0x0023
#define REG_FIFO_NORMAL_COUNT    0x0024
#define REG_FIFO_HIGH_OVERFLOW   0x0025
#define REG_FIFO_NORMAL_OVERFLOW 0x0026
#define REG_BUSOFF_RECOVERY      0x0027
#define REG_HIST_CACHE_COUNT     0x0028
#define REG_SYS_UPTIME_HIGH      0x0029   /* Uptime seconds, high word */
#define REG_SYS_UPTIME_LOW       0x002A   /* Uptime seconds, low word */
#define REG_THROTTLE_LEVEL       0x002B   /* 0=normal, 1=low-freq, 2=emergency */
#define REG_OLED_AUTO_RETURN_MS  0x002C   /* OLED 自动回主页超时 ms, 0=永不 */

#define MODBUS_REG_COUNT         0x0030

/* ---- Modbus function codes ---- */
#define FC_READ_HOLDING_REGS     0x03
#define FC_WRITE_SINGLE_REG      0x06

/* Exception codes */
#define EX_ILLEGAL_FUNCTION      0x01
#define EX_ILLEGAL_ADDRESS       0x02
#define EX_SLAVE_FAILURE         0x04

/* ---- MBAP header ---- */
#define MBAP_TID_IDX             0
#define MBAP_PID_IDX             2
#define MBAP_LEN_IDX             4
#define MBAP_UID_IDX             6
#define MBAP_HEADER_LEN          7
#define PDU_FC_IDX               7
#define PDU_ADDR_IDX             8
#define PDU_DATA_IDX             10
#define RESP_DATA_IDX            9    /* MBAP(7) + FC(1) + ByteCnt(1) = 9 */

/* ---- History cache ---- */
#define HIST_CACHE_MAX           100
#define BATCH_UPLOAD_COUNT       20    /* Per-connection batch size */
#define SHORT_DISCONNECT_MS      3000
#define LONG_DISCONNECT_MS       3000

/* ---- Cache entry ---- */
typedef struct {
    uint16_t addr;
    uint16_t value;
    uint32_t timestamp;
    uint8_t  priority;   /* 0/1/2 */
} ModbusHistEntry;

/* ---- Modbus cache ---- */
typedef struct {
    ModbusHistEntry history[HIST_CACHE_MAX];
    uint8_t         hist_head;
    uint8_t         hist_tail;
    uint8_t         hist_count;
    uint32_t        disconnect_start_tick;
    uint8_t         was_disconnected;
    uint8_t         batch_sent;        /* entries sent in current batch */
} ModbusCache;

/* ---- Globals ---- */
extern volatile uint16_t g_modbus_rx_errs;
extern volatile uint8_t  g_modbus_offline;
extern uint16_t g_regs[MODBUS_REG_COUNT];

/* ---- API ---- */
void     ModbusTCP_Init(void);
void     ModbusTCP_Process(void);          /* Called from vTask_W5500 */
void     ModbusTCP_SyncFromCAN(void);      /* Called from vTask_Protocol */
void     ModbusTCP_OnDisconnect(void);     /* Called when link drops */
void     ModbusTCP_OnReconnect(void);      /* Called when link restores */
uint8_t  ModbusTCP_HasPendingBatch(void);  /* Non-zero if history to upload */
uint16_t ModbusTCP_BatchUpload(void);      /* Upload up to 20 entries */

#endif /* __MODBUSTCP_H */
