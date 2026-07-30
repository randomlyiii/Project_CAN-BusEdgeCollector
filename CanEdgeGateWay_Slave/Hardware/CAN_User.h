/**
 * CAN User Layer — Slave Final Phase (Config-Driven)
 *
 * Changes from Phase 2:
 *   - Node ID driven by slave_config.h (slave_node_id), not hardcoded
 *   - Added CAN_SendSensorData() generic frame builder (table-driven)
 *   - Added CAN_FUNC_LM393_DO + CAN_FUNC_RESERVED for new sensor types
 *   - Old per-sensor functions (CAN_SendTempHumi/Light) retained for backward compat
 */

#ifndef __CAN_USER_H
#define __CAN_USER_H

#include "stm32f10x.h"
#include <stdint.h>
#include "../FreeRTOS/inc/FreeRTOS.h"
#include "../FreeRTOS/inc/semphr.h"
#include "../Config/slave_config.h"
#include "local_cache.h"
#include "sensor_manager.h"

/* ---- CAN ID bases (aligned with README.md frame ID design) ---- */
#define CAN_ID_EMERGENCY_BASE     0x100   /* 0级: alarm/fault */
#define CAN_ID_NORMAL_BASE        0x200   /* 1级: normal-periodic */
#define CAN_ID_LOWFREQ_BASE       0x300   /* 2级: low-frequency */

/* ---- Function codes ---- */
#define CAN_FUNC_HEARTBEAT        0x01
#define CAN_FUNC_TEMP_HUMI        0x02
#define CAN_FUNC_ALARM            0x03
#define CAN_FUNC_RECOVER          0x04
#define CAN_FUNC_LIGHT            0x05   /* BH1750 / LM393 AO (光照) */
#define CAN_FUNC_LM393_DO         0x06   /* LM393 digital output (new) */
#define CAN_FUNC_RESERVED         0x07   /* Reserved channel (new) */

/* ---- Frame byte index ---- */
#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

/* Backward compat: SLAVE_NODE_ID now reads from config layer */
#define SLAVE_NODE_ID             slave_node_id

/* ---- Extern ---- */
extern volatile uint8_t   g_can_rx_flag;
extern LocalCache         g_local_cache;
extern volatile uint32_t  g_can_tx_success_count;
extern volatile uint32_t  g_can_tx_fail_count;

/* ---- Init ---- */
void CAN_User_Init(void);

/* ---- Frame send ---- */
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);

/* ---- Generic sensor data sender (Final Phase — table-driven) ---- */
void CAN_SendSensorData(uint8_t sensor_type_id, sensor_data_t *data);

/* ---- Protocol frame builders (Phase 2 compat — retained) ---- */
void CAN_SendHeartBeat(void);
void CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                      uint8_t humi_int, uint8_t humi_dec);
void CAN_SendLight(uint16_t lux);
void CAN_SendAlarm(void);
void CAN_SendRecover(void);

#endif /* __CAN_USER_H */
