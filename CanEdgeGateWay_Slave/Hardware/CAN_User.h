/**
 * CAN User Layer — Slave Phase 2 (FreeRTOS)
 *
 * Changes from Phase 1:
 *   - ISR simplified: read frame + signal semaphore
 *   - Added BH1750 light sensor frame (CAN_FUNC_LIGHT, ID 0x300+node)
 *   - Local cache for CAN-offline fault tolerance
 */

#ifndef __CAN_USER_H
#define __CAN_USER_H

#include "stm32f10x.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "local_cache.h"

/* ---- CAN ID bases ---- */
#define CAN_ID_EMERGENCY_BASE     0x100
#define CAN_ID_NORMAL_BASE        0x200
#define CAN_ID_LOWFREQ_BASE       0x300

/* ---- Function codes ---- */
#define CAN_FUNC_HEARTBEAT        0x01
#define CAN_FUNC_TEMP_HUMI        0x02
#define CAN_FUNC_ALARM            0x03
#define CAN_FUNC_RECOVER          0x04
#define CAN_FUNC_LIGHT            0x05

/* ---- Frame byte index ---- */
#define CAN_DATA_TYPE_IDX         0
#define CAN_DATA_SRC_IDX          1
#define CAN_DATA_FUNC_IDX         2
#define CAN_DATA_PAYLOAD_IDX      3
#define CAN_DATA_CHKSUM_IDX       7

#define SLAVE_NODE_ID             0x01

/* ---- Extern ---- */
extern volatile uint8_t   g_can_rx_flag;
extern SemaphoreHandle_t  g_can_rx_sem;
extern LocalCache         g_local_cache;
extern volatile uint32_t  g_can_tx_success_count;
extern volatile uint32_t  g_can_tx_fail_count;

/* ---- Init ---- */
void CAN_User_Init(void);

/* ---- Frame send ---- */
uint8_t CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len);

/* ---- Protocol frame builders ---- */
void CAN_SendHeartBeat(void);
void CAN_SendTempHumi(uint8_t temp_int, uint8_t temp_dec,
                      uint8_t humi_int, uint8_t humi_dec);
void CAN_SendLight(uint16_t lux);
void CAN_SendAlarm(void);
void CAN_SendRecover(void);

/* ---- ISR ---- */
void CAN1_RX0_IRQHandler(void);

#endif /* __CAN_USER_H */
