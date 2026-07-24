/**
 * Local cache for CAN-offline fault tolerance
 *
 * When CAN communication fails (3 consecutive heartbeat TX failures),
 * sensor data is cached locally. On CAN recovery, cached data is
 * replayed at 200ms intervals to avoid flooding the bus.
 */

#ifndef __LOCAL_CACHE_H
#define __LOCAL_CACHE_H

#include "stm32f10x.h"
#include <stdint.h>

#define LOCAL_CACHE_MAX     50
#define CAN_DATA_LEN         8

/* Cache entry */
typedef struct {
    uint8_t  data[CAN_DATA_LEN];
    uint32_t id;            /* CAN ID */
    uint32_t timestamp;     /* SysTick at capture */
} LocalCacheEntry;

/* Cache structure */
typedef struct {
    LocalCacheEntry entries[LOCAL_CACHE_MAX];
    uint8_t   head;
    uint8_t   tail;
    uint8_t   count;
    uint8_t   can_offline;              /* 1 = CAN communication lost */
    uint8_t   tx_fail_count;            /* consecutive TX failures */
    uint8_t   replay_active;            /* 1 = replaying cached data */
    uint32_t  last_tx_success_tick;
    uint32_t  last_replay_tick;
} LocalCache;

/* API */
void     LocalCache_Init(LocalCache *cache);
uint8_t  LocalCache_Push(LocalCache *cache, uint32_t id, uint8_t *data);
uint8_t  LocalCache_Pop(LocalCache *cache, uint32_t *id, uint8_t *data);
uint8_t  LocalCache_Count(LocalCache *cache);
void     LocalCache_OnTxSuccess(LocalCache *cache);
void     LocalCache_OnTxFail(LocalCache *cache);
uint8_t  LocalCache_IsOffline(LocalCache *cache);
uint8_t  LocalCache_ShouldReplay(LocalCache *cache);

#endif /* __LOCAL_CACHE_H */
