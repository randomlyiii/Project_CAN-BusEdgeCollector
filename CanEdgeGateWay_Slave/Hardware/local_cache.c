/**
 * Local cache implementation
 */

#include "local_cache.h"
#include "delay.h"
#include <string.h>

#define TX_FAIL_THRESHOLD        3     /* Consecutive fails → offline */
#define REPLAY_INTERVAL_MS       200   /* Between replay frames */

void LocalCache_Init(LocalCache *cache)
{
    memset(cache, 0, sizeof(LocalCache));
}

uint8_t LocalCache_Push(LocalCache *cache, uint32_t id, uint8_t *data)
{
    if (cache->count >= LOCAL_CACHE_MAX) {
        /* Full: drop oldest */
        cache->tail = (cache->tail + 1) % LOCAL_CACHE_MAX;
        cache->count--;
    }

    LocalCacheEntry *entry = &cache->entries[cache->head];
    entry->id        = id;
    entry->timestamp = Delay_GetTick();
    memcpy(entry->data, data, CAN_DATA_LEN);

    cache->head = (cache->head + 1) % LOCAL_CACHE_MAX;
    cache->count++;
    return 0;
}

uint8_t LocalCache_Pop(LocalCache *cache, uint32_t *id, uint8_t *data)
{
    if (cache->count == 0) return 1;

    LocalCacheEntry *entry = &cache->entries[cache->tail];
    *id = entry->id;
    memcpy(data, entry->data, CAN_DATA_LEN);

    cache->tail = (cache->tail + 1) % LOCAL_CACHE_MAX;
    cache->count--;
    return 0;
}

uint8_t LocalCache_Count(LocalCache *cache)
{
    return cache->count;
}

void LocalCache_OnTxSuccess(LocalCache *cache)
{
    cache->tx_fail_count = 0;
    cache->last_tx_success_tick = Delay_GetTick();

    if (cache->can_offline) {
        /* CAN recovered — start replay */
        cache->can_offline = 0;
        cache->replay_active = 1;
        cache->last_replay_tick = Delay_GetTick();
    }
}

void LocalCache_OnTxFail(LocalCache *cache)
{
    cache->tx_fail_count++;
    if (cache->tx_fail_count >= TX_FAIL_THRESHOLD) {
        cache->can_offline = 1;
        cache->replay_active = 0;
    }
}

uint8_t LocalCache_IsOffline(LocalCache *cache)
{
    return cache->can_offline;
}

uint8_t LocalCache_ShouldReplay(LocalCache *cache)
{
    if (!cache->replay_active) return 0;
    if (cache->count == 0) {
        cache->replay_active = 0;
        return 0;
    }

    uint32_t now = Delay_GetTick();
    if (now - cache->last_replay_tick >= REPLAY_INTERVAL_MS) {
        cache->last_replay_tick = now;
        return 1;
    }
    return 0;
}

/* 清理超过 10s 的过期缓存帧, 防恢复后总线流量风暴 */
#define CACHE_EXPIRE_MS     10000

void LocalCache_Cleanup(LocalCache *cache)
{
    uint32_t now = Delay_GetTick();
    uint8_t  src, dst;

    /* 从 tail 向后扫描, 将未过期帧紧凑排列到队列前端 */
    dst = cache->tail;
    for (uint8_t i = 0; i < cache->count; i++) {
        src = (cache->tail + i) % LOCAL_CACHE_MAX;
        if (now - cache->entries[src].timestamp < CACHE_EXPIRE_MS) {
            if (src != dst)
                cache->entries[dst] = cache->entries[src];
            dst = (dst + 1) % LOCAL_CACHE_MAX;
        }
    }

    cache->head  = dst;
    cache->count = (dst >= cache->tail) ? (dst - cache->tail) :
                   (LOCAL_CACHE_MAX - cache->tail + dst);
    if (cache->count == 0) cache->replay_active = 0;
}
