/**
 * Sensor Manager Implementation — Final Phase
 *
 * Table-driven: sensors are populated via #if conditional compilation.
 * Adapter pattern: thin wrappers bridge existing drivers to uniform API.
 * Reserved channel: init_fn/read_fn = NULL, only struct + CAN + OLED exist.
 */

#include "sensor_manager.h"
#include "dht11.h"
#include "bh150.h"
#include "lm393.h"
#include "../Config/slave_config.h"

/* ---- I2C bus mutex (BH1750 + OLED share PB8/PB9) ---- */
SemaphoreHandle_t g_i2c_mutex = NULL;

/* ================================================================
 * Driver Adapters — bridge existing driver APIs to sensor_t interface
 *
 * Each adapter:
 *   1. init_fn(void) → 0=OK
 *   2. read_fn(sensor_data_t *out) → 0=OK, non-zero=fail
 *
 * Underlying driver interfaces are NEVER modified (zero regression risk).
 * ================================================================ */

/* ---- DHT11 Adapter ---- */
#if slave_has_dht11
static uint8_t dht11_adapter_init(void)
{
    DHT11_GPIO_Init();
    return 0;
}
static uint8_t dht11_adapter_read(sensor_data_t *out)
{
    return DHT11_Read_Data(&out->dht11.humi_int, &out->dht11.humi_dec,
                           &out->dht11.temp_int, &out->dht11.temp_dec);
}
#endif /* slave_has_dht11 */

/* ---- BH1750 Adapter ---- */
#if slave_has_bh1750
static uint8_t bh1750_adapter_init(void)
{
    BH1750_Init();
    return 0;
}
static uint8_t bh1750_adapter_read(sensor_data_t *out)
{
    uint8_t ret;
    /* Protect shared I2C bus (PB8/PB9) — OLED may be writing */
    if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50));
    ret = BH1750_Read(&out->bh1750.lux);
    if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);
    return ret;
}
#endif /* slave_has_bh1750 */

/* ---- LM393 Adapter ---- */
#if slave_has_lm393
static uint8_t lm393_adapter_init(void)
{
    Lm393_Init();
    return 0;
}
static uint8_t lm393_adapter_read(sensor_data_t *out)
{
    out->lm393.digital = Lm393_ReadDigital();
    if (Lm393_ReadAnalog(&out->lm393.analog) != 0)
        return 1;                                   /* ADC 转换超时 */
    if (out->lm393.analog == 0 || out->lm393.analog > 4000)
        return 1;                                   /* 传感器掉线/短路 */
    return 0;
}
#endif /* slave_has_lm393 */

/* Reserved channel: no adapter — init_fn/read_fn remain NULL.
   When expanded with a real driver later:
     1. Write driver (my_sensor.h/c)
     2. Write adapter functions here under #if slave_has_reserved
     3. Replace NULL with adapter function pointers in table entry
     4. Update OLED renderer switch-case for actual data display */

/* ================================================================
 * Sensor Table — populated via #if conditional compilation
 *
 * Each entry consumes sizeof(sensor_t) bytes (≈48 bytes).
 * Unused sensors are EXCLUDED from the table → zero Flash/RAM overhead.
 *
 * Max table size: 4 sensors (compile-time guard in slave_config.h).
 * ================================================================ */

static sensor_t g_sensor_table[] = {
#if slave_has_dht11
    {
        .name           = "DHT11",
        .type_id        = SENSOR_TYPE_DHT11,
        .priority       = SENSOR_PRIOR_NORMAL,
        .interval_ms    = sensor_dht11_interval_ms,
        .init_fn        = dht11_adapter_init,
        .read_fn        = dht11_adapter_read,
        .enabled        = 1,
        .threshold      = { sensor_dht11_temp_thr },
        .threshold2     = { sensor_dht11_humi_thr },
        .online         = 0,
        .fault_count    = 0,
        .alarm_active   = 0,
        .recover_pending = 0,
        .last_read_tick = 0,
    },
#endif
#if slave_has_bh1750
    {
        .name           = "BH1750",
        .type_id        = SENSOR_TYPE_BH1750,
        .priority       = SENSOR_PRIOR_LOWFREQ,
        .interval_ms    = sensor_bh1750_interval_ms,
        .init_fn        = bh1750_adapter_init,
        .read_fn        = bh1750_adapter_read,
        .enabled        = 1,
        .threshold      = { sensor_bh1750_lux_thr },
        .online         = 0,
        .fault_count    = 0,
        .alarm_active   = 0,
        .recover_pending = 0,
        .last_read_tick = 0,
    },
#endif
#if slave_has_lm393
    {
        .name           = "LM393",
        .type_id        = SENSOR_TYPE_LM393_AO,
        .priority       = SENSOR_PRIOR_LOWFREQ,
        .interval_ms    = sensor_lm393_interval_ms,
        .init_fn        = lm393_adapter_init,
        .read_fn        = lm393_adapter_read,
        .enabled        = 1,
        .threshold      = { sensor_lm393_ao_thr },
        .online         = 0,
        .fault_count    = 0,
        .alarm_active   = 0,
        .recover_pending = 0,
        .last_read_tick = 0,
    },
#endif
#if slave_has_reserved
    {
        .name           = sensor_reserved_name,   /* "CH1" from config */
        .type_id        = SENSOR_TYPE_RESERVED,
        .priority       = sensor_reserved_priority,
        .interval_ms    = sensor_reserved_interval_ms,
        .init_fn        = NULL,                   /* No driver — placeholder */
        .read_fn        = NULL,                   /* No driver — placeholder */
        .enabled        = 1,
        .online         = 1,                      /* Always "online" for display */
        .fault_count    = 0,
        .alarm_active   = 0,
        .recover_pending = 0,
        .last_read_tick = 0,
    },
#endif
    /* Sentinel: at least one entry required (no sensors = placeholder disabled) */
    { .enabled = 0 }
};

#define SENSOR_COUNT  (sizeof(g_sensor_table) / sizeof(g_sensor_table[0]))

/* ================================================================
 * API Implementation
 * ================================================================ */

/**
 * Initialize all enabled sensors.
 * Returns 0 if ALL inits succeeded, 1 if any failed.
 * Sensors without drivers (reserved) default to online=1.
 */
uint8_t sensor_manager_init(void)
{
    uint8_t all_ok = 1;

    /* Create I2C mutex (BH1750 + OLED share PB8/PB9) — do this first */
    g_i2c_mutex = xSemaphoreCreateMutex();

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        sensor_t *s = &g_sensor_table[i];
        if (!s->enabled) continue;

        if (s->init_fn) {
            if (s->init_fn() != 0) {
                s->online = 0;
                all_ok = 0;
            } else {
                s->online = 1;
            }
        } else {
            /* Driverless sensor (reserved channel): default online=1 */
            s->online = 1;
        }
        s->last_read_tick = xTaskGetTickCount();
    }
    return all_ok;
}

uint8_t sensor_manager_get_count(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (g_sensor_table[i].enabled) cnt++;
    }
    return cnt;
}

sensor_t *sensor_manager_get_by_index(uint8_t idx)
{
    if (idx >= SENSOR_COUNT) return NULL;
    return &g_sensor_table[idx];
}

sensor_t *sensor_manager_get_by_type(uint8_t id)
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (g_sensor_table[i].type_id == id)
            return &g_sensor_table[i];
    }
    return NULL;
}

/**
 * Poll all enabled sensors.
 * Each sensor's actual sample interval is enforced via last_read_tick.
 * Call this at 200ms — internal per-sensor gating prevents over-sampling.
 *
 * Fault escalation: 3 consecutive failures → alarm_active = 1
 * Fault recovery:   any successful read clears alarm_active and sets
 *                   recover_pending for the CAN task to send RECOVER frame.
 */
void sensor_manager_read_all(void)
{
    TickType_t now = xTaskGetTickCount();

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        sensor_t *s = &g_sensor_table[i];
        if (!s->enabled) continue;

        /* Gate by per-sensor interval */
        if ((now - s->last_read_tick) < pdMS_TO_TICKS(s->interval_ms))
            continue;

        s->last_read_tick = now;

        /* Driverless sensor (reserved channel): no read, maintain online */
        if (!s->read_fn) {
            s->online = 1;
            continue;
        }

        /* Execute driver read */
        sensor_data_t data;
        memset(&data, 0, sizeof(data));

        if (s->read_fn(&data) == 0) {
            /* Success: update cache, reset fault tracking */
            s->last_data = data;
            s->online = 1;
            s->fault_count = 0;

            /* ---- Threshold alarm (全零 = 禁用) ---- */
            if (s->threshold.alarm_lo || s->threshold.alarm_hi) {
                int16_t val = sensor_value_of(s->type_id, &data);
                if (s->alarm_active) {
                    /* 已报警 → 滞回区内则恢复 */
                    if (val >= s->threshold.hyst_lo && val <= s->threshold.hyst_hi) {
                        s->alarm_active    = 0;
                        s->recover_pending = 1;
                    }
                } else {
                    /* 未报警 → 超限则触发 */
                    if (val < s->threshold.alarm_lo || val > s->threshold.alarm_hi) {
                        s->alarm_active    = 1;
                        /* 注意: fault报警的自动恢复不适用于阈值报警 */
                    }
                }
            } else {
                /* 未启用阈值: 旧行为 — 成功读即清除故障报警 */
                if (s->alarm_active) {
                    s->alarm_active    = 0;
                    s->recover_pending = 1;
                }
            }

            /* ---- threshold2 (次值, DHT11 湿度等) ---- */
            if (s->threshold2.alarm_lo || s->threshold2.alarm_hi) {
                int16_t val2 = sensor_value_of2(s->type_id, &data);
                if (s->alarm_active) {
                    if (val2 >= s->threshold2.hyst_lo && val2 <= s->threshold2.hyst_hi) {
                        s->alarm_active    = 0;
                        s->recover_pending = 1;
                    }
                } else {
                    if (val2 < s->threshold2.alarm_lo || val2 > s->threshold2.alarm_hi) {
                        s->alarm_active    = 1;
                    }
                }
            }
        } else {
            /* Failure: accumulate fault count */
            s->fault_count++;
            if (s->fault_count >= 3 && !s->alarm_active) {
                s->alarm_active = 1;
            }
        }
    }
}

/**
 * Check if any sensor is currently in alarm state.
 * Used by OLED renderer to show "ALARM" status.
 */
uint8_t sensor_manager_has_alarm(void)
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (g_sensor_table[i].alarm_active)
            return 1;
    }
    return 0;
}
