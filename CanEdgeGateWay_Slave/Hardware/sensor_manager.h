/**
 * Sensor Manager — Final Phase Sensor Abstraction Layer
 *
 * Table-driven sensor management with compile-time sensor selection.
 *
 * Architecture:
 *   g_sensor_table[]  — static array, populated by #if conditional compilation
 *   adapter functions  — thin wrappers bridging existing drivers to uniform API
 *   sensor_manager_read_all() — one-call sampling with per-sensor interval control
 *
 * Benefits:
 *   - Adding a new sensor: ①write driver + ②write adapter + ③add table entry
 *   - No changes needed to task code, CAN layer, or OLED rendering
 *   - Unused sensors excluded at compile time (zero Flash/RAM overhead)
 */

#ifndef __SENSOR_MANAGER_H
#define __SENSOR_MANAGER_H

#include "stm32f10x.h"
#include "../FreeRTOS/inc/FreeRTOS.h"
#include "../Config/sensor_type.h"
#include <string.h>

/* ---- Sensor data union (maps to CAN 8-byte payload) ---- */
typedef union {
    struct {
        uint8_t temp_int;
        uint8_t temp_dec;
        uint8_t humi_int;
        uint8_t humi_dec;
    } dht11;
    struct {
        uint16_t lux;
    } bh1750;
    struct {
        uint8_t  digital;   /* DO: 0=bright, 1=dark */
        uint16_t analog;    /* AO: raw ADC 0~4095 */
    } lm393;
    struct {
        uint16_t ch1;       /* Reserved channel 1 data */
    } reserved;
    uint8_t raw[8];          /* Compatible with CAN 8-byte frame payload */
} sensor_data_t;

/* ---- Threshold alarm config (lo=过低报警, hi=过高报警, 滞回恢复) ---- */
typedef struct {
    int16_t alarm_lo;           /* 低于此值 → 报警 */
    int16_t alarm_hi;           /* 高于此值 → 报警 */
    int16_t hyst_lo;            /* lo报警后, 值>=此值 → 恢复 */
    int16_t hyst_hi;            /* hi报警后, 值<=此值 → 恢复 */
} sensor_threshold_t;

/* ---- Sensor descriptor (one per enabled sensor) ---- */
typedef struct {
    /* Static configuration */
    const char      name[12];           /* Human-readable name */
    uint8_t         type_id;            /* SENSOR_TYPE_* */
    uint8_t         priority;           /* SENSOR_PRIOR_NORMAL / LOWFREQ */
    uint16_t        interval_ms;        /* Sample interval */
    uint8_t       (*init_fn)(void);     /* Init function, 0=OK; NULL=no driver */
    uint8_t       (*read_fn)(sensor_data_t *out); /* Read function, NULL=no driver */
    uint8_t         enabled;            /* 1=enabled, 0=disabled */
    sensor_threshold_t threshold;       /* 主值阈值 (全0=禁用) */
    sensor_threshold_t threshold2;      /* 次值阈值 (DHT11湿度等, 全0=禁用) */

    /* Runtime state */
    uint8_t         online;             /* 0=offline/fault, 1=online */
    uint8_t         fault_count;        /* Consecutive failure count */
    uint8_t         alarm_active;       /* 1=alarm sent, clear when recovered */
    uint8_t         recover_pending;    /* 1=need to send RECOVER frame */
    TickType_t      last_read_tick;     /* Last sample timestamp */
    sensor_data_t   last_data;          /* Most recent successful reading */
} sensor_t;

/* ---- Extract comparison value from sensor_data_t by type_id ---- */
static inline int16_t sensor_value_of(uint8_t type_id, const sensor_data_t *d)
{
    switch (type_id) {
    case SENSOR_TYPE_DHT11:     return (int16_t)d->dht11.temp_int * 10 + d->dht11.temp_dec;
    case SENSOR_TYPE_BH1750: {
        int16_t _v = (int16_t)d->bh1750.lux;
        return (_v < 0) ? 32767 : _v;      /* 防 uint16_t > 32767 溢出 */
    }
    case SENSOR_TYPE_LM393_AO:  return (int16_t)d->lm393.analog;
    case SENSOR_TYPE_LM393_DO:  return (int16_t)d->lm393.digital;
    default:                    return 0;
    }
}
/* ---- Extract secondary comparison value (humidity for DHT11) ---- */
static inline int16_t sensor_value_of2(uint8_t type_id, const sensor_data_t *d)
{
    switch (type_id) {
    case SENSOR_TYPE_DHT11:     return (int16_t)d->dht11.humi_int * 10 + d->dht11.humi_dec;
    default:                    return 0;
    }
}

/* ---- I2C bus mutex (BH1750 + OLED share PB8/PB9) ---- */
#include "../FreeRTOS/inc/semphr.h"
extern SemaphoreHandle_t g_i2c_mutex;

/* ---- Public API ---- */
uint8_t     sensor_manager_init(void);              /* Init all enabled sensors */
uint8_t     sensor_manager_get_count(void);          /* Get enabled sensor count */
sensor_t   *sensor_manager_get_by_index(uint8_t idx);/* Get sensor by table index */
sensor_t   *sensor_manager_get_by_type(uint8_t id);  /* Get sensor by type ID */
void        sensor_manager_read_all(void);           /* Poll all due sensors */
uint8_t     sensor_manager_has_alarm(void);          /* Any sensor in alarm state? */

#endif /* __SENSOR_MANAGER_H */
