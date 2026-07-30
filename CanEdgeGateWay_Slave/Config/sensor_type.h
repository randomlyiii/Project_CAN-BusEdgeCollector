/**
 * Sensor Type Enumerations — Final Phase
 *
 * Maps sensor type IDs to CAN function codes for uniform dispatching.
 * Priority levels align with README.md acquisition layer design:
 *   SENSOR_PRIOR_NORMAL  → 1级常态任务 (CAN_ID_NORMAL_BASE)
 *   SENSOR_PRIOR_LOWFREQ → 2级低频任务 (CAN_ID_LOWFREQ_BASE)
 */

#ifndef __SENSOR_TYPE_H
#define __SENSOR_TYPE_H

/* ---- Sensor type IDs (also map to CAN function codes) ---- */
#define SENSOR_TYPE_NONE        0x00
#define SENSOR_TYPE_DHT11       0x01   /* == CAN_FUNC_TEMP_HUMI */
#define SENSOR_TYPE_BH1750      0x02   /* == CAN_FUNC_LIGHT */
#define SENSOR_TYPE_LM393_DO    0x03   /* LM393 digital output */
#define SENSOR_TYPE_LM393_AO    0x04   /* LM393 analog value */
#define SENSOR_TYPE_RESERVED    0x05   /* Reserved channel (placeholder, no driver) */

/* ---- Sensor priority levels ---- */
#define SENSOR_PRIOR_NORMAL     1      /* 1级常态任务 — 高频/关键数据 */
#define SENSOR_PRIOR_LOWFREQ    2      /* 2级低频任务 — 辅助/可选数据 */

/* ---- Threshold alarm default: 0/0 = 禁用 ---- */
#define THR_DISABLED            0, 0, 0, 0

/* ---- Helper: 构建阈值 (alarm_lo, alarm_hi, hyst_lo, hyst_hi) ---- */
/*     lo=低于此值报警, hi=高于此值报警
       hyst_lo=恢复时需>=此值, hyst_hi=恢复时需<=此值 */
#define THR(lo, hi, hlo, hhi)   (int16_t)(lo), (int16_t)(hi), \
                                (int16_t)(hlo), (int16_t)(hhi)

#endif /* __SENSOR_TYPE_H */
