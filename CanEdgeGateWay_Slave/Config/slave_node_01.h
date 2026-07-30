/**
 * Slave Node 01 Configuration — DHT11 + BH1750
 *
 * Environment monitoring node (temperature, humidity, ambient light).
 * This is the Phase 2 original hardware configuration.
 */

#ifndef __SLAVE_NODE_01_H
#define __SLAVE_NODE_01_H

/* ---- Node identity ---- */
#define slave_node_id           0x01        /* CAN node address */
#define slave_node_name_str     "Node#01"   /* OLED display name (≤8 chars for 16-char line) */
#define slave_node_desc_str     "T+H+Light" /* Short description */
#define slave_online_flag       1           /* Online on power-up */

/* ---- Sensor enables — compile-time switch ---- */
#define slave_has_dht11         1   /* DHT11 on PB11 */
#define slave_has_bh1750        1   /* BH1750 on I2C (PB8/PB9) */
#define slave_has_lm393         0
#define slave_has_reserved      0

/* ---- CAN parameters ---- */
#define slave_can_id_offset     0x00        /* CAN ID offset (staggered for multi-slave bus) */
#define slave_heartbeat_ms      500         /* Heartbeat period */
#define slave_interval_normal   2000        /* 1级 normal sample period (ms) */
#define slave_interval_lowfreq  2000        /* 2级 low-frequency sample period (ms) */

/* ---- Per-sensor sample intervals (ms) — override defaults ---- */
#define sensor_dht11_interval_ms    2000
#define sensor_bh1750_interval_ms   2000

/* ---- Threshold alarm config (THR: alarm_lo, alarm_hi, hyst_lo, hyst_hi) ---- */
/* 温度: tenths °C, 例 253=25.3°C */
#define sensor_dht11_temp_thr       THR(  0, 450,  50, 400)  /* 0~45°C, 恢复 5~40°C */
/* 湿度: tenths %, 例 621=62.1% */
#define sensor_dht11_humi_thr       THR(200, 900, 250, 850)  /* 20%~90%, 恢复 25%~85% */
/* 光照: lux (封顶 32767, 超过视为极高亮度) */
#define sensor_bh1750_lux_thr       THR( 10, 500,  20, 500)
/*    报警: < 10 lux(太暗/遮挡) 或 > 30000 lux(极亮)
        恢复: ≥ 20 lux 且 ≤ 25000 lux */

#endif /* __SLAVE_NODE_01_H */
