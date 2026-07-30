/**
 * Slave Node 02 Configuration — LM393 + Reserved Channel
 *
 * Photo-resistor monitoring node with expansion placeholder.
 * LM393: DO (digital bright/dark) + AO (analog ADC 0~4095)
 * Reserved channel: sensor_t struct + CAN comm + OLED placeholder only (no driver)
 */

#ifndef __SLAVE_NODE_02_H
#define __SLAVE_NODE_02_H

/* ---- Node identity ---- */
#define slave_node_id           0x02
#define slave_node_name_str     "Node#02"
#define slave_node_desc_str     "Photo+Resv"
#define slave_online_flag       1

/* ---- Sensor enables ---- */
#define slave_has_dht11         0
#define slave_has_bh1750        0
#define slave_has_lm393         1
#define slave_has_reserved      1

/* ---- CAN parameters ---- */
#define slave_can_id_offset     0x10
#define slave_heartbeat_ms      500
#define slave_interval_normal   2000
#define slave_interval_lowfreq  2000

/* ---- Per-sensor sample intervals (ms) ---- */
#define sensor_lm393_interval_ms    2000

/* ---- Threshold alarm config ---- */
/* LM393 AO: raw ADC 0~4095 */
#define sensor_lm393_ao_thr         THR( 500, 3950, 550, 3900) /* 短路/断路检测 */

/* ---- Reserved channel parameters ---- */
#define sensor_reserved_name    "CH1"
#define sensor_reserved_type    SENSOR_TYPE_RESERVED
#define sensor_reserved_priority SENSOR_PRIOR_LOWFREQ
#define sensor_reserved_interval_ms  2000

#endif /* __SLAVE_NODE_02_H */
