/**
 * Slave Variant Selector — Final Phase Config-Driven Architecture
 *
 * Change ONE line below to switch between slave hardware configurations.
 * All differences (node ID, sensor list, CAN parameters) are resolved
 * via conditional compilation — no source code changes needed.
 *
 * Available variants:
 *   SLAVE_NODE_01  — DHT11 + BH1750       Temp/Humidity/Light (Phase 2 original)
 *   SLAVE_NODE_02  — LM393 + Reserved     Photo-sensor + expansion placeholder
 */

#ifndef __SLAVE_CONFIG_H
#define __SLAVE_CONFIG_H

/* ---- Variant tokens ---- */
#define SLAVE_NODE_01  1
#define SLAVE_NODE_02  2

/* ================================================================
 * SLAVE VARIANT SELECTION — modify this line to switch hardware
 * ================================================================ */
#define SLAVE_NODE_VARIANT   SLAVE_NODE_01

/* ---- Include the selected variant's configuration ---- */
#if   SLAVE_NODE_VARIANT == SLAVE_NODE_01
  #include "slave_node_01.h"
#elif SLAVE_NODE_VARIANT == SLAVE_NODE_02
  #include "slave_node_02.h"
#else
  #error "Unknown SLAVE_NODE_VARIANT — check slave_config.h"
#endif

/* ---- Safety defaults: undefined macros default to 0 (not silent compile) ---- */
#ifndef slave_has_dht11
  #define slave_has_dht11  0
#endif
#ifndef slave_has_bh1750
  #define slave_has_bh1750 0
#endif
#ifndef slave_has_lm393
  #define slave_has_lm393  0
#endif
#ifndef slave_has_reserved
  #define slave_has_reserved 0
#endif

/* ---- Compile-time guard: max 4 sensors for STM32F103C8 ---- */
#if (slave_has_dht11 + slave_has_bh1750 + slave_has_lm393 + slave_has_reserved) > 4
  #error "Too many sensors enabled — max 4 for STM32F103C8T6"
#endif

#endif /* __SLAVE_CONFIG_H */
