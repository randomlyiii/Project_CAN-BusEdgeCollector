/**
 * BH1750 Ambient Light Sensor Driver
 *
 * I2C interface: software I2C on PB8(SCL) / PB9(SDA)
 * Address: 0x23 (ADDR pin = GND)
 * Mode: continuous high-resolution (1 lux, 120ms measurement)
 */

#ifndef __BH1750_H
#define __BH1750_H

#include "stm32f10x.h"

/* I2C address (7-bit, shifted left 1) */
#define BH1750_ADDR_WRITE    0x46   /* 0x23 << 1 | 0 */
#define BH1750_ADDR_READ     0x47   /* 0x23 << 1 | 1 */

/* Commands */
#define BH1750_CMD_POWER_ON        0x01
#define BH1750_CMD_RESET           0x07
#define BH1750_CMD_CONT_HR_MODE    0x10   /* 1lux resolution, 120ms */

/* API */
void     BH1750_Init(void);
uint8_t  BH1750_Read(uint16_t *lux);   /* 0=OK, 1=fail */

#endif /* __BH1750_H */
