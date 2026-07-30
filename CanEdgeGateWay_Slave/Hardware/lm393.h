/**
 * LM393 Photoresistor Module Driver (DO + AO)
 *
 * Hardware: common 4-pin light sensor module with LM393 comparator
 *   DO → PB0 (digital, LOW = bright / above threshold)
 *   AO → PB1 (analog, ADC1_IN9, 0~4095)
 *
 * Threshold is adjustable via onboard potentiometer.
 */

#ifndef __LM393_H
#define __LM393_H

#include "stm32f10x.h"

/* ---- Pin definitions ---- */
#define LM393_DO_PORT        GPIOB
#define LM393_DO_PIN         GPIO_Pin_0

#define LM393_AO_PORT        GPIOB
#define LM393_AO_PIN         GPIO_Pin_1
#define LM393_ADC_CHANNEL    ADC_Channel_9

/* ---- API ---- */
void     Lm393_Init(void);
uint8_t  Lm393_ReadDigital(void);            /* 0 = bright, 1 = dark */
uint8_t  Lm393_ReadAnalog(uint16_t *adc_val); /* 0=OK, 1=timeout; adc_val 0~4095 or 0xFFFF */

#endif /* __LM393_H */
