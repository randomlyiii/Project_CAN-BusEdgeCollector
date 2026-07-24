#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include <stdint.h>

#define KEY1_GPIO_PORT      GPIOA
#define KEY1_GPIO_PIN       GPIO_Pin_0
#define KEY1_GPIO_RCC       RCC_APB2Periph_GPIOA
#define KEY2_GPIO_PORT      GPIOA
#define KEY2_GPIO_PIN       GPIO_Pin_1
#define KEY2_GPIO_RCC       RCC_APB2Periph_GPIOA

#define KEY_EVT_NONE        0
#define KEY_EVT_KEY1_SHRT   1
#define KEY_EVT_KEY2_SHRT   2
#define KEY_EVT_KEY1_LONG   3
#define KEY_EVT_KEY2_LONG   4

void     KEY_Init(void);
uint8_t  KEY_Scan(void);

#endif
