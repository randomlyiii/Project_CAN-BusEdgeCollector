#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include <stdint.h>

/* ========== 按键引脚定义 ========== */
#define KEY1_GPIO_PORT      GPIOA
#define KEY1_GPIO_PIN       GPIO_Pin_0
#define KEY1_GPIO_RCC       RCC_APB2Periph_GPIOA

#define KEY2_GPIO_PORT      GPIOA
#define KEY2_GPIO_PIN       GPIO_Pin_1
#define KEY2_GPIO_RCC       RCC_APB2Periph_GPIOA

/* 按键电平：按下为低电平 */
#define KEY1_PRESS()        (GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN) == 0)
#define KEY2_PRESS()        (GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN) == 0)

/* ========== 按键事件 ========== */
#define KEY_EVT_NONE        0
#define KEY_EVT_KEY1_SHRT   1       // KEY1 短按 (故障上报)
#define KEY_EVT_KEY2_SHRT   2       // KEY2 短按 (故障恢复)
#define KEY_EVT_KEY1_LONG   3       // KEY1 长按 (预留)
#define KEY_EVT_KEY2_LONG   4       // KEY2 长按 (预留)

/* ========== 函数声明 ========== */
void     KEY_Init(void);             // GPIO 初始化
uint8_t  KEY_Scan(void);             // 扫描按键 (非阻塞, 返回事件)

#endif
