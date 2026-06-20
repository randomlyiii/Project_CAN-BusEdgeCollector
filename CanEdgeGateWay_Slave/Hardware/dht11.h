#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f10x.h"

// 引脚宏定义
#define DHT11_DATA_GPIO_PORT GPIOB
#define DHT11_DATA_GPIO_PIN GPIO_Pin_11
#define DHT11_DATA_GPIO_RCC RCC_APB2Periph_GPIOB

// 电平操作
#define DHT11_DATA_HIGH() GPIO_SetBits(DHT11_DATA_GPIO_PORT, DHT11_DATA_GPIO_PIN)
#define DHT11_DATA_LOW() GPIO_ResetBits(DHT11_DATA_GPIO_PORT, DHT11_DATA_GPIO_PIN)
#define DHT11_DATA_READ() GPIO_ReadInputDataBit(DHT11_DATA_GPIO_PORT, DHT11_DATA_GPIO_PIN)

// 引脚模式切换
#define DHT11_DATA_MODE_OUT()                              \
    do                                                     \
    {                                                      \
        GPIO_InitTypeDef GPIO_InitStruct;                  \
        GPIO_InitStruct.GPIO_Pin = DHT11_DATA_GPIO_PIN;    \
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;      \
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;     \
        GPIO_Init(DHT11_DATA_GPIO_PORT, &GPIO_InitStruct); \
    } while (0)

#define DHT11_DATA_MODE_IN()                               \
    do                                                     \
    {                                                      \
        GPIO_InitTypeDef GPIO_InitStruct;                  \
        GPIO_InitStruct.GPIO_Pin = DHT11_DATA_GPIO_PIN;    \
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
        GPIO_Init(DHT11_DATA_GPIO_PORT, &GPIO_InitStruct); \
    } while (0)

// 函数声明
void DHT11_GPIO_Init(void);
uint8_t DHT11_Read_Data(uint8_t *humi_int, uint8_t *humi_dec, uint8_t *temp_int, uint8_t *temp_dec);

#endif
