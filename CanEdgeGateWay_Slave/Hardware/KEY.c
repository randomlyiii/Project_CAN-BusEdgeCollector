#include "KEY.h"
#include "delay.h"

void KEY_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(KEY1_GPIO_RCC | KEY2_GPIO_RCC, ENABLE);

    gpio.GPIO_Pin   = KEY1_GPIO_PIN | KEY2_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

/* 按下即触发, 松开200ms后才允许再次触发 */
uint8_t KEY_Scan(void)
{
    static uint8_t  k1_block = 0;
    static uint8_t  k2_block = 0;
    static uint32_t k1_tick  = 0;
    static uint32_t k2_tick  = 0;

    uint32_t now = Delay_GetTick();
    uint8_t  k1  = GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_GPIO_PIN);
    uint8_t  k2  = GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_GPIO_PIN);
    uint8_t  evt = KEY_EVT_NONE;

    if (k1 == 0 && !k1_block) {
        evt = KEY_EVT_KEY1_SHRT;
        k1_block = 1;
        k1_tick  = now;
    }
    if (k1 == 1 && k1_block && (now - k1_tick > 200)) {
        k1_block = 0;
    }

    if (k2 == 0 && !k2_block) {
        if (evt == KEY_EVT_NONE) evt = KEY_EVT_KEY2_SHRT;
        k2_block = 1;
        k2_tick  = now;
    }
    if (k2 == 1 && k2_block && (now - k2_tick > 200)) {
        k2_block = 0;
    }

    return evt;
}
