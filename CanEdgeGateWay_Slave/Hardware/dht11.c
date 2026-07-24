#include "dht11.h"
#include "delay.h"

void DHT11_GPIO_Init(void)
{
    GPIO_InitTypeDef s;
    RCC_APB2PeriphClockCmd(DHT11_DATA_GPIO_RCC, ENABLE);

    s.GPIO_Pin   = DHT11_DATA_GPIO_PIN;
    s.GPIO_Mode  = GPIO_Mode_Out_PP;
    s.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_DATA_GPIO_PORT, &s);
    DHT11_DATA_HIGH();
}

static void DHT11_Start(void)
{
    DHT11_DATA_MODE_OUT();
    DHT11_DATA_LOW();
    Delay_ms(20);
    DHT11_DATA_HIGH();
    Delay_us(30);
    DHT11_DATA_MODE_IN();
}

#define DHT11_TIMEOUT  10000   /* ~100us timeout per poll iteration */

static uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++)
    {
        uint16_t to = DHT11_TIMEOUT;
        while (DHT11_DATA_READ() == 0 && --to);
        if (to == 0) return 0;   /* timeout */

        Delay_us(40);
        dat <<= 1;
        if (DHT11_DATA_READ()) dat++;

        to = DHT11_TIMEOUT;
        while (DHT11_DATA_READ() == 1 && --to);
        if (to == 0) return 0;   /* timeout */
    }
    return dat;
}

uint8_t DHT11_Read_Data(uint8_t *humi_int, uint8_t *humi_dec,
                        uint8_t *temp_int, uint8_t *temp_dec)
{
    uint8_t buf[5];
    uint16_t to;

    DHT11_Start();

    if (DHT11_DATA_READ() == 0)
    {
        to = DHT11_TIMEOUT;
        while (DHT11_DATA_READ() == 0 && --to);
        if (to == 0) return 1;

        to = DHT11_TIMEOUT;
        while (DHT11_DATA_READ() == 1 && --to);
        if (to == 0) return 1;

        buf[0] = DHT11_Read_Byte();
        buf[1] = DHT11_Read_Byte();
        buf[2] = DHT11_Read_Byte();
        buf[3] = DHT11_Read_Byte();
        buf[4] = DHT11_Read_Byte();

        if (buf[0]+buf[1]+buf[2]+buf[3] == buf[4])
        {
            *humi_int = buf[0];
            *humi_dec = buf[1];
            *temp_int = buf[2];
            *temp_dec = buf[3];
            return 0;
        }
    }
    return 1;
}
