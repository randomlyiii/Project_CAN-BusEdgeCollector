#include "dht11.h"
#include "delay.h" // 标准库延时函数（毫秒/微秒）

// DHT11 引脚初始化
void DHT11_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(DHT11_DATA_GPIO_RCC, ENABLE);

    GPIO_InitStruct.GPIO_Pin = DHT11_DATA_GPIO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_DATA_GPIO_PORT, &GPIO_InitStruct);

    DHT11_DATA_HIGH(); // 空闲拉高
}

// DHT11 起始信号
static void DHT11_Start(void)
{
    DHT11_DATA_MODE_OUT();
    DHT11_DATA_LOW();
    Delay_ms(20); // 拉低18~20ms
    DHT11_DATA_HIGH();
    Delay_us(30); // 拉高20~40us
    DHT11_DATA_MODE_IN();
}

// 读取一个字节
static uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++)
    {
        while (DHT11_DATA_READ() == 0)
            ;         // 等待拉高
        Delay_us(40); // 延时判断电平
        dat <<= 1;
        if (DHT11_DATA_READ())
            dat++;
        while (DHT11_DATA_READ() == 1)
            ; // 等待拉低
    }
    return dat;
}

// 读取温湿度数据：temp=温度，humi=湿度
uint8_t DHT11_Read_Data(uint8_t *humi_int, uint8_t *humi_dec, uint8_t *temp_int, uint8_t *temp_dec)
{
    uint8_t buf[5];
    DHT11_Start();

    if(DHT11_DATA_READ() == 0)
    {
        while(DHT11_DATA_READ() == 0);
        while(DHT11_DATA_READ() == 1);

        buf[0] = DHT11_Read_Byte(); // 湿度整数
        buf[1] = DHT11_Read_Byte(); // 湿度小数
        buf[2] = DHT11_Read_Byte(); // 温度整数
        buf[3] = DHT11_Read_Byte(); // 温度小数
        buf[4] = DHT11_Read_Byte(); // 校验

        if(buf[0]+buf[1]+buf[2]+buf[3] == buf[4])
        {
            *humi_int = buf[0];
            *humi_dec = buf[1];
            *temp_int = buf[2];
            *temp_dec = buf[3];
            return 0; // 成功
        }
    }
    return 1; // 失败
}
