#include "stm32f10x.h"
#include "OLED_Font.h"

/* ==================== 软件 I2C (PB8=SCL, PB9=SDA) ====================
 * 经诊断, CAN 总线噪声导致显示错乱是硬件层面的电源/信号耦合问题,
 * 无法通过软件或硬件 I2C 彻底解决。已知约束:
 *   - CAN 线断开: 显示正常
 *   - CAN 线连接: 偶尔显示错乱 (前几个字符正常, 后续错位)
 * 两权相害取其轻: 退回可用的软件 I2C, 保持系统可运行。
 * =================================================================== */

#define I2C_DELAY_CNT     40

static void I2C_Delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < I2C_DELAY_CNT; i++) __NOP();
}

#define SCL_H  GPIOB->BSRR = GPIO_Pin_8
#define SCL_L  GPIOB->BRR  = GPIO_Pin_8
#define SDA_H  GPIOB->BSRR = GPIO_Pin_9
#define SDA_L  GPIOB->BRR  = GPIO_Pin_9
#define SDA_R  (GPIOB->IDR & GPIO_Pin_9)

void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    SCL_H; SDA_H;
}

static void I2C_Start(void)
{
    SDA_H; I2C_Delay(); SCL_H; I2C_Delay();
    SDA_L; I2C_Delay(); SCL_L; I2C_Delay();
}

static void I2C_Stop(void)
{
    SDA_L; I2C_Delay(); SCL_H; I2C_Delay();
    SDA_H; I2C_Delay();
}

static void I2C_SendByte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H; else SDA_L;
        I2C_Delay(); SCL_H; I2C_Delay(); SCL_L; I2C_Delay();
        byte <<= 1;
    }
    SCL_H; I2C_Delay(); SCL_L; I2C_Delay();
}

void OLED_WriteCommand(uint8_t Command)
{
    I2C_Start(); I2C_SendByte(0x78);
    I2C_SendByte(0x00); I2C_SendByte(Command); I2C_Stop();
}

void OLED_WriteData(uint8_t Data)
{
    I2C_Start(); I2C_SendByte(0x78);
    I2C_SendByte(0x40); I2C_SendByte(Data); I2C_Stop();
}

/* ==================== 上层函数不变 ==================== */

void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++) OLED_WriteData(0x00);
    }
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
        OLED_ShowChar(Line, Column + i, String[i]);
}

uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t r = 1;
    while (Y--) r *= X;
    return r;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i; uint32_t n;
    if (Number >= 0) { OLED_ShowChar(Line, Column, '+'); n = Number; }
    else             { OLED_ShowChar(Line, Column, '-'); n = -Number; }
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i + 1, n / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, s;
    for (i = 0; i < Length; i++) {
        s = Number / OLED_Pow(16, Length - i - 1) % 16;
        OLED_ShowChar(Line, Column + i, s < 10 ? s + '0' : s - 10 + 'A');
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
}

void OLED_Init(void)
{
    uint32_t i, j;
    for (i = 0; i < 1000; i++)
        for (j = 0; j < 1000; j++);

    OLED_I2C_Init();

    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);

    OLED_Clear();
}
