/**
 * BH1750 Driver — Software I2C on PB8/PB9 (shared with OLED)
 *
 * Uses same GPIO pins as OLED: PB8=SCL, PB9=SDA.
 * Bus is open-drain, so BH1750_Init() must not reconfigure
 * the GPIO mode (already set by OLED_I2C_Init).
 */

#include "bh150.h"
#include "delay.h"

/* GPIO helpers — same pins as OLED */
#define BH1750_SCL(x)    GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define BH1750_SDA(x)    GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))
#define BH1750_SDA_READ  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

/* Software I2C delay (~2us @72MHz) */
static void I2C_Delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 40; i++) __NOP();
}

static void I2C_Start(void)
{
    BH1750_SDA(1);
    I2C_Delay();
    BH1750_SCL(1);
    I2C_Delay();
    BH1750_SDA(0);
    I2C_Delay();
    BH1750_SCL(0);
    I2C_Delay();
}

static void I2C_Stop(void)
{
    BH1750_SDA(0);
    I2C_Delay();
    BH1750_SCL(1);
    I2C_Delay();
    BH1750_SDA(1);
    I2C_Delay();
}

static void I2C_SendByte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        BH1750_SDA(!!(dat & (0x80 >> i)));
        I2C_Delay();
        BH1750_SCL(1);
        I2C_Delay();
        BH1750_SCL(0);
        I2C_Delay();
    }
    /* ACK clock pulse (ignore ACK) */
    BH1750_SCL(1);
    I2C_Delay();
    BH1750_SCL(0);
    I2C_Delay();
}

static uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i, dat = 0;
    BH1750_SDA(1);  /* Release SDA for slave */
    for (i = 0; i < 8; i++) {
        BH1750_SCL(1);
        I2C_Delay();
        dat <<= 1;
        if (BH1750_SDA_READ) dat |= 1;
        BH1750_SCL(0);
        I2C_Delay();
    }
    /* Send ACK/NACK */
    BH1750_SDA(ack ? 0 : 1);
    I2C_Delay();
    BH1750_SCL(1);
    I2C_Delay();
    BH1750_SCL(0);
    I2C_Delay();
    BH1750_SDA(1);
    return dat;
}

/* ==================== Public API ==================== */

void BH1750_Init(void)
{
    /* GPIO is already initialized by OLED_I2C_Init() (PB8/PB9, open-drain).
       Just send the initialization sequence. */
    Delay_ms(10);

    /* Power on */
    I2C_Start();
    I2C_SendByte(BH1750_ADDR_WRITE);
    I2C_SendByte(BH1750_CMD_POWER_ON);
    I2C_Stop();

    Delay_ms(10);

    /* Reset */
    I2C_Start();
    I2C_SendByte(BH1750_ADDR_WRITE);
    I2C_SendByte(BH1750_CMD_RESET);
    I2C_Stop();

    Delay_ms(10);

    /* Set continuous high-resolution mode */
    I2C_Start();
    I2C_SendByte(BH1750_ADDR_WRITE);
    I2C_SendByte(BH1750_CMD_CONT_HR_MODE);
    I2C_Stop();

    Delay_ms(180);  /* Wait for first measurement (120ms + margin) */
}

uint8_t BH1750_Read(uint16_t *lux)
{
    uint8_t hi, lo;

    I2C_Start();
    I2C_SendByte(BH1750_ADDR_READ);
    hi = I2C_ReadByte(1);  /* ACK */
    lo = I2C_ReadByte(0);  /* NACK (last byte) */
    I2C_Stop();

    *lux = ((uint16_t)hi << 8) | lo;
    *lux = (uint16_t)((uint32_t)(*lux) * 5 / 6);  /* Convert to lux: raw / 1.2 */

    return 0;
}
