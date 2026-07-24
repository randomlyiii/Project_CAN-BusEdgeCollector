/**
 * LM393 Photoresistor Module Driver (DO + AO)
 *
 * PB0 = DO (digital comparator output)
 * PB1 = AO (analog, ADC1_IN9)
 *
 * The module uses an LM393 comparator: when light exceeds the
 * potentiometer-set threshold, DO goes LOW (open-drain output).
 * AO outputs 0~3.3V proportional to light intensity — brighter
 * light → lower photoresistor resistance → higher AO voltage.
 */

#include "lm393.h"
#include "delay.h"

/* ==================== GPIO + ADC Init ==================== */

void Lm393_Init(void)
{
    GPIO_InitTypeDef  gpio;
    ADC_InitTypeDef   adc;

    /* ---- GPIO clocks ---- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE);

    /* ---- PB0: DO (digital input, pull-up) ---- */
    gpio.GPIO_Pin   = LM393_DO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;   /* DO is open-drain, pull HIGH when dark */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LM393_DO_PORT, &gpio);

    /* ---- PB1: AO (analog input) ---- */
    gpio.GPIO_Pin   = LM393_AO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LM393_AO_PORT, &gpio);

    /* ---- ADC1 init ---- */
    ADC_DeInit(ADC1);

    /* ADC clock: PCLK2=72MHz / 6 = 12MHz (max 14MHz) */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;          /* single channel */
    adc.ADC_ContinuousConvMode = DISABLE;          /* software trigger each read */
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &adc);

    /* Calibrate ADC */
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) { }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1)) { }
    ADC_Cmd(ADC1, DISABLE);
}

/* ==================== Digital Read (DO) ==================== */

uint8_t Lm393_ReadDigital(void)
{
    /* DO = LOW → light exceeds threshold (bright)
       DO = HIGH → below threshold (dark)
       Pull-up is internal, so no external resistor needed. */
    return (GPIO_ReadInputDataBit(LM393_DO_PORT, LM393_DO_PIN) == Bit_RESET) ? 0 : 1;
}

/* ==================== Analog Read (AO) ==================== */

uint16_t Lm393_ReadAnalog(void)
{
    uint16_t adc_val;

    /* Configure channel, sample, convert, read */
    ADC_RegularChannelConfig(ADC1, LM393_ADC_CHANNEL, 1, ADC_SampleTime_55Cycles5);

    ADC_Cmd(ADC1, ENABLE);

    /* Single conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) { }

    adc_val = ADC_GetConversionValue(ADC1);
    ADC_Cmd(ADC1, DISABLE);

    return adc_val;
}
