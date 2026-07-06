/* press.c
 *
 * FSR pressure sensor driver implementation. See press.h.
 */

#include "press.h"

#define ADC_VREF        3.3f
#define ADC_RESOLUTION  4095.0f
#define FIXED_R_OHM     1000.0f   /* Fixed resistor in voltage divider */

/* Previous resistance reading, kept internal to this module so callers
 * don't need to track state between Press_Read() calls. */
static float resistance_ch = -1.0f;

void Press_Init(void)
{
    RCC->AHB1ENR |= (0x01 << 0);            /* GPIOA clock enable */
    GPIOA->MODER |= (0x03 << (3 * 2));      /* PA3 analog mode */

    RCC->APB2ENR |= (0x01 << 8);            /* ADC1 clock enable */
    ADC->CCR |= (0x01 << 16);

    ADC1->SQR3 |= (0x03 << 0);              /* first conversion = channel 3 */
    ADC1->SMPR2 |= (0x00 << (3 * 3));       /* sample time, channel 3 */

    ADC1->CR2 |= (0x01 << 0);               /* ADON */
}

void Press_Read(Press_Data_t *data)
{
    ADC1->CR2 |= (0x01 << 30);              /* SWSTART */
    while (!(ADC1->SR & (0x01 << 1))) { }   /* wait EOC */
    uint32_t raw = ADC1->DR;                /* reading DR clears EOC */

    float voltage = (raw / ADC_RESOLUTION) * ADC_VREF;
    float resistance = -1.0f;

    if (voltage > 0.01f)
    {
        resistance = FIXED_R_OHM * (ADC_VREF - voltage) / voltage;
    }

    uint16_t conductance_uS = 0;

    if (resistance != -1.0f && resistance_ch != -1.0f)
    {
        if (raw > 0 && raw < (uint32_t)ADC_RESOLUTION)
        {
            float g = 1000000.0f * (float)raw
                      / (FIXED_R_OHM * (ADC_RESOLUTION - (float)raw));
            if (g > 65535.0f) g = 65535.0f;  /* clamp to uint16_t range */
            conductance_uS = (uint16_t)g;
        }
    }

    resistance_ch = resistance;

    data->raw = raw;
    data->voltage = voltage;
    data->resistance = resistance;
    data->conductance_uS = conductance_uS;
}
