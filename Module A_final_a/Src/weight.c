/*
 * weight.c
 *
 *  Created on: 2026. 7. 7.
 *      Author: 한국전파진흥협회
 */

#include "weight.h"


void Weight_Init(){


  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIOF->MODER &= ~(3U << (12 * 2));         // clear PF12
  GPIOF->MODER |=  (1U << (12 * 2));         // PF12 = output (01)

  GPIOF->MODER &= ~(3U << (13 * 2));         // clear PF13 = input (00)

}

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

int32_t Data_Read(void) {
    uint32_t value = 0;
    uint32_t guard = 0;
    while (DT_READ() == 1) {
        delay_us(1);
        if (++guard > 200000) return 0;
    }
    for (uint8_t i = 0; i < 24; i++) {
        SCK_HIGH();
        delay_us(1);
        value <<= 1;
        if (DT_READ() == 1U) value |= 1;
        SCK_LOW();
        delay_us(1);
    }
    SCK_HIGH();
    delay_us(1);
    SCK_LOW();
    delay_us(1);
    if (value & 0x800000) value |= 0xFF000000;
    return (int32_t)value;
}

int32_t Data_ReadAverage(uint8_t count) {
    int64_t sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += Data_Read();
    return (int32_t)(sum / count);
}
