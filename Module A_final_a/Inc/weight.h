/*
 * weight.h
 *
 *  Created on: 2026. 7. 7.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_WEIGHT_H_
#define INC_WEIGHT_H_

#include <stdint.h>
#include "main.h"

/* PF12 = SCK (output), PF13 = DT (input) */
#define SCK_HIGH()  (GPIOF->BSRR = (1U << 12))
#define SCK_LOW()   (GPIOF->BSRR = (1U << (12 + 16)))
#define DT_READ()   ((GPIOF->IDR >> 13) & 1U)

void Weight_Init();
void DWT_Init(void);
void delay_us(uint32_t);
int32_t Data_Read(void);
int32_t Data_ReadAverage(uint8_t);

#endif /* INC_WEIGHT_H_ */
