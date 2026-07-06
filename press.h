/* press.h
 *
 * FSR pressure sensor driver. Bare-metal ADC1 access on PA3 (Nucleo-144 A0,
 * channel 3). Reads raw ADC, derives voltage-divider resistance, and derives
 * conductance in microsiemens (uS) as uint16_t for CAN/UART payload use.
 */

#ifndef PRESS_H
#define PRESS_H

#include <stdint.h>
#include "main.h"

typedef struct {
    uint32_t raw;             /* raw ADC1 conversion value */
    float voltage;            /* volts across the divider tap */
    float resistance;         /* estimated FSR resistance, ohm (-1.0f if invalid) */
    uint16_t conductance_uS;  /* estimated FSR conductance, microsiemens */
} Press_Data_t;

/* Enables GPIOA/ADC1 clocks, configures PA3 as analog input, and configures
 * ADC1 channel 3 (single conversion, software trigger). Call once at
 * startup before Press_Read(). */
void Press_Init(void);

/* Performs one blocking ADC conversion and fills *data with the raw value,
 * voltage, resistance, and conductance. Conductance is only computed once
 * two consecutive valid resistance readings exist (guards against a single
 * noisy/invalid sample). */
void Press_Read(Press_Data_t *data);

#endif /* PRESS_H */
