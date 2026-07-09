#ifndef LED_M_H
#define LED_M_H

#include "stm32f4xx.h"
#include <stdint.h>

/* MAX7219 register addresses */
#define LED_M_REG_NOOP        0x00
#define LED_M_REG_DECODEMODE  0x09
#define LED_M_REG_INTENSITY   0x0A
#define LED_M_REG_SCANLIMIT   0x0B
#define LED_M_REG_SHUTDOWN    0x0C
#define LED_M_REG_DISPLAYTEST 0x0F
/* Digit0-7 registers = row 1-8 for dot matrix mode */
#define LED_M_REG_DIGIT0      0x01

/* One LED matrix instance: raw SPI peripheral + its own CS pin.
 * Lets the same driver drive multiple matrices on different SPI buses,
 * with no HAL dependency - direct register access only. */
typedef struct
{
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;   /* e.g. GPIO_PIN_14 style bitmask, (1U << pin_number) */
} LED_M_Handle_t;

/* Public API - every call takes the instance handle */
void LED_M_Init(LED_M_Handle_t *led);
void LED_M_WriteReg(LED_M_Handle_t *led, uint8_t reg, uint8_t data);
void LED_M_ClearDisplay(LED_M_Handle_t *led);
void LED_M_DisplayPattern(LED_M_Handle_t *led, const uint8_t *pattern8);
void LED_M_SetIntensity(LED_M_Handle_t *led, uint8_t intensity); /* 0x00 - 0x0F */
void LED_M_ShutdownMode(LED_M_Handle_t *led, uint8_t on); /* 1 = normal operation, 0 = shutdown */

/* Predefined patterns (8 bytes, one per row) */
extern const uint8_t LED_M_PATTERN_WARNING_4[8]; /* sleep_flag == 4: exclamation mark */
extern const uint8_t LED_M_PATTERN_WARNING_5_L[8]; /* sleep_flag >= 5: left arrow, most urgent */
extern const uint8_t LED_M_PATTERN_WARNING_5_R[8]; /* sleep_flag >= 5: right arrow, most urgent */
extern const uint8_t LED_M_PATTERN_ALL_ON[8];
extern const uint8_t LED_M_PATTERN_ALL_OFF[8];

#endif /* LED_M_H */
