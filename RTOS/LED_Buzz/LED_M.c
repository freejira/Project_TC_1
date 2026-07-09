#include "LED_M.h"

/* Level 4 (medium warning) - exclamation mark, MSB = leftmost column */
const uint8_t LED_M_PATTERN_WARNING_4[8] = {
    0b00111100,
    0b00111100,
    0b11111111,
    0b11111111,
    0b11111111,
    0b11111111,
    0b00111100,
    0b00111100
};

/* Level 5 (most urgent) - left-pointing arrow, MSB = leftmost column */
const uint8_t LED_M_PATTERN_WARNING_5_L[8] = {
    0b00001000,
    0b00011000,
    0b00111000,
    0b01111111,
    0b01111111,
    0b00111000,
    0b00011000,
    0b00001000
};

/* Level 5 (most urgent) - right-pointing arrow, mirror of _L */
const uint8_t LED_M_PATTERN_WARNING_5_R[8] = {
    0b00010000,
    0b00011000,
    0b00011100,
    0b11111110,
    0b11111110,
    0b00011100,
    0b00011000,
    0b00010000
};

const uint8_t LED_M_PATTERN_ALL_ON[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

const uint8_t LED_M_PATTERN_ALL_OFF[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* SR register bit positions (STM32F4 SPI) */
#define SPI_SR_RXNE   (0x01 << 0)
#define SPI_SR_TXE    (0x01 << 1)
#define SPI_SR_BSY    (0x01 << 7)

static void LED_M_CS_Low(LED_M_Handle_t *led)
{
    led->cs_port->BSRR = (uint32_t)(led->cs_pin) << 16; /* reset bit via BSRR upper half */
}

static void LED_M_CS_High(LED_M_Handle_t *led)
{
    led->cs_port->BSRR = led->cs_pin; /* set bit via BSRR lower half */
}

/* Blocking 8-bit SPI transmit, register-level, no HAL */
static void SPI_Transmit8(SPI_TypeDef *spi, uint8_t data)
{
    while (!(spi->SR & SPI_SR_TXE)) { }          /* wait until TX buffer empty */
    *(volatile uint8_t *)&spi->DR = data;        /* write only the low byte (8-bit frame) */
    while (!(spi->SR & SPI_SR_RXNE)) { }         /* wait until byte fully shifted out */
    (void)spi->DR;                               /* dummy read, clears RXNE */
    while (spi->SR & SPI_SR_BSY) { }             /* wait until line truly idle before CS change */
}

void LED_M_WriteReg(LED_M_Handle_t *led, uint8_t reg, uint8_t data)
{
    LED_M_CS_Low(led);
    SPI_Transmit8(led->spi, reg);
    SPI_Transmit8(led->spi, data);
    LED_M_CS_High(led);
}

void LED_M_Init(LED_M_Handle_t *led)
{
    /* CS idles high */
    LED_M_CS_High(led);

    LED_M_WriteReg(led, LED_M_REG_DISPLAYTEST, 0x00); /* normal mode, not test */
    LED_M_WriteReg(led, LED_M_REG_DECODEMODE, 0x00);  /* no BCD decode, raw dot matrix */
    LED_M_WriteReg(led, LED_M_REG_SCANLIMIT, 0x07);   /* scan all 8 digits/rows */
    LED_M_WriteReg(led, LED_M_REG_INTENSITY, 0x08);   /* mid brightness, adjust as needed */
    LED_M_WriteReg(led, LED_M_REG_SHUTDOWN, 0x01);    /* wake up from shutdown */

    LED_M_ClearDisplay(led);
}

void LED_M_ClearDisplay(LED_M_Handle_t *led)
{
    LED_M_DisplayPattern(led, LED_M_PATTERN_ALL_OFF);
}

void LED_M_DisplayPattern(LED_M_Handle_t *led, const uint8_t *pattern8)
{
    for (uint8_t row = 0; row < 8; row++)
    {
        LED_M_WriteReg(led, LED_M_REG_DIGIT0 + row, pattern8[row]);
    }
}

void LED_M_SetIntensity(LED_M_Handle_t *led, uint8_t intensity)
{
    if (intensity > 0x0F)
    {
        intensity = 0x0F;
    }
    LED_M_WriteReg(led, LED_M_REG_INTENSITY, intensity);
}

void LED_M_ShutdownMode(LED_M_Handle_t *led, uint8_t on)
{
    LED_M_WriteReg(led, LED_M_REG_SHUTDOWN, on ? 0x01 : 0x00);
}
