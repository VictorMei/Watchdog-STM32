/**
  ******************************************************************************
  * @file    lcd_i2c.c
  * @brief   Minimal HD44780 (16x2) driver over a PCF8574 I2C backpack.
  ******************************************************************************
  */
#include "lcd_i2c.h"
#include <string.h>

/* Common PCF8574 backpack bit mapping: P0=RS, P1=RW, P2=EN, P3=backlight,
   P4..P7 = D4..D7 (the 4-bit data nibble). RW is always driven low (write). */
#define LCD_BIT_RS         0x01U
#define LCD_BIT_RW         0x02U
#define LCD_BIT_EN         0x04U
#define LCD_BIT_BACKLIGHT  0x08U

#define LCD_IO_TIMEOUT_MS  5U   /* short: a missing/stuck LCD must never hang the robot */

static I2C_HandleTypeDef *lcd_hi2c;
static uint8_t lcd_addr;
static uint8_t lcd_present;

static HAL_StatusTypeDef lcd_write_raw(uint8_t value)
{
  return HAL_I2C_Master_Transmit(lcd_hi2c, (uint16_t)(lcd_addr << 1), &value, 1, LCD_IO_TIMEOUT_MS);
}

/* EN must be high >= 450ns; the I2C transaction to set it high already takes
   far longer than that at 100 kHz, so no extra delay is needed before
   dropping it again. */
static void lcd_pulse_enable(uint8_t data)
{
  lcd_write_raw((uint8_t)(data | LCD_BIT_EN));
  lcd_write_raw((uint8_t)(data & (uint8_t)~LCD_BIT_EN));
}

static void lcd_write4(uint8_t nibble_high)
{
  uint8_t out = (uint8_t)(nibble_high | LCD_BIT_BACKLIGHT);
  lcd_write_raw(out);
  lcd_pulse_enable(out);
}

/* mode: 0 = command (RS=0), 1 = data (RS=1). Sends the high nibble then the
   low nibble, per the HD44780 4-bit interface. */
static void lcd_send(uint8_t value, uint8_t mode)
{
  uint8_t rs = mode ? LCD_BIT_RS : 0U;

  lcd_write4((uint8_t)((value & 0xF0U) | rs));
  lcd_write4((uint8_t)(((uint8_t)(value << 4) & 0xF0U) | rs));
}

static void lcd_command(uint8_t cmd)
{
  lcd_send(cmd, 0U);
}

static void lcd_data(uint8_t data)
{
  lcd_send(data, 1U);
}

/* HD44780 4-bit init sequence. Boot-time only - the millisecond delays here
   are the datasheet-mandated power-on/settle timing and must never run
   again once tracking is live. */
static void lcd_run_init_sequence(void)
{
  HAL_Delay(50U);   /* > 40 ms after Vcc rises to 4.5V, per datasheet */

  lcd_write4(0x30U);
  HAL_Delay(5U);
  lcd_write4(0x30U);
  HAL_Delay(1U);
  lcd_write4(0x30U);
  HAL_Delay(1U);
  lcd_write4(0x20U);  /* switch to 4-bit mode */

  lcd_command(0x28U);  /* function set: 4-bit, 2 line, 5x8 font */
  lcd_command(0x08U);  /* display off */
  lcd_command(0x01U);  /* clear display */
  HAL_Delay(2U);        /* clear/home needs ~1.52 ms */
  lcd_command(0x06U);  /* entry mode: increment, no shift */
  lcd_command(0x0CU);  /* display on, cursor off, blink off */
}

static uint8_t lcd_probe_one(uint8_t addr7)
{
  return (HAL_I2C_IsDeviceReady(lcd_hi2c, (uint16_t)(addr7 << 1), 2, 5U) == HAL_OK) ? 1U : 0U;
}

static uint8_t lcd_probe_address(void)
{
  static const uint8_t common[] = { 0x27U, 0x3FU };
  uint8_t i;
  uint16_t addr;

  for (i = 0U; i < (uint8_t)(sizeof(common) / sizeof(common[0])); i++)
  {
    if (lcd_probe_one(common[i]))
    {
      return common[i];
    }
  }

  /* Fall back to a full normal 7-bit range scan. Boot-time only - this is
     never repeated once the robot is running/tracking. */
  for (addr = 0x08U; addr <= 0x77U; addr++)
  {
    if (lcd_probe_one((uint8_t)addr))
    {
      return (uint8_t)addr;
    }
  }

  return 0U;  /* 0 is not a valid 7-bit I2C device address: "not found" */
}

uint8_t lcd_i2c_probe_and_init(I2C_HandleTypeDef *hi2c)
{
  lcd_hi2c = hi2c;
  lcd_addr = lcd_probe_address();
  lcd_present = (lcd_addr != 0U) ? 1U : 0U;

  if (lcd_present)
  {
    lcd_run_init_sequence();
  }

  return lcd_present;
}

uint8_t lcd_i2c_is_present(void)
{
  return lcd_present;
}

uint8_t lcd_i2c_detected_address(void)
{
  return lcd_addr;
}

void lcd_i2c_write_line(uint8_t row, const char *text)
{
  uint8_t col;
  uint8_t len;

  if (!lcd_present || (row >= LCD_I2C_ROWS))
  {
    return;
  }

  len = (uint8_t)strnlen(text, LCD_I2C_COLS);

  /* DDRAM row base addresses for a standard 16x2 HD44780: 0x00 / 0x40. */
  lcd_command((uint8_t)(0x80U | ((row == 0U) ? 0x00U : 0x40U)));

  for (col = 0U; col < LCD_I2C_COLS; col++)
  {
    lcd_data((col < len) ? (uint8_t)text[col] : (uint8_t)' ');
  }
}
