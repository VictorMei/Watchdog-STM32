/**
  ******************************************************************************
  * @file    lcd_i2c.h
  * @brief   Minimal HD44780 (16x2) driver over a PCF8574 I2C backpack.
  *
  * Generic display driver only - it knows nothing about the vision/tracking
  * protocol. The caller decides what text to show and calls
  * lcd_i2c_write_line() only when that text actually changes.
  ******************************************************************************
  */
#ifndef LCD_I2C_H
#define LCD_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define LCD_I2C_COLS 16U
#define LCD_I2C_ROWS 2U

/* One-time boot probe: tries the common PCF8574 backpack addresses (0x27,
   0x3F) first, then falls back to a full 7-bit bus scan (0x08..0x77) if
   neither answers. Runs the HD44780 4-bit init sequence (with its mandatory
   power-on/settle delays) only if a device is found. Never call this again
   after boot - it is not safe to run while tracking is live.
   Returns 1 and leaves the driver ready to use if a display was found and
   initialized, 0 otherwise. On 0, every other lcd_i2c_* call is a safe no-op,
   so the rest of the firmware can run unmodified with no LCD attached. */
uint8_t lcd_i2c_probe_and_init(I2C_HandleTypeDef *hi2c);

/* True once lcd_i2c_probe_and_init() found and initialized a display. */
uint8_t lcd_i2c_is_present(void);

/* The 7-bit address actually detected. Only meaningful if lcd_i2c_is_present(). */
uint8_t lcd_i2c_detected_address(void);

/* Writes exactly LCD_I2C_COLS characters to one row (row is 0 or 1):
   truncates a longer string and space-pads a shorter one, so leftover
   characters from whatever was previously shown on that row are always
   overwritten without needing a separate (slower) clear-display command.
   No-op if no LCD was detected. */
void lcd_i2c_write_line(uint8_t row, const char *text);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H */
