/**
 * @file ssd1306.h
 * @brief SSD1306 OLED driver for Heltec WiFi LoRa 32 V3
 *
 * Heltec V3 OLED: 128x64, I2C 0x3C
 * SDA=GPIO17, SCL=GPIO18, RST=GPIO21, Vext_Ctrl=GPIO36 (LOW=on)
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

typedef struct ssd1306 ssd1306_t;

esp_err_t ssd1306_init(ssd1306_t **out_dev);
void      ssd1306_clear(ssd1306_t *dev);
void      ssd1306_draw_string(ssd1306_t *dev, uint8_t row, uint8_t col, const char *str);
esp_err_t ssd1306_update(ssd1306_t *dev);
