#pragma once

#include "Arduino.h"
#include "driver/spi_master.h"
#include "stdint.h"

#define TFT_X_OFFSET  2
#define TFT_Y_OFFSET  1

#define TFT_MADCTL    0x36
#define TFT_MAD_MY    0x80
#define TFT_MAD_MX    0x40
#define TFT_MAD_MV    0x20
#define TFT_MAD_ML    0x10
#define TFT_MAD_BGR   0x08
#define TFT_MAD_MH    0x04
#define TFT_MAD_RGB   0x00
// cmd
#define TFT_SLPIN     0x10
#define TFT_INVOFF    0x20
#define TFT_INVON     0x21

#define LCD_RES_Clr() digitalWrite(_lcd_setting->res_gpio_num, LOW);
#define LCD_CS_Clr()  digitalWrite(_lcd_setting->cs_gpio_num, LOW);
#define LCD_DC_Clr()  digitalWrite(_lcd_setting->dc_gpio_num, LOW);

#define LCD_RES_Set() digitalWrite(_lcd_setting->res_gpio_num, HIGH);
#define LCD_CS_Set()  digitalWrite(_lcd_setting->cs_gpio_num, HIGH);
#define LCD_DC_Set()  digitalWrite(_lcd_setting->dc_gpio_num, HIGH);

#define TFT_SPI_MODE  SPI_MODE0
#define LCD_WIDTH     194
#define LCD_HEIGHT    368

typedef struct {
  uint8_t clk_gpio_num;
  uint8_t d0_gpio_num;
  uint8_t d1_gpio_num;
  uint8_t d2_gpio_num;
  uint8_t d3_gpio_num;
  uint8_t cs_gpio_num;
  uint8_t dc_gpio_num;
  uint8_t res_gpio_num;
  uint8_t te_gpio_num;
  uint32_t frequency; // Spi communication record, the higher the value, the
                      // faster the refresh
} lcd_settings_t;

typedef struct {
  uint32_t addr;
  uint8_t param[4];
  uint32_t len;
} lcd_cmd_t;

void lcd_send_data8(uint8_t dat);
void lcd_send_data16(uint16_t dat);
void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void lcd_init(lcd_settings_t *config);
// void lcd_setRotation(uint8_t r);//This screen does not support hardware rotation
void lcd_push_colors(uint16_t x, uint16_t y, uint16_t width, uint16_t hight, uint16_t *data);
void lcd_push_colors_soft_rotation(uint16_t x, uint16_t y, uint16_t width, uint16_t hight, uint16_t *data, uint8_t r);
void DispColorQSPI(uint8_t data1, uint8_t data2);
void lcd_sleep(void);
void lcd_set_brightness(uint8_t value);