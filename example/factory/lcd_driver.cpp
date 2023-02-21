#include "lcd_driver.h"
#include "stdint.h"

static uint8_t horizontal;
static spi_device_handle_t spi;
static lcd_settings_t *_lcd_setting;
static uint16_t *soft_rota_p = nullptr;

#define USE_SORF_QSPI  0
#define SEND_BUF_SIZE  (16384) //(LCD_WIDTH * LCD_HEIGHT + 8) / 10
#define PARALLEL_LINES 36

static lcd_cmd_t sh8501_cmd[] = {
    {0xc0, {0x5a, 0x5a}, 0x02},
    {0x2a, {0x00, 0x00, 0x00, 0xc1}, 0x04},
    {0x2b, {0x00, 0x00, 0x01, 0x6f}, 0x04},
    {0x44, {0x01, 0x6f}, 0x02},
    {0x35, {0x00}, 0x01},
    // {0x44, {0x00, 0x00}, 0x02},
    // {0x35, {0x00}, 0x01},

    {0x3a, {0x55}, 0x01},
    {0x53, {0x20}, 0x01},
    {0x11, {0x00}, 0x80},
    {0x29, {0x00}, 0x80},
    {0x2C, {0x00}, 0x80},
};

static void lcd_gpio_init() {
  pinMode(_lcd_setting->cs_gpio_num, OUTPUT);
  pinMode(_lcd_setting->res_gpio_num, OUTPUT);
  pinMode(_lcd_setting->te_gpio_num, INPUT);

  digitalWrite(_lcd_setting->cs_gpio_num, HIGH);
  digitalWrite(_lcd_setting->res_gpio_num, HIGH);
}

static void lcd_send_cmd(uint32_t cmd, uint8_t *dat, uint32_t len) {
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.flags = (SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR);
  t.cmd = 0x02;
  t.addr = cmd << 8;
  if (len != 0) {
    t.tx_buffer = dat;
    t.length = 8 * len;
  } else {
    t.tx_buffer = NULL;
    t.length = 0;
  }
  LCD_CS_Clr();
  spi_device_polling_transmit(spi, &t);
  LCD_CS_Set();
}

void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  lcd_cmd_t t[3] = {
      {0x2a, {x1 >> 8, x1, x2 >> 8, x2}, 0x04},
      {0x2b, {y1 >> 8, y1, y2 >> 8, y2}, 0x04},
      {0x2c, {0x00}, 0x00},
  };

  for (uint32_t i = 0; i < 3; i++) {
    lcd_send_cmd(t[i].addr, t[i].param, t[i].len);
  }
}

void lcd_push_colors(uint16_t x, uint16_t y, uint16_t width, uint16_t hight, uint16_t *data) {

  bool first_send = 1;
  size_t len = width * hight;
  uint16_t *p = (uint16_t *)data;

  lcd_address_set(x, y, x + width - 1, y + hight - 1);
  LCD_CS_Clr();
  do {
    size_t chunk_size = len;
    spi_transaction_ext_t t = {0};
    memset(&t, 0, sizeof(t));
    if (first_send) {
      t.base.flags = SPI_TRANS_MODE_QIO /* | SPI_TRANS_MODE_DIOQIO_ADDR */;
      t.base.cmd = 0x32 /* 0x12 */;
      t.base.addr = 0x002C00;
      first_send = 0;
    } else {
      t.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
      t.command_bits = 0;
      t.address_bits = 0;
      t.dummy_bits = 0;
    }
    if (chunk_size > SEND_BUF_SIZE) {
      chunk_size = SEND_BUF_SIZE;
    }
    t.base.tx_buffer = p;
    t.base.length = chunk_size * 16;

    // spi_device_queue_trans(spi, (spi_transaction_t *)&t, portMAX_DELAY);
    spi_device_polling_transmit(spi, (spi_transaction_t *)&t);
    len -= chunk_size;
    p += chunk_size;
  } while (len > 0);
  LCD_CS_Set();
}

void lcd_push_colors_soft_rotation(uint16_t x, uint16_t y, uint16_t width, uint16_t hight, uint16_t *data, uint8_t r) {
  if (r == 0) {
    lcd_push_colors(x, y, width, hight, data);
  } else if (r == 1) {
    uint16_t _x = LCD_WIDTH - (y + hight);
    uint16_t _y = x;
    uint16_t _h = width;
    uint16_t _w = hight;
    uint16_t *p = data;
    uint32_t cum = 0;

    for (uint16_t j = 0; j < width; j++) {
      for (uint16_t i = 0; i < hight; i++) {
        soft_rota_p[cum] = ((uint16_t)p[width * (hight - i - 1) + j]);
        cum++;
      }
    }
    lcd_push_colors(_x, _y, _w, _h, soft_rota_p);
  } else if (r == 2) {
    uint16_t _x = LCD_WIDTH - x - width;
    uint16_t _y = LCD_HEIGHT - y - hight;
    uint16_t _h = hight;
    uint16_t _w = width;
    uint16_t *p = data;
    uint32_t cum = width * hight;
    for (uint32_t i = 0; i < cum; i++) {
      soft_rota_p[i] = p[cum - i];
    }
    lcd_push_colors(_x, _y, _w, _h, soft_rota_p);
  } else {
    uint16_t _x = y;
    uint16_t _y = LCD_HEIGHT - (x + width);
    uint16_t _h = width;
    uint16_t _w = hight;
    uint16_t *p = data;
    uint32_t cum = 0;

    for (uint16_t j = 0; j < width; j++) {
      for (uint16_t i = 0; i < hight; i++) {
        soft_rota_p[cum] = (uint16_t)p[width * (LCD_WIDTH - (hight - i - 1)) + j];
        cum++;
      }
    }
    lcd_push_colors(_x, _y, _w, _h, soft_rota_p);
  }
}

void lcd_setRotation(uint8_t r) {
  horizontal = r % 4;
  lcd_cmd_t t = {0x36, {0x00}, 0x01};
  // lcd_send_cmd(TFT_MADCTL);
  switch (horizontal) {
  case 0: // Portrait
    // lcd_send_data8(TFT_MAD_BGR);
    t.param[0] = TFT_MAD_RGB;
    break;
  case 1:                                  // Landscape (Portrait + 90)
    t.param[0] = TFT_MAD_MX | TFT_MAD_RGB; // 镜像
    // lcd_send_data8(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_BGR);
    break;
  case 2:                    // Inverter portrait
    t.param[0] = TFT_MAD_MX; // TFT_MAD_MY | TFT_MAD_RGB;
    // lcd_send_data8(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_BGR);
    break;
  case 3: // Inverted landscape
    t.param[0] = TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_RGB;
    // lcd_send_data8(TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_BGR);
    break;
  }

  lcd_send_cmd(t.addr, t.param, t.len);
}

void lcd_sleep(void) {
  lcd_send_cmd(TFT_SLPIN, NULL, 0);
  delay(100);
}

void lcd_init(lcd_settings_t *config) {
  _lcd_setting = config;
  lcd_gpio_init();
  esp_err_t ret;
  horizontal = 0;

  spi_bus_config_t buscfg = {
      .data0_io_num = config->d0_gpio_num,
      .data1_io_num = config->d1_gpio_num,
      .sclk_io_num = config->clk_gpio_num,
      .data2_io_num = config->d2_gpio_num,
      .data3_io_num = config->d3_gpio_num,
      .max_transfer_sz = (SEND_BUF_SIZE * 16) + 8,
      .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
  };

  spi_device_interface_config_t devcfg = {
      .command_bits = 8,
      .address_bits = 24,
      .mode = TFT_SPI_MODE,
      .clock_speed_hz = config->frequency,
      .spics_io_num = -1,
      // .spics_io_num = config->cs_gpio_num,
      .flags = SPI_DEVICE_HALFDUPLEX,
      .queue_size = 17,
  };
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  LCD_RES_Clr(); // reset
  delay(200);
  LCD_RES_Set();
  delay(200);

  lcd_cmd_t *t = sh8501_cmd;
  for (uint32_t i = 0; i < (sizeof(sh8501_cmd) / sizeof(lcd_cmd_t)); i++) {
    lcd_send_cmd(t[i].addr, t[i].param, t[i].len & 0x7f);
    if (t[i].len & 0x80) {
      delay(120);
    }
  }

  soft_rota_p = (uint16_t *)heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
                                             /* MALLOC_CAP_INTERNAL */ MALLOC_CAP_SPIRAM);
}

void lcd_set_brightness(uint8_t value) {
  lcd_cmd_t t = {0x51, {value}, 0x01};
  lcd_send_cmd(t.addr, t.param, t.len);
}