#define TOUCH_MODULES_ZTW622
#include "TouchLib.h" //https://github.com/mmMicky/TouchLib

#include "FastLED.h" //https://github.com/FastLED/FastLED

#include "XPowersLib.h" //https://github.com/lewisxhe/XPowersLib

#include "lv_conf.h"
#include "lvgl.h" //https://github.com/lvgl/lvgl

#include "logo_img.h"

#include "lcd_driver.h"

#include "pin_config.h"
#include <Arduino.h>

#include "WiFi.h"
#include "sntp.h"
#include "time.h"

XPowersPMU PMU;

TouchLib touch(Wire, PIN_IIC_SDA, PIN_IIC_SCL, ZTW622_SLAVE2_ADDRESS, PIN_TOUCH_RST);
const char *ssid = "xinyuandianzi";
const char *password = "AA15994823428";

const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

const char *time_zone = "CET-1CEST,M3.5.0,M10.5.0/3"; // TimeZone rule for Europe/Rome including
                                                      // daylight adjustment rules (optional)

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("No time available (yet)");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t) {
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}
//clang-format off
void print_pmu_output() {
  Serial.println("DCDC=======================================================================");
  Serial.printf("DC1  : %s   Voltage:%u mV \n", PMU.isEnableDC1() ? "+" : "-", PMU.getDC1Voltage());
  Serial.printf("DC2  : %s   Voltage:%u mV \n", PMU.isEnableDC2() ? "+" : "-", PMU.getDC2Voltage());
  Serial.printf("DC3  : %s   Voltage:%u mV \n", PMU.isEnableDC3() ? "+" : "-", PMU.getDC3Voltage());
  Serial.printf("DC4  : %s   Voltage:%u mV \n", PMU.isEnableDC4() ? "+" : "-", PMU.getDC4Voltage());
  Serial.printf("DC5  : %s   Voltage:%u mV \n", PMU.isEnableDC5() ? "+" : "-", PMU.getDC5Voltage());
  Serial.println("ALDO=======================================================================");
  Serial.printf("ALDO1: %s   Voltage:%u mV\n", PMU.isEnableALDO1() ? "+" : "-", PMU.getALDO1Voltage());
  Serial.printf("ALDO2: %s   Voltage:%u mV\n", PMU.isEnableALDO2() ? "+" : "-", PMU.getALDO2Voltage());
  Serial.printf("ALDO3: %s   Voltage:%u mV\n", PMU.isEnableALDO3() ? "+" : "-", PMU.getALDO3Voltage());
  Serial.printf("ALDO4: %s   Voltage:%u mV\n", PMU.isEnableALDO4() ? "+" : "-", PMU.getALDO4Voltage());
  Serial.println("BLDO=======================================================================");
  Serial.printf("BLDO1: %s   Voltage:%u mV\n", PMU.isEnableBLDO1() ? "+" : "-", PMU.getBLDO1Voltage());
  Serial.printf("BLDO2: %s   Voltage:%u mV\n", PMU.isEnableBLDO2() ? "+" : "-", PMU.getBLDO2Voltage());
  Serial.println("CPUSLDO====================================================================");
  Serial.printf("CPUSLDO: %s Voltage:%u mV\n", PMU.isEnableCPUSLDO() ? "+" : "-", PMU.getCPUSLDOVoltage());
  Serial.println("DLDO=======================================================================");
  Serial.printf("DLDO1: %s   Voltage:%u mV\n", PMU.isEnableDLDO1() ? "+" : "-", PMU.getDLDO1Voltage());
  Serial.printf("DLDO2: %s   Voltage:%u mV\n", PMU.isEnableDLDO2() ? "+" : "-", PMU.getDLDO2Voltage());
  Serial.println("===========================================================================");
}
//clang-format on

void scan_i2c_device() {
  Serial.println("Scanning for I2C devices ...");
  Serial.print("      ");
  for (int i = 0; i < 0x10; i++) {
    Serial.printf("0x%02X|", i);
  }
  uint8_t error;
  for (int j = 0; j < 0x80; j += 0x10) {
    Serial.println();
    Serial.printf("0x%02X |", j);
    for (int i = 0; i < 0x10; i++) {
      Wire.beginTransmission(i | j);
      error = Wire.endTransmission();
      if (error == 0)
        Serial.printf("0x%02X|", i | j);
      else
        Serial.print(" -- |");
    }
  }
  Serial.println();
  Serial.println("I2C device scan ends");
}

static void led_task(void *param) {
  CRGB leds[1];
  FastLED.addLeds<WS2812, PIN_WS2812_DATA, RGB>(leds, 1);
  static uint8_t hue = 0;
  while (1) {
    leds[0] = CHSV(hue++, 255, 100);
    FastLED.show();
    delay(50);
  }
  vTaskDelete(NULL);
}
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  // copy a buffer's content to a specific area of the display
  lcd_push_colors_soft_rotation(area->x1, area->y1, w, h, (uint16_t *)color_map, 1);
  lv_disp_flush_ready(drv);
}
static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (touch.read()) {
    uint8_t fn = touch.getPointNum();
    TP_Point t = touch.getPoint(0);
    data->point.x = t.x;
    data->point.y = t.y;
    data->state = LV_INDEV_STATE_PR;
  } else
    data->state = LV_INDEV_STATE_REL;
}

static void lvgl_init(void) {
  static lv_disp_draw_buf_t disp_buf;
  static lv_disp_drv_t disp_drv; // contains callback functions
  static lv_color_t *lv_disp_buf;
  static lv_indev_drv_t indev_drv;

  lv_init();
  lv_disp_buf = (lv_color_t *)heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);

  lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, NULL, LCD_WIDTH * LCD_HEIGHT);
  /*Initialize the display*/
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = LCD_HEIGHT;
  disp_drv.ver_res = LCD_WIDTH;
  disp_drv.flush_cb = lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.full_refresh = 1;
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touchpad_read;
  lv_indev_drv_register(&indev_drv);
}

void setup() {
  Serial.begin(115200);
  Serial.println("T-AMOLED Factory");

  Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);

  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, PIN_IIC_SDA, PIN_IIC_SCL)) {
    Serial.println("PMU is not online...");
    while (1)
      delay(50);
  }
  PMU.setALDO1Voltage(1800);
  PMU.enableALDO1();

  PMU.setALDO2Voltage(2800);
  PMU.enableALDO2();

  PMU.setALDO3Voltage(3300);
  PMU.enableALDO3();

  PMU.setALDO4Voltage(3300);
  PMU.enableALDO4();

  PMU.setBLDO2Voltage(2800);
  PMU.enableBLDO2();

  PMU.disableTSPinMeasure();
  // print_pmu_output();

  xTaskCreate(led_task, "led_task", 1024 * 2, NULL, 1, NULL);

  pinMode(PIN_TOUCH_IRQ, INPUT_PULLUP);
  delay(10);
  pinMode(PIN_TOUCH_RST, OUTPUT);
  digitalWrite(PIN_TOUCH_RST, 1);

  scan_i2c_device();
  touch.init();
  touch.setRotation(1);

  // lcd module :SH8501B
  lcd_settings_t lcd_config = {.clk_gpio_num = PIN_LCD_CLK,
                               .d0_gpio_num = PIN_LCD_D0,
                               .d1_gpio_num = PIN_LCD_D1,
                               .d2_gpio_num = PIN_LCD_D2,
                               .d3_gpio_num = PIN_LCD_D3,
                               .cs_gpio_num = PIN_LCD_CS,
                               .res_gpio_num = PIN_LCD_RES,
                               .te_gpio_num = PIN_LCD_TE,
                               .frequency = 30 * 1000 * 1000};

  lcd_init(&lcd_config);
  lcd_push_colors_soft_rotation(0, 0, 368, 194, (uint16_t *)gImage_logo_img, 1);
  delay(5000);
  // lvgl_init();
}

void loop() {
  // delay(2);
  // lv_timer_handler();

  // if (touch.read()) {
  //   TP_Point t = touch.getPoint(0);
  //   Serial.printf("[Touch] x:%d , y:%d\r\n", t.x, t.y);
  // }

  Serial.printf("[BAT]:percent: %d%%\r\n", PMU.getBatteryPercent());
  delay(5000);
}
