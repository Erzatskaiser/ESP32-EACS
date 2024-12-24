#pragma once

// Header files
#include "driver/gpio.h"
#include "driver/ledc.h"

// RGB LED Class
class RGB_LED {
 public:
  RGB_LED(gpio_num_t R, gpio_num_t G, gpio_num_t B,
          ledc_timer_t timer = LEDC_TIMER_0,
          ledc_channel_t r_chan = LEDC_CHANNEL_0,
          ledc_channel_t g_chan = LEDC_CHANNEL_1,
          ledc_channel_t b_chan = LEDC_CHANNEL_2);
  void setColor(uint8_t* colors);
  void setRed(uint8_t R);
  void setGreen(uint8_t G);
  void setBlue(uint8_t B);
  const uint8_t* getColor();
  void off();
  void on();

 private:
  uint8_t colors[3]{0, 0, 0};
  ledc_timer_t time{LEDC_TIMER_0};
  ledc_channel_t chans[3]{LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
  void updateColor(uint8_t* color);
};
