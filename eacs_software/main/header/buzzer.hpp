#pragma once

// Header files
#include "driver/gpio.h"
#include "driver/ledc.h"

// RGB LED Class
class Buzzer {
 public:
  Buzzer(gpio_num_t in, ledc_timer_t timer = LEDC_TIMER_0,
         ledc_channel_t chan = LEDC_CHANNEL_0);
  void set(uint8_t val);
  void increase(uint8_t val);
  void decrease(uint8_t val);
  uint8_t getState();
  void off();
  void on();

 private:
  uint8_t state{0};
  ledc_timer_t time;
  ledc_channel_t chans;
  void update(uint8_t val);
};
