#pragma once

// Header files
#include "driver/gpio.h"

// RGB LED Class
class RGB_LED {
 public:
  RGB_LED(gpio_num_t R, gpio_num_t G, gpio_num_t B);
  void setColor(int8_t R, int8_t G, int8_t B);
  void setColor(int8_t* colors);
  int8_t* getColor();
  void off();
  void on();

 private:
  int8_t colors[3]{0, 0, 0};
  gpio_num_t pinR, pinG, pinB;
};
