#pragma once

// Header files
#include "driver/gpio.h"
#include "driver/ledc.h"

// RGB LED Class
class RGB_LED {
 public:
  RGB_LED(gpio_num_t R, gpio_num_t G, gpio_num_t B);
  void setColor(uint8_t* colors);
  void setRed(uint8_t R);
  void setGreen(uint8_t G);
  void setBlue(uint8_t B);
  uint8_t* getColor();
  void off();
  void on();

 private:
  uint8_t colors[3]{0, 0, 0};
  gpio_num_t pinR, pinG, pinB;
  void updateColor(uint8_t R, uint8_t G, uint8_t B);
};
