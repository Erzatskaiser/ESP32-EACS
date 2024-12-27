// Custom headers
#include "./header/buzzer.hpp"
#include "./header/rgb.hpp"
#include "./header/utilities.hpp"

// HAL headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// RGB LED pins
#define R_PIN GPIO_NUM_12
#define G_PIN GPIO_NUM_14
#define B_PIN GPIO_NUM_27

// Buzzer pin
#define BUZZER_PIN GPIO_NUM_3

extern "C" void app_main() {
  // Enable thread safe operation
  ledc_fade_func_install(0);

  // Initialize and turn on LED
  RGB_LED viz{R_PIN, G_PIN, B_PIN};
  uint8_t color1[3]{255, 190, 80};
  uint8_t color2[3]{80, 190, 255};

  test_rgb_led(viz, color1, color2, 2000);
}
