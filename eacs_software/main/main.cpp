// Custom headers
#include "./header/buzzer.hpp"
#include "./header/rgb.hpp"

// STL headers
#include <iostream>

// HAL headers
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
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
  uint8_t color1[3]{255, 0, 0};
  uint8_t color2[3]{0, 255, 0};
  while (1) {
    viz.setColor(color1);
    std::cout << 1 << std::endl;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    viz.setColor(color2);
    std::cout << 2 << std::endl;
  }

  // std::cout << '1' << std::endl;

  // Initialize and turn on the BUZZER_PIN
  // Buzzer audio{BUZZER_PIN, LEDC_TIMER_1, LEDC_CHANNEL_3};
  // audio.on();

  // std::cout << '2' << std::endl;

  // Initialize MFRC522

  // Start bluetooth task and card reading task (How to implement ISR?)
}
