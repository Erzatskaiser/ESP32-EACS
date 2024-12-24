// Custom headers
#include "./header/buzzer.hpp"
#include "./header/rgb.hpp"

// HAL headers
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

// RGB LED pins
#define R_PIN GPIO_NUM_23
#define G_PIN GPIO_NUM_22
#define B_PIN GPIO_NUM_1

// Buzzer pin
#define BUZZER_PIN GPIO_NUM_3

extern "C" void app_main() {
  // Initialize and turn on LED
  RGB_LED viz{R_PIN, G_PIN, B_PIN};
  viz.on();

  // Initialize and turn on the buzzer
  Buzzer audio{BUZZER_PIN, LEDC_TIMER_1, LEDC_CHANNEL_3};
  audio.on();

  // Initialize MFRC522

  // Start bluetooth task and card reading task (How to implement ISR?)
}
