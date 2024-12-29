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
  viz.on();

  // Initialize buzzer
  Buzzer audio{BUZZER_PIN, LEDC_TIMER_1, LEDC_CHANNEL_3};

  // Initialize shared variables (user_id)

  // RFID task:
  // --> Start SPI bus
  // --> Initialize MFRC522 device

  // Initialize bluetooth
}
