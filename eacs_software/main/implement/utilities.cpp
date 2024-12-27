// Custom headers
#include "../header/utilities.hpp"

// HAL headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// test_rgb_led: (RGB_LED, uint8_t*, uint8_t*, int) --> (none)
// Tests an RGB LED by setting it to one color then another
void test_rgb_led(RGB_LED& led, uint8_t* color1, uint8_t* color2, int time) {
  uint8_t col1[3];
  uint8_t col2[3];

  // Read color inputs
  for (size_t i = 0; i < 3; i++) {
    col1[i] = *color1;
    col2[i] = *color2;
    ++color1;
    ++color2;
  }

  // Set colors, with a wait in between
  led.setColor(col1);
  vTaskDelay(time / portTICK_PERIOD_MS);
  led.setColor(col2);
  vTaskDelay(time / portTICK_PERIOD_MS);

  // Turn of the LED
  led.off();
}
