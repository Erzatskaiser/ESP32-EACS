// Custom headers
#include "./rgb.hpp"

// HAL headers
#include "freertos/reeRTOS.h"
#include "freertos/task.h"

// test_rgb_led: (RGB_LED, uint8_t*, uint8_t*) --> (none)
// Tests an RGB led by setting it to one color then another
void test_rgb_led(RGB_LED& led, uint8_t* color1, uint8_t* color2, int time);
