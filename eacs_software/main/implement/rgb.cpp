// Header file
#include "../header/rgb.hpp"

// RGB LED Class constructor
RGB_LED::RGB_LED(gpio_num_t R, gpio_num_t G, gpio_num_t B) {
  // Store pin numbers
  pinR = R;
  pinG = G;
  pinB = B;

  // Configure the pins
  gpio_config_t configuration{(1ULL << R) | (1ULL << G) | (1ULL << B),
                              GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
                              GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};
}

// setColor: (int8_t, int8_t, int8_t) --> (none)
// Sets the color of the LED
void RGB_LED::setColor(int8_t R, int8_t G, int8_t B) { /* Code goes here */ }

// setColor: (init8_t *) --> (none)
// Sets the color of the LED
void RGB_LED::setColor(int8_t* colors) { /* Code goes here */ }

// getColor: (none) --> (int8_t*)
// Returns a pointer to the array of current values
int8_t* RGB_LED::getColor() { return colors; }

// off: (none) --> (none)
// Turns of the RGB LED
void RGB_LED::off() {
  for (size_t i = 0; i < 3; i++) {
    colors[i] = 0;
  }
  setColor(colors);
}

// on: (none) --> (none)
// Turns on the RGB LED (White)
void RGB_LED::on() {
  for (size_t i = 0; i < 3; i++) {
    colors[i] = 1;
  }
  setColor(colors);
}
