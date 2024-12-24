// Header file
#include "../header/rgb.hpp"

#include "driver/ledc.h"

// RGB LED Class constructor
RGB_LED::RGB_LED(gpio_num_t R, gpio_num_t G, gpio_num_t B, ledc_timer_t timer,
                 ledc_channel_t r_chan, ledc_channel_t g_chan,
                 ledc_channel_t b_chan) {
  // Store the channels and timer
  time = timer;
  chans[0] = r_chan;
  chans[1] = g_chan;
  chans[2] = b_chan;

  // Configure the pins
  const gpio_config_t gpio_conf{(1ULL << R) | (1ULL << G) | (1ULL << B),
                                GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE,
                                GPIO_PULLDOWN_DISABLE, GPIO_INTR_DISABLE};
  gpio_config(&gpio_conf);

  // PWM timer configuration
  const ledc_timer_config_t timer_conf{LEDC_HIGH_SPEED_MODE, LEDC_TIMER_8_BIT,
                                       timer, 5000};
  ledc_timer_config(&timer_conf);

  // PWM red channel configuration
  const ledc_channel_config_t red_channel_conf{
      R,      LEDC_HIGH_SPEED_MODE,
      r_chan, LEDC_INTR_DISABLE,
      timer,  static_cast<uint32_t>(colors[0]),
      0,      {0}};
  ledc_channel_config(&red_channel_conf);

  // PWM green channel configuration
  const ledc_channel_config_t green_channel_conf{
      G,      LEDC_HIGH_SPEED_MODE,
      g_chan, LEDC_INTR_DISABLE,
      timer,  static_cast<uint32_t>(colors[1]),
      0,      {0}};
  ledc_channel_config(&green_channel_conf);

  // PWM blue channel configuration
  const ledc_channel_config_t blue_channel_conf{
      B,      LEDC_HIGH_SPEED_MODE,
      b_chan, LEDC_INTR_DISABLE,
      timer,  static_cast<uint32_t>(colors[2]),
      0,      {0}};
  ledc_channel_config(&blue_channel_conf);
}

// setColor: (uint8_t, uint8_t, uint8_t) --> (none)
// Sets the color of the LED
void RGB_LED::updateColor(uint8_t* color) {
  for (size_t i = 0; i < 3; i++) {
    ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, chans[i],
                             static_cast<uint32_t>(*color), 0);
    color++;
  }
}

// setColor: (uint8_t*) --> (none)
// Sets the color of the LED, updates state
void RGB_LED::setColor(uint8_t* color) {
  for (size_t i = 0; i < 3; i++) {
    colors[i] = *color;
    color++;
  }
  updateColor(colors);
}

// setRed: (uint8_t) --> (none)
// Adjust the red value of the LED, updates state
void RGB_LED::setRed(uint8_t R) {
  colors[0] = R;
  setColor(colors);
}

// setGreen: (uint8_t) --> (none)
// Adjust the green value of the LED, updates state
void RGB_LED::setGreen(uint8_t G) {
  colors[1] = G;
  setColor(colors);
}

// setBlue: (uint8_t) --> (none)
// Adjust the blue value of the LED, updates state
void RGB_LED::setBlue(uint8_t B) {
  colors[2] = B;
  setColor(colors);
}

// getColor: (none) --> (uint8_t*)
// Returns a pointer to the array of current values
const uint8_t* RGB_LED::getColor() { return colors; }

// off: (none) --> (none)
// Turns off the LED, updates state
void RGB_LED::off() {
  for (size_t i = 0; i < 3; i++) {
    colors[i] = 0;
  }
  setColor(colors);
}

// on: (none) --> (none)
// Sets the LED to white, updates state
void RGB_LED::on() {
  for (size_t i = 0; i < 3; i++) {
    colors[i] = 255;
  }
  setColor(colors);
}
