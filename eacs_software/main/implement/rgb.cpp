﻿// Header file
#include "../header/rgb.hpp"

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
  const ledc_timer_config_t timer_conf{LEDC_LOW_SPEED_MODE, LEDC_TIMER_8_BIT,
                                       timer, 1000, LEDC_AUTO_CLK};
  ledc_timer_config(&timer_conf);

  // PWM channel configuration
  gpio_num_t pins[3]{R, G, B};
  ledc_channel_config_t channel_confs[3];
  for (int i = 0; i < 3; i++) {
    channel_confs[i] = {pins[i],  LEDC_LOW_SPEED_MODE,
                        chans[i], LEDC_INTR_DISABLE,
                        timer,    static_cast<uint32_t>(colors[i]),
                        0,        {0}};
    ledc_channel_config(&channel_confs[i]);
  }
}

// setColor: (uint8_t, uint8_t, uint8_t) --> (none)
// Sets the color of the LED based on current values
void RGB_LED::updateColor() {
  uint8_t* color = colors;
  for (size_t i = 0; i < 3; i++) {
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, chans[i],
                             static_cast<uint32_t>(*color), 0);
    ++color;
  }
}

// fadeColor: (uint8_t*) --> (none)
// Fades the color of the LED, updates state
void RGB_LED::fadeColor(uint8_t* color, unsigned int steps,
                        unsigned int delay) {
  // Read target color
  uint8_t target[3];
  for (size_t i = 0; i < 3; i++) {
    target[i] = *color;
    color++;
  }

  // Compute differences in color values
  int red_diff = target[0] - colors[0];
  int green_diff = target[1] - colors[1];
  int blue_diff = target[2] - colors[2];

  // Perform fading in steps
  for (size_t i = 0; i < steps; i++) {
    // Compute current RGB values
    colors[0] += red_diff / steps;
    colors[1] += green_diff / steps;
    colors[2] += blue_diff / steps;

    // Update color
    updateColor();

    // Delay to give fading effect
    vTaskDelay(delay / portTICK_PERIOD_MS);
  }
}

// setColor: (uint8_t*) --> (none)
// Sets the color of the LED, updates state
void RGB_LED::setColor(uint8_t* color) {
  for (size_t i = 0; i < 3; i++) {
    colors[i] = *color;
    color++;
  }
  updateColor();
}

// setRed: (uint8_t) --> (none)
// Adjust the red value of the LED, updates state
void RGB_LED::setRed(uint8_t R) {
  colors[0] = R;
  updateColor();
}

// setGreen: (uint8_t) --> (none)
// Adjust the green value of the LED, updates state
void RGB_LED::setGreen(uint8_t G) {
  colors[1] = G;
  updateColor();
}

// setBlue: (uint8_t) --> (none)
// Adjust the blue value of the LED, updates state
void RGB_LED::setBlue(uint8_t B) {
  colors[2] = B;
  updateColor();
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
  updateColor();
}

// on: (none) --> (none)
// Sets the LED to white, updates state
void RGB_LED::on() {
  for (size_t i = 0; i < 3; i++) {
    colors[i] = 255;
  }
  updateColor();
}
