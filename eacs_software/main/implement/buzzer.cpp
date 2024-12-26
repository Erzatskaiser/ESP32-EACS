// Header file
#include "../header/buzzer.hpp"

#include "soc/clk_tree_defs.h"

// Buzzer class constructor
Buzzer::Buzzer(gpio_num_t in, ledc_timer_t timer, ledc_channel_t chan) {
  // Store pin number
  time = timer;
  chans = chan;

  // Configure the pins
  const gpio_config_t gpio_conf{(1ULL << in), GPIO_MODE_OUTPUT,
                                GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE,
                                GPIO_INTR_DISABLE};
  gpio_config(&gpio_conf);

  // PWM timer configuration
  const ledc_timer_config_t timer_conf{LEDC_HIGH_SPEED_MODE, LEDC_TIMER_8_BIT,
                                       timer, 5000, LEDC_AUTO_CLK};
  ledc_timer_config(&timer_conf);

  // PWM channel configuration
  const ledc_channel_config_t channel_conf{
      in,    LEDC_HIGH_SPEED_MODE,         chan, LEDC_INTR_DISABLE,
      timer, static_cast<uint32_t>(state), 0,    {0}};
  ledc_channel_config(&channel_conf);
}

// update: (uint8_t*) --> (none)
// Updates duty cycle of PWM
void Buzzer::update(uint8_t val) {
  ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, chans,
                           static_cast<uint32_t>(val), 0);
}

// set: (uint8_t*) --> (none)
// Sets the state of the buzzer, updates state
void Buzzer::set(uint8_t val) {
  state = val;
  update(state);
}

// increase: (uint8_t) --> (none)
// Increases PWN duty cycle by set amount
void Buzzer::increase(uint8_t val) {
  if (state == 255) return;
  state += val;
  update(state);
}

// decrease: (uint8_t) --> (none)
// Decreases PWM duty cycle by set amount
void Buzzer::decrease(uint8_t val) {
  if (state == 0) return;
  state -= val;
  update(state);
}

// getState: (none) --> (none)
// Returns current PWM duty cycle
uint8_t Buzzer::getState() { return state; }

// off: (none) --> (none)
// Turns off the buzzer
void Buzzer::off() {
  state = 0;
  update(state);
}

// on: (none)
// Turns on the buzzer
void Buzzer::on() {
  state = 255;
  update(state);
}
