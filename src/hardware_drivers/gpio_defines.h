#pragma once
#include "pico/stdlib.h"

namespace gpio
{
enum class pins
{
    right_pwm_pin = 3,
    right_forward_pin = 4,
    right_backward_pin = 5,
    left_pwm_pin = 6,
    left_forward_pin = 7,
    left_backward_pin = 8,
    // STBY on the motor driver, an Adafruit TB6612 breakout (product 2448,
    // Toshiba TB6612FNG). Active high: LOW is standby, HIGH enables the
    // H-bridges. That breakout carries R1, a 10 kΩ pull-up from STBY to VCC,
    // which outvotes the RP2350 pad pull-down - so the driver is ENABLED
    // whenever this pin is undriven, including from power-on until firmware
    // drives it low. Closing that window needs a board change (remove R1, or
    // fit an external pull-down of ≤ 4.3 kΩ, ideally ≈2.2 kΩ), not a firmware
    // change; see SAF-19 in ADR-0010 and the watchdog note in src/app/rt.h.
    driver_enable_pin = 2,
    right_encoder_pin = 10,
    left_encoder_pin = 11,
    neopixel_pin = 15
};

const uint16_t pwm_frequency = 1000;
const float encoder_ticks = 20.0;
} // namespace gpio