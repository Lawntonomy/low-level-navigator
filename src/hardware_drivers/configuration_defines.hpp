#pragma once
#include "pico/stdlib.h"

namespace config_defines
{
namespace config_1
{
enum class gpio_num
{
    right_pwm_pin = 3,
    right_forward_pin = 4,
    right_backward_pin = 5,
    left_pwm_pin = 6,
    left_forward_pin = 7,
    left_backward_pin = 8,
    driver_enable_pin = 2,
    right_encoder_pin = 10,
    left_encoder_pin = 11
};

const uint16_t pwm_frequency = 1000;
const float encoder_ticks = 20.0;
} // namespace config_1
} // namespace config_defines