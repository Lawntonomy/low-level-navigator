#pragma once
#include "pico/stdlib.h"

namespace config_defines
{
    namespace config_1
    {
        const uint16_t right_pwm_pin = 3;
        const uint16_t right_forward_pin = 4;
        const uint16_t right_backward_pin = 5;

        const uint16_t left_pwm_pin = 6;
        const uint16_t left_forward_pin = 7;
        const uint16_t left_backward_pin = 8;

        const uint16_t driver_enable_pin = 2;

        const uint16_t right_encoder_pin = 10;
        const uint16_t left_encoder_pin = 11;

        const uint16_t pwm_frequency = 1000;
        const float encoder_ticks = 20.0;
    }




};