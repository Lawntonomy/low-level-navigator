#pragma once
#include "hardware/pio.h"
#include "pico/stdlib.h"

namespace pwm
{
void init(PIO pio, uint sm_index);

void pio_pwm_set_period(PIO pio, uint sm, bool left, uint32_t period);

void pio_pwm_set_level(bool left, uint32_t level);

} // namespace pwm
