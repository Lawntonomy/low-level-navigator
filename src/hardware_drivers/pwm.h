#pragma once
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pwm.pio.h"

namespace pwm
{
void init(PIO pio, uint sm_index);

void pio_pwm_set_period(PIO pio, uint sm, uint32_t period);

// Queue a new duty level. NON-BLOCKING: returns false and counts a drop rather
// than spinning if the state machine's TX FIFO is full.
//
// **A dropped write is NOT fail-safe.** pwm.pio does `pull noblock`, which
// copies X back into the OSR when the FIFO is empty, so the state machine holds
// its PREVIOUS level indefinitely. A dropped write therefore means "keep doing
// what you were doing", not "stop". Any caller on a stop path must check the
// return value; motors::safe_state() is that caller once PIO PWM is attached.
bool pio_pwm_set_level(PIO pio, uint sm, uint32_t level);

// Writes refused because the TX FIFO was full, since boot. Monotonic.
//
// Should be zero in normal operation and is worth reporting if it ever is not:
// at a 65535-count period with no clock divider the state machine consumes one
// word every ~437 us, while the control task writes every 5 ms, so the FIFO
// drains about eleven times faster than it fills. A nonzero count means the
// state machine is not running.
uint32_t dropped_writes();

} // namespace pwm
