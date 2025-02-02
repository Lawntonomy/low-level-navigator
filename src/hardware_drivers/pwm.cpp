#include "pwm.h"
#include <stdio.h>
#include "gpio_defines.h"

void pwm::init(PIO pio, uint sm_index)
{
    puts("pwm encoder");
    int offset = pio_add_program(pio, &pwm_program);
    assert(offset > 0);
    pwm_program_init(pio, sm_index, offset, static_cast<uint>(gpio::pins::left_pwm_pin));
    pwm_program_init(pio, sm_index + 1, offset, static_cast<uint>(gpio::pins::right_pwm_pin));
    puts("pwm encoder complete");
}

// Write `period` to the input shift register
void pwm::pio_pwm_set_period(PIO pio, uint sm, uint32_t period)
{
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// pio_pwm_set_period(pio[0], sm[2], (1u << 16) - 1);
// pio_pwm_set_period(pio[0], sm[3], (1u << 16) - 1);
// Write `level` to TX FIFO. State machine will copy this into X.
void pwm::pio_pwm_set_level(PIO pio, uint sm, uint32_t level)
{
    pio_sm_put_blocking(pio, sm, level);
}
