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

    // Clearing the FIFOs is not tidiness, it is the reason this can use a
    // blocking put at all. The state machine is disabled on the line above, so
    // nothing will drain a FIFO that already holds queued levels -- and
    // pio_sm_put_blocking would then spin forever, in whichever task changed
    // the period, with the wheel held at its last duty by pwm.pio's
    // `pull noblock`. It also discards stale levels that would otherwise be
    // consumed ahead of the caller's new one on re-enable.
    pio_sm_clear_fifos(pio, sm);

    pio_sm_put(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

namespace
{
// Not per-state-machine: a nonzero value means "a state machine stopped
// draining", and which one it was is a bench question, not a control-loop one.
//
// Incremented atomically rather than with ++. The intended callers reach this
// from both cores -- the control task on core 1 once PID/PWM is wired, and
// motors::safe_state() from link_tx_task on core 0 -- and volatile gives
// neither atomicity nor cross-core ordering on a Cortex-M33, so two concurrent
// refusals would record one. A driver should not pull in FreeRTOS for this.
volatile uint32_t drop_count = 0;
} // namespace

// Write `level` to TX FIFO. State machine will copy this into X.
//
// Deliberately NOT pio_sm_put_blocking(). The blocking form spins on a full
// FIFO with no timeout, and the FIFO only fills if the state machine has
// stopped draining it -- disabled, held in reset, or never started. In that
// state the blocking form never returns, and its caller is the control task:
// priority 20 on core 1, which nothing preempts and which feeds the watchdog.
// The recovery would be a watchdog reset, which is a poor way to discover a
// misconfigured state machine.
//
// The drop is not itself safe -- see the header comment on why a refused write
// leaves the previous duty running -- so it is counted rather than swallowed.
bool pwm::pio_pwm_set_level(PIO pio, uint sm, uint32_t level)
{
    if (pio_sm_is_tx_fifo_full(pio, sm))
    {
        __atomic_fetch_add(&drop_count, 1, __ATOMIC_RELAXED);
        return false;
    }
    pio_sm_put(pio, sm, level);
    return true;
}

uint32_t pwm::dropped_writes()
{
    return drop_count;
}
