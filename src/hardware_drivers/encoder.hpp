#pragma once

#include "encoder.pio.h"
#include "encoder_math.hpp"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware_drivers/gpio_defines.h"
#include "pico/stdlib.h"

namespace encoder
{

// A wheel speed together with whether it may be believed.
//
// **`rpm` is meaningless unless `valid` is true, and `rpm == 0` with
// `valid == false` does NOT mean the wheel is stopped** — it means nothing is
// known. A controller that reads the zero and ignores the flag sees a large
// error against a nonzero target and commands maximum output into a wheel that
// may be turning perfectly well. That is a runaway, and it is the reason this
// type exists instead of a bare float.
//
// SAF-20 requires that stale data not be used for control. The correct response
// to `valid == false` is to remove drive, never to treat the reading as zero.
struct [[nodiscard]] Reading
{
    float rpm;
    bool valid;
};

void init(PIO pio, uint sm_index);

// Reads one wheel and updates its freshness state.
//
// **Call exactly once per control iteration.** Freshness is judged by whether
// the DMA write pointer has moved since the previous call, so the call cadence
// is part of the measurement rather than incidental to it — see the aliasing
// note in encoder.cpp before changing the control period.
Reading read_left();
Reading read_right();

} // namespace encoder
