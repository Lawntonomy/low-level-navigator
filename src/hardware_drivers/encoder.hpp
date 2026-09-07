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

// The two most recent readings, carried across the core boundary.
//
// read_left()/read_right() run on the control task (core 1); telemetry runs on
// core 0. This pair follows safety.hpp's pattern: publish() is called once per
// control iteration right after the reads, snapshot() is called by telemetry,
// and both take a genuine SMP critical section rather than a bare flag. Unlike
// safety.hpp there is no timestamp to sample inside the lock — the readings
// already carry everything time-sensitive (Reading::valid) — so the section is
// a plain struct copy: two floats and two bools, nothing more (rt.h: every
// critical section here contends the spinlock pair the control task needs).
struct WheelReadings
{
    Reading left;
    Reading right;
};

void publish_readings(const WheelReadings& readings);
WheelReadings latest_readings();

} // namespace encoder
