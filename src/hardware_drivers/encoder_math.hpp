#pragma once

// Pure period-to-speed conversion for the wheel encoders.
//
// Deliberately free of the Pico SDK and FreeRTOS so it compiles into the host
// test build (`test/`). The hardware half — PIO, DMA, and the freshness check —
// stays in encoder.cpp; only the arithmetic lives here, because arithmetic is
// the part that can be wrong in a way no amount of bench time will reveal.
//
// **This header cannot tell a stopped wheel from a stale reading**, and it is
// not trying to. `encoder.pio` is a period counter: when the wheel stops,
// pushes stop and the capture ring keeps its last values forever. That is
// issue #12, and the fix is a freshness check on the DMA channel, made by the
// caller. A mean of zero here means "no sample has ever landed", not "stopped".
// See CAL-0 in system-design/test-plans/0002-self-calibration.md.

#include <cstddef>
#include <cstdint>

namespace encoder
{

// --- The constants the conversion rests on -------------------------------
//
// Named rather than inlined because a bare `20` in a division is how a
// measurement turns back into a guess. Both are provisional: CAL-3 measures
// the effective value under load, which is not the geometric one.

// Edges per wheel revolution, at the encoder disc. `encoder.pio` pushes once
// per high->low transition, so this counts pushes, not quadrature states.
inline constexpr float ticks_per_rev = 20.0f;

// Rate at which the PIO counter advances, in counts per second.
//
// Derivation: `encoder_program_init` sets the state-machine clock divider to
// clk_sys / 5000, so the state machine runs at 5 kHz. Both paths through the
// program — `low_loop` and `high_loop` — are exactly 5 PIO cycles per
// iteration (the `[1]` delay on the second `MOV Y, ~Y` in `high_loop` exists
// to make them match), and the counter advances once per iteration. 5 kHz / 5
// cycles = 1000 counts per second, i.e. one count per millisecond.
//
// If either the divider or the instruction count in encoder.pio changes, this
// number changes with it and every speed reading scales.
inline constexpr float counts_per_second = 1000.0f;

inline constexpr float seconds_per_minute = 60.0f;

// --- Staleness ------------------------------------------------------------

// **PROVISIONAL PLACEHOLDER, NOT A DERIVATION.** Chosen by Andrew 2026-08-23 as
// a working value for bench bring-up. TP-0002 CAL-0 requires this be derived
// from the slowest speed the control loop must resolve and the derivation
// recorded, explicitly warning against a round number. No such speed floor
// exists yet: requirements/navigation.md states an accuracy target (NAV-1) and
// no minimum speed.
//
// What this value means, so the cost is visible:
//   - it sets the slowest resolvable wheel speed at 3 rpm, since a wheel
//     turning slower than one edge per t_stale is indistinguishable from
//     stopped (60 s/min / (1 s x 20 ticks/rev));
//   - it is also the delay before a STALLED wheel is detected. For that whole
//     second the controller sees its stale pre-stall reading, computes an error
//     near zero, and holds duty into a stalled motor. That bears on SAF-21 and
//     on the TB6612's 1.2 A continuous rating.
//
// Reduce it before the machine drives anywhere that is not blocks.
inline constexpr uint32_t t_stale_us = 1'000'000;

// True if a sample has arrived recently enough to trust the conversion.
//
// Guards the unsigned-underflow trap that safety.hpp documents: if `now` is
// somehow older than `last_change`, an unguarded subtraction reads as a
// multi-thousand-year gap and trips the timeout instantly. Treat that as fresh
// and let the next poll settle it, rather than declaring a moving wheel stopped
// on the strength of a clock anomaly.
inline bool is_fresh(uint64_t now_us, uint64_t last_change_us, uint32_t stale_us = t_stale_us)
{
    if (now_us <= last_change_us)
    {
        return true;
    }
    return (now_us - last_change_us) < static_cast<uint64_t>(stale_us);
}

// --- Conversion ----------------------------------------------------------

// Mean of the capture ring. Returns 0 if no sample has landed yet.
//
// Takes the buffer as an argument rather than reaching for a file-scope one, so
// it is testable and so the left/right duplication in encoder.cpp collapses.
inline float mean_period_counts(const uint32_t* samples, std::size_t count)
{
    if (samples == nullptr || count == 0)
    {
        return 0.0f;
    }

    float total = 0.0f;
    for (std::size_t i = 0; i < count; ++i)
    {
        total += static_cast<float>(samples[i]);
    }
    return total / static_cast<float>(count);
}

// Converts a mean inter-edge period, in PIO counts, to wheel RPM.
//
//   counts/edge / (counts/second)      = seconds per edge
//   x ticks_per_rev                    = seconds per revolution
//   inverted, x 60                     = revolutions per minute
//
// A mean of zero yields zero rather than infinity: no sample has landed, and
// there is nothing to say about the speed.
inline float rpm_from_mean_period(float mean_counts)
{
    if (!(mean_counts > 0.0f))
    {
        return 0.0f;
    }
    return (seconds_per_minute * counts_per_second) / (mean_counts * ticks_per_rev);
}

inline float rpm_from_periods(const uint32_t* samples, std::size_t count)
{
    return rpm_from_mean_period(mean_period_counts(samples, count));
}

} // namespace encoder
