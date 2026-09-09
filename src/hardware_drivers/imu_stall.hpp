#pragma once

// Stall detection and gap accounting for the IMU sample stream.
//
// Deliberately free of the Pico SDK and FreeRTOS so it compiles into the host
// test build (test/), following the pattern of encoder_math.hpp: the hardware
// half stays in imu_i2c.cpp and only the arithmetic lives here, because
// arithmetic is the part that can be wrong in a way no amount of bench time
// will reveal.
//
// **Why this is a timeout and not a DMA poll.** It is tempting to detect a
// stalled read from the DMA channel itself, and the RX channel does expose
// usable state -- TRANS_COUNT decrements 12 -> 0 in NORMAL mode, CTRL.BUSY is
// high for the sequence, and the error flags are loud. None of that helps for
// the failure this driver is most likely to have. Data-ready is latched, so the
// burst read is what re-arms INT1; if a read is ever missed, INT1 stays
// asserted, no further edge is generated, and the DMA channel is simply never
// triggered again. In that state TRANS_COUNT sits at its reload value, BUSY is
// low, no error flag is set, and **every DMA register reports a perfectly
// healthy idle channel.** Only the clock notices. See
// system-design/research/imu-driver-findings.md section 5 and issue #46.
//
// So the primary detector is elapsed time since the last completed sample. The
// DMA registers are for explaining why, after this has fired.
//
// This header decides nothing about what to do on a stall -- no recovery, no
// abort, no message. It answers "has too long passed" and "how many samples
// went missing", and the caller, which does not exist yet, owns the rest.

#include <cstdint>

namespace imu_stall
{

// --- Sample cadence -------------------------------------------------------

// Provisional, and provisional in the specific sense that IF-0001 marks the
// 208 Hz rate itself provisional. It is the ODR diagnostics/imu-probe ran at
// and confirmed on the pin. Whatever configures CTRL1_XL/CTRL2_G owns the real
// value; this is the number the timeout is derived from, and the two must not
// be allowed to drift apart.
inline constexpr uint32_t nominal_odr_hz = 208;

// Nominal microseconds between data-ready edges at a given ODR.
//
// Returns 0 for an ODR of 0 rather than dividing by it. A period of 0 makes
// every timeout 0, i.e. permanently stalled, which is the direction of error
// this file chooses everywhere: see stallTimeoutUs().
inline constexpr uint32_t periodUsForOdr(uint32_t odrHz)
{
    return odrHz == 0 ? 0u : 1000000u / odrHz;
}

// 4807 us at 208 Hz.
inline constexpr uint32_t nominal_period_us = periodUsForOdr(nominal_odr_hz);

// --- Stall threshold, derived ---------------------------------------------
//
// T_imu_stale = 2 x T_s_worst + D. Derived 2026-09-09; this replaces an earlier
// "three nominal periods" placeholder that landed nearby by luck rather than by
// argument. Expressed as arithmetic, not as a literal, so it recomputes if the
// ODR changes -- IF-0001 still marks 208 Hz provisional.
//
// **Why 2x and not 1x.** The largest age observable in a HEALTHY stream is
// T_s + D, not T_s: the timestamp is latched at INT1 IRQ entry but only becomes
// visible when the burst completes. Just before sample N+1 publishes, the newest
// visible stamp is N's, already aged T_s + D. That sets the floor at 6081 us.
// The second T_s_worst is headroom for DMA-completion delay on core 0, which
// shares that core with link RX, link TX, telemetry and logging and has no
// measured worst-case latency.
//
// **Provenance, honestly labelled.** E_slow is the weakest input and the one
// that dominates: it is an ENCODING bound, not a datasheet bound. Nobody has
// found an ODR accuracy spec across process and temperature; INTERNAL_FREQ_FINE
// (0x63) is 8 bits at 0.15%/step, which read as signed gives -19.2%..+19.05%,
// and that range is standing in for a tolerance. D has never been measured
// either -- it is 138 SCL periods at 400 kHz, arithmetic.
//
// **The saving grace is that it barely matters.** Across the whole plausible
// range of E_slow (0% to 19.2%) the threshold moves 9966 -> 11812 us, a span
// smaller than one 5 ms control period. The detector's output is quantised to
// that period anyway, so taking the worst case costs nothing observable.
//
// **Validity condition, so it can be checked rather than assumed:** this holds
// while the worst-case INT1-edge-to-visible-on-core-1 delay stays <= 6081 us.
// Beyond that the headroom is gone and the threshold false-fires.
inline constexpr uint32_t odr_error_slow_ppk = 192; // 19.2%, parts per thousand

// Burst transport delay: 138 SCL periods at 400 kHz. Arithmetic, never timed.
inline constexpr uint32_t burst_delay_us = 350;

inline constexpr uint32_t worstCasePeriodUs(uint32_t nominalUs)
{
    return nominalUs + (nominalUs * odr_error_slow_ppk) / 1000u;
}

inline constexpr uint32_t worst_case_period_us = worstCasePeriodUs(nominal_period_us);

// 11812 us at 208 Hz nominal.
inline constexpr uint32_t stall_timeout_us = 2u * worst_case_period_us + burst_delay_us;

// --- Recovery policy -------------------------------------------------------
//
// Consecutive failed recovery attempts before a stall is called persistent
// rather than transient.
//
// **This is policy, not a derivation, and it cannot be derived from what
// exists.** No fault has ever been injected on this board, so there is no
// distribution of "attempts needed to recover" to derive from. What IS confirmed
// is the mechanism, deterministically and on the cause that matters most: the
// first probe run polled STATUS_REG without draining and saw 0 edges; the second
// drained the output registers and saw 209 edges against 210 drains, its first
// drain performed on an INT1 that had been latched high through a 100 ms settle.
// A forced read either clears the latch or it does not -- repetition buys no
// information once that is true.
//
// So M exists only to separate causes, and is bracketed by argument: M >= 2
// because one attempt can be lost to a race with an in-flight burst, which is
// transient by construction and not a fault; M <= 5 because the blind window is
// (M + 1) x (stall_timeout_us + control period), and at M = 5 that is 101 ms
// against the 224 ms it takes to cover d_allow at the 0.68 m/s cap. M = 3 gives
// 67 ms, 30% of that.
//
// **The success criterion matters more than the value.** An attempt counts as
// FAILED unless a completed sample follows within stall_timeout_us. "The forced
// read returned 12 bytes" is the wrong test: after a sensor brown-out INT1_CTRL
// reverts to 0x00, so the read succeeds forever while no edge is ever generated
// -- and M would never increment in exactly the failure it exists to report.
//
// What would make this measured: fault injection -- hold SDA low, pull the
// sensor's 3V3 mid-stream, deliberately skip a drain -- and count attempts to
// recovery. That has never been run.
inline constexpr uint32_t recovery_attempts_before_persistent = 3;

// Retained for callers that want an explicit period count.
inline constexpr uint32_t stall_periods = 3;

// --- Timeout arithmetic ---------------------------------------------------

// Microseconds of silence that constitute a stall.
//
// Saturates rather than wrapping: periodUs * periods is computed in 64 bits and
// clamped, so a large ODR period cannot fold round into a tiny timeout and
// produce a detector that fires constantly.
inline constexpr uint32_t stallTimeoutUs(uint32_t periodUs, uint32_t periods = stall_periods)
{
    const uint64_t product = static_cast<uint64_t>(periodUs) * static_cast<uint64_t>(periods);
    return product > static_cast<uint64_t>(UINT32_MAX) ? UINT32_MAX
                                                       : static_cast<uint32_t>(product);
}

// True if too long has passed since the last completed sample.
//
// `lastSampleUs` is the timestamp of the last burst that *finished*, not of the
// last INT1 edge: a burst that was started and never completed must read as a
// stall, so arming the transfer cannot be what refreshes this.
//
// Guards the unsigned-underflow trap safety.hpp documents, the same way
// encoder_math::is_fresh does. If `now` is somehow older than `last`, an
// unguarded subtraction reads as a multi-thousand-year gap and trips instantly.
// Both stamps come from the same monotonic microsecond clock, so this should be
// impossible; treat it as healthy and let the next evaluation settle it, since
// `now` only moves forward and one evaluation of tolerance costs nothing.
inline constexpr bool isStalled(uint64_t nowUs, uint64_t lastSampleUs, uint32_t timeoutUs)
{
    if (nowUs <= lastSampleUs)
    {
        return false;
    }
    return (nowUs - lastSampleUs) >= static_cast<uint64_t>(timeoutUs);
}

// --- Gap accounting -------------------------------------------------------

// Samples that should have arrived between two consecutive completed samples
// and did not.
//
// A perfect stream gives elapsed == periodUs and therefore 0. Two periods of
// silence means one sample was lost, and so on.
//
// **Rounds to the nearest slot rather than truncating, and that is a fix, not a
// preference.** Truncation looked like free jitter tolerance and was in fact
// hiding real losses, because the divisor is the NOMINAL period while the part
// runs measurably faster: 208 Hz nominal is 4807 us, and this board measures
// 4772 us (0.75% fast, 208 intervals, bench log 2026-09-09). One genuinely lost
// sample therefore gives elapsed = 2 x 4772 = 9544 us, and 9544 / 4807 = 1 slot,
// so missed came out 0. Single-sample losses were invisible in a wire field
// ADR-0007 requires be honest, and they were invisible *because* the divisor was
// 0.75% too large -- the tolerance was being paid for with a whole sample.
//
// Rounding is correct in both directions and does not depend on the sign of that
// error: a healthy 4772 us interval still gives 0, and a 9544 us one gives 1. It
// declares a loss above 1.5 periods rather than 2.0, which is ample -- the
// measured interval spread across a full second was within the 1 us capture
// resolution.
//
// Returns 0 for a zero period -- a cadence that is not known cannot be used to
// count what is missing from it, and inventing a number here would put a
// fabricated value into a wire field that ADR-0007 requires be honest.
inline constexpr uint32_t missedSamples(uint64_t nowUs, uint64_t lastSampleUs, uint32_t periodUs)
{
    if (periodUs == 0 || nowUs <= lastSampleUs)
    {
        return 0;
    }
    const uint64_t elapsed = nowUs - lastSampleUs;
    const uint64_t slots =
        (elapsed + static_cast<uint64_t>(periodUs) / 2u) / static_cast<uint64_t>(periodUs);
    if (slots == 0)
    {
        return 0;
    }
    const uint64_t missed = slots - 1;
    return missed > static_cast<uint64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(missed);
}

// The value at which LAWN_IMU_RAW.gap stops counting. IF-0001: "Samples lost
// immediately before this one, 255 = saturated."
inline constexpr uint8_t gap_saturated = 255;

// Saturating accumulate into the gap field.
//
// **Never wraps, and that is the entire point.** The field is a uint8_t on the
// wire, so an unguarded `current + lost` folds 256 lost samples into a reported
// gap of 0 -- a total stream outage that reads on the Pi as a perfect
// uninterrupted stream. Pegged at 255 it reads as "at least 255 lost", which is
// what the interface already defines it to mean. ADR-0007: gaps are reported,
// never silently interpolated, and a wrapped counter is the most convincing
// possible form of silent interpolation.
//
// The sum is widened to 64 bits before it is compared. Adding in uint32 is not
// good enough and the host tests caught it: `255 + UINT32_MAX` wraps to 254, so
// the largest possible loss reported the second-smallest possible gap. The
// overflow this guards against is one addition away from the overflow it is
// documenting.
inline constexpr uint8_t saturatingAddGap(uint8_t current, uint32_t lost)
{
    const uint64_t total = static_cast<uint64_t>(current) + static_cast<uint64_t>(lost);
    return total >= static_cast<uint64_t>(gap_saturated) ? gap_saturated
                                                         : static_cast<uint8_t>(total);
}

// Gaps accumulated since the last sample that was actually emitted.
//
// Separate from a bare counter because losses and emissions are not
// one-for-one: samples can be dropped while the link is busy, and the count of
// what was lost has to survive until there is a message to carry it. take()
// hands the count over and clears it, so each LAWN_IMU_RAW reports the losses
// immediately preceding it and no others -- and a sample lost twice is not
// counted twice.
struct GapCounter
{
    uint8_t pending = 0;

    void add(uint32_t lost)
    {
        pending = saturatingAddGap(pending, lost);
    }

    // Records whatever the interval between two completed samples implies.
    void addForInterval(uint64_t nowUs, uint64_t lastSampleUs, uint32_t periodUs)
    {
        add(missedSamples(nowUs, lastSampleUs, periodUs));
    }

    uint8_t take()
    {
        const uint8_t value = pending;
        pending = 0;
        return value;
    }
};

} // namespace imu_stall
