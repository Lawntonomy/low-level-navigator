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

// How many nominal periods may pass with no completed sample before the stream
// is called stalled.
//
// **This is a policy placeholder, not a derivation.** Three periods is ~14 ms
// at 208 Hz: long enough that ordinary interrupt latency and ODR jitter cannot
// trip it, short enough to be far below any plausible estimator horizon.
// Nothing in system-design pins it down -- no requirement states how long the
// estimator may go without an inertial sample -- so it is chosen, and it is
// chosen deliberately tight because the cost of a false stall (the consumer
// distrusts IMU data) is much lower than the cost of a missed one (the
// estimator is fed confidently stale attitude, which findings section 4 calls
// out as the worse outcome by design).
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
// silence means one sample was lost, and so on. Integer division supplies the
// jitter tolerance for free: an elapsed time of 1.9 periods still reports 0.
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
    const uint64_t slots = elapsed / static_cast<uint64_t>(periodUs);
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
