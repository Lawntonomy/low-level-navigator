#pragma once

// The published IMU sample and the arithmetic that fills it: decoding a 12-byte
// burst, pairing it with the timestamp INT1 latched, and accounting for what
// went missing between one completed sample and the next.
//
// Deliberately free of the Pico SDK and FreeRTOS so it compiles into the host
// test build, following encoder_math.hpp and imu_stall.hpp. The hardware half --
// the two interrupt handlers, the spinlocks, the DMA channel -- lives in
// imu_drdy.cpp and is not testable anywhere but a bench. What IS testable is
// exactly the part a bench would not catch: a gap that is counted twice, or
// counted once and then thrown away, still produces a wire message that looks
// perfectly well formed.
//
// Owns no synchronisation of its own. Every member here is written by the DMA
// completion handler on core 0 and read by the control task on core 1, so
// imu_drdy.cpp holds imu_sample_lock across every call. Do not add a caller
// that forgets that.

#include "imu_stall.hpp"

#include <cstddef>
#include <cstdint>

namespace imu_sample
{

// Bytes in one burst: OUTX_L_G onwards. Must match imu_i2c::burst_bytes; the
// static_assert that ties them together lives in imu_drdy.cpp, because this
// header must not include the transport header.
inline constexpr std::size_t burst_bytes = 12;

// One data-ready sample, as published to whatever consumes it.
//
// Raw counts, not engineering units. ADR-0006 keeps estimation on the Pi and
// this tier forwards what the part produced; the full-scale settings needed to
// convert are provisional anyway (see the CAL-6 note in lsm6dsox.cpp).
struct Sample
{
    // Microseconds, from time_us_64(), latched as the FIRST statement of the
    // INT1 handler -- so it is the time of the data-ready EDGE, not the time
    // the bytes arrived. 64-bit and not 32-bit: a 32-bit microsecond stamp
    // wraps every 71.6 minutes, and imu_stall::isStalled() would then read the
    // wrap as either a multi-thousand-second age or, through its
    // nowUs <= lastSampleUs guard, as perfectly healthy.
    uint64_t stamp_us = 0;

    int16_t gyro[3] = {0, 0, 0};  // X, Y, Z
    int16_t accel[3] = {0, 0, 0}; // X, Y, Z

    // IF-0001 LAWN_IMU_RAW.gap: "samples lost immediately before this one,
    // 255 = saturated". Filled from the accumulator, never from a single
    // interval -- see SampleStream below.
    uint8_t gap = 0;

    // Monotonic count of samples published since boot, starting at 1. Not on
    // the wire; it exists so a consumer can tell "no new sample yet" from "a
    // new sample that happens to look the same".
    uint32_t sequence = 0;
};

// Assembles a signed 16-bit value from a little-endian pair.
//
// Written out as two's complement rather than casting uint16_t to int16_t,
// which is implementation-defined before C++20. Both compilers involved do the
// obvious thing today; this does not depend on that.
inline constexpr int16_t decodeInt16Le(uint8_t lo, uint8_t hi)
{
    const uint32_t raw = (static_cast<uint32_t>(hi) << 8) | static_cast<uint32_t>(lo);
    return static_cast<int16_t>(raw < 0x8000u ? static_cast<int32_t>(raw)
                                              : static_cast<int32_t>(raw) - 0x10000);
}

// Splits one burst into six signed values.
//
// Register order from OUTX_L_G (0x22) with IF_INC set: gyro X/Y/Z then accel
// X/Y/Z, each a little-endian low/high pair. Confirmed on hardware by
// diagnostics/imu-probe, which decoded the same twelve bytes the same way.
//
// Touches only the six output fields. stamp_us, gap and sequence are the
// caller's to set, so decoding cannot silently reset the bookkeeping.
inline constexpr void decodeBurst(const uint8_t (&raw)[burst_bytes], Sample* out)
{
    if (out == nullptr)
    {
        return;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        out->gyro[axis] = decodeInt16Le(raw[2 * axis], raw[2 * axis + 1]);
        out->accel[axis] = decodeInt16Le(raw[6 + 2 * axis], raw[6 + 2 * axis + 1]);
    }
}

// The published sample slot plus the gap bookkeeping that spans publications.
//
// **Why the gap accumulator is separate from the slot.** Publications and
// emissions are not one-for-one: samples land at 208 Hz and whatever emits
// LAWN_IMU_RAW runs slower, so most published samples are overwritten before
// anyone sees them. If each publication carried away only its own interval's
// losses, every loss that landed on an overwritten sample would vanish -- a
// stream that dropped a sample per period would report gap 0 forever, which is
// the silent interpolation ADR-0007 forbids. So losses accumulate here and are
// cleared by take(), which is the point at which a consumer has actually
// received them.
struct SampleStream
{
    Sample slot{};
    imu_stall::GapCounter gaps{};

    // Stamp of the last sample PUBLISHED, which is also the last one completed:
    // a burst that was started and never finished must not refresh this, or a
    // stalled stream would look fresh (imu_stall.hpp says the same).
    uint64_t last_stamp_us = 0;
    bool has_sample = false;

    uint32_t published = 0;
    uint32_t duplicates = 0;

    // Records a completed burst. Returns false if it was discarded.
    //
    // Discards a stamp that is not strictly newer than the last published one.
    // The DMA completion handler can be entered twice for one burst -- a
    // coalesced interrupt, or a completion acknowledged while the channel was
    // already re-armed -- and republishing the same bytes would invent a sample
    // that the sensor never produced. The stamp is the identity: it is latched
    // once per INT1 edge from a monotonic clock.
    bool onBurstComplete(uint64_t stampUs, const uint8_t (&raw)[burst_bytes], uint32_t periodUs)
    {
        if (has_sample && stampUs <= last_stamp_us)
        {
            ++duplicates;
            return false;
        }

        // Only against a PREVIOUS sample. The first sample after boot has no
        // predecessor, and measuring its interval from last_stamp_us == 0 would
        // charge the whole uptime to it and report an instant saturated gap.
        if (has_sample)
        {
            gaps.addForInterval(stampUs, last_stamp_us, periodUs);
        }

        decodeBurst(raw, &slot);
        slot.stamp_us = stampUs;
        slot.gap = gaps.pending; // mirrors the accumulator; take() clears both
        slot.sequence = ++published;

        last_stamp_us = stampUs;
        has_sample = true;
        return true;
    }

    // Hands the latest sample and the accumulated gap over, and clears the gap.
    //
    // Returns false, writing nothing, before the first sample has landed. A
    // consumer must not be handed a zeroed Sample it cannot distinguish from a
    // real one at rest.
    bool take(Sample* out)
    {
        if (out == nullptr || !has_sample)
        {
            return false;
        }
        *out = slot;
        out->gap = gaps.take();
        slot.gap = 0; // a second take() with no new sample must not re-report it
        return true;
    }

    // Reads the slot without consuming the gap, for a staleness check that only
    // wants the timestamp and must not eat an accounting the emitter needs.
    bool peek(Sample* out) const
    {
        if (out == nullptr || !has_sample)
        {
            return false;
        }
        *out = slot;
        return true;
    }
};

} // namespace imu_sample
