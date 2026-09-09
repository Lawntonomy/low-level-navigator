// Host-side unit tests for the IMU stall timeout and gap accounting.
//
// These exist because both halves fail silently on hardware. A timeout that is
// off by a factor, or that underflows on an out-of-order timestamp, produces a
// driver that either never reports a stall or reports one constantly -- and a
// gap counter that wraps turns a total stream outage into a message that claims
// nothing was lost. Neither is visible on a bench: the wire still carries
// well-formed samples.
//
// The hardware half (i2c0, the DMA channel, the FIFO-direct command words) is
// not testable here and is not tested.

#include <cstdint>

#include <gtest/gtest.h>

#include "hardware_drivers/imu_stall.hpp"

namespace
{

constexpr uint32_t kPeriodUs = imu_stall::nominal_period_us; // 4807 at 208 Hz

} // namespace

// --- Cadence --------------------------------------------------------------

TEST(ImuStallCadence, PeriodMatchesTheConfiguredOdr)
{
    EXPECT_EQ(imu_stall::periodUsForOdr(208), 4807u);
    EXPECT_EQ(imu_stall::periodUsForOdr(104), 9615u);
    EXPECT_EQ(imu_stall::periodUsForOdr(1000000), 1u);
}

TEST(ImuStallCadence, ZeroOdrDoesNotDivideByZero)
{
    EXPECT_EQ(imu_stall::periodUsForOdr(0), 0u);
}

TEST(ImuStallCadence, NominalPeriodIsDerivedFromNominalOdr)
{
    EXPECT_EQ(imu_stall::nominal_period_us, imu_stall::periodUsForOdr(imu_stall::nominal_odr_hz));
}

// --- Timeout arithmetic ---------------------------------------------------

TEST(ImuStallTimeout, TimeoutIsPeriodTimesCount)
{
    EXPECT_EQ(imu_stall::stallTimeoutUs(kPeriodUs, 3), 3u * kPeriodUs);
    EXPECT_EQ(imu_stall::stallTimeoutUs(kPeriodUs), imu_stall::stall_periods * kPeriodUs);
}

TEST(ImuStallTimeout, TimeoutSaturatesInsteadOfWrapping)
{
    // 3 x 2e9 us overflows uint32. A wrapped product would yield a tiny timeout
    // and a detector that fires on every evaluation.
    EXPECT_EQ(imu_stall::stallTimeoutUs(2000000000u, 3), UINT32_MAX);
    EXPECT_EQ(imu_stall::stallTimeoutUs(UINT32_MAX, 2), UINT32_MAX);
}

TEST(ImuStallTimeout, AnUnknownCadenceReadsAsStalled)
{
    // periodUs == 0 means the ODR was never established. The file's stated
    // policy is that the unsafe direction is a missed stall, so an unknown
    // cadence must not read as healthy.
    EXPECT_EQ(imu_stall::stallTimeoutUs(0), 0u);
    EXPECT_TRUE(imu_stall::isStalled(1000, 1000 - 1, imu_stall::stallTimeoutUs(0)));
}

// --- isStalled ------------------------------------------------------------

TEST(ImuStallDetector, FreshStreamIsNotStalled)
{
    const uint32_t timeout = imu_stall::stallTimeoutUs(kPeriodUs);
    const uint64_t last = 5'000'000;

    EXPECT_FALSE(imu_stall::isStalled(last, last, timeout));
    EXPECT_FALSE(imu_stall::isStalled(last + kPeriodUs, last, timeout));
    EXPECT_FALSE(imu_stall::isStalled(last + 2 * kPeriodUs, last, timeout));
}

TEST(ImuStallDetector, BoundaryIsInclusive)
{
    const uint32_t timeout = imu_stall::stallTimeoutUs(kPeriodUs);
    const uint64_t last = 5'000'000;

    EXPECT_FALSE(imu_stall::isStalled(last + timeout - 1, last, timeout));
    EXPECT_TRUE(imu_stall::isStalled(last + timeout, last, timeout));
    EXPECT_TRUE(imu_stall::isStalled(last + timeout + 1, last, timeout));
}

TEST(ImuStallDetector, LongSilenceIsStalled)
{
    const uint32_t timeout = imu_stall::stallTimeoutUs(kPeriodUs);
    // The failure this is really for: INT1 latched high, never re-armed, and
    // every DMA register still reporting a healthy idle channel.
    EXPECT_TRUE(imu_stall::isStalled(5'000'000 + 1'000'000, 5'000'000, timeout));
}

TEST(ImuStallDetector, TimestampFromTheFutureDoesNotUnderflow)
{
    const uint32_t timeout = imu_stall::stallTimeoutUs(kPeriodUs);
    // An unguarded subtraction here reads as a ~584,000-year gap and trips the
    // timeout instantly on what is really a one-microsecond clock anomaly.
    EXPECT_FALSE(imu_stall::isStalled(1'000'000, 1'000'001, timeout));
    EXPECT_FALSE(imu_stall::isStalled(0, UINT64_MAX, timeout));
}

TEST(ImuStallDetector, WorksAcrossThe32BitClockBoundary)
{
    // The timestamps are 64-bit microseconds. A value past 2^32 must behave
    // exactly like any other, so nothing here can be narrowed to uint32.
    const uint32_t timeout = imu_stall::stallTimeoutUs(kPeriodUs);
    const uint64_t last = 5'000'000'000; // ~83 minutes of uptime

    EXPECT_FALSE(imu_stall::isStalled(last + kPeriodUs, last, timeout));
    EXPECT_TRUE(imu_stall::isStalled(last + 10 * kPeriodUs, last, timeout));
}

// --- missedSamples --------------------------------------------------------

TEST(ImuStallGaps, PerfectCadenceLosesNothing)
{
    EXPECT_EQ(imu_stall::missedSamples(kPeriodUs, 0, kPeriodUs), 0u);
    EXPECT_EQ(imu_stall::missedSamples(0, 0, kPeriodUs), 0u);
}

TEST(ImuStallGaps, JitterShortOfHalfAPeriodLosesNothing)
{
    // Rounds to the nearest slot, so tolerance is half a period either way, not
    // a whole one. This test previously asserted that 1.9 periods reported 0 --
    // which looked like generous jitter tolerance and was in fact the bug: the
    // divisor is the NOMINAL period while the part runs 0.75% faster, so a real
    // single-sample loss (2 x 4772 = 9544 us against a 4807 us divisor) landed
    // at 1.98 slots and was swallowed by exactly that tolerance.
    EXPECT_EQ(imu_stall::missedSamples((kPeriodUs * 14) / 10, 0, kPeriodUs), 0u);
    EXPECT_EQ(imu_stall::missedSamples((kPeriodUs * 16) / 10, 0, kPeriodUs), 1u);
}

TEST(ImuStallGaps, RealSingleLossAtMeasuredCadenceIsNotSwallowed)
{
    // The regression that motivated the rounding change, in its own units.
    // Measured period on this board is 4772 us (bench log 2026-09-09); the
    // divisor is the nominal 4807 us. One genuinely lost sample must report 1,
    // and a healthy interval must still report 0.
    constexpr uint32_t kMeasuredUs = 4772;
    EXPECT_EQ(imu_stall::missedSamples(kMeasuredUs, 0, imu_stall::nominal_period_us), 0u);
    EXPECT_EQ(imu_stall::missedSamples(2 * kMeasuredUs, 0, imu_stall::nominal_period_us), 1u);
    EXPECT_EQ(imu_stall::missedSamples(3 * kMeasuredUs, 0, imu_stall::nominal_period_us), 2u);
}

TEST(ImuStallThreshold, DerivedFromWorstCasePeriodNotNominal)
{
    // T_imu_stale = 2 x T_s_worst + D. Guards the arithmetic, not the inputs:
    // if the ODR or the assumed oscillator bound changes, this recomputes.
    EXPECT_GT(imu_stall::worst_case_period_us, imu_stall::nominal_period_us);
    EXPECT_EQ(imu_stall::stall_timeout_us,
              2u * imu_stall::worst_case_period_us + imu_stall::burst_delay_us);

    // The floor the derivation rests on: the largest age a HEALTHY stream can
    // show is one worst-case period plus the burst delay, so the threshold must
    // sit strictly above it or it fires on a working sensor.
    EXPECT_GT(imu_stall::stall_timeout_us,
              imu_stall::worst_case_period_us + imu_stall::burst_delay_us);
}

TEST(ImuStallGaps, EachWholeSkippedPeriodIsOneLostSample)
{
    EXPECT_EQ(imu_stall::missedSamples(2 * kPeriodUs, 0, kPeriodUs), 1u);
    EXPECT_EQ(imu_stall::missedSamples(3 * kPeriodUs, 0, kPeriodUs), 2u);
    EXPECT_EQ(imu_stall::missedSamples(10 * kPeriodUs, 0, kPeriodUs), 9u);
}

TEST(ImuStallGaps, UnknownCadenceCountsNothingRatherThanInventing)
{
    EXPECT_EQ(imu_stall::missedSamples(1'000'000, 0, 0), 0u);
}

TEST(ImuStallGaps, OutOfOrderTimestampsCountNothing)
{
    EXPECT_EQ(imu_stall::missedSamples(0, 1'000'000, kPeriodUs), 0u);
}

// --- Saturation, the defect this file exists for --------------------------

TEST(ImuStallGapCounter, CountsBelowTheCeiling)
{
    imu_stall::GapCounter gaps;
    EXPECT_EQ(gaps.pending, 0);

    gaps.add(3);
    gaps.add(4);
    EXPECT_EQ(gaps.pending, 7);
}

TEST(ImuStallGapCounter, SaturatesAtTwoHundredAndFiftyFive)
{
    EXPECT_EQ(imu_stall::saturatingAddGap(0, 255), 255);
    EXPECT_EQ(imu_stall::saturatingAddGap(0, 256), 255);
    EXPECT_EQ(imu_stall::saturatingAddGap(200, 100), 255);
    EXPECT_EQ(imu_stall::saturatingAddGap(255, 1), 255);
    EXPECT_EQ(imu_stall::saturatingAddGap(255, UINT32_MAX), 255);
}

TEST(ImuStallGapCounter, NeverWraps)
{
    // The defect in one test. An unguarded uint8_t add reports 256 lost samples
    // as 0 -- a total outage that reads on the Pi as an unbroken stream.
    imu_stall::GapCounter gaps;
    for (int i = 0; i < 1000; ++i)
    {
        gaps.add(1);
        EXPECT_NE(gaps.pending, 0);
    }
    EXPECT_EQ(gaps.pending, imu_stall::gap_saturated);

    // And once pegged it stays pegged, however much more is lost.
    gaps.add(UINT32_MAX);
    EXPECT_EQ(gaps.pending, imu_stall::gap_saturated);
}

TEST(ImuStallGapCounter, LargeSingleAdditionSaturatesRatherThanTruncating)
{
    // A naive static_cast<uint8_t>(lost) would turn 512 lost samples into 0.
    imu_stall::GapCounter gaps;
    gaps.add(512);
    EXPECT_EQ(gaps.pending, imu_stall::gap_saturated);
}

TEST(ImuStallGapCounter, TakeReportsAndClearsSoLossesAreNotDoubleCounted)
{
    imu_stall::GapCounter gaps;
    gaps.add(5);

    EXPECT_EQ(gaps.take(), 5);
    EXPECT_EQ(gaps.take(), 0);
    EXPECT_EQ(gaps.pending, 0);

    gaps.add(2);
    EXPECT_EQ(gaps.take(), 2);
}

TEST(ImuStallGapCounter, IntervalAccumulationMatchesMissedSamples)
{
    imu_stall::GapCounter gaps;
    gaps.addForInterval(3 * kPeriodUs, 0, kPeriodUs);             // 2 lost
    gaps.addForInterval(5 * kPeriodUs, 3 * kPeriodUs, kPeriodUs); // 1 lost
    EXPECT_EQ(gaps.take(), 3);
}
