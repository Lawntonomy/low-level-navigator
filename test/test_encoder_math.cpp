// Host-side unit tests for the encoder period-to-RPM conversion.
//
// These exist because the conversion is the part of the encoder path that can
// be silently wrong: a factor of 20 or a factor of 1000 in the wrong place
// produces a plausible-looking speed that no bench session will flag. The
// hardware half (PIO, DMA, freshness) is not testable here and is not tested.

#include <cmath>

#include <gtest/gtest.h>

#include "hardware_drivers/encoder_math.hpp"

namespace
{
constexpr std::size_t kDepth = 8; // CAPTURE_DEPTH in encoder.cpp

// The formula as it was written inline in encoder.cpp before extraction, so a
// refactor that changes the numbers cannot pass unnoticed.
float legacy_rpm(const uint32_t* samples, std::size_t count)
{
    float total = 0.0f;
    for (std::size_t i = 0; i < count; ++i)
    {
        total += static_cast<float>(samples[i]);
    }
    if (total > 0.0f)
    {
        return (60.0f * 1000.0f) / ((total / static_cast<float>(count)) * 20);
    }
    return 0.0f;
}
} // namespace

// --------------------------------------------------------------------------
// Mean of the capture ring
// --------------------------------------------------------------------------

TEST(EncoderMath, MeanOfUniformBuffer)
{
    const uint32_t samples[kDepth] = {10, 10, 10, 10, 10, 10, 10, 10};
    EXPECT_FLOAT_EQ(encoder::mean_period_counts(samples, kDepth), 10.0f);
}

TEST(EncoderMath, MeanOfMixedBuffer)
{
    const uint32_t samples[kDepth] = {4, 6, 4, 6, 4, 6, 4, 6};
    EXPECT_FLOAT_EQ(encoder::mean_period_counts(samples, kDepth), 5.0f);
}

TEST(EncoderMath, MeanOfUnwrittenBufferIsZero)
{
    const uint32_t samples[kDepth] = {0, 0, 0, 0, 0, 0, 0, 0};
    EXPECT_FLOAT_EQ(encoder::mean_period_counts(samples, kDepth), 0.0f);
}

TEST(EncoderMath, MeanRejectsNullAndEmpty)
{
    const uint32_t samples[kDepth] = {1, 1, 1, 1, 1, 1, 1, 1};
    EXPECT_FLOAT_EQ(encoder::mean_period_counts(nullptr, kDepth), 0.0f);
    EXPECT_FLOAT_EQ(encoder::mean_period_counts(samples, 0), 0.0f);
}

// --------------------------------------------------------------------------
// Period to RPM, worked by hand
// --------------------------------------------------------------------------

TEST(EncoderMath, OneCountPerMillisecondGivesKnownRpm)
{
    // 1000 counts per edge at 1000 counts/second = 1 s per edge.
    // 20 edges per revolution = 20 s per revolution = 3 rpm.
    EXPECT_FLOAT_EQ(encoder::rpm_from_mean_period(1000.0f), 3.0f);
}

TEST(EncoderMath, TenMillisecondsPerEdgeGivesThreeHundredRpm)
{
    // 10 ms per edge x 20 edges = 200 ms per revolution = 5 rev/s = 300 rpm.
    EXPECT_FLOAT_EQ(encoder::rpm_from_mean_period(10.0f), 300.0f);
}

TEST(EncoderMath, LongerPeriodMeansSlowerWheel)
{
    EXPECT_GT(encoder::rpm_from_mean_period(10.0f), encoder::rpm_from_mean_period(20.0f));
    EXPECT_FLOAT_EQ(encoder::rpm_from_mean_period(20.0f),
                    encoder::rpm_from_mean_period(10.0f) / 2.0f);
}

TEST(EncoderMath, ZeroMeanYieldsZeroNotInfinity)
{
    const float rpm = encoder::rpm_from_mean_period(0.0f);
    EXPECT_FLOAT_EQ(rpm, 0.0f);
    EXPECT_TRUE(std::isfinite(rpm));
}

TEST(EncoderMath, NegativeMeanYieldsZeroNotNegativeSpeed)
{
    // Cannot arise from an unsigned buffer, but the guard must not invert.
    EXPECT_FLOAT_EQ(encoder::rpm_from_mean_period(-5.0f), 0.0f);
}

// --------------------------------------------------------------------------
// Extraction did not change the numbers
// --------------------------------------------------------------------------

TEST(EncoderMath, MatchesTheFormulaItReplaced)
{
    const uint32_t fast[kDepth] = {3, 3, 4, 3, 3, 4, 3, 3};
    const uint32_t slow[kDepth] = {900, 1000, 1100, 950, 1050, 900, 1000, 1100};
    const uint32_t unwritten[kDepth] = {0, 0, 0, 0, 0, 0, 0, 0};

    EXPECT_FLOAT_EQ(encoder::rpm_from_periods(fast, kDepth), legacy_rpm(fast, kDepth));
    EXPECT_FLOAT_EQ(encoder::rpm_from_periods(slow, kDepth), legacy_rpm(slow, kDepth));
    EXPECT_FLOAT_EQ(encoder::rpm_from_periods(unwritten, kDepth), legacy_rpm(unwritten, kDepth));
}

// --------------------------------------------------------------------------
// The boundary this header does not cross
// --------------------------------------------------------------------------

TEST(EncoderMath, StaleSamplesStillReportSpeedByDesign)
{
    // Issue #12: when a wheel stops, the ring keeps its last values and this
    // function keeps converting them. That is correct behaviour *here* — the
    // arithmetic has no notion of time and must not invent one. Detecting that
    // no new sample has landed is the caller's job, via the DMA channel.
    // This test exists so that boundary is asserted rather than assumed.
    const uint32_t last_samples_before_stopping[kDepth] = {5, 5, 5, 5, 5, 5, 5, 5};
    EXPECT_GT(encoder::rpm_from_periods(last_samples_before_stopping, kDepth), 0.0f);
}

// --------------------------------------------------------------------------
// Freshness — the half of CAL-0 that makes "stopped" observable
// --------------------------------------------------------------------------

TEST(EncoderMath, SampleJustArrivedIsFresh)
{
    EXPECT_TRUE(encoder::is_fresh(1'000'000, 1'000'000));
    EXPECT_TRUE(encoder::is_fresh(1'000'001, 1'000'000));
}

TEST(EncoderMath, JustInsideTStaleIsFresh)
{
    EXPECT_TRUE(encoder::is_fresh(0 + encoder::t_stale_us - 1, 0));
}

TEST(EncoderMath, ExactlyTStaleIsStale)
{
    // Boundary is closed on the stale side: at exactly t_stale the wheel has
    // failed to produce an edge for the whole window, which is the definition.
    EXPECT_FALSE(encoder::is_fresh(0 + encoder::t_stale_us, 0));
}

TEST(EncoderMath, LongSilenceIsStale)
{
    EXPECT_FALSE(encoder::is_fresh(60'000'000, 0));
}

TEST(EncoderMath, ClockGoingBackwardsDoesNotDeclareAMovingWheelStopped)
{
    // safety.hpp documents this trap: an unguarded unsigned subtraction with
    // `now` older than the stamp reads as a multi-thousand-year gap and trips
    // every timeout at once. Declaring a moving wheel stopped is the dangerous
    // direction, so a clock anomaly must resolve to fresh.
    EXPECT_TRUE(encoder::is_fresh(500, 1'000'000));
}

TEST(EncoderMath, StaleWindowIsOverridable)
{
    EXPECT_TRUE(encoder::is_fresh(50, 0, 100));
    EXPECT_FALSE(encoder::is_fresh(150, 0, 100));
}

TEST(EncoderMath, TStalePlaceholderImpliesThreeRpmFloor)
{
    // t_stale sets the slowest resolvable speed by construction: a wheel
    // turning slower than one edge per t_stale cannot be told from a stopped
    // one. If this test fails, t_stale was changed — the new number below is
    // the speed floor that change just chose, and it belongs in the record.
    const float counts_at_t_stale =
        (static_cast<float>(encoder::t_stale_us) / 1e6f) * encoder::counts_per_second;
    EXPECT_FLOAT_EQ(encoder::rpm_from_mean_period(counts_at_t_stale), 3.0f);
}
