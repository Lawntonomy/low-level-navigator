// Host-side unit tests for the IMU sample slot: burst decoding and the gap
// accounting that spans a publish cycle.
//
// **These exist because a bench cannot see any of it.** The failures here all
// produce a wire message that is perfectly well formed and simply wrong: a
// sample decoded with its bytes the wrong way round still looks like motion, a
// gap cleared at the wrong moment still reports a number, and a duplicated
// completion still reports a sample. The stream on a scope looks identical in
// every case.
//
// imu_stall.hpp's own arithmetic -- the timeout, missedSamples(), the
// saturating counter -- is covered by test_imu_stall.cpp and is not repeated.
// What is tested here is the layer above it: WHEN that arithmetic is applied,
// and what happens to the result between publications.
//
// The hardware half -- the INT1 handler, the DMA completion handler, the two
// spinlocks, the arming order -- is not testable here and is not tested.

#include <cstdint>
#include <cstring>

#include <gtest/gtest.h>

#include "hardware_drivers/imu_sample.hpp"

namespace
{

constexpr uint32_t period = imu_stall::nominal_period_us; // 4807 us at 208 Hz

// A burst with a recognisable value on every axis, so a transposition or an
// endian slip shows up as a wrong number rather than a wrong sign.
//   gyro  X = 0x0201, Y = 0x0403, Z = 0x0605
//   accel X = 0x0807, Y = 0x0A09, Z = 0x0C0B
constexpr uint8_t counted_burst[imu_sample::burst_bytes] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                                            0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};

constexpr uint8_t zero_burst[imu_sample::burst_bytes] = {};

} // namespace

// --- Decoding -------------------------------------------------------------

TEST(ImuSampleDecode, LittleEndianPairsBecomeSignedValues)
{
    EXPECT_EQ(imu_sample::decodeInt16Le(0x00, 0x00), 0);
    EXPECT_EQ(imu_sample::decodeInt16Le(0x34, 0x12), 0x1234);
    EXPECT_EQ(imu_sample::decodeInt16Le(0xFF, 0x7F), 32767);
}

// The reason decodeInt16Le() writes the two's complement out by hand instead of
// casting a uint16_t. Gravity on a level machine is a large negative count on
// one accelerometer axis, so getting this wrong is not an edge case.
TEST(ImuSampleDecode, NegativeValuesAreNegative)
{
    EXPECT_EQ(imu_sample::decodeInt16Le(0xFF, 0xFF), -1);
    EXPECT_EQ(imu_sample::decodeInt16Le(0x00, 0x80), -32768);
    EXPECT_EQ(imu_sample::decodeInt16Le(0x18, 0xFC), -1000);
}

TEST(ImuSampleDecode, GyroComesBeforeAccelInRegisterOrder)
{
    imu_sample::Sample s;
    imu_sample::decodeBurst(counted_burst, &s);

    EXPECT_EQ(s.gyro[0], 0x0201);
    EXPECT_EQ(s.gyro[1], 0x0403);
    EXPECT_EQ(s.gyro[2], 0x0605);
    EXPECT_EQ(s.accel[0], 0x0807);
    EXPECT_EQ(s.accel[1], 0x0A09);
    EXPECT_EQ(s.accel[2], 0x0C0B);
}

// decodeBurst() must not touch the bookkeeping fields, or a decode would
// silently reset the sequence and the gap the caller is about to fill in.
TEST(ImuSampleDecode, LeavesTheBookkeepingFieldsAlone)
{
    imu_sample::Sample s;
    s.stamp_us = 12345;
    s.gap = 7;
    s.sequence = 99;

    imu_sample::decodeBurst(counted_burst, &s);

    EXPECT_EQ(s.stamp_us, 12345u);
    EXPECT_EQ(s.gap, 7);
    EXPECT_EQ(s.sequence, 99u);
}

// --- The publish cycle ----------------------------------------------------

TEST(ImuSampleStream, NothingIsHandedOutBeforeTheFirstSample)
{
    imu_sample::SampleStream stream;
    imu_sample::Sample out;

    EXPECT_FALSE(stream.take(&out));
    EXPECT_FALSE(stream.peek(&out));
}

TEST(ImuSampleStream, FirstSamplePublishesWithNoGap)
{
    imu_sample::SampleStream stream;

    // A large stamp on purpose: measuring the first sample's interval from
    // last_stamp_us == 0 would charge the whole uptime to it and report an
    // instantly saturated gap on a stream that has lost nothing.
    ASSERT_TRUE(stream.onBurstComplete(9'000'000, counted_burst, period));

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.stamp_us, 9'000'000u);
    EXPECT_EQ(out.gap, 0);
    EXPECT_EQ(out.sequence, 1u);
    EXPECT_EQ(out.gyro[0], 0x0201);
}

TEST(ImuSampleStream, SequenceCountsPublicationsNotCalls)
{
    imu_sample::SampleStream stream;
    uint64_t t = 1'000'000;

    for (int i = 0; i < 5; ++i)
    {
        ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
        t += period;
    }

    imu_sample::Sample out;
    ASSERT_TRUE(stream.peek(&out));
    EXPECT_EQ(out.sequence, 5u);
    EXPECT_EQ(stream.published, 5u);
}

TEST(ImuSampleStream, HealthyCadenceReportsNoGap)
{
    imu_sample::SampleStream stream;
    uint64_t t = 1'000'000;

    for (int i = 0; i < 20; ++i)
    {
        ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
        t += 4772; // the measured interval on this board, 0.75% fast of nominal
    }

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.gap, 0);
}

// --- Gap accumulation ACROSS a publish cycle ------------------------------
//
// The defect this group guards. Publications run at 208 Hz and whatever emits
// LAWN_IMU_RAW runs slower, so most published samples are overwritten before
// anyone sees them. If a publication carried away only its own interval's
// losses, every loss landing on an overwritten sample would vanish.

TEST(ImuSampleStream, LossesSurviveSamplesNobodyTook)
{
    imu_sample::SampleStream stream;
    uint64_t t = 1'000'000;

    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));

    // Three consecutive single-sample losses, none of them taken.
    for (int i = 0; i < 3; ++i)
    {
        t += 2 * period;
        ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
    }

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.gap, 3) << "losses on overwritten samples must not vanish";
}

TEST(ImuSampleStream, TakeClearsSoLossesAreNotCountedTwice)
{
    imu_sample::SampleStream stream;
    uint64_t t = 1'000'000;

    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
    t += 3 * period; // two samples lost
    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));

    imu_sample::Sample first;
    ASSERT_TRUE(stream.take(&first));
    EXPECT_EQ(first.gap, 2);

    // Same sample again, no new arrival: the losses have already been reported.
    imu_sample::Sample again;
    ASSERT_TRUE(stream.take(&again));
    EXPECT_EQ(again.gap, 0);
    EXPECT_EQ(again.sequence, first.sequence);

    // And a clean sample after the take reports a clean gap.
    t += period;
    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
    imu_sample::Sample third;
    ASSERT_TRUE(stream.take(&third));
    EXPECT_EQ(third.gap, 0);
}

// peek() is what a staleness check uses: it wants the timestamp and must not
// eat an accounting the emitter has not received yet.
TEST(ImuSampleStream, PeekDoesNotConsumeTheGap)
{
    imu_sample::SampleStream stream;
    uint64_t t = 1'000'000;

    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
    t += 2 * period;
    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));

    imu_sample::Sample peeked;
    ASSERT_TRUE(stream.peek(&peeked));
    EXPECT_EQ(peeked.gap, 1);
    ASSERT_TRUE(stream.peek(&peeked));
    EXPECT_EQ(peeked.gap, 1);

    imu_sample::Sample taken;
    ASSERT_TRUE(stream.take(&taken));
    EXPECT_EQ(taken.gap, 1) << "a peek must not have consumed it";
}

// A total outage must not fold round into a gap of 0, which would read on the
// Pi as a perfect uninterrupted stream. imu_stall.hpp saturates; this checks the
// saturation survives being accumulated one interval at a time.
TEST(ImuSampleStream, LongOutageSaturatesRatherThanWrapping)
{
    imu_sample::SampleStream stream;
    uint64_t t = 1'000'000;

    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
    t += 400 * period; // 399 samples lost in one go
    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.gap, imu_stall::gap_saturated);
}

TEST(ImuSampleStream, ManySmallLossesAlsoSaturate)
{
    imu_sample::SampleStream stream;
    uint64_t t = 1'000'000;

    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
    for (int i = 0; i < 300; ++i)
    {
        t += 2 * period;
        ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));
    }

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.gap, imu_stall::gap_saturated);
}

// --- Duplicate and out-of-order completions -------------------------------
//
// The DMA completion handler can be entered twice for one burst -- a coalesced
// interrupt, or a completion acknowledged while the channel was already
// re-armed. Republishing the same bytes would invent a sample the sensor never
// produced, and at a stamp that makes it look like the stream is healthy.

TEST(ImuSampleStream, RepeatedStampIsDiscarded)
{
    imu_sample::SampleStream stream;

    ASSERT_TRUE(stream.onBurstComplete(1'000'000, counted_burst, period));
    EXPECT_FALSE(stream.onBurstComplete(1'000'000, zero_burst, period));

    EXPECT_EQ(stream.published, 1u);
    EXPECT_EQ(stream.duplicates, 1u);

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.sequence, 1u);
    EXPECT_EQ(out.gyro[0], 0x0201) << "the discarded burst must not have overwritten the slot";
}

TEST(ImuSampleStream, BackwardsStampIsDiscarded)
{
    imu_sample::SampleStream stream;

    ASSERT_TRUE(stream.onBurstComplete(2'000'000, zero_burst, period));
    EXPECT_FALSE(stream.onBurstComplete(1'000'000, zero_burst, period));

    imu_sample::Sample out;
    ASSERT_TRUE(stream.peek(&out));
    EXPECT_EQ(out.stamp_us, 2'000'000u);
    EXPECT_EQ(stream.duplicates, 1u);
}

TEST(ImuSampleStream, DiscardedCompletionAddsNoGap)
{
    imu_sample::SampleStream stream;

    ASSERT_TRUE(stream.onBurstComplete(1'000'000, zero_burst, period));
    EXPECT_FALSE(stream.onBurstComplete(1'000'000, zero_burst, period));

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.gap, 0);
}

// The 64-bit stamp is what makes this a non-event. The same sequence on a
// 32-bit microsecond stamp wraps at 71.6 minutes, and every sample after the
// wrap would be discarded as "backwards" by the check above -- a stream that
// stops dead just under one and a quarter hours into a mow.
TEST(ImuSampleStream, KeepsPublishingPastThe32BitClockBoundary)
{
    imu_sample::SampleStream stream;

    uint64_t t = static_cast<uint64_t>(UINT32_MAX) - period;
    ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period));

    for (int i = 0; i < 4; ++i)
    {
        t += period;
        ASSERT_TRUE(stream.onBurstComplete(t, zero_burst, period)) << "sample " << i;
    }

    imu_sample::Sample out;
    ASSERT_TRUE(stream.take(&out));
    EXPECT_EQ(out.gap, 0);
    EXPECT_EQ(out.sequence, 5u);
    EXPECT_GT(out.stamp_us, static_cast<uint64_t>(UINT32_MAX));
}
