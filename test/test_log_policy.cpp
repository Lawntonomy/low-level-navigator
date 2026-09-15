// Host-side unit tests for IF-0002's content policy.
//
// These rules are the difference between a stream an engineer can read and
// 100 kB/s of noise, and every one of them is pure logic that a refactor can
// make vacuous without any bench session noticing -- a token bucket that always
// grants, a suppressor that never suppresses, and a floor comparison with the
// wrong sense all compile and all look plausible.
//
// A4 in particular is not a formatting preference: IF-0002 section 10.5 point 4
// makes it the bound on spinlock contention between a logging task on core 0
// and the control task on core 1. The tests that pin its rate are guarding a
// control-loop property.
//
// The hardware half -- the UART, the ring, the critical section -- is not
// testable here and is not tested.

#include <gtest/gtest.h>

#include "app/log_policy.hpp"

using log_policy::Cat;
using log_policy::Level;
using log_policy::LevelTable;
using log_policy::RepeatSuppressor;
using log_policy::TokenBucket;

namespace
{
// Helper: feed a line to the suppressor by C string.
RepeatSuppressor::Decision feed(RepeatSuppressor& s, const char* line, uint32_t now_ms)
{
    return s.consider(line, strlen(line), now_ms);
}
} // namespace

// --------------------------------------------------------------------------
// Category resolution
// --------------------------------------------------------------------------

TEST(LogPolicyCat, ResolvesTheFixedSet)
{
    EXPECT_EQ(log_policy::resolveCat("log"), Cat::log);
    EXPECT_EQ(log_policy::resolveCat("sys"), Cat::sys);
    EXPECT_EQ(log_policy::resolveCat("saf"), Cat::saf);
    EXPECT_EQ(log_policy::resolveCat("whl"), Cat::whl);
    EXPECT_EQ(log_policy::resolveCat("pid"), Cat::pid);
    EXPECT_EQ(log_policy::resolveCat("mot"), Cat::mot);
    EXPECT_EQ(log_policy::resolveCat("imu"), Cat::imu);
    EXPECT_EQ(log_policy::resolveCat("lnk"), Cat::lnk);
    EXPECT_EQ(log_policy::resolveCat("boot"), Cat::boot);
}

// The reason the drivers did not all have to collapse onto three-letter names.
TEST(LogPolicyCat, SubTokensBucketOnTheirSlot)
{
    EXPECT_EQ(log_policy::resolveCat("imu.i2c"), Cat::imu);
    EXPECT_EQ(log_policy::resolveCat("imu.drdy"), Cat::imu);
    EXPECT_EQ(log_policy::resolveCat("whl.enc"), Cat::whl);
}

// An unrecognised token must cost a bucket slot and a counter, never a line.
TEST(LogPolicyCat, UnknownFallsToOtherRatherThanBeingDropped)
{
    EXPECT_EQ(log_policy::resolveCat("nonsense"), Cat::other);
    EXPECT_EQ(log_policy::resolveCat(""), Cat::other);
    EXPECT_EQ(log_policy::resolveCat(nullptr), Cat::other);
}

// "imu" must not match a token that merely starts with it.
TEST(LogPolicyCat, DoesNotPrefixMatchAcrossTheWholeToken)
{
    EXPECT_EQ(log_policy::resolveCat("imux"), Cat::other);
    EXPECT_EQ(log_policy::resolveCat("im"), Cat::other);
}

// --------------------------------------------------------------------------
// Severity floors
// --------------------------------------------------------------------------

TEST(LogPolicyFloor, EmitsAtAndAboveTheFloor)
{
    LevelTable t(Level::info);
    EXPECT_FALSE(t.enabled(Cat::imu, Level::trace));
    EXPECT_FALSE(t.enabled(Cat::imu, Level::debug));
    EXPECT_TRUE(t.enabled(Cat::imu, Level::info));
    EXPECT_TRUE(t.enabled(Cat::imu, Level::warn));
    EXPECT_TRUE(t.enabled(Cat::imu, Level::error));
}

TEST(LogPolicyFloor, IsPerCategory)
{
    LevelTable t(Level::info);
    t.setFloor(Cat::imu, Level::debug, 0, 0);

    EXPECT_TRUE(t.enabled(Cat::imu, Level::debug));
    EXPECT_FALSE(t.enabled(Cat::saf, Level::debug)) << "raising one category must not raise others";
}

// The near-term direction, per IF-0002 section 11.1: at a debug default the
// useful operation is muting a chatty category, not raising a quiet one.
TEST(LogPolicyFloor, CanSilenceOneCategoryBelowTheBuildDefault)
{
    LevelTable t(Level::debug);
    t.setFloor(Cat::imu, Level::warn, 0, 0);

    EXPECT_FALSE(t.enabled(Cat::imu, Level::debug));
    EXPECT_FALSE(t.enabled(Cat::imu, Level::info));
    EXPECT_TRUE(t.enabled(Cat::imu, Level::warn));
    EXPECT_TRUE(t.enabled(Cat::saf, Level::debug)) << "other categories keep the build default";
}

TEST(LogPolicyFloor, OffSuppressesEverythingIncludingError)
{
    LevelTable t(Level::info);
    t.setFloor(Cat::imu, Level::off, 0, 0);
    EXPECT_FALSE(t.enabled(Cat::imu, Level::error));
}

// --------------------------------------------------------------------------
// TTL -- the rule that stops a bench session degrading the stream forever
// --------------------------------------------------------------------------

TEST(LogPolicyTtl, RevertsToBuildDefaultAfterExpiry)
{
    LevelTable t(Level::info);
    t.setFloor(Cat::imu, Level::trace, 30, 1000);

    EXPECT_EQ(t.expire(20000), 0u);
    EXPECT_TRUE(t.enabled(Cat::imu, Level::trace)) << "still inside the TTL";

    EXPECT_EQ(t.expire(31000), 1u);
    EXPECT_FALSE(t.enabled(Cat::imu, Level::trace));
    EXPECT_EQ(t.floor(Cat::imu), Level::info);
}

TEST(LogPolicyTtl, ZeroMeansNoExpiry)
{
    LevelTable t(Level::info);
    t.setFloor(Cat::imu, Level::trace, 0, 1000);

    EXPECT_EQ(t.expire(1000u * 1000u * 1000u), 0u);
    EXPECT_TRUE(t.enabled(Cat::imu, Level::trace));
}

TEST(LogPolicyTtl, ExpiryIsIdempotentAndReportsOnlyTheTransition)
{
    LevelTable t(Level::info);
    t.setFloor(Cat::imu, Level::trace, 1, 0);

    EXPECT_EQ(t.expire(2000), 1u) << "the sweep that reverts it reports one";
    EXPECT_EQ(t.expire(3000), 0u) << "later sweeps must report nothing, or the event logs forever";
}

// A machine up for ~49.7 days wraps the 32-bit ms clock. Reverting a TTL a
// sweep early there is fine; never reverting it again is not.
TEST(LogPolicyTtl, SurvivesTheMillisecondWrap)
{
    LevelTable t(Level::info);
    const uint32_t near_wrap = 0xFFFFF000u;
    t.setFloor(Cat::imu, Level::trace, 30, near_wrap); // expiry wraps past zero

    EXPECT_EQ(t.expire(near_wrap + 1000u), 0u) << "not yet due";
    EXPECT_EQ(t.expire(near_wrap + 31000u), 1u) << "due, across the wrap";
    EXPECT_EQ(t.floor(Cat::imu), Level::info);
}

// --------------------------------------------------------------------------
// A3 -- repeat suppression
// --------------------------------------------------------------------------

TEST(LogPolicyRepeat, FirstLineIsAlwaysEmitted)
{
    RepeatSuppressor s;
    const auto d = feed(s, "I 10 imu something", 10);
    EXPECT_TRUE(d.emit_line);
    EXPECT_EQ(d.flush_count, 0u);
}

TEST(LogPolicyRepeat, CollapsesARunAndReportsItWhenItBreaks)
{
    RepeatSuppressor s;
    EXPECT_TRUE(feed(s, "A", 10).emit_line);

    EXPECT_FALSE(feed(s, "A", 11).emit_line);
    EXPECT_FALSE(feed(s, "A", 12).emit_line);

    const auto d = feed(s, "B", 13);
    EXPECT_TRUE(d.emit_line);
    EXPECT_EQ(d.flush_count, 2u) << "the two suppressed A lines are accounted for";
    EXPECT_EQ(d.flush_first_ms, 11u) << "stamped at the first SUPPRESSED line, not the emitted one";
}

TEST(LogPolicyRepeat, DifferentLinesAreNeverSuppressed)
{
    RepeatSuppressor s;
    EXPECT_TRUE(feed(s, "A", 10).emit_line);
    EXPECT_TRUE(feed(s, "B", 11).emit_line);
    EXPECT_TRUE(feed(s, "A", 12).emit_line) << "suppression is against the LAST line only";
}

// A line differing only in its trailing bytes must not be treated as a repeat.
// This is what an exact comparison buys over a hash.
TEST(LogPolicyRepeat, DistinguishesLinesThatDifferLate)
{
    RepeatSuppressor s;
    EXPECT_TRUE(feed(s, "I 10 imu edges=1000 published=1000", 10).emit_line);
    EXPECT_TRUE(feed(s, "I 10 imu edges=1000 published=1001", 11).emit_line);
}

TEST(LogPolicyRepeat, DistinguishesAPrefixFromItsLongerLine)
{
    RepeatSuppressor s;
    EXPECT_TRUE(feed(s, "AB", 10).emit_line);
    EXPECT_TRUE(feed(s, "ABC", 11).emit_line) << "equal-prefix, different length, is not a repeat";
}

// A run that never breaks would otherwise never be reported: a wedged machine
// repeating one line forever would look silent.
TEST(LogPolicyRepeat, FlushesAStuckRunOnTheWindow)
{
    RepeatSuppressor s;
    EXPECT_TRUE(feed(s, "A", 0).emit_line);

    for (uint32_t t = 1; t < RepeatSuppressor::flush_window_ms; t++)
    {
        const auto d = feed(s, "A", t);
        ASSERT_FALSE(d.emit_line);
        ASSERT_EQ(d.flush_count, 0u) << "no flush before the window elapses, at t=" << t;
    }

    const auto d = feed(s, "A", RepeatSuppressor::flush_window_ms + 1);
    EXPECT_FALSE(d.emit_line) << "the repeated line itself still does not go out";
    EXPECT_GT(d.flush_count, 0u) << "but the run is reported";
}

TEST(LogPolicyRepeat, StuckRunReportsRepeatedlyRatherThanOnlyOnce)
{
    RepeatSuppressor s;
    feed(s, "A", 0);

    uint32_t flushes = 0;
    for (uint32_t t = 1; t <= 5000; t++)
    {
        if (feed(s, "A", t).flush_count > 0)
        {
            flushes++;
        }
    }
    EXPECT_GE(flushes, 4u) << "roughly one report per window, not one report ever";
}

// --------------------------------------------------------------------------
// A4 -- the token bucket, and the rate it actually enforces
// --------------------------------------------------------------------------

TEST(LogPolicyBucket, GrantsTheBurstThenDenies)
{
    TokenBucket b(4, 20);
    for (int i = 0; i < 4; i++)
    {
        EXPECT_TRUE(b.take(0)) << "burst token " << i;
    }
    EXPECT_FALSE(b.take(0)) << "burst exhausted within the same millisecond";
    EXPECT_EQ(b.denied(), 1u);
}

TEST(LogPolicyBucket, RefillsAtOneTokenPerConfiguredInterval)
{
    TokenBucket b(4, 20);
    for (int i = 0; i < 4; i++)
    {
        ASSERT_TRUE(b.take(0));
    }
    EXPECT_FALSE(b.take(19)) << "not yet one interval";
    EXPECT_TRUE(b.take(20)) << "one interval has elapsed";
    EXPECT_FALSE(b.take(20)) << "and only one token was returned";
}

// The sustained rate is the property the safety argument rests on: it is what
// bounds how often push() can contend the control task's spinlock.
TEST(LogPolicyBucket, SustainedRateIsOneThousandOverMsPerToken)
{
    TokenBucket b(1, 20); // capacity 1 removes the burst from the measurement
    ASSERT_TRUE(b.take(0));

    uint32_t granted = 0;
    for (uint32_t t = 1; t <= 1000; t++)
    {
        if (b.take(t))
        {
            granted++;
        }
    }
    // 1000 ms at 20 ms per token.
    EXPECT_EQ(granted, 50u);
}

TEST(LogPolicyBucket, DoesNotBankAnUnboundedBurstWhileIdle)
{
    TokenBucket b(4, 20);
    // Idle for a long time, then ask for far more than the capacity.
    uint32_t granted = 0;
    for (int i = 0; i < 100; i++)
    {
        if (b.take(60000))
        {
            granted++;
        }
    }
    EXPECT_EQ(granted, 4u) << "an idle category must not accumulate credit beyond its capacity";
}

TEST(LogPolicyBucket, CountsEveryDenialSoNothingIsLostSilently)
{
    TokenBucket b(1, 20);
    ASSERT_TRUE(b.take(0));
    for (int i = 0; i < 7; i++)
    {
        EXPECT_FALSE(b.take(1));
    }
    EXPECT_EQ(b.denied(), 7u);
}

// A category that has never been used must not be treated as having been idle
// since boot and immediately over-refilled.
TEST(LogPolicyBucket, FirstTakeDoesNotDependOnTheStartTimestamp)
{
    TokenBucket b(2, 20);
    EXPECT_TRUE(b.take(500000));
    EXPECT_TRUE(b.take(500000));
    EXPECT_FALSE(b.take(500000));
}
