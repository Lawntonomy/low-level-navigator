// Host-side unit tests for log_detail::Ring.
//
// This is the ring behind the USB verbose-log path (src/utility/logger.h):
// producers push whole formatted lines and a single low-priority task drains
// them to printf(). Everything interesting about "drop instead of block" and
// "never emit a truncated line" lives in this class's push/pop bookkeeping,
// which is exactly the kind of arithmetic a bench session will never exercise
// on purpose. The concurrency half (the FreeRTOS critical section around each
// call, in logger.cpp) is not testable here and is not tested.

#include <cstring>
#include <string>

#include <gtest/gtest.h>

#include "utility/logger.h"

namespace
{
using TinyRing = log_detail::Ring<8>; // capacity 8 -> 7 usable bytes

std::string pop_all(TinyRing& ring)
{
    std::string out;
    char c;
    while (ring.pop(&c))
    {
        out.push_back(c);
    }
    return out;
}
} // namespace

// --------------------------------------------------------------------------
// Empty ring
// --------------------------------------------------------------------------

TEST(LoggerRing, StartsEmpty)
{
    TinyRing ring;
    char c;
    EXPECT_EQ(ring.used(), 0u);
    EXPECT_FALSE(ring.pop(&c));
    EXPECT_EQ(ring.dropped(), 0u);
    EXPECT_EQ(ring.peak_bytes(), 0u);
}

// --------------------------------------------------------------------------
// Basic push / pop, and whole-message discipline on the happy path
// --------------------------------------------------------------------------

TEST(LoggerRing, PushThenPopReturnsExactBytesInOrder)
{
    TinyRing ring;
    EXPECT_TRUE(ring.push("abc", 3));
    EXPECT_EQ(ring.used(), 3u);
    EXPECT_EQ(pop_all(ring), "abc");
    EXPECT_EQ(ring.used(), 0u);
}

TEST(LoggerRing, ZeroLengthPushIsANoOp)
{
    TinyRing ring;
    EXPECT_TRUE(ring.push("", 0));
    EXPECT_EQ(ring.used(), 0u);
    EXPECT_EQ(ring.dropped(), 0u);
}

TEST(LoggerRing, MultipleMessagesPreserveOrder)
{
    TinyRing ring;
    ASSERT_TRUE(ring.push("ab", 2));
    ASSERT_TRUE(ring.push("cd", 2));
    EXPECT_EQ(pop_all(ring), "abcd");
}

// --------------------------------------------------------------------------
// Fill: capacity is Capacity-1 usable bytes, matching link.cpp's TX ring and
// log_console's console ring (one slot always kept empty to distinguish full
// from empty without a separate counter).
// --------------------------------------------------------------------------

TEST(LoggerRing, FillsToCapacityMinusOne)
{
    TinyRing ring; // capacity 8, 7 usable
    EXPECT_TRUE(ring.push("1234567", 7));
    EXPECT_EQ(ring.used(), 7u);
    EXPECT_EQ(pop_all(ring), "1234567");
}

TEST(LoggerRing, PeakBytesTracksTheHighWaterMark)
{
    TinyRing ring;
    ASSERT_TRUE(ring.push("abc", 3));
    ASSERT_TRUE(ring.push("de", 2));
    EXPECT_EQ(ring.peak_bytes(), 5u);

    char c;
    ring.pop(&c);
    ring.pop(&c);
    ring.pop(&c);
    ring.pop(&c);
    ring.pop(&c);
    // Draining does not lower the high-water mark; it is a mark, not the
    // current level.
    EXPECT_EQ(ring.peak_bytes(), 5u);
}

// --------------------------------------------------------------------------
// Wrap: push, drain enough to move tail_ past the buffer end, then push again
// so the write wraps around index 0. A ring whose index math is wrong reads
// back garbage or the wrong length only here, never on a run that starts empty
// and never wraps.
// --------------------------------------------------------------------------

TEST(LoggerRing, WrapsAroundTheBackingBuffer)
{
    TinyRing ring; // capacity 8

    ASSERT_TRUE(ring.push("12345", 5)); // head_ = 5
    char c;
    for (int i = 0; i < 5; i++)
    {
        ASSERT_TRUE(ring.pop(&c)); // tail_ = 5, ring empty
    }
    EXPECT_EQ(ring.used(), 0u);

    // head_ and tail_ both sit at 5; this push wraps head_ from 5 through the
    // end of an 8-byte buffer and back around to 2.
    ASSERT_TRUE(ring.push("wxyzabc", 7));
    EXPECT_EQ(ring.used(), 7u);
    EXPECT_EQ(pop_all(ring), "wxyzabc");
}

TEST(LoggerRing, WrappedDataSurvivesInterleavedPushAndPop)
{
    TinyRing ring; // capacity 8, 7 usable

    ASSERT_TRUE(ring.push("abcd", 4));
    char c;
    ASSERT_TRUE(ring.pop(&c)); // 'a'
    ASSERT_TRUE(ring.pop(&c)); // 'b'
    // used_ is now 2 ("cd"); push 5 more, wrapping around the end.
    ASSERT_TRUE(ring.push("EFGHI", 5));
    EXPECT_EQ(ring.used(), 7u);
    EXPECT_EQ(pop_all(ring), "cdEFGHI");
}

// --------------------------------------------------------------------------
// Drop counting and whole-message discipline under pressure
// --------------------------------------------------------------------------

TEST(LoggerRing, MessageThatDoesNotFitIsDroppedWhole)
{
    TinyRing ring; // 7 usable bytes

    EXPECT_FALSE(ring.push("too-long-for-this-ring", 22));
    EXPECT_EQ(ring.dropped(), 1u);
    // Nothing was written -- not even the leading bytes that WOULD have fit.
    EXPECT_EQ(ring.used(), 0u);
    char c;
    EXPECT_FALSE(ring.pop(&c));
}

TEST(LoggerRing, DropDoesNotCorruptDataAlreadyQueued)
{
    TinyRing ring;
    ASSERT_TRUE(ring.push("abc", 3)); // 3 used, 4 free

    // Does not fit in the 4 remaining bytes: dropped whole, "abc" untouched.
    EXPECT_FALSE(ring.push("defgh", 5));
    EXPECT_EQ(ring.dropped(), 1u);
    EXPECT_EQ(ring.used(), 3u);
    EXPECT_EQ(pop_all(ring), "abc");
}

TEST(LoggerRing, DropCounterIsMonotonicAcrossMultipleFullPushes)
{
    TinyRing ring;
    ASSERT_TRUE(ring.push("1234567", 7)); // fill to capacity-1

    EXPECT_FALSE(ring.push("x", 1));
    EXPECT_FALSE(ring.push("yz", 2));
    EXPECT_EQ(ring.dropped(), 2u);
    // The queued message survived both failed pushes untouched.
    EXPECT_EQ(pop_all(ring), "1234567");
}

TEST(LoggerRing, RingRecoversAfterADropOnceSpaceIsFreed)
{
    TinyRing ring;
    ASSERT_TRUE(ring.push("1234567", 7));
    EXPECT_FALSE(ring.push("x", 1));

    char c;
    ring.pop(&c);
    ring.pop(&c);
    // 2 bytes now free; a 2-byte message fits again.
    EXPECT_TRUE(ring.push("ab", 2));
    EXPECT_EQ(ring.dropped(), 1u); // unchanged by the successful push
    EXPECT_EQ(pop_all(ring), "34567ab");
}
