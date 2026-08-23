// Host-side unit tests for PidClass.
//
// Tests named DISABLED_* document known defects: each one asserts the behavior
// the controller SHOULD have, and currently fails. Remove the DISABLED_ prefix
// as part of the fix so the test becomes the regression guard. googletest
// prints a reminder that disabled tests exist, so they stay visible.

#include <gtest/gtest.h>

#include "high_level_drivers/pid.hpp"

namespace
{
// Wide clamps, so a test only hits a limit when it means to.
constexpr float kWideMax = 1'000'000.0f;
constexpr float kWideMin = -1'000'000.0f;

PidClass make_pid(float kp, float ki, float kd, float max = kWideMax, float min = kWideMin)
{
    PidClass pid(kp, ki, kd);
    pid.set_max_output(max);
    pid.set_min_output(min);
    return pid;
}
} // namespace

// --------------------------------------------------------------------------
// Behavior that is currently correct.
// --------------------------------------------------------------------------

TEST(PidClass, ProportionalResponseToPositiveError)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f);

    // error = 10, delta = kp * error = 20, output = 0 + 20.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f), 20.0f);
}

TEST(PidClass, ZeroErrorLeavesOutputUnchanged)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(pid.control_loop(10.0f, 10.0f), 0.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(10.0f, 10.0f), 0.0f);
}

TEST(PidClass, MaxOutputClampIsReturnedOnceCommandExceedsIt)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f, /*max=*/25.0f, /*min=*/kWideMin);

    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f), 20.0f); // 0 + 20, under the clamp
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f), 25.0f); // 20 + 20 would exceed 25
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f), 25.0f); // stays clamped
}

TEST(PidClass, OutputLimitsRoundTripThroughAccessors)
{
    PidClass pid(1.0f, 0.0f, 0.0f);

    pid.set_max_output(1000.0f);
    pid.set_min_output(-500.0f);

    EXPECT_FLOAT_EQ(pid.get_max_output(), 1000.0f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -500.0f);
}

TEST(PidClass, OutputLimitsTruncateTowardZero)
{
    // max_output/min_output are declared int32_t while the accessors take and
    // return float, so fractional limits silently truncate. Documented here so
    // the narrowing is a deliberate, visible property rather than a surprise.
    PidClass pid(1.0f, 0.0f, 0.0f);

    pid.set_max_output(10.7f);
    pid.set_min_output(-10.7f);

    EXPECT_FLOAT_EQ(pid.get_max_output(), 10.0f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -10.0f);
}

// --------------------------------------------------------------------------
// Known defects. See the issue tracker; each of these should be enabled by the
// change that fixes it.
// --------------------------------------------------------------------------

// The minimum-output guard tests `output - delta_output <= min_output`, but the
// value about to be committed is `output + delta_output`. With a positive error
// and a modest negative floor, the controller returns its MINIMUM while being
// asked to accelerate forward.
TEST(PidClass, DISABLED_MinClampMustNotTriggerWhileErrorIsPositive)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f, /*max=*/kWideMax, /*min=*/-10.0f);

    // Wants +20. The sign error makes it return -10 instead.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f), 20.0f);
}

// Both clamp branches return early WITHOUT assigning to `output`, so the
// controller's internal state stops tracking what it actually commanded. After
// saturating, it resumes from the last unclamped value.
TEST(PidClass, DISABLED_ClampedCommandMustUpdateInternalState)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f, /*max=*/25.0f, /*min=*/kWideMin);

    ASSERT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f), 20.0f);
    ASSERT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f), 25.0f); // saturated

    // Error now zero, so the command should hold at what was last sent: 25.
    // Internal `output` is still 20, so it drops back to 20 instead.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f), 25.0f);
}

// The two defects above compound into a one-way latch under a NEGATIVE target,
// which is what main() commands on the right wheel (right_target = -50.0).
//
// With a negative error, delta_output is negative, so `output - delta_output`
// moves UP, away from the floor -- the guard cannot fire while output marches
// down past min_output. Once the error shrinks and delta_output approaches
// zero, the test degenerates to `output <= min_output`, which is permanently
// true because output overshot. From then on the controller returns min_output
// on every call regardless of measured speed, and never updates `output`.
//
// Physical consequence: the wheel is pinned at ~99% duty in reverse and stops
// responding to its encoder entirely. Reproduces main()'s exact gains.
TEST(PidClass, DISABLED_NegativeTargetMustNotLatchAtMinimumOutput)
{
    PidClass pid = make_pid(32.0f, 0.0f, 0.0f, /*max=*/65000.0f, /*min=*/-65000.0f);

    // Wheel stalled at 0 rpm against a -50 rpm target: drive output to the floor.
    for (int i = 0; i < 60; ++i)
    {
        pid.control_loop(0.0f, -50.0f);
    }

    // Now the wheel reaches target, so error is zero and the controller should
    // stop commanding full reverse. It returns min_output forever instead.
    const float at_target = pid.control_loop(-50.0f, -50.0f);
    EXPECT_GT(at_target, -65000.0f);
}

// `integral_component += error` is unbounded. After a sustained error the
// accumulator is large enough that reversing the target does not reverse the
// command for many iterations.
TEST(PidClass, DISABLED_IntegralMustNotWindUpUnboundedly)
{
    PidClass pid = make_pid(0.0f, 1.0f, 0.0f, /*max=*/10000.0f, /*min=*/-10000.0f);

    for (int i = 0; i < 20; ++i)
    {
        pid.control_loop(0.0f, 10.0f); // sustained positive error
    }

    const float at_reversal = pid.control_loop(0.0f, -10.0f);

    // Five steps after asking for the opposite direction, the command should be
    // heading down. Wound-up integral keeps it climbing instead.
    float latest = at_reversal;
    for (int i = 0; i < 5; ++i)
    {
        latest = pid.control_loop(0.0f, -10.0f);
    }

    EXPECT_LT(latest, at_reversal);
}
