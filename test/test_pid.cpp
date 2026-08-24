// Host-side unit tests for PidClass.
//
// Tests named DISABLED_* document known defects: each one asserts the behavior
// the controller SHOULD have, and currently fails. Remove the DISABLED_ prefix
// as part of the fix so the test becomes the regression guard. googletest
// prints a reminder that disabled tests exist, so they stay visible.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "high_level_drivers/pid.hpp"

namespace
{
// Wide clamps, so a test only hits a limit when it means to.
constexpr float kWideMax = 1'000'000.0f;
constexpr float kWideMin = -1'000'000.0f;

// The nominal control period: 200 Hz, per src/app/rt.h.
constexpr float kDt = 0.005f;

PidClass make_pid(float kp, float ki, float kd, float max = kWideMax, float min = kWideMin)
{
    PidClass pid(kp, ki, kd);
    pid.set_output_limits(min, max);
    return pid;
}
} // namespace

// --------------------------------------------------------------------------
// Positional form: the output is a function of the present state, not an
// accumulated increment.
// --------------------------------------------------------------------------

TEST(PidClass, ProportionalResponseToPositiveError)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f);

    // error = 10, output = kp * error = 20.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 20.0f);
}

TEST(PidClass, ZeroErrorProducesZeroOutput)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(pid.control_loop(10.0f, 10.0f, kDt), 0.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(10.0f, 10.0f, kDt), 0.0f);
}

// Previously asserted 20 then 25: the old form added a full positional term as
// an increment every cycle, so a constant error ramped the output until it hit
// the clamp. That ramp WAS the defect. A constant error is a constant command.
TEST(PidClass, ConstantErrorHoldsAConstantCommand)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f, /*max=*/25.0f, /*min=*/kWideMin);

    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 20.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 20.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 20.0f);
    EXPECT_FALSE(pid.saturated());
}

TEST(PidClass, OutputLimitsRoundTripThroughAccessors)
{
    PidClass pid(1.0f, 0.0f, 0.0f);

    pid.set_output_limits(-500.0f, 1000.0f);

    EXPECT_FLOAT_EQ(pid.get_max_output(), 1000.0f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -500.0f);
}

// The limits used to be int32_t while the accessors took float, so
// set_max_output(0.95f) silently became 0 -- total loss of drive authority with
// no diagnostic. They are float now and fractional limits survive.
TEST(PidClass, FractionalOutputLimitsSurvive)
{
    PidClass pid(1.0f, 0.0f, 0.0f);

    pid.set_output_limits(-0.95f, 0.95f);

    EXPECT_FLOAT_EQ(pid.get_max_output(), 0.95f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -0.95f);
}

// std::clamp with lo > hi is undefined behaviour, and a degenerate range would
// pin the output. Bad limits are rejected outright, leaving the old ones.
TEST(PidClass, RejectsInvertedLimits)
{
    PidClass pid = make_pid(1.0f, 0.0f, 0.0f, /*max=*/100.0f, /*min=*/-100.0f);

    pid.set_output_limits(10.0f, -10.0f); // inverted
    EXPECT_FLOAT_EQ(pid.get_max_output(), 100.0f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -100.0f);

    pid.set_output_limits(5.0f, 5.0f); // degenerate
    EXPECT_FLOAT_EQ(pid.get_max_output(), 100.0f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -100.0f);

    pid.set_output_limits(-std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN());
    EXPECT_FLOAT_EQ(pid.get_max_output(), 100.0f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -100.0f);
}

// --------------------------------------------------------------------------
// SAF-33: the output is analytically bounded, for every gain set.
// --------------------------------------------------------------------------

TEST(PidClass, OutputIsAlwaysWithinLimits)
{
    constexpr float kMax = 1000.0f;
    constexpr float kMin = -1000.0f;

    const float gains[] = {0.0f, 0.5f, 32.0f, 5000.0f};
    const float errors[] = {-10000.0f, -50.0f, -0.1f, 0.0f, 0.1f, 50.0f, 10000.0f};

    for (float kp : gains)
    {
        for (float ki : gains)
        {
            for (float kd : gains)
            {
                PidClass pid = make_pid(kp, ki, kd, kMax, kMin);
                for (float error : errors)
                {
                    for (int i = 0; i < 50; ++i)
                    {
                        const float out = pid.control_loop(0.0f, error, kDt);
                        ASSERT_GE(out, kMin);
                        ASSERT_LE(out, kMax);
                        ASSERT_TRUE(std::isfinite(out));
                    }
                }
            }
        }
    }
}

// --------------------------------------------------------------------------
// Regression guards for the defects in issue #13.
// --------------------------------------------------------------------------

// The minimum-output guard used to test `output - delta_output <= min_output`,
// but the value about to be committed was `output + delta_output`. With a
// positive error and a modest negative floor, the controller returned its
// MINIMUM while being asked to accelerate forward.
TEST(PidClass, MinClampMustNotTriggerWhileErrorIsPositive)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f, /*max=*/kWideMax, /*min=*/-10.0f);

    // Wants +20. The sign error made it return -10 instead.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 20.0f);
}

// SAF-32. The two defects above compounded into a one-way latch under a
// NEGATIVE target: once the output overshot the floor, the guard degenerated to
// `output <= min_output`, which was permanently true, and the controller
// returned min_output on every call regardless of measured speed.
//
// Physical consequence: the wheel pinned at ~99% duty in reverse, no longer
// responding to its encoder. Limits are set so that the commanded value
// genuinely saturates -- the guard is worthless if nothing ever clamps.
TEST(PidClass, NegativeTargetMustNotLatchAtMinimumOutput)
{
    PidClass pid = make_pid(32.0f, 0.0f, 0.0f, /*max=*/1000.0f, /*min=*/-1000.0f);

    // Wheel stalled at 0 rpm against a -50 rpm target: kp * error = -1600,
    // well past the -1000 floor, so it saturates on every one of these.
    for (int i = 0; i < 60; ++i)
    {
        ASSERT_FLOAT_EQ(pid.control_loop(0.0f, -50.0f, kDt), -1000.0f);
    }
    EXPECT_TRUE(pid.saturated());

    // The wheel now reaches target. The command must come back off the floor as
    // the error does, not latch.
    const float half_way = pid.control_loop(-25.0f, -50.0f, kDt);
    EXPECT_GT(half_way, -1000.0f);
    EXPECT_FLOAT_EQ(half_way, -800.0f);

    const float at_target = pid.control_loop(-50.0f, -50.0f, kDt);
    EXPECT_FLOAT_EQ(at_target, 0.0f);
    EXPECT_FALSE(pid.saturated());
}

// SAF-32, the positive-going half. Saturating must not corrupt the controller's
// state: the moment the error shrinks enough to be deliverable, the command
// must track it again. (The old test asserted that a saturated command HELD
// once the error went to zero -- a velocity-form invariant. Under the positional
// form with ki = 0, zero error means zero output, and that is correct.)
TEST(PidClass, SaturatedControllerResumesTrackingWhenErrorShrinks)
{
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f, /*max=*/15.0f, /*min=*/kWideMin);

    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 15.0f); // wants 20, clamped
    EXPECT_TRUE(pid.saturated());
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 15.0f); // still clamped
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 5.0f, kDt), 10.0f);  // deliverable again
    EXPECT_FALSE(pid.saturated());
}

// Anti-windup. Saturation is the mechanism under test: with kp = 0 the integral
// alone drives the output, and the limits are tight enough that a sustained
// error would bury them.
TEST(PidClass, IntegralMustNotWindUpUnboundedly)
{
    PidClass pid = make_pid(0.0f, 100.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);

    // 200 cycles of e = 10 at dt = 0.005. Unbounded, the integral reaches 10 and
    // ki * integral reaches 1000 -- twenty times the limit.
    for (int i = 0; i < 200; ++i)
    {
        pid.control_loop(0.0f, 10.0f, kDt);
    }
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 50.0f);

    // Reverse. Each cycle unwinds ki * e * dt = 5, so a correctly bounded
    // integral (pinned at max/ki = 0.5, i.e. a 50-count term) crosses zero in
    // about ten cycles. An unbounded one, sitting at a 1000-count term, would
    // need two hundred.
    int cycles_to_cross = -1;
    for (int i = 0; i < 15; ++i)
    {
        if (pid.control_loop(0.0f, -10.0f, kDt) < 0.0f)
        {
            cycles_to_cross = i;
            break;
        }
    }

    EXPECT_GE(cycles_to_cross, 0) << "command never reversed within 15 cycles";
}

// --------------------------------------------------------------------------
// Properties of the positional form itself.
// --------------------------------------------------------------------------

TEST(PidClass, DtScalesTheIntegralTerm)
{
    // Same elapsed time, different cycle rates: the accumulated integral must
    // match. Under the old form the integral counted CYCLES, not seconds.
    PidClass fast = make_pid(0.0f, 1.0f, 0.0f, /*max=*/1000.0f, /*min=*/-1000.0f);
    for (int i = 0; i < 100; ++i)
    {
        fast.control_loop(0.0f, 10.0f, 0.005f);
    }

    PidClass slow = make_pid(0.0f, 1.0f, 0.0f, /*max=*/1000.0f, /*min=*/-1000.0f);
    for (int i = 0; i < 50; ++i)
    {
        slow.control_loop(0.0f, 10.0f, 0.010f);
    }

    // Probe at zero error: with kp = kd = 0 the returned value is ki * integral,
    // and a zero error leaves the accumulator untouched.
    const float fast_term = fast.control_loop(0.0f, 0.0f, 0.005f);
    const float slow_term = slow.control_loop(0.0f, 0.0f, 0.010f);

    // 0.5 s of e = 10 gives an integral of 5. The tolerance covers the
    // difference in float accumulation order, not a difference in the result.
    EXPECT_NEAR(fast_term, 5.0f, 1e-3f);
    EXPECT_NEAR(slow_term, 5.0f, 1e-3f);
    EXPECT_NEAR(fast_term, slow_term, 1e-3f);
}

// Derivative on measurement, not on error. Differentiating the error puts a
// (setpoint step / dt) spike on the motors every time a new target arrives:
// here that would be 10 * 100 / 0.005 = 200000 counts.
TEST(PidClass, DerivativeDoesNotKickOnSetpointStep)
{
    PidClass pid = make_pid(1.0f, 0.0f, 10.0f);

    ASSERT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f, kDt), 0.0f);

    // Setpoint steps 0 -> 100 while the measurement has not moved.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 100.0f, kDt), 100.0f);
}

TEST(PidClass, DerivativeOpposesMeasurementChange)
{
    // Sanity check on the sign: with the measurement climbing and the setpoint
    // fixed, the derivative term must push back.
    PidClass pid = make_pid(0.0f, 0.0f, 2.0f);

    ASSERT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f, kDt), 0.0f);

    // measurement 0 -> 1 over 0.005 s: d = -200, term = -400.
    EXPECT_FLOAT_EQ(pid.control_loop(1.0f, 0.0f, kDt), -400.0f);
}

TEST(PidClass, ResetClearsIntegralAndDerivativeHistory)
{
    PidClass pid = make_pid(1.0f, 5.0f, 3.0f, /*max=*/1000.0f, /*min=*/-1000.0f);
    for (int i = 0; i < 40; ++i)
    {
        pid.control_loop(static_cast<float>(i), 100.0f, kDt);
    }

    pid.reset();

    PidClass fresh = make_pid(1.0f, 5.0f, 3.0f, /*max=*/1000.0f, /*min=*/-1000.0f);

    // A reset controller must be indistinguishable from a new one: no residual
    // integral, and no derivative history to differentiate against.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f, kDt), fresh.control_loop(0.0f, 0.0f, kDt));
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f, kDt), 0.0f);
    EXPECT_FALSE(pid.saturated());

    EXPECT_FLOAT_EQ(pid.control_loop(4.0f, 10.0f, kDt), fresh.control_loop(4.0f, 10.0f, kDt));
}

// --------------------------------------------------------------------------
// Anti-windup referenced to what was actually applied downstream.
// --------------------------------------------------------------------------

TEST(PidClass, AntiWindupHonoursAppliedValue)
{
    // A downstream deadband swallows the command outright: nothing moves, so
    // the error never dies. The integral must still not run away.
    PidClass pid = make_pid(0.0f, 100.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);
    for (int i = 0; i < 500; ++i)
    {
        pid.note_applied(0.0f);
        const float out = pid.control_loop(0.0f, 10.0f, kDt);
        ASSERT_LE(out, 50.0f);
    }
    // Bounded: ki * integral cannot exceed max_output, so one cycle of the
    // opposite error is enough to walk it straight back down.
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 50.0f);

    // And the applied value is load-bearing in its own right: a downstream stage
    // pinned at the limit means further integration buys authority that cannot
    // be delivered, even though the controller's own unclamped output is well
    // inside the range.
    PidClass held = make_pid(0.0f, 100.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);
    for (int i = 0; i < 100; ++i)
    {
        held.note_applied(50.0f); // slew limiter or PWM clamp sitting at max
        EXPECT_FLOAT_EQ(held.control_loop(0.0f, 1.0f, kDt), 0.0f);
    }
}

TEST(PidClass, ZeroKiDoesNotDivideByZero)
{
    // ki = 0 is the live configuration, so the integral bound divides by zero
    // unless guarded.
    PidClass pid = make_pid(2.0f, 0.0f, 0.0f, /*max=*/100.0f, /*min=*/-100.0f);

    for (int i = 0; i < 100; ++i)
    {
        const float out = pid.control_loop(0.0f, 500.0f, kDt);
        ASSERT_TRUE(std::isfinite(out));
        ASSERT_FLOAT_EQ(out, 100.0f);
    }

    const float back = pid.control_loop(0.0f, 10.0f, kDt);
    EXPECT_TRUE(std::isfinite(back));
    EXPECT_FLOAT_EQ(back, 20.0f);
}
