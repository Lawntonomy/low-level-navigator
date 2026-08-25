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

    // On STATE, a reset controller must be indistinguishable from an
    // identically configured new one: no residual integral, and no derivative
    // history to differentiate against. (Configuration is a separate matter --
    // see ResetPreservesConfigurationAndClearsOnlyState.)
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

// SAF-32, second instance -- found in the fix for the first.
//
// `applied_` starts at 0.0f and nothing in this firmware calls note_applied()
// yet. With a range that straddles zero the applied_ clauses are inert and the
// behaviour is correct, which is why every earlier test missed this. But a
// caller gating direction writes set_output_limits(0.0f, +N) -- the pattern
// src/low-level-navigator.cpp uses, and the natural way to express SAF-30 with
// this API -- and then `applied_ <= min_output_` is 0 <= 0, permanently true.
// Every cycle with a negative error has its integration blocked, so the
// integral is monotonically non-decreasing: it ratchets up to the step 7 bound
// of max/ki and the integral term alone is then full scale, with no measured
// over-speed able to remove it.
//
// The fix is that the applied_ clauses are OPT-IN: they apply only on a cycle
// preceded by note_applied(). An uncalled note_applied() degrades to pure
// clamp-based conditional integration -- the same correct behaviour the
// straddling case already had -- instead of silently asserting that the
// downstream stage is pinned.
TEST(PidClass, ForwardOnlyRangeDoesNotLatchTheIntegral)
{
    // kp = kd = 0, so the output IS the integral term and nothing else can
    // mask the latch. Forward-only gate: [0, 1000].
    PidClass pid = make_pid(0.0f, 100.0f, 0.0f, /*max=*/1000.0f, /*min=*/0.0f);

    // Wind the integral up legitimately. Nothing saturates: 20 cycles of
    // e = +10 at dt = 0.005 leave the integral at 1.0, a 100-count term
    // against a 1000-count ceiling.
    float last = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        last = pid.control_loop(0.0f, 10.0f, kDt);
    }
    ASSERT_GT(last, 0.0f);

    // Now the wheel overshoots: measurement 20 against a setpoint of 10, so
    // e = -10 sustained. The integral must unwind and the command must come
    // back to zero. Latched, it sits at 100 forever -- a wheel commanded to
    // stop that never does.
    last = pid.control_loop(20.0f, 10.0f, kDt);
    ASSERT_GT(last, 0.0f);
    for (int i = 1; i < 25; ++i)
    {
        const float out = pid.control_loop(20.0f, 10.0f, kDt);
        ASSERT_LE(out, last) << "command rose on cycle " << i;
        last = out;
    }
    EXPECT_NEAR(last, 0.0f, 1e-3f);
}

// The mirror case: max_output_ == 0 makes `applied_ >= max_output_` permanently
// true, so it is positive-going integration that is blocked and the latch holds
// in reverse.
TEST(PidClass, ReverseOnlyRangeDoesNotLatchTheIntegral)
{
    PidClass pid = make_pid(0.0f, 100.0f, 0.0f, /*max=*/0.0f, /*min=*/-1000.0f);

    float last = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        last = pid.control_loop(10.0f, 0.0f, kDt); // e = -10
    }
    ASSERT_LT(last, 0.0f);

    last = pid.control_loop(0.0f, 10.0f, kDt); // e = +10
    ASSERT_LT(last, 0.0f);
    for (int i = 1; i < 25; ++i)
    {
        const float out = pid.control_loop(0.0f, 10.0f, kDt);
        ASSERT_GE(out, last) << "command fell on cycle " << i;
        last = out;
    }
    EXPECT_NEAR(last, 0.0f, 1e-3f);
}

// The report is good for one cycle and one only. If it persisted, a single
// note_applied() at a rail would block integration in that direction for the
// rest of the controller's life -- the same latch by a slower route.
TEST(PidClass, AppliedValueExpiresAfterOneCycle)
{
    PidClass pid = make_pid(0.0f, 100.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);

    // One cycle with the downstream stage pinned at the ceiling: blocked.
    pid.note_applied(50.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 1.0f, kDt), 0.0f);

    // Nothing reports anything after that, so the controller must go back to
    // judging by its own clamp -- which is not firing -- and integrate.
    float out = 0.0f;
    for (int i = 0; i < 5; ++i)
    {
        out = pid.control_loop(0.0f, 1.0f, kDt);
    }
    EXPECT_GT(out, 0.0f);
    // Four cycles of e = 1 at dt = 0.005 accumulated before the fifth returned.
    EXPECT_NEAR(out, 2.0f, 1e-3f);
}

// Step 7 -- bounding the integral to [min/ki, max/ki] -- is only separable from
// the conditional integration of step 6 when the LIMITS MOVE. The orphaned
// superloop rewrote the output limits on every iteration, so this is not a
// hypothetical: the integral here is accumulated entirely legitimately, under
// limits it never came close to, and is only excessive after the range narrows
// underneath it. Step 6 has no cause to block any of that accumulation and
// cannot undo it afterwards.
TEST(PidClass, NarrowedOutputLimitsBoundTheExistingIntegral)
{
    PidClass pid = make_pid(0.0f, 100.0f, 0.0f, /*max=*/1000.0f, /*min=*/-1000.0f);

    // 100 cycles of e = 10 at dt = 0.005 leave the integral at 5, i.e. a
    // 500-count term against a 1000-count ceiling. Nothing saturates on the way
    // there, and 5 is well inside the step 7 bound of max/ki = 10.
    for (int i = 0; i < 100; ++i)
    {
        pid.control_loop(0.0f, 10.0f, kDt);
    }
    // Probe at zero error: with kp = kd = 0 the returned value is ki * integral,
    // and a zero error leaves the accumulator untouched.
    ASSERT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f, kDt), 500.0f);

    pid.set_output_limits(-50.0f, 50.0f);

    // One cycle under the new limits. The output is clamped, as it must be...
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f, kDt), 50.0f);
    EXPECT_TRUE(pid.saturated());

    // ...but the clamp only masks the oversized integral; it does not correct
    // it. Widening the range again makes the accumulator observable, which is
    // the only reason this call is here. The integral must have been pulled
    // down to max/ki = 0.5, a 50-count term -- the most the narrow range could
    // ever have authorised. Without step 7 it is still 5, and the controller
    // carries a 500-count term it accumulated under limits that no longer
    // apply, taking ten times as long to unwind.
    pid.set_output_limits(-1000.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 0.0f, kDt), 50.0f);
}

// --------------------------------------------------------------------------
// Non-finite inputs. std::clamp(v, lo, hi) returns v when both comparisons are
// false, which is exactly what a NaN gives -- so neither the dt clamp nor the
// output clamp is a guard against one.
// --------------------------------------------------------------------------

TEST(PidClass, NonFiniteDtFallsBackToTheNominalPeriod)
{
    // Unguarded, a NaN dt makes the derivative divisor NaN and the output clamp
    // passes the NaN straight through into a motor command.
    PidClass nan_dt = make_pid(0.0f, 0.0f, 2.0f);
    ASSERT_FLOAT_EQ(nan_dt.control_loop(0.0f, 0.0f, kDt), 0.0f);

    // Substituting the nominal 5 ms gives the same answer as
    // DerivativeOpposesMeasurementChange: d = -200, term = -400.
    const float from_nan = nan_dt.control_loop(1.0f, 0.0f, std::numeric_limits<float>::quiet_NaN());
    EXPECT_TRUE(std::isfinite(from_nan));
    EXPECT_FLOAT_EQ(from_nan, -400.0f);

    // An infinite dt does survive the clamp, but as dt_max (0.025 s), which
    // silently divides the derivative by five. Same substitution.
    PidClass inf_dt = make_pid(0.0f, 0.0f, 2.0f);
    ASSERT_FLOAT_EQ(inf_dt.control_loop(0.0f, 0.0f, kDt), 0.0f);

    const float from_inf = inf_dt.control_loop(1.0f, 0.0f, std::numeric_limits<float>::infinity());
    EXPECT_TRUE(std::isfinite(from_inf));
    EXPECT_FLOAT_EQ(from_inf, -400.0f);
}

// Defence in depth: the encoder reports Reading{rpm, valid} and the caller is
// supposed to gate on `valid`. A controller that emits NaN when it is lied to
// anyway is a poor last line.
TEST(PidClass, NonFiniteMeasurementOrSetpointReturnsZeroAndLeavesStateUntouched)
{
    PidClass pid = make_pid(1.0f, 5.0f, 3.0f, /*max=*/1000.0f, /*min=*/-1000.0f);
    PidClass twin = make_pid(1.0f, 5.0f, 3.0f, /*max=*/1000.0f, /*min=*/-1000.0f);

    for (int i = 0; i < 20; ++i)
    {
        const float measurement = static_cast<float>(i);
        pid.control_loop(measurement, 50.0f, kDt);
        twin.control_loop(measurement, 50.0f, kDt);
    }

    const float not_a_number = std::numeric_limits<float>::quiet_NaN();
    const float infinite = std::numeric_limits<float>::infinity();

    // Zero is the safe value BECAUSE IT IS AN OUTPUT: it commands no drive.
    EXPECT_FLOAT_EQ(pid.control_loop(not_a_number, 50.0f, kDt), 0.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(19.0f, not_a_number, kDt), 0.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(infinite, 50.0f, kDt), 0.0f);
    EXPECT_FLOAT_EQ(pid.control_loop(19.0f, -infinite, kDt), 0.0f);

    // The twin was never shown any of them. An unusable sample must not reach
    // the integral or become the derivative history, so the next good cycle
    // must be identical for both controllers.
    EXPECT_FLOAT_EQ(pid.control_loop(20.0f, 50.0f, kDt), twin.control_loop(20.0f, 50.0f, kDt));
    EXPECT_FLOAT_EQ(pid.control_loop(21.0f, 50.0f, kDt), twin.control_loop(21.0f, 50.0f, kDt));
}

// The zero returned above is a deliberate exception to the range invariant.
// Zero is the safe actuator command whatever range is configured; clamping into
// a range whose minimum is positive would answer an unusable measurement by
// carrying on driving. Documented in pid.hpp, pinned here.
TEST(PidClass, NonFiniteInputReturnsZeroEvenWhenZeroIsOutsideTheRange)
{
    PidClass pid = make_pid(1.0f, 0.0f, 0.0f, /*max=*/1000.0f, /*min=*/100.0f);

    ASSERT_FLOAT_EQ(pid.control_loop(0.0f, 500.0f, kDt), 500.0f);

    const float not_a_number = std::numeric_limits<float>::quiet_NaN();
    const float out = pid.control_loop(not_a_number, 500.0f, kDt);

    EXPECT_FLOAT_EQ(out, 0.0f);
    EXPECT_LT(out, pid.get_min_output()) << "the range exception is the point of this test";
}

// --------------------------------------------------------------------------
// reset() clears state; it does not touch configuration.
// --------------------------------------------------------------------------

// pid.hpp used to claim a reset controller was "indistinguishable from a
// freshly constructed one". It is not, and should not be: a fresh controller
// carries the default limits of [-1, 1], while reset() leaves whatever range
// was configured in place. Restoring the limits would silently discard the
// caller's configuration, which is the worse of the two surprises. reset()
// clears STATE and preserves CONFIGURATION; this pins that.
TEST(PidClass, ResetPreservesConfigurationAndClearsOnlyState)
{
    PidClass pid = make_pid(1.0f, 0.0f, 0.0f, /*max=*/250.0f, /*min=*/-75.0f);
    pid.control_loop(0.0f, 10.0f, kDt);

    pid.reset();

    EXPECT_FLOAT_EQ(pid.get_max_output(), 250.0f);
    EXPECT_FLOAT_EQ(pid.get_min_output(), -75.0f);
    EXPECT_TRUE(pid.gains_valid());

    // A freshly constructed one is a different controller: default limits, so
    // the same error gives a different command.
    PidClass fresh(1.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(fresh.get_max_output(), 1.0f);
    EXPECT_FLOAT_EQ(fresh.get_min_output(), -1.0f);

    EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 10.0f, kDt), 10.0f);
    EXPECT_FLOAT_EQ(fresh.control_loop(0.0f, 10.0f, kDt), 1.0f);
}

TEST(PidClass, ResetClearsTheAppliedValue)
{
    PidClass pid = make_pid(0.0f, 100.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);

    // A downstream stage pinned at the ceiling blocks positive integration, so
    // the controller sits at zero however long the error persists.
    for (int i = 0; i < 10; ++i)
    {
        pid.note_applied(50.0f);
        ASSERT_FLOAT_EQ(pid.control_loop(0.0f, 1.0f, kDt), 0.0f);
    }

    pid.reset();

    // A reset taken in that state must clear the controller's belief about what
    // was last applied too. The per-cycle expiry of the report now makes that
    // structural -- the value cannot outlive the cycle it was noted for -- and
    // reset() clearing it covers the remaining gap, a reset falling between
    // note_applied() and the control_loop() that would have consumed it.
    // Nothing calls note_applied() below, so a reset controller and an
    // identically configured fresh one must track each other exactly.
    PidClass fresh = make_pid(0.0f, 100.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_FLOAT_EQ(pid.control_loop(0.0f, 1.0f, kDt), fresh.control_loop(0.0f, 1.0f, kDt));
    }

    // ki * integral after five cycles of e = 1 at dt = 0.005 is 2.5, not 0.
    EXPECT_GT(pid.control_loop(0.0f, 1.0f, kDt), 0.0f);
}

// --------------------------------------------------------------------------
// Gain validation. A negative gain inverts the sign of the feedback; a
// non-finite one poisons every term it touches. Both are coerced to zero,
// which is inert, and gains_valid() reports the coercion.
//
// This replaces an earlier NegativeKiDoesNotInvertTheIntegralBound, which
// asserted that ki = -100 against a POSITIVE error held the output at the
// MINIMUM for ten consecutive cycles -- i.e. it enshrined issue #13's headline
// symptom (sustained full reverse in answer to "accelerate forward") as a
// passing regression guard. The ordered std::min/std::max bound in step 7 is
// kept as defence in depth, but a negative ki can no longer reach it, so
// nothing here can exercise the ordering directly any more.
// --------------------------------------------------------------------------

TEST(PidClass, NegativeGainsAreCoercedToZero)
{
    // Negative kp: error +10 would give -20, i.e. reverse in answer to a
    // request to accelerate forward, sustained for as long as the error lasts.
    PidClass negative_kp = make_pid(-2.0f, 0.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);
    EXPECT_FALSE(negative_kp.gains_valid());
    for (int i = 0; i < 10; ++i)
    {
        ASSERT_FLOAT_EQ(negative_kp.control_loop(0.0f, 10.0f, kDt), 0.0f) << "cycle " << i;
    }

    // Negative ki: the integral term marches away from the error rather than
    // toward it, and ends pinned at the wrong rail.
    PidClass negative_ki = make_pid(0.0f, -100.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);
    EXPECT_FALSE(negative_ki.gains_valid());
    for (int i = 0; i < 30; ++i)
    {
        ASSERT_FLOAT_EQ(negative_ki.control_loop(0.0f, 10.0f, kDt), 0.0f) << "cycle " << i;
    }

    // Negative kd: the derivative term reinforces the measurement change it is
    // supposed to damp.
    PidClass negative_kd = make_pid(0.0f, 0.0f, -2.0f, /*max=*/50.0f, /*min=*/-50.0f);
    EXPECT_FALSE(negative_kd.gains_valid());
    ASSERT_FLOAT_EQ(negative_kd.control_loop(0.0f, 0.0f, kDt), 0.0f);
    EXPECT_FLOAT_EQ(negative_kd.control_loop(1.0f, 0.0f, kDt), 0.0f);
}

TEST(PidClass, NonFiniteGainsAreCoercedToZero)
{
    const float not_a_number = std::numeric_limits<float>::quiet_NaN();
    const float infinite = std::numeric_limits<float>::infinity();

    // An infinite kp makes every non-zero error saturate instantly; a NaN one
    // makes the output NaN, which the output clamp does not catch.
    PidClass infinite_kp = make_pid(infinite, 0.0f, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);
    EXPECT_FALSE(infinite_kp.gains_valid());
    EXPECT_FLOAT_EQ(infinite_kp.control_loop(0.0f, 10.0f, kDt), 0.0f);

    PidClass nan_ki = make_pid(0.0f, not_a_number, 0.0f, /*max=*/50.0f, /*min=*/-50.0f);
    EXPECT_FALSE(nan_ki.gains_valid());
    for (int i = 0; i < 10; ++i)
    {
        const float out = nan_ki.control_loop(0.0f, 10.0f, kDt);
        ASSERT_TRUE(std::isfinite(out)) << "cycle " << i;
        ASSERT_FLOAT_EQ(out, 0.0f) << "cycle " << i;
    }

    // Only the offending gain is coerced. The rest of the controller keeps
    // working: this one is left as pure proportional.
    PidClass nan_kd = make_pid(1.0f, 0.0f, not_a_number, /*max=*/50.0f, /*min=*/-50.0f);
    EXPECT_FALSE(nan_kd.gains_valid());
    ASSERT_FLOAT_EQ(nan_kd.control_loop(0.0f, 0.0f, kDt), 0.0f);
    // measurement 0 -> 1 with the setpoint at 0: kp * error = -1, and the
    // coerced kd contributes nothing rather than a NaN.
    const float proportional_only = nan_kd.control_loop(1.0f, 0.0f, kDt);
    EXPECT_TRUE(std::isfinite(proportional_only));
    EXPECT_FLOAT_EQ(proportional_only, -1.0f);
}

// Zero is a setting, not a missing value: ki = kd = 0 is the live
// configuration and kp = 0 is how the windup tests isolate the integral.
// Coercing or flagging it would make gains_valid() false for every controller
// this firmware actually runs.
TEST(PidClass, ZeroGainsAreValid)
{
    PidClass live = make_pid(2.0f, 0.0f, 0.0f);
    EXPECT_TRUE(live.gains_valid());
    EXPECT_FLOAT_EQ(live.control_loop(0.0f, 10.0f, kDt), 20.0f);

    PidClass all_zero(0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(all_zero.gains_valid());

    PidClass all_positive = make_pid(1.0f, 2.0f, 3.0f, /*max=*/1000.0f, /*min=*/-1000.0f);
    EXPECT_TRUE(all_positive.gains_valid());
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
