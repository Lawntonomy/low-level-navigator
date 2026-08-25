#include "pid.hpp"
#include <algorithm>
#include <cmath>
#include "utility/logger.h"

static const char* category = "PID";

namespace
{
// The control task runs at 200 Hz (src/app/rt.h, period_control_ms = 5).
// Restated as a plain float because this translation unit is compiled by the
// host test build, which has no FreeRTOS headers.
constexpr float control_period_s = 0.005f;

// A cycle shorter than this makes the derivative divisor meaningless; one
// longer than five nominal periods means the loop has already missed its
// deadline badly enough that pretending otherwise is worse than under-reacting.
// Clamping instead of branching keeps every path through control_loop identical,
// so a bad dt can never skip the state update.
constexpr float dt_min_s = 1e-4f;
constexpr float dt_max_s = 5.0f * control_period_s;

// Default limits. Deliberately narrow and non-degenerate: a controller whose
// limits were never configured should have almost no authority, not unbounded
// authority, and must still satisfy min < max.
constexpr float default_max_output = 1.0f;
constexpr float default_min_output = -1.0f;

// A usable gain is finite and non-negative. Zero counts: ki = kd = 0 is the
// live configuration and kp = 0 is how a test isolates the integral, so zero
// is a deliberate setting, not a missing one. Negative is not a setting at all
// -- it inverts the sign of the feedback, so a request to accelerate forward
// is answered with reverse and held there. Non-finite propagates into every
// term that gain touches, and the output clamp does not stop a NaN.
//
// (`gain >= 0.0f` is already false for a NaN; the isfinite test is what
// catches +infinity.)
bool gainIsUsable(float gain)
{
    return std::isfinite(gain) && gain >= 0.0f;
}

// Coerce rather than reject. A zero gain is inert -- the term stops
// contributing and the rest of the controller keeps working -- whereas
// refusing to construct, or substituting a guessed default, would either
// remove the controller entirely or invent a configuration nobody chose.
float coerceGain(float gain)
{
    return gainIsUsable(gain) ? gain : 0.0f;
}
} // namespace

PidClass::PidClass(float kp_init, float ki_init, float kd_init)
    : kp_(coerceGain(kp_init)), ki_(coerceGain(ki_init)), kd_(coerceGain(kd_init)),
      min_output_(default_min_output), max_output_(default_max_output), integral_(0.0f),
      prior_measurement_(0.0f), applied_(0.0f), have_prior_(false), saturated_(false),
      have_applied_(false),
      gains_valid_(gainIsUsable(kp_init) && gainIsUsable(ki_init) && gainIsUsable(kd_init))
{
    Log::info(category, "init Pid");
}

PidClass::~PidClass()
{
}

float PidClass::control_loop(float measurement, float setpoint, float dt)
{
    // The note_applied() report is consumed here, at the top, and the flag
    // dropped -- it is good for this cycle and no other. Latching it into a
    // local first is what makes the anti-windup below OPT-IN: without a report,
    // `applied_` is just the default 0.0f, and treating that as fact asserts
    // that the downstream stage is pinned wherever zero happens to sit.
    //
    // That assertion is exactly the SAF-32 latch. With a range that straddles
    // zero it is harmless, but a caller gating direction with
    // set_output_limits(0.0f, 65000.0f) -- this codebase's own pattern, see
    // src/low-level-navigator.cpp -- puts a rail AT zero, so
    // `applied_ <= min_output_` is 0 <= 0, permanently true, and integration in
    // that direction is blocked forever. No measured over-speed can then remove
    // the integral term.
    //
    // Consumed before the guards below, so a report noted for a cycle that then
    // rejected its samples cannot speak for a later one. That is the one piece
    // of state the non-finite path touches, and it is the fail-safe direction:
    // dropping the report can only make the controller integrate MORE readily,
    // never assert a block it has no evidence for.
    const bool have_applied = have_applied_;
    have_applied_ = false;

    // std::clamp returns its input unchanged when both comparisons are false,
    // which is precisely what a NaN produces -- so the dt clamp below is not a
    // guard against non-finite input, and neither is the output clamp at the
    // end. Both have to be checked explicitly, before they are relied on.

    // Defence in depth. The encoder reports Reading{rpm, valid} and the caller
    // is supposed to gate on `valid`, but a controller that emits NaN when it
    // is lied to is a poor last line. Returning zero is safe here BECAUSE IT IS
    // AN OUTPUT: a zero command means "do not drive", which is the correct
    // response to an unusable input. A zero *measurement* would be a different
    // thing entirely -- a lie about the plant, claiming the wheel is stopped --
    // and must never be synthesised that way.
    //
    // No controller state is updated on this path. The sample is unusable, so
    // integrating it or adopting it as the derivative history would corrupt the
    // next cycle that does have good data. saturated() likewise keeps reporting
    // the last real cycle rather than claiming a fresh, fabricated result. The
    // sole exception is the note_applied() report consumed above, which expires
    // by design.
    //
    // Returning 0.0f is also the one place the output can fall OUTSIDE
    // [min_output_, max_output_]: for a range that excludes zero, clamping into
    // it would answer an unusable measurement by continuing to drive. Zero is
    // the safe actuator command whatever range is configured. Documented in
    // pid.hpp so callers do not treat the range as total.
    if (!std::isfinite(measurement) || !std::isfinite(setpoint))
    {
        return 0.0f;
    }

    // A caller passing a non-finite dt is broken, but substituting the nominal
    // control period keeps the loop running with slightly wrong integral
    // scaling, which beats emitting NaN into a motor command.
    if (!std::isfinite(dt))
    {
        dt = control_period_s;
    }
    dt = std::clamp(dt, dt_min_s, dt_max_s);

    const float error = setpoint - measurement;

    // Derivative on MEASUREMENT, not on error: a setpoint step would otherwise
    // put a (step / dt) spike straight onto the motors. The history is updated
    // unconditionally, before anything that could clamp or short-circuit, so
    // the next cycle's derivative is always referenced to the sample that was
    // actually seen.
    const float derivative = have_prior_ ? -(measurement - prior_measurement_) / dt : 0.0f;
    prior_measurement_ = measurement;
    have_prior_ = true;

    const float unsaturated = kp_ * error + ki_ * integral_ + kd_ * derivative;

    const float output = std::clamp(unsaturated, min_output_, max_output_);
    saturated_ = (output != unsaturated);

    // Conditional integration, evaluated AFTER the clamp so it sees this
    // cycle's saturation. `applied_` is what the downstream stages actually
    // sent: if a deadband or slew limit is swallowing the command, integrating
    // further only builds up authority that cannot be delivered.
    const bool blocked_high =
        (unsaturated > max_output_ || (have_applied && applied_ >= max_output_)) && error > 0.0f;
    const bool blocked_low =
        (unsaturated < min_output_ || (have_applied && applied_ <= min_output_)) && error < 0.0f;
    if (!blocked_high && !blocked_low)
    {
        integral_ += error * dt;
    }

    // Bound the integral term's contribution to the output range. ki_ == 0 is
    // the live configuration, so the guard is a real divide-by-zero path, not a
    // theoretical one.
    if (ki_ != 0.0f)
    {
        // The bounds are ordered explicitly rather than passed in min/max
        // order. The constructor now coerces a negative ki_ to zero, which this
        // branch already excludes, so the ordering should be unreachable -- but
        // std::clamp with lo > hi is undefined behaviour, the same case
        // set_output_limits rejects for the limits themselves, and two calls to
        // std::min/std::max cost nothing next to that.
        const float a = min_output_ / ki_;
        const float b = max_output_ / ki_;
        integral_ = std::clamp(integral_, std::min(a, b), std::max(a, b));
    }

    return output;
}

void PidClass::note_applied(float applied)
{
    applied_ = applied;
    have_applied_ = true;
}

void PidClass::reset()
{
    integral_ = 0.0f;
    prior_measurement_ = 0.0f;
    have_prior_ = false;
    saturated_ = false;

    // Also any unconsumed downstream report. The per-cycle flag already stops a
    // stale value from outliving the cycle it was noted for; this covers the
    // narrow case of a reset() taken between note_applied() and the
    // control_loop() that would have consumed it.
    //
    // The output limits are deliberately NOT restored: they are configuration,
    // not state, and a reset that quietly reverted a caller's range would be a
    // worse surprise than one that keeps it.
    applied_ = 0.0f;
    have_applied_ = false;
}

bool PidClass::saturated() const
{
    return saturated_;
}

bool PidClass::gains_valid() const
{
    return gains_valid_;
}

void PidClass::set_output_limits(float min, float max)
{
    if (!std::isfinite(min) || !std::isfinite(max) || min >= max)
    {
        return;
    }
    min_output_ = min;
    max_output_ = max;
}

float PidClass::get_max_output() const
{
    return max_output_;
}

float PidClass::get_min_output() const
{
    return min_output_;
}
