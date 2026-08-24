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
} // namespace

PidClass::PidClass(float kp_init, float ki_init, float kd_init)
    : kp_(kp_init), ki_(ki_init), kd_(kd_init), min_output_(default_min_output),
      max_output_(default_max_output), integral_(0.0f), prior_measurement_(0.0f), applied_(0.0f),
      have_prior_(false), saturated_(false)
{
    Log::info(category, "init Pid");
}

PidClass::~PidClass()
{
}

float PidClass::control_loop(float measurement, float setpoint, float dt)
{
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
        (unsaturated > max_output_ || applied_ >= max_output_) && error > 0.0f;
    const bool blocked_low = (unsaturated < min_output_ || applied_ <= min_output_) && error < 0.0f;
    if (!blocked_high && !blocked_low)
    {
        integral_ += error * dt;
    }

    // Bound the integral term's contribution to the output range. ki_ == 0 is
    // the live configuration, so the guard is a real divide-by-zero path, not a
    // theoretical one.
    if (ki_ != 0.0f)
    {
        integral_ = std::clamp(integral_, min_output_ / ki_, max_output_ / ki_);
    }

    return output;
}

void PidClass::note_applied(float applied)
{
    applied_ = applied;
}

void PidClass::reset()
{
    integral_ = 0.0f;
    prior_measurement_ = 0.0f;
    have_prior_ = false;
    saturated_ = false;
}

bool PidClass::saturated() const
{
    return saturated_;
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
