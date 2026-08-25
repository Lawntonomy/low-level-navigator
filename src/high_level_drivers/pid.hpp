#pragma once

// Positional (parallel) PID controller, one per wheel.
//
// u = kp*e + ki*integral(e) + kd*d(-measurement)/dt, saturated to
// [min_output, max_output]. The output is a pure function of the present
// state, so an absolute command is always expressible: reset() genuinely
// produces zero, and ramp-to-zero (SAF-1), stop-before-reversal (SAF-30) and
// disarm can all drive the controller to a known output.
//
// This class does NOT slew-limit (SAF-31) and does NOT apply the CAL-1
// deadband map. Those are separate downstream stages. Because they sit between
// this controller and the hardware, the controller cannot know what was
// actually applied -- tell it with note_applied() so anti-windup references
// reality rather than its own unclamped intent.
//
// Nothing here depends on the Pico SDK or FreeRTOS: the host tests in test/
// compile this file directly.

class PidClass
{
  public:
    // Gains are caller-supplied and deliberately have no defaults; they are
    // re-derived from CAL-1/CAL-2 results, not inherited from the pre-rewrite
    // firmware (where kp was really an integral rate).
    //
    // A gain that is NEGATIVE or NON-FINITE is coerced to 0.0f and gains_valid()
    // reports false. A negative gain turns negative feedback into positive
    // feedback -- the controller answers "go faster" with full reverse and holds
    // it there -- and a non-finite one poisons every term downstream of it.
    // Coercion, not rejection, because a zero gain is inert: the term simply
    // stops contributing, which is the safe degradation and leaves the caller
    // with a working (if reduced) controller. ZERO GAINS ARE LEGITIMATE and are
    // not coerced or flagged: ki = kd = 0 is the live configuration.
    PidClass(float kp_init, float ki_init, float kd_init);
    ~PidClass();

    // One control cycle. `dt` is the elapsed time in SECONDS since the previous
    // call; it is clamped to a sane window rather than rejected, so that a late
    // or duplicated cycle can never skip the state update. A non-finite `dt`
    // (which the clamp cannot catch) falls back to the nominal control period.
    //
    // A non-finite `measurement` or `setpoint` returns 0.0f and updates no
    // controller state: the sample is unusable, and a zero OUTPUT means "do not
    // drive".
    //
    // That 0.0f is a DELIBERATE EXCEPTION to the otherwise total invariant that
    // the return value lies within [min_output, max_output]. For a range that
    // does not contain zero -- e.g. the [0, 65000] direction gate -- zero is
    // outside it, and clamping into the range instead would answer an unusable
    // measurement by continuing to drive. Callers that assume the return is
    // always within the configured range must not.
    //
    // Every call consumes any pending note_applied() report; see below.
    float control_loop(float measurement, float setpoint, float dt);

    // What the downstream stages (slew limit, deadband map, PWM clamp) actually
    // sent. Feeds conditional-integration anti-windup.
    //
    // The report is good for exactly ONE control_loop() call, which consumes it.
    // Calling it is optional: a controller that is never told what was applied
    // falls back to conditional integration driven purely by its own output
    // clamp. That fallback is why the report expires -- a stale or absent value
    // reads as "downstream is pinned at a rail", which blocks integration in
    // that direction indefinitely. With min_output == 0 or max_output == 0
    // (the SAF-30 direction gate) the default of 0.0f sits exactly on a rail,
    // so an assumed report would latch the integral on the very first cycle.
    void note_applied(float applied);

    // Clears controller STATE -- the integral, the derivative history, the
    // saturation flag and any unconsumed note_applied() report -- and preserves
    // CONFIGURATION: the gains and the output limits are left exactly as they
    // were. A reset controller is therefore NOT identical to a freshly
    // constructed one, which carries the default limits of [-1, 1]. Restoring
    // the limits here would silently discard whatever range the caller had
    // configured, which is the worse failure of the two.
    void reset();

    // True when the last control_loop() call had to clamp its output. Replaces
    // the per-iteration warn logging: telemetry samples this, the control loop
    // never formats a string.
    bool saturated() const;

    // False when any gain handed to the constructor was negative or non-finite
    // and therefore coerced to 0.0f. Sampled by telemetry: a controller running
    // on coerced gains is running a configuration nobody asked for, and that
    // should be visible rather than inferred from the motion.
    bool gains_valid() const;

    // Rejects min >= max and non-finite bounds, leaving the previous limits in
    // place. Rejection is silent -- read it back with the accessors. An
    // inverted range would be undefined behaviour in the clamp below, and a
    // degenerate one would destroy all drive authority.
    void set_output_limits(float min, float max);

    float get_max_output() const;
    float get_min_output() const;

  private:
    float kp_;
    float ki_;
    float kd_;

    float min_output_;
    float max_output_;

    float integral_;
    float prior_measurement_;
    float applied_;

    bool have_prior_;
    bool saturated_;
    bool have_applied_;
    bool gains_valid_;
};
