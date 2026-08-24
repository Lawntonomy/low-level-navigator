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
    PidClass(float kp_init, float ki_init, float kd_init);
    ~PidClass();

    // One control cycle. `dt` is the elapsed time in SECONDS since the previous
    // call; it is clamped to a sane window rather than rejected, so that a late
    // or duplicated cycle can never skip the state update.
    float control_loop(float measurement, float setpoint, float dt);

    // What the downstream stages (slew limit, deadband map, PWM clamp) actually
    // sent. Feeds conditional-integration anti-windup.
    void note_applied(float applied);

    // Clears the integral, the derivative history and the saturation flag.
    void reset();

    // True when the last control_loop() call had to clamp its output. Replaces
    // the per-iteration warn logging: telemetry samples this, the control loop
    // never formats a string.
    bool saturated() const;

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
};
