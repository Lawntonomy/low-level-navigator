#pragma once

// Arming, command freshness, and fault state.
//
// This module owns every decision the high-level tier is not allowed to make.
// It exists as one place because these rules are the ones that must survive the
// Pi being absent, stale, or wrong (ADR-0001), and scattering them through task
// bodies is how they get quietly weakened.
//
// Requirements this bears on:
//   SAF-1   ramp to zero on command loss — **PARTIAL**. The target is stepped to
//           zero, not ramped, and the terminal state is drive-removed (coast)
//           rather than drive-held. ADR-0001 requires a ramp; SAF-31 (slew
//           limit) does not exist yet. Do NOT read this module as closing SAF-1.
//   SAF-3   heartbeat distinguishes "nothing to say" from "wire fell off"
//   SAF-11  no drive without an explicit arming action
//   SAF-33  no message may relax a limit this tier enforces
//   IF-0001 §7.2  drain-to-newest, and freshness judged on arrival
//   IF-0001 §7.5  a stall is not a restart; only the latter disarms
//
// Writers run on core 0 (link RX); the reader runs on core 1 (control). All
// shared state crosses that boundary through the accessors below, which take a
// critical section — a genuine SMP-wide spinlock pair on this port, not merely
// an interrupt disable.
//
// **Timestamps are taken inside the lock, never passed in.** A caller that
// samples time_us_64() before acquiring the lock can be overtaken by the other
// core, making a stored stamp *newer* than the caller's "now" — and the
// resulting unsigned underflow reads as a multi-thousand-year gap, which trips
// every timeout at once. That is a real defect this interface exists to prevent
// rather than a hypothetical.

#include <cstdint>

namespace safety
{

// Mirrors LAWN_NAV_STATE in the dialect. Kept as its own type so the control
// path does not depend on MAVLink headers.
enum class NavState : uint8_t
{
    precal_idle = 0,
    calibrating = 1,
    idle = 2,
    active = 3,
    exiting = 4,
    fault = 5,
};

enum class Fault : uint8_t
{
    none = 0,
    cmd_timeout = 1,
    link_degraded = 2,
    wheel_stall = 3,
    wheel_invalid = 4,
    dir_mismatch = 5,
    tilt = 6,
    init_failed = 7,
};

// What the control task should do this iteration. Returned by value so the
// control loop never holds a lock while acting on it.
struct Decision
{
    bool armed;        // drive enable may be asserted
    int16_t left_drpm; // accepted target, deci-rpm
    int16_t right_drpm;
    NavState state;
    Fault fault;
    bool ramp_to_zero; // command or heartbeat lost; SAF-1 (partial, see above)
};

// Snapshot for telemetry. Read-only; never used for control decisions.
struct Status
{
    NavState state;
    Fault fault;
    bool armed;
    uint32_t cmd_age_ms; // 0xFFFFFFFF if none ever accepted
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t win_ok;
    uint32_t win_bad;
    uint32_t win_hb_missed;
    uint32_t window_ms;
    int16_t left_applied;
    int16_t right_applied;
};

void init();

// Our own session id, drawn once at boot. Changes only across a reset, which is
// what lets the Pi tell a stall from a restart (IF-0001 §7.5).
uint32_t session_id();

// ---- called from the link RX task (core 0) --------------------------------

// Returns true if the peer restarted, in which case the caller should log it.
// A restart disarms: a rebooted Pi has lost its pose estimate and its place in
// the plan, so its next command describes intentions formed before it knew
// either.
bool on_heartbeat(uint32_t peer_session);

void on_arm_request(bool arm, uint16_t magic);
void on_stop_request();

// Posts a drive request, stamping its ARRIVAL time. Newest wins: a burst
// draining after a Pi stall must collapse to its most recent frame rather than
// replay stale steering in order.
//
// Returns true if this superseded an unconsumed request — the caller should
// count that as rejected, because IF-0001 §7.3 wants a post-stall burst visible
// in link quality rather than reported as 100% healthy.
bool on_drive_request(int16_t left_drpm, int16_t right_drpm);

void on_frame_accepted();
void on_frame_rejected(uint32_t count = 1);

// ---- called from the control task (core 1) --------------------------------

// Evaluates timeouts and returns what to do. Must be called every control
// iteration: the timeouts are enforced here, in the task that owns actuation,
// not computed somewhere hopeful and read later.
Decision evaluate();

// Records what was actually applied after limiting, so telemetry can report
// requested and applied separately and divergence is visible.
void record_applied(int16_t left_drpm, int16_t right_drpm);

// Latches a fault. Disarms immediately. There is no clear path in protocol v1
// (IF-0001 §11 item 4) — a latched fault means a deliberate reset.
//
// NOTE: nothing calls this yet. Until something does, `Fault` can never leave
// `none`, which makes NavState::fault, MAV_STATE_CRITICAL and LAWN_FAULT_EVENT
// unreachable. Its first caller should be the SAF-54 link-quality check.
void raise_fault(Fault f);

// ---- called from the telemetry task (core 0) ------------------------------

Status status();

// Closes the link-quality window and returns what it held. IF-0001 §7.3.
Status take_window();

} // namespace safety
