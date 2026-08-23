#pragma once

// Arming, command freshness, and fault state.
//
// This module owns every decision the high-level tier is not allowed to make.
// It exists as one place because these rules are the ones that must survive the
// Pi being absent, stale, or wrong (ADR-0001), and scattering them through task
// bodies is how they get quietly weakened.
//
// Requirements implemented here:
//   SAF-1   ramp to zero when no valid command arrives within the timeout
//   SAF-3   heartbeat distinguishes "nothing to say" from "wire fell off"
//   SAF-11  no drive without an explicit arming action
//   SAF-33  no message may relax a limit this tier enforces
//   IF-0001 §7.2  drain-to-newest; a stale command must not be replayed
//   IF-0001 §7.5  a stall is not a restart; only the latter disarms
//
// Writers run on core 0 (link RX); the reader runs on core 1 (control). All
// shared state crosses that boundary through the accessors below, which take a
// critical section — an SMP-wide spinlock, not merely an interrupt disable.

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
    bool armed;          // drive enable may be asserted
    int16_t left_drpm;   // accepted target, deci-rpm
    int16_t right_drpm;
    NavState state;
    Fault fault;
    bool ramp_to_zero;   // command or heartbeat lost; SAF-1
};

// Snapshot for telemetry. Read-only; never used for control decisions.
struct Status
{
    NavState state;
    Fault fault;
    bool armed;
    uint32_t cmd_age_ms;   // 0xFFFFFFFF if none ever accepted
    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t win_ok;
    uint32_t win_bad;
    uint32_t win_hb_missed;
    uint64_t win_start_us;
    int16_t left_applied;
    int16_t right_applied;
};

void init(uint64_t now_us);

// Our own session id, drawn once at boot. Changes only across a reset, which is
// what lets the Pi tell a stall from a restart (IF-0001 §7.5).
uint32_t session_id();

// ---- called from the link RX task (core 0) --------------------------------

// Returns true if the peer restarted, in which case the caller should log it.
// A restart disarms: a rebooted Pi has lost its pose estimate and its place in
// the plan, so its next command describes intentions formed before it knew
// either.
bool on_heartbeat(uint64_t now_us, uint32_t peer_session);

void on_arm_request(bool arm, uint16_t magic, uint64_t now_us);
void on_stop_request(uint64_t now_us);

// Posts a drive request. Newest wins: a burst draining after a Pi stall must
// collapse to its most recent frame, not replay stale steering in order.
void on_drive_request(int16_t left_drpm, int16_t right_drpm, uint64_t now_us);

void on_frame_accepted();
void on_frame_rejected(uint32_t count = 1);

// ---- called from the control task (core 1) --------------------------------

// Evaluates timeouts and returns what to do. Must be called every control
// iteration: the timeouts are enforced here, in the task that owns actuation,
// not computed somewhere hopeful and read later.
Decision evaluate(uint64_t now_us);

// Records what was actually applied after limiting, so telemetry can report
// requested and applied separately and divergence is visible.
void record_applied(int16_t left_drpm, int16_t right_drpm);

// Latches a fault. Disarms immediately. There is no clear path in protocol v1
// (IF-0001 §11 item 4) — a latched fault means a deliberate reset.
void raise_fault(Fault f, uint64_t now_us);

// ---- called from the telemetry task (core 0) ------------------------------

Status status(uint64_t now_us);

// Closes the link-quality window and returns what it held. IF-0001 §7.3.
Status take_window(uint64_t now_us);

} // namespace safety
