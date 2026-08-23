#include "app/safety.hpp"

#include "FreeRTOS.h"
#include "pico/rand.h"
#include "pico/time.h"
#include "task.h"

namespace safety
{
namespace
{

// IF-0001 §7.1 / §7.5. PROVISIONAL: SAF-2 requires the command timeout to be
// derived from measured stopping distance at maximum commanded speed and
// recorded with its derivation. TP-0002 specifies how to measure it; it has not
// been measured. These are placeholders chosen to be self-consistent, not
// correct.
constexpr uint64_t t_cmd_us = 150000;     // 3 x heartbeat
constexpr uint64_t t_disarm_us = 1500000; // 10 x t_cmd

// Magnitude clamp standing in for the real limit set. SAF-31 (slew) and SAF-30
// (reversal through zero) are NOT implemented and must be before this firmware
// drives anything with traction. The value itself is arbitrary pending CAL-2.
constexpr int16_t max_drpm = 2000; // 200.0 rpm

constexpr uint16_t arm_magic = 0xA57E;

struct State
{
    uint32_t session;
    uint32_t peer_session;
    bool peer_known;

    bool armed;
    NavState nav;
    Fault fault;

    // Pending drive request. Newest-wins mailbox: on_drive_request overwrites
    // rather than queueing, which is IF-0001 §7.2 rule 5 expressed as a data
    // structure instead of a rule someone has to remember.
    bool req_pending;
    int16_t req_left;
    int16_t req_right;
    uint64_t req_arrival_us; // when it ARRIVED, not when it is consumed

    // Last ACCEPTED request, held between arrivals so a 200 Hz control loop
    // does not zero the target between 20 Hz commands.
    int16_t acc_left;
    int16_t acc_right;
    bool ever_accepted;
    uint64_t last_accept_us;

    uint64_t last_hb_us;

    int16_t applied_left;
    int16_t applied_right;

    uint32_t frames_ok;
    uint32_t frames_bad;
    uint32_t win_ok;
    uint32_t win_bad;
    uint32_t win_hb_missed;
    uint64_t win_start_us;
    uint64_t hb_gap_ref_us;
};

State s{};

// Monotonic difference that cannot underflow.
//
// Every timestamp here is now sampled inside the same critical section that
// reads it, so `now` should never precede a stored stamp. This is belt and
// braces: an underflow in a timeout comparison does not degrade gracefully, it
// reads as ~584,000 years elapsed and trips everything simultaneously.
inline uint64_t elapsed(uint64_t now, uint64_t then)
{
    return (now >= then) ? (now - then) : 0;
}

} // namespace

void init()
{
    taskENTER_CRITICAL();
    const uint64_t now = time_us_64();
    s = State{};
    s.session = get_rand_32();
    s.nav = NavState::precal_idle;
    s.fault = Fault::none;
    s.last_hb_us = now;
    s.win_start_us = now;
    s.hb_gap_ref_us = now;
    taskEXIT_CRITICAL();
}

uint32_t session_id()
{
    taskENTER_CRITICAL();
    const uint32_t v = s.session;
    taskEXIT_CRITICAL();
    return v;
}

bool on_heartbeat(uint32_t peer_session)
{
    bool restarted = false;

    taskENTER_CRITICAL();
    s.last_hb_us = time_us_64();

    if (!s.peer_known)
    {
        s.peer_session = peer_session;
        s.peer_known = true;
    }
    else if (peer_session != s.peer_session)
    {
        // IF-0001 §7.5 — a restart, not a stall.
        s.peer_session = peer_session;
        s.armed = false;
        s.acc_left = s.acc_right = 0;
        s.req_pending = false;
        s.ever_accepted = false;
        s.nav = (s.fault == Fault::none) ? NavState::idle : NavState::fault;
        restarted = true;
    }
    taskEXIT_CRITICAL();

    return restarted;
}

void on_arm_request(bool arm, uint16_t magic)
{
    taskENTER_CRITICAL();
    if (!arm || magic != arm_magic)
    {
        // Anything that is not an unambiguous arm is a disarm. A corrupted
        // frame that passed CRC must not be able to enable drive.
        s.armed = false;
        s.acc_left = s.acc_right = 0;
        s.req_pending = false;
        s.nav = (s.fault == Fault::none) ? NavState::idle : NavState::fault;
    }
    else if (s.fault != Fault::none)
    {
        // A latched fault is not clearable by asking again.
        s.armed = false;
    }
    else
    {
        s.armed = true;
        s.nav = NavState::idle;

        // SAF-11: arming is a decision, and the first motion after it must come
        // from a command issued afterwards. Without this, a LAWN_DRIVE_CMD that
        // arrived while disarmed — in the same RX batch, microseconds earlier —
        // would be applied at the instant of arming.
        s.req_pending = false;
    }
    taskEXIT_CRITICAL();
}

void on_stop_request()
{
    taskENTER_CRITICAL();
    s.armed = false;
    s.acc_left = s.acc_right = 0;
    s.req_pending = false;
    s.nav = NavState::exiting;
    taskEXIT_CRITICAL();
}

bool on_drive_request(int16_t left_drpm, int16_t right_drpm)
{
    taskENTER_CRITICAL();
    // Overwrite, never queue. If a previous request is still pending it was
    // superseded before the control loop ever saw it, which is the correct
    // outcome for a burst draining after a stall — and worth counting, because
    // otherwise that burst reports as 100% link quality.
    const bool superseded = s.req_pending;
    s.req_pending = true;
    s.req_left = left_drpm;
    s.req_right = right_drpm;
    s.req_arrival_us = time_us_64();
    taskEXIT_CRITICAL();
    return superseded;
}

void on_frame_accepted()
{
    taskENTER_CRITICAL();
    s.frames_ok++;
    s.win_ok++;
    taskEXIT_CRITICAL();
}

void on_frame_rejected(uint32_t count)
{
    taskENTER_CRITICAL();
    s.frames_bad += count;
    s.win_bad += count;
    taskEXIT_CRITICAL();
}

Decision evaluate()
{
    Decision d{};

    taskENTER_CRITICAL();
    const uint64_t now = time_us_64();

    // Accept a pending request only if armed, and only if it is still fresh
    // *as of when it arrived* (IF-0001 §7.2 rule 4). Judging freshness at the
    // moment of consumption would hide the receive path's own latency from the
    // very timeout that is supposed to bound it.
    if (s.req_pending)
    {
        const bool fresh = elapsed(now, s.req_arrival_us) <= t_cmd_us;
        if (s.armed && s.fault == Fault::none && fresh)
        {
            s.acc_left = s.req_left;
            s.acc_right = s.req_right;
            s.last_accept_us = s.req_arrival_us;
            s.ever_accepted = true;
            if (s.nav == NavState::idle)
            {
                s.nav = NavState::active;
            }
        }
        else if (!fresh)
        {
            s.win_bad++;
            s.frames_bad++;
        }
        // An unarmed request is accepted as a frame and discarded as an
        // intention. SAF-11: no volume of command traffic arms the machine.
        s.req_pending = false;
    }

    const uint64_t since_hb = elapsed(now, s.last_hb_us);
    const uint64_t since_cmd = s.ever_accepted ? elapsed(now, s.last_accept_us) : 0;

    // Count a missed heartbeat at most once per nominal interval, so a long
    // outage does not inflate the window count without bound.
    if (since_hb > (t_cmd_us / 3) * 2 && elapsed(now, s.hb_gap_ref_us) > t_cmd_us / 3)
    {
        s.win_hb_missed++;
        s.hb_gap_ref_us = now;
    }

    const bool hb_lost = since_hb > t_cmd_us;
    const bool cmd_lost = s.ever_accepted && s.armed && since_cmd > t_cmd_us;
    const bool lost = hb_lost || cmd_lost;

    if (lost)
    {
        // SAF-1, partially. The target is stepped to zero; there is no ramp,
        // because SAF-31 does not exist. IF-0001 §7.5: a stall does NOT disarm.
        // A short interruption on a non-PREEMPT_RT Linux host is expected, and
        // forcing a re-arm handshake for every scheduling hiccup would make the
        // arming action a reflex rather than a decision.
        s.acc_left = s.acc_right = 0;
        if (s.nav == NavState::active)
        {
            s.nav = NavState::exiting;
        }
    }

    if (since_hb > t_disarm_us && s.armed)
    {
        // Sustained loss. Now the peer's state is genuinely in question.
        s.armed = false;
        s.peer_known = false; // force a fresh handshake
        s.nav = NavState::idle;
    }

    d.armed = s.armed && s.fault == Fault::none;
    d.left_drpm = d.armed ? s.acc_left : 0;
    d.right_drpm = d.armed ? s.acc_right : 0;
    d.state = s.nav;
    d.fault = s.fault;
    d.ramp_to_zero = lost || !d.armed;

    taskEXIT_CRITICAL();

    // Clamp outside the lock. SAF-14 must be enforced again at the PWM
    // boundary; this is the policy limit, not the hardware one.
    if (d.left_drpm > max_drpm)
        d.left_drpm = max_drpm;
    if (d.left_drpm < -max_drpm)
        d.left_drpm = -max_drpm;
    if (d.right_drpm > max_drpm)
        d.right_drpm = max_drpm;
    if (d.right_drpm < -max_drpm)
        d.right_drpm = -max_drpm;

    return d;
}

void record_applied(int16_t left_drpm, int16_t right_drpm)
{
    taskENTER_CRITICAL();
    s.applied_left = left_drpm;
    s.applied_right = right_drpm;
    if (s.nav == NavState::exiting && left_drpm == 0 && right_drpm == 0)
    {
        // Never precal_idle: IF-0001 §7.5 reserves that for the post-reset
        // state, and reporting it after a routine stop would tell the Pi the
        // machine had lost its calibration.
        s.nav = NavState::idle;
    }
    taskEXIT_CRITICAL();
}

void raise_fault(Fault f)
{
    taskENTER_CRITICAL();
    if (s.fault == Fault::none)
    {
        s.fault = f;
    }
    // SAF-13: drive comes off before the fault is reported, not after.
    s.armed = false;
    s.acc_left = s.acc_right = 0;
    s.req_pending = false;
    s.nav = NavState::fault;
    taskEXIT_CRITICAL();
}

namespace
{
// Caller must hold the critical section.
Status fill(uint64_t now)
{
    Status st{};
    st.state = s.nav;
    st.fault = s.fault;
    st.armed = s.armed;
    st.cmd_age_ms = s.ever_accepted ? static_cast<uint32_t>(elapsed(now, s.last_accept_us) / 1000)
                                    : 0xFFFFFFFFu;
    st.frames_ok = s.frames_ok;
    st.frames_bad = s.frames_bad;
    st.win_ok = s.win_ok;
    st.win_bad = s.win_bad;
    st.win_hb_missed = s.win_hb_missed;
    st.window_ms = static_cast<uint32_t>(elapsed(now, s.win_start_us) / 1000);
    st.left_applied = s.applied_left;
    st.right_applied = s.applied_right;
    return st;
}
} // namespace

Status status()
{
    taskENTER_CRITICAL();
    const Status st = fill(time_us_64());
    taskEXIT_CRITICAL();
    return st;
}

Status take_window()
{
    taskENTER_CRITICAL();
    const uint64_t now = time_us_64();
    const Status st = fill(now);
    s.win_ok = s.win_bad = s.win_hb_missed = 0;
    s.win_start_us = now;
    taskEXIT_CRITICAL();
    return st;
}

} // namespace safety
