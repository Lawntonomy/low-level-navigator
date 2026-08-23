#include "app/safety.hpp"

#include "FreeRTOS.h"
#include "pico/rand.h"
#include "task.h"

#include "app/log.hpp"

namespace safety
{
namespace
{

// IF-0001 §7.1 / §7.5. PROVISIONAL: SAF-2 requires the command timeout to be
// derived from measured stopping distance at maximum commanded speed and
// recorded with its derivation. That measurement has not been taken, and it
// depends on nothing in this firmware — it can be done today. Until then these
// are placeholders chosen to be self-consistent, not correct.
constexpr uint64_t t_cmd_us = 150000;     // 3 x heartbeat
constexpr uint64_t t_disarm_us = 1500000; // 10 x t_cmd

// Magnitude clamp standing in for the real limit set. SAF-31 (slew) and SAF-30
// (reversal through zero) are NOT implemented here and must be before this
// firmware drives anything with traction.
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

} // namespace

void init(uint64_t now_us)
{
    taskENTER_CRITICAL();
    s = State{};
    s.session = get_rand_32();
    s.nav = NavState::precal_idle;
    s.fault = Fault::none;
    s.last_hb_us = now_us;
    s.win_start_us = now_us;
    s.hb_gap_ref_us = now_us;
    taskEXIT_CRITICAL();
}

uint32_t session_id()
{
    taskENTER_CRITICAL();
    const uint32_t v = s.session;
    taskEXIT_CRITICAL();
    return v;
}

bool on_heartbeat(uint64_t now_us, uint32_t peer_session)
{
    bool restarted = false;

    taskENTER_CRITICAL();
    s.last_hb_us = now_us;

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

void on_arm_request(bool arm, uint16_t magic, uint64_t now_us)
{
    (void)now_us;

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
        // SAF-13 territory: a latched fault is not clearable by asking again.
        s.armed = false;
    }
    else
    {
        s.armed = true;
        s.nav = NavState::idle;
    }
    taskEXIT_CRITICAL();
}

void on_stop_request(uint64_t now_us)
{
    (void)now_us;

    taskENTER_CRITICAL();
    s.armed = false;
    s.acc_left = s.acc_right = 0;
    s.req_pending = false;
    s.nav = NavState::exiting;
    taskEXIT_CRITICAL();
}

void on_drive_request(int16_t left_drpm, int16_t right_drpm, uint64_t now_us)
{
    (void)now_us;

    taskENTER_CRITICAL();
    // Overwrite, never queue. If a previous request is still pending it was
    // superseded before the control loop ever saw it, which is the correct
    // outcome for a burst draining after a stall.
    s.req_pending = true;
    s.req_left = left_drpm;
    s.req_right = right_drpm;
    taskEXIT_CRITICAL();
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

Decision evaluate(uint64_t now_us)
{
    Decision d{};

    taskENTER_CRITICAL();

    // Accept a pending request only if armed. SAF-11: no volume of command
    // traffic arms the machine, and an unarmed request is discarded as an
    // intention even though it was accepted as a frame.
    if (s.req_pending)
    {
        if (s.armed && s.fault == Fault::none)
        {
            s.acc_left = s.req_left;
            s.acc_right = s.req_right;
            s.last_accept_us = now_us;
            s.ever_accepted = true;
            if (s.nav == NavState::idle)
            {
                s.nav = NavState::active;
            }
        }
        s.req_pending = false;
    }

    const uint64_t since_hb = now_us - s.last_hb_us;
    const uint64_t since_cmd = s.ever_accepted ? (now_us - s.last_accept_us) : 0;

    // Count a missed heartbeat at most once per nominal interval, so a long
    // outage does not inflate the window count without bound.
    if (since_hb > (t_cmd_us / 3) * 2 && (now_us - s.hb_gap_ref_us) > t_cmd_us / 3)
    {
        s.win_hb_missed++;
        s.hb_gap_ref_us = now_us;
    }

    const bool hb_lost = since_hb > t_cmd_us;
    const bool cmd_lost = s.ever_accepted && s.armed && since_cmd > t_cmd_us;
    const bool lost = hb_lost || cmd_lost;

    if (lost)
    {
        // SAF-1: ramp to zero. IF-0001 §7.5: a stall does NOT disarm. A short
        // interruption on a non-PREEMPT_RT Linux host is expected, and forcing
        // a re-arm handshake for every scheduling hiccup would make the arming
        // action a reflex rather than a decision.
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

    // Clamp outside the lock. SAF-14 is enforced again at the PWM boundary;
    // this is the policy limit, not the hardware one.
    if (d.left_drpm > max_drpm) d.left_drpm = max_drpm;
    if (d.left_drpm < -max_drpm) d.left_drpm = -max_drpm;
    if (d.right_drpm > max_drpm) d.right_drpm = max_drpm;
    if (d.right_drpm < -max_drpm) d.right_drpm = -max_drpm;

    return d;
}

void record_applied(int16_t left_drpm, int16_t right_drpm)
{
    taskENTER_CRITICAL();
    s.applied_left = left_drpm;
    s.applied_right = right_drpm;
    if (s.nav == NavState::exiting && left_drpm == 0 && right_drpm == 0)
    {
        s.nav = s.armed ? NavState::idle : NavState::precal_idle;
    }
    taskEXIT_CRITICAL();
}

void raise_fault(Fault f, uint64_t now_us)
{
    (void)now_us;

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
Status fill(uint64_t now_us)
{
    Status st{};
    st.state = s.nav;
    st.fault = s.fault;
    st.armed = s.armed;
    st.cmd_age_ms = s.ever_accepted
                        ? static_cast<uint32_t>((now_us - s.last_accept_us) / 1000)
                        : 0xFFFFFFFFu;
    st.frames_ok = s.frames_ok;
    st.frames_bad = s.frames_bad;
    st.win_ok = s.win_ok;
    st.win_bad = s.win_bad;
    st.win_hb_missed = s.win_hb_missed;
    st.win_start_us = s.win_start_us;
    st.left_applied = s.applied_left;
    st.right_applied = s.applied_right;
    return st;
}
} // namespace

Status status(uint64_t now_us)
{
    taskENTER_CRITICAL();
    const Status st = fill(now_us);
    taskEXIT_CRITICAL();
    return st;
}

Status take_window(uint64_t now_us)
{
    taskENTER_CRITICAL();
    const Status st = fill(now_us);
    s.win_ok = s.win_bad = s.win_hb_missed = 0;
    s.win_start_us = now_us;
    taskEXIT_CRITICAL();
    return st;
}

} // namespace safety
