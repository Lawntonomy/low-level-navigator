/*
 * Lawntonomy inter-tier link stub — RP2350 side.
 *
 * Exercises the decisions in IF-0001 (protocol) and TP-0001 (transport) on real
 * hardware. It is NOT the navigator: there is no control loop, no PID, no PWM,
 * and the wheel speeds it reports are synthetic. Its only job is to make the
 * link's behaviour measurable.
 *
 * Deliberately bare-metal (no FreeRTOS). The properties under test here are
 * transport properties; adding a scheduler would put an untested variable
 * between the wire and the measurement. TP-0001 Phase 2 needs the task-based
 * version — this is not it.
 *
 * What it demonstrates:
 *   D1  command link on uart0 GPIO 0/1; console on uart1 GPIO 20, unframed
 *   D2  1 Mbaud, with the achieved rate asserted at boot (silent-clamp guard)
 *   D3  no hardware flow control
 *   D4  64-bit microsecond timestamps
 *   D5  both stdio backends off; TX through a drop-on-full ring, never blocking
 *   D6  explicit pull-up on the command RX pin (RP2350-E9)
 *   §7.2 drain-to-newest for LAWN_DRIVE_CMD
 *   §7.5 stop vs disarm, with a session id distinguishing stall from restart
 *
 * Scope pin: GPIO 22 toggles once per loop iteration (TP-0001 Phase 2 ground
 * truth, independent of any firmware instrumentation that could itself stall).
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/rand.h"
#include "pico/stdlib.h"

#include "lawntonomy/mavlink.h"

/* ---------------------------------------------------------------- config -- */

#define CMD_UART uart0
#define CMD_TX_PIN 16 // matches already-wired, previously-confirmed hardware
#define CMD_RX_PIN 17 // (valid alt UART0 pins - same peripheral as GPIO 0/1)
#define CMD_BAUD 1000000

#define CON_UART uart1
#define CON_TX_PIN 20
#define CON_BAUD 115200

#define SCOPE_PIN 22

/* IF-0001 §3 */
#define SYSID 1
#define COMPID_PICO MAV_COMP_ID_AUTOPILOT1     /* 1   */
#define COMPID_PI MAV_COMP_ID_ONBOARD_COMPUTER /* 191 */

/* IF-0001 §7.1 / §7.5 — PROVISIONAL. T_cmd must come from SAF-2's coast-down
 * measurement, which has not been taken. These are placeholders. */
#define T_HB_US 50000u       /* 20 Hz    */
#define T_CMD_US 150000u     /* 3 x T_hb */
#define T_DISARM_US 1500000u /* 10 x T_cmd */

#define RATE_HB_US 50000u      /* 20 Hz */
#define RATE_STATUS_US 50000u  /* 20 Hz */
#define RATE_WHEEL_US 20000u   /* 50 Hz */
#define RATE_STATS_US 1000000u /* 1 Hz  */

#define TX_RING_BYTES 2048

/* ------------------------------------------------------------- tx ring ---- */

/* Single producer (main loop), single consumer (drain, also main loop). A frame
 * is pushed whole or dropped whole: a partially-written frame would desync the
 * far end, which is a worse failure than a gap the sequence number reveals. */
static uint8_t tx_buf[TX_RING_BYTES];
static uint16_t tx_head, tx_tail;
static uint32_t tx_dropped;
static uint16_t tx_peak;

static inline uint16_t tx_used(void)
{
    return (uint16_t)((tx_head - tx_tail) & (TX_RING_BYTES - 1));
}

static inline uint16_t tx_free(void)
{
    return (uint16_t)(TX_RING_BYTES - 1 - tx_used());
}

/* Returns false and counts a drop if the frame does not fit. Never blocks. */
static bool tx_push(const uint8_t* p, uint16_t n)
{
    if (n > tx_free())
    {
        tx_dropped++;
        return false;
    }
    for (uint16_t i = 0; i < n; i++)
    {
        tx_buf[tx_head] = p[i];
        tx_head = (uint16_t)((tx_head + 1) & (TX_RING_BYTES - 1));
    }
    if (tx_used() > tx_peak)
    {
        tx_peak = tx_used();
    }
    return true;
}

/* Move as many bytes as the FIFO will take. uart_is_writable() is the only
 * gate; we never spin on it, so a far end that stops reading costs us dropped
 * frames rather than a stalled loop. */
static void tx_drain(void)
{
    while (tx_used() && uart_is_writable(CMD_UART))
    {
        uart_get_hw(CMD_UART)->dr = tx_buf[tx_tail];
        tx_tail = (uint16_t)((tx_tail + 1) & (TX_RING_BYTES - 1));
    }
}

static bool send_msg(mavlink_message_t* msg)
{
    static uint8_t scratch[MAVLINK_MAX_PACKET_LEN];
    uint16_t n = mavlink_msg_to_send_buffer(scratch, msg);
    return tx_push(scratch, n);
}

/* ------------------------------------------------------------- console ---- */

/* Unframed, best-effort, and deliberately not MAVLink: its entire purpose is to
 * be readable before anything is initialised. Blocking here is acceptable only
 * because the console is never on the safety path — but keep it terse. */
static void con_puts(const char* s)
{
    while (*s)
    {
        uart_putc_raw(CON_UART, *s++);
    }
}

static void con_printf(const char* fmt, ...)
{
    static char line[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    con_puts(line);
}

/* --------------------------------------------------------------- state ---- */

static uint32_t session_id; /* §7.5 — changes only across a reset */
static uint32_t peer_session;
static bool peer_session_known;

static bool armed;
static uint8_t nav_state = LAWN_NAV_PRECAL_IDLE;
static uint8_t fault = LAWN_FAULT_NONE;

static uint64_t last_cmd_us; /* last ACCEPTED drive request */
static uint64_t last_hb_us;
static bool ever_cmd;

static int16_t left_target, right_target;   /* requested */
static int16_t left_applied, right_applied; /* after limiting  */

static uint32_t frames_ok, frames_bad;
static uint32_t win_ok, win_bad, win_hb_missed;
static uint64_t win_start_us;

/* Timestamp of the byte that opened the frame currently being parsed.
 * IF-0001 §7.4 wants the start-bit edge captured in hardware; polling the FIFO
 * is the cheap approximation and is good to roughly the FIFO trigger jitter.
 * TP-0001 T4.1 specifies the PIO/GPIO-IRQ version that replaces this. */
static uint64_t rx_frame_start_us;

/* -------------------------------------------------------------- senders --- */

static void send_heartbeat(void)
{
    mavlink_message_t m;
    mavlink_msg_heartbeat_pack(SYSID, COMPID_PICO, &m, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                               armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0,
                               session_id, /* §7.5 session id */
                               (fault != LAWN_FAULT_NONE) ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE);
    send_msg(&m);
}

static void send_nav_status(uint64_t now)
{
    uint32_t age = 65535;
    if (ever_cmd)
    {
        uint64_t d = (now - last_cmd_us) / 1000u;
        age = (d > 65534) ? 65534 : (uint32_t)d;
    }
    mavlink_message_t m;
    mavlink_msg_lawn_nav_status_pack(SYSID, COMPID_PICO, &m, now, (uint16_t)age, nav_state,
                                     armed ? 1 : 0, fault);
    send_msg(&m);
}

static void send_wheel_state(uint64_t now)
{
    /* Synthetic: the applied value plus a small lag, so the Pi sees something
     * that tracks its requests. There is no encoder behind this. */
    int16_t l = (int16_t)(left_applied - left_applied / 8);
    int16_t r = (int16_t)(right_applied - right_applied / 8);

    uint8_t flags = LAWN_WHEEL_LEFT_VALID | LAWN_WHEEL_RIGHT_VALID |
                    LAWN_WHEEL_DIR_UNMEASURED; /* ADR-0002: always set */

    mavlink_message_t m;
    mavlink_msg_lawn_wheel_state_pack(SYSID, COMPID_PICO, &m, now, l, r, left_applied,
                                      right_applied, flags);
    send_msg(&m);
}

static void send_link_stats(uint64_t now)
{
    uint32_t total = win_ok + win_bad;
    uint8_t q = total ? (uint8_t)((win_ok * 100u) / total) : 0u;

    mavlink_message_t m;
    mavlink_msg_lawn_link_stats_pack(SYSID, COMPID_PICO, &m, now, win_ok, win_bad, tx_dropped,
                                     (uint16_t)((now - win_start_us) / 1000u),
                                     (uint16_t)win_hb_missed, q);
    send_msg(&m);

    win_ok = win_bad = win_hb_missed = 0;
    win_start_us = now;
}

static void send_fault_event(uint64_t now, uint8_t code, bool latched)
{
    mavlink_message_t m;
    mavlink_msg_lawn_fault_event_pack(SYSID, COMPID_PICO, &m, now, code, nav_state,
                                      latched ? 1 : 0);
    send_msg(&m);
}

/*
 * TIMESYNC response. IF-0001 §7.4 requires t3 at the last byte leaving the
 * shift register; there is no hardware for that, so this drains the ring,
 * stamps, then writes the frame directly.
 *
 * This BLOCKS for roughly one frame time (~370 us at 1 Mbaud) and therefore
 * violates the never-block rule everywhere else in this file. It is confined to
 * a 5 Hz message and is here so the clock model can be measured at all. The
 * real firmware needs PIO edge capture (TP-0001 T4.1) instead.
 */
static void send_timesync_response(uint64_t t1, uint64_t t2, uint8_t seq)
{
    while (tx_used())
    {
        tx_drain();
    }
    uart_tx_wait_blocking(CMD_UART);

    mavlink_message_t m;
    mavlink_msg_lawn_timesync_pack(SYSID, COMPID_PICO, &m, t1, t2, time_us_64(), seq);

    static uint8_t scratch[MAVLINK_MAX_PACKET_LEN];
    uint16_t n = mavlink_msg_to_send_buffer(scratch, &m);
    uart_write_blocking(CMD_UART, scratch, n);
}

/* -------------------------------------------------------------- receive --- */

/* IF-0001 §7.2 rule 5: never queue drive requests. The newest one seen during
 * a single drain wins; the rest are counted and discarded. A stalled Pi drains
 * as a burst whose frames all arrive "fresh" while their contents predate the
 * stall — acting on them in order replays stale steering. */
static bool pending_drive;
static int16_t pending_left, pending_right;
static uint32_t superseded;

static void handle_message(const mavlink_message_t* m, uint64_t now)
{
    if (m->sysid != SYSID || m->compid != COMPID_PI)
    {
        frames_bad++;
        win_bad++;
        return;
    }

    frames_ok++;
    win_ok++;

    switch (m->msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(m, &hb);
        last_hb_us = now;

        if (!peer_session_known)
        {
            peer_session = hb.custom_mode;
            peer_session_known = true;
            con_printf("[link] peer session %08lx\r\n", (unsigned long)peer_session);
        }
        else if (hb.custom_mode != peer_session)
        {
            /* §7.5 — the Pi restarted. It has lost its pose estimate and its
             * place in the plan; its next command predates knowing either. */
            con_printf("[link] peer RESTART %08lx -> %08lx : disarm\r\n",
                       (unsigned long)peer_session, (unsigned long)hb.custom_mode);
            peer_session = hb.custom_mode;
            armed = false;
            nav_state = LAWN_NAV_IDLE;
            left_target = right_target = 0;
            send_fault_event(now, LAWN_FAULT_NONE, false);
        }
        break;
    }

    case MAVLINK_MSG_ID_LAWN_DRIVE_CMD:
    {
        mavlink_lawn_drive_cmd_t c;
        mavlink_msg_lawn_drive_cmd_decode(m, &c);
        if (pending_drive)
        {
            superseded++; /* older request in the same burst */
        }
        pending_drive = true;
        pending_left = c.left_drpm;
        pending_right = c.right_drpm;
        break;
    }

    case MAVLINK_MSG_ID_LAWN_ARM_CMD:
    {
        mavlink_lawn_arm_cmd_t a;
        mavlink_msg_lawn_arm_cmd_decode(m, &a);
        if (a.arm && a.magic == 0xA57E)
        {
            if (fault != LAWN_FAULT_NONE)
            {
                con_puts("[link] arm refused: fault latched\r\n");
            }
            else
            {
                armed = true;
                nav_state = LAWN_NAV_IDLE;
                con_puts("[link] ARMED\r\n");
            }
        }
        else
        {
            armed = false;
            nav_state = LAWN_NAV_IDLE;
            left_target = right_target = 0;
            con_puts("[link] disarmed\r\n");
        }
        break;
    }

    case MAVLINK_MSG_ID_LAWN_STOP_REQ:
        armed = false;
        nav_state = LAWN_NAV_EXITING;
        left_target = right_target = 0;
        con_puts("[link] stop requested\r\n");
        break;

    case MAVLINK_MSG_ID_LAWN_TIMESYNC:
    {
        mavlink_lawn_timesync_t ts;
        mavlink_msg_lawn_timesync_decode(m, &ts);
        if (ts.t2_us == 0)
        { /* a request, not somebody's response */
            send_timesync_response(ts.t1_us, rx_frame_start_us, ts.exchange_seq);
        }
        break;
    }

    default: break;
    }
}

static void rx_poll(uint64_t now)
{
    static mavlink_message_t msg;
    static mavlink_status_t status;

    while (uart_is_readable(CMD_UART))
    {
        uint8_t c = (uint8_t)uart_get_hw(CMD_UART)->dr;
        uint64_t ts = time_us_64();

        if (status.parse_state == MAVLINK_PARSE_STATE_IDLE)
        {
            rx_frame_start_us = ts; /* candidate STX */
        }

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            handle_message(&msg, now);
        }
    }

    /* Frames the parser rejected outright (bad CRC, bad CRC_EXTRA, bad length).
     * IF-0001 §7.3 wants these visible before they become a stop. */
    static uint16_t last_drops;
    if (status.packet_rx_drop_count != last_drops)
    {
        uint16_t d = (uint16_t)(status.packet_rx_drop_count - last_drops);
        frames_bad += d;
        win_bad += d;
        last_drops = status.packet_rx_drop_count;
    }
}

/* ----------------------------------------------------------------- main --- */

static void apply_limits(void)
{
    /* Stands in for the real slew and magnitude limits (SAF-31, SAF-33). The
     * point here is only that "requested" and "applied" are separate values and
     * the Pi is told both, so divergence is visible rather than silent. */
    const int16_t MAXD = 2000; /* 200.0 rpm */
    int16_t l = left_target, r = right_target;
    if (l > MAXD)
        l = MAXD;
    if (l < -MAXD)
        l = -MAXD;
    if (r > MAXD)
        r = MAXD;
    if (r < -MAXD)
        r = -MAXD;
    left_applied = armed ? l : 0;
    right_applied = armed ? r : 0;
}

int main(void)
{
    /* USB stdio is enabled (see CMakeLists.txt) SOLELY for picotool's vendor
     * reset interface - this file never calls printf/puts, so the blocking
     * write path D5 warns about has nothing to invoke it. stdio_init_all()
     * only brings up USB here: UART stdio is off at the CMake level, so the
     * LIB_PICO_STDIO_UART branch inside it compiles out. */
    stdio_init_all();

    gpio_init(SCOPE_PIN);
    gpio_set_dir(SCOPE_PIN, GPIO_OUT);

    /* Console first, so a failure below is reportable. */
    uart_init(CON_UART, CON_BAUD);
    gpio_set_function(CON_TX_PIN, GPIO_FUNC_UART);
    con_puts("\r\n\r\n[boot] lawntonomy link stub\r\n");

    /* Command link. */
    uint actual = uart_init(CMD_UART, CMD_BAUD);
    gpio_set_function(CMD_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(CMD_RX_PIN, GPIO_FUNC_UART);

    /* D6 / RP2350-E9: pads reset pulled-down with IE enabled, and on A2 silicon
     * leakage holds a floating pad near 2.2 V — right on the VIH boundary. An
     * absent Pi would then produce plausible garbage instead of clean silence,
     * and SAF-3's heartbeat would be discriminating against corruption rather
     * than absence. The errata confirms the pull-up still works. */
    gpio_pull_up(CMD_RX_PIN);

    uart_set_hw_flow(CMD_UART, false, false); /* D3 */
    uart_set_format(CMD_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(CMD_UART, true);

    /* D2: uart_set_baudrate clamps ibrd to >= 1 and reports nothing, so an
     * over-request silently becomes 3 Mbaud. Check what we actually got. */
    con_printf("[boot] cmd uart requested %u, achieved %u\r\n", (unsigned)CMD_BAUD,
               (unsigned)actual);
    int err_ppm = (int)(((int64_t)actual - CMD_BAUD) * 1000000 / CMD_BAUD);
    if (err_ppm > 20000 || err_ppm < -20000)
    {
        con_printf("[boot] FATAL: baud error %d ppm exceeds 2%%\r\n", err_ppm);
        while (1)
        {
            gpio_xor_mask(1u << SCOPE_PIN);
            sleep_ms(100);
        }
    }
    con_printf("[boot] baud error %d ppm\r\n", err_ppm);

    session_id = get_rand_32();
    con_printf("[boot] session %08lx\r\n", (unsigned long)session_id);
    con_puts("[boot] running\r\n");

    uint64_t now = time_us_64();
    uint64_t next_hb = now, next_status = now, next_wheel = now, next_stats = now;
    win_start_us = now;
    last_hb_us = now;

    uint64_t hb_gap_ref = now;

    while (1)
    {
        gpio_xor_mask(1u << SCOPE_PIN); /* TP-0001 Phase 2 ground truth */
        now = time_us_64();

        /* --- receive, then collapse the burst to its newest request ------- */
        pending_drive = false;
        rx_poll(now);

        if (pending_drive)
        {
            if (armed)
            {
                left_target = pending_left;
                right_target = pending_right;
                last_cmd_us = now;
                ever_cmd = true;
                if (nav_state == LAWN_NAV_IDLE)
                {
                    nav_state = LAWN_NAV_ACTIVE;
                }
            }
            /* If not armed the request is accepted as a frame and discarded as
             * an intention: SAF-11 means no amount of traffic arms us. */
        }

        /* --- timeouts ---------------------------------------------------- */
        uint64_t since_cmd = ever_cmd ? (now - last_cmd_us) : (uint64_t)0;
        uint64_t since_hb = now - last_hb_us;

        if (since_hb > T_HB_US * 2 && (now - hb_gap_ref) > T_HB_US)
        {
            win_hb_missed++;
            hb_gap_ref = now;
        }

        bool lost = (since_hb > T_CMD_US) || (ever_cmd && armed && since_cmd > T_CMD_US);

        if (lost && nav_state == LAWN_NAV_ACTIVE)
        {
            /* §7.5 — ramp to zero, stay armed. A pause is not a restart. */
            nav_state = LAWN_NAV_EXITING;
            left_target = right_target = 0;
            con_printf("[link] command lost (%llu ms) : ramp to zero\r\n",
                       (unsigned long long)(since_hb / 1000));
        }

        if (since_hb > T_DISARM_US && armed)
        {
            armed = false;
            nav_state = LAWN_NAV_IDLE;
            peer_session_known = false; /* force a fresh handshake */
            con_puts("[link] sustained loss : DISARM\r\n");
            send_fault_event(now, LAWN_FAULT_CMD_TIMEOUT, false);
        }

        if (nav_state == LAWN_NAV_EXITING && left_applied == 0 && right_applied == 0)
        {
            nav_state = armed ? LAWN_NAV_IDLE : LAWN_NAV_PRECAL_IDLE;
        }

        apply_limits();

        /* --- periodic transmit ------------------------------------------- */
        if (now >= next_hb)
        {
            send_heartbeat();
            next_hb = now + RATE_HB_US;
        }
        if (now >= next_status)
        {
            send_nav_status(now);
            next_status = now + RATE_STATUS_US;
        }
        if (now >= next_wheel)
        {
            send_wheel_state(now);
            next_wheel = now + RATE_WHEEL_US;
        }
        if (now >= next_stats)
        {
            send_link_stats(now);
            next_stats = now + RATE_STATS_US;
        }

        tx_drain();
    }
}
