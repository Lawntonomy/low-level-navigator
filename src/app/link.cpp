#include "app/link.hpp"

#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include "task.h"

#include "app/board.h"
#include "app/bootloader.hpp"
#include "app/log.hpp"
#include "app/safety.hpp"

#include "lawntonomy/mavlink.h"

namespace link
{
namespace
{

constexpr uint8_t sysid = 1;
constexpr uint8_t compid_self = MAV_COMP_ID_AUTOPILOT1;       // 1
constexpr uint8_t compid_peer = MAV_COMP_ID_ONBOARD_COMPUTER; // 191

constexpr uint32_t ring_bytes = 2048; // power of two
constexpr uint32_t ring_mask = ring_bytes - 1;
static_assert((ring_bytes & ring_mask) == 0, "ring must be a power of two");

uint8_t tx_buf[ring_bytes];
uint32_t tx_head;
uint32_t tx_tail;
volatile uint32_t tx_drop; // read unlocked by tx_dropped()
volatile uint32_t tx_peak; // read unlocked by tx_peak_bytes()

// Parser state. Touched only by the link RX task, so it needs no lock — but it
// must stay that way; nothing else may call rx_poll().
mavlink_message_t rx_msg;
mavlink_status_t rx_status;

// Deferred TIMESYNC response, handed from the RX task to the TX task.
struct PendingSync
{
    bool armed;
    uint64_t t1;
    uint64_t t2;
    uint8_t seq;
};
PendingSync pending_sync;

// RX byte ring. Produced by the UART interrupt, consumed by rx_poll() on the
// link RX task.
//
// Single producer, single consumer, and BOTH RUN ON CORE 0: link::init() is
// called from main() before vTaskStartScheduler(), so irq_set_enabled() arms
// the interrupt on core 0's NVIC, and the RX task is pinned to rt::core_service
// which is core 0. That is what makes plain volatile sufficient here rather
// than the real cross-core synchronisation rt.h warns about. **If either half
// ever moves cores, this needs revisiting**, not merely re-testing.
//
// 512 bytes is 16x the UART's 32-byte FIFO and 5.12 ms of airtime at 1 Mbaud,
// against a 1 ms task period — so the ring, not the FIFO, absorbs a burst.
constexpr uint32_t rx_ring_bytes = 512;
constexpr uint32_t rx_ring_mask = rx_ring_bytes - 1;
static_assert((rx_ring_bytes & rx_ring_mask) == 0, "ring must be a power of two");

uint8_t rx_buf[rx_ring_bytes];
volatile uint32_t rx_head;    // written by the ISR only
volatile uint32_t rx_tail;    // written by the task only
volatile uint32_t rx_overrun; // bytes lost because the ring was full

// Arrival stamp for the first byte of a burst, and the ring position it belongs
// to, so the task can tell WHICH byte it describes rather than assuming.
volatile uint64_t rx_burst_us;
volatile uint32_t rx_burst_pos;
volatile bool rx_burst_stamped;

// Timestamp of the byte that opened the frame being parsed, and whether it can
// be trusted.
//
// IF-0001 §7.4 requires t2 at the start-bit edge, captured in hardware. This is
// the moment the UART interrupt observed the first byte of a burst instead.
// TP-0001 T4.1's table puts that in the **±25 µs** class, dominated by FIFO
// trigger-level jitter — against the **±1 ms** class the previous task-polled
// version sat in, which the same table marks FORBIDDEN and labels "the trap".
//
// T4.1's ±2 µs class still needs a pin-edge capture (GPIO IRQ or PIO). That is
// deliberately not done here: it fixes the stamp but not the FIFO overrun this
// change also fixes, and it needs an arm/disarm dance to avoid one interrupt
// per data edge. Add it if the clock-residual measurement (ADR-0009's "single
// most important bench measurement") shows the fit is sync-limited.
//
// `valid` is false when a frame began on a byte whose burst stamp had already
// been consumed — i.e. two frames arrived without the ring draining between
// them. A TIMESYNC answered from a stale t2 corrupts the fit silently, so the
// response is declined instead and the peer retries.
uint64_t rx_frame_start_us;
bool rx_frame_start_valid;
volatile uint32_t sync_declined; // TIMESYNC requests dropped for a stale t2

// UART RX interrupt. Drains the hardware FIFO into the ring so that FIFO
// residency is bounded by interrupt latency rather than by the 1 ms task
// period; at 1 Mbaud the 32-byte FIFO overflows in 320 us, so the old poll was
// three times slower than the thing it was racing.
void on_cmd_uart_rx()
{
    uart_hw_t* const hw = uart_get_hw(board::cmd_uart());

    // Taken once, at ISR entry, before any FIFO read: this is the closest
    // observation of the burst's arrival available without a pin-edge capture.
    const uint64_t now = time_us_64();

    // Empty *before* this batch means the next byte pushed opens a new burst.
    bool opens_burst = (rx_head == rx_tail);

    while (!(hw->fr & UART_UARTFR_RXFE_BITS))
    {
        const uint8_t c = static_cast<uint8_t>(hw->dr);
        const uint32_t next = (rx_head + 1) & rx_ring_mask;

        // Full. Keep draining the FIFO — leaving bytes in it only converts a
        // ring overrun into a hardware overrun, and loses the same data.
        if (next == rx_tail)
        {
            rx_overrun++;
            continue;
        }

        if (opens_burst)
        {
            rx_burst_us = now;
            rx_burst_pos = rx_head;
            rx_burst_stamped = true;
            opens_burst = false;
        }

        rx_buf[rx_head] = c;
        rx_head = next;
    }

    hw->icr = UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS;
}

inline uint32_t tx_used_unsafe()
{
    return (tx_head - tx_tail) & ring_mask;
}

// Caller must hold the critical section.
bool push_unsafe(const uint8_t* p, uint32_t n)
{
    if (n > ring_mask - tx_used_unsafe())
    {
        tx_drop++;
        return false;
    }
    for (uint32_t i = 0; i < n; i++)
    {
        tx_buf[(tx_head + i) & ring_mask] = p[i];
    }
    tx_head = (tx_head + n) & ring_mask;
    if (tx_used_unsafe() > tx_peak)
    {
        tx_peak = tx_used_unsafe();
    }
    return true;
}

// Pack and enqueue under one lock.
//
// mavlink_msg_*_pack() ends in a read-modify-write of a shared static tx
// sequence number with no guard of its own. Two tasks pack frames here
// (telemetry, and the TX task's TIMESYNC response), so without this they can
// be handed the same seq — corrupting the one mechanism that makes a dropped
// frame visible at the far end.
template <typename PackFn> bool pack_and_push(PackFn pack)
{
    mavlink_message_t m;
    uint8_t scratch[MAVLINK_MAX_PACKET_LEN];

    taskENTER_CRITICAL();
    pack(&m);
    const uint16_t n = mavlink_msg_to_send_buffer(scratch, &m);
    const bool ok = push_unsafe(scratch, n);
    taskEXIT_CRITICAL();
    return ok;
}

// Microseconds to clock n bytes out at 8N1 (10 bits per byte).
constexpr uint64_t wire_time_us(uint16_t n)
{
    return (static_cast<uint64_t>(n) * 10u * 1000000u) / board::cmd_baud;
}

// Emit the deferred TIMESYNC response. TX task only.
//
// t3 must refer to the LAST byte leaving the shift register (IF-0001 §7.4) —
// for a sync frame, transmit time IS the measurement. Stamping before the write
// would put t3 one whole frame early, and because that error is one-sided it is
// a bias the min-filtered regression on the Pi cannot remove.
//
// So: drain, wait for the shifter, then stamp t3 = now + the frame's own wire
// time. The length is learned by packing once; both passes carry a non-zero t3
// so v2's trailing-zero truncation cannot change it between them, which is
// checked rather than assumed.
void emit_timesync(const PendingSync& p)
{
    uart_tx_wait_blocking(board::cmd_uart());

    uint8_t scratch[MAVLINK_MAX_PACKET_LEN];

    mavlink_message_t probe;
    mavlink_msg_lawn_timesync_pack(sysid, compid_self, &probe, p.t1, p.t2, time_us_64(), p.seq);
    const uint16_t probe_len = mavlink_msg_to_send_buffer(scratch, &probe);

    const uint64_t t3 = time_us_64() + wire_time_us(probe_len);

    mavlink_message_t m;
    mavlink_msg_lawn_timesync_pack(sysid, compid_self, &m, p.t1, p.t2, t3, p.seq);
    const uint16_t len = mavlink_msg_to_send_buffer(scratch, &m);

    if (len != probe_len)
    {
        // Length moved between passes, so the wire time we folded into t3 is
        // wrong. Report rather than emit a silently biased sync sample.
        log_console::write("[link] timesync len %u!=%u, dropped\r\n", (unsigned)len,
                           (unsigned)probe_len);
        return;
    }

    uart_write_blocking(board::cmd_uart(), scratch, len);
}

void dispatch(const mavlink_message_t* m)
{
    // IF-0001 §3. The parser will happily hand us a well-formed frame from a
    // stranger; dropping it is the receiver's job, not the parser's.
    if (m->sysid != sysid || m->compid != compid_peer)
    {
        safety::on_frame_rejected();
        return;
    }

    safety::on_frame_accepted();

    switch (m->msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(m, &hb);
        if (safety::on_heartbeat(hb.custom_mode))
        {
            log_console::write("[link] peer restarted -> disarm\r\n");
        }
        break;
    }

    case MAVLINK_MSG_ID_LAWN_DRIVE_CMD:
    {
        mavlink_lawn_drive_cmd_t c;
        mavlink_msg_lawn_drive_cmd_decode(m, &c);
        if (safety::on_drive_request(c.left_drpm, c.right_drpm))
        {
            // Superseded before the control loop saw it. IF-0001 §7.3 wants a
            // post-stall burst visible in link quality, not reported as
            // perfectly healthy traffic.
            safety::on_frame_rejected();
        }
        break;
    }

    case MAVLINK_MSG_ID_LAWN_ARM_CMD:
    {
        mavlink_lawn_arm_cmd_t a;
        mavlink_msg_lawn_arm_cmd_decode(m, &a);
        safety::on_arm_request(a.arm != 0, a.magic);
        log_console::write("[link] arm request: arm=%u magic=%04x\r\n", (unsigned)a.arm,
                           (unsigned)a.magic);
        break;
    }

    case MAVLINK_MSG_ID_LAWN_STOP_REQ:
        safety::on_stop_request();
        log_console::write("[link] stop requested\r\n");
        break;

    case MAVLINK_MSG_ID_LAWN_ENTER_BOOTLOADER:
    {
        mavlink_lawn_enter_bootloader_t b;
        mavlink_msg_lawn_enter_bootloader_decode(m, &b);

        const safety::Status st = safety::status();
        const bootloader::Verdict v = bootloader::judge(b.magic, st.armed);

        if (v == bootloader::Verdict::accept)
        {
            // DISARM as part of accepting, before latching the request.
            //
            // Checking `armed` here and rebooting later is a time-of-check to
            // time-of-use gap, and it is not a narrow one by construction:
            // rx_poll() drains the whole UART FIFO in a single pass, so a
            // LAWN_ARM_CMD arriving in the same batch is dispatched microseconds
            // after this, and the reset is not taken until the TX task has
            // drained the ring — a wait tx_quiesce() is explicitly allowed to
            // retry. The stated invariant is "refused unless disarmed"; without
            // this the code only delivers "was disarmed a moment ago".
            //
            // Disarming makes the precondition unfalsifiable rather than merely
            // observed: a later LAWN_ARM_CMD can still re-arm, which is why
            // enter() re-checks and abandons. Either half alone leaves the hole.
            safety::on_stop_request();

            // Latch only. The reset is taken by the TX task once the ring has
            // drained — see link.hpp's ownership rule and bootloader.hpp.
            bootloader::request();
            log_console::write("[link] bootloader request accepted; disarmed\r\n");
        }
        else
        {
            // A refusal must reach the Pi, or it cannot tell "refused" from
            // "command lost" — and this firmware's console has no listener
            // attached in normal operation, so reporting it there only is the
            // same as not reporting it.
            //
            // LAWN_FAULT_EVENT rather than a new channel: it is already the
            // repo's asynchronous, event-shaped, Pi-visible report, it already
            // carries `latched` so a non-latching event needs no new field, and
            // giving the Pi a second place to look for "something was refused"
            // is how one of them stops being read. The code is NOT added to
            // safety::Fault, because that enum is the set of LATCHED faults and
            // this must never become one.
            //
            // Which refusal it was is only on the console. The Pi can tell the
            // two apart from context: it knows whether it is armed, at 20 Hz,
            // from LAWN_NAV_STATUS.
            //
            // NOT rate-limited, and that is a deliberate trade rather than an
            // oversight. A peer spamming bad-magic requests gets 23 bytes out
            // per 16 bytes in, so it can crowd the TX ring and cost heartbeats.
            // The alternative — suppressing repeats inside a window — can eat
            // the one refusal the Pi was waiting for, which is the failure this
            // report exists to prevent. A flood is already visible as
            // dropped_tx in LAWN_LINK_STATS, and a peer able to saturate the
            // inbound link is a larger problem than the amplification.
            send_fault_event(LAWN_FAULT_BOOTLOADER_REFUSED, static_cast<uint8_t>(st.state), false);
            log_console::write("[link] bootloader request REFUSED (%s)\r\n",
                               v == bootloader::Verdict::armed ? "armed" : "bad magic");
        }
        break;
    }

    case MAVLINK_MSG_ID_LAWN_TIMESYNC:
    {
        mavlink_lawn_timesync_t ts;
        mavlink_msg_lawn_timesync_decode(m, &ts);
        if (ts.t2_us == 0) // a request, not somebody else's response
        {
            // A t2 we cannot vouch for is worse than no answer: IF-0001 §7.4
            // fits offset and skew by regression, so one bad sample biases the
            // model rather than being averaged out, and nothing downstream can
            // tell it happened. Decline; the peer retries.
            if (!rx_frame_start_valid)
            {
                sync_declined++;
                break;
            }

            // Hand it to the TX task. Answering inline would block the only
            // reader of the RX FIFO for longer than the FIFO holds.
            taskENTER_CRITICAL();
            pending_sync.t1 = ts.t1_us;
            pending_sync.t2 = rx_frame_start_us;
            pending_sync.seq = ts.exchange_seq;
            pending_sync.armed = true;
            taskEXIT_CRITICAL();
        }
        break;
    }

    default: break;
    }
}

} // namespace

bool init()
{
    const uint achieved = uart_init(board::cmd_uart(), board::cmd_baud);

    gpio_set_function(board::cmd_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(board::cmd_rx_pin, GPIO_FUNC_UART);

    // TP-0001 D6 / erratum RP2350-E9: pads reset pulled-down with the input
    // buffer enabled, and on A2 silicon leakage holds a floating pad near
    // 2.2 V — on the VIH boundary. Without this an absent Pi produces plausible
    // garbage rather than clean silence, and SAF-3's heartbeat would be
    // discriminating against corruption instead of absence.
    gpio_pull_up(board::cmd_rx_pin);

    uart_set_hw_flow(board::cmd_uart(), false, false); // TP-0001 D3
    uart_set_format(board::cmd_uart(), 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(board::cmd_uart(), true);

    // RX interrupt. The FIFO stays enabled — it is what absorbs the burst
    // between interrupts — but the trigger drops to 1/8 (4 of 32 bytes) so a
    // frame is observed early rather than at half-full, and RTIM is enabled so
    // a burst that stops below the trigger is not stranded there.
    //
    // Only RX is enabled. TX remains task-driven: link.hpp's ownership rule
    // makes the TX task the sole writer, and an ISR that also wrote would
    // break it.
    hw_write_masked(&uart_get_hw(board::cmd_uart())->ifls,
                    0u << UART_UARTIFLS_RXIFLSEL_LSB,
                    UART_UARTIFLS_RXIFLSEL_BITS);
    uart_get_hw(board::cmd_uart())->imsc =
        UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;

    // Arms on the calling core's NVIC — core 0, since this runs from main()
    // before the scheduler starts. See the ring's comment on why that matters.
    const uint irq = UART_IRQ_NUM(board::cmd_uart());
    irq_set_exclusive_handler(irq, on_cmd_uart_rx);
    irq_set_enabled(irq, true);

    // TP-0001 D2: uart_set_baudrate clamps and reports nothing, so an
    // over-request silently becomes clk_peri/16. UART framing tolerates roughly
    // 2% total ACROSS BOTH ENDS, so spending it all here would leave the Pi
    // nothing; D2's table shows 1 Mbaud is exact on both ends, making 1% free.
    const int32_t err_ppm = static_cast<int32_t>(
        (static_cast<int64_t>(achieved) - board::cmd_baud) * 1000000 / board::cmd_baud);
    log_console::write("[link] baud requested %u achieved %u (%ld ppm)\r\n",
                       (unsigned)board::cmd_baud, (unsigned)achieved, (long)err_ppm);

    return err_ppm > -10000 && err_ppm < 10000;
}

void rx_poll()
{
    // Consumes the ring the UART ISR fills. Does NOT touch the UART itself:
    // two readers of one FIFO would race, and the ISR is the reader.
    while (rx_tail != rx_head)
    {
        const uint32_t pos = rx_tail;
        const uint8_t c = rx_buf[pos];

        if (rx_status.parse_state == MAVLINK_PARSE_STATE_IDLE)
        {
            // This byte opens a frame. It carries a trustworthy arrival time
            // only if it is the byte the ISR actually stamped.
            if (rx_burst_stamped && rx_burst_pos == pos)
            {
                rx_frame_start_us = rx_burst_us;
                rx_frame_start_valid = true;
                rx_burst_stamped = false; // consumed; the next burst re-arms it
            }
            else
            {
                // A frame began without the ring draining first, so the stamp
                // on record belongs to an earlier byte. Say so rather than
                // answer a sync exchange from it.
                rx_frame_start_valid = false;
            }
        }

        rx_tail = (pos + 1) & rx_ring_mask;

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_msg, &rx_status))
        {
            dispatch(&rx_msg);
        }

        // packet_rx_drop_count is NOT cumulative. mavlink_frame_char_buffer
        // assigns it from status->parse_error and then zeroes parse_error on
        // every byte, so it is a per-byte 0/1 indicator (see
        // mavlink/c/mavlink_helpers.h:859,862 — the cumulative increment at
        // :841 is commented out upstream). Sampling it once after draining the
        // FIFO both misses almost every error and, on the poll after one that
        // landed on the last byte, computes 0 - 1 = 65535 rejected frames.
        // Accumulate per byte instead.
        if (rx_status.packet_rx_drop_count != 0)
        {
            safety::on_frame_rejected(rx_status.packet_rx_drop_count);
        }
    }
}

void service_tx()
{
    // Drain first: a deferred TIMESYNC must go out of an empty ring so that t3
    // is not offset by whatever was queued ahead of it.
    while (true)
    {
        uint8_t c;
        taskENTER_CRITICAL();
        if (tx_used_unsafe() == 0 || !uart_is_writable(board::cmd_uart()))
        {
            taskEXIT_CRITICAL();
            break; // never spin: a far end that stopped reading costs drops
        }
        c = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) & ring_mask;
        // The write stays inside the section. Releasing between consuming the
        // byte and writing it lets a preemption reorder it after a later frame.
        uart_get_hw(board::cmd_uart())->dr = c;
        taskEXIT_CRITICAL();
    }

    PendingSync p{};
    taskENTER_CRITICAL();
    const bool have = pending_sync.armed;
    if (have)
    {
        p = pending_sync;
        pending_sync.armed = false;
    }
    const bool ring_empty = (tx_used_unsafe() == 0);
    taskEXIT_CRITICAL();

    if (have && ring_empty)
    {
        emit_timesync(p);
    }
    else if (have)
    {
        // Ring still backed up; re-arm and answer on the next pass rather than
        // emit a sample whose t3 is offset by the backlog.
        taskENTER_CRITICAL();
        if (!pending_sync.armed)
        {
            pending_sync = p;
            pending_sync.armed = true;
        }
        taskEXIT_CRITICAL();
    }
}

bool tx_quiesce()
{
    taskENTER_CRITICAL();
    const bool empty = (tx_used_unsafe() == 0);
    taskEXIT_CRITICAL();

    if (!empty)
    {
        return false;
    }

    // The ring being empty is not enough: up to a FIFO's worth of bytes plus
    // the byte in the shift register are still being clocked out. At 1 Mbaud
    // that is tens of microseconds, so this blocks only briefly — and it is
    // only ever reached on a path that is about to stop executing anyway.
    uart_tx_wait_blocking(board::cmd_uart());
    return true;
}

bool send_heartbeat()
{
    const safety::Status st = safety::status();
    const uint32_t sess = safety::session_id();
    return pack_and_push(
        [&](mavlink_message_t* m)
        {
            mavlink_msg_heartbeat_pack(
                sysid, compid_self, m, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                st.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, sess,
                (st.fault != safety::Fault::none) ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE);
        });
}

bool send_nav_status()
{
    const safety::Status st = safety::status();
    const uint64_t now = time_us_64();
    const uint16_t age = (st.cmd_age_ms == 0xFFFFFFFFu)
                             ? 65535
                             : static_cast<uint16_t>(st.cmd_age_ms > 65534 ? 65534 : st.cmd_age_ms);

    return pack_and_push(
        [&](mavlink_message_t* m)
        {
            mavlink_msg_lawn_nav_status_pack(sysid, compid_self, m, now, age,
                                             static_cast<uint8_t>(st.state), st.armed ? 1 : 0,
                                             static_cast<uint8_t>(st.fault));
        });
}

bool send_wheel_state(int16_t left_drpm, int16_t right_drpm, int16_t left_cmd, int16_t right_cmd,
                      bool left_valid, bool right_valid)
{
    uint8_t flags = LAWN_WHEEL_DIR_UNMEASURED; // ADR-0002: always set today
    if (left_valid)
    {
        flags |= LAWN_WHEEL_LEFT_VALID;
    }
    if (right_valid)
    {
        flags |= LAWN_WHEEL_RIGHT_VALID;
    }
    const uint64_t now = time_us_64();

    return pack_and_push(
        [&](mavlink_message_t* m)
        {
            mavlink_msg_lawn_wheel_state_pack(sysid, compid_self, m, now, left_drpm, right_drpm,
                                              left_cmd, right_cmd, flags);
        });
}

bool send_link_stats()
{
    const safety::Status st = safety::take_window();
    const uint64_t now = time_us_64();
    const uint32_t total = st.win_ok + st.win_bad;
    const uint8_t quality = total ? static_cast<uint8_t>((st.win_ok * 100u) / total) : 0u;
    const uint32_t dropped = tx_drop + log_console::dropped();

    return pack_and_push(
        [&](mavlink_message_t* m)
        {
            mavlink_msg_lawn_link_stats_pack(
                sysid, compid_self, m, now, st.win_ok, st.win_bad, dropped,
                static_cast<uint16_t>(st.window_ms > 65535 ? 65535 : st.window_ms),
                static_cast<uint16_t>(st.win_hb_missed), quality);
        });
}

bool send_fault_event(uint8_t code, uint8_t nav_state, bool latched)
{
    const uint64_t now = time_us_64();
    return pack_and_push(
        [&](mavlink_message_t* m)
        {
            mavlink_msg_lawn_fault_event_pack(sysid, compid_self, m, now, code, nav_state,
                                              latched ? 1 : 0);
        });
}

uint32_t tx_dropped()
{
    return tx_drop;
}

uint32_t rx_overrun_bytes()
{
    return rx_overrun;
}

uint32_t sync_declined_count()
{
    return sync_declined;
}

uint32_t tx_peak_bytes()
{
    return tx_peak;
}

} // namespace link
