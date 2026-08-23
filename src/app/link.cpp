#include "app/link.hpp"

#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "task.h"

#include "app/board.h"
#include "app/log.hpp"
#include "app/safety.hpp"

#include "lawntonomy/mavlink.h"

namespace link
{
namespace
{

constexpr uint8_t sysid = 1;
constexpr uint8_t compid_self = MAV_COMP_ID_AUTOPILOT1;      // 1
constexpr uint8_t compid_peer = MAV_COMP_ID_ONBOARD_COMPUTER; // 191

constexpr uint32_t ring_bytes = 2048; // power of two
constexpr uint32_t ring_mask = ring_bytes - 1;
static_assert((ring_bytes & ring_mask) == 0, "ring must be a power of two");

uint8_t tx_buf[ring_bytes];
volatile uint32_t tx_head;
volatile uint32_t tx_tail;
volatile uint32_t tx_drop;
volatile uint32_t tx_peak;

// Parser state. Touched only by the link RX task, so it needs no lock — but it
// must stay that way; nothing else may call rx_poll().
mavlink_message_t rx_msg;
mavlink_status_t rx_status;
uint16_t last_parse_drops;

// Timestamp of the byte that opened the frame being parsed.
//
// IF-0001 §7.4 requires t2 at the start-bit edge, captured in hardware. This
// is the FIFO-read time of the first byte instead: good to roughly the FIFO
// trigger jitter (~25 us), not the ~2 us the pin-edge path gives. TP-0001 T4.1
// specifies the PIO capture that replaces it. Everything the clock model
// reports is bounded by this approximation.
uint64_t rx_frame_start_us;

inline uint32_t tx_used_unsafe()
{
    return (tx_head - tx_tail) & ring_mask;
}

bool push(const uint8_t *p, uint32_t n)
{
    taskENTER_CRITICAL();
    if (n > ring_mask - tx_used_unsafe())
    {
        tx_drop++;
        taskEXIT_CRITICAL();
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
    taskEXIT_CRITICAL();
    return true;
}

bool send(mavlink_message_t *m)
{
    uint8_t scratch[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(scratch, m);
    return push(scratch, n);
}

// A TIMESYNC response is the one message whose timestamp is taken at transmit
// rather than at measurement — for a sync frame, transmit time IS the
// measurement (IF-0001 §7.4, a deliberate exception to §7 item 1).
//
// Without hardware capture the closest achievable is: drain everything, stamp,
// then write. That blocks for roughly one frame time (~370 us at 1 Mbaud) and
// is the only blocking write in this file. It is confined to a 5 Hz message
// and exists so the clock model can be measured at all. Replace with PIO edge
// capture before this matters (TP-0001 T4.1).
void send_timesync_response(uint64_t t1, uint64_t t2, uint8_t seq)
{
    while (true)
    {
        taskENTER_CRITICAL();
        const bool empty = (tx_used_unsafe() == 0);
        taskEXIT_CRITICAL();
        if (empty)
        {
            break;
        }
        tx_drain();
    }
    uart_tx_wait_blocking(board::cmd_uart());

    mavlink_message_t m;
    mavlink_msg_lawn_timesync_pack(sysid, compid_self, &m, t1, t2,
                                   time_us_64(), seq);
    uint8_t scratch[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(scratch, &m);
    uart_write_blocking(board::cmd_uart(), scratch, n);
}

void dispatch(const mavlink_message_t *m, uint64_t now_us)
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
        if (safety::on_heartbeat(now_us, hb.custom_mode))
        {
            log_console::write("[link] peer restarted -> disarm\r\n");
        }
        break;
    }

    case MAVLINK_MSG_ID_LAWN_DRIVE_CMD:
    {
        mavlink_lawn_drive_cmd_t c;
        mavlink_msg_lawn_drive_cmd_decode(m, &c);
        safety::on_drive_request(c.left_drpm, c.right_drpm, now_us);
        break;
    }

    case MAVLINK_MSG_ID_LAWN_ARM_CMD:
    {
        mavlink_lawn_arm_cmd_t a;
        mavlink_msg_lawn_arm_cmd_decode(m, &a);
        safety::on_arm_request(a.arm != 0, a.magic, now_us);
        log_console::write("[link] arm request: arm=%u magic=%04x\r\n",
                           (unsigned)a.arm, (unsigned)a.magic);
        break;
    }

    case MAVLINK_MSG_ID_LAWN_STOP_REQ:
        safety::on_stop_request(now_us);
        log_console::write("[link] stop requested\r\n");
        break;

    case MAVLINK_MSG_ID_LAWN_TIMESYNC:
    {
        mavlink_lawn_timesync_t ts;
        mavlink_msg_lawn_timesync_decode(m, &ts);
        if (ts.t2_us == 0) // a request, not somebody else's response
        {
            send_timesync_response(ts.t1_us, rx_frame_start_us,
                                   ts.exchange_seq);
        }
        break;
    }

    default:
        break;
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
    // 2.2 V — on the VIH boundary. Without this an absent Pi produces
    // plausible garbage rather than clean silence, and SAF-3's heartbeat would
    // be discriminating against corruption instead of absence.
    gpio_pull_up(board::cmd_rx_pin);

    uart_set_hw_flow(board::cmd_uart(), false, false); // TP-0001 D3
    uart_set_format(board::cmd_uart(), 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(board::cmd_uart(), true);

    // TP-0001 D2: uart_set_baudrate clamps and reports nothing, so an
    // over-request silently becomes clk_peri/16. UART framing tolerates about
    // 2% total across both ends.
    const int32_t err_ppm = static_cast<int32_t>(
        (static_cast<int64_t>(achieved) - board::cmd_baud) * 1000000 /
        board::cmd_baud);
    log_console::write("[link] baud requested %u achieved %u (%ld ppm)\r\n",
                       (unsigned)board::cmd_baud, (unsigned)achieved,
                       (long)err_ppm);

    return err_ppm > -20000 && err_ppm < 20000;
}

void rx_poll(uint64_t now_us)
{
    while (uart_is_readable(board::cmd_uart()))
    {
        const uint8_t c = static_cast<uint8_t>(uart_get_hw(board::cmd_uart())->dr);

        if (rx_status.parse_state == MAVLINK_PARSE_STATE_IDLE)
        {
            rx_frame_start_us = time_us_64();
        }

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &rx_msg, &rx_status))
        {
            dispatch(&rx_msg, now_us);
        }
    }

    // Frames the parser rejected outright: bad CRC, bad CRC_EXTRA, bad length.
    // IF-0001 §7.3 wants these visible in the window before they become a stop.
    if (rx_status.packet_rx_drop_count != last_parse_drops)
    {
        const uint16_t d =
            static_cast<uint16_t>(rx_status.packet_rx_drop_count - last_parse_drops);
        safety::on_frame_rejected(d);
        last_parse_drops = rx_status.packet_rx_drop_count;
    }
}

void tx_drain()
{
    while (true)
    {
        uint8_t c;
        taskENTER_CRITICAL();
        if (tx_used_unsafe() == 0)
        {
            taskEXIT_CRITICAL();
            return;
        }
        if (!uart_is_writable(board::cmd_uart()))
        {
            taskEXIT_CRITICAL();
            return; // never spin: a far end that stopped reading costs drops
        }
        c = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) & ring_mask;
        taskEXIT_CRITICAL();

        uart_get_hw(board::cmd_uart())->dr = c;
    }
}

bool send_heartbeat()
{
    const safety::Status st = safety::status(time_us_64());
    mavlink_message_t m;
    mavlink_msg_heartbeat_pack(
        sysid, compid_self, &m, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
        st.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0,
        safety::session_id(), // IF-0001 §7.5
        (st.fault != safety::Fault::none) ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE);
    return send(&m);
}

bool send_nav_status(uint64_t now_us)
{
    const safety::Status st = safety::status(now_us);
    const uint16_t age =
        (st.cmd_age_ms == 0xFFFFFFFFu)
            ? 65535
            : static_cast<uint16_t>(st.cmd_age_ms > 65534 ? 65534 : st.cmd_age_ms);

    mavlink_message_t m;
    mavlink_msg_lawn_nav_status_pack(sysid, compid_self, &m, now_us, age,
                                     static_cast<uint8_t>(st.state),
                                     st.armed ? 1 : 0,
                                     static_cast<uint8_t>(st.fault));
    return send(&m);
}

bool send_wheel_state(uint64_t now_us, int16_t left_drpm, int16_t right_drpm,
                      int16_t left_cmd, int16_t right_cmd, bool left_valid,
                      bool right_valid)
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

    mavlink_message_t m;
    mavlink_msg_lawn_wheel_state_pack(sysid, compid_self, &m, now_us, left_drpm,
                                      right_drpm, left_cmd, right_cmd, flags);
    return send(&m);
}

bool send_link_stats(uint64_t now_us)
{
    const safety::Status st = safety::take_window(now_us);
    const uint32_t total = st.win_ok + st.win_bad;
    const uint8_t quality =
        total ? static_cast<uint8_t>((st.win_ok * 100u) / total) : 0u;

    mavlink_message_t m;
    mavlink_msg_lawn_link_stats_pack(
        sysid, compid_self, &m, now_us, st.win_ok, st.win_bad,
        tx_drop + log_console::dropped(),
        static_cast<uint16_t>((now_us - st.win_start_us) / 1000),
        static_cast<uint16_t>(st.win_hb_missed), quality);
    return send(&m);
}

bool send_fault_event(uint64_t now_us, uint8_t code, uint8_t nav_state,
                      bool latched)
{
    mavlink_message_t m;
    mavlink_msg_lawn_fault_event_pack(sysid, compid_self, &m, now_us, code,
                                      nav_state, latched ? 1 : 0);
    return send(&m);
}

uint32_t tx_dropped()
{
    return tx_drop;
}

uint32_t tx_peak_bytes()
{
    return tx_peak;
}

} // namespace link
