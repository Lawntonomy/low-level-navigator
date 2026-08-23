#pragma once

// MAVLink transport for the command link (IF-0001).
//
// Deliberately does not expose MAVLink types: the control path should not have
// to include generated headers to know what it was told. Everything the rest of
// the firmware cares about is handed to safety.hpp on receipt.
//
// Never blocks in either direction. TX goes through a drop-on-full ring; a
// frame is pushed whole or dropped whole, because a partial frame desyncs the
// far end and that is worse than a gap the sequence number will reveal.

#include <cstdint>

namespace link
{

// Configures uart0, pulls up RX, and verifies the achieved baud rate.
// Returns false if the rate is out of tolerance — see TP-0001 D2: the SDK
// clamps silently, so an over-request becomes 3 Mbaud with no error.
bool init();

// Reads and parses everything currently in the UART FIFO, dispatching to
// safety.hpp. Called by the link RX task.
void rx_poll(uint64_t now_us);

// Moves queued bytes to the UART, as many as the FIFO accepts. Called by the
// link TX task.
void tx_drain();

// Periodic messages. Each returns false if the frame was dropped.
bool send_heartbeat();
bool send_nav_status(uint64_t now_us);
bool send_wheel_state(uint64_t now_us, int16_t left_drpm, int16_t right_drpm,
                      int16_t left_cmd, int16_t right_cmd, bool left_valid,
                      bool right_valid);
bool send_link_stats(uint64_t now_us);
bool send_fault_event(uint64_t now_us, uint8_t code, uint8_t nav_state,
                      bool latched);

// Frames dropped because the TX ring was full, since boot.
uint32_t tx_dropped();
uint32_t tx_peak_bytes();

} // namespace link
