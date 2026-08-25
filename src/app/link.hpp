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
//
// **Ownership rule: only the link TX task writes to the UART.** rx_poll() may
// enqueue but must never transmit. An earlier version answered TIMESYNC inline
// from the RX path, which stalled the only reader of a 32-byte RX FIFO for
// longer than 320 us — the time that FIFO takes to overflow at 1 Mbaud — and
// could interleave a byte from a half-completed drain into the middle of a
// frame.

#include <cstdint>

namespace link
{

// Configures uart0, pulls up RX, and verifies the achieved baud rate.
// Returns false if the rate is out of tolerance — see TP-0001 D2: the SDK
// clamps silently, so an over-request becomes clk_peri/16 with no error.
bool init();

// Reads and parses everything currently in the UART FIFO, dispatching to
// safety.hpp. Called by the link RX task. Enqueues only; never transmits.
void rx_poll();

// Moves queued bytes to the UART and emits any deferred TIMESYNC response.
// Called by the link TX task, which is the sole writer.
void service_tx();

// True once the TX ring is empty AND the UART shift register has drained, so
// nothing this module was asked to send is still in flight.
//
// TX-task only, and only immediately after service_tx(): it waits on the UART,
// which no other task may touch. Exists for the link-commanded BOOTSEL reboot
// (bootloader.hpp), where the reset does not return and a frame cut in half is
// worse for the far end than a frame never sent. Returns false rather than
// spinning when the ring still holds bytes — the caller retries next pass.
bool tx_quiesce();

// Periodic messages. Each returns false if the frame was dropped.
bool send_heartbeat();
bool send_nav_status();
bool send_wheel_state(int16_t left_drpm, int16_t right_drpm, int16_t left_cmd, int16_t right_cmd,
                      bool left_valid, bool right_valid);
bool send_link_stats();
bool send_fault_event(uint8_t code, uint8_t nav_state, bool latched);

// Frames dropped because the TX ring was full, since boot.
uint32_t tx_dropped();
uint32_t tx_peak_bytes();

} // namespace link
