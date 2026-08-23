#pragma once

// Board-level assignments the application layer needs. Motor, encoder, and
// NeoPixel pins stay in hardware_drivers/gpio_defines.h; this holds the link
// and instrumentation pins so the two are not tangled.
//
// Assignments and their reasoning: TP-0001 D1.

#include "hardware/uart.h"
#include "pico/stdlib.h"

namespace board
{

// Command link — the authoritative control path (ADR-0003).
//
// GPIO 16/17, not 0/1. TP-0001 D1 originally put this on 0/1 to dodge the OLD
// firmware's stdio UART, which lived on 16/17 — but the physical wire was
// never moved when that decision was made, and hardware bring-up on the link
// stub confirmed it: zero bytes at GPIO 0/1, full clean traffic the moment the
// firmware's pin assignment was corrected to match the existing wire. GPIO
// 16/17 is a valid alternate mapping for the same uart0 peripheral as 0/1
// (datasheet Table 3), and the conflict this was dodging is moot now that
// stdio is disabled outright (TP-0001 D5) — so there is no remaining reason
// to prefer 0/1, and every reason to match the wire that is actually there.
//
// Functions, not constexpr: the SDK's uart0/uart1 expand to a reinterpret_cast
// of a fixed address, which C++ will not accept in a constant expression.
inline uart_inst_t* cmd_uart()
{
    return uart0;
}
constexpr uint cmd_tx_pin = 16;
constexpr uint cmd_rx_pin = 17;
constexpr uint cmd_baud = 1000000; // TP-0001 D2: exact on both ends at 48 MHz

// Console — unframed, readable from the first instruction, independent of the
// Pi so it survives the Pi rebooting or being unplugged.
inline uart_inst_t* console_uart()
{
    return uart1;
}
constexpr uint console_tx_pin = 20; // only free UART1 pair on the Pico 2 header
constexpr uint console_baud = 115200;

// Scope pin. Toggled once per control-loop iteration: TP-0001 Phase 2 needs
// ground truth for loop period that does not depend on firmware
// instrumentation which could itself be the thing that stalled.
constexpr uint scope_pin = 22;

} // namespace board
