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
// Moved 20 -> 8 on 2026-09-08. GP20 is I2C0 SDA as well as UART1 TX, and the
// LSM6DSOX is soldered to GP20/21, so the console had to give the pin up.
//
// UART1 TX exists only on GP4, GP8, GP20 and GP24; GP24 is not brought out on
// the Pico 2 header, and GP4/GP5 are the right-motor direction pins. GP8 was
// left_backward_pin, which is a plain GPIO output to the TB6612 with no
// special-function requirement, so it moved to GP9 (free, and the adjacent
// header pin) and the console took GP8. That keeps the console on a HARDWARE
// UART, which matters: IF-0001 §2 keeps it for being readable before any stack
// is initialised, during a panic, before USB enumerates. A PIO console would
// have coupled the debug channel to the same block that drives PWM and the
// encoders — least trustworthy exactly when debugging a PIO fault.
constexpr uint console_tx_pin = 8;
constexpr uint console_baud = 115200;

// Scope pin. Toggled once per control-loop iteration: TP-0001 Phase 2 needs
// ground truth for loop period that does not depend on firmware
// instrumentation which could itself be the thing that stalled.
constexpr uint scope_pin = 22;

} // namespace board
