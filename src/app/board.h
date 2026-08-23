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
// GPIO 0/1 and GPIO 16/17 are BOTH uart0 on RP2350 (datasheet Table 3), so the
// old stdio UART on 16/17 cannot coexist with this. stdio is disabled outright
// in CMakeLists.txt; see TP-0001 D5.
// Functions, not constexpr: the SDK's uart0/uart1 expand to a reinterpret_cast
// of a fixed address, which C++ will not accept in a constant expression.
inline uart_inst_t* cmd_uart()
{
    return uart0;
}
constexpr uint cmd_tx_pin = 0;
constexpr uint cmd_rx_pin = 1;
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
