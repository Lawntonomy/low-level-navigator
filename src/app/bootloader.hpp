#pragma once

// Link-commanded reboot into the RP2350 bootrom's BOOTSEL mode.
//
// Exists so the main firmware can be reflashed from the Pi without a physical
// button press AND without enabling USB stdio. TP-0001 D5 disables both stdio
// backends because stdio_usb_out_chars() blocks for up to
// PICO_STDIO_USB_STDOUT_TIMEOUT_US once a host opens the port and stops
// reading, which is not a thing that may happen anywhere near the control
// loop. rom_reset_usb_boot() sidesteps that entirely: the ROM brings USB up
// itself *after* the reset, so the application never links a USB stack, never
// owns a blocking write path, and never exposes a permanently-resident vendor
// reset interface. D5 is unchanged by this file.
//
// Deliberately free of the Pico SDK and FreeRTOS in the header so judge() —
// the whole of the gating decision — compiles into the host test build.
// enter() is declared here but defined only in bootloader.cpp; nothing in
// test/ may call it, and nothing links it.
//
// **Ordering matters more than anything else here.** rom_reset_usb_boot() does
// not return, so every byte still in the link TX ring is lost and a reset taken
// mid-frame desynchronises the far end. The request is therefore latched on the
// RX path and consumed by the link TX task once the ring and the UART shift
// register are both empty — see link_tx_task() in main.cpp. link.hpp's rule
// that only the TX task touches the UART is why this cannot be done inline.

#include <cstdint>

namespace bootloader
{

// Distinct from safety.cpp's arm_magic (0xA57E, and only 16 bits wide) so that
// neither value can be read as the other, whatever field it lands in.
inline constexpr uint32_t request_magic = 0xB00710ADu;

// Passed to rom_reset_usb_boot()'s disable_interface_mask: 1 = disable USB
// Mass Storage, keep PICOBOOT. picotool speaks PICOBOOT, and suppressing mass
// storage stops a spurious drive appearing on the Pi every time we reflash.
inline constexpr uint32_t disable_msc_keep_picoboot = 1u;

enum class Verdict : uint8_t
{
    accept = 0,
    bad_magic = 1, // frame did not carry request_magic
    armed = 2,     // machine is armed; a reboot now is an uncommanded change
};

// The entire gating rule, as a pure function.
//
// Frame integrity is NOT checked here because it is already guaranteed: SAF-53
// discards a rejected frame whole, so dispatch() only ever sees a frame that
// passed CRC and the identity check.
//
// The arm test is not redundant with the magic test. Rebooting while armed is
// an uncommanded state transition even though the pads come out of reset with
// IN1 = IN2 = LOW, which is the TB6612's coast row and therefore torque-free.
// Note also that with R1 fitted on the breakout (issue #26) the reset state
// floats STBY HIGH, so the driver sits *enabled* for as long as the board is in
// the bootloader — harmless with both inputs low, and one more reason #26 wants
// fixing rather than a reason this gate can be relaxed.
constexpr Verdict judge(uint32_t magic, bool armed)
{
    if (magic != request_magic)
    {
        return Verdict::bad_magic;
    }
    if (armed)
    {
        return Verdict::armed;
    }
    return Verdict::accept;
}

// Latch an accepted request. Called from the link RX task, which must never
// take the reset itself — see the ordering note above.
void request();

// True once a request has been latched. Called from the link TX task. There is
// no way to un-latch: a reboot request that has been accepted is not something
// a later frame gets to withdraw.
bool pending();

// Puts the motors in their safe state and calls rom_reset_usb_boot(). Does not
// return. Only the link TX task may call this, and only once link::tx_quiesce()
// has confirmed nothing is still on its way out.
[[noreturn]] void enter();

} // namespace bootloader
