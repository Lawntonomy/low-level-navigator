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
// 0 = expose both USB interfaces, matching pico-examples/flash/nuke/nuke.c.
//
// NOT 1 (BOOTSEL_FLAG_DISABLE_MSD_INTERFACE), which reads tidier: it would
// suppress the mass-storage drive and keep only PICOBOOT, the interface
// picotool actually uses. That is what the first version passed, and the ROM
// refused the reboot -- no enumeration, no reset, and the calling task dead.
// Every working example passes 0. A spurious drive appearing on the Pi is
// cosmetic; not working is not.
inline constexpr uint32_t interface_mask = 0u;

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
// Written to a watchdog scratch register to survive the reset, then read back
// in early main(). Scratch 4 is spoken for by the SDK's watchdog_enable /
// watchdog_enable_caused_reboot pair, so this uses 3.
inline constexpr uint32_t deferred_magic = 0xB0075E1u;
inline constexpr unsigned scratch_index = 3;

// Called from early main(), BEFORE the scheduler, motors_init(), or anything
// that enables an interrupt. Enters BOOTSEL if the previous boot asked for it,
// and returns immediately otherwise.
//
// The ROM call has to happen HERE rather than where the request arrives.
// Measured on the bench 2026-08-25: rom_reset_usb_boot() reaches BOOTSEL from a
// bare program -- including with a second core spinning -- but hangs inside the
// ROM when called from a FreeRTOS task, with DMA and PIO quiesced and the
// watchdog disabled. It neither reboots nor returns. Deferring to early main()
// reproduces the context that works instead of arguing with the one that does
// not.
void enter_if_requested();

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
