#include "app/bootloader.hpp"

#include "hardware/watchdog.h"
#include "pico/bootrom.h"

#include "app/log.hpp"
#include "app/motors.hpp"

namespace bootloader
{
namespace
{

// Set by the link RX task, read by the link TX task, once per TX pass forever.
//
// Deliberately a plain volatile rather than a critical section, matching how
// link.cpp already publishes tx_drop and tx_peak. This is a ONE-WAY latch on a
// single naturally-aligned word with no companion data: there is no second
// variable a reader could observe out of order, and no value it can take that
// is neither the old one nor the new one. What volatile buys is that the
// compiler cannot hoist the load out of the TX task's loop; nothing more is
// needed. rt.h notes that every SMP critical section briefly contends the same
// spinlock pair the control task needs, and taking one at 1 kHz for the whole
// life of the machine, to guard a flag that changes at most once per boot,
// would be paying that cost for nothing.
volatile bool requested;

} // namespace

void request()
{
    requested = true;
}

bool pending()
{
    return requested;
}

void enter()
{
    // Drive off first, and explicitly. The pad reset that follows does put the
    // TB6612 inputs low on its own, but a function whose safety depends on a
    // side effect of the next instruction is one refactor away from not having
    // it. See judge()'s note on issue #26 for what the reset state actually is.
    motors::safe_state();

    // No acknowledgement is sent, deliberately. Anything packed here would race
    // a reset that does not return, so it would be a frame the Pi sometimes
    // sees and sometimes does not — worse than none. The PICOBOOT device
    // enumerating on the Pi is the acknowledgement, and it is unambiguous.
    //
    // write_blocking, not write: the logger task will never run again, so a
    // line left in the ring would be lost. This clocks the bytes out of uart1
    // before returning — about 3.5 ms at 115200, spent by a task that is about
    // to stop existing, on the one channel a bench operator can actually see.
    log_console::write_blocking("[boot] entering BOOTSEL on link request\r\n");

    // Disarm the watchdog BEFORE handing over to the bootrom. This is not
    // belt-and-braces; without it the feature does not work at all, and the
    // failure is indistinguishable from a dead board.
    //
    // Measured on the bench 2026-08-25: the link went silent and no PICOBOOT
    // device ever enumerated. rom_reset_usb_boot() does not touch the watchdog
    // -- on RP2350 it routes through rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL,
    // 10, ...), scheduling the reboot 10 ms out -- and rt::watchdog_timeout_ms
    // is 100 ms with PSM_WDSEL set to reset RESETS. Bootrom USB enumeration
    // takes far longer than 100 ms, so the watchdog fires part-way through,
    // resets, and the cycle repeats: never enumerating, never running the
    // application. Recovery is a physical BOOTSEL press.
    //
    // main.cpp's halt() disables it for an adjacent reason -- there, so a
    // deliberate stop stays stopped instead of rebooting at 10 Hz.
    watchdog_disable();

    rom_reset_usb_boot(0, disable_msc_keep_picoboot);
}

} // namespace bootloader
