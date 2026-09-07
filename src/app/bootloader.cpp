#include "app/bootloader.hpp"

#include "hardware/structs/watchdog.h"
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
    // Drive off first and explicitly, before anything that reboots.
    motors::safe_state();

    log_console::write_blocking("[boot] BOOTSEL requested; rebooting to take it\r\n");

    // Hand the request to the next boot rather than taking it here. See
    // enter_if_requested() in the header for why: the ROM call hangs when made
    // from a task context and works from early main().
    //
    // Scratch registers survive a watchdog reset -- that is what they are for --
    // so this is the channel between the two contexts.
    watchdog_hw->scratch[scratch_index] = deferred_magic;

    watchdog_reboot(0, 0, 0);
    for (;;)
    {
        tight_loop_contents();
    }
}

void enter_if_requested()
{
    if (watchdog_hw->scratch[scratch_index] != deferred_magic)
    {
        return;
    }

    // Clear BEFORE handing over. If the ROM refuses, the next boot must come up
    // as a normal application rather than looping back here forever.
    watchdog_hw->scratch[scratch_index] = 0;

    rom_reset_usb_boot(0, interface_mask);

    // Only reached if the ROM refused. Fall through and boot normally: the
    // request is already cleared, the machine comes up safe, and the Pi sees
    // the link return instead of silence it cannot tell from dead hardware.
}

} // namespace bootloader
