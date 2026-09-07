#include "app/bootloader.hpp"

#include "boot/picoboot_constants.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"

#include "app/log.hpp"
#include "app/motors.hpp"
#include "app/safety.hpp"

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

void clear()
{
    requested = false;
}

void enter()
{
    // Re-check the interlock in the state that is actually being acted on.
    //
    // The gate ran when the request was latched, on a different task, before
    // the ring drained. Accepting the request disarms (link.cpp), but a
    // LAWN_ARM_CMD in the same receive batch can re-arm in between — so the
    // condition is checked again here, where the consequence happens. If the
    // machine armed in the interim, abandon: clear the latch and let the
    // control loop carry on. The Pi sees no reboot and can ask again.
    if (safety::status().armed)
    {
        clear();
        log_console::write_blocking("[boot] BOOTSEL abandoned: re-armed after request\r\n");
        return;
    }

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

    // Call the bootrom API directly rather than through rom_reset_usb_boot(),
    // which is declared noreturn and followed by __builtin_unreachable(). The
    // ROM's own contract is narrower than that: datasheet §5.4.8.24 (printed
    // 397) says NO_RETURN_ON_SUCCESS "forces this method not to return **if**
    // the reboot is successfully initiated" -- on a failure it returns a
    // negative error code (§5.4.3, printed 378).
    //
    // Through the noreturn wrapper the compiler deletes everything after the
    // call, so an error return executes the literal pool: verified in the
    // disassembly, where the only `pop {r3, pc}` is the magic-mismatch path.
    // That is a hard fault with motors::init() not yet run and the TB6612
    // enabled by the breakout's STBY pull-up (issue #26) -- the worst available
    // outcome, reached by the one path that is supposed to be recoverable.
    //
    // Called here rather than where the request arrives because the ROM call
    // hangs from a FreeRTOS task and works from a bare context (measured; the
    // datasheet documents no calling-context restriction either way, so this is
    // a bench fact to design around rather than a citable rule).
    const int rc = rom_reboot(REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL | REBOOT2_FLAG_NO_RETURN_ON_SUCCESS,
                              reboot_delay_ms, interface_mask, 0);

    // Reached only on refusal. The magic is already cleared, so this returns
    // into a normal boot: the machine comes up safe and the Pi sees the link
    // return, rather than silence it cannot tell from dead hardware.
    (void)rc;
}

} // namespace bootloader
