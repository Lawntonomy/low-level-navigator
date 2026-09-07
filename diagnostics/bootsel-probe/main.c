// bootsel-probe: does rom_reset_usb_boot() actually reach BOOTSEL on this board?
//
// Isolation test. The link-commanded reboot in src/app/bootloader.cpp fails on
// hardware: the command is received, the link goes silent, and no PICOBOOT
// device ever enumerates. Two guesses at the cause (the watchdog, then the
// interface mask) were both wrong, so this strips away everything that is not
// the ROM call itself.
//
// No FreeRTOS. No MAVLink. No watchdog. No motors. Just: come up, say so, wait,
// call rom_reset_usb_boot(0, 1), and let the Pi watch for 2e8a:000f.
//
//   BOOTSEL appears  -> the ROM call is fine, and the fault is in the context
//                       around it: FreeRTOS privilege, task state, drain
//                       ordering, or something else in the firmware.
//   BOOTSEL does not -> the ROM call does not do what we think on this board,
//                       and the feature needs rethinking rather than debugging.
//
// USB stdio is on, so this stays recoverable with `picotool load -f` right up
// until the moment it calls the reset.

#include <stdio.h>
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

// The mask is the variable under test.
//
// src/app/bootloader.cpp passes 1 (BOOTSEL_FLAG_DISABLE_MSD_INTERFACE) to
// suppress the mass-storage drive and keep PICOBOOT, which is the interface
// picotool uses. That is what fails on hardware.
//
// pico-examples/flash/nuke/nuke.c -- the canonical use of this call -- passes
// **0**, enabling both interfaces. That is the single parameter we chose
// differently from the known-good example, so it is what this probe tests
// first. If 0 enumerates and 1 does not, the mask is the bug and the firmware
// should take a spurious mass-storage drive over not working.
#define INTERFACE_MASK 0u

// Busy work for core 1. The point is only that the second core is EXECUTING
// when the ROM call happens, which is what the firmware does and what the
// first version of this probe did not.
static void core1_spin(void)
{
    volatile uint32_t x = 0;
    for (;;)
    {
        x++;
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(3000); // let the host enumerate and open the port

    // THE VARIABLE UNDER TEST.
    //
    // rom_reset_usb_boot(0, 0) reached BOOTSEL from a bare single-core program.
    // The real firmware, with the same mask, hangs inside the ROM call instead:
    // enter() is reached (the link goes silent) and the call neither reboots
    // nor returns (the refusal path never fires). The firmware runs SMP
    // FreeRTOS with configNUMBER_OF_CORES 2 and the control task pinned to
    // core 1, so a running second core is the largest structural difference
    // between the two.
    multicore_launch_core1(core1_spin);
    sleep_ms(100);
    printf("[bootsel-probe] core1 launched and spinning\n");

    for (int i = 10; i > 0; i--)
    {
        printf("[bootsel-probe] alive, calling rom_reset_usb_boot(0, %u) in %d s\n", INTERFACE_MASK,
               i);
        sleep_ms(1000);
    }

    printf("[bootsel-probe] calling now; this is the last line you will see\n");
    sleep_ms(200); // let the USB write drain before the world ends

    rom_reset_usb_boot(0, INTERFACE_MASK);
}
