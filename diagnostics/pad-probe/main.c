// pad-probe: what do the motor-driver pins sit at when nothing drives them?
//
// Standalone, bench-only, and deliberately separate from the main firmware --
// which cannot answer this question, because it disables both stdio backends
// and its console UART has no listener attached.
//
// The question it settles: the Adafruit TB6612 breakout (product 2448) is
// documented as carrying R1, a 10k pull-up from STBY to VCC. If that is fitted,
// the motor driver is ENABLED from the moment VCC rises until firmware drives
// GPIO 2 low, which makes SAF-10 a hardware property rather than a matter of
// ordering in motors_init(). The divider (10k up against the RP2350's 36-113k
// pad pull-down in parallel with the TB6612's own 200k) predicts 2.49-2.90 V at
// 3.3 V VCC, comfortably above the RP2350's input threshold.
//
// Merely needing gpio_put(enable, true) to make motors spin does NOT establish
// this: gpio_init() zeroes the SIO output register, so gpio_set_dir(pin, OUT)
// drives the pad low whether or not a pull-up exists. The only way to read the
// undriven state is to read it BEFORE the output driver is ever enabled, which
// is what this does and why it exists as its own program.
//
// This never enables an output. gpio_init() leaves the pin an input with the
// reset pull-down still selected; nothing here can drive a pin or move a wheel.

#include <stdio.h>
#include "pico/stdlib.h"

// Mirrors gpio_defines.h. Duplicated rather than included so this program has
// no dependency on the firmware tree it is diagnosing.
typedef struct
{
    const char* name;
    uint pin;
    const char* note;
} pad_t;

static const pad_t pads[] = {
    {"driver_enable (STBY)", 2, "HIGH here means R1 is fitted and the driver boots ENABLED"},
    {"right_pwm", 3, ""},
    {"right_forward (AIN1)", 4, ""},
    {"right_backward (AIN2)", 5, ""},
    {"left_pwm", 6, ""},
    {"left_forward (BIN1)", 7, ""},
    {"left_backward (BIN2)", 8, ""},
};

int main(void)
{
    stdio_init_all();

    // Give the USB host time to enumerate and open the port, otherwise the
    // first report is written into the void.
    sleep_ms(3000);

    // Read every pad BEFORE touching direction on any of them. gpio_init sets
    // SIO function and enables the input buffer, leaving the pin an input with
    // the pad's reset pull-down still selected -- which is exactly the
    // condition the divider analysis describes.
    for (unsigned i = 0; i < count_of(pads); i++)
    {
        gpio_init(pads[i].pin);
    }
    sleep_ms(10); // let the input buffers settle before sampling

    bool level[count_of(pads)];
    for (unsigned i = 0; i < count_of(pads); i++)
    {
        level[i] = gpio_get(pads[i].pin);
    }

    for (;;)
    {
        printf("\n=== pad-probe: undriven motor pin states ===\n");
        for (unsigned i = 0; i < count_of(pads); i++)
        {
            printf("  GPIO %2u  %-22s %s%s%s\n", pads[i].pin, pads[i].name,
                   level[i] ? "HIGH" : "LOW ", pads[i].note[0] ? "   <- " : "", pads[i].note);
        }

        printf("\n  VERDICT: ");
        if (level[0])
        {
            printf("STBY reads HIGH undriven -- R1 pull-up IS fitted.\n"
                   "           The driver is enabled before firmware runs. SAF-10 is\n"
                   "           violated by the hardware; issue #26 is a real defect.\n");
        }
        else
        {
            printf("STBY reads LOW undriven -- NO effective pull-up.\n"
                   "           The board does not match the published 2448 schematic, or R1\n"
                   "           is absent/removed. Issue #26 should be closed and rt.h and\n"
                   "           ADR-0010 corrected back.\n");
        }

        bool any_drive_pin_high = false;
        for (unsigned i = 1; i < count_of(pads); i++)
        {
            any_drive_pin_high |= level[i];
        }
        printf("           Direction/PWM pins all low: %s%s\n", any_drive_pin_high ? "NO" : "yes",
               any_drive_pin_high
                   ? "  <- unexpected; a high IN or PWM pin means torque is possible at boot"
                   : "  (IN1=IN2=L is the TB6612 coast row, so no torque regardless of STBY)");

        sleep_ms(2000);
    }
}
