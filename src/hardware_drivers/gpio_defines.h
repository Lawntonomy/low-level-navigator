#pragma once
#include "pico/stdlib.h"

namespace gpio
{
enum class pins
{
    right_pwm_pin = 3,
    right_forward_pin = 4,
    right_backward_pin = 5,
    left_pwm_pin = 6,
    left_forward_pin = 7,
    left_backward_pin = 8,
    // STBY on the motor driver, an Adafruit TB6612 breakout (product 2448,
    // Toshiba TB6612FNG). Active high: LOW is standby, HIGH enables the
    // H-bridges. That breakout carries R1, a 10 kΩ pull-up from STBY to VCC,
    // which outvotes the RP2350 pad pull-down - so the driver is ENABLED
    // whenever this pin is undriven, including from power-on until firmware
    // drives it low. Closing that window needs a board change (remove R1, or
    // fit an external pull-down of ≤ 4.3 kΩ, ideally ≈2.2 kΩ), not a firmware
    // change; see SAF-19 in ADR-0010 and the watchdog note in src/app/rt.h.
    driver_enable_pin = 2,
    right_encoder_pin = 10,
    left_encoder_pin = 11,
    neopixel_pin = 15

    // RESERVED, not enumerated here because no driver owns them yet. Wired and
    // confirmed on hardware 2026-09-09 by diagnostics/imu-probe:
    //   GP12 = I2C0 SDA, GP13 = I2C0 SCL -- LSM6DSOX at 0x6A, with a LIS3MDL
    //     at 0x1C on the same breakout (both address jumpers open). 400 kHz is
    //     the ceiling: the LSM6DSOX slave timing table has no Fast-mode-plus
    //     row. External pull-ups are required and present; RP2350 §12.2.1.3
    //     warns the pad pull-ups may not be strong enough, so do not remove
    //     them and do not enable the internal ones instead.
    //     I2C0 rather than I2C1 is arbitrary -- identical instances, separate
    //     IRQs and DREQs -- and leaves I2C1 free on GP18/19 and GP26/27.
    //   GP14 = LSM6DSOX INT1, the data-ready edge ADR-0007 timestamps. Not the
    //     breakout's DRDY pad, which belongs to the LIS3MDL. GP14 costs least:
    //     its own I2C1 SDA role is already dead because its SCL partner GP15
    //     is the NeoPixel, and it is not one of the scarce ADC-capable pins.
    //     Keep CTRL3_C PP_OD at its default (push-pull). Under erratum
    //     RP2350-E9, which applies to this A2 silicon, an undriven Bank 0 pad
    //     leaks toward ~2.2 V and the internal pull-down cannot hold it.
    //     Data-ready is LATCHED by default: INT1 clears only when the OUTPUT
    //     registers are read, so the driver's burst read is what re-arms the
    //     interrupt, not merely how the sample arrives.
};

const uint16_t pwm_frequency = 1000;
const float encoder_ticks = 20.0;
} // namespace gpio