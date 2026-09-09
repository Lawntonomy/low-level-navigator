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
    // Moved 8 -> 9 on 2026-09-08 to free GP8 for the console UART1 TX, which
    // in turn freed GP20 for I2C0 SDA where the LSM6DSOX is soldered. Nothing
    // about this pin needs a peripheral function; it is a plain output to the
    // TB6612, and GP9 is the adjacent header pin. See app/board.h.
    left_backward_pin = 9,
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

    // RESERVED, not enumerated here because no driver owns them yet:
    //   GP20 = I2C0 SDA, GP21 = I2C0 SCL — LSM6DSOX, soldered 2026-09-08.
    //   INT1 is still unassigned and unconfirmed on the board; ADR-0007 needs
    //   it (see low-level-navigator#44), and an unrouted INT1 is a blocker for
    //   data-ready timestamping rather than a degraded mode.
};

const uint16_t pwm_frequency = 1000;
const float encoder_ticks = 20.0;
} // namespace gpio