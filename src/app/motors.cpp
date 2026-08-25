#include "app/motors.hpp"

#include "hardware/gpio.h"

#include "hardware_drivers/gpio_defines.h"

namespace motors
{

void safe_state()
{
    // SAF-10: every output at a defined zero BEFORE drive enable is asserted.
    //
    // The PWM pins are included even though PIO PWM is not attached yet. When
    // it is, this function is the whole stop mechanism, and PIO drives those
    // pins autonomously — so zeroing duty has to happen here or the function
    // whose entire job is "make the outputs safe" becomes a no-op for speed.
    gpio_put(static_cast<uint>(gpio::pins::left_pwm_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::right_pwm_pin), 0);

    gpio_put(static_cast<uint>(gpio::pins::left_forward_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::left_backward_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::right_forward_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::right_backward_pin), 0);

    gpio_put(static_cast<uint>(gpio::pins::driver_enable_pin), 0);
}

void init()
{
    const uint pins[] = {
        static_cast<uint>(gpio::pins::driver_enable_pin),
        static_cast<uint>(gpio::pins::left_pwm_pin),
        static_cast<uint>(gpio::pins::right_pwm_pin),
        static_cast<uint>(gpio::pins::left_forward_pin),
        static_cast<uint>(gpio::pins::left_backward_pin),
        static_cast<uint>(gpio::pins::right_forward_pin),
        static_cast<uint>(gpio::pins::right_backward_pin),
    };
    for (uint p : pins)
    {
        gpio_init(p);
        gpio_put(p, 0); // drive low before enabling the output driver
        gpio_set_dir(p, GPIO_OUT);
    }
    safe_state();
}

} // namespace motors
