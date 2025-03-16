// we need to add states so we can successfully go from forware to backware without jerking the
// machine a machine should never go from commanding forward, to straight backward state transitions
// consist of: forward -> stopped stopped -> forward backward -> stopped stopped -> backward
#include "navigator.hpp"
#include <cmath>
#include "FreeRTOS.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "task.h"

#include "hardware_drivers/encoder.hpp"
#include "hardware_drivers/gpio_defines.h"
#include "hardware_drivers/pwm.h"
#include "high_level_drivers/pid.hpp"

void navigator::navigator_task(void* pvParameters)
{

    float left_target = 25.0;
    float right_target = -50.0;

    PIO pio[] = {pio1, pio2};
    uint sm[] = {0, 1, 2, 3};
    stdio_init_all();
    printf("Starting autonomy\n");

    gpio_init(static_cast<uint>(gpio::pins::driver_enable_pin));
    gpio_init(static_cast<uint>(gpio::pins::left_forward_pin));
    gpio_init(static_cast<uint>(gpio::pins::right_forward_pin));
    gpio_init(static_cast<uint>(gpio::pins::left_backward_pin));
    gpio_init(static_cast<uint>(gpio::pins::right_backward_pin));

    gpio_set_dir(static_cast<uint>(gpio::pins::driver_enable_pin), true);
    gpio_set_dir(static_cast<uint>(gpio::pins::left_forward_pin), true);
    gpio_set_dir(static_cast<uint>(gpio::pins::right_forward_pin), true);
    gpio_set_dir(static_cast<uint>(gpio::pins::left_backward_pin), true);
    gpio_set_dir(static_cast<uint>(gpio::pins::right_backward_pin), true);

    gpio_put(static_cast<uint>(gpio::pins::driver_enable_pin), true);

    encoder::init(pio[0], sm[0]);
    pwm::init(pio[0], sm[2]);

    PidClass pidLeft(32.0, 0.0, 0.0);
    PidClass pidRight(32.0, 0.0, 0.0);
    pidLeft.set_max_output(65000);
    pidLeft.set_min_output(-65000);
    pidRight.set_max_output(65000);
    pidRight.set_min_output(-65000);

    pwm::pio_pwm_set_period(pio[0], sm[2], (1u << 16) - 1);
    pwm::pio_pwm_set_period(pio[0], sm[3], (1u << 16) - 1);

    // left_target = 5.0;
    bool left_backward_state = false;
    bool right_backward_state = false;
    for (;;)
    {

        float left_rpm = encoder::get_left_rpm(pio[0], sm[0]);
        float right_rpm = encoder::get_right_rpm(pio[0], sm[1]);

        if (left_backward_state == true)
        {
            left_rpm = 0.0 - left_rpm;
        }

        if (right_backward_state == true)
        {
            right_rpm = 0.0 - right_rpm;
        }

        float left_command = pidLeft.control_loop(left_rpm, left_target);
        float right_command = pidRight.control_loop(right_rpm, right_target);

        pidLeft.set_max_output(left_rpm > -0.5 ? 65000.0 : 0.0);
        pidLeft.set_min_output(left_rpm < 0.5 ? -65000.0 : 0.0);
        pidRight.set_max_output(right_rpm > -0.5 ? 65000.0 : 0.0);
        pidRight.set_min_output(right_rpm < 0.5 ? -65000.0 : 0.0);

        pwm::pio_pwm_set_level(pio[0], sm[2], static_cast<uint32_t>(abs(left_command)));
        pwm::pio_pwm_set_level(pio[0], sm[3], static_cast<uint32_t>(abs(right_command)));

        if (left_rpm == 0.0)
        {
            left_backward_state = left_target < 0.0;
            gpio_put(static_cast<uint>(gpio::pins::left_forward_pin), !left_backward_state);
            gpio_put(static_cast<uint>(gpio::pins::left_backward_pin), left_backward_state);
        }
        if (right_rpm == 0.0)
        {
            right_backward_state = right_target < 0.0;
            gpio_put(static_cast<uint>(gpio::pins::right_forward_pin), !right_backward_state);
            gpio_put(static_cast<uint>(gpio::pins::right_backward_pin), right_backward_state);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}