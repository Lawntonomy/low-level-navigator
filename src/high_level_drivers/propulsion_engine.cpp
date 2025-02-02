#include "propulsion_engine.hpp"
#include <cstring>
#include "hardware_drivers/hardware_defines.hpp"
#include "pid.hpp"

void PropulsionEngineClass::main_prop_task(void* parameter)
{
    PropulsionEngineClass* prop = static_cast<PropulsionEngineClass*>(parameter);

    PidClass pid_left(32.0, 0.0, 0.0);
    PidClass pid_right(32.0, 0.0, 0.0);

    pid_left.set_max_output(65000);
    pid_left.set_min_output(-65000);
    pid_right.set_max_output(65000);
    pid_right.set_min_output(-65000);

    for (;;)
    {
        auto [command_left_rpm, command_right_rpm] = prop->get_command_rpm();
        // get target speed
        // get current speed
        prop->update_rpm(config_defines::config_1::gpio_num::left_encoder_pin);
        prop->update_rpm(config_defines::config_1::gpio_num::right_encoder_pin);

        // run pid loop
        pid_left.control_loop(
            prop->rpms[static_cast<uint>(config_defines::config_1::gpio_num::left_encoder_pin)].rpm,
            command_left_rpm);
        pid_right.control_loop(
            prop->rpms[static_cast<uint>(config_defines::config_1::gpio_num::right_encoder_pin)]
                .rpm,
            command_right_rpm);
        // set pwm values
    }
}

void PropulsionEngineClass::update_rpm(config_defines::config_1::gpio_num gpio)
{

    uint gpio_uint = static_cast<uint>(gpio);
    uint32_t current_timing = time_us_32();
    uint32_t time_slice;
    // get encoder values
    while (xQueueReceive(hardware::irq_queue[gpio_uint], &time_slice, 0))
    {
        std::memcpy(&rpms[gpio_uint].readings[1], &rpms[gpio_uint].readings[0],
                    (13) * sizeof(float));

        rpms[gpio_uint].readings[0] = time_slice / 60000000.0;

        std::memcpy(&rpms[gpio_uint].time_recorded[1], &rpms[gpio_uint].time_recorded[0],
                    (13) * sizeof(uint32_t));
        rpms[gpio_uint].time_recorded[0] = current_timing;
    }

    float times_summed = 0;
    uint8_t iter = 0;

    // averages time between 2 tick marks, up to 14 ticks, or 300ms,
    // whichever comes first
    while (rpms[gpio_uint].time_recorded[iter] + 300000 > current_timing && iter <= 14)
    {
        times_summed += rpms[gpio_uint].readings[iter];
        iter++;
    }

    if (iter == 0)
    {
        rpms[gpio_uint].rpm = 0;
        return;
    }

    float rpm =
        (static_cast<float>(iter) / config_defines::config_1::encoder_ticks) / (times_summed);

    rpms[gpio_uint].rpm = static_cast<uint>(rpm);
}

std::pair<uint32_t, uint32_t> PropulsionEngineClass::get_command_rpm()
{
    return std::make_pair(command_left_rpm, command_right_rpm);
}

void Propulsion::startup_test()
{
    enum class SUBTASKS
    {
        INITIALIZATION,
        ENCODER_TEST,
        SUSPEND
    };

    SUBTASKS test_step = SUBTASKS::INITIALIZATION;

    for (;;)
    {
        switch (test_step)
        {
        case SUBTASKS::INITIALIZATION:
        {
            Log::info("hardware_state", "Initializing hardware");
            uint16_t returnstat = hardware.init_other_gpio();
            returnstat += hardware.left_hardware_start();
            returnstat += hardware.right_hardware_start();

            if (returnstat == 0)
            {
                hardware.set_motors_enabled(true);
                Log::info("hardware_state", "Hardware initialization successful");
                test_step = SUBTASKS::ENCODER_TEST;
            }
            else
            {
                Log::error("hardware_state", "Hardware initialization errors");
                test_step = SUBTASKS::SUSPEND;
            }

            break;
        }
        case SUBTASKS::ENCODER_TEST:
        {
            Log::info("hardware_state", "Starting Motion, stand clear!");
            vTaskDelay(3000);
            hardware.(30000, true, false);
            hardware.set_right_pwm_output(30000, false, true);

            vTaskDelay(1000);
            uint16_t left_rpm = hardware.get_left_rpm();
            uint16_t right_rpm = hardware.get_right_rpm();
            if (left_rpm > 0 && right_rpm > 0)
            {
                Log::info("hardware_state", "Encoders working: %u %u", left_rpm, right_rpm);
                hardware.set_left_pwm_output(0, false, false);
                hardware.set_right_pwm_output(0, false, false);
            }
            else
            {
                Log::info("hardware_state", "Encoders failed startup test: %u %u", left_rpm,
                          right_rpm);
                hardware.set_left_pwm_output(0, false, false);
                hardware.set_right_pwm_output(0, false, false);
                test_step = SUBTASKS::SUSPEND;
            }

            break;
        }
        case SUBTASKS::SUSPEND:
        {
            Log::info("hardware_state", "Startup Task Completed");
            vTaskDelete(NULL);

            break;
        }
        default: break;
        }
    }
}