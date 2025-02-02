#include "propulsion_engine.hpp"
#include <algorithm>
#include "utility/logger.h"

static const char* category = "prop_engine";

PropulsionEngineClass::PropulsionEngineClass(float kp_init, float ki_init, float kd_init)
    : kp(kp_init), ki(ki_init), kd(kd_init)
{
}

uint8_t PropulsionEngineClass::set_speed(uint16_t new_rpm)
{
    // turn this into a queue recieive
    if (new_rpm < 200)
    {
        Log::info(category, "Set Speed to %d", new_rpm);
        target_rpm = new_rpm;
    }
    else
    {
        Log::error(category, "Speed Command Invalid! Speed: %d", new_rpm);
    }
}

void PropulsionEngineClass::control_loop(int32_t current_rpm)
{
    // any initialization

    for (;;)
    {
        error_rpm = static_cast<float>(target_rpm - current_rpm);

        integral_rpm += error_rpm;
        float derivative = error_rpm - prior_error;
        int delta_output = static_cast<int>(kp * error_rpm + ki * integral_rpm + kd * derivative);

        // clamp outputs to prevent overruns
        if (output + delta_output >= 65000)
        {
            Log::warn(category, "commanding max value");
            return output;
        }
        if (output - delta_output <= -65000)
        {
            Log::warn(category, "commanding min value");
            return output;
        }
        prior_error = error_rpm;

        vtaskdelay(50);
    }
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