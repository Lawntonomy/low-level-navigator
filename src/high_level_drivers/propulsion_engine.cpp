#include "propulsion_engine.hpp"
#include <cstring>
#include "hardware_drivers/hardware_defines.hpp"
#include "logger.h"
#include "pid.hpp"

static const char* category = "prop_engine";

PropulsionEngineClass::PropulsionEngineClass() : command_left_rpm(0), command_right_rpm(0)
{
}

void PropulsionEngineClass::start()
{
    xTaskCreate(PropulsionEngineClass::main_prop_task, "main_prop_task", 1000, this,
                tskIDLE_PRIORITY + 2UL, NULL);
    assert((void("failed to init:  main_prop_task"),
            PropulsionEngineClass::main_prop_task != nullptr));

    xTaskCreate(PropulsionEngineClass::startup_test_task, "startup_test_task", 1000, this,
                tskIDLE_PRIORITY + 2UL, NULL);
    assert((void("failed to init:  startup_test_task"),
            PropulsionEngineClass::startup_test_task != nullptr));
}

void PropulsionEngineClass::main_prop_task(void* parameter)
{
    PropulsionEngineClass* prop = static_cast<PropulsionEngineClass*>(parameter);

    PidClass pid_left(16.0, 0.0, 0.0);
    PidClass pid_right(16.0, 0.0, 0.0);

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
        int32_t left_command = pid_left.control_loop(
            prop->rpms[static_cast<uint>(config_defines::config_1::gpio_num::left_encoder_pin)].rpm,
            command_left_rpm);
        int32_t right_command = pid_right.control_loop(
            prop->rpms[static_cast<uint>(config_defines::config_1::gpio_num::right_encoder_pin)]
                .rpm,
            command_right_rpm);

        prop->command_propulsion(left_command, right_command);
        // set pwm values

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void PropulsionEngineClass::set_commanded_rpm(int32_t left_command, int32_t right_command)
{
    command_left_rpm = left_command;
    command_right_rpm = right_command;
}

void PropulsionEngineClass::update_rpm(config_defines::config_1::gpio_num gpio)
{
    uint gpio_uint = static_cast<uint>(gpio);
    uint32_t current_timing = time_us_32();
    uint32_t time_slice;
    // get encoder values
    while (xQueueReceive(hardware::irq_queue[gpio_uint], &time_slice, 0) == pdTRUE)
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

void PropulsionEngineClass::startup_test_task(void* parameter)
{
    PropulsionEngineClass* prop = static_cast<PropulsionEngineClass*>(parameter);
    Log::info("hardware_state", "Starting Motion, stand clear!");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // move forward
    Log::info("hardware_state", "Moving Forward");
    prop->set_commanded_rpm(20, 20);
    for (int i = 0; i < 50; i++)
    {
        std::pair<uint, uint> rpm_pins = std::make_pair<uint, uint>(
            static_cast<uint>(config_defines::config_1::gpio_num::left_encoder_pin),
            static_cast<uint>(config_defines::config_1::gpio_num::right_encoder_pin));

        Log::info(category, "RPM: %d, %d", prop->rpms[rpm_pins.first].rpm,
                  prop->rpms[rpm_pins.second].rpm);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // check for rpm
    Log::info("hardware_state", "Stopping");
    prop->set_commanded_rpm(0, 0);
    for (int i = 0; i < 50; i++)
    {
        std::pair<uint, uint> rpm_pins = std::make_pair<uint, uint>(
            static_cast<uint>(config_defines::config_1::gpio_num::left_encoder_pin),
            static_cast<uint>(config_defines::config_1::gpio_num::right_encoder_pin));
        Log::info(category, "RPM: %d, %d", prop->rpms[rpm_pins.first].rpm,
                  prop->rpms[rpm_pins.second].rpm);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    Log::info("hardware_state", "Moving Backwards");
    // move backward
    prop->set_commanded_rpm(-20, -20);
    for (int i = 0; i < 50; i++)
    {
        std::pair<uint, uint> rpm_pins = std::make_pair<uint, uint>(
            static_cast<uint>(config_defines::config_1::gpio_num::left_encoder_pin),
            static_cast<uint>(config_defines::config_1::gpio_num::right_encoder_pin));

        Log::info(category, "RPM: %d, %d", prop->rpms[rpm_pins.first].rpm,
                  prop->rpms[rpm_pins.second].rpm);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    prop->set_commanded_rpm(0, 0);
    // check for rpm
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Log::info("hardware_state", "Startup Task Completed");
    vTaskDelete(NULL);
}

void PropulsionEngineClass::command_propulsion(int32_t left_command, int32_t right_command)
{

    // Left Encoder
    uint32_t timestamp = time_us_32() / 1000;
    hardware::gpio_update update;
    update.max_value = 65000;
    update.value = abs(left_command);
    update.pin = config_defines::config_1::gpio_num::left_pwm_pin;
    update.timestamp = timestamp;
    xQueueSend(hardware::gpio_update_queue, &update, 10);
    // Right Encoder
    update.max_value = 65000;
    update.value = abs(right_command);
    update.pin = config_defines::config_1::gpio_num::right_pwm_pin;
    update.timestamp = timestamp;
    xQueueSend(hardware::gpio_update_queue, &update, 10);
    // Left Forward
    update.max_value = 1;
    update.value = left_command > 0;
    update.pin = config_defines::config_1::gpio_num::left_forward_pin;
    update.timestamp = timestamp;
    xQueueSend(hardware::gpio_update_queue, &update, 10);
    // Left Backward
    update.max_value = 1;
    update.value = left_command < 0;
    update.pin = config_defines::config_1::gpio_num::left_backward_pin;
    update.timestamp = timestamp;
    xQueueSend(hardware::gpio_update_queue, &update, 10);
    // Right Forward
    update.max_value = 1;
    update.value = right_command > 0;
    update.pin = config_defines::config_1::gpio_num::right_forward_pin;
    update.timestamp = timestamp;
    xQueueSend(hardware::gpio_update_queue, &update, 10);
    // Right Backward
    update.max_value = 1;
    update.value = right_command < 0;
    update.pin = config_defines::config_1::gpio_num::right_backward_pin;
    update.timestamp = timestamp;
    xQueueSend(hardware::gpio_update_queue, &update, 10);
    // Enable
    update.max_value = 1;
    update.value = 1;
    update.pin = config_defines::config_1::gpio_num::driver_enable_pin;
    update.timestamp = timestamp;
    xQueueSend(hardware::gpio_update_queue, &update, 10);
}
