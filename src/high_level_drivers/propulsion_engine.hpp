#pragma once
#include <map>
#include "hardware_drivers/configuration_defines.hpp"
#include "pico/stdlib.h"

class PropulsionEngineClass
{
  public:
    PropulsionEngineClass(float kp_init, float ki_init, float kd_init);

    std::pair<uint32_t, uint32_t> get_command_rpm();

  private:
    static void main_prop_task(void* parameter);
    void update_rpm(config_defines::config_1::gpio_num gpio);
    struct rpm_type
    {
        uint32_t rpm;
        float readings[15];
        float time_recorded[15];
        uint32_t time_slice;
    };
    std::map<uint, rpm_type> rpms;

    uint32_t command_left_rpm;
    uint32_t command_right_rpm;
};