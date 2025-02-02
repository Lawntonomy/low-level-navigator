#pragma once
#include <map>
#include "hardware_drivers/configuration_defines.hpp"
#include "pico/stdlib.h"

class PropulsionEngineClass
{
  public:
    PropulsionEngineClass();
    void start();

  private:
    std::pair<uint32_t, uint32_t> get_command_rpm();
    static void main_prop_task(void* parameter);
    void update_rpm(config_defines::config_1::gpio_num gpio);
    static void startup_test_task(void* parameter);
    void command_propulsion(int32_t, int32_t);
    void set_commanded_rpm(int32_t, int32_t);
    struct rpm_type
    {
        int32_t rpm;
        float readings[15];
        float time_recorded[15];
        uint32_t time_slice;
    };
    std::map<uint, rpm_type> rpms;

    int32_t command_left_rpm;
    int32_t command_right_rpm;
};