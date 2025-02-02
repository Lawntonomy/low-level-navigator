#pragma once
#include "pico/stdlib.h"

class PropulsionEngineClass
{
  public:
    PropulsionEngineClass(float kp_init, float ki_init, float kd_init);

    uint8_t set_speed(uint16_t new_rpm);
    void control_loop(int32_t current_rpm);

  private:
    int32_t target_rpm;
    float error_rpm;
    float prior_error;
    int32_t output = 0;
    float integral_rpm;

    float kp;
    float ki;
    float kd;
};