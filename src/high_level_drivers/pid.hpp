#pragma once
#include "pico/stdlib.h"

class PidClass
{
  public:
    PidClass(float kp_init, float ki_init, float kd_init);
    ~PidClass();

    float control_loop(float current, float target);

    void set_max_output(float num_in);
    float get_max_output();

    void set_min_output(float num_in);
    float get_min_output();

  private:
    int32_t max_output;
    int32_t min_output;

    float prior_error;
    float integral_component;
    float output;
    float kp;
    float ki;
    float kd;
};