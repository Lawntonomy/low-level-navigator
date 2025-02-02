#pragma once
#include "pico/stdlib.h"

class PidClass {
  public:
    PidClass(float kp_init, float ki_init, float kd_init);
    ~PidClass();

    int32_t control_loop(int32_t current, int32_t target);

    void set_max_output(int32_t num_in);
    int32_t get_max_output();

    void set_min_output(int32_t num_in);
    int32_t get_min_output();

  private:
    int32_t max_output;
    int32_t min_output;

    int32_t target;
    float error;
    float integral;
    float prior_error;
    int32_t output = 0;
    float integral_component;

    float kp;
    float ki;
    float kd;
};