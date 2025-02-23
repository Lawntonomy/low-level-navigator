#pragma once

#include <stdint.h>
#include "pico/stdlib.h"

class MotorControl
{
  public:
    MotorControl(uint pin_pwm, uint pin_dir, uint enc_a, uint enc_b);
    void set_speed(float target_speed); // m/s
    void update_pid();
    float get_rpm();

  private:
    uint pwm_pin, dir_pin;
    uint encA, encB;
    float target_rpm;
    float current_rpm;
    float pwm_output;

    float Kp = 1.0, Ki = 0.1, Kd = 0.01; // Tune these
    float integral, prev_error;
};
