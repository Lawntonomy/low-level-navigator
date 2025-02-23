#include "motor_driver.hpp"
#include "hardware/pwm.h"

MotorControl::MotorControl(uint pin_pwm, uint pin_dir, uint enc_a, uint enc_b)
    : pwm_pin(pin_pwm), dir_pin(pin_dir), encA(enc_a), encB(enc_b), target_rpm(0), current_rpm(0),
      pwm_output(0), integral(0), prev_error(0)
{
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    gpio_set_dir(dir_pin, GPIO_OUT);
}

void MotorControl::set_speed(float target_speed)
{
    // Convert speed (m/s) to RPM
    target_rpm = (target_speed / (WHEEL_DIAMETER * M_PI)) * 60;
}

float MotorControl::get_rpm()
{
    // Read encoder pulses and compute RPM (to be implemented)
    return current_rpm;
}

void MotorControl::update_pid()
{
    float error = target_rpm - getRPM();
    integral += error;
    float derivative = error - prev_error;
    prev_error = error;

    pwm_output = Kp * error + Ki * integral + Kd * derivative;
    pwm_output = pwm_output > 255 ? 255 : (pwm_output < 0 ? 0 : pwm_output);

    // Apply PWM output to motor
    pwm_set_gpio_level(pwm_pin, pwm_output);
}
