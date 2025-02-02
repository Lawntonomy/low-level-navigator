#include "pid.hpp"
#include <algorithm>
#include "utility/logger.h"

static const char* category = "PID";

PidClass::PidClass(float kp_init, float ki_init, float kd_init)
    : kp(kp_init), ki(ki_init), kd(kd_init), integral_component(0.0), output(0.0)
{
    Log::info(category, "init Pid");
}

PidClass::~PidClass()
{
}

float PidClass::control_loop(float current, float target)
{
    float error = target - current;

    integral_component += error;
    float derivative = error - prior_error;
    float delta_output = kp * error + ki * integral_component + kd * derivative;
    prior_error = error;
    // clamp outputs to prevent overruns
    if (output + delta_output >= max_output)
    {
        Log::warn(category, "commanding max value");
        return max_output;
    }
    if (output - delta_output <= min_output)
    {
        Log::warn(category, "commanding min value");
        return min_output;
    }

    return output += delta_output;
}

void PidClass::set_max_output(float num_in)
{
    max_output = num_in;
}
float PidClass::get_max_output()
{
    return max_output;
}
void PidClass::set_min_output(float num_in)
{
    min_output = num_in;
}
float PidClass::get_min_output()
{
    return min_output;
}
