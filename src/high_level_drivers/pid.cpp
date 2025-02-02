#include "pid.hpp"
#include <algorithm>
#include "utility/logger.h"

static const char* category = "PID";

PidClass::PidClass(float kp_init, float ki_init, float kd_init)
    : kp(kp_init), ki(ki_init), kd(kd_init) {}

PidClass::~PidClass() {}

int32_t PidClass::control_loop(int32_t current, int32_t target) {

    error = static_cast<float>(target - current);

    integral += error;
    float derivative = error - prior_error;
    int delta_output =
        static_cast<int>(kp * error + ki * integral + kd * derivative);

    // clamp outputs to prevent overruns
    if (output + delta_output >= 65000) {
        Log::warn(category, "commanding max value");
        return output;
    }
    if (output - delta_output <= -65000) {
        Log::warn(category, "commanding min value");
        return output;
    }
    prior_error = error;
    return output += delta_output;
}

void PidClass::set_max_output(int32_t num_in) {
    max_output = num_in;
}
int32_t PidClass::get_max_output() {
    return max_output;
}
void PidClass::set_min_output(int32_t num_in) {
    min_output = num_in;
}
int32_t PidClass::get_min_output() {
    return min_output;
}