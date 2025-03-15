#include "propulsion_engine.hpp"
#include <algorithm>
#include "utility/logger.h"

static const char* category = "prop_engine";

PropulsionEngineClass::PropulsionEngineClass(float kp_init, float ki_init,
                                             float kd_init)
    : kp(kp_init), ki(ki_init), kd(kd_init) {}

PropulsionEngineClass::~PropulsionEngineClass() {}

uint8_t PropulsionEngineClass::set_speed(uint16_t new_rpm) {
    // turn this into a queue recieive
    if (new_rpm < 200) {
        Log::info(category, "Set Speed to %d", new_rpm);
        target_rpm = new_rpm;
    } else {
        Log::error(category, "Speed Command Invalid! Speed: %d", new_rpm);
    }
}

void PropulsionEngineClass::control_loop(int32_t current_rpm) {
    // any initialization

    for (;;) {
        error_rpm = static_cast<float>(target_rpm - current_rpm);

        integral_rpm += error_rpm;
        float derivative = error_rpm - prior_error;
        int delta_output = static_cast<int>(kp * error_rpm + ki * integral_rpm +
                                            kd * derivative);

        // clamp outputs to prevent overruns
        if (output + delta_output >= 65000) {
            Log::warn(category, "commanding max value");
            return output;
        }
        if (output - delta_output <= -65000) {
            Log::warn(category, "commanding min value");
            return output;
        }
        prior_error = error_rpm;

        vtaskdelay(50);
    }
}