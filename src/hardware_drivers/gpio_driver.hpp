#pragma
#include "pico/stdlib.h"
#include "hardware_defines.hpp"
#include <map>
class GpioDriver
{
//initialization
public:
    GpioDriver(/* args */);
    ~GpioDriver();
    void gpio_start();
//usage
public:
    void get_time_slices(uint16_t gpio);
//tasks
private:
    static void led_heartbeat_task(void* parameter);
    static void neopixel_status_task(void* parameter);
    static void update_gpio_task(void* parameter);
//utility functions
private:
    void init_freq_input_pin(uint16_t gpio);
    void init_pwm_output_pin(uint16_t gpio, uint16_t frequency);
    void init_gpio_output_pin(uint16_t gpio);
    void init_gpio_input_pin(uint16_t gpio, bool pull_up, bool pull_down);
    void static frequency_irq(uint gpio, uint32_t event_mask);
    std::map<uint16_t, hardware::gpio_update> gpio_updates;
};


