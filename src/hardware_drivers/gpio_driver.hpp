#pragma
#include <map>
#include <vector>
#include "hardware_defines.hpp"
#include "pico/stdlib.h"
class GpioDriver
{
    // initialization
  public:
    GpioDriver();
    void gpio_start();
    // usage
  public:
    bool is_hardware_ready();
    static std::vector<uint32_t> get_time_slices(config_defines::config_1::gpio_num gpio);
    // tasks
  private:
    static void led_heartbeat_task(void* parameter);
    static void neopixel_status_task(void* parameter);
    static void update_gpio_task(void* parameter);
    // utility functions
  private:
    void init_freq_input_pin(uint16_t gpio);
    void init_pwm_output_pin(uint16_t gpio, uint16_t frequency);
    void init_gpio_output_pin(uint16_t gpio);
    void init_gpio_input_pin(uint16_t gpio, bool pull_up, bool pull_down);
    void static frequency_irq(uint gpio, uint32_t event_mask);

  private:
    std::map<uint16_t, hardware::gpio_update> gpio_updates;
    bool hardware_ready;
};
