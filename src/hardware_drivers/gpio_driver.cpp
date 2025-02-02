#include "gpio_driver.hpp"
#include <cassert>
#include <pico/time.h>
#include "FreeRTOS.h"
#include "hardware/pwm.h"
#include "queue.h"
#include "task.h"
#include "utility/logger.h"
std::map<uint, QueueHandle_t> irq_queue;
static const char* category = "gpio_driver";

GpioDriver::GpioDriver(/* args */) : hardware_ready(false) {}

GpioDriver::~GpioDriver() {}

void GpioDriver::gpio_start() {
    // start all tasks and queues
    init_gpio_output_pin(PICO_DEFAULT_LED_PIN);
    init_gpio_output_pin(static_cast<uint>(
        config_defines::config_1::gpio_num::driver_enable_pin));
    init_gpio_output_pin(static_cast<uint>(
        config_defines::config_1::gpio_num::right_forward_pin));
    init_gpio_output_pin(static_cast<uint>(
        config_defines::config_1::gpio_num::right_backward_pin));
    init_gpio_output_pin(static_cast<uint>(
        config_defines::config_1::gpio_num::left_forward_pin));
    init_gpio_output_pin(static_cast<uint>(
        config_defines::config_1::gpio_num::left_backward_pin));
    init_pwm_output_pin(
        static_cast<uint>(config_defines::config_1::gpio_num::right_pwm_pin),
        config_defines::config_1::pwm_frequency);
    init_pwm_output_pin(
        static_cast<uint>(config_defines::config_1::gpio_num::left_pwm_pin),
        config_defines::config_1::pwm_frequency);

    hardware::gpio_update_queue =
        xQueueCreate(25, sizeof(hardware::gpio_update));
    assert((void("failed to init:  gpio_update queue"),
            hardware::gpio_update_queue == nullptr));

    hardware::neo_pixel_queue =
        xQueueCreate(10, sizeof(hardware::neopixel_update));
    assert((void("failed to init:  neopixel_update queue"),
            hardware::gpio_update_queue == nullptr));

    irq_queue[static_cast<uint>(
        config_defines::config_1::gpio_num::left_encoder_pin)] =
        xQueueCreate(25, sizeof(uint16_t));
    assert((void("failed to init:  left_encoder queue"),
            irq_queue[static_cast<uint>(
                config_defines::config_1::gpio_num::left_encoder_pin)] ==
                nullptr));

    irq_queue[static_cast<uint>(
        config_defines::config_1::gpio_num::right_encoder_pin)] =
        xQueueCreate(25, sizeof(uint16_t));
    assert((void("failed to init:  right_encoder queue"),
            irq_queue[static_cast<uint>(
                config_defines::config_1::gpio_num::right_encoder_pin)] ==
                nullptr));

    xTaskCreate(GpioDriver::led_heartbeat_task, "led_heartbeat_task", 1000,
                this, tskIDLE_PRIORITY + 2UL, NULL);
    assert((void("failed to init:  led_heartbeat_task"),
            GpioDriver::led_heartbeat_task == nullptr));

    xTaskCreate(GpioDriver::neopixel_status_task, "neopixel_status_task", 1000,
                this, tskIDLE_PRIORITY + 2UL, NULL);
    assert((void("failed to init:  neopixel_status_task"),
            GpioDriver::neopixel_status_task == nullptr));

    xTaskCreate(GpioDriver::update_gpio_task, "update_gpio_task", 1000, this,
                tskIDLE_PRIORITY + 2UL, NULL);
    assert((void("failed to init:  update_gpio_task"),
            GpioDriver::update_gpio_task == nullptr));
    hardware_ready = true;
}

void GpioDriver::get_time_slices(uint16_t gpio) {}

bool GpioDriver::is_hardware_ready() {
    return hardware_ready;
}

void GpioDriver::led_heartbeat_task(void* parameter) {
    bool flipper = 0;
    for (;;) {
        flipper = true;
        gpio_put(PICO_DEFAULT_LED_PIN, flipper);
        vTaskDelay(500);
        flipper = false;
        gpio_put(PICO_DEFAULT_LED_PIN, flipper);
        vTaskDelay(500);
    }
}

void GpioDriver::neopixel_status_task(void* parameter) {

    // first led reserved for update gpio
    // flashing red: update_gpio has exited
    // flashing yellow: within 100ms of timing out in last 5 seconds
    // flashing green: all gpio updating correctly

    vTaskDelay(1000);
}

/// @brief Watches gpio_update queue, if pin isn't updated within 300ms, io
/// locks out
void GpioDriver::update_gpio_task(void* parameter) {
    GpioDriver* driver = static_cast<GpioDriver*>(parameter);
    // run initialization

    hardware::gpio_update update;

    // run any checks nessesary

    for (;;) {
        // recieve from queue
        if (xQueueReceive(hardware::gpio_update_queue, &update, 10)) {
            // check if time and value makes sense
            if (update.timestamp + 300 < (time_us_32() / 1000)) {
                if (update.max_value >= update.value) {
                    if (update.max_value == 1) {
                        if (update.value !=
                            driver->gpio_updates[static_cast<uint>(update.pin)]
                                .value) {
                            gpio_put(static_cast<uint>(update.pin),
                                     update.value);
                        }
                        driver->gpio_updates[static_cast<uint>(update.pin)] =
                            update;
                    } else {

                        if (update.value !=
                            driver->gpio_updates[static_cast<uint>(update.pin)]
                                .value) {
                            pwm_set_gpio_level(static_cast<uint>(update.pin),
                                               update.value);
                        }
                        driver->gpio_updates[static_cast<uint>(update.pin)] =
                            update;
                    }
                }
            }
        }

        // check if any gpio have timed out//
        // turn off all gpio
        // exit loop
    }

    Log::error(category, "gpio_update has exited");
}

void GpioDriver::init_freq_input_pin(uint16_t gpio) {
    Log::info(category, "Initializing Encoder at pin: %d", gpio);
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
    gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_FALL, true,
                                       GpioDriver::frequency_irq);
}

void GpioDriver::init_pwm_output_pin(uint16_t gpio, uint16_t frequency) {
    Log::info(category, "Initializing PWM at pin: %d", gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.f); // Set the clock divider
    pwm_set_wrap(
        slice_num,
        65535); // Set the PWM period to maximum value (16-bit resolution)
    pwm_set_gpio_level(gpio, 0); // Set initial duty cycle
    pwm_set_enabled(slice_num, true);
    pwm_set_clkdiv(slice_num, 125.f * (float)1000 / frequency);
    gpio_updates[gpio] = hardware::gpio_update(
        static_cast<config_defines::config_1::gpio_num>(gpio), 0, 65535, 0);
}

void GpioDriver::init_gpio_output_pin(uint16_t gpio) {
    Log::info(category, "Initializing GPIO output at pin: %d", gpio);
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_updates[gpio] = hardware::gpio_update(
        static_cast<config_defines::config_1::gpio_num>(gpio), 0, 1, 0);
}

void GpioDriver::init_gpio_input_pin(uint16_t gpio, bool pull_up,
                                     bool pull_down) {}
// this needs to sends gpio pin with timestamp
void GpioDriver::frequency_irq(uint gpio, uint32_t event_mask) {
    static std::map<uint, uint32_t> last_reading;
    uint32_t current_reading = 0;
    current_reading = time_us_32();
    current_reading - last_reading[gpio];
    uint32_t to_queue = current_reading - last_reading[gpio];
    QueueHandle_t queue = irq_queue[gpio];
    xQueueSendFromISR(queue, &to_queue, NULL);
    last_reading[gpio] = current_reading;
}
