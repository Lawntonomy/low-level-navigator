#pragma once
#include "FreeRTOS.h"
#include "configuration_defines.hpp"
#include "pico/stdlib.h"
#include "queue.h"

namespace hardware {
struct gpio_update {
    config_defines::config_1::gpio_num pin;
    uint16_t value;
    uint16_t max_value;
    uint32_t timestamp;

    gpio_update() {};
    gpio_update(config_defines::config_1::gpio_num pin_in, uint16_t value_in,
                uint16_t max_in, uint32_t time_in)
        : pin(pin_in), value(value_in), timestamp(time_in),
          max_value(max_in) {};
};

struct neopixel_update {
    uint8_t led_num;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    neopixel_update() {};
    neopixel_update(uint8_t led_num, uint8_t r, uint8_t g, uint8_t b)
        : led_num(led_num), r(r), g(g), b(b) {};
};

static QueueHandle_t gpio_update_queue;
static QueueHandle_t neo_pixel_queue;
} // namespace hardware