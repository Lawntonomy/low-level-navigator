#include "hardware_defines.hpp"

namespace hardware
{
std::map<uint, QueueHandle_t> irq_queue;
QueueHandle_t gpio_update_queue = nullptr;
QueueHandle_t neo_pixel_queue = nullptr;
} // namespace hardware