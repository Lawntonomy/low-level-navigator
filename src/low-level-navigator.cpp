/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "hardware_drivers/gpio_driver.hpp"
#include "high_level_drivers/propulsion_engine.hpp"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "task.h"
#include "utility/logger.h"
// Stack sizes of our threads in words (4 bytes)
#define MIN_STACK_SIZE configMINIMAL_STACK_SIZE

static const char* category = "main_task";
// TaskHandle_t main_task;

void main_task(void* pvParameters)
{
    Log::info(category, "Main task started");

    GpioDriver gpio_driver;
    gpio_driver.gpio_start();

    PropulsionEngineClass prop_engine;

    prop_engine.start();

    for (;;)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void vLaunch(void)
{
    TaskHandle_t task;
    Log::info(category, "Launching initial tasks");
    xTaskCreate(main_task, "main_task", MIN_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();

    vLaunch();
    return 0;
}