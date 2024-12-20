/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "utility/logger.h"
#include "hardware_drivers/gpio_driver.hpp"
// Stack sizes of our threads in words (4 bytes)
#define MIN_STACK_SIZE configMINIMAL_STACK_SIZE

static const char* category = "gpio_driver";
//TaskHandle_t main_task;

void main_task(void *pvParameters)
{
    Log::info(category, "Main task started");
    
    GpioDriver gpio_driver; 
    gpio_driver.gpio_start();


    for (;;)
    {
        vTaskDelay(10000);
    }
}



void vLaunch( void) {
    TaskHandle_t task;
    Log::info(category, "Launching initial tasks");
    xTaskCreate(main_task, "main_task", MIN_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main( void )
{
    stdio_init_all();

    vLaunch();

    return 0;
}