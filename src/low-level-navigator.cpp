#include "FreeRTOS.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "queue.h"
#include "task.h"
#include "utility/logger.h"

#include "high_level_drivers/navigator.hpp"
static const char* category = "main";

void led_heartbeat_task(void* parameter)
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);
    bool flipper = 0;
    for (;;)
    {
        flipper = true;
        gpio_put(PICO_DEFAULT_LED_PIN, flipper);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        flipper = false;
        gpio_put(PICO_DEFAULT_LED_PIN, flipper);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void vLaunch(void)
{
    TaskHandle_t task;
    Log::info(category, "Launching initial tasks");
    xTaskCreate(led_heartbeat_task, "led_task", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 2UL, NULL);

    xTaskCreate(navigator::navigator_task, "nav_task", configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 2UL, NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();

    vLaunch();
    return 0;
}
