#include "logger.h"
#include <pico/time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

QueueHandle_t loggerQueue;
#define QUEUE_LENGTH 20
#define QUEUE_ITEM_SIZE 100 // Max string length: 100 characters

TaskHandle_t loggerTaskHandle = NULL;

void Log::trace(const char* cat, const char *format, ...)
{
        char message[QUEUE_ITEM_SIZE];
        char formattedMessage[QUEUE_ITEM_SIZE];
        va_list args;
        va_start(args, format);
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);

        snprintf(formattedMessage, sizeof(formattedMessage), "1 %lu %s %s\n", time_us_32()/1000, cat, message);

        if (xQueueSend(loggerQueue, formattedMessage, pdMS_TO_TICKS(50)) != pdTRUE)
        {
            // Handle queue full error (optional)
            printf("Logger queue is full. Dropping message");
        }
    
}
void Log::info(const char* cat, const char *format, ...)
{
    char message[QUEUE_ITEM_SIZE];
    char formattedMessage[QUEUE_ITEM_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    snprintf(formattedMessage, sizeof(formattedMessage), "2 %lu %s %s\n", time_us_32()/1000, cat, message);

    if (xQueueSend(loggerQueue, formattedMessage, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        // Handle queue full error (optional)
        printf("message dropped");
    }
}
void Log::warn(const char* cat, const char *format, ...)
{
    char message[QUEUE_ITEM_SIZE];
    char formattedMessage[QUEUE_ITEM_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    snprintf(formattedMessage, sizeof(formattedMessage), "3 %lu %s %s\n", time_us_32()/1000, cat, message);

    if (xQueueSend(loggerQueue, formattedMessage, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        // Handle queue full error (optional)
        printf("Logger queue is full. Dropping message");
    }
}
void Log::debug(const char* cat, const char *format, ...)
{
    char message[QUEUE_ITEM_SIZE];
    char formattedMessage[QUEUE_ITEM_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    snprintf(formattedMessage, sizeof(formattedMessage), "4 %lu %s %s\n", time_us_32()/1000, cat, message);

    if (xQueueSend(loggerQueue, formattedMessage, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        // Handle queue full error (optional)
        printf("Logger queue is full. Dropping message");
    }
}
void Log::error(const char* cat, const char *format, ...)
{
    char message[QUEUE_ITEM_SIZE];
    char formattedMessage[QUEUE_ITEM_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    
    snprintf(formattedMessage, sizeof(formattedMessage), "5 %lu %s %s\n", time_us_32()/1000, cat, message);
    printf("%s", formattedMessage);
    fflush(stdout);
    if (xQueueSend(loggerQueue, formattedMessage, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        // Handle queue full error (optional)
        printf("Logger queue is full. Dropping message");
    }
}

void Log::logger_task(void *pvParameters)
{
    char logMessage[100]; // Buffer to hold the received message

    while (1)
    {
        // Wait indefinitely for a message from the queue
        if (xQueueReceive(loggerQueue, logMessage, portMAX_DELAY) == pdTRUE)
        {
            printf("%s", logMessage);
        }
    }
}


void Log::start()
{
        loggerQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
        if (loggerQueue == NULL) {
    // Queue creation failed
            printf("Queue creation failed.\n");
        } else {
            // Queue created successfully
            printf("Queue created successfully.\n");
        }

        if(xTaskCreate(logger_task, "LoggerTask", 256, NULL, 2, &loggerTaskHandle) != pdPASS)
        {
            printf("Task creation failed.\n");
        }
        else
        {
            printf("Logger has started\n");
        }


}


bool Log::isLoggerTaskRunning()
{
    if (loggerTaskHandle == NULL)
    {
        return false; // Task handle is not initialized
    }

    eTaskState state = eTaskGetState(loggerTaskHandle);
    return (state == eRunning || state == eReady || state == eBlocked || state == eSuspended);
}