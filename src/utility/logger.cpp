#include "logger.h"

#include <cstdio>

#include "FreeRTOS.h"
#include "pico/time.h"
#include "task.h"

namespace
{

log_detail::Ring<log_detail::ring_capacity> log_ring;

TaskHandle_t loggerTaskHandle = NULL;

// Longest formatted line, including the "<level> <ms> <cat> " prefix, the
// caller's message, the trailing '\n' and the snprintf nul. Matches
// src/app/log.cpp's line[192] for the same reason: long enough for a real
// message, short enough to sit on the stack of whichever task is logging.
constexpr uint32_t max_line_len = 200;

// Bytes drained and handed to printf() in one call. Larger than one line so a
// burst does not cost one printf() call per byte; bounded so the logger task
// cannot monopolise its core if a producer fills the ring as fast as this
// empties it -- mirrors log_console::drain()'s per-call cap in main.cpp.
constexpr uint32_t drain_batch_len = 512;

// Formats "<level> <uptime_ms> <cat> <message>\n" and pushes it whole into
// the ring. Never calls printf; safe from any task, on either core. NOT safe
// from an ISR -- nothing on the safety path should be logging from interrupt
// context anyway (see log_console::write()'s equivalent note).
void log_line(int level, const char* cat, const char* format, va_list args)
{
    char message[max_line_len];
    vsnprintf(message, sizeof(message), format, args);

    char line[max_line_len];
    const int n = snprintf(line, sizeof(line), "%d %lu %s %s\n", level,
                           static_cast<unsigned long>(time_us_32() / 1000), cat, message);
    if (n <= 0)
    {
        return;
    }

    // snprintf returns the length it WOULD have written; clamp so a line that
    // ran long is truncated rather than read past the buffer.
    const uint32_t len =
        (static_cast<uint32_t>(n) >= sizeof(line)) ? sizeof(line) - 1 : static_cast<uint32_t>(n);

    // The region is a bounded memcpy of at most one line, so there is nothing
    // to gain from a mutex here, and a mutex would risk priority inversion on
    // a path that must never delay whichever task is logging -- matching
    // log_console::push()'s reasoning in src/app/log.cpp.
    taskENTER_CRITICAL();
    log_ring.push(line, len);
    taskEXIT_CRITICAL();
}

} // namespace

void Log::trace(const char* cat, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log_line(1, cat, format, args);
    va_end(args);
}

void Log::info(const char* cat, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log_line(2, cat, format, args);
    va_end(args);
}

void Log::warn(const char* cat, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log_line(3, cat, format, args);
    va_end(args);
}

void Log::debug(const char* cat, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log_line(4, cat, format, args);
    va_end(args);
}

void Log::error(const char* cat, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log_line(5, cat, format, args);
    va_end(args);
}

void Log::logger_task(void* /*pvParameters*/)
{
    for (;;)
    {
        char batch[drain_batch_len];
        uint32_t n = 0;

        taskENTER_CRITICAL();
        while (n < sizeof(batch) - 1 && log_ring.pop(&batch[n]))
        {
            n++;
        }
        taskEXIT_CRITICAL();

        if (n == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(5)); // idle: nothing queued
            continue;
        }

        batch[n] = '\0';

        // The ONLY call to printf in this firmware -- see the "why this
        // cannot just call printf from wherever" note in logger.h. Blocking
        // here for up to ~1.5 s if the host has stopped reading is
        // acceptable BECAUSE this task is not the control task and does not
        // feed the watchdog: a stuck USB host stalls diagnostics, not the
        // machine.
        //
        // "%s", not the batch itself: the batch is a runtime string built
        // from caller-supplied messages, and using it as the format would be
        // undefined behavior the moment a message contains '%'.
        printf("%s", batch);
    }
}

void Log::start()
{
    // Priority 1: one above idle, intentionally lower than any application
    // task. This mirrors rt::prio_logger's rationale in src/app/rt.h --
    // logging is diagnostic, never required for safe operation (ADR-0003),
    // and should be the first thing that starves -- but is defined locally
    // rather than by including rt.h, which lives in src/app/ and is out of
    // scope for this change. Worth consolidating into rt.h later.
    constexpr UBaseType_t logger_task_priority = 1;
    constexpr configSTACK_DEPTH_TYPE logger_task_stack = 512;

    xTaskCreate(logger_task, "log_usb", logger_task_stack, NULL, logger_task_priority,
                &loggerTaskHandle);
}

uint32_t Log::dropped()
{
    return log_ring.dropped();
}

uint32_t Log::peak_bytes()
{
    return log_ring.peak_bytes();
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
