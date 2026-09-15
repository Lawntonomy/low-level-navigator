#include "logger.h"

#include <cstdarg>
#include <cstdio>

#include "app/log.hpp"
#include "pico/time.h"

// Log:: is an ADAPTER over log_console, not a second logging channel.
//
// It used to format inline and call printf() in the caller's context. Three
// things were wrong with that, and the first hid the others:
//
//   1. printf reached no driver. Both stdio backends are disabled (TP-0001 D5),
//      so all of these call sites were silently discarded -- which is why the
//      IMU driver could name its failing check and a bench session still could
//      not find out what it was.
//   2. It never deferred. D5 specified producer -> ring -> low-priority drain
//      task, and the queue and task here were scaffolding: xQueueSend was
//      commented out, so the task blocked forever on an empty queue.
//   3. `printf(formattedMessage)` passed a non-literal as the format string, so
//      any '%' in a logged message was undefined behaviour. Routing through
//      log_console::write(), which carries
//      __attribute__((format(printf, 1, 2))), makes that unrepresentable rather
//      than merely fixed.
//
// The strategy, now in one place: **uart1 (GP20) carries every log and
// diagnostic this device emits.** log_console owns the ring, the drop counter
// and the drain task; this file only formats a line and hands it over. Nothing
// chooses a sink at a call site.
//
// **Not callable from an interrupt handler.** log_console::push() takes a
// FreeRTOS critical section, and rt.h puts the INT1 handler above
// configMAX_SYSCALL_INTERRUPT_PRIORITY where no kernel API may be called.
// Handlers expose counters instead; imu_drdy does exactly that.

namespace
{

// One line. Long enough for a register dump, short enough to sit on the stack of
// a task with a 512-word allocation.
constexpr size_t line_max = 160;

void emit(char level, const char* cat, const char* format, va_list args)
{
    char message[line_max];
    vsnprintf(message, sizeof(message), format, args);

    // Milliseconds since boot, then level, then category. Same shape the old
    // implementation used, so existing log-reading habits still work.
    log_console::write("%c %lu %s %s\r\n", level, (unsigned long)(time_us_64() / 1000u), cat,
                       message);
}

} // namespace

#define LOG_LEVEL(name, level_char)                                                                \
    void Log::name(const char* cat, const char* format, ...)                                       \
    {                                                                                              \
        va_list args;                                                                              \
        va_start(args, format);                                                                    \
        emit(level_char, cat, format, args);                                                       \
        va_end(args);                                                                              \
    }

// Single characters rather than words: these go out a 115200 baud line, and the
// level is the least interesting part of any line that also carries a category.
LOG_LEVEL(trace, 'T')
LOG_LEVEL(debug, 'D')
LOG_LEVEL(info, 'I')
LOG_LEVEL(warn, 'W')
LOG_LEVEL(error, 'E')

void Log::start()
{
    // Nothing to start. log_console::init() brings up the UART and main.cpp's
    // logger_task drains the ring at rt::prio_logger on core 0. Kept so the
    // call in any existing startup path stays valid.
}
