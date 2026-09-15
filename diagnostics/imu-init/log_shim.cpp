// The console channel for a bare-metal diagnostic.
//
// The firmware's channel (src/app/log.cpp) drains a ring to uart1 from a
// FreeRTOS task and enforces IF-0002's floors, suppression and token buckets.
// This program has no scheduler and wants none of that policy: it exists to
// make the driver's own error strings -- the ones naming the failing check --
// the output of the program, so every line goes straight to stdout unfiltered.
//
// Stubbing the channel rather than stripping the LOG_* calls is what keeps
// imu_i2c.cpp and lsm6dsox.cpp byte-identical to the ones the firmware runs.

#include <stdarg.h>
#include <stdio.h>

#include "app/log.hpp"

namespace
{
void emit_stdout(char kind, const char* cat, const char* fmt, va_list args)
{
    printf("%c %s ", kind, cat ? cat : "?");
    vprintf(fmt, args);
    printf("\n");
}
} // namespace

namespace log_console
{

bool init()
{
    return true; // stdout is already up.
}

void emit(log_policy::Level level, const char* cat, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    emit_stdout(log_policy::levelChar(level), cat, fmt, args);
    va_end(args);
}

void snapshot(const char* cat, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    emit_stdout('S', cat, fmt, args);
    va_end(args);
}

void boot(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    emit_stdout('B', "boot", fmt, args);
    va_end(args);
}

// fatal() is an inline wrapper in the header and must NOT be defined here.
void blocking(log_policy::Level level, const char* cat, const char* message)
{
    printf("%c %s %s\n", log_policy::levelChar(level), cat, message);
}

DrainResult drain()
{
    return {0, DrainStop::ring_empty};
}

uint32_t expireLevels(uint32_t)
{
    return 0;
}

Counters counters()
{
    return Counters{};
}

log_policy::Level floorOf(log_policy::Cat)
{
    return log_policy::Level::trace;
}

log_policy::Level buildDefaultLevel()
{
    return log_policy::Level::trace;
}

} // namespace log_console
