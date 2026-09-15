// Host-side stub for the console channel.
//
// The real channel (src/app/log.cpp) depends on the Pico SDK's UART and on
// FreeRTOS critical sections, neither of which exists on the host. Unit tests
// link this implementation instead so that production code under test keeps its
// LOG_* calls unchanged, rather than having the calls stripped out of
// production code to make it testable.
//
// Set LLN_TEST_LOG_TO_STDERR=1 in the environment to see log output while
// debugging a failing test.
//
// **The policy itself is not stubbed.** log_policy.hpp is SDK-free and is
// compiled and tested directly by test_log_policy.cpp -- the floors, the
// suppressor and the token bucket are exercised for real. What this file
// replaces is only the transport.

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "app/log.hpp"

namespace
{
bool log_to_stderr()
{
    static const bool enabled = getenv("LLN_TEST_LOG_TO_STDERR") != nullptr;
    return enabled;
}

void emit_stderr(char kind, const char* cat, const char* fmt, va_list args)
{
    if (!log_to_stderr())
    {
        return;
    }
    fprintf(stderr, "%c %s ", kind, cat);
    vfprintf(stderr, fmt, args);
    fputc('\n', stderr);
}
} // namespace

namespace log_console
{

bool init()
{
    return true;
}

void emit(log_policy::Level level, const char* cat, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    emit_stderr(log_policy::levelChar(level), cat, fmt, args);
    va_end(args);
}

void snapshot(const char* cat, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    emit_stderr('S', cat, fmt, args);
    va_end(args);
}

void boot(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    emit_stderr('B', "boot", fmt, args);
    va_end(args);
}

// fatal() is an inline wrapper in the header and must NOT be defined here.
void blocking(log_policy::Level level, const char* cat, const char* message)
{
    if (log_to_stderr())
    {
        fprintf(stderr, "%c %s %s\n", log_policy::levelChar(level), cat, message);
    }
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
    return log_policy::Level::debug;
}

log_policy::Level buildDefaultLevel()
{
    return log_policy::Level::debug;
}

} // namespace log_console
