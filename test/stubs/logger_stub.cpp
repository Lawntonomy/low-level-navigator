// Host-side stub for the Log class.
//
// The real logger (src/utility/logger.cpp) depends on FreeRTOS queues and
// pico/time.h, neither of which exist on the host. Unit tests link this
// no-op implementation instead so that production code under test can keep
// its Log:: calls unchanged.
//
// Set LLN_TEST_LOG_TO_STDERR=1 in the environment to see log output while
// debugging a failing test.

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "utility/logger.h"

namespace
{
bool log_to_stderr()
{
    static const bool enabled = getenv("LLN_TEST_LOG_TO_STDERR") != nullptr;
    return enabled;
}

void emit(int level, const char* cat, const char* format, va_list args)
{
    if (!log_to_stderr())
    {
        return;
    }
    fprintf(stderr, "%d %s ", level, cat);
    vfprintf(stderr, format, args);
    fputc('\n', stderr);
}
} // namespace

#define LLN_DEFINE_LOG_LEVEL(name, level)                                                          \
    void Log::name(const char* cat, const char* format, ...)                                       \
    {                                                                                              \
        va_list args;                                                                              \
        va_start(args, format);                                                                    \
        emit(level, cat, format, args);                                                            \
        va_end(args);                                                                              \
    }

LLN_DEFINE_LOG_LEVEL(trace, 1)
LLN_DEFINE_LOG_LEVEL(info, 2)
LLN_DEFINE_LOG_LEVEL(warn, 3)
LLN_DEFINE_LOG_LEVEL(debug, 4)
LLN_DEFINE_LOG_LEVEL(error, 5)

#undef LLN_DEFINE_LOG_LEVEL

void Log::start(unsigned /*priority*/, unsigned /*stack_words*/)
{
}

bool Log::log_started = false;
