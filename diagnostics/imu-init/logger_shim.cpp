// Log:: for a bare-metal diagnostic. The real src/utility/logger.cpp is a
// FreeRTOS task draining a ring to a backend that is disabled in the firmware
// build, which is precisely why the firmware cannot say what went wrong. Here
// the same calls go straight to stdout, so the driver's own error strings -- the
// ones naming the failing check -- become the output of this program.
#include <stdarg.h>
#include <stdio.h>
#include "utility/logger.h"

static void emit(const char* level, const char* cat, const char* format, va_list args)
{
    printf("[%s] %s: ", level, cat);
    vprintf(format, args);
    printf("\n");
}

#define SHIM(name, label)                                                                          \
    void Log::name(const char* cat, const char* format, ...)                                       \
    {                                                                                              \
        va_list args;                                                                              \
        va_start(args, format);                                                                    \
        emit(label, cat, format, args);                                                            \
        va_end(args);                                                                              \
    }

SHIM(trace, "trace")
SHIM(info, "info")
SHIM(warn, "warn")
SHIM(debug, "debug")
SHIM(error, "ERROR")

void Log::start()
{
    // No-op: there is no task to start here, and stdout is already up.
}
