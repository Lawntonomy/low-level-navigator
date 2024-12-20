#pragma once
#include <stdio.h>
#include <stdarg.h>
#include <string>

class Log
{
    public:
        static void trace(const char* cat, const char *format, ...);
        static void info(const char* cat, const char *format, ...);
        static void warn(const char* cat, const char *format, ...);
        static void debug(const char* cat, const char *format, ...);
        static void error(const char* cat, const char *format, ...);
        static void start();
    private:
        static void logger_task(void *pvParameters);
        static bool isLoggerTaskRunning();
        const char* category;
        static bool log_started;
};