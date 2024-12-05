#pragma once
#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"

static uint64_t log_count;

static void log_trace(const char *format, ...)
{
    printf("1 %ld ", log_count++);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    printf("\n");
}
static void log_info(const char *format, ...)
{
    printf("2 %ld ", log_count++);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
static void log_warn(const char *format, ...)
{
    printf("3 %ld ", log_count++);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
static void log_debug(const char *format, ...)
{
    printf("4 %ld ", log_count++);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
static void log_error(const char *format, ...)
{
    printf("5 %ld ", log_count++);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
