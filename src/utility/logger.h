#pragma once

// Levelled, categorised logging. A thin adapter over log_console -- see
// logger.cpp for why it is an adapter and not a channel of its own.
//
// **uart1 (GP20, 115200 8N1) carries every log and diagnostic this device
// emits.** There is no second sink to choose between at a call site.
//
// Never call these from an interrupt handler: the underlying ring takes a
// FreeRTOS critical section, and rt.h puts the INT1 handler above
// configMAX_SYSCALL_INTERRUPT_PRIORITY where no kernel API is legal. Handlers
// expose counters for a task to read instead.

class Log
{
  public:
    static void trace(const char* cat, const char* format, ...);
    static void info(const char* cat, const char* format, ...);
    static void warn(const char* cat, const char* format, ...);
    static void debug(const char* cat, const char* format, ...);
    static void error(const char* cat, const char* format, ...);
    static void start();
};