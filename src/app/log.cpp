#include "app/log.hpp"

#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "task.h"

#include "app/board.h"

namespace log_console
{
namespace
{

constexpr uint32_t ring_bytes = 4096; // power of two
constexpr uint32_t ring_mask = ring_bytes - 1;
static_assert((ring_bytes & ring_mask) == 0, "ring must be a power of two");

// Producers on either core plus one consumer. Guarded by a critical section
// rather than a mutex: the region is a bounded memcpy of at most one line, so
// there is nothing to gain from blocking and a mutex would introduce priority
// inversion on a path that must never delay anyone.
char buffer[ring_bytes];
volatile uint32_t head;
volatile uint32_t tail;
volatile uint32_t drop_count;
volatile uint32_t peak;

inline uint32_t used_unsafe()
{
    return (head - tail) & ring_mask;
}

void push(const char *src, uint32_t n)
{
    taskENTER_CRITICAL();
    const uint32_t space = ring_mask - used_unsafe();
    if (n > space)
    {
        // Drop the whole line. A truncated line in a diagnostic stream is
        // worse than an absent one: it reads as corruption rather than loss.
        drop_count++;
        taskEXIT_CRITICAL();
        return;
    }
    for (uint32_t i = 0; i < n; i++)
    {
        buffer[(head + i) & ring_mask] = src[i];
    }
    head = (head + n) & ring_mask;
    if (used_unsafe() > peak)
    {
        peak = used_unsafe();
    }
    taskEXIT_CRITICAL();
}

} // namespace

void init()
{
    uart_init(board::console_uart(), board::console_baud);
    gpio_set_function(board::console_tx_pin, GPIO_FUNC_UART);
    // TX only. Nothing is expected to talk back on the console, and leaving RX
    // unclaimed keeps the pin available.
}

void write_blocking(const char *s)
{
    while (*s)
    {
        uart_putc_raw(board::console_uart(), *s++);
    }
}

void write(const char *fmt, ...)
{
    // On the stack, not static: two tasks on two cores may be here at once,
    // and a shared scratch buffer would interleave their output.
    char line[192];

    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return;
    }
    // vsnprintf returns the length it *would* have written; clamp so a long
    // line is truncated rather than read past the buffer.
    const uint32_t len = (static_cast<uint32_t>(n) >= sizeof line)
                             ? sizeof line - 1
                             : static_cast<uint32_t>(n);
    push(line, len);
}

uint32_t drain()
{
    uint32_t written = 0;

    // Bounded per call so the logger task cannot monopolise its core if a
    // producer is filling the ring as fast as we empty it.
    while (written < 256)
    {
        char c;
        taskENTER_CRITICAL();
        if (used_unsafe() == 0)
        {
            taskEXIT_CRITICAL();
            break;
        }
        c = buffer[tail];
        tail = (tail + 1) & ring_mask;
        taskEXIT_CRITICAL();

        // Outside the critical section: this can spin if the console UART is
        // busy, and holding a lock across it would stall producers.
        uart_putc_raw(board::console_uart(), c);
        written++;
    }
    return written;
}

uint32_t dropped()
{
    return drop_count;
}

uint32_t peak_bytes()
{
    return peak;
}

} // namespace log_console
