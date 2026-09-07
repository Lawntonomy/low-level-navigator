#pragma once

// Verbose runtime logging over USB CDC (bulk channel, IF-0001 SS2 / ADR-0003
// "USB carries verbose logging, diagnostics, and firmware flashing").
//
// --- Why this cannot just call printf from wherever ------------------------
//
// stdio_usb_out_chars() (pico-sdk src/rp2_common/pico_stdio_usb/stdio_usb.c)
// has two blocking paths: it waits for the host to keep reading, up to
// PICO_STDIO_USB_STDOUT_TIMEOUT_US (CMakeLists.txt sets this to 0, so an
// unread port drops instead of waiting there); and separately it takes
// stdio_usb_mutex with mutex_try_enter_block_until(...,
// PICO_STDIO_DEADLOCK_TIMEOUT_MS), which the 0 setting does NOT touch and
// which the SDK defaults to 1000 ms. So the write cannot be made fully
// non-blocking from where it is called -- the fix has to be about WHICH task
// is allowed to call it.
//
// --- The design -------------------------------------------------------------
//
// Log::trace/info/warn/debug/error format a line and push it whole into
// log_detail::Ring -- a drop-on-full ring buffer, matching the shape of
// src/app/link.cpp's TX ring and src/app/log.cpp's console ring: a message is
// pushed whole or dropped whole, never truncated, and a counter tracks how
// many were dropped. These calls never call printf and never block.
//
// A single low-priority FreeRTOS task, created by Log::start(), drains the
// ring and is the ONLY code in this firmware that calls printf. Blocking
// there for up to ~1.5 s (the two timeouts above, combined) is harmless BY
// CONSTRUCTION: this is not the control task (src/app/rt.h pins that alone on
// core 1 at the highest application priority) and it does not feed the
// watchdog (fed only by control_task in main.cpp) -- so a USB host that opens
// the port and stops reading stalls diagnostics, not the machine.
//
// This REPLACES the FreeRTOS queue this file used to build (a loggerQueue
// plus a draining task) rather than running alongside it: that queue was
// created and drained on its own task, but nothing ever pushed to it --
// every level function called printf() directly instead, bypassing it
// entirely. Keeping both would have left two half-wired logging systems: one
// ring, one drain path, is the whole of it now.
//
// log_console (src/app/log.hpp/.cpp) is a SEPARATE, unframed uart1 console
// that exists to be readable before any stack is up and during a panic. This
// is complementary to it, not a replacement -- see log_console's own header
// for why it stays.

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

namespace log_detail
{

// Fixed-capacity, byte-oriented ring buffer with whole-message push/drop
// discipline.
//
// Deliberately free of the Pico SDK and FreeRTOS so it compiles into the host
// test build (test/test_logger_ring.cpp exercises it directly, the way
// src/hardware_drivers/encoder_math.hpp and bootloader.hpp's judge() do).
//
// Concurrency is NOT this class's job: push() and pop() assume they are not
// called concurrently with each other or with themselves. logger.cpp
// serialises access with a FreeRTOS critical section around each call --
// exactly how src/app/link.cpp's push_unsafe() and src/app/log.cpp's push()
// split the same concern between pure array logic and the lock around it.
template <uint32_t Capacity> class Ring
{
    static_assert(Capacity >= 2, "ring needs room for at least one byte of headroom");
    static_assert((Capacity & (Capacity - 1)) == 0, "ring must be a power of two");

  public:
    // Pushes n bytes as a single unit. A message that does not fit is
    // dropped whole and counted, never partially written -- a truncated line
    // in a diagnostic stream reads as corruption rather than as loss (see
    // src/app/log.cpp's push()).
    bool push(const char* data, uint32_t n)
    {
        if (n > mask - used())
        {
            drop_count_++;
            return false;
        }
        for (uint32_t i = 0; i < n; i++)
        {
            buffer_[(head_ + i) & mask] = data[i];
        }
        head_ = (head_ + n) & mask;
        if (used() > peak_)
        {
            peak_ = used();
        }
        return true;
    }

    // Pops a single byte into *out. False, and *out untouched, if empty.
    bool pop(char* out)
    {
        if (used() == 0)
        {
            return false;
        }
        *out = buffer_[tail_];
        tail_ = (tail_ + 1) & mask;
        return true;
    }

    uint32_t used() const
    {
        return (head_ - tail_) & mask;
    }

    uint32_t capacity() const
    {
        return Capacity;
    }

    // Messages dropped because the ring was full, since construction.
    uint32_t dropped() const
    {
        return drop_count_;
    }

    // High-water mark of ring occupancy in bytes, for sizing.
    uint32_t peak_bytes() const
    {
        return peak_;
    }

  private:
    static constexpr uint32_t mask = Capacity - 1;
    char buffer_[Capacity]{};
    uint32_t head_ = 0;
    uint32_t tail_ = 0;
    uint32_t drop_count_ = 0;
    uint32_t peak_ = 0;
};

// 2048 bytes: the same order as link.cpp's command-link TX ring. Verbose
// logging is bursty (a boot sequence, a fault) rather than steady, so this is
// sized for a burst of ~10-20 lines to survive one scheduling latency of the
// drain task, not for sustained throughput -- peak_bytes() is exposed so that
// guess can be replaced with a measurement.
constexpr uint32_t ring_capacity = 2048;

} // namespace log_detail

class Log
{
  public:
    static void trace(const char* cat, const char* format, ...);
    static void info(const char* cat, const char* format, ...);
    static void warn(const char* cat, const char* format, ...);
    static void debug(const char* cat, const char* format, ...);
    static void error(const char* cat, const char* format, ...);

    // Creates the single logger task that drains the ring to USB CDC.
    //
    // Safe to call before vTaskStartScheduler() -- xTaskCreate() only needs
    // the kernel's data structures, not a running scheduler -- and safe to
    // call trace/info/warn/debug/error before this: they only need the ring,
    // which exists independently of the task, so nothing logged before
    // start() is lost unless the ring fills first.
    // Creates the task that drains the ring to USB CDC. Must be called once,
    // after the console is up and before the scheduler starts.
    //
    // Takes its task parameters rather than reading them, because this header
    // is compiled into the host tests and must stay free of the Pico SDK and
    // FreeRTOS. src/app/rt.h owns the priority and stack table for every task
    // in the firmware, with the rationale beside each; main() passes the
    // entries from there so this one is not the exception that lives somewhere
    // else. Plain types for the same reason.
    static void start(unsigned priority, unsigned stack_words);

    // Messages dropped because the ring was full, since boot. Not yet wired
    // to telemetry (that would touch src/app/link.cpp, out of scope here);
    // exposed so a caller that IS allowed to touch it can report it the way
    // link.cpp already folds tx_drop and log_console::dropped() together.
    static uint32_t dropped();

    // High-water mark of ring occupancy in bytes, for sizing the ring.
    static uint32_t peak_bytes();

  private:
    static void logger_task(void* pvParameters);
    static bool isLoggerTaskRunning();
    const char* category;
    static bool log_started;
};
