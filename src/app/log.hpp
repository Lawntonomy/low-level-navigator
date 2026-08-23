#pragma once

// Non-blocking console logging.
//
// ADR-0003: logging is diagnostic and must never be required for safe
// operation, which means it must never block the control loop. The existing
// firmware couples its loop period to printf over stdio; this exists so the
// rewrite does not.
//
// Producers format into a byte ring and return. One low-priority task drains
// it to the console UART. When the ring is full, the *newest* message is
// dropped and a counter increments — the drop count is reported over the
// command link, never only on the channel that is dropping.
//
// The console is deliberately unframed and not MAVLink: its purpose is to be
// readable before anything is initialised, during a panic, and while the Pi is
// absent. See TP-0001 D1.

#include <cstdarg>
#include <cstdint>

namespace log_console
{

// Brings up the console UART. Safe to call before the scheduler starts, and
// intended to be the first thing main() does so that any later failure is
// reportable.
void init();

// Formats and enqueues. Never blocks, never allocates. Safe from any task.
// NOT safe from an ISR — nothing on the safety path should be logging from
// interrupt context anyway.
void write(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Bypasses the ring and writes straight to the UART, blocking until the bytes
// are out. For boot messages before the scheduler runs, and for panics where
// the drain task will never get another chance to run. Do not call from the
// control loop.
void write_blocking(const char *s);

// Drains one batch to the UART. Called by the logger task; returns the number
// of bytes written so the caller can decide whether to yield.
uint32_t drain();

// Messages dropped because the ring was full, since boot. Monotonic.
uint32_t dropped();

// High-water mark of ring occupancy in bytes, for sizing.
uint32_t peak_bytes();

} // namespace log_console
