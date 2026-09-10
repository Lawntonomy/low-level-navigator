#pragma once

// The LSM6DSOX data-ready path: the INT1 interrupt handler, the RX DMA
// completion handler, and the two hardware spinlocks that carry a sample from
// core 0 to core 1.
//
// **There is no task here, and that is the safety argument, not an omission.**
// Data-ready is latched, so the burst read is what re-arms INT1: a read that
// does not happen does not lower the sample rate, it stops the stream
// permanently with no further edge to notice. An ISR cannot be starved by a
// task at any priority and a DMA channel cannot be starved at all, so nothing
// schedulable sits in the re-arm loop. The full reasoning, the priorities, the
// arming order and the three lock rules are in src/app/rt.h; this file
// implements them and does not restate them where they have not moved.
//
// Split the way CLAUDE.md asks: everything here needs the SDK, so all of the
// arithmetic worth testing -- decoding, gap accounting across a publish cycle --
// lives in imu_sample.hpp and is exercised on the host.

#include "imu_sample.hpp"

#include <cstdint>

namespace imu_drdy
{

// Diagnostic counters. Every field is a thing that is otherwise invisible: the
// stream simply produces fewer samples, or none, and nothing says why.
struct Counters
{
    uint32_t edges = 0;                 // INT1 rising edges handled
    uint32_t burst_lock_misses = 0;     // try-lock failed repeatedly: reservation lost
    uint32_t burst_lock_contended = 0;  // lock genuinely held by core 1's recovery read
    uint32_t burst_start_failures = 0;  // startBurstRead() refused
    uint32_t completion_discards = 0;   // completion could not be paired with its stamp
    uint32_t duplicate_completions = 0; // completion for an already-published stamp
    uint32_t published = 0;             // samples published into the slot
};

// Claims the two spinlocks, installs both handlers, sets their priorities and
// enables them.
//
// **Must be called from main() on core 0, before vTaskStartScheduler(), and
// after imu_i2c::init() has succeeded.** Interrupt affinity on RP2350 is
// decided by where the arming call runs -- see rt.h -- so calling this from
// anywhere else silently moves both interrupts onto the core that owns the
// motors.
//
// Does NOT start the stream. Nothing reaches the INT1 pin until
// lsm6dsox::enableDataReadyInterrupt() writes INT1_CTRL, which is deliberately
// the last thing main() does before the scheduler.
bool init();

// Takes the latest sample and the gap accumulated since the previous take().
//
// Returns false, writing nothing, until the first sample has landed. Safe to
// call from core 1: it holds imu_sample_lock for a struct copy.
bool takeSample(imu_sample::Sample* out);

// Reads the latest sample WITHOUT consuming the gap, for a staleness check that
// only wants the timestamp.
bool peekSample(imu_sample::Sample* out);

// Lock-free. Safe to call from any task on either core; NOT from an interrupt
// handler above configMAX_SYSCALL_INTERRUPT_PRIORITY, though nothing needs to.
Counters counters();

} // namespace imu_drdy
