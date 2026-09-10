#include "imu_drdy.hpp"

#include "app/rt.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "imu_i2c.hpp"
#include "imu_stall.hpp"
#include "pico/stdlib.h"
#include "utility/logger.h"

#include <cstddef>

namespace
{

constexpr const char* category = "imu_drdy";

// LSM6DSOX INT1. Wiring confirmed on hardware 2026-09-09 by
// diagnostics/imu-probe (issue #44) and reserved in gpio_defines.h. Kept local
// for the same reason imu_i2c.cpp keeps GP12/GP13 local: the pin belongs to the
// driver that owns the peripheral, not to a shared enum of motor pins.
constexpr uint int1_pin = 14;

static_assert(imu_sample::burst_bytes == imu_i2c::burst_bytes,
              "the sample decoder and the transport must agree on the burst length");

// The cadence the gap accounting divides by. lsm6dsox.cpp configures 208 Hz in
// CTRL1_XL/CTRL2_G and imu_stall.hpp derives its numbers from the same 208; the
// two are not mechanically tied and imu_stall.hpp says so. If the ODR changes,
// both move.
constexpr uint32_t sample_period_us = imu_stall::nominal_period_us;

// ---------------------------------------------------------------------------
// State shared by the two handlers
//
// Single writer (INT1), single reader (DMA completion), both on core 0, and
// INT1 sits strictly above the completion handler in NVIC priority -- so the
// writer can preempt the reader and never the reverse, and core 1 touches
// neither. That asymmetry is what lets the reader validate its snapshot with a
// counter instead of a lock, which matters because the obvious lock is the
// wrong one: save_and_disable_interrupts() sets PRIMASK, and PRIMASK masks even
// priority 0x00 -- the one property this priority scheme exists to preserve.
//
// volatile so the compiler neither hoists nor merges these loads. It is not
// standing in for a memory barrier; the explicit __dmb() calls do that.
// ---------------------------------------------------------------------------

volatile uint64_t latched_stamp_us = 0;

// Incremented once per burst actually started. The completion handler reads it
// either side of its copy: if it moved, INT1 ran in between and the stamp and
// the bytes may belong to different samples.
volatile uint32_t bursts_started = 0;

volatile uint32_t edges = 0;
volatile uint32_t burst_lock_misses = 0;
volatile uint32_t burst_start_failures = 0;
volatile uint32_t completion_discards = 0;

// Two dedicated hardware spinlocks, claimed at init and never released.
//
//   sample_lock -- the published slot. Written by the completion handler on
//                  core 0, read by the control task on core 1.
//   burst_lock  -- serialises imu_i2c::startBurstRead() between the INT1 ISR on
//                  core 0 and the control task's forced recovery read on core 1
//                  (that reader is issue #46's remainder and does not exist
//                  yet; the lock does, because adding it afterwards means
//                  auditing this file again).
//
// Deliberately not taskENTER_CRITICAL: the writer is an interrupt handler that
// runs before the scheduler exists, INT1 at 0x00 may make no kernel call at
// all, and a 208 Hz writer on the kernel's shared SIO pair would add exactly
// the core-1 coupling rt.h's Cores note is trying to keep small.
spin_lock_t* sample_lock = nullptr;
spin_lock_t* burst_lock = nullptr;

imu_sample::SampleStream stream;

bool armed = false;

// ---------------------------------------------------------------------------
// INT1, GP14 rising edge, rt::irq_prio_imu_int1 == 0x00
//
// **No FreeRTOS API and no floating point in here or in anything it calls.**
// 0x00 is above configMAX_SYSCALL_INTERRUPT_PRIORITY, so a kernel call from
// here trips configASSERT, and this project's configASSERT is a cpsid-and-spin:
// the failure mode is a wedged core 0, not a message.
//
// Latching the timestamp without starting the read would be worse than doing
// nothing. Data-ready is latched in the sensor, so the burst read is what
// re-arms INT1; an ISR that stamps and defers the start produces exactly one
// edge and then a permanently dead stream.
// ---------------------------------------------------------------------------
void int1Handler()
{
    // FIRST statement. Everything after it adds pipeline latency before the
    // sample is visible, and none of it contaminates the stamp -- which is the
    // whole reason IF-0001 section 7.4's +-2 us GPIO-IRQ-at-the-pin budget can
    // be met by a handler that also drives an I2C burst.
    //
    // **time_us_64(), never time_us_32().** A 32-bit microsecond stamp wraps
    // every 71.6 minutes. Past the wrap the staleness check would read either a
    // multi-thousand-second age -- an instant false stall on a perfectly
    // healthy stream -- or, landing on the other side of
    // imu_stall::isStalled()'s nowUs <= lastSampleUs guard, a false healthy on
    // a stream that has stopped. time_us_64() reads TIMERAWH/TIMERAWL in a
    // retry loop with no lock, no kernel call and no float (pico-sdk timer.c),
    // which is what makes it legal at this priority.
    const uint64_t stamp_us = time_us_64();

    // The SDK dispatches IO_IRQ_BANK0 through a shared handler chain, so this
    // is called for every bank 0 event, not only ours.
    if ((gpio_get_irq_event_mask(int1_pin) & GPIO_IRQ_EDGE_RISE) == 0)
    {
        return;
    }
    gpio_acknowledge_irq(int1_pin, GPIO_IRQ_EDGE_RISE);

    edges = edges + 1;

    // TRY-lock, never a spin -- rt.h rule (a), and it is deadlock avoidance
    // rather than tidiness. INT1 at 0x00 can preempt a core 0 task sitting
    // inside a FreeRTOS critical section holding the kernel spinlock; spinning
    // here for a lock held by core 1 would deadlock both cores. A miss means
    // the control task is issuing its forced recovery read at this instant, and
    // that read re-arms latched DRDY just as well, so skipping loses nothing.
    //
    // The _unsafe form because it must NOT disable interrupts: spin_lock_
    // blocking() sets PRIMASK, which would mask priority 0x00 itself. Nothing
    // on core 0 can preempt a 0x00 handler, so there is nothing local to
    // protect against anyway.
    if (!spin_try_lock_unsafe(burst_lock))
    {
        burst_lock_misses = burst_lock_misses + 1;
        return;
    }

    // Non-blocking: pushes 13 command words into a 16-deep TX FIFO, arms the RX
    // channel, returns. The bytes land later without CPU involvement, which is
    // what makes a burst read affordable at this priority.
    //
    // A false return leaves DRDY asserted and therefore stops the stream until
    // the forced recovery read clears it. That is loud rather than silent by
    // construction -- there is no such thing here as a stream that keeps
    // running with stale contents -- and it is counted.
    if (imu_i2c::startBurstRead())
    {
        // Order is the reader's entire guarantee: the stamp is written BEFORE
        // the counter it will be validated against. See dmaHandler().
        latched_stamp_us = stamp_us;
        __dmb();
        bursts_started = bursts_started + 1;
    }
    else
    {
        burst_start_failures = burst_start_failures + 1;
    }

    spin_unlock_unsafe(burst_lock);
}

// ---------------------------------------------------------------------------
// RX DMA completion, rt::irq_prio_imu_dma == 0x80
//
// Below configMAX_SYSCALL_INTERRUPT_PRIORITY, and deliberately: it may be
// delayed by core 0 critical sections and by INT1, and none of that matters.
// It is NOT in the re-arm loop -- the read that triggered it has already
// cleared DRDY -- so if it never ran again the stream would keep running and
// only the published copy would go stale, which is what the control task's
// staleness check is for.
// ---------------------------------------------------------------------------
void dmaHandler()
{
    const int chan = imu_i2c::rxChannel();
    if (chan < 0)
    {
        return;
    }
    dma_channel_acknowledge_irq0(static_cast<uint>(chan));

    // The burst buffer is stable only between a completion and the next arming.
    // If a newer burst is already in flight, DMA is overwriting it right now and
    // latched_stamp_us belongs to that burst rather than to the one that
    // completed. Discard. The sample is not lost silently: the interval to the
    // next completed sample is a period longer, which imu_stall::missedSamples()
    // turns into a gap of 1.
    if (imu_i2c::burstBusy())
    {
        completion_discards = completion_discards + 1;
        return;
    }

    // **The bytes are known to have landed by the time this runs, and that is an
    // RP2350 guarantee rather than a general DMA one.** Datasheet section 12.6.1:
    // "Previously, a channel was considered to complete on the first cycle of its
    // last write's data phase. Now, a channel is considered to complete on the
    // last cycle of its last write's data phase." On RP2040 this handler could
    // have observed the final byte in flight; on RP2350 it cannot. Checked
    // directly, because assuming RP2040 semantics here would produce a last byte
    // that is occasionally stale -- a corrupt accelerometer Z that still looks
    // like a plausible reading.
    //
    // Reader side of the single-writer handoff: snapshot the counter, then the
    // stamp and the bytes, then re-check the counter. If INT1 ran at any point
    // in between -- including part-way through the non-atomic 64-bit stamp load
    // -- the pair may straddle two samples and is thrown away rather than
    // published.
    const uint32_t started_before = bursts_started;
    __dmb();
    const uint64_t stamp_us = latched_stamp_us;

    uint8_t raw[imu_sample::burst_bytes];
    const volatile uint8_t* src = imu_i2c::burstBuffer();
    for (std::size_t i = 0; i < imu_sample::burst_bytes; ++i)
    {
        raw[i] = src[i];
    }

    __dmb();
    if (bursts_started != started_before)
    {
        completion_discards = completion_discards + 1;
        return;
    }

    // rt.h rule (b): the two locks are never nested. burst_lock is not held
    // here and must not be taken here.
    //
    // The _unsafe form again, for the PRIMASK reason in int1Handler(): blocking
    // INT1 for the length of a struct copy would be the one thing this priority
    // scheme is built to prevent. INT1 does not take this lock, so being
    // preempted while holding it cannot deadlock -- it only lengthens the window
    // core 1 may spin for, by one INT1 handler.
    spin_lock_unsafe_blocking(sample_lock);
    stream.onBurstComplete(stamp_us, raw, sample_period_us);
    spin_unlock_unsafe(sample_lock);
}

} // namespace

bool imu_drdy::init()
{
    const int chan = imu_i2c::rxChannel();
    if (chan < 0)
    {
        Log::error(category, "imu i2c not initialised");
        return false;
    }

    const int sample_lock_num = spin_lock_claim_unused(false);
    const int burst_lock_num = spin_lock_claim_unused(false);
    if (sample_lock_num < 0 || burst_lock_num < 0)
    {
        Log::error(category, "no free spinlock for the imu");
        return false;
    }
    sample_lock = spin_lock_init(static_cast<uint>(sample_lock_num));
    burst_lock = spin_lock_init(static_cast<uint>(burst_lock_num));

    // -----------------------------------------------------------------------
    // Everything below arms an interrupt, and WHERE IT RUNS IS THE POINT.
    //
    // Interrupt affinity on RP2350 is decided by the arming call, not by a
    // mask, and there are two independent per-core gates. IO_BANK0 has per-core
    // interrupt enables and gpio_set_irq_enabled() selects proc0_irq_ctrl or
    // proc1_irq_ctrl from get_core_num() (datasheet section 9.10.2, printed
    // page 603 / PDF 604), and the NVIC set-enable is per core. This function
    // runs from main() on core 0, so core 1 never enables either: the control
    // task cannot be preempted by INT1 or by this DMA channel, and neither can
    // the watchdog feed it owns.
    // -----------------------------------------------------------------------

    // Completion handler first, INT1 second. Nothing can reach the pin yet --
    // INT1_CTRL is still at its 0x00 reset value -- but ordering it this way
    // does not depend on that staying true: an edge arriving before the
    // completion path exists would start a burst whose result nothing collects.
    dma_channel_acknowledge_irq0(static_cast<uint>(chan)); // clear anything stale
    irq_set_exclusive_handler(DMA_IRQ_0, dmaHandler);
    irq_set_priority(DMA_IRQ_0, rt::irq_prio_imu_dma);
    dma_channel_set_irq0_enabled(static_cast<uint>(chan), true);
    irq_set_enabled(DMA_IRQ_0, true);

    gpio_init(int1_pin);
    gpio_set_dir(int1_pin, GPIO_IN);

    // Pulls left at their reset state (pull-down enabled). CTRL3_C's PP_OD
    // stays at its push-pull default, so the sensor drives this pad both ways
    // and a pull has nothing to do while the part is present. It does not
    // provide a defined level if the part is absent either: under erratum
    // RP2350-E9, which applies to this A2 silicon, an input-enabled bank 0 pad
    // leaks toward ~2.2 V and the pad pull-down cannot hold it. A disconnected
    // INT1 therefore shows up as an absence of edges, which the staleness check
    // reads as a stall -- not as a level anything here could test.

    // gpio_add_raw_irq_handler() rather than irq_set_exclusive_handler(), and
    // this is forced rather than preferred: gpio_set_irq_enabled() asserts that
    // the calling core has either a raw handler registered for the pin or a
    // shared callback set (pico-sdk hardware_gpio/gpio.c, the revision pinned in
    // the build image), and this firmware builds with NDEBUG undefined, so that
    // assert is live. An exclusive IO_IRQ_BANK0 handler would satisfy neither
    // arm of it and would fail at boot.
    //
    // NOT gpio_set_irq_enabled_with_callback(): rt.h forbids it because the
    // shared-callback form hides which core it is arming, which is the one
    // thing about this call that has to stay visible.
    gpio_add_raw_irq_handler(int1_pin, int1Handler);
    irq_set_priority(IO_IRQ_BANK0, rt::irq_prio_imu_int1);
    gpio_set_irq_enabled(int1_pin, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    armed = true;
    Log::info(category, "imu int1 armed");
    return true;
}

bool imu_drdy::takeSample(imu_sample::Sample* out)
{
    if (out == nullptr || sample_lock == nullptr)
    {
        return false;
    }

    // The blocking, interrupt-disabling form on THIS side, unlike the two
    // handlers. The caller is the control task on core 1, and a task can be
    // preempted mid-hold; that would leave the completion handler spinning in
    // interrupt context on core 0 until core 1 was scheduled again. Disabling
    // core 1 interrupts for the length of a struct copy is the cheaper end of
    // that trade, and it does not touch core 0, where INT1 must stay unmasked.
    const uint32_t save = spin_lock_blocking(sample_lock);
    const bool got = stream.take(out);
    spin_unlock(sample_lock, save);
    return got;
}

bool imu_drdy::peekSample(imu_sample::Sample* out)
{
    if (out == nullptr || sample_lock == nullptr)
    {
        return false;
    }

    const uint32_t save = spin_lock_blocking(sample_lock);
    const bool got = stream.peek(out);
    spin_unlock(sample_lock, save);
    return got;
}

imu_drdy::Counters imu_drdy::counters()
{
    Counters c;
    if (!armed || sample_lock == nullptr)
    {
        return c;
    }

    // The four ISR counters are single-writer 32-bit words and are read without
    // a lock; a snapshot torn across them is a diagnostic being off by one, not
    // a decision being made on bad data. The two that live inside the stream
    // share its lock because they must not be read while it is half-updated.
    c.edges = edges;
    c.burst_lock_misses = burst_lock_misses;
    c.burst_start_failures = burst_start_failures;
    c.completion_discards = completion_discards;

    const uint32_t save = spin_lock_blocking(sample_lock);
    c.duplicate_completions = stream.duplicates;
    c.published = stream.published;
    spin_unlock(sample_lock, save);
    return c;
}
