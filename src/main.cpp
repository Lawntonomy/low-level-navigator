// Low-level navigator — task foundation.
//
// Replaces the flat superloop in low-level-navigator.cpp. What exists here is
// the real-time skeleton and the safety-critical plumbing; the control law
// itself is a placeholder and the PID and PIO PWM paths are NOT wired up. That
// is deliberate: getting the task structure, the link, and the arming/timeout
// rules right is the part that is expensive to retrofit.
//
// Structure and rationale: src/app/rt.h
// Protocol:                system-design/interfaces/inter-tier-protocol.md
// Transport:               system-design/test-plans/0001-inter-tier-link-bringup.md
//
// NOT implemented, and required before this drives anything with traction:
//   SAF-30  direction reversal through a commanded stop
//   SAF-31  slew limit on commanded speed. Note IF-0001 §7.5's "stays armed
//           through a stall" only makes sense once this exists: without it, a
//           149 ms stall followed by recovery re-applies the full previous
//           command in a single 5 ms period.
//   SAF-20  wheel-speed validity now reaches telemetry (issue #12), but is
//           still not consumed for control — nothing here disarms or faults
//           on encoder::Reading::valid == false
//   SAF-54  local link-quality degrade (the inputs are collected, the check is
//           not written — the Pi is currently the only consumer)

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "app/board.h"
#include "app/bootloader.hpp"
#include "app/link.hpp"
#include "app/log.hpp"
#include "app/motors.hpp"
#include "app/rt.h"
#include "app/safety.hpp"

#include "hardware_drivers/encoder.hpp"

namespace
{

// Every path that gives up must remove drive first. SAF-13: drive enable comes
// off before a fault is reported, not after.
[[noreturn]] void halt(const char* why)
{
    motors::safe_state();

    // Deliberate: with no hardware interlock, holding these pins driven low
    // forever is safer than letting the watchdog reset the chip every 100 ms
    // and return the pads to their undriven reset state at ~10 Hz. Revisit if
    // an interlock is ever fitted, because then the opposite is true.
    watchdog_disable();

    log_console::write_blocking(why);
    for (;;)
    {
        tight_loop_contents();
    }
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

// Owns the motors. Pinned to core 1 so link and logging work on core 0 cannot
// lengthen its period (rt.h).
[[noreturn]] void control_task(void*)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        gpio_xor_mask(1u << board::scope_pin); // TP-0001 Phase 2 ground truth

        const safety::Decision d = safety::evaluate();

        // PLACEHOLDER: the PID and PIO PWM path is not connected, so the
        // outputs are held safe on every iteration regardless of the decision.
        //
        // record_applied(0, 0) even when armed and commanded, because
        // LAWN_WHEEL_STATE.left_cmd_drpm means "what was actually applied"
        // (IF-0001 §8). Reporting the request there would make requested and
        // applied identical by construction and hide the divergence the field
        // exists to expose — and a Phase 3 stop test would "pass" without ever
        // having commanded motion.
        motors::safe_state();
        safety::record_applied(0, 0);
        (void)d;

        // Observation only (issue #12, TP-0002 CAL-0): read both wheels every
        // iteration, exactly once, per encoder.hpp's contract — freshness is
        // judged by whether the DMA write pointer moved since the previous
        // call, so skipping or doubling a poll here would corrupt the
        // staleness measurement, not just delay it. NOT fed into control: the
        // PID is not connected (see file header), so this exists only to carry
        // the reading to telemetry across the core boundary.
        encoder::publish_readings({encoder::read_left(), encoder::read_right()});

        // Fed here and nowhere else. A watchdog fed from a timer or an ISR
        // proves only that interrupts still work; it says nothing about whether
        // the task that can stop the motors is still running. Proposed SAF-16.
        watchdog_update();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(rt::period_control_ms));
    }
}

[[noreturn]] void link_rx_task(void*)
{
    for (;;)
    {
        link::rx_poll();
        // Polled rather than interrupt-driven. At 1 Mbaud the 32-byte FIFO
        // fills in 320 us, so this period is 3x the overflow time — adequate at
        // the IF-0001 §6 rates (~1 byte/ms inbound) but not under a burst.
        // An RX interrupt would fix that and the t2 timestamp error together;
        // it is the change to make before bench time, not after.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Sole writer to the command UART — see link.hpp's ownership rule.
[[noreturn]] void link_tx_task(void*)
{
    for (;;)
    {
        link::service_tx();

        // A link-commanded BOOTSEL reboot is taken here and nowhere else.
        //
        // The RX path only latches the request (bootloader.hpp). It cannot take
        // the reset itself for two reasons: rom_reset_usb_boot() does not
        // return, so a frame half-way out of the shift register would be cut
        // off and desynchronise the far end; and this task is the sole writer
        // to the command UART, so it is the only one allowed to wait on it.
        //
        // Motors are safed on every pass while the request is outstanding, not
        // once at the end, so drive is off from the first opportunity rather
        // than after however many passes the ring takes to drain. safe_state()
        // is idempotent. bootloader::enter() safes them again; that is
        // deliberate belt and braces, not an oversight.
        if (bootloader::pending())
        {
            motors::safe_state();
            if (link::tx_quiesce())
            {
                bootloader::enter(); // does not return
            }
        }

        vTaskDelay(pdMS_TO_TICKS(rt::period_link_tx_ms));
    }
}

// Builds the periodic messages at the rates in IF-0001 §6.
[[noreturn]] void telemetry_task(void*)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t next_hb = 0, next_status = 0, next_wheel = 0, next_stats = 0;

    for (;;)
    {
        const uint64_t now = time_us_64();

        if (now >= next_hb)
        {
            link::send_heartbeat();
            next_hb = now + 50000; // 20 Hz
        }
        if (now >= next_status)
        {
            link::send_nav_status();
            next_status = now + 50000; // 20 Hz
        }
        if (now >= next_wheel)
        {
            // SAF-20: the Pi is told the data is untrustworthy rather than
            // handed a plausible zero. encoder::Reading::valid crosses the
            // core boundary unmodified in encoder::WheelReadings and is
            // reported exactly as read — this task never manufactures a zero
            // rpm and calls it valid, and never invents a valid flag either.
            const encoder::WheelReadings wheels = encoder::latest_readings();
            const safety::Status st = safety::status();
            link::send_wheel_state(encoder::rpm_to_deci_rpm(wheels.left.rpm),
                                   encoder::rpm_to_deci_rpm(wheels.right.rpm), st.left_applied,
                                   st.right_applied, wheels.left.valid, wheels.right.valid);
            next_wheel = now + 20000; // 50 Hz
        }
        if (now >= next_stats)
        {
            link::send_link_stats();
            next_stats = now + 1000000; // 1 Hz
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(rt::period_telemetry_ms));
    }
}

[[noreturn]] void logger_task(void*)
{
    for (;;)
    {
        if (log_console::drain() == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(5)); // idle: nothing queued
        }
        else
        {
            taskYIELD();
        }
    }
}

// Creates a task or halts. Firmware that silently comes up without its control
// loop is worse than firmware that does not come up.
void must_create(TaskFunction_t fn, const char* name, configSTACK_DEPTH_TYPE stack,
                 UBaseType_t prio, UBaseType_t affinity)
{
    TaskHandle_t h = nullptr;
    if (xTaskCreate(fn, name, stack, nullptr, prio, &h) != pdPASS || h == nullptr)
    {
        halt("[boot] FATAL: task create failed\r\n");
    }
#if (configNUMBER_OF_CORES > 1) && (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(h, affinity);
#else
    (void)affinity;
#endif
}

} // namespace

// FreeRTOS hooks. Both are enabled in FreeRTOSConfig.h so a memory fault stops
// the machine loudly instead of corrupting it quietly.

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char* name)
{
    motors::safe_state();
    log_console::write_blocking("[FATAL] stack overflow in task: ");
    log_console::write_blocking(name ? name : "?");
    halt("\r\n");
}

extern "C" void vApplicationMallocFailedHook()
{
    halt("[FATAL] malloc failed\r\n");
}

int main()
{
    // Before anything that could move, and before the console: motors::init()
    // needs no logging, and doing it first shortens the window in which
    // driver_enable_pin sits in its undriven reset state by the time it takes
    // to clock out the boot banner.
    motors::init();

    log_console::init();
    log_console::write_blocking("\r\n\r\n[boot] low-level navigator\r\n");
    log_console::write_blocking("[boot] motors in safe state\r\n");

    // Observation only (issue #12, TP-0002 CAL-0): wires the wheel-speed
    // reading to telemetry. Does NOT connect the PID or command any motion —
    // control_task keeps calling motors_safe_state() unconditionally, exactly
    // as before this change.
    //
    // PIO block: RP2350 has three (pio0-2). This claims pio2 and leaves the
    // other two alone rather than picking whichever compiles. pwm.cpp's PIO
    // PWM path is the other block's eventual tenant — not wired into this
    // build yet (see the file header) but its two state machines need a home
    // when it is. ADR-0010 (Proposed) separately wants a PIO block reserved
    // for a drive-enable interlock and explicitly calls out keeping that
    // block distinct from PWM and from the encoders, so this cannot simply
    // take "the other" free block either. That leaves pio2 as the one block
    // this firmware can claim today without pre-empting either future tenant;
    // it has all four state machines free and encoder::init() only needs two
    // (sm_index and sm_index+1).
    encoder::init(pio2, 0);
    log_console::write_blocking("[boot] encoder init complete\r\n");

    gpio_init(board::scope_pin);
    gpio_set_dir(board::scope_pin, GPIO_OUT);

    safety::init();

    // safety::init() took a critical section before the scheduler was running.
    // The SMP vTaskEnterCritical masks interrupts unconditionally, but
    // vTaskExitCritical only unmasks when xSchedulerRunning is true — so
    // interrupts are still masked here. Benign for the polled calls below, and
    // a landmine for anything interrupt-driven added later.
    portENABLE_INTERRUPTS();

    if (!link::init())
    {
        halt("[boot] FATAL: command UART baud out of tolerance\r\n");
    }

    if (watchdog_caused_reboot())
    {
        log_console::write_blocking("[boot] WARNING: last reset was watchdog\r\n");
    }

    must_create(control_task, "control", rt::stack_control, rt::prio_control, rt::core_control);
    must_create(link_rx_task, "link_rx", rt::stack_link_rx, rt::prio_link_rx, rt::core_service);
    must_create(link_tx_task, "link_tx", rt::stack_link_tx, rt::prio_link_tx, rt::core_service);
    must_create(telemetry_task, "telem", rt::stack_telemetry, rt::prio_telemetry, rt::core_service);
    must_create(logger_task, "logger", rt::stack_logger, rt::prio_logger, rt::core_service);

    // Armed last, so a slow boot cannot trip it before the control task exists
    // to feed it. Nothing blocking may be added after this point without
    // re-checking the margin against rt::watchdog_timeout_ms.
    //
    // pause_on_debug = true stops the counter while a debugger has a core
    // halted. Convenient on the bench and a hole in production: proposed
    // SAF-17 would require it disabled in non-development builds.
    watchdog_enable(rt::watchdog_timeout_ms, true);

    log_console::write_blocking("[boot] starting scheduler\r\n");
    vTaskStartScheduler();

    halt("[FATAL] scheduler returned\r\n");
}
