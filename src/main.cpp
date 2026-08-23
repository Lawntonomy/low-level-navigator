// Low-level navigator — task foundation.
//
// Replaces the flat superloop in low-level-navigator.cpp. What exists here is
// the real-time skeleton and the safety-critical plumbing; the control law
// itself is a placeholder (see control_task) and the PID and PWM paths are NOT
// wired up yet. That is deliberate: getting the task structure, the link, and
// the arming/timeout rules right is the part that is expensive to retrofit.
//
// Structure and rationale: src/app/rt.h
// Protocol:                system-design/interfaces/inter-tier-protocol.md
// Transport:               system-design/test-plans/0001-inter-tier-link-bringup.md
//
// NOT yet implemented, and required before this drives anything with traction:
//   SAF-30  direction reversal through a commanded stop
//   SAF-31  slew limit on commanded speed
//   SAF-20  wheel-speed validity (encoder still reports stale RPM, issue #12)
//   SAF-12  hardware interlock on drive enable — no firmware change closes this

#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "app/board.h"
#include "app/link.hpp"
#include "app/log.hpp"
#include "app/rt.h"
#include "app/safety.hpp"

#include "hardware_drivers/gpio_defines.h"

namespace
{

// ---------------------------------------------------------------------------
// Motor outputs
//
// Kept behind one function so there is exactly one place where a number
// becomes motion, and so the ordering rule in SAF-10 is visible rather than
// implied. PIO PWM is not attached yet; this sets direction and enable only.
// ---------------------------------------------------------------------------

void motors_safe_state()
{
    // SAF-10: outputs at a defined zero BEFORE drive enable is ever asserted.
    // The current firmware asserts enable first, which is the defect this
    // ordering exists to avoid.
    gpio_put(static_cast<uint>(gpio::pins::left_forward_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::left_backward_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::right_forward_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::right_backward_pin), 0);
    gpio_put(static_cast<uint>(gpio::pins::driver_enable_pin), 0);
}

void motors_init()
{
    const uint pins[] = {
        static_cast<uint>(gpio::pins::driver_enable_pin),
        static_cast<uint>(gpio::pins::left_forward_pin),
        static_cast<uint>(gpio::pins::left_backward_pin),
        static_cast<uint>(gpio::pins::right_forward_pin),
        static_cast<uint>(gpio::pins::right_backward_pin),
    };
    for (uint p : pins)
    {
        gpio_init(p);
        gpio_put(p, 0);          // drive low before enabling the output driver
        gpio_set_dir(p, GPIO_OUT);
    }
    motors_safe_state();
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------

// Owns the motors. Pinned alone to core 1 so that nothing on the link or
// logging path can lengthen its period (rt.h).
[[noreturn]] void control_task(void *)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        gpio_xor_mask(1u << board::scope_pin); // TP-0001 Phase 2 ground truth

        const uint64_t now = time_us_64();
        const safety::Decision d = safety::evaluate(now);

        if (!d.armed || d.ramp_to_zero)
        {
            motors_safe_state();
            safety::record_applied(0, 0);
        }
        else
        {
            // PLACEHOLDER. The PID and PIO PWM path is not connected: this
            // records the decision so telemetry shows requested-vs-applied,
            // and leaves the outputs in their safe state.
            //
            // Wiring this up requires SAF-30 (reversal through zero) and
            // SAF-31 (slew limit) to exist first — without them the first
            // command that crosses zero shocks the drivetrain.
            safety::record_applied(d.left_drpm, d.right_drpm);
        }

        // Fed here and nowhere else. A watchdog fed from a timer or an ISR
        // proves only that interrupts still work; it says nothing about
        // whether the task that can stop the motors is still running.
        watchdog_update();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(rt::period_control_ms));
    }
}

[[noreturn]] void link_rx_task(void *)
{
    for (;;)
    {
        link::rx_poll(time_us_64());
        // Polled rather than interrupt-driven for now. At 1 Mbaud the 32-byte
        // FIFO fills in 320 us, so this period must stay well inside that;
        // TP-0001 Phase 3 watches /proc-side overrun counters for the symptom.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

[[noreturn]] void link_tx_task(void *)
{
    for (;;)
    {
        link::tx_drain();
        vTaskDelay(pdMS_TO_TICKS(rt::period_link_tx_ms));
    }
}

// Builds the periodic messages at the rates in IF-0001 §6.
[[noreturn]] void telemetry_task(void *)
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
            link::send_nav_status(now);
            next_status = now + 50000; // 20 Hz
        }
        if (now >= next_wheel)
        {
            // Encoder read is not wired up yet; report the applied value with
            // validity FALSE so the Pi is told the data is not trustworthy
            // rather than being handed a plausible zero (SAF-20).
            const safety::Status st = safety::status(now);
            link::send_wheel_state(now, 0, 0, st.left_applied, st.right_applied,
                                   false, false);
            next_wheel = now + 20000; // 50 Hz
        }
        if (now >= next_stats)
        {
            link::send_link_stats(now);
            next_stats = now + 1000000; // 1 Hz
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(rt::period_telemetry_ms));
    }
}

[[noreturn]] void logger_task(void *)
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

// Creates a task or halts. A firmware that silently comes up without its
// control loop is worse than one that does not come up.
void must_create(TaskFunction_t fn, const char *name,
                 configSTACK_DEPTH_TYPE stack, UBaseType_t prio,
                 UBaseType_t affinity)
{
    TaskHandle_t h = nullptr;
    if (xTaskCreate(fn, name, stack, nullptr, prio, &h) != pdPASS || h == nullptr)
    {
        log_console::write_blocking("[boot] FATAL: task create failed\r\n");
        for (;;)
        {
            tight_loop_contents();
        }
    }
#if (configNUMBER_OF_CORES > 1) && (configUSE_CORE_AFFINITY == 1)
    vTaskCoreAffinitySet(h, affinity);
#else
    (void)affinity;
#endif
}

} // namespace

// FreeRTOS hooks. Both are configured on in FreeRTOSConfig.h so that a memory
// fault stops the machine loudly instead of corrupting it quietly.

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *name)
{
    log_console::write_blocking("[FATAL] stack overflow in task: ");
    log_console::write_blocking(name ? name : "?");
    log_console::write_blocking("\r\n");
    for (;;)
    {
        tight_loop_contents();
    }
}

extern "C" void vApplicationMallocFailedHook()
{
    log_console::write_blocking("[FATAL] malloc failed\r\n");
    for (;;)
    {
        tight_loop_contents();
    }
}

int main()
{
    // Console first, so anything that fails below is reportable. Nothing here
    // uses stdio: both backends are disabled in CMakeLists.txt because both
    // block, and stdio fans every write out to every enabled driver.
    log_console::init();
    log_console::write_blocking("\r\n\r\n[boot] low-level navigator\r\n");

    gpio_init(board::scope_pin);
    gpio_set_dir(board::scope_pin, GPIO_OUT);

    // Before anything else that could move: outputs defined, enable deasserted.
    motors_init();
    log_console::write_blocking("[boot] motors in safe state\r\n");

    safety::init(time_us_64());

    if (!link::init())
    {
        log_console::write_blocking(
            "[boot] FATAL: command UART baud out of tolerance\r\n");
        for (;;)
        {
            tight_loop_contents();
        }
    }

    if (watchdog_caused_reboot())
    {
        // Worth knowing about: it means a task missed its deadline badly
        // enough to stop feeding, and PIO PWM kept running through the reset.
        log_console::write_blocking("[boot] WARNING: last reset was watchdog\r\n");
    }

    must_create(control_task, "control", rt::stack_control, rt::prio_control,
                rt::core_control);
    must_create(link_rx_task, "link_rx", rt::stack_link_rx, rt::prio_link_rx,
                rt::core_service);
    must_create(link_tx_task, "link_tx", rt::stack_link_tx, rt::prio_link_tx,
                rt::core_service);
    must_create(telemetry_task, "telem", rt::stack_telemetry,
                rt::prio_telemetry, rt::core_service);
    must_create(logger_task, "logger", rt::stack_logger, rt::prio_logger,
                rt::core_service);

    // Armed last, so a slow boot cannot trip it before the control task exists
    // to feed it.
    watchdog_enable(rt::watchdog_timeout_ms, true);

    log_console::write_blocking("[boot] starting scheduler\r\n");
    vTaskStartScheduler();

    // Unreachable unless the scheduler could not start.
    log_console::write_blocking("[FATAL] scheduler returned\r\n");
    for (;;)
    {
        tight_loop_contents();
    }
}
