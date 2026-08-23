#pragma once

// Real-time structure of the low-level navigator: one place for every task
// priority, stack size, core affinity, and period, with the reason for each.
//
// A priority without a rationale is a bug waiting to happen, so each one says
// what it is protecting and what it is willing to lose.
//
// The governing rule (ADR-0001): this tier is independently safe and does not
// assume the high-level tier is alive, correct, or timely. No task here may
// block on anything the Pi controls.

#include "FreeRTOS.h"
#include "task.h"

namespace rt
{

// ---------------------------------------------------------------------------
// Cores
//
// RP2350 has two Cortex-M33s and FreeRTOS V11 runs them as SMP. The control
// task is pinned alone to core 1; everything else is pinned to core 0.
//
// This is not a performance decision. TP-0001 Phase 2's pass criterion is that
// the control-loop period is *statistically independent* of link state.
// Pinning makes that structurally true instead of a scheduling accident: no
// amount of MAVLink parsing, telemetry packing, or logging on core 0 can
// lengthen a control period on core 1.
//
// The cost is that anything shared across the two cores needs real
// synchronisation, not just a critical section that happens to work on one
// core. Everything crossing this boundary goes through link.hpp / safety.hpp,
// which handle it in one place rather than scattered through task bodies.
// ---------------------------------------------------------------------------

constexpr UBaseType_t core_control = (1u << 1); // core 1, exclusively
constexpr UBaseType_t core_service = (1u << 0); // core 0, everything else

// ---------------------------------------------------------------------------
// Priorities (higher number preempts lower; configMAX_PRIORITIES is 32)
// ---------------------------------------------------------------------------

// Owns the motors. A missed deadline has a physical consequence, so nothing
// preempts it — including the software timer task, which FreeRTOSConfig.h
// demotes to 10 for exactly this reason.
constexpr UBaseType_t prio_control = 20;

// Command freshness feeds the SAF-1 timeout, so parsing must not sit behind
// telemetry packing. It runs on core 0 and cannot delay control regardless,
// but ordering still matters against its peers.
constexpr UBaseType_t prio_link_rx = 15;

// Draining the TX ring to the UART. Losing telemetry is acceptable; delaying
// a heartbeat the Pi uses to judge our liveness is less so.
constexpr UBaseType_t prio_link_tx = 12;

// Builds periodic telemetry. Pure loss on overload: a dropped frame is visible
// in the sequence number, which is the whole point of having one.
constexpr UBaseType_t prio_telemetry = 8;

// Lowest. Logging is diagnostic, never required for safe operation (ADR-0003),
// and is the first thing that should starve.
constexpr UBaseType_t prio_logger = 3;

// ---------------------------------------------------------------------------
// Stacks, in words. configCHECK_FOR_STACK_OVERFLOW=2 catches an underestimate
// at runtime; telemetry reports high-water marks so these become measured
// rather than guessed.
// ---------------------------------------------------------------------------

constexpr configSTACK_DEPTH_TYPE stack_control = 1024;   // float PID + FPU frame
constexpr configSTACK_DEPTH_TYPE stack_link_rx = 1024;   // mavlink_message_t ~300 B
constexpr configSTACK_DEPTH_TYPE stack_link_tx = 512;
constexpr configSTACK_DEPTH_TYPE stack_telemetry = 1024; // packs messages on stack
constexpr configSTACK_DEPTH_TYPE stack_logger = 1024;    // vsnprintf is hungry

// ---------------------------------------------------------------------------
// Periods
// ---------------------------------------------------------------------------

// 200 Hz. Fast enough that the SAF-2 command timeout is set by stopping
// distance rather than by loop granularity, which is the right way round.
constexpr TickType_t period_control_ms = 5;

constexpr TickType_t period_telemetry_ms = 10; // scheduler for IF-0001 §6 rates
constexpr TickType_t period_link_tx_ms = 1;

// ---------------------------------------------------------------------------
// Watchdog
//
// Fed by the control task and by nothing else. A watchdog fed from a timer or
// an ISR proves only that interrupts still work; it says nothing about whether
// the task that can stop the motors is still running.
//
// Note what this does NOT fix: PIO generates PWM autonomously, so a watchdog
// reset leaves the last duty cycle driving until reinitialisation. SAF-12
// needs a hardware interlock and no firmware change closes it.
// ---------------------------------------------------------------------------

constexpr uint32_t watchdog_timeout_ms = 100;

} // namespace rt
