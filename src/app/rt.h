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
// the control-loop period is *statistically independent* of link state, and
// pinning gets most of the way there: no amount of MAVLink parsing, telemetry
// packing, or logging on core 0 can preempt the control task on core 1.
//
// It is NOT total isolation, and claiming otherwise would be wrong. The SMP
// critical sections these modules use take a shared pair of SIO spinlocks, so
// every push(), drain() and service_tx() byte on core 0 briefly contends the
// same locks the control task needs. The coupling is single-digit microseconds,
// but it is real, and Phase 2 will measure it rather than assume it away.
//
// Note also that the idle and timer tasks carry no affinity mask, so they do
// run on core 1. "Core 1 is the control task's" is a statement about priority,
// not about occupancy.
//
// The other cost is that anything shared across the two cores needs real
// synchronisation, not a critical section that happens to work on one core.
// Everything crossing this boundary goes through link.hpp / safety.hpp, which
// handle it in one place rather than scattered through task bodies.
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
constexpr configSTACK_DEPTH_TYPE stack_link_tx = 512;    // holds one byte + a frame
constexpr configSTACK_DEPTH_TYPE stack_telemetry = 1024; // packs messages on stack
constexpr configSTACK_DEPTH_TYPE stack_logger = 512;     // drain() holds one byte

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
// What a watchdog reset actually does is stronger than the project record
// claims, and narrower than SAF-12 asks for.
//
// watchdog_enable() sets PSM_WDSEL bit 4 (RESETS), so a timeout resets the
// RESETS block; PIO0-2, IO_BANK0 and PADS_BANK0 all revert to held-in-reset
// (datasheet Table 535, printed p.505). The pad is then disconnected outright
// rather than merely idled: IO_BANK0 FUNCSEL resets to 0x1f = NULL (printed
// p.610) and PADS_BANK0 resets to IE=0, PDE=1 (printed p.787) - no peripheral
// muxed, input buffer off, and a 36-113 kΩ pull-down on (§14.9.4, printed
// p.1340) - weak, and quantified here because how weak turns out to matter.
// So system-overview.md's "a halted RP2350 leaves the last duty cycle running
// indefinitely" is wrong for the watchdog case. Erratum RP2350-E9 also does
// not apply here, because its precondition is IE=1 and this reset clears IE.
//
// It still does NOT close SAF-12:
//   - a watchdog measures "did some code call feed", not "is the output
//     right", so a live-but-wrong task satisfies it forever;
//   - it shares die, rail and clock tree with what it supervises;
//   - pause_on_debug stops the counter under a debugger;
//   - that pull-down does not win at the pad where it matters: the motor
//     driver reads its enable HIGH while nothing drives it (see below);
//   - no datasheet figure exists for reset-to-pads-quiet latency.
// An interlock independent of the RP2350 remains required. Bench measurement
// that would settle the last item, and confirm the divider below: scope
// GPIO 2/3/6 AT THE DRIVER IC while forcing a watchdog timeout.
//
// The undriven-enable case is no longer an open question; it has been worked
// and the answer is the unfavourable one. The motor driver is an Adafruit
// TB6612 breakout (product 2448, Toshiba TB6612FNG) and GPIO 2 is its
// active-high STBY input. Adafruit fit R1, a 10 kΩ pull-up from STBY to VCC,
// the only resistor added anywhere on that board (their published EAGLE
// schematic, part R1). Against it stand the TB6612's 200 kΩ internal
// pull-down on every control pin (datasheet Pin Functions) and the RP2350 pad
// pull-down above. Both pull-downs in parallel against the 10 kΩ give STBY =
// 2.49 V at RPD = 36 kΩ and 2.90 V at 113 kΩ for VCC = 3.3 V, and 3.77 V at
// 36 kΩ for VCC = 5.0 V, against V_IH(min) = 0.7·VCC. Every corner reads
// HIGH, worst-case margin +0.18 V.
//
// So the driver is enabled from the moment VCC rises until firmware drives
// GPIO 2 low, and a watchdog reset reopens that gate rather than closing it.
// SAF-10 is violated by the hardware, not by the ordering in motors_init().
//
// It does not follow that the machine drives in that window: IN1/IN2/PWM are
// held low by the same two pull-downs, and IN1=IN2=L is the coast row of the
// TB6612 truth table. What is lost is the second gate — the enable is open
// with nothing behind it, which is exactly the defence in depth SAF-10 exists
// to provide.
//
// The fix is a board modification, not a firmware change: remove R1, or fit an
// external pull-down ≤ 4.3 kΩ (≈2.2 kΩ is the sensible pick, since 4.3 kΩ
// lands at 0.98 V against a 0.99 V threshold). Firmware cannot close this.
// Full analysis: ADR-0010, as SAF-19.
// ---------------------------------------------------------------------------

constexpr uint32_t watchdog_timeout_ms = 100;

} // namespace rt
