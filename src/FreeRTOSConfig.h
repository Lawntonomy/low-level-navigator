#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// Baseline comes from the Pico example config (2 cores, 1 kHz tick,
// 32 priorities, 128 KiB heap). Everything below is a deliberate override for
// this application; see src/app/rt.h for the task table those settings serve.
#include "FreeRTOSConfig_examples_common.h"

// ---------------------------------------------------------------------------
// Failures must be loud, not silent.
//
// This firmware cannot be run by a reviewer and has no emulator, so a defect
// that corrupts memory quietly is the worst outcome available. Each of these
// converts a silent failure into a stop.
// ---------------------------------------------------------------------------

#undef configCHECK_FOR_STACK_OVERFLOW
#define configCHECK_FOR_STACK_OVERFLOW 2 // pattern-fill check, not just pointer

#undef configUSE_MALLOC_FAILED_HOOK
#define configUSE_MALLOC_FAILED_HOOK 1

// Uses the Cortex-M primitive rather than taskDISABLE_INTERRUPTS(): configASSERT
// expands inside kernel internals that are compiled before task.h's macros are
// visible, so the FreeRTOS-level wrapper is not available here.
#undef configASSERT
#define configASSERT(x)                                                                            \
    if ((x) == 0)                                                                                  \
    {                                                                                              \
        __asm volatile("cpsid i" ::: "memory");                                                    \
        for (;;)                                                                                   \
            ;                                                                                      \
    }

// ---------------------------------------------------------------------------
// The software timer task must not outrank the control loop.
//
// The example config puts it at configMAX_PRIORITIES-1, which would let any
// timer callback preempt the task that owns the motors. Nothing on the
// actuation path may be preempted by housekeeping.
// ---------------------------------------------------------------------------

#undef configTIMER_TASK_PRIORITY
#define configTIMER_TASK_PRIORITY 10 // below RT_PRIO_CONTROL (20)

#undef configTIMER_TASK_STACK_DEPTH
#define configTIMER_TASK_STACK_DEPTH 512

// Runtime introspection: needed to report stack headroom in telemetry rather
// than guessing at the numbers in rt.h.
#undef INCLUDE_uxTaskGetStackHighWaterMark
#define INCLUDE_uxTaskGetStackHighWaterMark 1

#undef INCLUDE_xTaskGetCurrentTaskHandle
#define INCLUDE_xTaskGetCurrentTaskHandle 1

#endif // FREERTOS_CONFIG_H
