#pragma once

// The motor outputs, in one place.
//
// Lifted out of main.cpp so that every path which must remove drive — the boot
// sequence, the fault halt, the stack-overflow hook, and the link-commanded
// BOOTSEL reboot — calls the same function rather than each open-coding the
// SAF-10 ordering rule. Nothing here decides *whether* to drive; that is
// safety.hpp's job.
//
// Header kept free of the Pico SDK so callers that must stay SDK-free can
// still declare their intent to use it.

namespace motors
{

// Drives every motor pin low, then configures it as an output. Call before
// anything that could move.
void init();

// SAF-10: every output at a defined zero, and drive enable deasserted.
// Idempotent, and safe to call from a fault hook.
void safe_state();

} // namespace motors
