#pragma once

// LSM6DSOX device configuration: what to write, in what order, and what must
// read back before the part is trusted.
//
// Separate from imu_i2c.{hpp,cpp} on purpose. That file scopes itself to
// transport -- moving bytes to and from a device on the bus, knowing nothing
// about what they mean -- and this is the other half: which registers, which
// values, and why those values. Keeping the split means a bus problem and a
// configuration problem are diagnosed in different files.
//
// Raises no faults and takes no safety action. hardware_drivers/ holds thin
// register-level drivers with no business logic (CLAUDE.md), so the decision
// that a failure here disarms the machine belongs to the caller, in app/.

#include <cstdint>

namespace lsm6dsox
{

// Reads WHO_AM_I, then writes and verifies the accelerometer and gyroscope
// configuration. Returns false if the part does not identify as an LSM6DSOX or
// if any register fails to read back what was written.
//
// Requires imu_i2c::init() to have succeeded first.
//
// **Deliberately does not write INT1_CTRL (0x0D).** Nothing reaches the INT1
// pin until that register is written, and writing it has an ordering
// constraint: data-ready is latched, so if a sample becomes ready during the
// ~35 ms filter settle before a GPIO handler exists, INT1 asserts once, is
// never cleared by a read, and no second edge is ever generated -- the stream
// never starts and it looks exactly like a disconnected pin. So INT1_CTRL is
// written by the interrupt-handling piece, after the IRQ is armed, and its
// absence here is a decision rather than an omission.
bool configure();

// Writes INT1_CTRL = 0x03 (INT1_DRDY_XL | INT1_DRDY_G), which is what starts
// the data-ready stream.
//
// **Separate from configure() because of an ordering constraint that has to
// stay visible.** This must be the LAST thing done before
// vTaskStartScheduler(), after imu_drdy::init() has installed and enabled both
// interrupt handlers. Two reasons, in rt.h:
//
//   - Until then, configure() and the register helpers are still busy-waiting
//     on i2c0 from main(). An edge arriving mid-write would start a burst on a
//     bus a blocking call is already driving -- interleaved IC_DATA_CMD writes,
//     a TX FIFO that empties mid-transaction, SCL held low. A bus wedged at
//     boot.
//   - Data-ready is latched, so if a sample becomes ready before a handler
//     exists to drain it, INT1 asserts once, is never cleared by a read, and no
//     second edge is ever generated. The stream never starts and it looks
//     exactly like a disconnected pin.
//
// INT1_CTRL's 0x00 reset value means nothing reaches the pin before this, so
// waiting costs no samples and loses no early latched edge.
//
// **Deliberately not read back**, unlike every register configure() writes: the
// read-back would itself be a blocking i2c transaction on a bus that, by the
// time it ran, could already be carrying an interrupt-driven burst. Verifying
// the write is worth less than not creating the collision it would be verifying
// against. A write that is ACKed but not applied shows up as a stream that
// never produces an edge, which the staleness check reports.
bool enableDataReadyInterrupt();

} // namespace lsm6dsox
