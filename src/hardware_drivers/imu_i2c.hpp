#pragma once

// I2C transport for the LSM6DSOX on i2c0 (GP12 SDA / GP13 SCL).
//
// **Transport only.** This file knows how to move bytes to and from a device on
// the bus and nothing about what the bytes mean. Sensor configuration, the INT1
// interrupt handler, the sample timestamp, staleness policy and message
// emission are separate pieces with their own reviews.
//
// The design this implements is issue #46 and
// system-design/research/imu-driver-findings.md sections 4-5. The one-line
// version: the burst read is what re-arms a latched data-ready, so it has to
// happen in the INT1 ISR rather than in a task that can be starved -- and 315
// microseconds of busy-wait in interrupt context would block every
// lower-priority interrupt on core 0, including link RX. RX DMA is what makes
// the guaranteed path short enough to live in an ISR: push 13 command words
// into a 16-deep FIFO, trigger a channel, return.
//
// SDK-free header on purpose, matching the rule in CLAUDE.md. Nothing here is
// host-tested today -- it is all register access -- but keeping the SDK out of
// the header means a future caller that *is* testable does not inherit it.

#include <cstddef>
#include <cstdint>

namespace imu_i2c
{

// Bytes in one sample burst: OUTX_L_G onwards, gyro XYZ then accel XYZ.
inline constexpr std::size_t burst_bytes = 12;

// IC_DATA_CMD words needed to perform that burst: one write of the
// sub-address, then one read command per byte returned. See startBurstRead().
inline constexpr std::size_t burst_cmd_words = burst_bytes + 1;

// Brings up i2c0 at 400 kHz on GP12/GP13 and claims the RX DMA channel.
//
// Returns false if the DMA channel could not be claimed or if the TX FIFO
// turns out to be shallower than burst_cmd_words -- see the FIFO-depth note in
// the .cpp, which is the assumption the whole TX strategy rests on.
//
// Does NOT touch the sensor: no register is written, WHO_AM_I is not read, and
// INT1 routing is not configured. Call the register helpers below for that.
bool init();

// Blocking single-register write and multi-byte read, for initialisation.
//
// Blocking is acceptable *here* and nowhere else on this path: these run before
// INT1 is armed and before the sample stream exists, so the busy-wait competes
// with nothing. They take the device address explicitly because the LIS3MDL
// shares this bus.
//
// Both use a bounded timeout rather than the unbounded SDK calls, so a bus that
// is wedged at startup fails a check instead of hanging the caller forever.
bool writeReg(uint8_t deviceAddr, uint8_t reg, uint8_t value);
bool readReg(uint8_t deviceAddr, uint8_t reg, uint8_t* dst, std::size_t len);

// Starts one 12-byte burst read from OUTX_L_G into the driver-owned buffer.
//
// Non-blocking: it returns as soon as the command words are in the TX FIFO and
// the RX DMA channel is armed. The bytes land later, without CPU involvement.
// Intended to be called from the INT1 handler once that exists.
//
// Returns false, having started nothing, if a previous burst is still in
// flight or if the TX FIFO cannot accept all the command words at once.
bool startBurstRead();

// True while the RX channel is still transferring. Completion of the DMA is
// the "sample ready" signal; how that is observed (channel IRQ or poll) belongs
// to the piece that consumes samples, not here.
bool burstBusy();

// Transfers remaining in the current burst, 12 down to 0.
//
// This is a live value, not a frozen one -- see the MODE note in the .cpp. It
// is a diagnostic for explaining *why* a burst did not complete; it is not a
// stall detector, because the most likely stall leaves the channel idle and
// looking perfectly healthy. imu_stall.hpp holds the detector that works.
uint32_t burstRemaining();

// The DMA destination. `volatile` because DMA writes it behind the compiler's
// back; read it only once burstBusy() is false, or bytes will be torn.
const volatile uint8_t* burstBuffer();

} // namespace imu_i2c
