#pragma once

// LSM6DSOX register addresses (DS12814 Rev 3), and nothing else.
//
// **Addresses only, deliberately.** No configuration values live here: what to
// write into CTRL1_XL or CTRL2_G is a sensor-configuration decision (ODR, full
// scale, filter path) owned by the initialisation sequence in
// system-design/research/imu-driver-findings.md section 4, and putting a value
// next to an address invites it being treated as settled when it is not --
// full-scale selection in particular is still provisional, because CAL-6
// (effective track width) has not been measured and the worst-case pivot yaw
// rate is therefore unknown.
//
// The addresses themselves are confirmed against hardware, not just the
// datasheet: diagnostics/imu-probe read WHO_AM_I = 0x6C and drained 12-byte
// bursts from OUTX_L_G on this exact board on 2026-09-09.
//
// Provenance caveat, carried forward from the research note: st.com blocked
// direct fetch, so the datasheet used was ST's own PDF retrieved from a
// third-party GitHub mirror. Primary source, mirrored, not a summary.

#include <cstdint>

namespace lsm6dsox
{

// 7-bit I2C address with the breakout's ADAG pad open (its default). A device
// answering at 0x6B instead means ADAG is bridged.
//
// **The bus is not single-drop.** The same breakout carries a LIS3MDL
// magnetometer at 0x1C (ADM open), so every transaction on i2c0 must address
// its target explicitly; nothing may assume the last IC_TAR value still points
// here.
inline constexpr uint8_t device_addr = 0x6A;

// Identity. Reads 0x6C on the LSM6DSOX.
inline constexpr uint8_t reg_who_am_i = 0x0F;

// Interrupt routing onto the INT1 pin. Reset default is 0x00, so nothing
// reaches INT1 until this is written -- and writing it has an ordering
// constraint against arming the GPIO IRQ (findings section 4), which is why
// this driver does not write it.
inline constexpr uint8_t reg_int1_ctrl = 0x0D;

// Data-ready pulse-vs-latch selection lives in bit 7 (DRDY_PULSED). Named here
// because the decision to leave it alone is load-bearing: latched data-ready
// turns a stalled reader into a stopped stream rather than a silently stale
// one (findings section 4, decided 2026-09-09).
inline constexpr uint8_t reg_counter_bdr_reg1 = 0x0B;

// Accelerometer control: ODR, full scale, LPF2 selection.
inline constexpr uint8_t reg_ctrl1_xl = 0x10;

// Gyroscope control: ODR, full scale.
inline constexpr uint8_t reg_ctrl2_g = 0x11;

// Block Data Update and general device control. Bit 0 is SW_RESET, which
// matters more than BDU does: the LSM6DSOX is NOT reset by an RP2350 reset, so
// without this write the part carries the previous boot's configuration --
// including INT1_CTRL -- across every warm reset, reflash and debugger attach.
inline constexpr uint8_t reg_ctrl3_c = 0x12;
inline constexpr uint8_t ctrl3_c_sw_reset = 0x01;

// DRDY_MASK and other secondary control bits.
inline constexpr uint8_t reg_ctrl4_c = 0x13;

// First byte of the burst the driver reads: 12 consecutive bytes, gyro XYZ
// then accel XYZ, little-endian int16 pairs. Reading these is also what
// de-asserts a latched INT1.
inline constexpr uint8_t reg_outx_l_g = 0x22;

} // namespace lsm6dsox
