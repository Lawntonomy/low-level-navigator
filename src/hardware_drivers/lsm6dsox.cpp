#include "lsm6dsox.hpp"

#include "imu_i2c.hpp"
#include "lsm6dsox_regs.h"
#include "pico/stdlib.h"
#include "utility/logger.h"

namespace
{

constexpr const char* category = "imu";

constexpr uint8_t who_am_i_expected = 0x6C;

// WHO_AM_I is read more than once before the part is declared absent. A
// marginal connector should not permanently block a drivetrain calibration
// session that does not need the IMU at all, and the retry costs microseconds.
// There is deliberately no compile-time bypass: those ship.
constexpr unsigned who_am_i_attempts = 3;
constexpr uint32_t who_am_i_retry_ms = 2;

// Ton from power-down is ~35 ms typical, and CTRL4_C's DRDY_MASK withholds
// data-ready until the filters have settled. 100 ms is that with margin; it is
// paid once at boot and buys a first sample that is not a startup transient.
constexpr uint32_t settle_ms = 100;

// SW_RESET self-clears. The datasheet gives no time for it, so this is a
// generous bound rather than a derived one: the whole configuration sequence is
// already budgeted in tens of milliseconds and this is paid once at boot.
constexpr unsigned sw_reset_poll_limit = 20;

// ---------------------------------------------------------------------------
// Configuration values. Order matters: see the comment in configure().
//
// **"Raw" needs qualifying, and this is the place to do it.** ADR-0006
// mandates raw counts and the part cannot supply unfiltered ones. The
// accelerometer chain is analog anti-alias -> ADC -> digital LPF1, which is
// always in path (clearing LPF2_XL_EN selects LPF1's output, it does not
// remove LPF1). The gyroscope chain ends in a mandatory LPF2 whose cutoff is
// fixed by ODR and is not user-configurable -- 66.8 Hz at 208 Hz. This does
// not conflict with ADR-0006, which means "not fused or estimated", not "no
// analog conditioning". Written down here rather than rediscovered later.
// ---------------------------------------------------------------------------

// BDU=1 (bit 6) and IF_INC=1 (bit 2). BDU defaults to 0, and without it a
// multi-byte read can straddle an internal update and return a torn high/low
// pair -- unlikely at 208 Hz against a ~0.3 ms burst, but silent when it
// happens, which is not something ADR-0006's raw mandate should tolerate by
// accident.
constexpr uint8_t cfg_ctrl3_c = 0x44;

// DRDY_MASK=1 (bit 3): withhold data-ready per channel until filter settling
// ends. The datasheet's own answer to startup transients.
constexpr uint8_t cfg_ctrl4_c = 0x08;

// 208 Hz (ODR 0101), +-4 g (FS 10), LPF2 bypassed.
//
// **The full-scale values here and in CTRL2_G are provisional and should be
// read as guesses.** CAL-6 (effective track width) is unmeasured, so the
// worst-case pivot-turn yaw rate this skid-steer chassis produces is unknown,
// and that is the measurement that would justify a gyro range. +-500 dps is a
// reasonable default for an indoor buggy and nothing more. Do not treat these
// as settled because they appear in a constant.
constexpr uint8_t cfg_ctrl1_xl = 0x58;

// 208 Hz (ODR 0101), +-500 dps (FS_G 01). See the CAL-6 note above.
constexpr uint8_t cfg_ctrl2_g = 0x54;

// INT1_DRDY_XL (bit 0) and INT1_DRDY_G (bit 1). The pin is the OR of the two
// ready flags; with both sensors on the same 208 Hz ODR that is one edge per
// sample, which diagnostics/imu-probe confirmed on this board with this exact
// value -- 209 edges against 210 drains in one second, 2026-09-09.
//
// COUNTER_BDR_REG1 (0x0B) bit 7, DRDY_PULSED, is left at its default and that
// is a decision. Pulsed mode would decouple the edge from the read and let the
// pin keep producing edges while nobody consumes them, degrading a stalled
// reader into a stream of silently stale samples. Latched turns the same fault
// into a stopped stream: loud, unambiguous, impossible to mistake for working.
// An estimator fed confidently wrong attitude is worse than one fed nothing
// (issue #47, decided 2026-09-09).
constexpr uint8_t cfg_int1_ctrl = 0x03;

struct RegWrite
{
    uint8_t reg;
    uint8_t value;
    const char* name;
};

// DRDY_MASK is set BEFORE the ODRs are enabled, so the mask is already in
// effect the moment either sensor starts producing. Writing it afterwards
// would leave a window in which unsettled samples can assert data-ready. This
// is the order diagnostics/imu-probe used and verified on hardware.
constexpr RegWrite config_sequence[] = {
    {lsm6dsox::reg_ctrl3_c, cfg_ctrl3_c, "CTRL3_C"},
    {lsm6dsox::reg_ctrl4_c, cfg_ctrl4_c, "CTRL4_C"},
    {lsm6dsox::reg_ctrl1_xl, cfg_ctrl1_xl, "CTRL1_XL"},
    {lsm6dsox::reg_ctrl2_g, cfg_ctrl2_g, "CTRL2_G"},
};

bool readWhoAmI(uint8_t* out)
{
    for (unsigned attempt = 0; attempt < who_am_i_attempts; ++attempt)
    {
        if (imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_who_am_i, out, 1) &&
            *out == who_am_i_expected)
        {
            return true;
        }
        sleep_ms(who_am_i_retry_ms);
    }
    return false;
}

} // namespace

bool lsm6dsox::configure()
{
    uint8_t who = 0;
    if (!readWhoAmI(&who))
    {
        Log::error(category, "WHO_AM_I mismatch or no response");
        return false;
    }

    // **SW_RESET first, and it is not hygiene -- it is what makes the warm-boot
    // path work at all.**
    //
    // The LSM6DSOX is not reset by an RP2350 reset: its VDD is not cycled. So
    // after a watchdog reset, a picotool reboot, a debugger attach, or a reflash
    // over diagnostics/imu-probe, the part comes up still holding the previous
    // boot's INT1_CTRL = 0x03, still sampling at 208 Hz, with nobody having read
    // its output registers since. Data-ready is latched and clears only on an
    // output-register read, so **INT1 is already high and stays high** -- and
    // arming a RISING-edge interrupt on a pin that is already high never fires.
    // Worse, the SDK's gpio_set_irq_enabled() acknowledges stale events as it
    // enables, so even a latched edge is dropped.
    //
    // The result is a stream that is dead before it starts while every
    // observable says otherwise: WHO_AM_I passes, all four CTRL registers read
    // back correct, the DMA channel is idle with no error flag, and so
    // raise_fault() is never reached. SAF-25 is satisfied in letter and defeated
    // in substance. Cold boot works, which is exactly why this would have looked
    // fine once.
    //
    // SW_RESET restores INT1_CTRL to its 0x00 reset value, which drives the pin
    // low because nothing is routed to it -- re-establishing the cold-boot
    // precondition that the whole arming order in rt.h depends on. Note this
    // does not rely on SW_RESET clearing DRDY itself; it relies on nothing
    // reaching the pin until enableDataReadyInterrupt() writes 0x03, at which
    // point the first data-ready produces a genuine rising edge.
    if (!imu_i2c::writeReg(lsm6dsox::device_addr, lsm6dsox::reg_ctrl3_c,
                           lsm6dsox::ctrl3_c_sw_reset))
    {
        Log::error(category, "SW_RESET write failed");
        return false;
    }
    for (unsigned waited = 0;; ++waited)
    {
        uint8_t ctrl3 = 0;
        if (!imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_ctrl3_c, &ctrl3, 1))
        {
            Log::error(category, "SW_RESET poll failed");
            return false;
        }
        if ((ctrl3 & lsm6dsox::ctrl3_c_sw_reset) == 0)
        {
            break;
        }
        if (waited >= sw_reset_poll_limit)
        {
            Log::error(category, "SW_RESET did not self-clear");
            return false;
        }
        sleep_ms(1);
    }

    for (const RegWrite& w : config_sequence)
    {
        if (!imu_i2c::writeReg(lsm6dsox::device_addr, w.reg, w.value))
        {
            Log::error(category, w.name);
            return false;
        }
    }

    sleep_ms(settle_ms);

    // Read back rather than trusting the writes. A write that is ACKed but not
    // applied -- wrong target, a device that reset underneath us -- produces a
    // part running at some other ODR and full scale, which then shows up as
    // scaling errors in the estimator rather than as a bus fault.
    for (const RegWrite& w : config_sequence)
    {
        uint8_t got = 0;
        if (!imu_i2c::readReg(lsm6dsox::device_addr, w.reg, &got, 1) || got != w.value)
        {
            Log::error(category, w.name);
            return false;
        }
    }

    // INT1_CTRL must still be 0x00 here. Checking it is how the warm-boot defect
    // above is prevented from coming back silently: if a future change drops the
    // SW_RESET, or SW_RESET stops clearing this register, the symptom is a dead
    // stream with every other observable healthy -- which is expensive to
    // diagnose and free to assert against.
    uint8_t int1 = 0xFF;
    if (!imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_int1_ctrl, &int1, 1) || int1 != 0x00)
    {
        Log::error(category, "INT1_CTRL not clear after reset");
        return false;
    }

    Log::info(category, "lsm6dsox configured");
    return true;
}

bool lsm6dsox::enableDataReadyInterrupt()
{
    // The last i2c transaction main() is allowed to issue. See the ordering
    // constraint in the header; there is no read-back for the reason given
    // there.
    if (!imu_i2c::writeReg(lsm6dsox::device_addr, lsm6dsox::reg_int1_ctrl, cfg_int1_ctrl))
    {
        Log::error(category, "INT1_CTRL");
        return false;
    }

    Log::info(category, "lsm6dsox data-ready interrupt enabled");
    return true;
}
