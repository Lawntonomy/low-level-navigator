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

    Log::info(category, "lsm6dsox configured");
    return true;
}
