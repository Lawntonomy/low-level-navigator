// imu-init: which step of the real driver's initialisation fails?
//
// Exists because low-level-nav latches Fault::init_failed on hardware and has no
// reachable log output to say why -- stdio is disabled and the console UART has
// no listener on the Pi. The probe next door runs the same register sequence
// successfully, so the difference is in the driver code rather than the sensor.
//
// This links the REAL src/hardware_drivers/imu_i2c.cpp and lsm6dsox.cpp -- not a
// copy, not a reimplementation -- and calls them in the same order main() does,
// printing each outcome. Neither of those files includes app/rt.h, which is what
// makes this possible without FreeRTOS; imu_drdy.cpp does, and is therefore not
// linked here.
//
// Touches GP12/GP13 and nothing else. Commands no motion.

#include <stdio.h>
#include "hardware_drivers/imu_i2c.hpp"
#include "hardware_drivers/lsm6dsox.hpp"
#include "hardware_drivers/lsm6dsox_regs.h"
#include "pico/stdlib.h"

int main()
{
    stdio_init_all();
    sleep_ms(3000); // let the host enumerate, or the first report goes nowhere

    const bool i2c_ok = imu_i2c::init();
    const bool cfg_ok = i2c_ok && lsm6dsox::configure();

    // Read back the registers configure() should have left, so a false return
    // can be attributed to a specific check rather than guessed at.
    uint8_t who = 0, ctrl3 = 0, ctrl4 = 0, ctrl1 = 0, ctrl2 = 0, int1 = 0;
    const bool r_who = imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_who_am_i, &who, 1);
    const bool r_c3 = imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_ctrl3_c, &ctrl3, 1);
    const bool r_c4 = imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_ctrl4_c, &ctrl4, 1);
    const bool r_c1 = imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_ctrl1_xl, &ctrl1, 1);
    const bool r_c2 = imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_ctrl2_g, &ctrl2, 1);
    const bool r_i1 = imu_i2c::readReg(lsm6dsox::device_addr, lsm6dsox::reg_int1_ctrl, &int1, 1);

    for (;;)
    {
        printf("\n=== imu-init: the real driver's init path ===\n");
        printf("  imu_i2c::init()        %s\n", i2c_ok ? "true" : "FALSE");
        printf("  lsm6dsox::configure()  %s%s\n", cfg_ok ? "true" : "FALSE",
               i2c_ok ? "" : "  (not attempted)");
        printf("\n  registers after the attempt (read independently):\n");
        printf("    WHO_AM_I   %s 0x%02X   want 0x6C\n", r_who ? "ok " : "ERR", who);
        printf("    CTRL3_C    %s 0x%02X   want 0x44\n", r_c3 ? "ok " : "ERR", ctrl3);
        printf("    CTRL4_C    %s 0x%02X   want 0x08\n", r_c4 ? "ok " : "ERR", ctrl4);
        printf("    CTRL1_XL   %s 0x%02X   want 0x58\n", r_c1 ? "ok " : "ERR", ctrl1);
        printf("    CTRL2_G    %s 0x%02X   want 0x54\n", r_c2 ? "ok " : "ERR", ctrl2);
        printf("    INT1_CTRL  %s 0x%02X   want 0x00 at this point\n", r_i1 ? "ok " : "ERR", int1);
        printf("\n  Log:: lines printed above this block, if any, name the failing\n");
        printf("  check directly -- that is the whole reason this program exists.\n");
        sleep_ms(5000);
    }
}
