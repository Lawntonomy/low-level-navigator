// imu-probe: is the LSM6DSOX actually there, and is INT1 actually wired?
//
// Standalone and bench-only, for the same reason pad-probe is: the main
// firmware disables both stdio backends and its console UART has no listener
// attached on the Pi, so a probe built into it would have nowhere to report.
//
// This answers the physical questions in system-design/research/
// imu-driver-findings.md section 7, in the order that makes a failure
// diagnosable rather than just visible:
//
//   0. What do SDA/SCL/INT1 sit at before anything is configured?
//   1. Does anything ACK on the bus at 400 kHz?
//   2. Does WHO_AM_I read 0x6C?
//   3. Does the configuration sequence stick when read back?
//   4. Does INT1 produce edges at the configured ODR -- and if it does not,
//      is the sensor still producing data (STATUS_REG toggling)? That is the
//      difference between "no data" and "data, but the INT1 wire is wrong",
//      which is the single most likely bring-up fault and the one a bare
//      "no samples" message would leave ambiguous.
//   5. Do the raw counts look like a stationary sensor in 1 g?
//
// Wiring this expects (gpio_defines.h, 2026-09-09):
//   GP12 = I2C0 SDA, GP13 = I2C0 SCL, GP14 = LSM6DSOX INT1.
//
// The breakout, per its silkscreen (confirmed by Andrew 2026-09-09), carries
// INT1, INT2, INTM, DRDY, ADM and ADAG. INT1/INT2 belong to the LSM6DSOX;
// INTM/DRDY belong to the LIS3MDL, which is why DRDY is NOT the pin the
// data-ready architecture wants. Only INT1 is wired; INT2, INTM and DRDY are
// left unconnected, which is harmless -- they are outputs on the breakout, and
// RP2350-E9 is a hazard for undriven RP2350 pads, not for unread sensor pins.
//
// ADAG and ADM are the address-select jumper pads for the accel/gyro and the
// magnetometer. **Both are open**, so both parts sit at their default
// addresses: LSM6DSOX 0x6A and LIS3MDL 0x1C. An ACK at 0x6B or 0x1E instead
// would mean a jumper is bridged -- worth knowing, because it is a five-second
// fix that otherwise reads as a dead bus.
//
// This drives no motor pin and touches nothing outside GP12/13/14.

#include <math.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define SDA_PIN 12
#define SCL_PIN 13
#define INT1_PIN 14
#define I2C_BAUD 400000 // ceiling: the LSM6DSOX slave timing table has no Fast-mode-plus row

// LSM6DSOX registers (DS12814 Rev 3).
#define REG_WHO_AM_I 0x0F
#define REG_INT1_CTRL 0x0D
#define REG_CTRL1_XL 0x10
#define REG_CTRL2_G 0x11
#define REG_CTRL3_C 0x12
#define REG_CTRL4_C 0x13
#define REG_STATUS 0x1E
#define REG_OUTX_L_G 0x22 // 12 bytes: gyro XYZ then accel XYZ
#define CTRL3_C_SW_RESET 0x01

#define LSM6DSOX_WHO_AM_I 0x6C
#define LIS3MDL_WHO_AM_I 0x3D // expected, but not read from the LIS3MDL datasheet in this repo

// 208 Hz on both channels, +-4 g, +-500 dps, LPF2 bypassed, BDU set, DRDY
// masked until the filters settle, accel+gyro data-ready routed to INT1.
#define CFG_CTRL3_C 0x44
#define CFG_CTRL1_XL 0x58
#define CFG_CTRL2_G 0x54
#define CFG_CTRL4_C 0x08
#define CFG_INT1_CTRL 0x03
#define CFG_ODR_HZ 208

#define MAX_EDGES 600
static volatile uint32_t edge_us[MAX_EDGES];
static volatile uint32_t edge_count;

// Latch the capture at IRQ entry, before anything else. This is the same
// mechanism ADR-0007 relies on and link.cpp already uses for t2_us: one
// register read, so the transfer that follows cannot contaminate the stamp.
static void int1Isr(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
    const uint32_t t = time_us_32();
    const uint32_t n = edge_count;
    if (n < MAX_EDGES)
    {
        edge_us[n] = t;
    }
    edge_count = n + 1;
}

// Bounded, not blocking. The unbounded SDK calls cost a whole bench cycle on
// 2026-09-09: a write that left the part not ACKing hung this program before its
// first printf, so it reported nothing at all and looked identical to a board
// that had not been flashed. A diagnostic that can hang is not a diagnostic.
#define I2C_TIMEOUT_US 10000

static bool regRead(uint8_t addr, uint8_t reg, uint8_t* dst, size_t len)
{
    if (i2c_write_timeout_us(i2c0, addr, &reg, 1, true, I2C_TIMEOUT_US) != 1)
    {
        return false;
    }
    return i2c_read_timeout_us(i2c0, addr, dst, len, false, I2C_TIMEOUT_US) == (int)len;
}

static bool regWrite(uint8_t addr, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
    return i2c_write_timeout_us(i2c0, addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static const char* addrNote(uint8_t addr)
{
    switch (addr)
    {
    case 0x6A: return "LSM6DSOX, ADAG open -- expected";
    case 0x6B: return "LSM6DSOX, but ADAG is BRIDGED";
    case 0x1C: return "LIS3MDL, ADM open -- expected";
    case 0x1E: return "LIS3MDL, but ADM is BRIDGED";
    default: return "unexpected -- not a device this board should carry";
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(3000); // let the USB host enumerate, or the first report goes nowhere

    // Stage 0 must happen before i2c_init assigns the pad functions, so read
    // the pads first and print later.
    gpio_init(SDA_PIN);
    gpio_init(SCL_PIN);
    gpio_init(INT1_PIN);
    sleep_ms(10);
    const bool idle_sda = gpio_get(SDA_PIN);
    const bool idle_scl = gpio_get(SCL_PIN);
    const bool idle_int1 = gpio_get(INT1_PIN);

    i2c_init(i2c0, I2C_BAUD);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    // Deliberately NO gpio_pull_up() here. The bus must work on its external
    // pull-ups, because that is what the running firmware will depend on;
    // RP2350 section 12.2.1.3 warns the pad pull-ups may not be strong enough,
    // and enabling them here would hide a missing resistor until the first
    // long cable or fast edge exposed it.

    uint8_t found[8];
    unsigned found_n = 0;
    uint8_t imu_addr = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        uint8_t rx;
        if (i2c_read_timeout_us(i2c0, addr, &rx, 1, false, I2C_TIMEOUT_US) >= 0)
        {
            if (found_n < count_of(found))
            {
                found[found_n++] = addr;
            }
            if (addr == 0x6A || addr == 0x6B)
            {
                imu_addr = addr;
            }
        }
    }

    uint8_t who = 0;
    const bool who_ok = imu_addr && regRead(imu_addr, REG_WHO_AM_I, &who, 1);

    // SW_RESET characterisation. The driver's configure() now issues SW_RESET
    // first, because the part is not reset by an RP2350 reset and otherwise
    // carries the previous boot's INT1_CTRL across a reflash -- leaving a
    // latched INT1 that a rising-edge IRQ can never see. That fix was written
    // without a datasheet self-clear time, and low-level-nav then latched
    // init_failed on hardware with no log output to say which check failed.
    // This isolates it: how long the bit takes to clear, whether it clears at
    // all, and what INT1_CTRL and CTRL3_C read afterwards.
    bool sw_reset_written = false;
    bool sw_reset_cleared = false;
    int sw_reset_ms = -1;
    uint8_t int1_after_reset = 0xFF;
    uint8_t ctrl3_after_reset = 0xFF;
    if (who_ok && who == LSM6DSOX_WHO_AM_I)
    {
        sw_reset_written = regWrite(imu_addr, REG_CTRL3_C, CTRL3_C_SW_RESET);
        if (sw_reset_written)
        {
            for (int ms = 0; ms <= 50; ms++)
            {
                uint8_t c3 = 0xFF;
                if (regRead(imu_addr, REG_CTRL3_C, &c3, 1) && (c3 & CTRL3_C_SW_RESET) == 0)
                {
                    sw_reset_cleared = true;
                    sw_reset_ms = ms;
                    break;
                }
                sleep_ms(1);
            }
            regRead(imu_addr, REG_INT1_CTRL, &int1_after_reset, 1);
            regRead(imu_addr, REG_CTRL3_C, &ctrl3_after_reset, 1);
        }
    }

    bool cfg_written = false;
    bool cfg_verified = false;
    uint8_t rb[5] = {0};
    if (who_ok && who == LSM6DSOX_WHO_AM_I)
    {
        cfg_written = regWrite(imu_addr, REG_CTRL3_C, CFG_CTRL3_C) &&
                      regWrite(imu_addr, REG_CTRL4_C, CFG_CTRL4_C) &&
                      regWrite(imu_addr, REG_CTRL1_XL, CFG_CTRL1_XL) &&
                      regWrite(imu_addr, REG_CTRL2_G, CFG_CTRL2_G) &&
                      regWrite(imu_addr, REG_INT1_CTRL, CFG_INT1_CTRL);
        // Ton is ~35 ms from power-down, and DRDY_MASK holds data-ready off
        // until the filters settle, so give it room before believing anything.
        sleep_ms(100);
        cfg_verified = regRead(imu_addr, REG_CTRL1_XL, &rb[0], 1) &&
                       regRead(imu_addr, REG_CTRL2_G, &rb[1], 1) &&
                       regRead(imu_addr, REG_CTRL3_C, &rb[2], 1) &&
                       regRead(imu_addr, REG_CTRL4_C, &rb[3], 1) &&
                       regRead(imu_addr, REG_INT1_CTRL, &rb[4], 1);
    }

    // Stage 4. Count INT1 edges for one second WHILE draining the sensor.
    //
    // The draining is the whole point, and getting this wrong on 2026-09-09
    // produced a false "INT1 DEAD" against wiring that was fine. Data-ready is
    // LATCHED by default: INT1 asserts when a sample is ready and de-asserts
    // only when the OUTPUT registers are read. Polling STATUS_REG does not
    // clear it. A loop that only polls therefore sees INT1 go high once --
    // during the settle above, before the IRQ is even armed -- and stay high,
    // which reads as zero rising edges and is indistinguishable from a
    // disconnected pin.
    //
    // So the loop reads the 12 output bytes every time data is ready, exactly
    // as the real driver will. Then INT1 pulses once per sample, and the
    // comparison that matters is edges against drains: equal means the wire is
    // good, drains without edges means it is not.
    unsigned drains = 0;
    bool int1_before = false;
    if (cfg_written)
    {
        int1_before = gpio_get(INT1_PIN);
        edge_count = 0;
        gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_EDGE_RISE, true, &int1Isr);

        const absolute_time_t deadline = make_timeout_time_ms(1000);
        while (!time_reached(deadline))
        {
            uint8_t st;
            uint8_t raw[12];
            if (regRead(imu_addr, REG_STATUS, &st, 1) && (st & 0x03) == 0x03 &&
                regRead(imu_addr, REG_OUTX_L_G, raw, sizeof(raw)))
            {
                drains++;
            }
        }
        gpio_set_irq_enabled(INT1_PIN, GPIO_IRQ_EDGE_RISE, false);
    }

    const uint32_t edges = edge_count;
    const uint32_t stored = edges < MAX_EDGES ? edges : MAX_EDGES;
    uint32_t gap_min = 0xFFFFFFFF;
    uint32_t gap_max = 0;
    uint32_t gap_sum = 0;
    for (uint32_t i = 1; i < stored; i++)
    {
        const uint32_t d = edge_us[i] - edge_us[i - 1];
        gap_min = d < gap_min ? d : gap_min;
        gap_max = d > gap_max ? d : gap_max;
        gap_sum += d;
    }
    const uint32_t gap_mean = stored > 1 ? gap_sum / (stored - 1) : 0;

    // Stage 5. Five bursts, each taken after a fresh data-ready.
    int16_t s[5][6];
    unsigned samples = 0;
    if (cfg_written)
    {
        for (unsigned i = 0; i < 5; i++)
        {
            uint8_t st = 0;
            const absolute_time_t give_up = make_timeout_time_ms(50);
            while (!time_reached(give_up))
            {
                if (regRead(imu_addr, REG_STATUS, &st, 1) && (st & 0x03) == 0x03)
                {
                    break;
                }
            }
            uint8_t raw[12];
            if (!regRead(imu_addr, REG_OUTX_L_G, raw, sizeof(raw)))
            {
                break;
            }
            for (unsigned a = 0; a < 6; a++)
            {
                s[i][a] = (int16_t)((uint16_t)raw[2 * a] | ((uint16_t)raw[2 * a + 1] << 8));
            }
            samples++;
        }
    }

    for (;;)
    {
        printf("\n=== imu-probe: LSM6DSOX on I2C0 GP%d/GP%d, INT1 GP%d ===\n", SDA_PIN, SCL_PIN,
               INT1_PIN);

        printf("\n[0] Pad levels before any peripheral was configured\n");
        printf("      GP%-2d SDA   %s\n", SDA_PIN, idle_sda ? "HIGH" : "LOW");
        printf("      GP%-2d SCL   %s\n", SCL_PIN, idle_scl ? "HIGH" : "LOW");
        printf("      GP%-2d INT1  %s\n", INT1_PIN, idle_int1 ? "HIGH" : "LOW");
        printf("      LOW on SDA/SCL means no pull-up, or a short. HIGH is NOT proof of a\n");
        printf("      pull-up: this is A2 silicon, and under RP2350-E9 a floating Bank 0 pad\n");
        printf("      leaks toward ~2.2 V and reads HIGH. Stage 1 is the real evidence.\n");

        printf("\n[1] Bus scan at %d kHz\n", I2C_BAUD / 1000);
        printf("      expecting 0x6A (LSM6DSOX) and 0x1C (LIS3MDL): both address jumpers open\n");
        if (found_n == 0)
        {
            printf("      nothing ACKed. Check pull-ups, 3V3, GND, and that SDA/SCL are not\n");
            printf("      swapped -- a swap looks exactly like this.\n");
        }
        for (unsigned i = 0; i < found_n; i++)
        {
            printf("      0x%02X  ACK   (%s)\n", found[i], addrNote(found[i]));
        }

        printf("\n[2] Identity\n");
        if (!imu_addr)
        {
            printf("      no LSM6DSOX at 0x6A or 0x6B -- nothing further can be tested.\n");
        }
        else
        {
            printf("      WHO_AM_I @0x%02X = 0x%02X, expected 0x%02X   %s\n", imu_addr, who,
                   LSM6DSOX_WHO_AM_I, (who_ok && who == LSM6DSOX_WHO_AM_I) ? "PASS" : "FAIL");
            printf("      (a LIS3MDL, if present, should read 0x%02X at its own address)\n",
                   LIS3MDL_WHO_AM_I);
        }

        printf("\n[2b] SW_RESET behaviour (the driver now depends on this)\n");
        printf("      write CTRL3_C=0x01   %s\n", sw_reset_written ? "ACKed" : "FAILED");
        if (sw_reset_written)
        {
            if (sw_reset_cleared)
            {
                printf("      self-cleared after   %d ms\n", sw_reset_ms);
            }
            else
            {
                printf("      >> DID NOT self-clear within 50 ms. The driver polls for 20 ms\n");
                printf("         and fails configure() if it does not, which latches\n");
                printf("         init_failed and refuses to arm.\n");
            }
            printf("      INT1_CTRL after      0x%02X  (want 0x00 -- this is the point of\n",
                   int1_after_reset);
            printf("                           the reset: nothing routed to the pin)\n");
            printf("      CTRL3_C after        0x%02X\n", ctrl3_after_reset);
        }

        printf("\n[3] Configuration readback\n");
        if (!cfg_written)
        {
            printf("      not attempted.\n");
        }
        else if (!cfg_verified)
        {
            printf("      readback failed -- the bus stopped ACKing after the writes.\n");
        }
        else
        {
            const uint8_t want[5] = {CFG_CTRL1_XL, CFG_CTRL2_G, CFG_CTRL3_C, CFG_CTRL4_C,
                                     CFG_INT1_CTRL};
            const char* names[5] = {"CTRL1_XL", "CTRL2_G ", "CTRL3_C ", "CTRL4_C ", "INT1_CTRL"};
            for (unsigned i = 0; i < 5; i++)
            {
                printf("      %s  wrote 0x%02X  read 0x%02X  %s\n", names[i], want[i], rb[i],
                       rb[i] == want[i] ? "ok" : "MISMATCH");
            }
        }

        printf("\n[4] INT1 data-ready over 1 s (expect ~%d Hz, or ~%d if accel and gyro\n",
               CFG_ODR_HZ, 2 * CFG_ODR_HZ);
        printf("    data-ready do not coincide -- INT1 carries the OR of both)\n");
        printf("      edges          %lu\n", (unsigned long)edges);
        if (stored > 1)
        {
            printf("      period us      min %lu  mean %lu  max %lu\n", (unsigned long)gap_min,
                   (unsigned long)gap_mean, (unsigned long)gap_max);
            printf("      spread         %lu us (sensor ODR stability as seen at the pin; this\n",
                   (unsigned long)(gap_max - gap_min));
            printf("                     bounds capture jitter from above, it is not capture\n");
            printf("                     jitter on its own)\n");
        }
        printf("      samples drained     %u\n", drains);
        printf("      INT1 level before arming the IRQ: %s\n", int1_before ? "HIGH" : "LOW");
        printf("      (HIGH is expected -- data-ready latches. It is not proof of a\n");
        printf("       connection: under RP2350-E9 a floating pad reads HIGH too.)\n");
        if (drains == 0)
        {
            printf("      >> No data drained at all. Suspect configuration, not wiring.\n");
        }
        else if (edges == 0)
        {
            printf("      >> %u samples drained and NOT ONE edge on GP%d. The sensor is\n", drains,
                   INT1_PIN);
            printf("         producing data and the pin is silent: that is the wire.\n");
        }
        else if (edges + edges / 4 < drains)
        {
            printf("      >> Edges well below drains. A marginal or intermittent connection,\n");
            printf("         or a second INT1 consumer contending for the pad.\n");
        }

        printf("\n[5] Raw counts, stationary (gyro XYZ then accel XYZ)\n");
        for (unsigned i = 0; i < samples; i++)
        {
            const double ax = s[i][3], ay = s[i][4], az = s[i][5];
            const double g = sqrt(ax * ax + ay * ay + az * az) * 0.122 / 1000.0;
            printf("      g %6d %6d %6d   a %6d %6d %6d   |a| = %.3f g\n", s[i][0], s[i][1],
                   s[i][2], s[i][3], s[i][4], s[i][5], g);
        }
        if (samples)
        {
            printf("      At +-4 g, 1 g is ~8192 counts. |a| far from 1.000 with the board\n");
            printf("      still means the full-scale setting and the maths disagree.\n");
        }

        printf("\n  VERDICT: ");
        if (!imu_addr)
        {
            printf("NO SENSOR ON THE BUS\n");
        }
        else if (who != LSM6DSOX_WHO_AM_I)
        {
            printf("SOMETHING ANSWERS, BUT IT IS NOT AN LSM6DSOX\n");
        }
        else if (edges == 0)
        {
            printf("SENSOR ALIVE, INT1 DEAD -- data-ready timestamping is blocked\n");
        }
        else
        {
            printf("SENSOR ALIVE AND INT1 LIVE\n");
        }

        sleep_ms(5000);
    }
}
