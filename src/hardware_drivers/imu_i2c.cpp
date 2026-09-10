#include "imu_i2c.hpp"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lsm6dsox_regs.h"
#include "pico/stdlib.h"

#include "utility/logger.h"

namespace
{

const char* category = "imu_i2c";

// Wiring, confirmed on hardware 2026-09-09 by diagnostics/imu-probe and
// recorded in gpio_defines.h. RP2350 Table 3 fixes the mod-4 pattern that makes
// GP12/GP13 an I2C0 SDA/SCL pair; I2C0 rather than I2C1 is arbitrary and leaves
// I2C1 free.
constexpr uint sda_pin = 12;
constexpr uint scl_pin = 13;

// 400 kHz is a ceiling, not a choice. The LSM6DSOX slave timing table lists
// Standard and Fast only -- there is no Fast-mode-plus row -- even though the
// RP2350 master supports 1000 kb/s.
constexpr uint baud_hz = 400 * 1000;

// Bound on the blocking init-time helpers. Generous by two orders of magnitude
// against a single-byte transaction at 400 kHz (~50 us); it exists so a wedged
// bus fails a check instead of hanging, not to police timing.
constexpr uint init_timeout_us = 10000;

// IC_DATA_CMD command bits. RP2350 datasheet section 12.2.7, Figure 76
// (printed page 994): DATA[7:0], CMD bit 8 (1 = read, 0 = write), Stop bit 9,
// Restart bit 10.
constexpr uint32_t cmd_read = I2C_IC_DATA_CMD_CMD_BITS;
constexpr uint32_t cmd_stop = I2C_IC_DATA_CMD_STOP_BITS;
constexpr uint32_t cmd_restart = I2C_IC_DATA_CMD_RESTART_BITS;

// DMA destination. `volatile` for the same reason encoder.cpp's rings are: the
// compiler has no way to know this memory changes behind its back.
volatile uint8_t burst_buf[imu_i2c::burst_bytes];

int rx_chan = -1;
bool ready = false;

// Points IC_TAR at a device. The DW_apb_i2c requires the block to be disabled
// while IC_TAR changes, which is why the SDK's blocking calls do exactly this
// dance on every transfer.
//
// **Only ever called with the bus idle.** Disabling the block mid-transaction
// would abort it. startBurstRead() calls this at most once, and normally not
// at all -- see the note there.
void setTarget(uint8_t deviceAddr)
{
    i2c_hw_t* hw = i2c_get_hw(i2c0);
    hw->enable = 0;
    hw->tar = deviceAddr;
    hw->enable = 1;
}

} // namespace

bool imu_i2c::init()
{
    Log::info(category, "init imu i2c");

    i2c_init(i2c0, baud_hz);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    // Deliberately NO gpio_pull_up() on either pin.
    //
    // RP2350 datasheet section 12.2.1.3 (printed page 985) says the pads should
    // be pull-up enabled, and then immediately notes that there should also be
    // external pull-ups "as the internal pad pull-ups may not be strong enough
    // to pull up external circuits". External resistors are fitted and the bus
    // was confirmed working on them alone by diagnostics/imu-probe. Enabling
    // the internal ones in addition would buy nothing and would hide a missing
    // or lifted external resistor until a longer cable or a faster edge exposed
    // it -- a fault that then presents as intermittent corrupt samples rather
    // than a dead bus.

    // i2c_init sets both TDMAE and RDMAE unconditionally, so the DREQ handshake
    // is already live and nothing further is needed to enable it. The DMA
    // watermark registers are also correct at their reset value of zero:
    // section 12.2.15.3 (printed page 1008) states IC_DMA_TDLR and IC_DMA_RDLR
    // "can be left at reset values of zero ... because only single transfers
    // are needed due to the low bandwidth of I2C". So one DREQ per byte, which
    // is what a byte-sized RX channel wants.

    // The TX strategy below rests on the FIFO being deep enough to hold every
    // command word of a burst. Check it once, here, rather than assuming it:
    // with the FIFO empty, i2c_get_write_available() returns the depth itself.
    const std::size_t depth = i2c_get_write_available(i2c0);
    if (depth < burst_cmd_words)
    {
        Log::error(category, "i2c0 TX FIFO too shallow for a burst");
        return false;
    }

    rx_chan = dma_claim_unused_channel(false);
    if (rx_chan < 0)
    {
        Log::error(category, "no free DMA channel for imu rx");
        return false;
    }

    dma_channel_config c = dma_channel_get_default_config(rx_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false); // always the same FIFO register
    channel_config_set_write_increment(&c, true); // walk the sample buffer
    channel_config_set_dreq(&c, i2c_get_dreq(i2c0, false));

    // **MODE is 0x0 (NORMAL), and that is not an oversight.**
    //
    // encoder.cpp configures its channels with a count of 0xFFFFFFFF, which on
    // RP2350 selects MODE = 0xF, ENDLESS (datasheet section 12.6.2.2.1, printed
    // page 1097 -- bits 31:28 of TRANS_COUNT are a MODE field, unlike RP2040).
    // That is right for the encoders and wrong here, and the difference is
    // structural rather than a matter of taste. The encoder is producer-driven:
    // the PIO pushes whenever a wheel transition occurs and nothing has to ask,
    // so an endless ring simply catches whatever arrives. I2C is
    // master-initiated -- section 12.2.7 requires one IC_DATA_CMD write per
    // byte read -- so an endless RX channel would remove none of the per-sample
    // cost, which *is* those command writes, and would give up both the
    // completion signal and the count.
    //
    // The payoff of NORMAL mode: section 12.6.2.2 says reading TRANS_COUNT
    // "returns the number of transfers remaining in the current transfer
    // sequence. This value updates continuously as the channel progresses."
    // The frozen-count behaviour that is familiar from encoder.cpp belongs only
    // to ENDLESS. Here the count really does walk 12 -> 0, so burstRemaining()
    // is a live progress indicator. Read the encoder comment and this one
    // together before copying either.
    //
    // Not triggered here: init() must leave the bus quiet. The reload value of
    // burst_bytes is latched now and reused on every trigger, so
    // startBurstRead() only has to reset the write address.
    dma_channel_configure(rx_chan, &c,
                          const_cast<uint8_t*>(burst_buf), // destination
                          &i2c_get_hw(i2c0)->data_cmd,     // source: RX FIFO
                          burst_bytes,                     // MODE = 0x0, 12 transfers
                          false                            // do not start
    );

    ready = true;
    Log::info(category, "init imu i2c complete");
    return true;
}

bool imu_i2c::writeReg(uint8_t deviceAddr, uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(i2c0, deviceAddr, buf, 2, false, init_timeout_us) == 2;
}

bool imu_i2c::readReg(uint8_t deviceAddr, uint8_t reg, uint8_t* dst, std::size_t len)
{
    if (dst == nullptr || len == 0)
    {
        return false;
    }

    // nostop = true on the write, so the read that follows issues a RESTART
    // rather than releasing the bus between the sub-address and the data.
    if (i2c_write_timeout_us(i2c0, deviceAddr, &reg, 1, true, init_timeout_us) != 1)
    {
        return false;
    }
    return i2c_read_timeout_us(i2c0, deviceAddr, dst, len, false, init_timeout_us) ==
           static_cast<int>(len);
}

// Upper bound on bytes the RX FIFO can be holding. The DW_apb_i2c RX and TX
// FIFOs are both 16 deep on RP2350; init() verifies the TX depth directly, and
// this is the same number used as a drain ceiling.
static constexpr std::size_t fifo_depth_guard = 16;

bool imu_i2c::startBurstRead()
{
    if (!ready || rx_chan < 0)
    {
        return false;
    }

    // A burst still in flight means the previous one never completed. Re-arming
    // would move the write pointer under a running transfer and splice two
    // samples together, which is worse than losing one: the result looks like a
    // valid sample. Refuse, and let the timeout in imu_stall.hpp be what
    // notices.
    if (dma_channel_is_busy(rx_chan))
    {
        return false;
    }

    i2c_hw_t* hw = i2c_get_hw(i2c0);

    // Normally a no-op: init-time register access leaves IC_TAR pointing at
    // whichever device it last addressed, and after a normal startup that is
    // the LSM6DSOX. It is checked rather than assumed because the LIS3MDL at
    // 0x1C shares this bus, and a burst issued to the wrong target would return
    // twelve plausible-looking bytes from the wrong sensor.
    //
    // The check is conditional so that the steady-state path does not disable
    // and re-enable the I2C block on every sample. It is also the reason this
    // driver assumes exclusive ownership of i2c0 once bursts are running: a
    // concurrent LIS3MDL transaction from a task could be aborted mid-transfer
    // by this. Nothing reads the magnetometer yet; whatever does will need a
    // lock, and that belongs with that work rather than here.
    if (hw->tar != lsm6dsox::device_addr)
    {
        setTarget(lsm6dsox::device_addr);
    }

    // Drain anything the RX FIFO is still holding before arming the channel.
    // A leftover byte from an aborted transaction would be consumed as byte 0
    // of this sample and shift all twelve, turning a recoverable fault into
    // silently wrong gyro and accel values.
    // Bounded, and the bound is not decoration. Under the rt.h decision this
    // function is also called from the control task on core 1 as the stall
    // recovery, so an unbounded loop here is an unbounded loop on the path that
    // feeds the watchdog: the control task would spin, watchdog_update() would
    // never be reached, and the 100 ms reset returns the pads to a state where
    // the TB6612 reads STBY high with the direction pins low -- coasting, not
    // stopped. The RX FIFO is the same depth as the TX FIFO, so anything beyond
    // that many bytes means the peripheral is not in the state this code thinks
    // it is; fail the call rather than spin.
    for (std::size_t drained = 0; i2c_get_read_available(i2c0) > 0; ++drained)
    {
        if (drained >= fifo_depth_guard)
        {
            return false;
        }
        (void)hw->data_cmd;
    }

    if (i2c_get_write_available(i2c0) < burst_cmd_words)
    {
        return false;
    }

    // Arm RX before pushing TX. Nothing can arrive until the command words go
    // out and the bus turns around, so the order is not strictly forced -- but
    // this way there is no window in which returned bytes have nowhere to go.
    dma_channel_set_write_addr(rx_chan, const_cast<uint8_t*>(burst_buf), true);

    // **The TX side is deliberately FIFO-direct, with no DMA channel.**
    //
    // Reading a register is: write the sub-address, RESTART, then N reads -- and
    // the DW_apb_i2c wants one IC_DATA_CMD write per byte received (section
    // 12.2.7, Figure 76). So a 12-byte burst is 13 command words out and 12
    // bytes back; the two directions are not symmetric, and that asymmetry is
    // what makes this cheap.
    //
    // **This rests on IC_TX_BUFFER_DEPTH == 16** (pico-sdk
    // hardware/i2c.h:370). All 13 words fit at once, so pushing them is 13
    // register writes that never wait on the bus -- microseconds, safe in
    // interrupt context. A TX DMA channel would buy nothing here and would cost
    // a channel plus a configuration to get wrong. init() verifies the depth
    // and the guard above re-checks the free space, so if that assumption ever
    // stops holding this fails a check rather than silently stalling SCL: a TX
    // FIFO that empties mid-transaction does not generate a STOP, it holds SCL
    // low and wedges the bus (section 12.2.7.1).
    hw->data_cmd = lsm6dsox::reg_outx_l_g; // write sub-address, no Stop

    for (std::size_t i = 0; i < burst_bytes; ++i)
    {
        uint32_t word = cmd_read;
        if (i == 0)
        {
            word |= cmd_restart; // turn the bus around after the sub-address
        }
        if (i == burst_bytes - 1)
        {
            word |= cmd_stop; // release the bus after the last byte
        }
        hw->data_cmd = word;
    }

    return true;
}

bool imu_i2c::burstBusy()
{
    return rx_chan >= 0 && dma_channel_is_busy(rx_chan);
}

uint32_t imu_i2c::burstRemaining()
{
    if (rx_chan < 0)
    {
        return 0;
    }
    return dma_channel_hw_addr(rx_chan)->transfer_count;
}

const volatile uint8_t* imu_i2c::burstBuffer()
{
    return burst_buf;
}

int imu_i2c::rxChannel()
{
    return rx_chan;
}
