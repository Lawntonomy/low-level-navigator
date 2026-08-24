#include "encoder.hpp"
#include "utility/logger.h"

#define CAPTURE_DEPTH 8
#define CAPTURE_RING_BITS 5

// Written by DMA, read by the control task. `volatile` because the compiler is
// otherwise free to hoist or cache these loads across the polling loop — there
// is no sequence point telling it the memory changes behind its back.
//
// Aligned for the DMA ring address warp: CAPTURE_RING_BITS = 5 gives a 32-byte
// window, which is exactly CAPTURE_DEPTH * sizeof(uint32_t).
volatile uint32_t left_buffer[CAPTURE_DEPTH] __attribute__((aligned(32)));
volatile uint32_t right_buffer[CAPTURE_DEPTH] __attribute__((aligned(32)));

static const char* category = "encoder";

namespace
{

// Per-wheel capture state.
//
// The DMA channel number is kept because it is the ONLY way to tell a stopped
// wheel from a stale buffer. `encoder.pio` is a period counter: when the wheel
// stops, pushes stop, DMA stops writing, and the ring holds its last values
// indefinitely. Watching the channel's write pointer is what makes standstill
// observable at all (issue #12, TP-0002 CAL-0).
struct Capture
{
    volatile uint32_t* buffer;
    int dma_chan;
    uint32_t last_write_addr;
    uint64_t last_change_us;
};

Capture left{left_buffer, -1, 0, 0};
Capture right{right_buffer, -1, 0, 0};

int setup_dma(PIO pio, uint sm, volatile uint32_t* array)
{
    const int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, CAPTURE_RING_BITS);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    // 0xFFFFFFFF selects MODE=0xF (ENDLESS) with COUNT=0x0FFFFFFF. On RP2350
    // bits 31:28 of TRANS_COUNT are a MODE field (datasheet 12.6.1), and
    // ENDLESS runs the channel forever until an explicit abort — which is what
    // this capture wants.
    //
    // The consequence to know about: in ENDLESS mode the count **does not
    // decrement**, so transfer_count is frozen and carries no information. It
    // cannot be used to detect that a sample arrived, which is why the
    // freshness check below reads write_addr instead.
    //
    // Not portable to RP2040, where TRANS_COUNT has no MODE field and this same
    // value means "count down from 2^32-1" before halting.
    dma_channel_configure(dma_chan, &c,
                          const_cast<uint32_t*>(array), // destination ring
                          &pio->rxf[sm],                // source: PIO RX FIFO
                          0xFFFFFFFF,                   // MODE=ENDLESS, see above
                          true                          // start immediately
    );

    return dma_chan;
}

// Samples the write pointer and reports whether the stream is still live.
//
// **On aliasing.** The ring is 5 bits, so write_addr cycles through 8 values
// and repeats every 8 samples: a poll interval spanning exactly 8, 16, 24 ...
// samples would see no change on a fast-spinning wheel and wrongly call it
// stopped — the dangerous direction of error. At the 5 ms control period that
// needs 8 samples in 5 ms = 1600 edges/s = 4800 rpm, which this drivetrain
// cannot reach, so it cannot occur. **That safety margin is a property of the
// poll rate.** If the control period is ever lengthened, redo this arithmetic.
bool poll_fresh(Capture& cap)
{
    const uint32_t addr = dma_channel_hw_addr(cap.dma_chan)->write_addr;
    const uint64_t now = time_us_64();

    if (addr != cap.last_write_addr)
    {
        cap.last_write_addr = addr;
        cap.last_change_us = now;
    }

    return encoder::is_fresh(now, cap.last_change_us);
}

encoder::Reading read(Capture& cap)
{
    if (cap.dma_chan < 0 || !poll_fresh(cap))
    {
        return {0.0f, false};
    }

    // Snapshot before converting: the ring is written by DMA underneath us, and
    // summing it in place could mix samples from either side of a wrap.
    uint32_t snapshot[CAPTURE_DEPTH];
    for (int i = 0; i < CAPTURE_DEPTH; ++i)
    {
        snapshot[i] = cap.buffer[i];
    }

    return {encoder::rpm_from_periods(snapshot, CAPTURE_DEPTH), true};
}

} // namespace

void encoder::init(PIO pio, uint sm_index)
{
    Log::info(category, "init encoder");

    const int offset1 = pio_add_program(pio, &encoder_program);
    encoder_program_init(pio, sm_index, offset1, static_cast<uint>(gpio::pins::left_encoder_pin));
    left.dma_chan = setup_dma(pio, sm_index, left_buffer);

    encoder_program_init(pio, sm_index + 1, offset1,
                         static_cast<uint>(gpio::pins::right_encoder_pin));
    right.dma_chan = setup_dma(pio, sm_index + 1, right_buffer);

    // Seed freshness from the channels themselves, so a wheel that never turns
    // reads invalid from the first poll rather than inheriting a zero stamp.
    const uint64_t now = time_us_64();
    left.last_write_addr = dma_channel_hw_addr(left.dma_chan)->write_addr;
    right.last_write_addr = dma_channel_hw_addr(right.dma_chan)->write_addr;
    left.last_change_us = now;
    right.last_change_us = now;

    Log::info(category, "init encoder complete");
}

encoder::Reading encoder::read_left()
{
    return read(left);
}

encoder::Reading encoder::read_right()
{
    return read(right);
}
