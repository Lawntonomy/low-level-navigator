#include "encoder.hpp"
#include "utility/logger.h"

#define CAPTURE_DEPTH 8
#define CAPTURE_RING_BITS 5

// Aligned for the DMA ring address warp.
uint32_t left_buffer[CAPTURE_DEPTH] __attribute__((aligned(32)));
uint32_t right_buffer[CAPTURE_DEPTH] __attribute__((aligned(32)));

// #define ARRAY_SIZE 8
static const char* category = "encoder";

// uint32_t left_buffer[ARRAY_SIZE] = {0};
// uint32_t right_buffer[ARRAY_SIZE] = {0};
void setup_dma(PIO pio, uint sm, uint32_t* array)
{
    // DMA channel configuration
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, CAPTURE_RING_BITS);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    // Configure DMA
    dma_channel_configure(dma_chan, &c,
                          array,         // Destination pointer (buffer array)
                          &pio->rxf[sm], // Source pointer (PIO RX FIFO)
                          0xFFFFFFFF,    // Transfer count
                          true           // Start immediately
    );

    // Set DREQ to trigger DMA when FIFO has data
}

void encoder::init(PIO pio, uint sm_index)
{
    Log::info(category, "init encoder");
    int offset1 = pio_add_program(pio, &encoder_program);
    assert(offset1 > 0);
    encoder_program_init(pio, sm_index, offset1, static_cast<uint>(gpio::pins::left_encoder_pin));
    setup_dma(pio, sm_index, left_buffer);

    encoder_program_init(pio, sm_index + 1, offset1,
                         static_cast<uint>(gpio::pins::right_encoder_pin));
    setup_dma(pio, sm_index + 1, right_buffer);
    // setup_dma(pio, sm_index, left_buffer);
    // setup_dma(pio, sm_index + 1, right_buffer);
    Log::info(category, "init encoder complete");
}

const float cycles_per_loop = 5.0;

float encoder::get_left_rpm(PIO pio, uint sm)
{
    float total_for_average = 0.0;
    for (int i = 0; i < CAPTURE_DEPTH; i++)
    {
        total_for_average += left_buffer[i];
    }

    if (total_for_average > 0.0)
    {
        float rpm = (60.0f * 1000.0f) / ((float)(total_for_average / CAPTURE_DEPTH) * 20);
        return rpm;
    }
    return 0.0;
}

float encoder::get_right_rpm(PIO pio, uint sm)
{
    float total_for_average = 0.0;
    for (int i = 0; i < CAPTURE_DEPTH; i++)
    {
        total_for_average += right_buffer[i];
    }

    if (total_for_average > 0.0)
    {
        float rpm = (60.0f * 1000.0f) / ((float)(total_for_average / CAPTURE_DEPTH) * 20);
        return rpm;
    }
    return 0.0;
}
