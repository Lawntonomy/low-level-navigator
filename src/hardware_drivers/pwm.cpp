#include "pwm.h"

#include "gpio_defines.h"
#include "hardware/dma.h"
#include "pwm.pio.h"
#include "utility/logger.h"

static const char* category = "pwm";

uint32_t left_pwm = 0;
uint32_t right_pwm = 0;
int left_dma = 0;
int right_dma = 0;

static int setup_dma(PIO pio, uint sm, uint32_t* value)
{

    // DMA channel configuration
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    // Configure DMA
    dma_channel_configure(dma_chan, &c,
                          &pio->txf[sm], // Destination pointer (buffer array)
                          value,         // Source pointer (PIO RX FIFO)
                          0xF0000001,    // Transfer count
                          true           // Start immediately
    );
    return dma_chan;
}

void pwm::init(PIO pio, uint sm_index)
{
    Log::info(category, "starting init");
    int offset = pio_add_program(pio, &pwm_program);
    assert(offset > 0);
    pwm_program_init(pio, sm_index, offset, static_cast<uint>(gpio::pins::left_pwm_pin));
    pwm_program_init(pio, sm_index + 1, offset, static_cast<uint>(gpio::pins::right_pwm_pin));

    left_dma = setup_dma(pio, sm_index, &left_pwm);
    right_dma = setup_dma(pio, sm_index + 1, &right_pwm);
    Log::info(category, "initialization finished");
}

// Write `period` to the input shift register
void pwm::pio_pwm_set_period(PIO pio, uint sm, bool left, uint32_t period)
{
    int dma_channel;
    if (left)
    {
        dma_channel = left_dma;
    }
    else
    {
        dma_channel = right_dma;
    }

    dma_channel_abort(dma_channel);
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
    dma_channel_start(dma_channel);
}

// pio_pwm_set_period(pio[0], sm[2], (1u << 16) - 1);
// pio_pwm_set_period(pio[0], sm[3], (1u << 16) - 1);
// Write `level` to TX FIFO. State machine will copy this into X.
void pwm::pio_pwm_set_level(bool left, uint32_t level)
{
    if (left)
    {
        left_pwm = level;
    }
    else
    {
        right_pwm = level;
    }
}
