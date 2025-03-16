#include "pwm.h"
#include "gpio_defines.h"
#include "hardware/dma.h"

uint32_t pwm_left = 0;
uint32_t pwm_right = 0;
int dma_left = 0;
int dma_right = 0;

void setup_dma(PIO pio, uint sm, uint32_t* num, int& dma_chan)
{
    // DMA channel configuration
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);

    // Configure DMA
    dma_channel_configure(dma_chan, &c,
                          &pio->txf[sm], // Destination pointer (buffer array)
                          &num,          // Source pointer (PIO RX FIFO)
                          1,             // Transfer count
                          false          // Start immediately
    );
}

void pwm::init(PIO pio, uint sm_index)
{
    int offset = pio_add_program(pio, &pwm_program);
    assert(offset > 0);
    pwm_program_init(pio, sm_index, offset, static_cast<uint>(gpio::pins::left_pwm_pin));
    pwm_program_init(pio, sm_index + 1, offset, static_cast<uint>(gpio::pins::right_pwm_pin));
    setup_dma(pio, sm_index, &pwm_left, dma_left);
    setup_dma(pio, sm_index, &pwm_right, dma_right);
}

// Write `period` to the input shift register
void pwm::pio_pwm_set_period(PIO pio, uint sm, uint32_t period)
{
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pwm::pio_pwm_set_level(PIO pio, uint sm, uint32_t level)
{
    pwm_left = level;
    dma_channel_start(dma_left);
}
