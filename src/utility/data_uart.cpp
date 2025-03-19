#include "data_uart.hpp"

#include "hardware/dma.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#define UART_ID uart1
#define BAUD_RATE 115200
#define TX_PIN 4
#define RX_PIN 5
#define BUFFER_SIZE 64

#define CAPTURE_DEPTH 1024   // (8 * 4 bytes = 32)
#define CAPTURE_RING_BITS 10 // (1 << 5 = 32)

// Aligned for the DMA ring address warp.
uint8_t ring_buffer[CAPTURE_DEPTH] __attribute__((aligned(1024)));

static const char* category = "uart1";

// DMA channel IDs
int dma_tx_chan;
int dma_rx_chan;

void uart::init()
{
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);

    // --- TX DMA Setup ---
    dma_tx_chan = dma_claim_unused_channel(true);
    dma_channel_config tx_config = dma_channel_get_default_config(dma_tx_chan);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_dreq(&tx_config, uart_get_dreq(UART_ID, true)); // TX DREQ

    dma_channel_configure(dma_tx_chan, &tx_config,
                          &uart_get_hw(UART_ID)->dr, // Destination: UART data register
                          tx_buffer,                 // Source: TX buffer
                          BUFFER_SIZE,               // Number of bytes
                          false                      // Do not start immediately
    );

    // --- RX DMA Setup ---
    // DMA channel configuration
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, CAPTURE_RING_BITS);
    channel_config_set_dreq(&c, uart_get_dreq(UART_ID, false));

    // Configure DMA
    dma_channel_configure(dma_chan, &c,
                          ring_buffer,               // Destination pointer (buffer array)
                          &uart_get_hw(UART_ID)->dr, // Source pointer (PIO RX FIFO)
                          0xFFFFFFFF,                // Transfer count
                          true                       // Start immediately
    );
}

// setup_dma_uart();
// dma_channel_start(dma_tx_chan);
