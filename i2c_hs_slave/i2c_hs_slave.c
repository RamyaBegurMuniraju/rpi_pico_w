#include <stdio.h>
#include "i2c_hs_slave.h"
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "i2c_hs_slave.pio.h"

// ---- Configurable ring size ----
#ifndef HS_SLAVE_RING_BYTES
//#define HS_SLAVE_RING_BYTES (256 * 1024)  // 256 KB cushion for 3.4 MHz bursts over run RAM
#define HS_SLAVE_RING_BYTES (128 * 1024)  // 256 KB cushion for 3.4 MHz bursts
#endif

// ---- Ring buffer ----
static uint8_t ring_buf[HS_SLAVE_RING_BYTES];
static volatile uint32_t wr_idx = 0, rd_idx = 0;
static inline uint32_t rmask(uint32_t v){ return v & (HS_SLAVE_RING_BYTES - 1); }
static inline uint32_t ring_count(void){ return (wr_idx - rd_idx) & (HS_SLAVE_RING_BYTES - 1); }
uint32_t i2c_hs_slave_ring_free(void){ return (HS_SLAVE_RING_BYTES - 1) - ring_count(); }

// ---- STOP detection ----
static uint sda_pin_g = 0, scl_pin_g = 1;
static volatile bool stop_flag = false;
static void gpio_irq_isr(uint gpio, uint32_t events){
    (void)gpio;
    if ((events & GPIO_IRQ_EDGE_RISE) && gpio == sda_pin_g){
        if (gpio_get(scl_pin_g)) stop_flag = true;
    }
}
bool i2c_hs_slave_stop_seen(void){ return stop_flag; }
void i2c_hs_slave_clear_stop(void){ stop_flag = false; }

// ---- DMA: PIO RX FIFO -> ring (byte-wise, HW-wrapped) ----
static int dma_chan_g = -1;
static void __isr dma_irq_handler(void){
    dma_hw->ints0 = 1u << dma_chan_g;
    // We configured a fixed batch size; advance wr_idx by that amount
    wr_idx += 256;
    printf("// Restart the channel\n");
    dma_channel_set_transfer_count(dma_chan_g, 256, true);
}

void i2c_hs_slave_init(i2c_hs_slave_t* ctx, PIO pio, uint sm, uint sda_pin, uint scl_pin){
    ctx->pio = pio; ctx->sm = sm; ctx->sda_pin = sda_pin; ctx->scl_pin = scl_pin;

    sda_pin_g = sda_pin;
    scl_pin_g = scl_pin;

    // Pull-ups (use EXTERNAL strong pull-ups for HS)
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    gpio_set_dir(sda_pin, false);
    gpio_set_dir(scl_pin, false);

      // 1) Print idle levels BEFORE the PIO owns the pins
    printf("Idle levels (before PIO): SDA=%d SCL=%d\n",
           gpio_get(sda_pin), gpio_get(scl_pin));
    sleep_ms(10);

    // STOP IRQ
    gpio_set_irq_enabled_with_callback(sda_pin, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_isr);

    // Load and start PIO program
    uint off = pio_add_program(pio, &i2c_hs_slave_rx_program);
    i2c_hs_slave_rx_program_init(pio, sm, off, sda_pin, scl_pin);


      // (Optional but recommended) make sure SDA starts released and pulls low when driven
    pio_sm_set_pins_with_mask   (pio, sm, 0u, 1u << sda_pin); // SDA output value = 0
    pio_sm_set_pindirs_with_mask(pio, sm, 0u, 1u << sda_pin); // SDA direction = input

    pio_sm_set_enabled(pio, sm, true);

    // DMA: PIO RX FIFO -> ring
    dma_chan_g = dma_claim_unused_channel(true);
    ctx->dma_chan = dma_chan_g;

    dma_channel_config c = dma_channel_get_default_config(dma_chan_g);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));   // RX
    // HW ring on write address: log2(HS_SLAVE_RING_BYTES)
    // HS_SLAVE_RING_BYTES must be power of two.
    int ring_bits = 0;
    for (uint32_t n = HS_SLAVE_RING_BYTES; n > 1; n >>= 1) ring_bits++;
    channel_config_set_ring(&c, true, ring_bits);

    dma_channel_configure(
        dma_chan_g, &c,
        ring_buf, &pio->rxf[sm],
        256,                           // batch count per IRQ
        false
    );

    dma_channel_set_irq0_enabled(dma_chan_g, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_chan_g);
}

// Non-blocking get byte
int i2c_hs_slave_get_byte(void){
    if (ring_count() == 0) return -1;
    uint8_t v = ring_buf[rmask(rd_idx++)];
    return (int)v;
}

