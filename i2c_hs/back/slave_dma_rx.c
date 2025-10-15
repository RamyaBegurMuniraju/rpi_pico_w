#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// ---- I2C slave pins/addr ----
#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define I2C_ADDR        0x17

// ---- Ring + DMA block sizes ----
#define RING_BYTES          (128 * 1024)        // fits with typical apps; adjust if needed
#define DMA_BLOCK_BYTES     4096
_Static_assert(RING_BYTES % DMA_BLOCK_BYTES == 0, "ring must be multiple of block size");

static __attribute__((aligned(4))) uint8_t ring_buf[RING_BYTES];
static volatile uint32_t wr_off = 0;            // producer write offset (bytes)
static volatile uint32_t total_rx = 0;          // total bytes received
static uint32_t next_dst = 0;

static int dma_ch = -1;

static inline uint32_t ring_avail(uint32_t r_off){
    return (wr_off >= r_off) ? (wr_off - r_off) : (RING_BYTES - (r_off - wr_off));
}

static void start_dma(uint32_t dst_off){
    volatile uint32_t *src = &i2c_get_hw(I2C_PORT)->data_cmd; // pops RX FIFO

    dma_channel_config c = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, I2C_PORT == i2c0 ? DREQ_I2C0_RX : DREQ_I2C1_RX);

    dma_channel_configure(
        dma_ch, &c,
        &ring_buf[0] + dst_off,
        src,
        DMA_BLOCK_BYTES,
        false
    );

    dma_channel_set_irq0_enabled(dma_ch, true);
    dma_start_channel_mask(1u << dma_ch);
}

static void __isr dma_irq_handler(void){
    if (dma_channel_get_irq0_status(dma_ch)){
        dma_channel_acknowledge_irq0(dma_ch);

        // one block completed at next_dst
        wr_off   = (wr_off + DMA_BLOCK_BYTES) % RING_BYTES;
        total_rx += DMA_BLOCK_BYTES;

        next_dst = (next_dst + DMA_BLOCK_BYTES) % RING_BYTES;
        start_dma(next_dst);
    }
}

static void i2c_slave_init_with_dma(void){
    // Pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Enable peripheral & slave mode
    i2c_init(I2C_PORT, 400000); // freq doesn't matter in slave; 400 kHz is stable
    i2c_set_slave_mode(I2C_PORT, true, I2C_ADDR);

    // Enable RX-DMA on the I2C block
    i2c_hw_t *hw = i2c_get_hw(I2C_PORT);
    hw->dma_rdlr = 0;                                      // DREQ on >=1 byte
    hw_set_bits(&hw->dma_cr, I2C_IC_DMA_CR_RDMAE_BITS);    // RX-DMA enable
    hw_clear_bits(&hw->dma_cr, I2C_IC_DMA_CR_TDMAE_BITS);  // TX-DMA off

    // Drain any stale RX
    while (i2c_get_read_available(I2C_PORT)) (void)hw->data_cmd;

    // DMA channel + IRQ
    dma_ch = dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(dma_ch, true);

    wr_off = 0; next_dst = 0; total_rx = 0;
    start_dma(next_dst);
}

int main(void){
    stdio_init_all();
    sleep_ms(300);
    printf("I2C slave + RX-DMA ring, addr=0x%02X, ring=%u KB, block=%u\n",
           I2C_ADDR, RING_BYTES/1024, DMA_BLOCK_BYTES);

    i2c_slave_init_with_dma();

    absolute_time_t last = get_absolute_time();
    while (1){
        tight_loop_contents();

        if (absolute_time_diff_us(last, get_absolute_time()) > 1000000){
            last = get_absolute_time();
            // Peek last 4 bytes of the last fully-written block for sanity
            uint32_t prev = (wr_off + RING_BYTES - DMA_BLOCK_BYTES) % RING_BYTES;
            printf("total_rx=%u bytes  wr_off=%u  last: %02x %02x %02x %02x\n",
                   total_rx, wr_off,
                   ring_buf[(prev+DMA_BLOCK_BYTES-4) % RING_BYTES],
                   ring_buf[(prev+DMA_BLOCK_BYTES-3) % RING_BYTES],
                   ring_buf[(prev+DMA_BLOCK_BYTES-2) % RING_BYTES],
                   ring_buf[(prev+DMA_BLOCK_BYTES-1) % RING_BYTES]);
        }
    }
}

