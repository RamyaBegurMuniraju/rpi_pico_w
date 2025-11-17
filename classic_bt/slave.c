#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// ---------- I2C TARGET CONFIG ----------
#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define I2C_ADDR        0x17

// ---------- TRANSFER SIZE ----------
#define TOTAL_BYTES     (2 * 1024 * 1024)   // 2 MB

// ---------- DMA BUFFERS ----------
#define BLOCK_BYTES     (8 * 1024)          // 8 KB per DMA block
#define NUM_BLOCKS      (TOTAL_BYTES / BLOCK_BYTES)

static uint8_t __attribute__((aligned(4))) bufA[BLOCK_BYTES];
static uint8_t __attribute__((aligned(4))) bufB[BLOCK_BYTES];

// ---------- DMA STATE ----------
static int dma_a = -1, dma_b = -1;
static volatile uint32_t blocks_done = 0;
static volatile uint64_t total_rx = 0;
static volatile bool a_in_flight = false, b_in_flight = false;

static absolute_time_t t0, t1;
static volatile bool t0_set = false;


static inline void hexdump(const uint8_t *p, uint32_t n){
    for (uint32_t i = 0; i < n; i++){
        printf("%02X%s", (unsigned)p[i], ((i+1)%16) ? " " : "\n");
    }
    if (n % 16) printf("\n");
}

static void start_dma_read(int ch, uint8_t *dst, size_t nbytes) {
    dma_channel_config c = dma_channel_get_default_config(ch);
    // Read 8-bit from I2C RX FIFO register -> memory
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, i2c_get_dreq(I2C_PORT, false)); // RX DREQ
    channel_config_set_read_increment(&c, false);   // fixed source (FIFO)
    channel_config_set_write_increment(&c, true);   // increment dest
    dma_channel_configure(
        ch, &c,
        dst,                                 // dest
        &I2C_PORT->hw->data_cmd,             // src (RX FIFO)
        nbytes,                               // count in bytes
        true                                  // start immediately
    );
}

static void __not_in_flash_func(dma_irq_handler)(void) {
    // Which channel finished?
    uint32_t irq = dma_hw->ints0;
    if (irq & (1u << dma_a)) {
        dma_hw->ints0 = (1u << dma_a);
    if (!t0_set) { t0 = get_absolute_time(); t0_set = true; }

        a_in_flight = false;
        blocks_done++;
        total_rx += BLOCK_BYTES;
        if (total_rx < TOTAL_BYTES) {
            // restart A on next block
            start_dma_read(dma_a, bufA, BLOCK_BYTES);
            a_in_flight = true;
        }
    }
    if (irq & (1u << dma_b)) {
        dma_hw->ints0 = (1u << dma_b);
        b_in_flight = false;
        blocks_done++;
        total_rx += BLOCK_BYTES;
        if (total_rx < TOTAL_BYTES) {
            // restart B on next block
            start_dma_read(dma_b, bufB, BLOCK_BYTES);
            b_in_flight = true;
        }
    }
}


int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("I2C SLAVE (target) with DMA ping-pong @0x%02X, expecting %u bytes\n",
           I2C_ADDR, (unsigned)TOTAL_BYTES);

    // pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_init(I2C_PORT, 3.4 * 1000 * 1000);               // 3.4MHz is not supported by pico w slave
    //i2c_init(I2C_PORT,  1000 * 1000);               // 1 MHz 
    //i2c_init(I2C_PORT, 400000);               // 400 KHz 
    //i2c_init(I2C_PORT, 100000);               // 100 KHz 
    i2c_set_slave_mode(I2C_PORT, true, I2C_ADDR);

    // Clear RX FIFO (defensive)
    while (i2c_get_read_available(I2C_PORT)) (void)I2C_PORT->hw->data_cmd;

    // DMA channels
    dma_a = dma_claim_unused_channel(true);
    dma_b = dma_claim_unused_channel(true);

    // IRQ
    dma_channel_set_irq0_enabled(dma_a, true);
    dma_channel_set_irq0_enabled(dma_b, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Kick both DMAs to form a continuous stream
    start_dma_read(dma_a, bufA, BLOCK_BYTES); a_in_flight = true;
    start_dma_read(dma_b, bufB, BLOCK_BYTES); b_in_flight = true;
    hexdump(bufB, BLOCK_BYTES);

    //absolute_time_t t0 = get_absolute_time();
    while (total_rx < TOTAL_BYTES) {
        // Optional: progress print every ~0.5 s
        static absolute_time_t last = {0};
        if (absolute_time_diff_us(last, get_absolute_time()) > 500000) {
            last = get_absolute_time();
            printf("RX: %llu / %u bytes (blocks_done=%lu)\n",
                   (unsigned long long)total_rx, (unsigned)TOTAL_BYTES,
                   (unsigned long)blocks_done);
        }
        tight_loop_contents();
    }
    absolute_time_t t1 = get_absolute_time();
    uint64_t us = absolute_time_diff_us(t0, t1);
    double kbps = (TOTAL_BYTES * 8.0) / (us / 1e6) / 1000.0;

    printf("DONE. Received %u bytes in %.3f s -> %.2f kbps (%.2f kB/s)\n",
           (unsigned)TOTAL_BYTES, us/1e6, kbps, kbps/8.0);

    #if 0
    // Quick integrity peek
    printf("First 16 bytes of bufA: ");
    for (int i = 0; i < 16; ++i) printf("%02X ", bufA[i]);
    printf("\n");
    #endif
    while (1) tight_loop_contents();
}

