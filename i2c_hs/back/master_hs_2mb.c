#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pio_i2c_hs.h"   // your header (includes i2c.pio.h from i2c.pio)

#define SDA_PIN     4         // SCL must be SDA+1
#define SCL_PIN     5
#define PIN_HS_EN   6         // drives your HS current-source / isolator

#define USE_PIO     pio0
#define USE_SM      0

#define I2C_ADDR        0x17  // 7-bit
#define CHUNK_BYTES     4096
#define TOTAL_BYTES     (2 * 1024 * 1024) // 2 MB
#define NUM_CHUNKS      (TOTAL_BYTES / CHUNK_BYTES)

static uint8_t  tx_chunk[CHUNK_BYTES];
static uint16_t tx_words[CHUNK_BYTES]; // 1 halfword per byte

static void build_words_from_bytes(const uint8_t *src, size_t len, uint16_t *words){
    for (size_t i = 0; i < len; ++i){
        // FINAL on last byte of CHUNK so the SM ignores a NAK there
        words[i] = pio_i2c_prepare_byte(src[i], (i == len - 1));
    }
}

static void dma_config_for_pio_tx(PIO pio, uint sm, int dma_ch,
                                  const uint16_t *words, size_t n_words){
    dma_channel_config c = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    int dreq = (pio == pio0 ? DREQ_PIO0_TX0 : DREQ_PIO1_TX0) + sm;
    channel_config_set_dreq(&c, dreq);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(dma_ch, &c, &pio->txf[sm], words, n_words, false);
}

int main(void){
    stdio_init_all();
    sleep_ms(1000);

    printf("printing from master\n");
    printf("PIO I2C HS master -> send 2MB (SDA=%d, SCL=%d, HS_EN=%d)\n", SDA_PIN, SCL_PIN, PIN_HS_EN);

    // Load & init the PIO HS program
    PIO pio = USE_PIO; uint sm = USE_SM;
    uint off = pio_add_program(pio, &i2c_program);
    i2c_program_init(pio, sm, off, SDA_PIN, SCL_PIN, PIN_HS_EN);

    // Build one chunk’s worth of data & words (we’ll reuse the same pattern)
    for (int i = 0; i < CHUNK_BYTES; ++i) tx_chunk[i] = (uint8_t)(i & 0xFF);
    build_words_from_bytes(tx_chunk, CHUNK_BYTES, tx_words);

    int dma_ch = dma_claim_unused_channel(true);

    absolute_time_t t0 = get_absolute_time();
    for (int chunk = 0; chunk < NUM_CHUNKS; ++chunk){
        // Configure DMA to feed PIO TX with our prepared words
        dma_config_for_pio_tx(pio, sm, dma_ch, tx_words, CHUNK_BYTES);

        // Start HS transfer: helper performs slow start + HS entry + HS start, then starts DMA
        int rc = pio_i2c_write_dma_start(pio, sm, PIN_HS_EN, I2C_ADDR, (uint)dma_ch);
        if (rc < 0){
            printf("Start failed at chunk %d\n", chunk);
            dma_channel_abort(dma_ch);
            break;
        }

        // Wait for this chunk to finish
        while (pio_i2c_dma_ongoing(pio, sm)) tight_loop_contents();

        if (pio_i2c_dma_check_error(pio, sm)){
            printf("DMA/PIO error at chunk %d\n", chunk);
            // Try to recover: stop, then continue or abort
            pio_i2c_dma_stop(pio, sm, PIN_HS_EN, (uint)dma_ch);
            break;
        }

        if ((chunk & 0x07) == 0){ // every 8 chunks (~32 KB)
            int64_t us = absolute_time_diff_us(t0, get_absolute_time());
            double sent = (double)(chunk + 1) * CHUNK_BYTES / (1024.0 * 1024.0);
            double mbps = (8.0 * (chunk + 1) * CHUNK_BYTES) / (double)us; // Mbit/us
            mbps *= 1e6 / 1e6; // Mbit/s (unit-friendly)
            printf("Progress: %.1f MB sent\n", sent);
        }
    }

    dma_channel_unclaim(dma_ch);
    printf("Done.\n");
    while (1) tight_loop_contents();
}

