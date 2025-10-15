// main.c — PIO I2C High-Speed master demo (blocking + DMA)
// Uses: pio_i2c.h, i2c.pio.h (generated from your i2c.pio)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "pio_i2c_hs.h"   // <- your header (includes i2c.pio.h)

// ---- Pins / PIO SM ----
#define SDA_PIN     4
#define SCL_PIN     5            // must be SDA_PIN + 1
#define PIN_HS_EN   6            // drives your HS current-source / isolator (HIGH in HS)

#define PIO_USE     pio0
#define SM_USE      0

// ---- Target (7-bit I2C address) ----
#define I2C_ADDR    0x17         // change to your device

// ---- DMA helpers: build 16-bit words the PIO expects ----
static int build_words_from_bytes(const uint8_t *src, size_t len, uint16_t **out_words){
    uint16_t *w = (uint16_t*)malloc(len * sizeof(uint16_t));
    if (!w) return -1;
    for (size_t i = 0; i < len; ++i) {
        // last byte: set FINAL + NAK per your macro
        w[i] = pio_i2c_prepare_byte(src[i], (i == len - 1));
    }
    *out_words = w;
    return 0;
}

static void dma_config_for_pio_tx(PIO pio, uint sm, int dma_ch, const uint16_t *words, size_t n_words){
    dma_channel_config c = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    // TX DREQ for the chosen PIO/SM
    int dreq = (pio == pio0 ? DREQ_PIO0_TX0 : DREQ_PIO1_TX0) + sm;
    channel_config_set_dreq(&c, dreq);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(dma_ch, &c,
        &pio->txf[sm],            // dest: PIO TX FIFO
        words,                    // src: prepared 16-bit words
        n_words,                  // number of halfwords
        false                     // don't start yet (HS helper starts it)
    );
}

int main(void){
    stdio_init_all();
    sleep_ms(500);
    printf("PIO I2C High-Speed master (SDA=%d, SCL=%d, HS_EN=%d)\n", SDA_PIN, SCL_PIN, PIN_HS_EN);

    // --- Load & init the PIO HS I2C program ---
    PIO pio = PIO_USE;
    uint sm = SM_USE;

    // Add the 'i2c' program defined in your i2c.pio (NOT the set_scl_sda table)
    uint offset = pio_add_program(pio, &i2c_program);

    // Provided inline in your i2c.pio.h (% c-sdk block):
    // configures pins, OE inversion, clkdiv for ~3.4MHz HS timing, clears IRQ, enables SM
    i2c_program_init(pio, sm, offset, SDA_PIN, SCL_PIN, PIN_HS_EN);

    // ---- Small blocking write (sanity check) ----
    uint8_t small[32];
    for (int i = 0; i < (int)sizeof small; ++i) small[i] = (uint8_t)i;

    int rc = pio_i2c_write_blocking(pio, sm, PIN_HS_EN, I2C_ADDR, small, sizeof small);
    printf("blocking write rc=%d\n", rc);
    if (rc < 0){
        printf("Blocking HS write failed (check HS hardware, target address, wiring)\n");
        // You can return here if you only want the blocking demo:
        // while (1) tight_loop_contents();
    }

    // ---- Larger DMA write ----
    // Build a bigger payload
    enum { BIG = 4096 };
    static uint8_t big[BIG];
    for (int i = 0; i < BIG; ++i) big[i] = (uint8_t)i;

    // Convert bytes -> 16-bit TX words for the PIO engine
    uint16_t *words = NULL;
    if (build_words_from_bytes(big, BIG, &words) != 0){
        printf("malloc failed for words buffer\n");
        while (1) tight_loop_contents();
    }

    // Configure a DMA channel to feed the PIO TX FIFO with those words
    int dma_ch = dma_claim_unused_channel(true);
    dma_config_for_pio_tx(pio, sm, dma_ch, words, BIG);

    // Kick off: helper performs slow start + HS entry + HS start, then starts DMA
    int rc2 = pio_i2c_write_dma_start(pio, sm, PIN_HS_EN, I2C_ADDR, (uint)dma_ch);
    printf("dma start rc=%d\n", rc2);
    if (rc2 < 0){
        printf("Failed to start HS DMA write\n");
        dma_channel_unclaim(dma_ch);
        free(words);
        while (1) tight_loop_contents();
    }

    // Wait until the HS helper reports completion (or error)
    while (pio_i2c_dma_ongoing(pio, sm)) {
        tight_loop_contents();
    }
    bool dma_err = pio_i2c_dma_check_error(pio, sm);
    printf("dma done; error=%d\n", (int)dma_err);

    // Cleanup
    dma_channel_unclaim(dma_ch);
    free(words);

    while (1) tight_loop_contents();
}

