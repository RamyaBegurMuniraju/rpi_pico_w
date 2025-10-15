#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "i2c.pio.h"       // from your PIO file (generates i2c.pio.h)
#include "pio_i2c_hs.h"    // your header (the one you just shared)

#define SDA_PIN      4
#define SCL_PIN      5
#define PIN_HS_EN    6      // set to -1 if you have no HS enable hardware
#define TARGET_ADDR  0x17   // 7-bit address of the slave

#define TOTAL   (2*1024*1024)  // 2 MB
#define CHUNK   4096            // send in 4 KB chunks

static void fill_pattern(uint8_t *buf, size_t off, size_t len){
    for (size_t i = 0; i < len; ++i) buf[i] = (uint8_t)((off + i) & 0xFF);
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("HS I2C Master: sending 2MB to 0x%02X...\n", TARGET_ADDR);

    PIO pio = pio0;
    uint sm = 0;

    // Load and init your HS master PIO program (from i2c.pio’s %c-sdk init)
    uint offset = pio_add_program(pio, &i2c_program);
    i2c_program_init(pio, sm, offset, SDA_PIN, SCL_PIN, PIN_HS_EN);

    if (PIN_HS_EN >= 0) gpio_put(PIN_HS_EN, 1); // enable external HS circuitry if present

    static uint8_t buf[CHUNK];
    uint32_t sent = 0;
    absolute_time_t t0 = get_absolute_time();

    while (sent < TOTAL) {
        size_t n = (TOTAL - sent) < CHUNK ? (TOTAL - sent) : CHUNK;
        fill_pattern(buf, sent, n);

        // This helper does: HS preamble + START + addr(W) + data + STOP
        int rc = pio_i2c_write_blocking(pio, sm, PIN_HS_EN, TARGET_ADDR, buf, (uint)n);
        if (rc < 0) {
            printf("Write error at %u bytes. Halting.\n", sent);
            // Optionally: pio_i2c_resume_after_error(pio, sm);
            while (1) tight_loop_contents();
        }

        sent += n;

        if ((sent % (128*1024)) == 0) {
            int64_t us = absolute_time_diff_us(t0, get_absolute_time());
            double mb = sent / (1024.0 * 1024.0);
            double mbps = mb / (us / 1e6);
            printf("Progress: %.1f MB (%.2f MB/s)\n", mb, mbps);
        }
    }

    int64_t us = absolute_time_diff_us(t0, get_absolute_time());
    double secs = us / 1e6;
    printf("Done: 2MB in %.3f s (%.2f MB/s)\n", secs, (2.0 / secs));
    while (1) tight_loop_contents();
}

