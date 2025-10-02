// pico_master.c — RP2040 (Pico) I²C master that streams 4096-byte blocks.
// Pins use default I2C0 (SDA=GP4, SCL=GP5) to match your slave.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define SLAVE_ADDR      0x17

#define BLOCK_SIZE      4096
#define STATUS_READY    0x01
#define STATUS_BUSY     0x00

static void fill_pattern(uint8_t *buf, size_t n, uint32_t base){
    for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)((base + i) & 0xFF);
}

static int poll_ready(uint timeout_ms){
    absolute_time_t start = get_absolute_time();
    uint8_t status = 0x00;

    while (true) {
        int r = i2c_read_blocking(I2C_PORT, SLAVE_ADDR, &status, 1, false);
        if (r == 1 && status == STATUS_READY) return 0;

        if (timeout_ms && absolute_time_diff_us(start, get_absolute_time()) / 1000 >= timeout_ms)
            return -1;

        sleep_us(2000); // 2 ms backoff
    }
}

int main(){
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(200);

    // Init I2C master @ 1 MHz (safe sweet spot for your setup)
    i2c_init(I2C_PORT, 1000 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    const uint32_t total = 2 * 1024 * 1024; // 2 MB demo
    uint8_t *block = (uint8_t*)malloc(BLOCK_SIZE);
    if (!block) { while(1); }

    uint32_t sent = 0, seq = 0;

    while (sent < total) {
        if (poll_ready(2000) < 0) {
            printf("Timeout waiting READY\n");
            break;
        }

        uint32_t chunk = (total - sent) >= BLOCK_SIZE ? BLOCK_SIZE : (total - sent);
        fill_pattern(block, chunk, seq * BLOCK_SIZE);

        // Single I2C write transaction for this block
        int w = i2c_write_blocking(I2C_PORT, SLAVE_ADDR, block, chunk, false);
        if (w < 0) { printf("i2c write failed=%d\n", w); break; }

        sent += chunk;
        seq++;

        if ((sent % (256*1024)) == 0) {
            printf("Sent %u / %u bytes\n", sent, total);
        }
    }

    printf("Done. Sent %u bytes\n", sent);
    while (1) tight_loop_contents();
    return 0;
}

