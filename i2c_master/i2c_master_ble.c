#if 0
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

#define I2C_PORT   i2c0
#define SDA_PIN    4
#define SCL_PIN    5
#define SLAVE_ADDR 0x17

#define CHUNK_SIZE 1024
#define TOTAL_SIZE (2 * 1024 * 1024)  // 2 MB

static uint8_t txbuf[CHUNK_SIZE];

static int poll_ack_ready(uint8_t *status_out) {
    // Try to read 1 byte; return 0 on success with *status_out set, -1 on NACK/error
    int r = i2c_read_blocking(I2C_PORT, SLAVE_ADDR, status_out, 1, false);
    if (r == 1) return 0;
    return -1;
}

int main() {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(1000);

    // I2C master @ 1 MHz (set what your slave can sustain)
    i2c_init(I2C_PORT, 3.4 * 1000 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Test pattern
    for (int i = 0; i < CHUNK_SIZE; i++) txbuf[i] = (uint8_t)(i & 0xFF);

    printf("I2C Master: sending %d bytes to 0x%02X (chunks of %d)\n",
           TOTAL_SIZE, SLAVE_ADDR, CHUNK_SIZE);

    uint32_t total_sent = 0;
    absolute_time_t t0 = get_absolute_time();

    while (total_sent < TOTAL_SIZE) {

        // 1) Poll ACK until slave says READY (0x01)
        uint8_t status = 0x00;
        while (1) {
            if (poll_ack_ready(&status) == 0 && status == 0x01) break;
            // backoff a touch to avoid hammering the bus
            sleep_ms(1);
        }

        // 2) Write exactly CHUNK_SIZE bytes (handle partial writes)
        uint32_t chunk_sent = 0;
        while (chunk_sent < CHUNK_SIZE) {
            int n = i2c_write_blocking(
                I2C_PORT,
                SLAVE_ADDR,
                &txbuf[chunk_sent],
                CHUNK_SIZE - chunk_sent,
                false 
            );
            if (n < 0) {
                // NACK or bus error: short pause then retry remaining bytes of this chunk
                sleep_us(200);
                continue;
            }
            chunk_sent += (uint32_t)n;
        }

        total_sent += CHUNK_SIZE;

        // 3) Loop: next iteration will poll ACK again before sending the next 1024 bytes
        // (Slave will only return 0x01 after it has finished sending the previous chunk over BLE.)
    }

    absolute_time_t t1 = get_absolute_time();
    double seconds = absolute_time_diff_us(t0, t1) / 1e6;
    double kbps = (total_sent * 8.0) / 1000.0 / seconds;
    double kBps = (total_sent / 1024.0) / seconds;

    printf("Sent %lu bytes in %.3f s -> %.2f kbps (%.2f kB/s)\n",
           (unsigned long)total_sent, seconds, kbps, kBps);

    while (1) sleep_ms(1000);
}
#endif
#if 0
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

#define I2C_PORT i2c0
#define SDA_PIN  4
#define SCL_PIN  5
#define SLAVE_ADDR 0x17

#define BLOCK_SIZE 1024               // Chunk per write //400 kbps
//#define BLOCK_SIZE 247 
//#define BLOCK_SIZE 5120 
//#define BLOCK_SIZE 6144 
#define TOTAL_SIZE (2 * 1024 * 1024)    // 2 MB

static uint8_t buffer[BLOCK_SIZE];

int main() {
    stdio_init_all();
    stdio_usb_init();

    sleep_ms(2000);
    // Init I2C master at 1 MHz (can also try 400 kHz)
    i2c_init(I2C_PORT, (1000 * 1000));
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("I2C Master: sending %d bytes to slave 0x%02X\n", TOTAL_SIZE, SLAVE_ADDR);

    // Fill buffer with a test pattern
    for (int i = 0; i < BLOCK_SIZE; i++) {
        buffer[i] = (uint8_t)(i & 0xFF);
    }

    absolute_time_t start = get_absolute_time();

    uint32_t total_sent = 0;
    uint8_t status = 0x00;
    sleep_ms(2000);
    while (total_sent < TOTAL_SIZE) {
	    do {
		    //printf("blocked here\n");
		    //sleep_ms(1000);
		    i2c_read_blocking(i2c0, SLAVE_ADDR, &status, 1, false);
		    if (status != 0x01) sleep_ms(1);   // tiny backoff
	    } while (status != 0x01);
		    //sleep_ms(2);   // tiny backoff
	    int written = i2c_write_blocking(I2C_PORT, SLAVE_ADDR, buffer, BLOCK_SIZE, false);
	    if (written < 0) {
		    printf("I2C write error at %lu bytes\n", total_sent);
            break;
        }
	//sleep_ms(110);
        total_sent += written;

        // Optionally print progress every 128 KB
	/*
        if ((total_sent % (128*1024)) == 0) {
            printf("Progress: %lu bytes sent\n", total_sent);
        }*/
    }

    absolute_time_t end = get_absolute_time();
    int64_t us = absolute_time_diff_us(start, end);

    double seconds = us / 1e6;
    double kbps = (total_sent * 8.0) / seconds / 1000.0;
    double kBps = (total_sent / seconds) / 1024.0;

    printf("Sent %lu bytes in %.3f s -> %.2f kbps (%.2f kB/s)\n",
           total_sent, seconds, kbps, kBps);

    while (1) {
        sleep_ms(1000);   // keeps alive
    }
}
#endif

#if 1

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define SLAVE_ADDR      0x17

#define CHUNK_SIZE      1024
#define TOTAL_SIZE      (2 * 1024 * 1024)   // 2 MB

static uint8_t txbuf[CHUNK_SIZE];

int main() {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(1000);

    // Init I2C master @ 400kHz (can try 1MHz or 3.4MHz if stable)
    i2c_init(I2C_PORT, 3.4 * 1000 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("I2C Master: starting transfer of %d bytes\n", TOTAL_SIZE);

    // Fill buffer with dummy pattern (0x00..0xFF repeating)
    for (int i = 0; i < CHUNK_SIZE; i++) {
        txbuf[i] = (uint8_t)(i & 0xFF);
    }

    int total_sent = 0;
    while (total_sent < TOTAL_SIZE) {
            // Slave is READY: send next chunk
            int n = i2c_write_blocking(I2C_PORT, SLAVE_ADDR, txbuf, CHUNK_SIZE, false);
            if (n < 0) {
                printf("I2C write failed at %d bytes\n", total_sent);
                sleep_ms(1);
                continue;
            }
            total_sent += CHUNK_SIZE;
            //printf("Sent %d / %d bytes\n", total_sent, TOTAL_SIZE);
        } 

    printf("Transfer complete!\n");
    return 0;
}
#endif
