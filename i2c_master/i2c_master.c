#if 1
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

#define I2C_PORT i2c0
#define SDA_PIN  4
#define SCL_PIN  5
#define SLAVE_ADDR 0x17

//#define BLOCK_SIZE 1024               // Chunk per write //400 kbps
#define BLOCK_SIZE 4096 
//#define BLOCK_SIZE 5120 
//#define BLOCK_SIZE 6144 
#define TOTAL_SIZE (2 * 1024 * 1024)    // 2 MB

static uint8_t buffer[BLOCK_SIZE];

int main() {
    stdio_init_all();
    stdio_usb_init();

    sleep_ms(2000);
    // Init I2C master at 1 MHz (can also try 400 kHz)
    i2c_init(I2C_PORT, (3.4 * 1000 * 1000));
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

#if 0

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
    i2c_init(I2C_PORT, 400 * 1000);
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
        uint8_t status = 0x00;

        // Poll slave status
        int ret = i2c_read_blocking(I2C_PORT, SLAVE_ADDR, &status, 1, false);
        if (ret < 0) {
            printf("I2C read failed\n");
            sleep_ms(1);
            continue;
        }

        if (status == 0x01) {
            // Slave is READY: send next chunk
            int n = i2c_write_blocking(I2C_PORT, SLAVE_ADDR, txbuf, CHUNK_SIZE, false);
            if (n < 0) {
                printf("I2C write failed at %d bytes\n", total_sent);
                sleep_ms(1);
                continue;
            }
            total_sent += CHUNK_SIZE;
            printf("Sent %d / %d bytes\n", total_sent, TOTAL_SIZE);
        } else {
            // Slave is BUSY: wait and retry
            sleep_ms(1);
        }
    }

    printf("Transfer complete!\n");
    return 0;
}
#endif
