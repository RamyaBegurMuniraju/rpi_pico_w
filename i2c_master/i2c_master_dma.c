#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define SLAVE_ADDR      0x17

#define BLOCK_SIZE      4096
#define TOTAL_BYTES     (2 * 1024 * 1024) // 2 MB test

static uint8_t buf[BLOCK_SIZE];

int main(){
    stdio_init_all();
    sleep_ms(1000);
    printf("I2C Master: send %u bytes to 0x%02X\n", (unsigned)TOTAL_BYTES, SLAVE_ADDR);

    i2c_init(I2C_PORT, 3.4 * 1000 * 1000); // 3.4 MHz 
    //i2c_init(I2C_PORT,  1000 * 1000); // 1 MHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    for (int i = 0; i < BLOCK_SIZE; i++) buf[i] = (uint8_t)i;

    uint32_t remaining = TOTAL_BYTES;
    absolute_time_t t0 = get_absolute_time();
    while (remaining){
        uint32_t n = remaining > BLOCK_SIZE ? BLOCK_SIZE : remaining;
        int w = i2c_write_blocking(I2C_PORT, SLAVE_ADDR, buf, n, false);
        if (w < 0){
            printf("i2c_write_blocking error=%d\n", w);
            sleep_ms(1);
            continue;
        }
        remaining -= (uint32_t)w;
        if (((TOTAL_BYTES - remaining) % (128*1024)) < BLOCK_SIZE){
            printf("Progress: %u / %u\n", (unsigned)(TOTAL_BYTES - remaining), (unsigned)TOTAL_BYTES);
        }
    }
    absolute_time_t t1 = get_absolute_time();
    int64_t us = absolute_time_diff_us(t0, t1);
    double secs = us / 1e6;
    double kbps = (TOTAL_BYTES * 8.0) / 1000.0 / secs;
    double kBps = (TOTAL_BYTES) / 1024.0 / secs;
    printf("Done in %.3f s → %.2f kbps (%.2f kB/s)\n", secs, kbps, kBps);

    while(1) tight_loop_contents();
}
