#if 0
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/i2c_slave.h"   // SDK’s I²C slave support

#define I2C_PORT    i2c0
#define SDA_PIN     4
#define SCL_PIN     5
#define SLAVE_ADDR  0x17

// I2C slave event handler
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            uint8_t b = i2c_read_byte_raw(i2c);
            printf("RX: 0x%02X\n", b);
            break;
        }
        case I2C_SLAVE_REQUEST: {
            i2c_write_byte_raw(i2c, 0xAA);  // respond with fixed value
            printf("TX: 0xAA\n");
            break;
        }
        case I2C_SLAVE_FINISH: {
            printf("STOP condition\n");
            break;
        }
        default:
            break;
    }
}

int main() {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(2000);
    printf("Pico I2C Slave test on address 0x%02X\n", SLAVE_ADDR);

    // Init I2C0 as slave
    //i2c_init(I2C_PORT, 3.4 * 1000 * 1000);   // start with 100kHz (safe)
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Attach ISR
    i2c_slave_init(I2C_PORT, SLAVE_ADDR, &i2c_slave_handler);

    while (true) {
        tight_loop_contents();  // idle, everything is interrupt-driven
    }
}
#endif



#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/i2c_slave.h"

#define I2C_PORT    i2c0
#define SDA_PIN     4
#define SCL_PIN     5
#define SLAVE_ADDR  0x17

// Max data size for test (e.g. 2 MB)
#define MAX_DATA    (2 * 1024 * 1024)

static uint8_t rx_buffer[MAX_DATA];   // store everything
static volatile uint32_t rx_index = 0;
static absolute_time_t start_time;
static absolute_time_t end_time;
static volatile bool started = false;
static volatile bool finished = false;

// ===== I2C slave handler =====
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: {
        uint8_t b = i2c0_hw->data_cmd & 0xFF;

        if (!started) {
            // mark start on first byte
            start_time = get_absolute_time();
            started = true;
        }

        if (rx_index < MAX_DATA) {
            rx_buffer[rx_index++] = b;
        }
        break;
    }

    case I2C_SLAVE_REQUEST: {
        // Master reads from slave
        i2c0_hw->data_cmd = 0xAA;
        break;
    }

    case I2C_SLAVE_FINISH: {
        // STOP condition -> mark end
        end_time = get_absolute_time();
        finished = true;
        break;
    }
    }
}

int main() {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(2000);
    printf("I2C Slave on addr 0x%02X\n", SLAVE_ADDR);

    // Setup pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Init as slave
    i2c_slave_init(I2C_PORT, SLAVE_ADDR, i2c_slave_handler);

    while (1) {
        if (finished) {
            uint64_t us = absolute_time_diff_us(start_time, end_time);
            double seconds = us / 1e6;
            double kbps = (rx_index * 8.0) / seconds / 1000.0;
            double kBps = (rx_index) / seconds / 1024.0;

            printf("\n--- Transfer complete ---\n");
            printf("Bytes received: %lu\n", rx_index);
            printf("Time: %.3f s\n", seconds);
            printf("Throughput: %.2f kbps (%.2f kB/s)\n", kbps, kBps);

            finished = false; // reset if you want multiple runs
            rx_index = 0;
            started = false;
        }
        tight_loop_contents();
    }
}

