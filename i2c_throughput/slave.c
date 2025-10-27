
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/i2c.h"
#include "i2c_slave.h"


// ---------------- CONFIG ----------------
#define I2C_PORT   i2c0
#define SDA_PIN    4
#define SCL_PIN    5
#define SLAVE_ADDR 0x17

#define RING_SIZE  (64 * 1024)

// ---------------- RING BUFFER ----------------
static uint8_t  ring_buf[RING_SIZE];
static volatile uint32_t ring_w = 0;
static volatile uint32_t ring_r = 0;

static inline uint32_t ring_mask(uint32_t v) { return v & (RING_SIZE - 1); }
static inline uint32_t ring_count(void)      { return (ring_w - ring_r) & (RING_SIZE - 1); }
static inline void     ring_put(uint8_t b)   { ring_buf[ring_mask(ring_w++)] = b; }
static inline uint8_t  ring_get(void)        { return ring_buf[ring_mask(ring_r++)]; }

// ---------------- I2C SLAVE ISR ----------------
static void
i2c_slave_isr(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            while (i2c_get_read_available(i2c)) {
                uint8_t b = i2c_read_byte_raw(i2c);
                if (ring_count() >= RING_SIZE - 1) {
                    ring_r++;  // drop oldest if full
                }
                ring_put(b);
            }
            break;

        case I2C_SLAVE_REQUEST:
            i2c_write_byte_raw(i2c, 0x00);  // respond with dummy byte
            break;

        case I2C_SLAVE_FINISH:
            break;

        default:
            break;
    }
}

// ---------------- MAIN ----------------
int main() {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(2000);
    printf("Pico W I2C Slave using uptime timestamps (addr 0x%02X)\n", SLAVE_ADDR);

    // ---- Configure pins ----
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // ---- Initialize I2C slave ----
    i2c_init(I2C_PORT, 3.4 * 1000 * 1000);               // 1 MHz (fast-mode plus on good wiring)
    i2c_slave_init(I2C_PORT, SLAVE_ADDR, &i2c_slave_isr);
      // init target mode
    //i2c_init(I2C_PORT,  1000 * 1000);               // 1 MHz (fast-mode plus on good wiring)
    //i2c_init(I2C_PORT, 400000);               // 1 MHz (fast-mode plus on good wiring)
    //i2c_init(I2C_PORT, 100000);               // 1 MHz (fast-mode plus on good wiring)



    // ---- Stats ----
    uint64_t total_bytes = 0;
    uint64_t next_report = 256 * 1024;  // every 256 KB
    bool receiving = false;

    while (true) {
        bool got_data = false;

        // Drain ring buffer
        while (ring_count() > 0) {
            ring_get();
            total_bytes++;
            got_data = true;

            if (total_bytes >= next_report) {
                int64_t ms = to_ms_since_boot(get_absolute_time());
                printf("[Time %lld ms] Received %llu bytes (%.2f KB)\n",
                       ms, total_bytes, total_bytes / 1024.0);
                next_report += 256 * 1024;
            }
        }

        if (got_data) {
        //printf("// Only print when receiving data\n");
            receiving = true;
        } else if (receiving) {
            receiving = false;
        }

        sleep_us(100);
    }
}

