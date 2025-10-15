#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PIO   pio;
    uint  sm;
    uint  sda_pin;
    uint  scl_pin;
    int   dma_chan;
} i2c_hs_slave_t;

// Configure HS slave (target) on given pins (SDA=N, SCL=N+1 recommended)
void i2c_hs_slave_init(i2c_hs_slave_t* ctx, PIO pio, uint sm, uint sda_pin, uint scl_pin);

// Non-blocking: returns -1 if no byte available
int  i2c_hs_slave_get_byte(void);

// STOP detection (SDA rising while SCL high)
bool i2c_hs_slave_stop_seen(void);
void i2c_hs_slave_clear_stop(void);

// Ring buffer status
uint32_t i2c_hs_slave_ring_free(void);

#ifdef __cplusplus
}
#endif

