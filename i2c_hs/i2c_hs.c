#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "i2c.pio.h"        // from your PIO assembly
//#include "set_scl_sda.pio.h"

static inline uint16_t tx_instr_header(uint8_t n /*0..63*/) {
    return ((uint16_t)n) << 10;  // "Instr" field
}
static inline uint16_t tx_data_byte(uint8_t byte, bool final_ignore_nak) {
    return ((0u << 10) | ((final_ignore_nak?1u:0u)<<9) | ((uint16_t)byte<<1));
}

static inline void push_word(PIO pio, uint sm, uint16_t w) {
    while (pio_sm_is_tx_fifo_full(pio, sm)) tight_loop_contents();
    pio->txf[sm] = w;
}
static inline void push_exec(PIO pio, uint sm, uint16_t instr) {
    while (pio_sm_is_tx_fifo_full(pio, sm)) tight_loop_contents();
    pio->txf[sm] = instr; // consumed by `out exec,16`
}

// Simple START/STOP using your set_scl_sda table
static void i2c_push_start(PIO pio, uint sm, const uint16_t *tab) {
    push_word(pio, sm, tx_instr_header(1));
    push_exec(pio, sm, tab[I2C_1S1_8]); // SCL=1, SDA=1 (hold)
    push_exec(pio, sm, tab[I2C_1S0_8]); // SCL=1, SDA=0 -> START
}
static void i2c_push_stop(PIO pio, uint sm, const uint16_t *tab) {
    push_word(pio, sm, tx_instr_header(1));
    push_exec(pio, sm, tab[I2C_1S0_8]); // SCL=1, SDA=0
    push_exec(pio, sm, tab[I2C_1S1_8]); // SCL=1, SDA=1 -> STOP
}

static inline void sm_set_rate(PIO pio, uint sm, float hz) {
    float div = (float) clock_get_hz(clk_sys) / (8.f * hz);
    // brief pause while we change clkdiv
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_set_clkdiv(pio, sm, div);
    pio_sm_set_enabled(pio, sm, true);
}

// --- One-shot: Start @400 kHz, HS entry, switch to 3.4 MHz, send payload ---
void i2c_write_hs_sequence(PIO pio, uint sm,
                           const uint16_t *tab,  // set_scl_sda instruction table
                           uint pin_hs,          // pass 0xFF if unused
                           uint8_t hs_code_low3, // 0..7
                           uint8_t slave7,       // 7-bit I2C address
                           const uint8_t *data, size_t len)
{
    // 1) Ensure we’re at 400 kHz
    sm_set_rate(pio, sm, 400000.0f);

    // 2) START (Fm)
    i2c_push_start(pio, sm, tab);

    // 3) HS master code (write=0): 00001xxx <<1 | 0
    uint8_t master_code = (uint8_t)((0x08u | (hs_code_low3 & 7u)) << 1);
    push_word(pio, sm, tx_data_byte(master_code, false)); // expect ACK from HS-capable device

    // 4) REPEATED START (still Fm)
    i2c_push_start(pio, sm, tab);

    // 5) Enable external HS pullups if present
    if (pin_hs < NUM_BANK0_GPIOS) gpio_put(pin_hs, true);

    // 6) Switch to 3.4 MHz
    sm_set_rate(pio, sm, 3400000.0f);

    // 7) Address real slave (write=0)
    uint8_t addr_w = (uint8_t)((slave7 << 1) | 0);
    push_word(pio, sm, tx_data_byte(addr_w, false)); // expect ACK

    // 8) Data payload at HS rate
    for (size_t i = 0; i < len; ++i) {
        bool last = (i + 1 == len);
        push_word(pio, sm, tx_data_byte(data[i], last /*ignore NAK on last*/));
    }

    // 9) STOP
    i2c_push_stop(pio, sm, tab);

    if (pin_hs < NUM_BANK0_GPIOS) gpio_put(pin_hs, false);
}

