#include <stdio.h>

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "i2c.pio.h"       // from your PIO file (generates i2c.pio.h)
#include "pio_i2c_hs.h"    // your header (the one you just shared)



int main() {
    stdio_init_all();
    sleep_ms(200);

    const uint SDA = 4, SCL = 5, PIN_HS = 6; // PIN_HS drives your HS hardware (optional)

    PIO pio = pio0;
    uint sm = 0;

    // Load programs
    uint off_i2c = pio_add_program(pio, &i2c_program);
    uint off_tab = pio_add_program(pio, &set_scl_sda_program);

    // Init master @ 1 MHz for now (we’ll switch to 400 kHz immediately after, just demo)
    i2c_program_init(pio, sm, off_i2c, SDA, SCL, PIN_HS);
    sm_set_rate(pio, sm, 400000.0f);  // force 400 kHz startup

    // Build a RAM table of the set_scl_sda instruction words
    uint16_t tab[set_scl_sda_program.length];
    for (int i = 0; i < set_scl_sda_program.length; ++i) {
        tab[i] = pio->instr_mem[off_tab + i];
    }

    // Example payload
    uint8_t buf[8] = {0xDE,0xAD,0xBE,0xEF,0x01,0x23,0x45,0x67};
    uint8_t slave7 = 0x50; // example 24xx EEPROM

    // Do: START@400k -> HS master code -> RSTART -> switch to 3.4MHz -> addr+data -> STOP
    i2c_write_hs_sequence(pio, sm, tab, PIN_HS, /*hs_code_low3=*/0, slave7, buf, sizeof buf);

    while (1) tight_loop_contents();
}

