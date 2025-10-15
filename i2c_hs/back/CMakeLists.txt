# CMakeLists.txt — PIO I2C High-Speed master (standalone target)

cmake_minimum_required(VERSION 3.13)



# ---- Executable ----
add_executable(master_hs
    master.c          # uses your pio_i2c.h API
    pio_i2c_hs.c    # your HS helper implementation (the big file you shared)
)

# Generate the header from your PIO program (i2c.pio -> i2c.pio.h)
pico_generate_pio_header(master_hs ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)

# If headers live next to sources, this is optional; keeps things explicit
target_include_directories(master_hs PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link the needed Pico libs
target_link_libraries(master_hs
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_irq
)

# USB stdio on, UART off (tweak if you want UART)
pico_enable_stdio_usb(master_hs 1)
pico_enable_stdio_uart(master_hs 0)

# UF2, bin, elf outputs
pico_add_extra_outputs(master_hs)


# ---- MASTER (PIO HS) ----
add_executable(master_hs_2mb
    master_hs_2mb.c
    pio_i2c_hs.c          # your HS helper .c (the big file you pasted earlier)
)
# generate i2c.pio.h from your i2c.pio
pico_generate_pio_header(master_hs_2mb ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)

target_include_directories(master_hs_2mb PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(master_hs_2mb
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_irq
)
pico_enable_stdio_usb(master_hs_2mb 1)
pico_enable_stdio_uart(master_hs_2mb 0)
pico_add_extra_outputs(master_hs_2mb)

# ---- SLAVE (HW I2C + DMA) ----
add_executable(slave_dma_rx
    slave_dma_rx.c
)
target_link_libraries(slave_dma_rx
    pico_stdlib
    hardware_i2c
    hardware_dma
    hardware_irq
)
pico_enable_stdio_usb(slave_dma_rx 1)
pico_enable_stdio_uart(slave_dma_rx 0)
pico_add_extra_outputs(slave_dma_rx)





