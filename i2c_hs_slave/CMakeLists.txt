
add_executable(hs_i2c_slave
    main.c
    i2c_hs_slave.c
    i2c_hs_slave.h
    i2c_hs_slave.pio
)

pico_generate_pio_header(hs_i2c_slave ${CMAKE_CURRENT_LIST_DIR}/i2c_hs_slave.pio)

target_link_libraries(hs_i2c_slave
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_gpio
)

pico_enable_stdio_usb(hs_i2c_slave 1)
pico_enable_stdio_uart(hs_i2c_slave 0)
pico_add_extra_outputs(hs_i2c_slave)

