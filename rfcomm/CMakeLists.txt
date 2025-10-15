add_executable(picow_i2c_dma_to_spp
	main.c
	#server.c
)

target_link_libraries(picow_i2c_dma_to_spp
    pico_stdlib
    pico_btstack_classic
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_i2c
    hardware_dma
    hardware_irq
)

target_include_directories(picow_i2c_dma_to_spp PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(picow_i2c_dma_to_spp PRIVATE BTSTACK_FILE=\"btstack_config.h\")

pico_enable_stdio_usb(picow_i2c_dma_to_spp 1)
pico_enable_stdio_uart(picow_i2c_dma_to_spp 0)
pico_add_extra_outputs(picow_i2c_dma_to_spp)

