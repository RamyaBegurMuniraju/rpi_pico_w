cmake_minimum_required(VERSION 3.28)

add_executable(picow_i2c_to_l2cap
	#slave.c
	#pico_source_test.c
	#new_method.c #DMA method 400 kbps only
	main.c
)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/pico_i2c_slave)

# Classic-only BTstack libs + CYW43 transport
target_link_libraries(picow_i2c_to_l2cap
    pico_stdio_usb
    pico_stdlib
    pico_btstack_classic
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
    hardware_i2c
    pico_i2c_slave      # from pico-extras
    pico_multicore       # <-- add this line
    #pico_btstack_flash_bank
    #pcio_multicore
)

target_include_directories(picow_i2c_to_l2cap PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

#target_compile_definitions(picow_i2c_to_l2cap PRIVATE BTSTACK_FILE=\"btstack_config.h\")
# Make every BTstack-compiled file include our config header
#add_compile_definitions(BTSTACK_FILE=\"btstack_config.h\")

#target_compile_definitions(picow_i2c_to_l2cap PRIVATE
#   BTSTACK_FILE="btstack_config.h"
#    CYW43_HOST_NAME="PicoW L2CAP Bridge"
#)


# USB stdio? (optional)
pico_enable_stdio_usb(picow_i2c_to_l2cap 1)
pico_enable_stdio_uart(picow_i2c_to_l2cap 0)

pico_add_extra_outputs(picow_i2c_to_l2cap)
