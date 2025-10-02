cmake_minimum_required(VERSION 3.28)
# Standalone example that reads from the on board temperature sensor and sends notifications via BLE
# Flashes slowly each second to show it's running

add_executable(i2c_data
	i2c_master.c
	# new_method.c
    )
target_link_libraries(i2c_data
    pico_stdio_usb
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
    hardware_i2c
	pico_multicore
    )
target_include_directories(i2c_data PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/lib # For our common btstack config
    ${PICO_SDK_PATH}/src/rp2_common/pico_i2c_slave/include
    )
pico_enable_stdio_usb(i2c_data 1)
pico_enable_stdio_uart(i2c_data 0)

#target_compile_definitions(picow_bt_example_common INTERFACE
 #    WANT_HCI_DUMP=1 # This enables btstack debug
  #   )

# create .uf2 etc  
pico_add_extra_outputs(i2c_data)


