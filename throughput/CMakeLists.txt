cmake_minimum_required(VERSION 4.1)
# Standalone example that reads from the on board temperature sensor and sends notifications via BLE
# Flashes slowly each second to show it's running
add_executable(thrput_tool
    main.c
    )
target_link_libraries(thrput_tool
    pico_stdio_usb
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
    hardware_i2c
    )
target_include_directories(thrput_tool PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/lib # For our common btstack config
    )

#pico_enable_stdio_usb(pico_ble_i2c_slave 1)
#pico_enable_stdio_uart(pico_ble_i2c_slave 0)
# create .uf2 etc  
pico_add_extra_outputs(thrput_tool)
