cmake_minimum_required(VERSION 4.1)
# Standalone example that reads from the on board temperature sensor and sends notifications via BLE
# Flashes slowly each second to show it's running
add_executable(picow_ble_i2c_data
    main.c
    )
target_link_libraries(picow_ble_i2c_data
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_adc
    hardware_i2c
    )
target_include_directories(picow_ble_i2c_data PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/lib # For our common btstack config
    )
pico_btstack_make_gatt_header(picow_ble_i2c_data PRIVATE "${CMAKE_CURRENT_LIST_DIR}/temp_sensor.gatt")

# create .uf2 etc  
pico_add_extra_outputs(picow_ble_i2c_data)
