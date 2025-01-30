add_library(BLiMS INTERFACE)

target_sources(BLiMS INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/blims.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/blims_state.cpp
)

target_link_libraries(BLiMS INTERFACE 
    pico_stdlib 
    hardware_i2c 
    hardware_pwm
    hardware_gpio
    hardware_timer
)