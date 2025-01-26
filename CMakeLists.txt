add_library(BLIMS INTERFACE)

target_sources(BLIMS INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/blims.cpp
    ${CMAKE_CURRENT_LIST_DIR}/blims_state.cpp
)

target_link_libraries(BLIMS INTERFACE 
    pico_stdlib 
    hardware_i2c 
    hardware_pwm
    hardware_gpio
    hardware_timer
)