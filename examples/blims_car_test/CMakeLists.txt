# BLiMS Car Test CMakeLists.txt
#
# Expected location: BLiMS/test/blims_car_test/CMakeLists.txt
#
# Project structure:
#   BLiMS/
#   ├── lib/
#   │   ├── pico-sdk/
#   │   └── ublox-MX-Pico/
#   ├── src/
#   │   ├── blims.cpp / blims.hpp
#   │   ├── blims_state.cpp / blims_state.hpp
#   │   └── blims_constants.hpp
#   └── test/
#       └── blims_car_test/        <── you are here
#           ├── CMakeLists.txt
#           ├── blims_car_test.cpp
#           └── descent_alt_data.hpp
#
# Build:
#   mkdir build && cd build
#   cmake ..
#   make
#   # Flash blims_car_test.uf2 to Pico

cmake_minimum_required(VERSION 3.22)

# Pull in Pico SDK (must be before project())
include(../../lib/pico-sdk/pico_sdk_init.cmake)

project(blims_car_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_options(
    -Wall
    -Wno-format
    -Wno-unused-function
)

# ---- Sources ----
add_executable(${PROJECT_NAME}
    blims_car_test.cpp
    ../../src/blims.cpp
    ../../src/blims_state.cpp
)

# ---- Include directories ----
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}              # descent_alt_data.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src    # blims.hpp, blims_state.hpp, blims_constants.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/ublox-MX-Pico  # ublox_mx.hpp, ublox_nav_pvt.hpp
)

# ---- ublox library ----
add_subdirectory(../../lib/ublox-MX-Pico ublox-MX-Pico)

# ---- Link libraries ----
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_pwm
    hardware_timer
    ublox-MX-Pico
)

# ---- Serial over USB (for visualizer) ----
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# ---- Generate UF2 ----
pico_add_uf2_output(${PROJECT_NAME})
