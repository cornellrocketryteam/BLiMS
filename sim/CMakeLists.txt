cmake_minimum_required(VERSION 3.10)
project(BLiMS_Sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Include the root BLiMS header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

# Simulation test library
add_library(sim_test_lib STATIC
    sim_test.cpp
    sim_test.hpp
)

# Standalone simulator runner
add_executable(sim_runner
    sim_runner.cpp
)
target_link_libraries(sim_runner sim_test_lib -lm)

# Optional: Test runner with more detailed output
add_executable(sim_detailed
    sim_runner.cpp
)
target_link_libraries(sim_detailed sim_test_lib -lm)
target_compile_definitions(sim_detailed PRIVATE DETAILED_OUTPUT)

# Build instructions
message(STATUS "Simulator targets created:")
message(STATUS "  - sim_runner: Basic flight simulation")
message(STATUS "  - sim_detailed: Verbose output with all state details")
message(STATUS "")
message(STATUS "Build with:")
message(STATUS "  mkdir -p build && cd build")
message(STATUS "  cmake .. && make")
message(STATUS "  ./sim_runner")
