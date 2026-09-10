cmake_minimum_required(VERSION 3.13)

# Links the REAL driver sources from ../../src rather than copying them. A
# diagnostic that reimplements what it is diagnosing proves nothing about the
# firmware; this one fails exactly where low-level-nav fails.
include(pico_sdk_import.cmake)

project(imu_init C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(LLN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

add_executable(imu_init
    main.cpp
    logger_shim.cpp
    ${LLN_SRC}/hardware_drivers/imu_i2c.cpp
    ${LLN_SRC}/hardware_drivers/lsm6dsox.cpp
)

# Neither of those two translation units includes app/rt.h, so no FreeRTOS
# headers are needed. imu_drdy.cpp does include it and is deliberately absent:
# the question this answers is which of init()/configure() fails, and both are
# upstream of the interrupt path.
target_include_directories(imu_init PRIVATE ${LLN_SRC})

target_link_libraries(imu_init
    pico_stdlib
    hardware_gpio
    hardware_i2c
    hardware_dma
)

pico_enable_stdio_uart(imu_init 0)
pico_enable_stdio_usb(imu_init 1)

target_compile_options(imu_init PRIVATE -Wall -Wextra)

pico_add_extra_outputs(imu_init)
