cmake_minimum_required(VERSION 3.13)

# Standalone from the main firmware for the reason pad-probe records: the main
# firmware disables both stdio backends and its console UART has no listener on
# the Pi, so a probe built into it would have nowhere to report.
include(pico_sdk_import.cmake)

project(imu_probe C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(imu_probe main.c)

target_link_libraries(imu_probe
    pico_stdlib
    hardware_gpio
    hardware_i2c
)

# D5 holds for the UART backend: it blocks, and nothing here needs it -- GP20
# is the console pin and no adapter is attached to it on the bench.
pico_enable_stdio_uart(imu_probe 0)

# USB stdio, same trade pad-probe documents: the blocking write path D5 avoids
# is reachable, and acceptable, because printing is most of the job. The one
# difference worth noting is that this program does its measuring FIRST and
# prints from memory afterwards, so a host that opens the port and stops
# reading can delay the report but cannot corrupt a result.
#
# It also brings PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE, which is
# what lets `picotool load -f` flash this and flash it away again with no
# BOOTSEL press.
pico_enable_stdio_usb(imu_probe 1)

target_compile_options(imu_probe PRIVATE -Wall -Wextra)

pico_add_extra_outputs(imu_probe)
