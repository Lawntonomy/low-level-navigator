cmake_minimum_required(VERSION 3.13)

# Isolation test for the link-commanded BOOTSEL reboot, which fails on hardware
# in a way that unit tests and the build cannot see. Deliberately has nothing in
# common with the firmware except the one ROM call under suspicion.
include(pico_sdk_import.cmake)

project(bootsel_probe C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(bootsel_probe main.c)

target_link_libraries(bootsel_probe
    pico_stdlib
    pico_bootrom
    pico_multicore
)

pico_enable_stdio_uart(bootsel_probe 0)

# USB stdio on: this program is nothing but printf, and it is also what keeps
# `picotool load -f` working right up until the reset is called. If the reset
# does not reach BOOTSEL, that is the difference between reflashing over USB
# and reaching for the button.
pico_enable_stdio_usb(bootsel_probe 1)

target_compile_options(bootsel_probe PRIVATE -Wall -Wextra)

pico_add_extra_outputs(bootsel_probe)
