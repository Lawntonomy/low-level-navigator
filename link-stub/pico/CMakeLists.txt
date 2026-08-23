cmake_minimum_required(VERSION 3.13)

# Standalone from the main firmware on purpose: this is bring-up code for the
# link, not a feature of the navigator, and it must not perturb that build.
include(pico_sdk_import.cmake)

project(link_stub C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(link_stub main.c)

target_include_directories(link_stub PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../mavlink/c)

target_link_libraries(link_stub
    pico_stdlib
    pico_rand      # session id (IF-0001 §7.5)
    hardware_uart
    hardware_gpio
)

# D5: neither stdio backend is enabled. Both block, and stdio fans every printf
# out to every enabled driver. The console in main.c writes uart1 directly.
pico_enable_stdio_usb(link_stub 0)
pico_enable_stdio_uart(link_stub 0)

# MAVLink's generated headers trip -Waddress-of-packed-member and friends on
# GCC. The generator's output is not ours to fix; keep the warnings for our own
# translation unit and silence them only where they come from.
target_compile_options(link_stub PRIVATE
    -Wall -Wextra
    -Wno-address-of-packed-member
    -Wno-unused-parameter
)

pico_add_extra_outputs(link_stub)
