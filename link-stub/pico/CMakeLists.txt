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

# D5 still holds for the UART stdio backend: it blocks, and this file writes
# its console over uart1 directly instead.
pico_enable_stdio_uart(link_stub 0)

# USB stdio IS enabled here, deliberately, and this is narrower than it looks.
#
# main.c never calls printf/puts/stdio - it has no stdio call sites at all -
# so pico_enable_stdio_usb's blocking write path (stdio_usb_out_chars, which
# can stall up to PICO_STDIO_USB_STDOUT_TIMEOUT_US if a host opens the port
# and stops reading - the exact hazard D5 exists to avoid) has nothing to ever
# call it. Turning this on here adds the USB device and its vendor reset
# interface without adding a blocking code path.
#
# The reason to want it: PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
# defaults on with this, which lets `picotool load -f` force a reboot into
# BOOTSEL over USB with no physical button press - bench-dev convenience the
# old pre-rewrite firmware had and this branch's earlier commit removed.
#
# IF-0001 §11 names "whether reset-over-USB stays enabled" as an open
# question for the real firmware, because it lets a Pi-side process reset the
# safety controller over a link ADR-0003 designates non-safety. That question
# is answered "yes" HERE, deliberately, for a bench-only stub with no motors
# ever wired - it is not a precedent for the FreeRTOS foundation, which still
# needs its own non-blocking USB path (D5) before this trade is worth making
# there too.
pico_enable_stdio_usb(link_stub 1)

# MAVLink's generated headers trip -Waddress-of-packed-member and friends on
# GCC. The generator's output is not ours to fix; keep the warnings for our own
# translation unit and silence them only where they come from.
target_compile_options(link_stub PRIVATE
    -Wall -Wextra
    -Wno-address-of-packed-member
    -Wno-unused-parameter
)

pico_add_extra_outputs(link_stub)
