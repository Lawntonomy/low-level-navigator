cmake_minimum_required(VERSION 3.13)

# Standalone from the main firmware on purpose. The main firmware cannot answer
# this question: it disables both stdio backends and its console UART has no
# listener attached, so a probe built into it would have nowhere to report.
include(pico_sdk_import.cmake)

project(pad_probe C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(pad_probe main.c)

target_link_libraries(pad_probe
    pico_stdlib
    hardware_gpio
)

# D5 holds for the UART backend: it blocks, and nothing here needs it.
pico_enable_stdio_uart(pad_probe 0)

# USB stdio IS enabled, and unlike link-stub the usual justification does NOT
# apply here -- this program is nothing but printf, so the blocking write path
# D5 exists to avoid (stdio_usb_out_chars stalling up to
# PICO_STDIO_USB_STDOUT_TIMEOUT_US when a host opens the port and stops
# reading) is genuinely reachable.
#
# It is acceptable anyway, for a reason specific to this program rather than
# inherited from link-stub: blocking costs nothing when printing IS the whole
# job. There is no control loop to starve, no watchdog to feed, and no deadline
# to miss. The samples are taken once, before the first print, so even an
# indefinite stall cannot corrupt the result -- it can only delay reporting a
# value already in memory.
#
# It also brings PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE, which is
# what lets `picotool load -f` flash this and flash it away again with no
# BOOTSEL press. That matters: the main firmware has USB stdio off, so once it
# is on the board, remote flashing stops working until someone presses the
# button.
#
# Not a precedent for the FreeRTOS foundation, which still needs its own
# non-blocking USB path before this trade is worth making there.
pico_enable_stdio_usb(pad_probe 1)

target_compile_options(pad_probe PRIVATE -Wall -Wextra)

pico_add_extra_outputs(pad_probe)
