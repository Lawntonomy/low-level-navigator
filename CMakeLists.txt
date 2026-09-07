cmake_minimum_required(VERSION 3.17)
set (CMAKE_CXX_STANDARD 17)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()

include(cmake/pico_sdk_import.cmake)

#include(GoogleTest)
include(cmake/FreeRTOS_Kernel_import.cmake)
project(low-level-nav)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#enable_testing()

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/googletest)
target_include_directories(
${PROJECT_NAME} 
PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/src/
${CMAKE_CURRENT_SOURCE_DIR}/include/
${CMAKE_CURRENT_SOURCE_DIR}/src/utility/
${CMAKE_CURRENT_SOURCE_DIR}/src/high_level_drivers/
${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/
${CMAKE_CURRENT_SOURCE_DIR}/mavlink/c/
)

target_link_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/high_level_drivers/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/
    )

target_sources(
    ${PROJECT_NAME}
    PUBLIC  
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app/bootloader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app/link.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app/motors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app/log.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app/safety.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/encoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/pwm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/high_level_drivers/pid.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/pwm.cpp


)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# TP-0001 D5: both stdio backends block, and stdio.c fans every write out to
# every enabled driver. The console in src/app/log.cpp writes uart1 directly.
# The old PICO_DEFAULT_UART_* override put stdio on GPIO 16/17, which is the
# SAME uart0 the command link now uses (RP2350 datasheet Table 3) - keeping it
# would have interleaved printf with protocol frames on the safety link.
#
# stdio_uart stays off for that reason. stdio_usb is now ON: ADR-0003 assigns
# verbose runtime logging to USB CDC, and IF-0001 SS2 puts it on the Bulk
# channel. src/utility/logger.{h,cpp} is what makes turning this on safe --
# every Log:: call formats into a drop-on-full ring and returns; only one
# low-priority task, which is not the control task and does not feed the
# watchdog, ever calls printf(). See logger.h's header comment for the whole
# argument. Nothing on this path may be required for safe operation.
pico_enable_stdio_usb(low-level-nav 1)
pico_enable_stdio_uart(low-level-nav 0)

# Belt-and-braces, NOT the fix by itself: this only breaks the wait in
# stdio_usb_out_chars() for tud_cdc_write_available() (pico-sdk
# src/rp2_common/pico_stdio_usb/stdio_usb.c:121), making an unread port drop
# bytes after 0 us instead of the 500000 us default. It does nothing about
# the SAME function's stdio_usb_mutex, taken with
# mutex_try_enter_block_until(..., PICO_STDIO_DEADLOCK_TIMEOUT_MS) at line 103
# -- default 1000 ms -- which is why the logger task, not this define, is the
# actual safety argument.
target_compile_definitions(low-level-nav PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
)

pico_generate_pio_header(low-level-nav ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/encoder.pio)
pico_generate_pio_header(low-level-nav ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/pwm.pio)
pico_generate_pio_header(low-level-nav ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/ws2812.pio)

target_include_directories(low-level-nav PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

add_library(freertos_config INTERFACE)

target_include_directories(freertos_config SYSTEM
INTERFACE
    include
)

target_compile_definitions(freertos_config
  INTERFACE
    projCOVERAGE_TEST=0
)

target_link_libraries(${PROJECT_NAME}
  #pico_async_context_freertos
  pico_multicore
  pico_rand
  # Link-commanded reboot into BOOTSEL (src/app/bootloader.cpp). A bootrom
  # call, not a USB stack: it is what lets remote reflashing work while
  # TP-0001 D5 keeps both stdio backends disabled above.
  pico_bootrom
  hardware_watchdog
  hardware_uart
  FreeRTOS-Kernel-Heap4
  pico_stdlib
  hardware_pio
  hardware_pwm
  hardware_dma
  #GTest::gtest_main
  )

  #gtest_discover_tests(${PROJECT_NAME})



# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${PROJECT_NAME})
