cmake_minimum_required(VERSION 3.17)
set (CMAKE_CXX_STANDARD 17)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()

include(cmake/pico_sdk_import.cmake)
include(cmake/FreeRTOS_Kernel_import.cmake)
project(low-level-nav)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/low-level-navigator.cpp)

target_include_directories(
${PROJECT_NAME} 
PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/src/
${CMAKE_CURRENT_SOURCE_DIR}/src/utility/
${CMAKE_CURRENT_SOURCE_DIR}/src/high_level_drivers/
${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/
)

target_link_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/high_level_drivers/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/
    )

target_sources(
    ${PROJECT_NAME}
    PUBLIC     
    ${CMAKE_CURRENT_SOURCE_DIR}/src/low-level-navigator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/high_level_drivers/propulsion_engine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/high_level_drivers/pid.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware_drivers/gpio_driver.cpp

)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

pico_enable_stdio_usb(low-level-nav 0)
pico_enable_stdio_uart(low-level-nav 1)

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
  FreeRTOS-Kernel-Heap4
  pico_stdlib
  hardware_pwm)

  target_compile_definitions(${PROJECT_NAME} PRIVATE
	PICO_DEFAULT_UART_TX_PIN=16
	PICO_DEFAULT_UART_RX_PIN=17
	)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${PROJECT_NAME})
