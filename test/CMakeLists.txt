# Host-side unit tests.
#
# This is a SEPARATE CMake project from the firmware build: it targets the host
# compiler, not arm-none-eabi, and never initializes the Pico SDK. Only
# hardware-independent logic is compiled in here. See scripts/test.sh.

cmake_minimum_required(VERSION 3.17)
project(lln-host-tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
FetchContent_MakeAvailable(googletest)

enable_testing()

set(LLN_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_executable(lln_tests
    test_pid.cpp
    test_encoder_math.cpp
    test_bootloader.cpp
    test_logger_ring.cpp
    stubs/logger_stub.cpp
    ${LLN_SRC_DIR}/high_level_drivers/pid.cpp
)

target_include_directories(lln_tests PRIVATE
    ${LLN_SRC_DIR}
    ${LLN_SRC_DIR}/high_level_drivers
    ${LLN_SRC_DIR}/utility
)

target_compile_options(lln_tests PRIVATE -Wall -Wextra)

target_link_libraries(lln_tests PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(lln_tests)
