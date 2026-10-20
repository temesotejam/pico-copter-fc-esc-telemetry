cmake_minimum_required(VERSION 3.13)

# This directory is intentionally self-contained as the distributable firmware
# source folder. Point PICO_SDK_PATH at a Raspberry Pi Pico SDK checkout.
if(NOT DEFINED PICO_SDK_PATH OR PICO_SDK_PATH STREQUAL "")
    if(DEFINED ENV{PICO_SDK_PATH} AND NOT "$ENV{PICO_SDK_PATH}" STREQUAL "")
        set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
    else()
        message(FATAL_ERROR
            "PICO_SDK_PATH is required. Set -DPICO_SDK_PATH=/path/to/pico-sdk "
            "or define the PICO_SDK_PATH environment variable.")
    endif()
endif()

include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(pico_copter C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# Optional build override: -DDSHOT_RATE_KBPS=600 and/or -DDSHOT_BIDIRECTIONAL=0
# Empty (default) uses one editable value in dshot_config.hpp: 300.
set(DSHOT_RATE_KBPS "" CACHE STRING "Override DShot speed in kbps (150, 300, 600, 1200); empty = dshot_config.hpp")
if(NOT DSHOT_RATE_KBPS STREQUAL "" AND
   NOT DSHOT_RATE_KBPS MATCHES "^(150|300|600|1200)$")
    message(FATAL_ERROR "DSHOT_RATE_KBPS must be empty, 150, 300, 600, or 1200")
endif()

# Fixed one-way or bidirectional firmware mode (NO runtime auto-switch).
# Empty uses dshot_config.hpp (default = bidirectional).
set(DSHOT_BIDIRECTIONAL "" CACHE STRING "Override DShot mode: 1=bidir, 0=one-way; empty=header default")
if(NOT DSHOT_BIDIRECTIONAL STREQUAL "" AND
   NOT DSHOT_BIDIRECTIONAL MATCHES "^[01]$")
    message(FATAL_ERROR "DSHOT_BIDIRECTIONAL must be empty, 0, or 1")
endif()

# Portable source ZIP bundles Eigen headers in third_party/eigen.
# Prefer this local copy, then a caller-provided EIGEN3_INCLUDE_DIR, then
# the pinned upstream download. Neither a Git checkout nor a ../../eigen
# folder is needed when building the extracted ZIP elsewhere.
set(EIGEN3_INCLUDE_DIR "" CACHE PATH "Directory containing Eigen/Dense")
if(NOT EIGEN3_INCLUDE_DIR AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/Eigen/Dense")
    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
endif()

if(NOT EIGEN3_INCLUDE_DIR)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c
        GIT_SHALLOW FALSE
    )
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
    endif()
    set(EIGEN3_INCLUDE_DIR "${eigen_SOURCE_DIR}")
endif()

if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Dense")
    message(FATAL_ERROR "Eigen/Dense was not found at ${EIGEN3_INCLUDE_DIR}")
endif()

add_executable(pico_copter
    pico_copter.cpp
    debug_usb.cpp
    sensor.cpp
    ekf.cpp
    lsm9ds1_reg.c
    pwm.cpp
    esc_serial_uart.cpp
    radio.cpp
    control.cpp
)

if(NOT DSHOT_RATE_KBPS STREQUAL "")
    target_compile_definitions(pico_copter PRIVATE DSHOT_RATE_KBPS=${DSHOT_RATE_KBPS})
endif()
if(NOT DSHOT_BIDIRECTIONAL STREQUAL "")
    target_compile_definitions(pico_copter PRIVATE DSHOT_BIDIRECTIONAL=${DSHOT_BIDIRECTIONAL})
endif()

target_include_directories(pico_copter PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${EIGEN3_INCLUDE_DIR}"
)

target_link_libraries(pico_copter PRIVATE
    hardware_spi
    hardware_pwm
    hardware_pio
    hardware_clocks
    hardware_uart
    pico_stdlib
    pico_multicore
)

pico_generate_pio_header(pico_copter ${CMAKE_CURRENT_LIST_DIR}/dshot_tx.pio)
pico_generate_pio_header(pico_copter ${CMAKE_CURRENT_LIST_DIR}/dshot_oneway.pio)

pico_enable_stdio_usb(pico_copter 1)
pico_enable_stdio_uart(pico_copter 0)
pico_add_extra_outputs(pico_copter)
