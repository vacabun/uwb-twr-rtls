# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# use base board stm32f401_mini
set(BOARD stm32f401_mini)

# use flash way openocd
set(BOARD_FLASH_RUNNER openocd)

# overlay the dts file
# set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/stm32f401_mini_dw1000.overlay)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/stm32f401_mini.overlay)

# add the dw3000-decadriver directory as a zephyr module by hand, not needed when using west
# list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/zephyr-dw3000-driver/)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/zephyr-dw1000-driver/)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(zephyr-dw3000)

FILE(GLOB app_sources src/main.cpp)
target_sources(app PRIVATE ${app_sources})

FILE(GLOB ex_sources src/device/*.cpp)
target_sources(app PRIVATE ${ex_sources})

FILE(GLOB ex_sources src/msg/*.cpp)
target_sources(app PRIVATE ${ex_sources})

target_include_directories(app PRIVATE include)

target_compile_definitions(app PRIVATE DS_TWR)

# target_compile_definitions(app PRIVATE SS_TWR)

# target_compile_definitions(app PRIVATE LOG_LEVEL=LOG_LEVEL_INF)
target_compile_definitions(app PRIVATE LOG_LEVEL=LOG_LEVEL_DBG)

target_compile_definitions(app PRIVATE PAN_ID=0x4321)

if(TAG_DEF)
    message("Build application for tag.")
    target_compile_definitions(app PRIVATE DEVICE_TAG)
    target_compile_definitions(app PRIVATE DEVICE_ADDR=0x0001)
endif()

if(ANCHOR_DEF)
    message("Build application for anthor.")
    target_compile_definitions(app PRIVATE DEVICE_ANCHOR)
    target_compile_definitions(app PRIVATE DEVICE_ADDR=0x1001)
endif()

target_compile_definitions(app PRIVATE CONFIG_FPU)

target_compile_definitions(app PRIVATE CMAKE_EXPORT_COMPILE_COMMANDS=1)
