# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

# use base board stm32f401_mini
set(BOARD stm32f401_mini)

# use flash way openocd
set(BOARD_FLASH_RUNNER openocd)

# overlay the dts file
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/stm32f401_mini.overlay)

# add the dw3000-decadriver directory as a zephyr module by hand, not needed when using west
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/zephyr-dw3000-driver/)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(zephyr-dw3000)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

FILE(GLOB ex_sources src/device/*/*.c)
target_sources(app PRIVATE ${ex_sources})

FILE(GLOB ex_sources src/test/*/*.c)
target_sources(app PRIVATE ${ex_sources})

FILE(GLOB share_sources src/shared_data/*.c)
target_sources(app PRIVATE ${share_sources})

target_sources(app PRIVATE src/MAC_802_15_4/mac_802_15_4.c)

target_include_directories(app PRIVATE src)
target_include_directories(app PRIVATE src/MAC_802_15_4)
target_include_directories(app PRIVATE src/shared_data)

target_compile_definitions(app PRIVATE TEST_READING_DEV_ID)
# target_compile_definitions(app PRIVATE TEST_AES_SS_TWR_INITIATOR) # stlink
# target_compile_definitions(app PRIVATE TEST_AES_SS_TWR_RESPONDER)   # daplink

# target_compile_definitions(app PRIVATE DEVICE_TAG) # stlink
# target_compile_definitions(app PRIVATE DEVICE_ANCHOR) # daplink

target_compile_definitions(app PRIVATE CONFIG_FPU)
target_compile_definitions(app PRIVATE CMAKE_EXPORT_COMPILE_COMMANDS=1)

