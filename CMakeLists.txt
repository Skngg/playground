cmake_minimum_required(VERSION 3.20.0)
set(BOARD nrf52840dk_nrf52840)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

FILE(GLOB app_sources src/*.c)
FILE(GLOB ext_drivers_sources src/ext_drivers/lsm6ds3/*.c)
FILE(GLOB bt_sample_sources src/bt_sample/*.c)
target_sources(app PRIVATE ${app_sources} ${ext_drivers_sources} ${bt_sample_sources})
