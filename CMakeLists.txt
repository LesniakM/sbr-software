cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sbr-software)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/mpu6050.c)
target_sources(app PRIVATE src/motors.c)
target_sources(app PRIVATE src/usb_module.c)
target_sources(app PRIVATE src/buttons.c)
