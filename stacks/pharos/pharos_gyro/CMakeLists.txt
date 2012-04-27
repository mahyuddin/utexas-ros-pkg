cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_find_ros_package(rosserial_arduino)
set(ARDUINO_SDK_PATH ~/source/arduino-1.0)
include(${rosserial_arduino_PACKAGE_PATH}/cmake_scripts/rosserial.cmake)

set(FIRMWARE_NAME gyro)

set(${FIRMWARE_NAME}_BOARD atmega328)         # Arduino Target board
set(${FIRMWARE_NAME}_SRCS src/gyro.cc)
set(${FIRMWARE_NAME}_PORT /dev/ttyUSB0)            # Serial upload port
generate_ros_firmware(${FIRMWARE_NAME})

