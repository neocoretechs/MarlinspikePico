# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (muscppt be before project)
include(pico_sdk_import.cmake)

project(Marlinspike C CXX ASM)

# Enable TinyUSB
set(PICO_TINYUSB_DEVICE 1)
set(PICO_TINYUSB_HOST 0)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Marlinspike
    main.cpp
    RoboCore_main.cpp
    Ultrasonic.cpp
    DigitalPin.h
    AnalogPin.h
    IMU/Adafruit_10DOF.cpp
    IMU/Adafruit_BMP085U.cpp
    IMU/Adafruit_L3GD20U.cpp
    IMU/Adafruit_LSM303U.cpp
    IMU/PitchRollHeading.cpp
    Propulsion/AbstractMotorControl.cpp
    Propulsion/AbstractSmartMotorControl.h
    Propulsion/DelayHBridgeDriver.cpp
    Propulsion/HBridgeDriver.cpp
    Propulsion/RoboteqDevice.cpp
    Propulsion/SplitBridgeDriver.cpp
    Propulsion/SwitchBridgeDriver.cpp
    Propulsion/SwitchHBridgeDriver.cpp
    Propulsion/DiffDrive.h
    Print.cpp
    pid_params.h
    Odom.h
    Stream.cpp
    VariablePWMDriver.cpp
    HardwareSerial.cpp
    WPWM.cpp
    WShift.cpp
    WString.cpp
    )
     
# Make sure tusb_config.h is visible to TinyUSB
target_include_directories(Marlinspike PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

message(STATUS "Project source dir: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Looking for tusb_config.h in: ${CMAKE_CURRENT_SOURCE_DIR}")
# pull in common dependencies
target_link_libraries(Marlinspike pico_stdlib 
hardware_uart hardware_timer hardware_pwm hardware_gpio hardware_adc hardware_i2c hardware_spi
hardware_watchdog hardware_clocks hardware_sync hardware_dma hardware_flash hardware_rtc
hardware_resets hardware_vreg hardware_xosc hardware_structs hardware_exception hardware_divider
tinyusb_device tinyusb_board
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(Marlinspike pico_cyw43_arch_none)
endif()

# create map/bin/hex file etc.
pico_add_extra_outputs(Marlinspike)

# add url via pico_set_program_url
