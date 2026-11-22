cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME cubesat)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Libraries/aic3104.c
    Libraries/adf7021.c
    Libraries/adf4360.c
    Libraries/ltc5599.c
    Libraries/communication.c
    Libraries/source.c
    Libraries/adi_delay.c
    Libraries/ttc_link.c
    Libraries/ttc_vote.c
    Libraries/ttc_store.c
    Libraries/ttc_radio.c
    Libraries/ttc_app.c
    Libraries/modem.c
    Libraries/ds18b20/ds18b20.c
    Libraries/ds18b20/onewire.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Libraries/ds18b20
    Libraries/
)

# Bench channel centres in Hz. The integer-N downlink PLL uses a 12.5 kHz grid.
set(TTC_UPLINK_HZ 145000000 CACHE STRING "VHF uplink centre in Hz")
set(TTC_DOWNLINK_HZ 436500000 CACHE STRING "UHF downlink centre in Hz")
if(NOT TTC_UPLINK_HZ MATCHES "^[1-9][0-9]*$" OR NOT TTC_DOWNLINK_HZ MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "TTC channel centres must be integer frequencies in Hz")
endif()
math(EXPR downlink_remainder "${TTC_DOWNLINK_HZ} % 12500")
if(TTC_UPLINK_HZ LESS 144000000 OR TTC_UPLINK_HZ GREATER 146000000 OR
   TTC_DOWNLINK_HZ LESS 435000000 OR TTC_DOWNLINK_HZ GREATER 438000000 OR downlink_remainder)
    message(FATAL_ERROR "Select a 144-146 MHz uplink and a 435-438 MHz downlink on the 12.5 kHz grid")
endif()

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    TTC_UPLINK_HZ=${TTC_UPLINK_HZ}U
    TTC_DOWNLINK_HZ=${TTC_DOWNLINK_HZ}U
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
