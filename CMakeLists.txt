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
set(CMAKE_PROJECT_NAME controlSystem)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    BSP/Sensor/WT_IMU.c
    BSP/Sensor/WT_IMU.h
    BSP/Sensor/REG.h

)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths

)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Tasks/DataTask.c
    App/Tasks/CmdCtrlTask.c
    App/Common/pc_cmd_parser.c
    App/Common/sensor_cmd_parser.c
    App/Common/cmd_packer.c
    App/CR/CR.c
    App/CR/kinematic.c
    App/Common/can_driver.c
    App/Common/can_driver.h
    BSP/Motor/Motor.c
    BSP/Sensor/IMU.c
    App/Common/XV2_cmd_parser.c
    BSP/Sensor/Sensor.c
    BSP/Motor/X_V2.c
    BSP/Motor/motor_limits.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App
    Drivers
    BSP
)


# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)



#测试代码
#add_executable(test
#    tests/test_todo_functions.c
#
#)
#
#target_include_directories(test PRIVATE
#
#)
#target_link_libraries(test
#    stm32cubemx
#)