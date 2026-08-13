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
set(CMAKE_PROJECT_NAME robocup)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
set_source_files_properties(
    "${CMAKE_SOURCE_DIR}/Core/Src/main.c"
    PROPERTIES LANGUAGE CXX
)



# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Code/1_Middleware/1_Driver/CAN/drv_can.cpp
    Code/1_Middleware/2_Algorithm/PID/alg_pid.cpp
    Code/1_Middleware/3_System/Timestamp/sys_timestamp.cpp
    Code/1_Middleware/1_Driver/SPI/drv_spi.cpp
    Code/1_Middleware/2_Algorithm/Filter/EKF/alg_filter_ekf.cpp
    Code/1_Middleware/2_Algorithm/Basic/alg_basic.cpp
    Code/1_Middleware/1_Driver/USB/drv_usb.cpp
    Code/1_Middleware/1_Driver/UART/drv_uart.cpp
    Code/1_Middleware/2_Algorithm/Matrix/alg_matrix.cpp
    Code/1_Middleware/2_Algorithm/Quaternion/alg_quaternion.cpp

    Code/2_Device/Motor/Motor_DJI/dvc_motor_dji.cpp
    Code/2_Device/ROS/dvc_motion_protocol.cpp
    Code/2_Device/BSP/Buzzer/bsp_buzzer.cpp
    Code/2_Device/BSP/Power/bsp_power.cpp
    Code/2_Device/BSP/BMI088/bsp_bmi088.cpp
    Code/2_Device/BSP/BMI088/Accel/bsp_bmi088_accel.cpp
    Code/2_Device/BSP/BMI088/Gyro/bsp_bmi088_gyro.cpp
    Code/2_Device/Synex/dvc_synex.cpp

    Code/3_Chariot/1_Module/Chassis/Four_whell/crt_fw.cpp
    Code/3_Chariot/1_Module/Music/crt_music.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Code/1_Middleware/1_Driver/CAN
    Code/1_Middleware/2_Algorithm/PID
    Code/1_Middleware/3_System/Timestamp
    Code/1_Middleware/2_Algorithm/Basic
    Code/1_Middleware/1_Driver/USB
    Code/1_Middleware/1_Driver/UART
    Code/1_Middleware/1_Driver/SPI
    Code/1_Middleware/2_Algorithm/Filter/EKF    
    Code/1_Middleware/2_Algorithm/Matrix
    Code/1_Middleware/2_Algorithm/Quaternion

    Code/2_Device/ROS
    Code/2_Device/Motor/Motor_DJI
    Code/2_Device/BSP/Buzzer
    Code/2_Device/BSP/Power
    Code/2_Device/BSP/BMI088
    Code/2_Device/BSP/BMI088/Accel
    Code/2_Device/BSP/BMI088/Gyro
    Code/2_Device/Synex
    
    Code/3_Chariot/1_Module/Chassis/Four_whell
    Code/3_Chariot/1_Module/Music

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
