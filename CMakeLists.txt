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
set(CMAKE_PROJECT_NAME chassis_board)

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
    # Add user defined source files
    ${CMAKE_CURRENT_SOURCE_DIR}/User/BSP/bsp_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/BSP/bsp_time.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/BSP/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/BSP/bsp_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/BSP/bsp_instance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/device_motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/device_dr16.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/router.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/device_instance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/user_register.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/filter_lpf1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/control_pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/chassis.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/bmi088/bmi08a.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/bmi088/bmi08g.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/bmi088/bmi08xa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/driver_bmi088.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/kalman.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/MahonyAHRS/MahonyAHRS.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/MadgwickAHRS/MadgwickAHRS.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Application/run.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Application/init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Utils/clamp.c

    # Add DSP source files
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/User/BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Application
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/bmi088
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Utils
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/MahonyAHRS
    # ${CMAKE_CURRENT_SOURCE_DIR}/User/Component/MadgwickAHRS
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/User/DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
    __FPU_PRESENT__=1
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
