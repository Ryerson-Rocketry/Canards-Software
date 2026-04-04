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

set(CMAKE_CXX_STANDARD 14) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ControlSystems_Canards)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# FPU Configuration for STM32F446RE (Cortex-M4F)
# Must come AFTER project() declaration
add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Add the same flags to linker
add_link_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)
# Find the Eigen package
set(Eigen3_DIR "C:/msys64/mingw64/share/eigen3/cmake")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/Drivers/mmc5983ma.c
    Core/Src/Drivers/ms5611.c
    Core/Src/Drivers/lsm6dso32.c
    Core/Src/Drivers/lsm6dso32_app.c
    Core/Src/StateEstimation/altitude_estimation.c
    Core/Src/Utils/math_utils.c
    Core/Src/Utils/servo.c
    Core/Src/Drivers/rfm69.c
    Core/Src/StateEstimation/attitude_estimation.cpp
    Core/Src/StateEstimation/math_utils.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc/Drivers
    Core/Inc/StateEstimation
    Core/Inc/Utils
    Core/Inc/Defs
    Core/Inc/Configs
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -u _printf_float
    # Add user defined libraries
    Eigen3::Eigen
)