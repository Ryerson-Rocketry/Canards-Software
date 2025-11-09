#!/bin/bash
# 1. Set up "Shebang" and Safety
set -e # exit immediately if any cmd fails

# 2. Configuration: Setup Variables
BUILD_TYPE=${1:-Debug}
BUILD_DIR="build/${BUILD_TYPE}"

CPU="cortex-m4"
FPU="fpv4-sp-d16"
FLOAT_ABI="hard"

LINKER_SCRIPT="stm32f446retx_flash.ld"
PREFIX="arm-none-eabi"

# 3. Find Include Paths FIRST
echo "Finding Include Paths"

# Hard-coded, MUST-HAVE Paths
INC_PATHS="-ICore/Inc"
INC_PATHS="$INC_PATHS -IDrivers/STM32F4xx_HAL_Driver/Inc"
INC_PATHS="$INC_PATHS -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy"
INC_PATHS="$INC_PATHS -IDrivers/CMSIS/Device/ST/STM32F4xx/Include"
INC_PATHS="$INC_PATHS -IDrivers/CMSIS/Include"
INC_PATHS="$INC_PATHS -IMiddlewares/Third_Party/FreeRTOS/Source/include"
INC_PATHS="$INC_PATHS -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"
INC_PATHS="$INC_PATHS -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
INC_PATHS="$INC_PATHS -ICore/Src/Drivers/ms5611"
INC_PATHS="$INC_PATHS -ICore/Src/Drivers"

# Automatic discovery for other custom headers
INC_PATHS_LIST=$(find Core Drivers Middlewares -name "*.h" -exec dirname {} \; 2>/dev/null | sort -u)

for path in $INC_PATHS_LIST; do
    # Add the path, but avoid duplicating what we just added
    if [[ "$INC_PATHS" != *"$path"* ]]; then
        INC_PATHS="$INC_PATHS -I$path"
    fi
done

# 4. Find Source Files
echo "Finding Source Files"

# Find all .c files, but filter out ALL problematic/unwanted files:
# -main_s.c, tz_context.c: ARMv8-M template files (wrong architecture).
# -os_systick.c, os_tick_gtim.c, os_tick_ptim.c: Template files that include missing headers.
C_SOURCES=$(find Core Drivers Middlewares -name "*.c" | grep -v "main_s.c" | grep -v "tz_context.c" | grep -v "os_systick.c" | grep -v "os_tick_gtim.c" | grep -v "os_tick_ptim.c")
S_SOURCES=$(find . -name "startup_*.s") # find assembly startup file

# 5. Release vs Debug Flags
echo "Setting Compiler flags for ${BUILD_TYPE}"

# CRITICAL FLAGS:
# -DSTM32F446xx: Defines the chip (fixes IRQn and RCC errors)
# -DCMSIS_OS_V2: Enables the V2 FreeRTOS API types (fixes osThreadId_t, etc.)
# -std=gnu11: Enables modern C syntax (fixes struct initialization errors)
# -include: Forces the master header file to load first.
BASE_CFLAGS="-mcpu=$CPU -mthumb -mfpu=$FPU -mfloat-abi=$FLOAT_ABI -DSTM32F446xx -DCMSIS_OS_V2 -std=gnu11 -include Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
LDFLAGS="-mcpu=$CPU -mthumb -mfpu=$FPU -mfloat-abi=$FLOAT_ABI -T$LINKER_SCRIPT --specs=nosys.specs"

if [ "$BUILD_TYPE" == "Release" ]; then
    CFLAGS="$BASE_CFLAGS -O3"
else
    CFLAGS="$BASE_CFLAGS -g3 -Wall"
fi

# 6. Handoff to Makefile
echo "Run Make"
mkdir -p $BUILD_DIR

make all \
    PREFIX="$PREFIX" \
    C_SOURCES="$C_SOURCES" \
    S_SOURCES="$S_SOURCES" \
    INC_PATHS="$INC_PATHS" \
    CFLAGS="$CFLAGS" \
    LDFLAGS="$LDFLAGS" \
    BUILD_DIR=$BUILD_DIR

# 8. If output does not have .elf force .elf extension
TARGET="Canards"

# Check if the linker outputted 'Canards' without the extension
if [ -f "$BUILD_DIR/$TARGET" ]; then

    mv "$BUILD_DIR/$TARGET" "$BUILD_DIR/$TARGET.elf" 

    if [ -f "$BUILD_DIR/$TARGET" ]; then
        echo "Removing duplicate linker output: $TARGET"
        rm -f "$BUILD_DIR/$TARGET"
    fi
fi