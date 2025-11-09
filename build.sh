## 1. Set up "Shebang" and Safety

#!/bin/bash # This tells the computer to use Bash interpreter to run this file
set -e # exit immediately if any cmd fails

## 2. Configuration: Setup Variables
BUILD_TYPE=${1:-Debug}
BUILD_DIR="build/${BUILD_TYPE}"

CPU="cortex-m4"
FPU="fpv4-sp-d16"
FLOAT_ABI="hard"

LINKER_SCRIPT="stm32f446retx_flash.ld"
PREFIX="arm-none-eabi"

## 3. Find files
echo "Finding Source Files"

C_SOURCES=$(find Core Drivers Middlewares -name "*.c") # find all .c files in those directories
S_SOURCES=$(find . -name "startup_*.s") # find assembly startup file

## 4. Find Include Paths
echo "Finding Include Paths"

INC_PATHS=$(find Core Drivers Middlewares -name "*.h" -exec dirname {} \; | \ 
    sort -u | \ 
    sed 's/^/-I/')

## 5. Release vs Debug

echo "Setting Compiler flags for ${BUILD_TYPE}"

BASE_CFLAGS="-mcpu=$CPU -mthumb -mfpu=$FPU -mfloat-abi=$FLOAT_ABI"
LDFLAGS="-mcpu=$CPU -mthumb -mfpu=$FPU -mfloat-abi=$FLOAT_ABI -T$LINKER_SCRIPT --specs=nosys.specs"

if [ "$BUILD_TYPE" == "Release" ]; then
    CFLAGS="$BASE_CFLAGS -O3"
else
    CFLAGS="$BASE_CFLAGS -g3 -Wall"
fi

## 6. Handoff to Makefile
echo "Run Make"
mkdir -p $BUILD_DIR

make all \
    PREFIX="$PREFIX" \
    C_SOURCES="$C_SOURCES" \
    S_SOURCES="$S_SOURCES" \
    INC_PATHS="$INC_PATHS" \
    CFLAGS="$CFLAGS" \
    LDFLAGS="$LDFLAGS" \
    BUILD_DIR=$BUILD_DIR"

    