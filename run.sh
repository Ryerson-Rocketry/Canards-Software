#!/bin/bash

# Since we are using an stm32f4, we are using the Arm Cortex M4 architecture
# This process involves three main components: a compiler, a build system and a script to glue them all together

# Before running this script you need three things:
# 1. gcc-none-arm-eabi -> this is the toolchain that compiles all the C/C++ code for ARM
# 2. Make/CMake -> Tool that will handle the actual compilation logic
# 3. Stm32CubeMX -> Useful for generating the inital Makefile so you don't have to write one from scratch

# -------- The Build Process ----------------
# 1. Compiling: Convert .c files into .o (object) files
# 2. Linking: Combining .0 files, library files and a Linker Script (.ld) into a final .elf file
# 3. Conversion (Optional): Conver that .elf into a .bin or .hex for flashing

# -------- Creating the Shell Script ---------------------

# exit immediately if a cmd exits with non-zero status
set -e 

# Configuration
PROJECT_NAME="Canards"
BUILD_DIR="build"
OPEN_OCD="interface/stlink.cfg -f target/stm32f4x.cfg"
MODE=${1:-debug}
echo "---- Starting Build Process for $PROJECT_NAME ----"

# 1. Determine mode 
if [ "$1" == "clean" ]; then
    echo "---- Cleaning Up Old Build Files ----"
    rm -rf $BUILD_DIR
elif [ "$MODE" == "run" ]; then
    OUT_DIR="$BUILD_DIR/Run"
    DEBUG_VAL=0
    OPT_VAL="-O3"
else 
    OUT_DIR="$BUILD_DIR/Run"
    DEBUG_VAL=0
    OPT_VAL="-O3"
fi

# 2. Create Clean File Structure
if [ ! -d $BUILD_DIR ]; then
    echo "---- Creating Build Directory ----"
    mkdir $BUILD_DIR
else
    rm -rf $BUILD_DIR
    mkdir $BUILD_DIR

fi

if [ ! -d $OUT_DIR ]; then
    echo "---- Creating Build/{Mode} Directory ----"
    mkdir $OUT_DIR
fi

# 3. Compile the project
# We use Makefile here, the nproc displays all available processing units
# By adding the -j we tell the compiler to run the makefile and do parallel compilation
# We tell it to make a max of nproc amount of jobs to speed up build processes 
echo "---- Compiling ----"
make -j$(nproc) BUILD_DIR="$OUT_DIR" DEBUG="$DEBUG_VAL" OPT="$OPT_VAL"

# 4. Verify the .elf file exists 
ELF_PATH="$OUT_DIR/$PROJECT_NAME.elf"

if [ -f "$ELF_PATH" ]; then
    echo "---- $PROJECT_NAME.elf created successfully ----"
    arm-none-eabi-size "$ELF_PATH"
else
    echo "---- Error: Unable to create $PROJECT_NAME.elf ----"
    exit 1
fi

echo "--- FLASHING ---"
openocd -f interface/stlink.cfg \
        -f target/stm32f4x.cfg \
        -c "program {$ELF_PATH} verify reset exit"

echo "--- Script is Done --- "
exit 0