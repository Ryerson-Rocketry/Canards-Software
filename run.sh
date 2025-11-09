#!/usr/bin/env bash

# 1. Configuration
BUILD_TYPE="Debug"
TARGET_ELF="build/${BUILD_TYPE}/Canards.elf"

OPENCD_CONFIGS="-f interface/stlink.cfg -f target/stm32f4x.cfg"

if [[ ! -f "$TARGET_ELF" ]]; then
    echo "Error: $TARGET_ELF not found"
    echo "Please run ./build.sh first"
    exit 1
fi

echo "Opening OpenOCD in background"

# run OpenOCD server in the background using config details then redirect output to opencd.log so it doesnt spam terminal
openocd $OPENOCD_CONFIGS > openocd.log 2>&1 &

# get process id of last cmd so we can kill it later if needed
OPENOCD_PID=$!

# sleep to let openocd to open
sleep 1

# sends a test signal to check if process is still running
# if test signal fails then stop script
if ! kill -0 $OPENOCD_PID > /dev/null 2>&1; then
    echo "Error: OpenOCD failed to start, check openocd.log for details."
    exit 1
fi

# load .elf file
arm-none-eabi-gdb "$TARGET_ELF" \
    # connect GDB client to OpenOCD server
    -ex "target extended-remote :3333" \

    -ex "monitor reset halt" \
    -ex "load" \

echo "Shutting down OpenOCD"
kill $OPENOCD_PID
rm openocd.log