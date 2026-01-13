#!/bin/bash

# Colors for pretty output
GREEN='\033[0-32m'
RED='\033[0-31m'
NC='\033[0m' # No Color

echo "Checking STM32 Development Environment..."

# 1. Check for Compiler
if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo -e "${GREEN}[OK]${NC} ARM GCC Compiler found."
else
    echo -e "${RED}[FAIL]${NC} ARM GCC Compiler NOT found. Check your Dockerfile."
fi

# 2. Check for Make
if command -v make >/dev/null 2>&1; then
    echo -e "${GREEN}[OK]${NC} Make found."
else
    echo -e "${RED}[FAIL]${NC} Make NOT found."
fi

# 3. Check for ST-Link Hardware
if lsusb | grep -qi "STMicroelectronics"; then
    echo -e "${GREEN}[OK]${NC} ST-Link hardware detected in container."
else
    echo -e "${RED}[FAIL]${NC} ST-Link NOT detected. If on Windows, did you run usbipd?"
fi

# 4. Try a dry-run compile
echo "Attempting test build..."
make -n >/dev/null 2>&1 # -n means 'dry run', don't actually build
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[OK]${NC} Makefile is valid."
else
    echo -e "${RED}[FAIL]${NC} Makefile test failed."
fi

# 5. Check if udev rules exist 
sudo chown -R vscode /dev/bus/usb