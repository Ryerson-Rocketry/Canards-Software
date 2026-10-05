#!/usr/bin/env bash

help(){
  echo "Usage: $0 [debug/release] [build/run/all]"
  echo " Build Types:"
  echo "   debug   : Compile the project with debug"
  echo "   release : Compile the project with optimizations"
  echo " Actions:"
  echo "   build   : Compile and build the project"
  echo "   run     : Flash the project to the MCU and run it"
  echo "   all     : Build and run the project (default action)"
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    help
    return 0
fi

TOOLCHAIN_FILE="cmake/gcc-arm-none-eabi.cmake"
BUILD_DIR="build"
PROJECT_NAME="ControlSystems_Canards"

TARGET_MODE="${1:-debug}" # take first positional arg, default to "debug"
ACTION="${2:-all}" # take second positional arg, default to "all"

case "$TARGET_MODE" in
    debug)
      BUILD_DIR="build/Debug"
      CMAKE_BUILD_TYPE=Debug
      ;;
    release)
      BUILD_DIR="build/Release"
      CMAKE_BUILD_TYPE=Release
      ;;
    *)
      echo "Invalid build type: ${TARGET_MODE}"
      help
      ;;
esac

ELF_FILE="${BUILD_DIR}/${CMAKE_BUILD_TYPE}/${PROJECT_NAME}.elf"

build(){
    cmake -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" -G "Ninja"
    echo "Compiling ${CMAKE_BUILD_TYPE} target..."
    
    cmake --build "${BUILD_DIR}" --parallel "$(nproc)"
}

do_run(){
  if [ ! -f "${ELF_FILE}" ]; then
    echo "Error: ${ELF_FILE} not found. Please build the project first."
    return 1
  fi

  echo "Flashing ${ELF_FILE} to the MCU..."
  #NEEDS RUNNING FUNCTIONALITY
}

case "$ACTION" in
    build)
        build
        ;;
    run)
        do_run
        ;;
    all)
        build
        do_run
        ;;
    *)
        echo "Invalid action: ${ACTION}"
        help
        ;;
esac