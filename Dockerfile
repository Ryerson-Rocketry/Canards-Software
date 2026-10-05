# syntax=docker/dockerfile:1

ARG DEBIAN_VERSION=bookworm-slim

FROM debian:${DEBIAN_VERSION}

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        build-essential \
        arm-none-eabi-gcc \
        arm-none-eabi-g++ \
        arm-none-eabi-newlib \
        binutils-arm-none-eabi \
        gdb-multiarch \
        cmake \
        ninja-build \
        libeigen3-dev \
        openocd \
        stlink-tools \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src