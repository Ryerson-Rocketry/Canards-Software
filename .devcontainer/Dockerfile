# get Ubuntu-22.04 image
FROM mcr.microsoft.com/devcontainers/base:jammy

# stop any text-based interface called "dialog" and noninteractive tells the installer:
# skip menus, pick default answer for every question and finish job without talking to me
# use ARG so it only affects build not process and not final container
ARG DEBIAN_FRONTEND=noninteractive

RUN groupadd -f plugdev && usermod -aG plugdev vscode

RUN apt-get update && apt-get -y install --no-install-recommends \
    make \
    gcc-arm-none-eabi \
    binutils-arm-none-eabi \
    libnewlib-arm-none-eabi \
    openocd \
    usbutils \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
