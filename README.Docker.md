### Build and flash

The image cross-compiles the project with `arm-none-eabi-gcc` and flashes the
resulting ELF through an ST-LINK probe with OpenOCD.

Build the image after changing the firmware:

```sh
docker build --build-arg BUILD_TYPE=Debug -t canards-firmware .
```

On Linux, connect the ST-LINK probe and run:

```sh
docker run --rm --privileged \
	--device=/dev/bus/usb:/dev/bus/usb \
	canards-firmware
```

The same workflow is available through Compose:

```sh
docker compose up --build
```

The USB device mapping above is for Linux. Docker Desktop on Windows and
macOS requires passing the probe through to the Docker VM or using a host-side
OpenOCD/ST-LINK utility; Docker cannot access physical USB devices there by
default.