# Kalman filter bench calibration mode

This is an **opt-in, non-flight firmware configuration** for the **old STM32 Canards PCB**. It is **not a calibrated filter** and **must never be flown**.

## Build

From a separate, reviewed branch:

```sh
cmake --preset Debug -DKF_BENCH_TEST=ON
cmake --build build
```

Check that CMake prints the `KF_BENCH_TEST is ON` warning. Make sure the intended binary was actually flashed. The CMake option is cached: **do not reuse a bench-mode build for flight**. Reconfigure with `-DKF_BENCH_TEST=OFF` in a clean build directory and independently review flight firmware before any flight use.

## Bench-only behavior

- State machine transitions are skipped, so the firmware remains in `STATE_PAD`.
- Altitude KF (`altEstTask`) is notified in PAD; attitude EKF runs normally.
- SD logging runs in PAD. The CSV filename is `KFBENCH.CSV` rather than `LOG.CSV`.
- Servo PWM is not started in `main.c`; the bench control task never commands PWM and holds the servo high-side switch low.
- Logging automatically flushes and closes the file **5 minutes after the SD task starts**, then stops. The duration is `KF_BENCH_LOG_DURATION_MS` in `flight_configs.h` and can be adjusted before building.
- **Each reboot overwrites `KFBENCH.CSV`**. Copy and rename it between tests.
- Normal firmware when `KF_BENCH_TEST=OFF` retains its existing state transitions, PAD logging restriction, and servo behavior.

## Critical safety and verification

1. Obtain approval from the PCB owner/avionics lead for the exact tested firmware revision.
2. **Physically disconnect** servo/actuator power, deployment circuits, igniters, motors, and anything capable of moving or firing. Software isolation is not a substitute for physical isolation.
3. Secure the board to a nonconductive fixture, use approved bench power, and review the complete startup GPIO behavior before energizing.
4. Back up existing SD contents first. Never remove the card during writes.
5. Build the bench firmware and review the resulting diff. Verify the state stays PAD, the servo power switch is low, PWM does not run, and no outputs actuate. Do this with hardware disconnected.
6. Start with a stationary recording; verify data is nonzero, timestamps increment, and **altitude/velocity estimates now update in PAD** rather than remaining permanently zero. If they remain zero, investigate before moving on.
7. Wait for the 5-minute automatic close, then power down safely, retrieve and rename `KFBENCH.CSV`.
8. For additional recordings, repeat with documented known-angle rotations or known heights, preferably with a reference fixture. Each 5-minute session overwrites the previous bench log.

## Limitations

- The source software applies low-pass filtering to IMU values before the CSV snapshot. Logged readings do not represent raw IMU noise. New raw-IMU logging would be needed for sensor-level noise identification.
- The altitude KF's present acceleration projection, gravity handling, and barometer-health handling remain unchanged. This PR enables testing; it does not claim those models are correct.
- Bench rotations and height tests do not validate performance under actual rocket thrust, vibration, or aerodynamic pressures.
- No hardware test, flash or full STM32 toolchain build has been performed by the author of this patch; **review and bench verification are mandatory**.
