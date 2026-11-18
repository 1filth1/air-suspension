# Air Suspension

**Веб‑флешер:** [запустить флешер](https://1filth1.github.io/air-suspension/webflasher/) · [скачать прошивки v0.1.0-rc.1](https://github.com/1filth1/air-suspension/releases/tag/v0.1.0-rc.1)

ESP32 based air-suspension controller with pressure telemetry and wireless remotes.

## Current release candidate scope

The controller is the source of truth for vehicle setup:

- four wheel-pressure presets (PARK, DRIVE, LIFT);
- wheel pressure limits and tank/compressor thresholds;
- automatic regulation and manual valve timeouts;
- sensor calibration offsets and calibration references.

The ESP32 controller saves these values in NVS. T-Display and 2-inch remotes request the settings after connecting, show controller values, and send edits back over encrypted ESP-NOW. The remote keeps only its own interface preferences, such as screen brightness and sound volume.

The settings schema moves from v2 to v3. On first boot with this firmware the controller migrates existing calibration and controller limits, then adds the three preset profiles. Remote-local pressure presets are ignored. Controller NVS is preserved by the web flasher's sparse Intel HEX images.

## Hardware

- Controller: ESP32 DevKit V1
- Remotes: ESP32-S3 T-Display and the 2-inch ST7789 remote
- Pressure inputs: four wheel sensors and one tank sensor
- Outputs: eight valve-driver signals and one compressor relay-driver signal

Use external drivers and fused vehicle power for valves and compressor. ESP32 GPIO pins are logic signals; do not connect inductive loads directly.

See [the wiring diagram](docs/wiring.svg) and [wiring notes](docs/wiring.md).

## Firmware source

- [Controller](controller/main.c)
- [T-Display remote](button%20pult%20t-display/main.cpp)
- [2-inch remote](buttonpult%202%20inch/main.cpp)

The pult CYD folders contain display diagnostics, not supported controller remotes.

## Build

The controller is an ESP-IDF 5.4 project in controller/. The remotes are PlatformIO projects in their respective directories. GitHub Actions builds sparse Intel HEX images for each supported device.

A release candidate is created by pushing a tag like v0.1.0-rc.1. The workflow builds and attaches:

- air-suspension-controller.hex
- air-suspension-remote-t-display.hex
- air-suspension-remote-2inch.hex

## Web flasher

Open the [web flasher](https://1filth1.github.io/air-suspension/webflasher/) in Chrome or Edge. Choose the release and device, download the matching Intel HEX asset from the GitHub release, select that file in the flasher, then connect the board over USB.

The HEX images are sparse: flashing updates the bootloader, partition table and app without erasing the NVS sector that stores vehicle settings. Use the explicit “erase all” option only when you intend to reset all controller settings.

The first release entry is maintained in [webflasher/releases.json](webflasher/releases.json). Add new release tags there when publishing later candidates.

## Release build

See [the release workflow](.github/workflows/release-candidate.yml). It creates a GitHub prerelease for every v*-rc* tag after all three firmware images build successfully.
