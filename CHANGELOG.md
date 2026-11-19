# Air Suspension — Release notes

## v0.1.0-rc.6

### Documentation
- Updated the keychain remote instructions to match the current two-button controls, preset selection and activation, controller settings synchronization, and manual-mode behavior.
- Clarified that the shared ESP-NOW key is present in the published firmware and that this release candidate is for bench testing only.
- Corrected the keychain release-workflow and minimum-pressure documentation.

### Firmware and release status
- No firmware behavior changes are included in this documentation-only RC6 preparation commit; firmware sources are unchanged from the selected base revision.
- Build and upload the controller, T-Display remote, 2-inch remote, and keychain as one matching release set. Do not publish RC6 until all four builds pass and the firmware set has been bench-tested.
- This is not a road-validated safety system. Verify sensor calibration, valve polarity, independent pressure protection, timeout behavior, and emergency stop with pneumatic actuators disconnected before any controlled bench test.
- **Security warning:** the ESP-NOW key is compiled into the firmware and is shared across devices. Anyone who obtains the key can attempt to pair with the controller. Do not treat ESP-NOW encryption as device-specific authentication or deploy this firmware in a vehicle until per-installation keys and an authenticated pairing process are implemented.
- Automatic target adjustment remains pulse-based; overshoot-then-slow-bleed settling is not implemented.
- The web flasher's full-erase option clears NVS settings and sensor calibration. Leave it unchecked for normal updates.

## v0.1.0-rc.5

### Keychain remote
- Added preset selection using a short SELECT press and preset activation using a short CONFIRM press.
- Added a display indicator for the selected controller-owned preset and its four wheel targets.
- Holding SELECT enters manual mode; existing manual valve control is retained.
- Preset targets and names remain authoritative in the controller.

### Notes
- Preset activation requires a controller connection and synchronized settings.
- All four firmware targets are included in the release build workflow.
- Hardware behavior still requires bench testing; this firmware is not road-validated.

## v0.1.0-rc.4

### Fixed
- Fixed controller-owned preset synchronization on the main TTGO T-Display remote by retrying settings requests until a valid settings frame arrives.
- Added version-2 settings-frame validation and controller preset reception to the keychain remote.
- Added controller preset names and four-wheel target pressures to the keychain display; presets remain read-only and authoritative in the controller.
- Reset settings synchronization state when the controller link is lost.

### Important notes
- Install the controller and all remotes from the same RC as one matching firmware set.
- Automatic target adjustment remains pulse-based. The requested overshoot-then-slow-bleed settling strategy is not included.
- This is a bench-test release candidate, not a road-validated safety system. Verify sensor calibration, valve polarity, pressure limits, timeout behavior, and emergency stop with the pneumatic actuators disconnected.
- The web flasher's full-erase option clears NVS settings and sensor calibration. Leave it unchecked for normal updates.

## v0.1.0-rc.3

### Added
- Added the compact TTGO T-Display keychain remote with live tank and four-wheel pressure telemetry, link/status indicators, and two-button manual wheel control.
- Added a dedicated PlatformIO build and Intel HEX artifact for the keychain remote.
- Added the keychain device to the web flasher and release firmware mirror.
- Preset names are stored by the controller in NVS and synchronized to compatible remotes; preset labels can be edited from the supported button remotes.

### Changed
- Added a 0.2 bar lower cutoff for manual and automatic wheel pressure release in the controller.
- Updated project and keychain documentation to describe the actual hardware and release workflow.

### Important notes
- Update the controller and all remotes as one matching release set. Older remotes do not understand the version-2 settings frame used for controller-owned preset names.
- The keychain remote uses the common ESP-NOW telemetry/manual-command subset and does not edit presets.
- Automatic target adjustment is still pulse-based. The requested overshoot-then-slow-bleed settling strategy is not part of this candidate yet.
- This is a bench-test release candidate, not a road-validated safety system. Verify sensor calibration, valve polarity, pressure limits, timeout behavior, and emergency stop with the pneumatic actuators disconnected.
- The web flasher's full-erase option clears NVS settings and sensor calibration. Leave it unchecked for normal updates.
