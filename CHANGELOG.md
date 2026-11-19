# Air Suspension — Release notes

## v0.1.0-rc.4

### Fixed
- Fixed controller-owned preset synchronization on the main TTGO T-Display remote by retrying settings requests until a valid settings frame arrives.
- Added version-2 settings-frame validation and controller preset reception to the keychain remote.
- Added controller preset names and four-wheel target pressures to the keychain display; presets remain read-only and authoritative in the controller.
- Reset settings synchronization state when the controller link is lost.

### Important notes
- Install the controller and all remotes from the same RC as one matching firmware set.
- This update addresses preset synchronization and display only; it does not add preset activation from the keychain.
- Automatic target adjustment remains pulse-based. The requested overshoot-then-slow-bleed settling strategy is not included in this candidate.
- This is a bench-test release candidate, not a road-validated safety system. Verify sensor calibration, valve polarity, pressure limits, timeout behavior, and emergency stop with the pneumatic actuators safely isolated before any vehicle use.
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
- This is a bench-test release candidate, not a road-validated safety system. Verify sensor calibration, valve polarity, pressure limits, timeout behavior, and emergency stop with the pneumatic actuators safely isolated before any vehicle use.
- The web flasher's full-erase option clears NVS settings and sensor calibration. Leave it unchecked for normal updates.
