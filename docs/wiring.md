# Wiring

The controller pin assignments below come from controller/main.c.

## Pressure sensors

Use five ratiometric pressure sensors with a common ground and signal conditioning suitable for the ESP32 ADC range. The firmware applies the 28/18 divider correction; keep the input divider used by the board and do not feed a 5 V signal directly into an ESP32 pin.

| Signal | ESP32 DevKit V1 GPIO | Firmware ADC channel |
| --- | ---: | --- |
| Front left pressure | 32 | ADC1_CH4 |
| Front right pressure | 33 | ADC1_CH5 |
| Rear left pressure | 34 | ADC1_CH6 |
| Rear right pressure | 35 | ADC1_CH7 |
| Tank pressure | 36 | ADC1_CH0 |

The firmware expects wheel sensors up to 150 psi and a tank sensor up to 300 psi. Confirm the installed sensor range and analog scaling before powering the controller.

## Valve and compressor drivers

Each GPIO output is a low-current logic signal. Connect it to a suitably rated automotive MOSFET or relay driver, with flyback protection for inductive coils and a fuse sized for the load. Share signal ground between the ESP32, driver stage, and sensors.

| Function | GPIO |
| --- | ---: |
| FL fill / dump | 4 / 13 |
| FR fill / dump | 14 / 16 |
| RL fill / dump | 17 / 18 |
| RR fill / dump | 19 / 21 |
| Compressor relay driver | 22 |

Use a separately fused supply for the compressor and valves. Keep load current off the DevKit and its USB supply. Verify valve polarity and fail-closed behavior before connecting the pneumatic system.

## Wireless remotes

The T-Display and 2-inch remote connect to the controller over encrypted ESP-NOW on channel 1. No signal cable is needed between them. Power each remote through its USB or board-rated supply.
