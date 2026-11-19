# Web Flasher

Run the live flasher at https://1filth1.github.io/air-suspension/webflasher/.

The page uses Espressif's esptool-js and the browser Web Serial API. It accepts the sparse Intel HEX asset for the selected release and board.

Run locally from this directory:

    python -m http.server 8000

Then open http://localhost:8000 in a current Chrome or Edge browser. Web Serial also requires a secure context, so use localhost or HTTPS.

Firmware releases are publicly available on GitHub. Open the release page, download the exact HEX file named under the selected device, return to the flasher, select that file and connect the board over USB.

The release list is in releases.json. Add a new entry after creating each prerelease. This page compares the selected filename with the selected device's expected asset before enabling Flash.

Sparse HEX images preserve sectors that are absent from the image, including the NVS data partition. Do not check “erase all” unless you want to clear controller profiles and calibration.
