# T3 Dashboard firmware

Firmware sources for the T3 instrument electronics. The repository keeps the current ESP32 and MSP430 projects together, with the earlier ESP8266 implementation as a migration reference.

**PCB design files are intentionally not part of this repository.** KiCad schematics, board layouts, libraries, fabrication outputs, and other hardware design files stay local and are excluded by `.gitignore`.

## License

Project-authored source in this repository is released under **GNU GPL-3.0-only**; see [`LICENSE`](LICENSE). The repository is intended to be an open-source project. GPL-3.0 does not prohibit commercial use or resale by recipients; it requires downstream distributors to preserve the license freedoms and comply with the license terms. Third-party files that carry their own license notices remain subject to those notices as described in [`ATTRIBUTION.md`](ATTRIBUTION.md).

## Projects

```text
MSP430/MSP_Speed_HW_V3_SIMPLE_20261113/       Current MSP430G2553 CCS project
T3-Dash-V4/firmware/esp32_msp430_webbsl/      ESP-IDF + FreeRTOS project
Korábbi ESP8266/T3_Speed_HW_V3_SW_V2_20250810_2/  Legacy ESP8266 reference
```

The ESP32 currently hosts a WPA2 SoftAP and a web interface at `192.168.4.1`. The web UI includes diagnostics and MSP430 Intel HEX upload over SBW. The MSP430 SBW wiring and file format are documented in the ESP32 project's README.

## Local Wi-Fi configuration

Copy `T3-Dash-V4/firmware/esp32_msp430_webbsl/main/wifi_ap_config.example.h` to `wifi_ap_config.h` and set a private WPA2 password. The local configuration file is ignored by Git. Do not commit home Wi-Fi credentials or access-point passwords.

## Build tools

- ESP32: ESP-IDF 6.x with the Espressif VS Code extension. Build from `T3-Dash-V4/firmware/esp32_msp430_webbsl`.
- MSP430: Code Composer Studio with the MSP430 compiler and target support.

Generated ESP-IDF `build/` and CCS `Debug/` outputs are excluded. Commit source, project configuration, and linker/target configuration files instead.

## Current integration status

The ESP32 SoftAP and web firmware updater are implemented. MSP430 I²C polling, K-Line/KWP1281, BLE, CAN, and display tasks remain planned until the new board interfaces are selected and wired. See the ESP32 project's `ARCHITECTURE.md` for the data-flow proposal and porting notes.

## Third-party attribution

The earlier ESP8266 project's K-Line/KWP1281 integration uses and adapts the `KLineKWP1281Lib` API and its measurement-group examples. Attribution, scope, and license notes are in [`ATTRIBUTION.md`](ATTRIBUTION.md). The library itself is an external Arduino dependency and is not bundled in this repository.
