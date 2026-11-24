# Development setup

This repository keeps the firmware source and the IDE project metadata for the T3 instrument electronics. PCB design files are deliberately kept local and must not be added to the public repository.

## ESP32 — VS Code and ESP-IDF

Open `T3-Dash-V4/firmware/esp32_msp430_webbsl/` as the ESP-IDF project in Visual Studio Code. Use the Espressif ESP-IDF extension and the ESP-IDF 6.x toolchain installed on the development computer. Build, flash, and monitor through the extension or its ESP-IDF terminal commands.

The `.vscode/settings.json` file contains a machine-specific ESP-IDF path and is local-only. Configure ESP-IDF in the extension on each computer; do not commit local paths, `sdkconfig`, build output, or `main/wifi_ap_config.h`. Copy `main/wifi_ap_config.example.h` to `main/wifi_ap_config.h` for local Wi-Fi configuration.

## MSP430 — Code Composer Studio

Import `MSP430/MSP_Speed_HW_V3_SIMPLE_20261113/` into Code Composer Studio as an existing CCS project. The project metadata, source files, linker command file, and target configuration are tracked. Generated `Debug/` output is local and ignored.

## Public repository contents

- ESP32 ESP-IDF / FreeRTOS firmware source
- MSP430G2553 CCS source and project metadata
- Earlier ESP8266 source retained as a porting reference
- Documentation, licensing, and attribution

KiCad files (`.kicad_sch`, `.kicad_pcb`, `.kicad_pro`), PCB libraries, manufacturing exports (Gerber/drill), and other hardware design documents are excluded. Before committing, check `git status` and confirm no PCB or manufacturing files are staged.

## Git workflow

Make changes on `main` for small local updates, then review the staged file list before committing. For larger features, create a short-lived feature branch. Never stage generated output, machine-specific IDE settings, credentials, firmware binaries containing secrets, or PCB design files.
