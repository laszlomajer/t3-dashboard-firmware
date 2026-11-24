# Third-party attribution

## K-Line / KWP1281 support in the legacy ESP8266 project

The legacy sketch at `Korábbi ESP8266/T3_Speed_HW_V3_SW_V2_20250810_2/` uses the external Arduino library **KLineKWP1281Lib** by **domnulvlad**:

- Upstream repository: <https://github.com/domnulvlad/KLineKWP1281Lib>
- Upstream documentation: <https://domnulvlad.github.io/KLineKWP1281Lib/>
- Upstream license: GNU General Public License v3.0 (GPL-3.0), as identified by the upstream repository.

The K-Line integration and measurement-group reading flow in the legacy sketch were adapted from that project's API and full-measurement examples. The project-specific ECU selection, 9600-baud configuration, callback glue, coolant-temperature use, display behavior, and MSP430 bridge logic are adaptations for this T3 instrument project. The upstream library source is **not** copied into this repository and remains a separately installed dependency. The precise upstream release used by the original sketch has not been identified.

### Redistribution status

The repository's project-authored source is released under GPL-3.0-only, and the full license text is included in the root `LICENSE` file. The K-Line comments identify the upstream source and record the 2026-11-24 attribution/license update. The legacy sketch was already dated 2025-08-10 in its build identifier. The exact upstream dependency release used by that original build is unknown, so the dependency should be installed from its upstream project and its version recorded when the legacy sketch is next built.

The K-Line library is an external dependency, not vendored source. When distributing a firmware image built by linking it with the GPL library, provide the corresponding source and retain the upstream notices and GPL terms. The user's intention not to earn money does not make GPL-3.0 noncommercial: GPL recipients may use or redistribute the software commercially, subject to the license terms.

Relevant local files:

- `Korábbi ESP8266/T3_Speed_HW_V3_SW_V2_20250810_2/T3_Speed_HW_V3_SW_V2_20250810_2.ino` — KWP1281 setup and measurement-group reads.
- `Korábbi ESP8266/T3_Speed_HW_V3_SW_V2_20250810_2/communication.h` — serial callbacks passed to the library.
- `Korábbi ESP8266/T3_Speed_HW_V3_SW_V2_20250810_2/configuration.h` — serial-port and connection configuration.

The ESP-IDF ESP32 firmware currently does not contain or port this K-Line implementation; K-Line/KWP1281 is still a planned module there.

## Texas Instruments SBW programmer source

The ESP32 project includes `SBW430.c` and `SBW430.h`, which carry Texas Instruments' copyright and redistribution terms in their source headers. The MSP430 linker command file also retains its Texas Instruments redistribution notice. Those files retain their own TI terms; the root GPL notice does not remove or replace those notices. TI's terms are a permissive three-clause BSD-style notice, a license family listed as GPL-compatible by the FSF. Preserve the TI notices in source and binary distributions.
