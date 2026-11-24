# V4 firmware review and architecture proposal

## What the supplied programs do

### Current MSP430G2553 firmware

The latest CCS project is `MSP430/MSP_Speed_HW_V3_SIMPLE_20261113`. The MSP430 remains the real-time instrument controller: it reads five ADC inputs, converts them through lookup tables, drives coolant/fuel gauge PWM and warning outputs, captures the speed input with Timer1_A, and sleeps between measurement cycles using MSP430 low-power mode. It is bare-metal interrupt-driven firmware; FreeRTOS belongs on the ESP32, not on this G2553 build.

The MSP430 is already an I²C slave at address `0x42`. It returns a 10-byte frame: `0x5A`, eight data bytes, then XOR of the preceding nine bytes. The current byte map is DC bus voltage, dimmer percentage, coolant level ADC, coolant temperature, fuel percentage, reserved, speed period, and status bits. The MSP also receives a four-byte I²C command and currently consumes only the fourth byte as `in_status`; the first three command bytes are not yet applied to outputs.

### Supplied ESP8266 firmware

The older project is `Korábbi ESP8266/T3_Speed_HW_V3_SW_V2_20250810_2`. It combines four responsibilities in one Arduino `loop()`: read the MSP430 frame over I²C, optionally query the engine ECU using KWP1281 over K-Line, send the ECU coolant value and status byte back to the MSP430, and update two SSD1306 displays plus PWM outputs. The MSP I²C read expects the same `0x5A + 8 data + XOR` frame, so that part is logically compatible with the current MSP firmware.

The K-Line measurement path is disabled in the supplied build because `K_ON` is commented out in the `.ino`. When enabled, it connects at 9600 baud and reads KWP1281 group 1, measurement 3 for coolant temperature. The automatic path then maps 0–120 °C to 0–255 and sends it as the first byte of a four-byte MSP I²C write. The current MSP firmware does not use that first byte, so the old external ECU coolant-to-gauge behavior is not ported yet. The old sketch also makes KWP read calls from display-page code; in the new design those reads should be centralized in one ECU polling task instead of being triggered by drawing a page.

## Recommended responsibility split

Keep the MSP430 responsible for deterministic, time-sensitive work: ADC sampling, pulse capture, warning decisions, gauge PWM, and the existing I²C slave protocol. Give the ESP32 network, presentation, and optional bridge duties. This avoids moving timing-sensitive instrument behavior into Wi-Fi/BLE code.

```text
MSP430G2553
  ADC / timers / warning outputs / gauge PWM
  I2C slave telemetry and commands
          │ I2C, versioned frame
          ▼
ESP32 FreeRTOS
  MSP430 I2C poll task ───────┐
  ECU K-Line/KWP task ────────┤
  telemetry model / queues ───┼── BLE GATT notifications
  display refresh task ───────┤── SPI or I2C display(s)
  CAN task (future transport) ┤── CAN controller/transceiver
  Wi-Fi AP + HTTP diagnostics ┘
          └── MSP430 SBW and ESP32 OTA maintenance pages
```

Suggested source layout as the implementation grows:

```text
main/
  app_main.c                 startup and task creation only
  model/telemetry.h          typed values, validity, source, timestamp
  drivers/msp430_i2c.*       frame read/write, retries, protocol version
  drivers/kwp1281.*          isolated K-Line UART and ECU polling
  services/telemetry.*       normalize MSP/ECU data and source selection
  transports/ble_gatt.*      BLE services and notifications
  transports/can.*           future CAN frames and health counters
  display/display_task.*     display-neutral pages and refresh scheduling
  web/web_server.*           AP, diagnostics API, MSP update page
  update/msp430_sbw.*        Intel HEX validation and SBW flash worker
  update/esp_ota.*           later ESP32 application OTA
```

Use FreeRTOS queues or task notifications to pass snapshots between tasks. Keep I²C and K-Line transactions out of display, HTTP, and BLE callbacks. Every published value should carry at least a timestamp and validity/error state so the UI can distinguish a real zero from stale or unavailable data. The telemetry model should define one coolant value with an explicit selected source (MSP sensor or ECU KWP) rather than silently letting both writers control one gauge.

For compatibility, preserve the current 10-byte I²C frame while bringing up the new hardware. For the next protocol revision, add a protocol version, sequence counter, explicit command identifier, and CRC-8; the current XOR check is useful for detecting simple errors but is not a robust CRC. Keep BLE and CAN encodings separate from the MSP register/frame layout so changing the transport does not change sensor logic.

## Current implementation scope

The ESP32 now starts a WPA2 SoftAP named `T3-Dashboard-V4` and serves the UI at `http://192.168.4.1/`. The page has a diagnostics tab and an MSP430 Intel HEX/SBW update tab. Diagnostics currently report AP clients, uptime, free/minimum heap, SBW update state, and explicitly mark I²C, BLE, and CAN as not configured. This matches the unfinished hardware link decision; no sensor polling or BLE/CAN data forwarding is claimed yet.

The MSP430 upload page accepts Intel HEX addresses only in main flash `0xC000–0xFFFF`. It erases/reprograms the whole 16 KiB main flash and verifies it. Protect access to this page by keeping the AP password private and changing the sample value before production use. This AP is for local service and does not route to the internet.

## Hardware decisions to settle before telemetry porting

1. Choose and route the ESP32–MSP430 data link. The existing MSP I²C slave makes I²C the least-change first prototype; check target voltage, pull-ups, bus length, and whether displays share that bus. UART is also viable if the MSP430 frame protocol is changed.
2. Decide whether coolant gauge control comes from the MSP430's local sensor, the ECU KWP reading, or a configurable source selection. Today the MSP430 uses its local ADC sensor, while the old bridge write's first byte is ignored.
3. Select the ESP32 UART and suitable automotive K-Line transceiver/protection before enabling KWP1281. ESP8266 UART pin assignments and Arduino libraries do not directly map to ESP-IDF.
4. Pick the BLE GATT schema, CAN controller/transceiver, and display buses after hardware selection. Implement each behind the telemetry model rather than coupling it into `app_main`.
5. Verify SBW wiring and reset-line loading on the finished board. A web upload erases the MSP430's main flash and must not be treated as a harmless data write.
