# T3 V4 – ESP32 → MSP430G2553 webes SBW programozó

ESP-IDF projekt FreeRTOS-szal. Az ESP32 WPA2-es Wi-Fi hozzáférési pontot indít `T3-Dashboard-V4` SSID-vel. Csatlakozás után nyisd meg a `http://192.168.4.1/` címet. A weboldalon diagnosztikai és MSP430 firmware-frissítési fül található.

## Feltöltés CCS-ből

Az ESP32 **Intel HEX (`.hex`)** fájlt fogad. A CCS szokásos `.out` fájlja ELF programfájl, a nyers BIN pedig nem tartalmaz célcímet, ezért azokat így közvetlenül nem lehet feltölteni. CCS-ben kapcsold be az MSP430 Hex Utility-t, és válaszd az Intel HEX kimenetet; a TI dokumentáció a TI-TXT és Intel HEX kimenet beállítását is leírja ([TI BSL Scripter útmutató](https://www.ti.com/lit/ug/slau655g/slau655g.pdf)).

A weboldal csak a G2553 fő flash-tartományára (`0xC000–0xFFFF`) eső adatot fogad el. A feltöltés SBW-n keresztül törli és újraprogramozza a teljes 16 KiB fő flash területet, majd visszaellenőrzést végez. Az információs memóriát nem érinti.

## SBW bekötés

Az aktuális alapértékek klasszikus ESP32 DevKithez:

| ESP32 | MSP430G2553 TSSOP-28 | SBW funkció |
|---|---:|---|
| GPIO18 | 25 – TEST | SBWTCK órajel |
| GPIO19 | 24 – RST | SBWTDIO kétirányú adat |
| GND | 28 – DVSS | közös föld |
| 3V3 | 1 – DVCC és AVCC | cél táp, csak közös tápterv esetén |

Az ESP32 GPIO-k alapértékei a `main/sbw/sbw_config.h` fájlban módosíthatók. A V4 kapcsolási rajzban az MSP430 `U14`; a GPIO-kiosztás a tényleges ESP32 boardhoz igazítandó. A cél MSP430 és ESP32 közös földet igényel, és a célfeszültség nem haladhatja meg egyik chip megengedett tápfeszültségét sem. Az SBW `RST/SBWTDIO` vonal kétirányú; a panelen lévő resetáramkört és felhúzást ellenőrizni kell csatlakoztatás előtt.

## Hálózati beállítás

Az alapértelmezett hozzáférési pont neve `T3-Dashboard-V4`, címe `192.168.4.1`. Az SSID és jelszó a helyi, verziókezelésből kizárt `main/wifi_ap_config.h` fájlban van. Új checkoutnál másold a `main/wifi_ap_config.example.h` fájlt `wifi_ap_config.h` néven, majd adj meg saját, legalább 8 karakteres WPA2-jelszót. Ez a hozzáférési pont internet-hozzáférést nem ad.

## TI-alapú SBW kód

A `main/sbw/SBW430.c` és `SBW430.h` a TI SLAA754 nyílt forrású mintájából származik; az eredeti TI licenc és szerzői jogi fejlécet a forrásfájlok megőrzik. A platform GPIO- és késleltetési réteg ESP-IDF-hez készült. A TI dokumentáció az SBW órajel/data időszeleteket, a JTAG elérést és a flash műveleteket írja le: [SLAU320](https://www.ti.com/lit/ug/slau320aj/slau320aj.pdf), [SLAA754](https://www.ti.com/lit/an/slaa754/slaa754.pdf).

## Megjegyzés

Az ESP32 firmware lefordítható, és az ESP32-n futtatható. Az MSP430 SBW programozást valódi MSP430 célpanelen még hitelesíteni kell. A feltöltés fő flash-t töröl, ezért a hibás vagy nem megfelelő firmware átmenetileg működésképtelenné teheti a műszert. A hardveres I²C/BLE/CAN adatút még nincs kiválasztva vagy bekötve.

## Források vizsgálata és javasolt felépítés

Lásd a [javasolt architektúrát és a két firmware összehasonlítását](ARCHITECTURE.md).
