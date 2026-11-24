/*
Lasd modification 2025.08.10.- 2
HW V3

This modified K-Line/KWP1281 integration is released under GNU GPL-3.0-only.
KWP1281 measurement-group flow adapted from KLineKWP1281Lib by domnulvlad.
Attribution/license metadata updated 2026-11-24; see repository ATTRIBUTION.md.
SPDX-License-Identifier: GPL-3.0-only
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// KWP1281 source attribution: K-LineKWP1281Lib by domnulvlad (GPL-3.0).
// This project's K-Line setup and measurement-group read flow are adapted
// from the upstream library API/examples; this project's changes are dated
// 2026-11-24. See repository ATTRIBUTION.md for redistribution status.
#include <KLineKWP1281Lib.h>
#include "configuration.h"
#include "communication.h"
#include "graphics.h"

#define BUILD "20250810_2 v3"        // Build Date

// i2C címek
#define MSP_ADDR 0x42             // MSP430 i2C címe
#define SCREEN_1_ADDRESS 0x3C   // Az SSD1306 1. kijelző i2c címe
#define SCREEN_2_ADDRESS 0x3D   // Az SSD1306 2. kijelző i2c címe

// Üzemmód makrók

// K-Line/KWP1281 ECU reading (adapted from KLineKWP1281Lib by domnulvlad).
// Upstream source and license: repository ATTRIBUTION.md.
//#define K_ON                      // K-Line olvasás bekapcsolva

// Debug üzemmód  - Serial porton
//#define SER_DEBUG               // Nem műküdik együtt K-line-al!!!!
#define DISP2                   // Másidik kijelző


// Üzemmód makrók vége


// Debugging can be enabled in "configuration.h" in order to print connection-related info on the Serial Monitor.
#if debug_info
  KLineKWP1281Lib diag(beginFunction, endFunction, sendFunction, receiveFunction, TX_pin, is_full_duplex, &Serial);
#else
  KLineKWP1281Lib diag(beginFunction, endFunction, sendFunction, receiveFunction, TX_pin, is_full_duplex);
#endif

// Please find more information on the topic of KWP1281 measurement groups in the "03.Full_measurement_test" example.
uint8_t measurement_buffer[80];
uint8_t measurement_body_buffer[4];

// This flag keeps track if a [Header] was received for the current group, meaning it's of the "header+body" type.
bool received_group_header = false;

// This will contain the amount of measurements received in the [Header] of a "header+body" group.
uint8_t amount_of_measurements_in_header = 0;

// K-Line reading változók
bool readingError = false;      // Olvasási hiba történt (Simple Read)

// Mérési stuktúra ECU olvasáshoz

struct Measurement {
  float value;
  char unit[16];
};

Measurement boost, coolantTemp, supplyVoltage, fuelTemp, consumption, injectedQty, intakeAirTemp, maf ;

// MSP430G2553 i2C olvasás

uint8_t received_data[10];        // MSP430 Olvasás buffer

typedef union {
    struct {
        unsigned char f1:1;
        unsigned char f22:1;
        unsigned char b2:1;
        unsigned char coolant_temp_high:1;
        unsigned char coolant_level_low:1;
        unsigned char fuel_low:1;
        unsigned char lt_on:1;
        unsigned char oilwarn:1;
    };
    uint8_t all;
} bitfield;

bitfield status     = { .all = 0 };          // Azonos státusz változó mint ami az MSP-ben van.
bitfield out_status = { .all = 0 };  

// OLED kijelző beállítások
#define SCREEN_1_WIDTH 128
#define SCREEN_1_HEIGHT 64
#define SCREEN_2_WIDTH 128
#define SCREEN_2_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_1_WIDTH, SCREEN_1_HEIGHT, &Wire, OLED_RESET);       // Display 1.

#ifdef DISP2
Adafruit_SSD1306 display2(SCREEN_2_WIDTH, SCREEN_2_HEIGHT, &Wire, OLED_RESET);       // Display 2.

const unsigned char* icons[6] = {
  tempIcon, coolantIcon, battIcon, turboIcon, fuelIcon, oilIcon
};
const char* pageLabels[6] = {
  "TEMP", "C-TEMP", "BATT", "BOOST", "FUEL", "OIL"
};
const char* units[6] = {
  "C", "C", "V", "bar", "", ""
};
float pageValues[6] = {
  0, 0, 0, 0, 0, 0
};

const char* statusText = "NORMAL";  // Dinamikus szöveg a 6. oldalhoz
int progressValue = 65;             // Progress bar érték (5. oldalhoz)

unsigned char pageIndex = 0;

#endif

// GPIO definíciók
#define SCL_PIN     2             // D3 - i2C 
#define SDA_PIN     0             // D4 - i2C

#define TX_PIN      1             // TX - K-Line és Programozás
#define RX_PIN      3             // RX - K-Line és Programozás

#define PWM1_PIN    5             // D1 - Másoldlagos PWM
#define PWM2_PIN    12            // D6 - Háttérvilágítás

#define BUTT_2_PIN  14            // D5 - BUtton 2
#define BUTT_1_PIN  13            // D7 - BUtton 1

#define MSP_RES     4             // MSP430G2553 Reset PIN


// Nyomógomb megszakítások kezelése

void IRAM_ATTR handleButton1() {                     // Gomb 1 kezelése
  changeDisplay_inc();
}

void IRAM_ATTR handleButton2() {                     // Gomb 2 kezelése
  changeDisplay2_inc();
}

// PWM beállítások
int pwm1Value = 0;                                       // Tartalék PWM
int pwm2Value = 0;                                       // Háttérvilágítás

// Grafikai elemek

uint8_t currentPage_disp  = 0;                          // Display 1 page
#ifdef DISP2
uint8_t currentPage_d2 = 0;                             // Display 2 page
#endif
void drawProgressBar(int percentage) {
//  display.clearDisplay(); // Képernyő törlése - közös kezelésnél nem itt kell alkalmazni
  
  int barWidth = map(percentage, 0, 100, 0, 128); // 100% = 100 pixel szélesség
  
  display.drawRect(0, 54, 128, 10, WHITE); // Külső keret
  display.fillRect(0, 54, barWidth, 10, WHITE); // Kitöltött rész
}

void drawCircles(bool states[10]) {
    for (int i = 0; i < 10; i++) {
        int x = i * 12 + 6; // Körök közötti távolság
        int y = 46;
        if (states[i]) {
            display.fillCircle(x, y, 3, SSD1306_WHITE);
        } else {
            display.drawCircle(x, y, 3, SSD1306_WHITE);
        }
    }
}

void init_display1(void){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_1_ADDRESS)) {

  #ifdef SER_DEBUG
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  #endif
  }
  delay(500);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(F("   VW T3   "));
  display.setCursor(30,20);
  display.setTextSize(1);
  display.print(F(BUILD));
  display.setCursor(0,40);
  display.print(F("    Laszlo Majer"));
  display.setCursor(0,50);
  display.print(F("  yseusi@gmail.com "));
  display.display();
  delay(2000);
}
#ifdef DISP2
void init_display2(void){
  if(!display2.begin(SSD1306_SWITCHCAPVCC, SCREEN_2_ADDRESS)) {

  #ifdef SER_DEBUG
    Serial.println(F("SSD1306 2 allocation failed"));
    for(;;);
  #endif
  }
  delay(500);
  display2.clearDisplay();
  display2.setTextSize(2);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(0,32);
  display2.print(F("VW T3 1985"));
  display2.display();
}
#endif

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);     // i2c inicializálás
  delay(200);
  inits();                          // Egyéb üzemmód inicializálások

  pwm2Value = 50;                   // Inicializált háttérvilágítás pwm
  pwm1Value = 30;                   // Külső műszer háttéevilágítás
  
  resetMSP();                       // MSP indítás

//  KWP1281 INIT

  // If debugging bus traffic was enabled, attach the debugging function.
  #if debug_traffic
    diag.KWP1281debugFunction(KWP1281debugFunction);
  #endif

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(4,0);
  display.print(F("Connecting"));
  display.setCursor(4,20);
  display.print(F("    to    "));
  display.setCursor(0,40);
  display.print(F("    ECU    "));
  display.display();
  delay(1000);

  #ifdef K_ON
  // Connect to the module.
  diag.connect(connect_to_module, module_baud_rate, true);
  #endif

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,20);
  display.print(F("    ECU    "));
  display.setCursor(8,40);
  display.print(F("Connected!"));
  display.display();
  delay(1000);

}

void loop() {
  readMSP();           
  analogWrite(PWM1_PIN, pwm1Value);   // Tartalék
  analogWrite(PWM2_PIN, pwm2Value);   // Háttérvilágítás

  // Cooland Temp Read

#ifdef K_ON

  simpleRead(1, 3, &coolantTemp.value, coolantTemp.unit, sizeof(coolantTemp.unit));     
  check_coolant();
  writeMSP(map((uint8_t)coolantTemp.value, 0, 120, 0, 255),0,0,out_status.all);       // 4 kimeneti paraméter van: 1-Coolant temp from ECU , Semmi, Semmi , 4 - out_status.all 


#endif

  backlight_change();

  //Kijelző kezelése

  switch (currentPage_disp) {
    case 0:
      drawPage0();
      break;
    case 1:
      drawPage1();
      break;
    case 2:
      drawPage2();
      break;
    case 3:
      drawPage3();
      break;
    case 4:
      drawPage4();
      break;
    case 5:
      drawPage5();
      break;
    case 6:
      drawPage6();
      break;
    case 7:
      drawPage7();
      break;
    case 8:
      drawPage8();
      break;
    case 9:
      drawPage9();
      break;

    default:
      currentPage_disp = 0;
      break;

  }
  delay(1000);

  #ifdef DISP2

  switch (currentPage_d2) {

    case 0:
      drawPage0_D2();
      break;

    case 1:
      drawPage1_D2();
      break;

    case 2:
      drawPage2_D2();
      break;

    case 3:
      drawPage3_D2();
      break;

    case 4:
      drawPage4_D2();
      break;

    case 5:
      drawPage5_D2();
      break;

    default:
      currentPage_d2 = 0;
      break;
    }
  #endif

}

void check_coolant(void)
{
  if(coolantTemp.value > 85.0)  out_status.coolant_temp_high =  1;
  else out_status.coolant_temp_high = 0;
}

void backlight_change(void)
{
  if(status.lt_on)
  {
    pwm1Value = 10;
    pwm2Value = 20;
  }
  else
  {
    pwm1Value = 50;
    pwm2Value = 75;
  }
}

//Display előre léptetés

void changeDisplay_inc() {
  currentPage_disp = (currentPage_disp + 1) % 10; //  oldal között váltogatunk
}

void changeDisplay_dec() {
  currentPage_disp = (currentPage_disp + 9) % 10; // ugyanaz, mintha -1 lenne, de mindig pozitív
}
// Display vissza léptetés

#ifdef DISP2

void changeDisplay2_inc() {
  currentPage_d2 = (currentPage_d2 + 1) % 10; //  oldal között váltogatunk
}

void changeDisplay2_dec() {
  currentPage_d2 = (currentPage_d2 + 9) % 10; // ugyanaz, mintha -1 lenne, de mindig pozitív
}

#endif

void resetMSP(void){
  pinMode(MSP_RES, OUTPUT);   // GPIO4 mint kimenet
  digitalWrite(MSP_RES, HIGH); // MSP reset
  delay(100);
  digitalWrite(MSP_RES, LOW); // MSP reset
}

void writeMSP(uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t data3)
{
  Wire.beginTransmission(MSP_ADDR);  // MSP430 cím
  Wire.write(cmd);                   // Küldendő adat (1 byte)
  Wire.write(data1);
  Wire.write(data2);
  Wire.write(data3);
  Wire.endTransmission();            // Átvitelt lezárjuk
}

void readMSP(void){
  Wire.requestFrom(MSP_ADDR, 10);

  int i = 0;
  while (Wire.available() && i < 10) {
    received_data[i++] = Wire.read();
  }

  if (i == 10 && received_data[0] == 0x5A) {
    uint8_t crc = 0;
    for (int j = 0; j < 9; j++) crc ^= received_data[j];

if (crc == received_data[9]) {

    status.all = received_data[8];

}
#ifdef SER_DEBUG
    if (crc == received_data[9]) {
      Serial.print("OK: ");
      for (int j = 1; j <= 8; j++) {
        Serial.print(received_data[j]); Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("CRC mismatch!");
    }
  } else {
    Serial.println("Invalid response!");
#endif
  }
}

// Általános inicializációk

void inits(void){
  #ifdef SER_DEBUG
  Serial.begin(115200);
  #endif

  init_display1();                  // SSD1306 init

  #ifdef DISP2
  init_display2();        // Második kijelző esetén...    
  #endif



  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(BUTT_1_PIN, INPUT_PULLUP);
  pinMode(BUTT_2_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTT_1_PIN), handleButton1, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTT_2_PIN), handleButton2, FALLING);
  
  analogWriteFreq(25000);  // PWM frekvencia 25 kHz-re állítása
  analogWriteRange(255);  // 8 bites felbontás (0-255)

}

// Kijelző 1 oldalak kezelése:

void drawPage0() {            // Eredeti kijelzés
display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("FUEL:");                                // FUEL LEVEL?
  display.print(received_data[1]);
  display.setCursor(0,10);
  display.print("CT:");                                //
  display.print(received_data[2]);
  display.print("");
  display.setCursor(0,20);
  display.print("CL:");
  display.print(received_data[3]);
  display.setCursor(0,30);
  display.print("STS:");                                // Status BIT
  display.print(received_data[8]);
  display.drawBitmap(96, 0, vWlogo, 32, 32, WHITE);

   bool states[10] = {LOW,  LOW, status.f1,  status.f22, status.b2,  status.coolant_temp_high, status.coolant_level_low, status.fuel_low, status.lt_on , status.oilwarn};
  drawCircles(states);

  display.display();
}

void drawPage1(){       // RPM reszt oldal

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("   SPEED   ");
  displayMeasurement(6,0);

}

void drawPage2(){       // BOOST test oldal
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("   BOOST   ");
  displayMeasurement(10,2);
}

void drawPage3(){       // Coolant test oldal
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("  Coolant  ");
  displayMeasurement(1,3);
}

void drawPage4(){       // DC BUS teszt oldal
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("  DC BUS  ");
  displayMeasurement(12,2);
}

void drawPage5(){       // Üzemynag hőmérséklet
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(" FUEL TEMP ");
  displayMeasurement(7,0);
}

void drawPage6(){       // Fogyasztás
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(" CONSUMPT. ");
  displayMeasurement(15,2);
}

void drawPage7(){       // Befecskendezés
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("Inject QTY");
  displayMeasurement(15,1);
}

void drawPage8(){       // Levegő hőmérsékéet
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("Intake Air");
  displayMeasurement(7,2);
}

void drawPage9(){       // simple read teszt oldal
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Page 10 Coolant Temp");
  simpleRead(1,3,&coolantTemp.value,coolantTemp.unit, sizeof(coolantTemp.unit));
  simpleRead(10,0,&maf.value,maf.unit, sizeof(maf.unit));
  display.setCursor(0,20);
  display.print(coolantTemp.value);
  display.print(" ");
  display.print(coolantTemp.unit);
  display.setCursor(0,30);
  display.print(maf.value);
  display.print(" ");
  display.print(maf.unit);
  display.display();

}

// Kijelzó 2. Oldalak kezelése

// Intake Air

void drawPage0_D2() {
  pageIndex = 0;
  display2.clearDisplay();
  // Ikon bal felső sarokban
  display2.drawBitmap(0, 0, icons[pageIndex], 32, 32, SSD1306_WHITE);
  // Címsor nagy betűkkel
  display2.setTextColor(SSD1306_WHITE);
  display2.setTextSize(2);
  display2.setCursor(36, 8);
  display2.print(pageLabels[pageIndex]);

  // Mértékegység kisebb betűmérettel a cím után
  int16_t x1, y1;
  uint16_t w, h;
  display2.getTextBounds(pageLabels[pageIndex], 36, 8, &x1, &y1, &w, &h);
  display2.setTextSize(1);
  display2.setCursor(36 + w + 4, 12);
  display2.print(units[pageIndex]);

  display2.setTextSize(3);
  display2.setCursor(0, 36);
  
  simpleRead(7, 2, &intakeAirTemp.value, intakeAirTemp.unit, sizeof(intakeAirTemp.unit));

  display2.print(intakeAirTemp.value,1);

  display2.display();

}

// Coolant temp value

void drawPage1_D2() {
  pageIndex = 1;
  display2.clearDisplay();
  // Ikon bal felső sarokban
  display2.drawBitmap(0, 0, icons[pageIndex], 32, 32, SSD1306_WHITE);
  // Címsor nagy betűkkel
  display2.setTextColor(SSD1306_WHITE);
  display2.setTextSize(2);
  display2.setCursor(36, 8);
  display2.print(pageLabels[pageIndex]);

  // Mértékegység kisebb betűmérettel a cím után
  int16_t x1, y1;
  uint16_t w, h;
  display2.getTextBounds(pageLabels[pageIndex], 36, 8, &x1, &y1, &w, &h);
  display2.setTextSize(1);
  display2.setCursor(36 + w + 4, 12);
  display2.print(units[pageIndex]);

  display2.setTextSize(3);
  display2.setCursor(0, 36);
  display2.print(coolantTemp.value,1);
  display2.display();
}

// DC BUS Voltage

void drawPage2_D2() {
  pageIndex = 2;
  display2.clearDisplay();
  // Ikon bal felső sarokban
  display2.drawBitmap(0, 0, icons[pageIndex], 32, 32, SSD1306_WHITE);
  // Címsor nagy betűkkel
  display2.setTextColor(SSD1306_WHITE);
  display2.setTextSize(2);
  display2.setCursor(36, 8);
  display2.print(pageLabels[pageIndex]);

  // Mértékegység kisebb betűmérettel a cím után
  int16_t x1, y1;
  uint16_t w, h;
  display2.getTextBounds(pageLabels[pageIndex], 36, 8, &x1, &y1, &w, &h);
  display2.setTextSize(1);
  display2.setCursor(36 + w + 4, 12);
  display2.print(units[pageIndex]);

  display2.setTextSize(3);
  display2.setCursor(0, 36);

  simpleRead(12, 2, &supplyVoltage.value, supplyVoltage.unit, sizeof(supplyVoltage.unit));

  display2.print(supplyVoltage.value,1);
  display2.display();
}

// Boost 

void drawPage3_D2() {
  pageIndex = 3;
  display2.clearDisplay();
  // Ikon bal felső sarokban
  display2.drawBitmap(0, 0, icons[pageIndex], 32, 32, SSD1306_WHITE);
  // Címsor nagy betűkkel
  display2.setTextColor(SSD1306_WHITE);
  display2.setTextSize(2);
  display2.setCursor(36, 8);
  display2.print(pageLabels[pageIndex]);

  // Mértékegység kisebb betűmérettel a cím után
  int16_t x1, y1;
  uint16_t w, h;
  display2.getTextBounds(pageLabels[pageIndex], 36, 8, &x1, &y1, &w, &h);
  display2.setTextSize(1);
  display2.setCursor(36 + w + 4, 12);
  display2.print(units[pageIndex]);

  display2.setTextSize(3);
  display2.setCursor(0, 36);
  
  simpleRead(10, 2, &boost.value, boost.unit, sizeof(boost.unit));

  display2.print(boost.value,1);

  display2.display();
}

void drawPage4_D2() {
  pageIndex = 4;
    display2.clearDisplay();
  // Ikon bal felső sarokban
  display2.drawBitmap(0, 0, icons[pageIndex], 32, 32, SSD1306_WHITE);
  // Címsor nagy betűkkel
  display2.setTextColor(SSD1306_WHITE);
  display2.setTextSize(2);
  display2.setCursor(36, 8);
  display2.print(pageLabels[pageIndex]);

  // Mértékegység kisebb betűmérettel a cím után
  int16_t x1, y1;
  uint16_t w, h;
  display2.getTextBounds(pageLabels[pageIndex], 36, 8, &x1, &y1, &w, &h);
  display2.setTextSize(1);
  display2.setCursor(36 + w + 4, 12);
  display2.print(units[pageIndex]);

  display2.setTextSize(2);
  display2.setCursor(0, 36);

if(status.fuel_low)  display2.print("   LOW   ");
else display2.print("    OK    ");
  display2.display();
}


void drawPage5_D2() {
  pageIndex = 5;
  display2.clearDisplay();
  // Ikon bal felső sarokban
  display2.drawBitmap(0, 0, icons[pageIndex], 32, 32, SSD1306_WHITE);
  // Címsor nagy betűkkel
  display2.setTextColor(SSD1306_WHITE);
  display2.setTextSize(2);
  display2.setCursor(36, 8);
  display2.print(pageLabels[pageIndex]);

  // Mértékegység kisebb betűmérettel a cím után
  int16_t x1, y1;
  uint16_t w, h;
  display2.getTextBounds(pageLabels[pageIndex], 36, 8, &x1, &y1, &w, &h);
  display2.setTextSize(1);
  display2.setCursor(36 + w + 4, 12);
  display2.print(units[pageIndex]);

  display2.setTextSize(2);
  display2.setCursor(0, 36);

if(status.oilwarn)  display2.print("   WARN   ");
else display2.print("  NORMAL  ");
  display2.display();
}


// Jelenleg nem használt egyszerűsített olvasás
/*
void updateMeasurements() {
  simpleRead(10, 2, &boost.value, boost.unit, sizeof(boost.unit));
  simpleRead(1, 3, &coolantTemp.value, coolantTemp.unit, sizeof(coolantTemp.unit));
  simpleRead(12, 2, &supplyVoltage.value, supplyVoltage.unit, sizeof(supplyVoltage.unit));
  simpleRead(7, 0, &fuelTemp.value, fuelTemp.unit, sizeof(fuelTemp.unit));
  simpleRead(15, 2, &consumption.value, consumption.unit, sizeof(consumption.unit));
  simpleRead(15, 1, &injectedQty.value, injectedQty.unit, sizeof(injectedQty.unit));
  simpleRead(7, 2, &intakeAirTemp.value, intakeAirTemp.unit, sizeof(intakeAirTemp.unit));
  simpleRead(10, 0, &maf.value, maf.unit, sizeof(maf.unit));
}
*/


// simple read sintax: simpleRead(3, 0, &rpm.value, rpm.unit, sizeof(rpm.unit));

void simpleRead(uint8_t group,  uint8_t measurement_index,  float* out_value,  char* out_unit,  size_t unit_size)
{
 uint8_t amount_of_measurements = 0;
 readingError = false;
  KLineKWP1281Lib::executionStatus readGroup_status;
  if (!received_group_header)
  {
    readGroup_status = diag.readGroup(amount_of_measurements, group, measurement_buffer, sizeof(measurement_buffer));
  }
  else
  {
    readGroup_status = diag.readGroup(amount_of_measurements, group, measurement_body_buffer, sizeof(measurement_body_buffer));
  }
  // Check the returned value.
  switch (readGroup_status)
  {
    case KLineKWP1281Lib::ERROR:
      {
        readingError = true;    // Egyszerűsítés, hogy a globális readingError változóban jelezzük, hogy kommunikációs hiba van.
      }
      // There is no reason to continue, exit the function.
      return;
    
    case KLineKWP1281Lib::FAIL:
      {
        readingError = true;    // Egyszerűsítés, hogy a globális readingError változóban jelezzük, hogy kommunikációs hiba van.
      }
      // There is no reason to continue, exit the function.
      return;

    case KLineKWP1281Lib::GROUP_BASIC_SETTINGS:
      {
        // If we have a [Header], it means this group sends responses of "header+body" type.
        // So, at this point, it doesn't make sense to receive something other than a [Body].
        if (received_group_header)
        {
        readingError = true;    // Egyszerűsítés, hogy a globális readingError változóban jelezzük, hogy kommunikációs hiba van.
          return;
        }
      }

      return;

    case KLineKWP1281Lib::GROUP_HEADER:
      {
        if (received_group_header)
        {
          readingError = true;    // Egyszerűsítés, hogy a globális readingError változóban jelezzük, hogy kommunikációs hiba van.
          return;
        }

        // Set the flag to indicate that a header was received, making this a "header+body" group response.
        received_group_header = true;

        // Store the number of measurements received in the header.
        amount_of_measurements_in_header = amount_of_measurements;
      }
      // We have nothing to display yet, the next readGroup() will get the actual data; exit the function.
      return;

    case KLineKWP1281Lib::GROUP_BODY:
      {
        // If we don't have a [Header], it doesn't make sense to receive a [Body].
        if (!received_group_header)
        {
          readingError = true;    // Egyszerűsítés, hogy a globális readingError változóban jelezzük, hogy kommunikációs hiba van.
          return;
        }
      }
      // If we have the [Header], now we also have the [Body]; execute the code after the switch().
      break;
    
    // Execute the code after the switch().
    case KLineKWP1281Lib::SUCCESS:
      break;
  }
   
  // Get the selected measurement's type.
  KLineKWP1281Lib::measurementType measurement_type;
  if (!received_group_header)
  {
    measurement_type = KLineKWP1281Lib::getMeasurementType(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer));
  }
  // For "header+body" measurements, you need to use this other function that specifically parses headers instead of regular responses.
  else
  {
    measurement_type = KLineKWP1281Lib::getMeasurementTypeFromHeader(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer));
  }
  
  // Check the returned value.
  switch (measurement_type)
  {
    // "Value and units" type
    case KLineKWP1281Lib::VALUE:
    {
      // The measurement's units will be copied into this array, so they can be displayed.
      char units_string[16];
        
      // Regular mode:
      if (!received_group_header)
      {
        *out_value = KLineKWP1281Lib::getMeasurementValue(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer));
        KLineKWP1281Lib::getMeasurementUnits(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer), units_string, sizeof(units_string));
      }
      // "Header+body" mode:
      else
      {
        *out_value = KLineKWP1281Lib::getMeasurementValueFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer));
        KLineKWP1281Lib::getMeasurementUnitsFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer), units_string, sizeof(units_string));
      }

      // Másoljuk át a visszaadott unit stringet a kimeneti bufferbe
      strncpy(out_unit, units_string, unit_size);
      out_unit[unit_size - 1] = '\0'; // biztos ami biztos
    }
    break;
    
    // "Text" type
    case KLineKWP1281Lib::TEXT:
    {
      // Nincs kezelve
    }
    break;
    
    // Invalid measurement index
    case KLineKWP1281Lib::UNKNOWN:
    {
      // Nincs kezelve
    }
      break;
  }
}

void displayMeasurement(uint8_t group, uint8_t measurement_index)
{
  uint8_t amount_of_measurements = 0;
  KLineKWP1281Lib::executionStatus readGroup_status;
  if (!received_group_header)
  {
    readGroup_status = diag.readGroup(amount_of_measurements, group, measurement_buffer, sizeof(measurement_buffer));
  }
  else
  {
    readGroup_status = diag.readGroup(amount_of_measurements, group, measurement_body_buffer, sizeof(measurement_body_buffer));
  }
  // Check the returned value.
  switch (readGroup_status)
  {
    case KLineKWP1281Lib::ERROR:
      {
        display.setTextSize(3);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print("-E-");
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
      // There is no reason to continue, exit the function.
      return;
    
    case KLineKWP1281Lib::FAIL:
      {
        display.setTextSize(3);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print("-E-");
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
      // There is no reason to continue, exit the function.
      return;

    case KLineKWP1281Lib::GROUP_BASIC_SETTINGS:
      {
        // If we have a [Header], it means this group sends responses of "header+body" type.
        // So, at this point, it doesn't make sense to receive something other than a [Body].
        if (received_group_header)
        {
          display.setTextSize(3);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 20);
          display.print("-E-");
          display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
          return;
        }
      }

      return;

    case KLineKWP1281Lib::GROUP_HEADER:
      {
        if (received_group_header)
        {
          display.setTextSize(3);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 20);
          display.print("-E-");
          display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
          return;
        }

        // Set the flag to indicate that a header was received, making this a "header+body" group response.
        received_group_header = true;

        // Store the number of measurements received in the header.
        amount_of_measurements_in_header = amount_of_measurements;
      }
      // We have nothing to display yet, the next readGroup() will get the actual data; exit the function.
      return;

    case KLineKWP1281Lib::GROUP_BODY:
      {
        // If we don't have a [Header], it doesn't make sense to receive a [Body].
        if (!received_group_header)
        {
          display.setTextSize(3);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 20);
          display.print("-E-");
          display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
          return;
        }
      }
      // If we have the [Header], now we also have the [Body]; execute the code after the switch().
      break;
    
    // Execute the code after the switch().
    case KLineKWP1281Lib::SUCCESS:
      break;
  }
  
  /*
    The getMeasurementType() function can return:
      *KLineKWP1281Lib::UNKNOWN - index out of range (measurement doesn't exist in group)
      *KLineKWP1281Lib::VALUE   - regular measurement, with a value and units
      *KLineKWP1281Lib::TEXT    - text measurement
  */
  
  // Get the selected measurement's type.
  KLineKWP1281Lib::measurementType measurement_type;
  if (!received_group_header)
  {
    measurement_type = KLineKWP1281Lib::getMeasurementType(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer));
  }
  // For "header+body" measurements, you need to use this other function that specifically parses headers instead of regular responses.
  else
  {
    measurement_type = KLineKWP1281Lib::getMeasurementTypeFromHeader(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer));
  }
  
  // Check the returned value.
  switch (measurement_type)
  {
    // "Value and units" type
    case KLineKWP1281Lib::VALUE:
    {
      // The measurement's units will be copied into this array, so they can be displayed.
      char units_string[16];
        
      // Regular mode:
      if (!received_group_header)
      {
        // Determine how many decimal places are best suited to this measurement.
        uint8_t decimals = KLineKWP1281Lib::getMeasurementDecimals(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer));
        
        // Display the calculated value, with the recommended amount of decimals.
        
        
        display.setTextSize(3);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print(KLineKWP1281Lib::getMeasurementValue(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer)), decimals);
        display.setCursor(0, 45);
        display.setTextSize(2);
        display.print(KLineKWP1281Lib::getMeasurementUnits(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer), units_string, sizeof(units_string)));
        display.setTextSize(1);
        display.setCursor(80, 50);
        display.print("REG");
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
        
      }
      // "Header+body" mode:
      else
      {
        // Determine how many decimal places are best suited to this measurement.
        uint8_t decimals = KLineKWP1281Lib::getMeasurementDecimalsFromHeader(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer));
        
        // Display the calculated value, with the recommended amount of decimals.
        
        
        display.setTextSize(3);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 20);
        display.print(KLineKWP1281Lib::getMeasurementValueFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer)), decimals);
        display.setCursor(0, 45);
        display.setTextSize(2);
        display.print(KLineKWP1281Lib::getMeasurementUnitsFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer), units_string, sizeof(units_string)));
        display.setTextSize(1);
        display.setCursor(80, 50);
        display.print("HBM");
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
    }
    break;
    
    // "Text" type
    case KLineKWP1281Lib::TEXT:
    {
      // The measurement's text will be copied into this array, so it can be displayed.
      char text_string[16];
      
      // Display the text.
      if (!received_group_header)
      {
        // The function getMeasurementText() returns the same string that it's given. It's the same as text_string.

        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print(KLineKWP1281Lib::getMeasurementText(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer), text_string, sizeof(text_string)));
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
      else
      {
        // The function getMeasurementTextFromHeaderBody() returns the same string that it's given. It's the same as text_string.
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print(KLineKWP1281Lib::getMeasurementTextFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer), text_string, sizeof(text_string)));
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
    }
    break;
    
    // Invalid measurement index
    case KLineKWP1281Lib::UNKNOWN:

      display.clearDisplay();
      display.setTextSize(3);
      display.setTextColor(SSD1306_WHITE);
      display.print("N/A");
      display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      break;
  }
}

void showSingleMeasurement(uint8_t group, uint8_t measurement_index)
{
  // This will contain the amount of measurements in the current group, after calling the readGroup() function.
  uint8_t amount_of_measurements = 0;
   
  // Read the requested group and store the returned value.
  KLineKWP1281Lib::executionStatus readGroup_status;
  // If the group is not of "header+body" type, or if it is and this is the first request, we don't have a [Header] (yet), so `received_group_header=false`.
  // The response to this request will be stored in the larger array.
  // If it is in fact of "header+body" type, the [Header] will be stored in this array.
  if (!received_group_header)
  {
    readGroup_status = diag.readGroup(amount_of_measurements, group, measurement_buffer, sizeof(measurement_buffer));
  }
  // If the group is of "header+body" type, and this is not the first request, it means we have a header, so `received_group_header=true`.
  // The response to this request will be stored in the smaller array, because it should be the [Body].
  else
  {
    readGroup_status = diag.readGroup(amount_of_measurements, group, measurement_body_buffer, sizeof(measurement_body_buffer));
  }
  
  // Check the returned value.
  switch (readGroup_status)
  {
    case KLineKWP1281Lib::ERROR:
      {
        display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
        display.setCursor(0, 0);
        display.print("Error reading group!");
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
      // There is no reason to continue, exit the function.
      return;
    
    case KLineKWP1281Lib::FAIL:
      {
        display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
        display.setCursor(0, 0);
        display.print("G:");
        display.print(group);
        display.print(" does not exist!");
        display.print("Error reading group!");
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
      // There is no reason to continue, exit the function.
      return;

    case KLineKWP1281Lib::GROUP_BASIC_SETTINGS:
      {
        // If we have a [Header], it means this group sends responses of "header+body" type.
        // So, at this point, it doesn't make sense to receive something other than a [Body].
        if (received_group_header)
        {
          display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
          display.setCursor(0, 0);
          display.print("Error reading body! 1"); //("Error reading body! (got basic settings)");
          display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
          return;
        }
        /* idegilenesen kiszedve!
        // We have 10 raw values in the `measurement_buffer` array.
        Serial.print("Basic settings in group ");
        Serial.print(group);
        Serial.print(": ");
        for (uint8_t i = 0; i < 10; i++)
        {
          Serial.print(measurement_buffer[i]);
          Serial.print(" ");
        }
        Serial.println();
        
        */
      }


      // We have nothing else to display, exit the function.
      return;

    case KLineKWP1281Lib::GROUP_HEADER:
      {
        // If we have a [Header], it means this group sends responses of "header+body" type.
        // So, at this point, it doesn't make sense to receive something other than a [Body].
        if (received_group_header)
        {
          display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
          display.setCursor(0, 0);
          display.print("Error reading body! 2"); //("Error reading body! (got header)");
          display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
          return;
        }

        // Set the flag to indicate that a header was received, making this a "header+body" group response.
        received_group_header = true;

        // Store the number of measurements received in the header.
        amount_of_measurements_in_header = amount_of_measurements;
      }
      // We have nothing to display yet, the next readGroup() will get the actual data; exit the function.
      return;

    case KLineKWP1281Lib::GROUP_BODY:
      {
        // If we don't have a [Header], it doesn't make sense to receive a [Body].
        if (!received_group_header)
        {
          display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
          display.setCursor(0, 0);
          display.print("Error reading header! 3"); //("Error reading header! (got body)");
          display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
          return;
        }
      }
      // If we have the [Header], now we also have the [Body]; execute the code after the switch().
      break;
    
    // Execute the code after the switch().
    case KLineKWP1281Lib::SUCCESS:
      break;
  }
  
  /*
    The getMeasurementType() function can return:
      *KLineKWP1281Lib::UNKNOWN - index out of range (measurement doesn't exist in group)
      *KLineKWP1281Lib::VALUE   - regular measurement, with a value and units
      *KLineKWP1281Lib::TEXT    - text measurement
  */
  
  // Get the selected measurement's type.
  KLineKWP1281Lib::measurementType measurement_type;
  if (!received_group_header)
  {
    measurement_type = KLineKWP1281Lib::getMeasurementType(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer));
  }
  // For "header+body" measurements, you need to use this other function that specifically parses headers instead of regular responses.
  else
  {
    measurement_type = KLineKWP1281Lib::getMeasurementTypeFromHeader(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer));
  }
  
  // Check the returned value.
  switch (measurement_type)
  {
    // "Value and units" type
    case KLineKWP1281Lib::VALUE:
    {
      // The measurement's units will be copied into this array, so they can be displayed.
      char units_string[16];
        
      // Regular mode:
      if (!received_group_header)
      {
        // Determine how many decimal places are best suited to this measurement.
        uint8_t decimals = KLineKWP1281Lib::getMeasurementDecimals(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer));
        
        // Display the calculated value, with the recommended amount of decimals.
        
        display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
        display.setCursor(0, 0);
        display.print(KLineKWP1281Lib::getMeasurementValue(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer)), decimals);
        display.print(' ');
        display.print(KLineKWP1281Lib::getMeasurementUnits(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer), units_string, sizeof(units_string)));
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
        
      }
      // "Header+body" mode:
      else
      {
        // Determine how many decimal places are best suited to this measurement.
        uint8_t decimals = KLineKWP1281Lib::getMeasurementDecimalsFromHeader(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer));
        
        // Display the calculated value, with the recommended amount of decimals.
        
      
        display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
        display.setCursor(0, 0);
        display.print(KLineKWP1281Lib::getMeasurementValueFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer)), decimals);
        display.print(' ');
        display.print(KLineKWP1281Lib::getMeasurementUnitsFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer), units_string, sizeof(units_string)));
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
    }
    break;
    
    // "Text" type
    case KLineKWP1281Lib::TEXT:
    {
      // The measurement's text will be copied into this array, so it can be displayed.
      char text_string[16];
      
      // Display the text.
      if (!received_group_header)
      {
        // The function getMeasurementText() returns the same string that it's given. It's the same as text_string.

        display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
        display.setCursor(0, 0);
        display.print(KLineKWP1281Lib::getMeasurementText(measurement_index, amount_of_measurements, measurement_buffer, sizeof(measurement_buffer), text_string, sizeof(text_string)));
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
      else
      {
        // The function getMeasurementTextFromHeaderBody() returns the same string that it's given. It's the same as text_string.
        display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
        display.setCursor(0, 0);
        display.print(KLineKWP1281Lib::getMeasurementTextFromHeaderBody(measurement_index, amount_of_measurements_in_header, measurement_buffer, sizeof(measurement_buffer), amount_of_measurements, measurement_body_buffer, sizeof(measurement_body_buffer), text_string, sizeof(text_string)));
        display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      }
    }
    break;
    
    // Invalid measurement index
    case KLineKWP1281Lib::UNKNOWN:

       display.fillRect(0, 0, SCREEN_1_WIDTH, 8, SSD1306_BLACK); // előző szöveg "kitörlése"
       display.setCursor(0, 0);
       display.print("N/A");
       display.display(); // teljes kijelző újraküldése, de csak ez a sor változott
      break;
  }
}
