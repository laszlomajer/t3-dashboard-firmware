/*
HW - v3 20261113

Tesztelt verzió Kijelzés és lookup table kalibrálandó lehet.

Az eredeti működő v3 program alapján.

ADC módosítás:
- NINCS szekvenciális ADC olvasás
- NINCS ADC10DTC
- NINCS ADC interrupt
- Az ADC csatornák egyenként kerülnek kiolvasásra

ADC csatornák:
A5 = Fuel Level
A4 = Coolant Temperature
A3 = Coolant Level
A0 = Voltage level
A1 = Dimmer input

Az eredeti logikák változatlanok:
- I2C
- PWM
- Oil pressure
- Coolant warning
- Fuel warning
- WDT
- Timer capture
- UART
*/

/*
I2C kommunikáció

BYTE 0    BYTE 1       BYTE 2       BYTE 3       BYTE 4       BYTE 5       BYTE 6       BYTE 7       BYTE 8       BYTE 9
┌───────┬────────────┬────────────┬────────────┬────────────┬────────────┬────────────┬────────────┬────────────┬───────┐
│ 0x5A  │ DC BUS     │ DIMMER     │ COOL LEVEL │ COOL TEMP  │ FUEL       │ RESERVED   │ PERIOD     │ STATUS     │ CRC   │
└───────┴────────────┴────────────┴────────────┴────────────┴────────────┴────────────┴────────────┴────────────┴───────┘
   0        1            2            3            4            5            6            7            8            9

Status:

bit 7     bit 6    bit 5      bit 4          bit 3             bit 2   bit 1   bit 0
OILWARN   LT_ON    FUEL_LOW   COOL_LOW       COOL_TEMP_HIGH    B2      F22     F1


Uart monitor:

================================
 T3 Dasboard STATUS MONITOR
================================

ADC
A0 DCBUS:        13
A1 DIMMER:       75
A3 COOL LEVEL:   512
A4 COOL TEMP:    92
A5 FUEL:         68

STATUS
F1:              1
F22:             0
B2:              0
COOL_TEMP_HIGH:  0
COOL_LEVEL_LOW:  0
FUEL_LOW:        0
LT_ON:           1
OILWARN:         0
STATUS ALL:      64

PWM
TA0.1 COOLANT:   460
TA0.2 FUEL:      320
TA1.2 BACKLIGHT: 750
PERIOD:          125

*/

#include "intrinsics.h"
#include "msp430g2553.h"
#include <msp430.h>
#include <stdint.h>
#include "lookup_tables.h"
#include "signal_processing.h"


#define FUEL_WARN     10

#define PWM_FREQ      30000UL
#define SMCLK_FREQ    1000000UL
#define TIMER_PERIOD  (SMCLK_FREQ / PWM_FREQ)


#define OIL_WARN_ON         (P3OUT |= BIT2)
#define OIL_WARN_OFF        (P3OUT &= ~BIT2)
#define OIL_WARN_TOGGLE     (P3OUT ^= BIT2)

#define SPARE_LED_ON        (P3OUT |= BIT3)
#define SPARE_LED_OFF       (P3OUT &= ~BIT3)
#define SPARE_LED_TOGGLE    (P3OUT ^= BIT3)

#define COOLANT_WARN_ON     (P2OUT |= BIT5)
#define COOLANT_WARN_OFF    (P2OUT &= ~BIT5)
#define COOLANT_WARN_TOGGLE (P2OUT ^= BIT5)

#define SPD_RAW_ON          (P3OUT |= BIT4)
#define SPD_RAW_OFF         (P3OUT &= ~BIT4)

#define COOL_AC_OUT         (P3OUT ^= BIT7)


#define F22_OIL             (P2IN & BIT3)
#define F1_OIL              (P2IN & BIT2)

#define LIGHT               (P2IN & BIT4)


// ============================================================
// ADC INDEXEK
//
// Fontos:
// Az eredeti logika miatt az adc_values[0..2] jelentése
// továbbra is ugyanaz marad.
//
// adc_values[0] = A5 = Fuel
// adc_values[1] = A4 = Coolant temperature
// adc_values[2] = A3 = Coolant level
// adc_values[3] = A0 = DC BUS Voltage
// adc_values[4] = A1 = Dimmer Voltage
// ============================================================

#define NUM_ADC_CHANNELS       5

#define ADC_FUEL_INDEX         0
#define ADC_COOLANT_TEMP_INDEX 1
#define ADC_COOLANT_LEVEL_INDEX 2
#define ADC_A0_INDEX           3
#define ADC_A1_INDEX           4

unsigned int adc_values[NUM_ADC_CHANNELS];

/* Physical sensor values are defined in signal_processing.c. */



// ============================================================
// TIMER VARIABLES
// ============================================================

volatile unsigned int capture1 = 0;
volatile unsigned int capture2 = 0;
volatile unsigned long period = 0;
uint8_t period_8bit = 0;

volatile unsigned int timer1_overflow_count = 0;
volatile unsigned int capture_overflow_count = 0;


// ============================================================
// STATUS BITFIELD
// ============================================================

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


bitfield status = { .all = 0 };
bitfield in_status = { .all = 0 };

unsigned int backlight_lookup_index = 0;
unsigned int backlight_pwm = 0;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

void set_CoolantPWM_duty_cycle(unsigned int duty);
void set_FuelPWM_duty_cycle(unsigned int duty);

unsigned int read_ADC_channel(unsigned int channel);

void read_all_ADC(void);

/* Convert raw ADC readings to the established physical/logical values.
 * Called from existing processing functions; main-loop order is unchanged.
 */
void update_physical_sensor_values(void);


// ============================================================
// PWM
// ============================================================

void init_PWM(void)
{
    // P3.5 -> TA0.1
    // P3.6 -> TA0.2

    P3DIR |= BIT5 | BIT6;
    P3SEL |= BIT5 | BIT6;

    TA0CCR0 = TIMER_PERIOD - 1;

    TA0CCTL1 = OUTMOD_7;
    TA0CCTL2 = OUTMOD_7;

    TA0CCR1 = 0;
    TA0CCR2 = 0;

    TA0CTL = TASSEL_2 | MC_1 | TACLR;
}

/*
void init_PWM_TA1(void)
{
    // P3.3 -> TA1.2

    P3DIR |= BIT3;
    P3SEL |= BIT3;

    TA1CCR0 = 1000 - 1;

    TA1CCTL2 = OUTMOD_7;

    TA1CCR2 = (1000 * 10) / 100;

    TA1CTL = TASSEL_2 | MC_1 | TACLR;
}
*/

void init_PWM_TA1(void)
{
    // --------------------------------------------------------
    // P3.3 -> TA1.2 PWM
    // --------------------------------------------------------

    P3DIR |= BIT3;
    P3SEL |= BIT3;

    TA1CCR0 = 1000 - 1;

    TA1CCTL2 = OUTMOD_7;

    TA1CCR2 = (1000 * 10) / 100;


    // --------------------------------------------------------
    // P2.1 -> TA1.1 CAPTURE
    // Fordulatszám bemenet
    // --------------------------------------------------------

    P2DIR &= ~BIT1;

    P2SEL |= BIT1;
    P2SEL2 &= ~BIT1;

    TA1CCTL1 = CM_1 | CCIS_0 | SCS | CAP | CCIE;


    // --------------------------------------------------------
    // Timer1_A
    // --------------------------------------------------------

    TA1CTL = TASSEL_2 | MC_1 | TACLR | TAIE;
}

void set_CoolantPWM_duty_cycle(unsigned int duty)
{
    if (duty > 100)
        duty = 100;

    TA0CCR1 = (TIMER_PERIOD * duty) / 100;
}


void set_FuelPWM_duty_cycle(unsigned int duty)
{
    if (duty > 100)
        duty = 100;

    TA0CCR2 = (TIMER_PERIOD * duty) / 100;
}


// ============================================================
// ADC
//
// NINCS sequence mode
// NINCS DTC
// NINCS ADC interrupt
//
// Egyetlen ADC csatorna kerül mérésre minden híváskor.
// ============================================================

void init_ADC(void)
{
    // ADC kikapcsolása konfiguráció előtt
    ADC10CTL0 &= ~ENC;

    // ADC10 alapbeállítás:
    // - 64 ADC10CLK mintavételi idő
    // - ADC bekapcsolva
    // - belső referencia bekapcsolva
    //
    // A referencia kiválasztása mérésenként történik:
    // A5 / FUEL: belső 1,5 V (SREF_1)
    // minden más csatorna: AVCC / AVSS (SREF_0)
    //
    // REF2_5V nincs beállítva, ezért a belső referencia 1,5 V.

    ADC10CTL0 = ADC10SHT_3 | ADC10ON | REFON;

    // Induláskor AVCC referencia legyen az alapértelmezett.
    ADC10CTL0 &= ~SREF_7;

    // Nem szekvenciális mód
    // CONSEQ bitek = 00
    ADC10CTL1 = 0;

    // Analóg bemenetek:
    // A0 = BIT0, A1 = BIT1, A3 = BIT3, A4 = BIT4, A5 = BIT5
    ADC10AE0 = BIT0 | BIT1 | BIT3 | BIT4 | BIT5;

    // A belső referencia bekapcsolási idejének kivárása.
    // A projekt MCLK frekvenciája 1 MHz.
    // 100 ciklus = kb. 100 us, ami kényelmes tartalékot ad.
    __delay_cycles(100);
}


// ============================================================
// EGY ADC CSATORNA OLVASÁSA
// ============================================================

unsigned int read_ADC_channel(unsigned int channel)
{
    unsigned int result;
    unsigned int dummy;

    // ADC konverzió letiltása konfigurációváltás előtt.
    ADC10CTL0 &= ~ENC;

    // Megvárjuk az esetleges előző konverzió végét.
    while (ADC10CTL1 & ADC10BUSY)
    {
        ;
    }

    // Csak az A5 / FUEL csatorna használja a belső 1,5 V referenciát.
    // Minden más csatorna továbbra is AVCC referenciával mér.
    ADC10CTL0 &= ~SREF_7;

    if (channel == INCH_5)
    {
        // Vref+ = belső 1,5 V, Vref- = AVSS.
        // REF2_5V nincs beállítva.
        ADC10CTL0 |= SREF_1;
    }
    else
    {
        // Vref+ = AVCC, Vref- = AVSS.
        ADC10CTL0 |= SREF_0;
    }

    // Belső referencia bekapcsolva marad, így csatornaváltáskor
    // nem kell újra kivárni a teljes referencia-indulási időt.
    ADC10CTL1 = channel;

    // Első konverzió eldobása:
    // segít a csatorna- és referencia-multiplexer átállása után
    // beállni a mintavevő kondenzátornak.
    ADC10CTL0 |= ENC;
    ADC10CTL0 |= ADC10SC;

    while (ADC10CTL1 & ADC10BUSY)
    {
        ;
    }

    dummy = ADC10MEM;
    (void)dummy;

    ADC10CTL0 &= ~ENC;

    // Második konverzió: ez lesz a tényleges mérési eredmény.
    ADC10CTL1 = channel;

    ADC10CTL0 |= ENC;
    ADC10CTL0 |= ADC10SC;

    while (ADC10CTL1 & ADC10BUSY)
    {
        ;
    }

    result = ADC10MEM;

    // ADC tiltása a következő csatorna konfigurálásához.
    ADC10CTL0 &= ~ENC;

    return result;
}


// ============================================================
// MIND AZ ÖT ADC CSATORNA KIOLVASÁSA
// ============================================================

void read_all_ADC(void)
{
    /*
     * Az eredeti logika miatt a tömb sorrendje:
     *
     * adc_values[0] = A5 = Fuel
     * adc_values[1] = A4 = Coolant temperature
     * adc_values[2] = A3 = Coolant level
     * adc_values[3] = A0
     * adc_values[4] = A1
     */


    // --------------------------------------------------------
    // A5 - Fuel Level
    // --------------------------------------------------------

    adc_values[ADC_FUEL_INDEX] =
        read_ADC_channel(INCH_5);


    // --------------------------------------------------------
    // A4 - Coolant Temperature
    // --------------------------------------------------------

    adc_values[ADC_COOLANT_TEMP_INDEX] =
        read_ADC_channel(INCH_4);


    // --------------------------------------------------------
    // A3 - Coolant Level
    // --------------------------------------------------------

    adc_values[ADC_COOLANT_LEVEL_INDEX] =
        read_ADC_channel(INCH_3);


    // --------------------------------------------------------
    // A0 - új analóg bemenet
    // --------------------------------------------------------

    adc_values[ADC_A0_INDEX] =
        read_ADC_channel(INCH_0);


    // --------------------------------------------------------
    // A1 - új analóg bemenet
    // --------------------------------------------------------

    adc_values[ADC_A1_INDEX] =
        read_ADC_channel(INCH_1);
}


// ============================================================
// OIL PRESSURE
// ============================================================

void checkOilPressure(void)
{
    char oil_state = 0;


    if ((F22_OIL) && (F1_OIL)) 
        oil_state = 1;

    else if ((!F22_OIL) && !(F1_OIL))
        oil_state = 2;

    else if ((F22_OIL) && !(F1_OIL))
        oil_state = 3;


    switch(oil_state)
    {

        case 1:

            OIL_WARN_ON;

            status.oilwarn = 1;

            break;


        case 2:

            OIL_WARN_OFF;

            status.oilwarn = 0;

            break;


        case 3:

            //OIL_WARN_TOGGLE;

            status.oilwarn = 1;

            break;


        default:

            OIL_WARN_ON;

            status.oilwarn = 1;

            break;
    }
}


// ============================================================
// IO INIT
// ============================================================

void init_IO(void)
{
    P3DIR |= BIT7 +
             BIT2 +
             BIT3 +
             BIT4;


    P3OUT &= ~BIT7;


    P2DIR |= BIT5;

    P2OUT &= ~BIT5;


    P2DIR &= ~(BIT2 | BIT3 | BIT4);


    P2REN |= BIT2 | BIT3 | BIT4;

    P2OUT |= BIT2 | BIT3 | BIT4;
}


// ============================================================
// WATCHDOG
// ============================================================

void init_WDT_timer(void)
{
    WDTCTL = WDT_MDLY_0_064;

    IE1 |= WDTIE;

    __enable_interrupt();
}


// ============================================================
// I2C
// ============================================================

#define I2C_ADDRESS 0x42


volatile uint8_t i2c_data[8] =
{
    0, 0, 0, 0, 0, 0, 0, 0
};


volatile uint8_t tx_buffer[10];

volatile uint8_t tx_index = 0;


// ------------------------------------------------------------
// I2C RX
// ------------------------------------------------------------

#define I2C_RX_BUFFER_SIZE 4

uint8_t i2c_rx_buffer[I2C_RX_BUFFER_SIZE];

uint8_t i2c_rx_index = 0;

uint8_t i2c_data_ready = 0;


// ============================================================
// CRC
// ============================================================

uint8_t calculate_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    uint8_t i;


    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
    }


    return crc;
}


// ============================================================
// I2C TX BUFFER
// ============================================================

void prepare_tx_buffer()
{
    tx_buffer[0] = 0x5A;


    uint8_t i;


    for (i = 0; i < 8; i++)
    {
        tx_buffer[1 + i] = i2c_data[i];
    }


    tx_buffer[9] = calculate_crc(tx_buffer, 9);


    tx_index = 0;
}


// ============================================================
// I2C INIT
// ============================================================

void i2c_init_slave()
{
    P1SEL |= BIT6 + BIT7;

    P1SEL2 |= BIT6 + BIT7;


    UCB0CTL1 |= UCSWRST;


    UCB0CTL0 = UCMODE_3 + UCSYNC;


    UCB0I2COA = I2C_ADDRESS;


    UCB0CTL1 &= ~UCSWRST;


    IE2 |= UCB0TXIE | UCB0RXIE;
}


// ============================================================
// UART
//
// FONTOS:
// P1.1 = A1 ADC bemenet lett.
//
// Ezért az UART RX funkciót NEM kapcsoljuk rá P1.1-re.
//
// UART TX továbbra is P1.2.
// ============================================================

void init_UART(void)
{
    // P1.2 = TXD
    //
    // P1.1 már ADC A1!

    P1SEL |= BIT2;

    P1SEL2 |= BIT2;


    UCA0CTL1 |= UCSSEL_2;


    UCA0BR0 = 104;

    UCA0BR1 = 0;


    UCA0MCTL = UCBRS0;


    UCA0CTL1 &= ~UCSWRST;
}


// ============================================================
// UART SEND
// ============================================================

void uart_putc(char c)
{
    while (!(IFG2 & UCA0TXIFG))
    {
        ;
    }


    UCA0TXBUF = c;
}


void uart_puts(const char *str)
{
    while (*str)
    {
        uart_putc(*str++);
    }
}


void uart_print_uint(unsigned int val)
{
    char buffer[6];

    int i = 0;


    if (val == 0)
    {
        uart_putc('0');

        return;
    }


    while (val > 0 && i < sizeof(buffer) - 1)
    {
        buffer[i++] = (val % 10) + '0';

        val /= 10;
    }


    while (i--)
    {
        uart_putc(buffer[i]);
    }
}

void uart_print_ulong(unsigned long val)
{
    char buffer[11];
    int i = 0;

    if (val == 0)
    {
        uart_putc('0');
        return;
    }

    while (val > 0 && i < sizeof(buffer) - 1)
    {
        buffer[i++] = (val % 10) + '0';
        val /= 10;
    }

    while (i--)
    {
        uart_putc(buffer[i]);
    }
}

// ============================================================
// ADC -> I2C
//
// AZ EREDETI LOGIKA MEGMARAD.
//
// I2C:
// #1 = Fuel = A5
// #2 = Coolant temperature = A4
// #3 = Coolant level = A3
//
// A0/A1 nem kerül ide.
// ============================================================

void adc_to_i2c(void)
{
    // A0 = DC BUS feszültség
    i2c_data[0] =
        dcbus_lookup[adc_values[ADC_A0_INDEX] >> 2];

    // A1 = Dimmer %
    i2c_data[1] =
        dimmer_lookup[adc_values[ADC_A1_INDEX] >> 2];

    // A3 = Coolant level
    i2c_data[2] =
        adc_values[ADC_COOLANT_LEVEL_INDEX] >> 2;

    // A4 = Coolant temperature
    i2c_data[3] =
        coolant_temp_lookup[adc_values[ADC_COOLANT_TEMP_INDEX] >> 2];

    // A5 = Fuel
    i2c_data[4] =
        fuel_percent_lookup[adc_values[ADC_FUEL_INDEX] >> 2];
}


// ============================================================
// STATUS
// ============================================================

void update_status(void)
{
    status.lt_on =
        LIGHT ? 1 : 0;


    status.f1 =
        F1_OIL ? 1 : 0;


    status.f22 =
        F22_OIL ? 1 : 0;


    if (status.lt_on)
        SPD_RAW_ON;

    else
        SPD_RAW_OFF;

    period_8bit = (uint8_t)(period >> 2);
    i2c_data[6] = period_8bit;

    i2c_data[7] =
        status.all;
}


// ============================================================
// SENSOR VALUE CONVERSION (NEW DATA-PROCESSING LAYER)
//
// The ADC acquisition itself is untouched. This function only looks up the
// already-calibrated sensor values. The fuel lookup is intentionally based
// on the raw ADC reading here, matching the existing I2C/UART behavior.
// The coolant-level AC conductivity probe is converted to a boolean state.
// ============================================================

/* Implemented in signal_processing.c. */


// ============================================================
// COOLANT CHECK
//
// EREDETI LOGIKA VÁLTOZATLAN
//
// adc_values[1] = A4
// adc_values[2] = A3
// ============================================================

void coolant_check(void)
{
    update_physical_sensor_values();

    /* Legacy ADC threshold <250 corresponds to >=95 °C in the existing
     * coolant_temp_lookup table. Warning now uses the physical value. */
    if (coolant_temp_c >= 95U)
    {
        COOLANT_WARN_ON;
        status.coolant_temp_high = 1;
    }
    else
    {
        status.coolant_temp_high = 0;

        if (coolant_level_ok == 0U)
        {
            COOLANT_WARN_TOGGLE;
            status.coolant_level_low = 1;
        }
        else
        {
            COOLANT_WARN_OFF;
            status.coolant_level_low = 0;
        }
    }
}


// ============================================================
// FUEL CHECK
//
// EREDETI LOGIKA VÁLTOZATLAN
//
// adc_values[0] = A5
// ============================================================

void fuel_check(void)
{
    if (i2c_data[4] < FUEL_WARN)
    {
        status.fuel_low = 1;
    }

    else
    {
        status.fuel_low = 0;
    }
}


// ============================================================
// TA1.2 DIMMER PWM
//
// P3.3 -> TA1.2
//
// ADC A1 = Dimmer input
// adc_values[ADC_A1_INDEX] = A1 ADC érték
//
// A dimmer_lookup táblából történik a PWM meghatározása.
//
// status.lt_on:
//     0 = LIGHT LOW
//     1 = LIGHT HIGH
//
// Ha LIGHT HIGH, a lookup index eltolható.
// ============================================================

#define DIMMER_LOOKUP_OFFSET  20

void update_backlight(void)
{
    unsigned int index;


    // --------------------------------------------------------
    // A1 ADC -> 8 bites lookup index
    //
    // 10 bites ADC (0...1023)
    // >> 2 -> 0...255
    // --------------------------------------------------------

    index = adc_values[ADC_A1_INDEX] >> 2;


    // --------------------------------------------------------
    // LIGHT állapot figyelembevétele
    //
    // Ha a LIGHT aktív, eltoljuk a lookup indexet.
    // --------------------------------------------------------

    if (status.lt_on)    {
        if (index >= DIMMER_LOOKUP_OFFSET)
            index -= DIMMER_LOOKUP_OFFSET;

        else
            index = 10;
    }


    // --------------------------------------------------------
    // Lookup table kiolvasása
    // --------------------------------------------------------

    backlight_lookup_index = index;

    backlight_pwm = dimmer_lookup[index];

    // --------------------------------------------------------
    // TA1.2 PWM frissítése
    //
    // TA1CCR0 = 999
    // ezért 0...1000 közötti értéket várunk.
    // --------------------------------------------------------

    if (backlight_pwm > 1000)
        backlight_pwm = 1000;

    TA1CCR2 = backlight_pwm;
}

// ============================================================
// UART STATUS KIÍRÁS
// ============================================================

void uart_print_diagnostics(void)
{
    static unsigned char first_run = 1;

    /* Diagnostic-only values. These do not alter the control outputs. */
    unsigned int coolant_duty;
    unsigned int fuel_duty;
    unsigned int backlight_duty;
    unsigned int backlight_offset;
    unsigned int coolant_lookup_value;
    unsigned int fuel_lookup_value;

    /* Reuse the exact lookup-table inputs used by the control loop. */
    coolant_lookup_value = coolant_temp_c;
    if (coolant_lookup_value > 115U)
        coolant_lookup_value = 115U;

    fuel_lookup_value = fuel_gauge_percent_filtered;
    if (fuel_lookup_value > 100U)
        fuel_lookup_value = 100U;

    coolant_duty = coolant_gauge_pwm_lookup[coolant_lookup_value];
    fuel_duty = fuel_gauge_pwm_lookup[fuel_lookup_value];

    backlight_offset = status.lt_on ? DIMMER_LOOKUP_OFFSET : 0U;

    if ((unsigned int)TA1CCR0 + 1U != 0U)
        backlight_duty =
            ((unsigned long)TA1CCR2 * 100UL) /
            ((unsigned int)TA1CCR0 + 1U);
    else
        backlight_duty = 0U;

    if (first_run)
    {
        uart_puts("\033[2J\033[H");
        uart_puts("================================\r\n");
        uart_puts(" T3 Dashboard STATUS MONITOR\r\n");
        uart_puts("================================\r\n");
        uart_puts("\r\n");
        uart_puts("ADC               FIRMWARE VALUE                 RAW ADC\r\n");
        uart_puts("A0 DCBUS:\r\n");
        uart_puts("A1 DIMMER:\r\n");
        uart_puts("A3 COOL LEVEL:\r\n");
        uart_puts("A4 COOL TEMP:\r\n");
        uart_puts("A5 FUEL:\r\n");
        uart_puts("\r\nSTATUS\r\n");
        uart_puts("F1:\r\n");
        uart_puts("F22:\r\n");
        uart_puts("B2:\r\n");
        uart_puts("COOL_TEMP_HIGH:\r\n");
        uart_puts("COOL_LEVEL_LOW:\r\n");
        uart_puts("FUEL_LOW:\r\n");
        uart_puts("LT_ON:\r\n");
        uart_puts("OILWARN:\r\n");
        uart_puts("STATUS ALL:\r\n");

        /* Explicit row/column labels for the PWM diagnostic matrix. */
        uart_puts("\033[23;1HPWM DIAGNOSTICS");
        uart_puts("\033[24;1HRAW ADC");
        uart_puts("\033[25;1HLOOKUP");
        uart_puts("\033[26;1HOFFSET");
        uart_puts("\033[27;1HDUTY (%)");
        uart_puts("\033[28;1HCCR");
        uart_puts("\033[29;1HLOOKUP INDEX");
        uart_puts("\033[30;1HCAPTURE PERIOD");

        uart_puts("\033[23;18HCOOLANT");
        uart_puts("\033[23;34HFUEL");
        uart_puts("\033[23;50HBACKLIGHT");

        first_run = 0;
    }

    /* ADC: converted value at column 18, raw 10-bit value at column 42. */
    uart_puts("\033[6;18H");
    uart_print_uint(dcbus_lookup[adc_values[ADC_A0_INDEX] >> 2]);
    uart_puts("    ");
    uart_puts("\033[6;42H");
    uart_print_uint(adc_values[ADC_A0_INDEX]);
    uart_puts("    ");

    uart_puts("\033[7;18H");
    uart_print_uint(dimmer_lookup[adc_values[ADC_A1_INDEX] >> 2]);
    uart_puts("    ");
    uart_puts("\033[7;42H");
    uart_print_uint(adc_values[ADC_A1_INDEX]);
    uart_puts("    ");

    uart_puts("\033[8;18H");
    uart_print_uint(adc_values[ADC_COOLANT_LEVEL_INDEX]);
    uart_puts("    ");
    uart_puts("\033[8;42H");
    uart_print_uint(adc_values[ADC_COOLANT_LEVEL_INDEX]);
    uart_puts("    ");

    uart_puts("\033[9;18H");
    uart_print_uint(coolant_temp_lookup[adc_values[ADC_COOLANT_TEMP_INDEX] >> 2]);
    uart_puts("    ");
    uart_puts("\033[9;42H");
    uart_print_uint(adc_values[ADC_COOLANT_TEMP_INDEX]);
    uart_puts("    ");

    uart_puts("\033[10;18H");
    uart_print_uint(fuel_percent_lookup[adc_values[ADC_FUEL_INDEX] >> 2]);
    uart_puts("    ");
    uart_puts("\033[10;42H");
    uart_print_uint(adc_values[ADC_FUEL_INDEX]);
    uart_puts("    ");

    /* STATUS */
    uart_puts("\033[13;18H"); uart_print_uint(status.f1);
    uart_puts("\033[14;18H"); uart_print_uint(status.f22);
    uart_puts("\033[15;18H"); uart_print_uint(status.b2);
    uart_puts("\033[16;18H"); uart_print_uint(status.coolant_temp_high);
    uart_puts("\033[17;18H"); uart_print_uint(status.coolant_level_low);
    uart_puts("\033[18;18H"); uart_print_uint(status.fuel_low);
    uart_puts("\033[19;18H"); uart_print_uint(status.lt_on);
    uart_puts("\033[20;18H"); uart_print_uint(status.oilwarn);
    uart_puts("\033[21;18H"); uart_print_uint(status.all);

    /*
     * PWM diagnostic matrix:
     * rows = measurement stage; columns = coolant / fuel / backlight.
     * No control register or lookup-table value is modified here.
     */

    /* Row 24: source RAW ADC */
    uart_puts("\033[24;18H"); uart_print_uint(adc_values[ADC_COOLANT_TEMP_INDEX]);
    uart_puts("\033[24;34H"); uart_print_uint(adc_values[ADC_FUEL_INDEX]);
    uart_puts("\033[24;50H"); uart_print_uint(adc_values[ADC_A1_INDEX]);

    /* Row 25: physical/lookup value used for gauge mapping */
    uart_puts("\033[25;18H"); uart_print_uint(coolant_lookup_value);
    uart_puts("\033[25;34H"); uart_print_uint(fuel_lookup_value);
    uart_puts("\033[25;50H"); uart_print_uint(backlight_pwm);

    /* Row 26: runtime offset; coolant/fuel corrections are embedded in LUTs. */
    uart_puts("\033[26;18H"); uart_print_uint(0U);
    uart_puts("\033[26;34H"); uart_print_uint(0U);
    uart_puts("\033[26;50H"); uart_print_uint(backlight_offset);

    /* Row 27: requested PWM duty in percent */
    uart_puts("\033[27;18H"); uart_print_uint(coolant_duty);
    uart_puts("\033[27;34H"); uart_print_uint(fuel_duty);
    uart_puts("\033[27;50H"); uart_print_uint(backlight_duty);

    /* Row 28: actual timer compare register values */
    uart_puts("\033[28;18H"); uart_print_uint(TA0CCR1);
    uart_puts("\033[28;34H"); uart_print_uint(TA0CCR2);
    uart_puts("\033[28;50H"); uart_print_uint(TA1CCR2);

    /* Row 29: backlight lookup index (only applicable to this channel). */
    uart_puts("\033[29;50H"); uart_print_uint(backlight_lookup_index);

    /* Row 30: existing speed/capture period diagnostic. */
    uart_puts("\033[30;18H"); uart_print_uint(period_8bit);
}

// ============================================================
// MAIN
// ============================================================

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;


    // --------------------------------------------------------
    // 1 MHz DCO
    // --------------------------------------------------------

    BCSCTL1 = CALBC1_1MHZ;

    DCOCTL = CALDCO_1MHZ;


    // --------------------------------------------------------
    // I2C TX buffer
    // --------------------------------------------------------

    prepare_tx_buffer();


    // --------------------------------------------------------
    // IO
    // --------------------------------------------------------

    init_IO();


    // --------------------------------------------------------
    // ADC
    // --------------------------------------------------------

    init_ADC();


    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    i2c_init_slave();


    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    init_PWM();

    init_PWM_TA1();


    // --------------------------------------------------------
    // UART
    // --------------------------------------------------------

    init_UART();


    // --------------------------------------------------------
    // WDT
    // --------------------------------------------------------

    init_WDT_timer();


    uart_puts("INIT DONE!\r\n");


    // --------------------------------------------------------
    // Kezdeti ADC mérés
    // --------------------------------------------------------

    read_all_ADC();


    __enable_interrupt();


    // --------------------------------------------------------
    // Kezdeti PWM állapot
    // --------------------------------------------------------

    set_CoolantPWM_duty_cycle(0);

    set_FuelPWM_duty_cycle(0);


    COOLANT_WARN_OFF;


    // ========================================================
    // FŐ CIKLUS
    // ========================================================

    while (1)
    {

        // ----------------------------------------------------
        // ÖT ADC CSATORNA KIOLVASÁSA
        //
        // A5
        // A4
        // A3
        // A0
        // A1
        //
        // Mindegyik külön konverzió.
        // ----------------------------------------------------

        read_all_ADC();


        // ----------------------------------------------------
        // ADC értékek I2C-re
        //
        // A0/A1 itt nem kerül továbbításra.
        // ----------------------------------------------------

        adc_to_i2c();


        // ----------------------------------------------------
        // Status frissítés
        // ----------------------------------------------------

        update_status();


        // ----------------------------------------------------
        // Coolant check
        // ----------------------------------------------------

        coolant_check();


        // ----------------------------------------------------
        // I2C TX előkészítés
        // ----------------------------------------------------

        prepare_tx_buffer();


        // ----------------------------------------------------
        // Oil pressure
        // ----------------------------------------------------

        checkOilPressure();


        // ----------------------------------------------------
        // Fuel check
        // ----------------------------------------------------

        fuel_check();


        // ----------------------------------------------------
        // Coolant PWM
        //
        // Eredeti:
        // adc_values[1] = A4
        // ----------------------------------------------------

        /* Physical temperature -> independently calibrated coolant gauge PWM.
         * The former -12 correction is embedded safely in the new table. */
        set_CoolantPWM_duty_cycle(
            coolant_gauge_pwm_lookup[coolant_temp_c]
        );


        // ----------------------------------------------------
        // Fuel PWM
        //
        // Eredeti:
        // adc_values[0] = A5
        // ----------------------------------------------------

        /* Filtered physical fuel percent -> independently calibrated gauge PWM.
         * I2C/UART continue to use the original unfiltered lookup values. */
        set_FuelPWM_duty_cycle(
            fuel_gauge_pwm_lookup[fuel_gauge_percent_filtered]
        );

        update_backlight();


        // ----------------------------------------------------
        // UART
        // ----------------------------------------------------

        uart_print_diagnostics();

        // ----------------------------------------------------
        // I2C RX feldolgozás
        // ----------------------------------------------------

        if (i2c_data_ready)
        {
            i2c_data_ready = 0;


            in_status.all =
                i2c_rx_buffer[3];


            /*
            uart_puts("I2C received: ");
            uart_putc(i2c_received_byte);
            uart_puts("\r\n");
            */


            // Itt bármilyen további logika
            // ráírható a fogadott adatokra.
        }


        // ----------------------------------------------------
        // Kis késleltetés
        // ----------------------------------------------------

        __delay_cycles(100000);


        // ----------------------------------------------------
        // Low Power Mode
        // ----------------------------------------------------

        __bis_SR_register(LPM0_bits + GIE);
    }
}


// ============================================================
// I2C TX / RX INTERRUPT
// ============================================================

#pragma vector = USCIAB0TX_VECTOR

__interrupt void USCIAB0TX_ISR(void)
{
    // --------------------------------------------------------
    // RX esemény
    // --------------------------------------------------------

    if (IFG2 & UCB0RXIFG)
    {
        i2c_rx_buffer[i2c_rx_index++] =
            UCB0RXBUF;


        if (i2c_rx_index >= I2C_RX_BUFFER_SIZE)
        {
            i2c_data_ready = 1;

            i2c_rx_index = 0;
        }
    }


    // --------------------------------------------------------
    // TX esemény
    // --------------------------------------------------------

    else if (IFG2 & UCB0TXIFG)
    {
        if (UCB0STAT & UCSTTIFG)
        {
            tx_index = 0;

            UCB0STAT &= ~UCSTTIFG;
        }


        UCB0TXBUF =
            tx_buffer[tx_index++];


        if (tx_index >= 10)
        {
            tx_index = 0;
        }
    }


    __bic_SR_register_on_exit(LPM0_bits);
}


// ============================================================
// WDT INTERRUPT
// ============================================================

#pragma vector = WDT_VECTOR

__interrupt void watchdog_timer_ISR(void)
{
    // minden kb. 16 ms

    COOL_AC_OUT;


    __bic_SR_register_on_exit(LPM0_bits);
}


// ============================================================
// TIMER1_A CAPTURE INTERRUPT
// ============================================================
/*
#pragma vector = TIMER1_A1_VECTOR

__interrupt void Timer1_A1_ISR(void)
{
    if (TA1IV == 2)
    {
        capture2 = TACCR1;

        period =
            (capture2 - capture1);

        capture1 = capture2;
    }


    __bic_SR_register_on_exit(LPM0_bits);
}

*/

#pragma vector = TIMER1_A1_VECTOR

__interrupt void Timer1_A1_ISR(void)
{
    if (TA1IV == 2)
    {
        capture2 = TA1CCR1;

        capture_overflow_count = timer1_overflow_count;

        period =
            ((unsigned long)capture_overflow_count * 1000UL)
            + capture2
            - capture1;

        capture1 = capture2;

        timer1_overflow_count = 0;
    }

    __bic_SR_register_on_exit(LPM0_bits);
}
