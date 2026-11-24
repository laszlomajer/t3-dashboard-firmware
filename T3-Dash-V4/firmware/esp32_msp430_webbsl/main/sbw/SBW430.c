/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

//******************************************************************************
// Version history:
// 1.0 07/17             Initial version. (Nima Eskandari)
// 1.1 07/17             Added Comments. (Nima Eskandari)
//----------------------------------------------------------------------------
//   Designed 2017 by Texas Instruments
//
//   Nima Eskandari
//   Texas Instruments Inc.
//   August 2017
//   Built with CCS Version: Code Composer Studio v7
//******************************************************************************

//*****************************************************************************
// Spi by Wire implementation for MSP430 Flash devices ************************
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <gpio_if.h>
#include <SBW430.h>
#include <debug.h>
#include <utils.h>
#include <Board.h>

//******************************************************************************
// Debug/Advanced Options ******************************************************
//******************************************************************************

#include <config.h>
#include <image/msp430_image.h>

//******************************************************************************
// Private functions ***********************************************************
//******************************************************************************

void delayInit();
bool DeviceHas_FastFlash(void);
bool DeviceHas_CpuX(void);
bool DeviceHas_EnhVerify(void);
void SetDevice (uint16_t wDeviceId);
void TMSH_TDIH(void);
void TMSL_TDIH(void);
void TMSL_TDIH_TDOrd(void);
void TMSL_TDIL_TDOrd(void);
void TMSH_TDIH_TDOrd(void);
void TMSH_TDIL_TDOrd(void);
void TMSL_TDIL(void);
void TMSH_TDIL(void);
void ClrTCLK(void);
void SetTCLK(void);
void IO_3state();
void DrvSignals( void );
void ResetTAP(void);
uint32_t Shift( uint32_t Format, uint32_t Data);
uint32_t IR_Shift(uint8_t instruction);
uint16_t DR_Shift16(uint16_t data);
void StartSBW(void);
uint16_t SetInstrFetch(void);
void HaltCPU(void);
void TCLKstrobes(uint16_t Amount);
void ReleaseCPU(void);
void WriteFLASH(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray);
void WriteFLASH_infoA(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray);
void StopSBW(void);
uint16_t ReadMem(uint16_t Format, uint16_t Addr);
void WriteMem(uint16_t Format, uint16_t Addr, uint16_t Data);
int ExecutePOR(void);
int GetDevice(void);
void SetPC(uint16_t Addr);
void EraseFLASH(uint16_t EraseMode, uint16_t EraseAddr);
void EraseFLASH_infoA(uint16_t EraseMode, uint16_t EraseAddr);
int VerifyPSA(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray);
int EraseCheck(uint16_t StartAddr, uint16_t Length);
int ReleaseDevice(uint16_t Addr);
int VerifyMem(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray);

#define RETRY_ON_ERROR(error)   if(error!=SC_ERR_NONE){continue;}

//******************************************************************************
// Global Variables  ***********************************************************
//******************************************************************************

uint8_t tdo_bit;               //holds the value of TDO-bit
uint8_t TCLK_saved;  // holds the last value of TCLK before entering a JTAG sequence
uint8_t JtagId = 0;

//******************************************************************************
// SBW Definition **************************************************************
//******************************************************************************

#define     SBWDATO          Board_RESET_PIN
#define     SBWDATI          Board_RESET_PIN
#define     SBWCLK           Board_TEST_PIN

#define     _NOP() __asm("    nop")
#define     SBW_DELAY //{ _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP();}

#define     ClrTST()    GPIO_IF_setOutputLowOnPin(TEST_PIN)
#define     SetTST()    GPIO_IF_setOutputHighOnPin(TEST_PIN)
#define     ClrRST()    GPIO_IF_setOutputLowOnPin(RESET_PIN)
#define     SetRST()    GPIO_IF_setOutputHighOnPin(RESET_PIN)

#define     SwitchInput()    GPIO_IF_setAsInputPin(RESET_PIN)
#define     SwitchOutput()   GPIO_IF_setAsOutputPin(RESET_PIN)

#define     GetTCLK()        GPIO_IF_getPinStatus(SBWDATO) == 0 ? 0 : 1

// SBW macros
#define     SetSBWTDIO()    GPIO_IF_setOutputHighOnPin(SBWDATO)
#define     ClrSBWTDIO()    GPIO_IF_setOutputLowOnPin(SBWDATO)
#define     SetSBWTCK()     GPIO_IF_setOutputHighOnPin(SBWCLK)
#define     ClrSBWTCK()     GPIO_IF_setOutputLowOnPin(SBWCLK)

#define     GetTDO()        GPIO_IF_getPinStatus(SBWDATO) == 0 ? 0 : 1

#define     TMSH    SetSBWTDIO(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define     TMSL    ClrSBWTDIO(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define     TMSLDH  ClrSBWTDIO(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTDIO(); SetSBWTCK(); // TMS = 0, then TCLK(TDI) immediately = 1

#define     TDIH    SetSBWTDIO();  SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();
#define     TDIL    ClrSBWTDIO();  SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK();

#define     TDOsbw  SwitchInput(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; SetSBWTCK(); SwitchOutput();
#define     TDO_RD  SwitchInput(); SBW_DELAY; ClrSBWTCK(); SBW_DELAY; tdo_bit = GetTDO(); SBW_DELAY; SetSBWTCK(); SwitchOutput();

#define TRUE          1
#define FALSE         0


static uint32_t DeviceIdx = 0;
struct tsDeviceFeatures {
    uint16_t Id;
    bool TestPin;
    bool CpuX;
    bool DataQuick;
    bool FastFlash;
    bool EnhVerify;
    bool JTAG;
    bool SpyBiWire;
    uint16_t RamStart;
    uint16_t RamEnd;
    uint16_t MainStart;
};

static const struct tsDeviceFeatures sDeviceFeatures[] =
{
//                      TestPin      DataQuick      EnhVerify     SpyBiWire        RamEnd
//                Id       |    CpuX     |  FastFlash  |    JTAG     |    RamStart    |   MainStart
//                 |       |      |      |      |      |     |       |        |       |       |
/* F11x(1)(A)*/ { 0xF112, TRUE,  FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xF000 }, // MSP430F1121A
/* F11x2 */     { 0x1132, TRUE,  FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F1132
/* F12x(A) */   { 0xF123, TRUE,  FALSE, FALSE, FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F123
/* F12x2 */     { 0x1232, TRUE,  FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F1232
/* F13x
 * F14x  */     { 0xF149, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F149
/* F15x
 * F16x  */     { 0xF169, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F169
/* F161x */     { 0xF16C, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x1100, 0x24FF, 0x8000 }, // MSP430F1610
/* F20xx */     { 0xF201, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , TRUE  , 0x0200, 0x027F, 0xF800 }, // MSP430F2013
/* F21x1
 * F21x2 */     { 0xF213, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F2131
/* F22x2
 * F22x4 */     { 0xF227, TRUE,  FALSE, TRUE , TRUE,  TRUE,  TRUE , TRUE  , 0x0200, 0x05FF, 0x8000 }, // MSP430F2274
/* F23x0 */     { 0xF237, TRUE,  FALSE, TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x0200, 0x09FF, 0x8000 }, // MSP430F2370
/* F23x
 * F24x
 * F24x1
 * F2410 */     { 0xF249, FALSE, FALSE, TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F249
/* F241x
 * F261x */     { 0xF26F, FALSE, TRUE,  TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x1100, 0x20FF, 0x2100 }, // MSP430F2619
/* F41x */      { 0xF413, FALSE, FALSE, FALSE, FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430F413
/* F42x(x) */   { 0xF427, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }, // MSP430FW427
/* F43x 80p */  { 0xF437, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x05FF, 0xA000 }, // MSP430F437
/* FG43x */     { 0xF439, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430FG439
/* F44x
 * F43x 100p */ { 0xF449, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x09FF, 0x1100 }, // MSP430F449
/* FG461x */    { 0xF46F, FALSE, TRUE,  TRUE , TRUE,  TRUE,  TRUE , FALSE , 0x1100, 0x20FF, 0x2100 }, // MSP430FG4619
/* G2452 */     { 0x2452, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , TRUE  , 0x0200, 0x03FF, 0xC000 }, // MSP430G2452
/* G2xx3 */     { 0x2553, TRUE,  FALSE, TRUE , TRUE,  FALSE, TRUE , TRUE  , 0x0200, 0x02FF, 0xC000 }, // MSP430G2553
/* GENERIC */   { 0xFFFF, FALSE, FALSE, TRUE , FALSE, FALSE, TRUE , FALSE , 0x0200, 0x02FF, 0xE000 }
};

bool DeviceHas_FastFlash(void)
{
    return (sDeviceFeatures[DeviceIdx].FastFlash);
}

bool DeviceHas_CpuX(void)
{
    return (sDeviceFeatures[DeviceIdx].CpuX);
}

bool DeviceHas_EnhVerify(void)
{
    return (sDeviceFeatures[DeviceIdx].EnhVerify);
}

void SetDevice (uint16_t wDeviceId)
{
    for(DeviceIdx = 0; DeviceIdx < (sizeof(sDeviceFeatures)/sizeof(*sDeviceFeatures)); DeviceIdx++)
    {
        if(sDeviceFeatures[DeviceIdx].Id == wDeviceId)
        {
              break;
        }
    }
}


void TMSH_TDIH(void)
{
    SBW_port_enter_critical();
    TMSH 
    TDIH  
    TDOsbw
    SBW_port_exit_critical();
}

void TMSL_TDIH(void)
{
    SBW_port_enter_critical();
    TMSL
    TDIH
    TDOsbw
    SBW_port_exit_critical();
}

void TMSL_TDIH_TDOrd(void)
{
    SBW_port_enter_critical();
    TMSL  TDIH  TDO_RD
    SBW_port_exit_critical();
}

void TMSL_TDIL_TDOrd(void)
{
    SBW_port_enter_critical();
    TMSL  TDIL  TDO_RD
    SBW_port_exit_critical();
}

void TMSH_TDIH_TDOrd(void)
{
    SBW_port_enter_critical();
    TMSH  TDIH  TDO_RD
    SBW_port_exit_critical();
}

void TMSH_TDIL_TDOrd(void)
{
    SBW_port_enter_critical();
    TMSH  TDIL  TDO_RD
    SBW_port_exit_critical();
}

void TMSL_TDIL(void)
{
    SBW_port_enter_critical();
    TMSL  TDIL  TDOsbw
    SBW_port_exit_critical();
}

void TMSH_TDIL(void)
{
    SBW_port_enter_critical();
    TMSH  TDIL  TDOsbw
    SBW_port_exit_critical();
}

void ClrTCLK(void)
{
    if (TCLK_saved)
    {
        TMSLDH
    }
    else
    {
        TMSL
    }

    ClrSBWTDIO();

    TDIL TDOsbw    //ExitTCLK
    TCLK_saved = 0;
}

void SetTCLK(void)
{
   if (TCLK_saved)
   {
        TMSLDH
   }
   else
   {
        TMSL
   }

   SetSBWTDIO();

   TDIH TDOsbw    //ExitTCLK
   TCLK_saved = 1;
}

//*****************************************************************************
//
// Tristate the SBW lines
//
//*****************************************************************************
void IO_3state()
{
    JtagId = 0;
    GPIO_IF_setAsInputPin(TEST_PIN);
    GPIO_IF_setAsInputPin(RESET_PIN);
}

//*****************************************************************************
//
// Drive the SBW signals
//
//*****************************************************************************
void DrvSignals( void )
{
    // tristate SBW lines
    IO_3state();

    // enable SBW lines
    GPIO_IF_setAsOutputPin(TEST_PIN);
    GPIO_IF_setAsOutputPin(RESET_PIN);

    // Set the pin output to high
    GPIO_IF_setOutputHighOnPin(TEST_PIN);
    GPIO_IF_setOutputHighOnPin(RESET_PIN);
}

//*****************************************************************************
//
// Initialize delay timer
//
//*****************************************************************************
void delayInit()
{
}

//*****************************************************************************
//
// Reset SBW TAP controller
//
//*****************************************************************************
void ResetTAP(void)
{
    uint32_t i;
    // Now fuse is checked, Reset JTAG FSM
    for (i = 6; i > 0; i--)      // 6 is nominal
    {
        TMSH_TDIH();
    }
    // JTAG FSM is now in Test-Logic-Reset
    TMSL_TDIH();                 // now in Run/Test Idle

    // Fuse check
    TMSH_TDIH();
    TMSL_TDIH();
    TMSH_TDIH();
    TMSL_TDIH();
    TMSH_TDIH();
    // In every TDI slot a TCK for the JTAG machine is generated.
    // Thus we need to get TAP in Run/Test Idle state back again.
    TMSH_TDIH();
    TMSL_TDIH();                // now in Run/Test Idle
}

//*****************************************************************************
//
// Shift bits
//
//*****************************************************************************
uint32_t Shift( uint32_t Format, uint32_t Data)
{
   uint32_t TDOword = 0x0000;
   uint32_t MSB = 0x0000;
   uint32_t i;

   (Format == F_WORD) ? (MSB = 0x8000) : (MSB = 0x80);
   for (i = Format; i > 0; i--)
   {
        if (i == 1)                     // last bit requires TMS=1; TDO one bit before TDI
        {
          ((Data & MSB) == 0) ? TMSH_TDIL_TDOrd() : TMSH_TDIH_TDOrd();
        }
        else
        {
          ((Data & MSB) == 0) ? TMSL_TDIL_TDOrd() : TMSL_TDIH_TDOrd();
        }
        Data <<= 1;
        if (tdo_bit)
            TDOword++;
        if (i > 1)
            TDOword <<= 1;               // TDO could be any port pin
   }

   TMSH_TDIH();                         // update IR
   if (TCLK_saved)
   {
        TMSL_TDIH();
   }
   else
   {
        TMSL_TDIL();
   }

   return(TDOword);
}

//*****************************************************************************
//
// IR scan
//
//*****************************************************************************
uint32_t IR_Shift(uint8_t instruction)
{
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSH_TDIH();

    // JTAG FSM state = Select IR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-IR
    TMSL_TDIH();
    // JTAG FSM state = Shift-IR, Shift in TDI (8-bit)
    return(Shift(F_BYTE, instruction));
    // JTAG FSM state = Run-Test/Idle
}

//*****************************************************************************
//
// 16 bit DR scan
//
//*****************************************************************************
uint16_t DR_Shift16(uint16_t data)
{
  
    // JTAG FSM state = Run-Test/Idle
    if (TCLK_saved)
    {
        TMSH_TDIH();
    }
    else
    {
        TMSH_TDIL();
    }
    // JTAG FSM state = Select DR-Scan
    TMSL_TDIH();
    // JTAG FSM state = Capture-DR
    TMSL_TDIH();    

    // JTAG FSM state = Shift-DR, Shift in TDI (16-bit)
    return(Shift(F_WORD, data));
    // JTAG FSM state = Run-Test/Idle
}

//*****************************************************************************
//
// Start SBW and get Jtag Id
//
//*****************************************************************************
void StartSBW(void)
{
   // drive JTAG/TEST signals
    {
      DrvSignals();
      MsDelay(10);             // delay 10ms
    }
    // added by FB  
    ClrTST();
    usDelay(800);              // delay min 800us - clr SBW controller
    SetTST();
    usDelay(50); 

    // SpyBiWire entry sequence      
    // Reset Test logic
    ClrSBWTDIO();                   // put device in normal operation: Reset = 0
    ClrSBWTCK();                    // TEST pin = 0
    MsDelay(1);                     // wait 1ms (minimum: 100us)

    // SpyBiWire entry sequence
    SetSBWTDIO();                   // Reset = 1
    SetSBWTCK();                    // TEST pin = 1
                                    // initial 1 SBWCLKs to enter sbw-mode
    ClrSBWTCK();
    SetSBWTCK();

    // SpyBiWire mode is active now
    ResetTAP();  // reset TAP state machine -> Run-Test/Idle

    //TRIGGER;
    JtagId = (uint8_t)IR_Shift(IR_BYPASS);
}

//*****************************************************************************
//
// Issue SBW scan
//
//*****************************************************************************
uint16_t SetInstrFetch(void)
{
    uint16_t i;

    IR_Shift(IR_CNTRL_SIG_CAPTURE);

    // Wait until CPU is in instr. fetch state, timeout after limited attempts
    for (i = 50; i > 0; i--)
    {
        if (DR_Shift16(0x0000) & 0x0080)
        {
            return SC_ERR_NONE;
        }
        ClrTCLK();
        SetTCLK();
    }
    return SC_ERR_ET_DCDC_CPU_STATE;
}

//*****************************************************************************
//
// Halt CPU
//
//*****************************************************************************
void HaltCPU(void)
{
    SetInstrFetch();              // Set CPU into instruction fetch mode

    IR_Shift(IR_DATA_16BIT);
    DR_Shift16(0x3FFF);           // Send JMP $ instruction
    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2409);           // Set JTAG_HALT bit
    SetTCLK();
}

//*****************************************************************************
//
// Strobes the TCLK
//
//*****************************************************************************
void TCLKstrobes(uint16_t Amount) // enters with TCLK_saved and exits with TCLK = 1
{
   uint16_t i;

   if (TCLK_saved)
   {
        TMSLDH
   }                         // TDI = 1 with rising sbwclk
   else
   {
        TMSL
   }

    // This implementation has 30 body cycles! -> 400kHz
    // DO NOT MODIFY IT !

   for (i = Amount; i > 0; i--)
   {
        ClrSBWTDIO();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();

        SetSBWTDIO();

        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
        _NOP();
   }
   TDIH TDOsbw    //ExitTCLK
   TCLK_saved = 1;
}

//*****************************************************************************
//
// Release CPU from halted state
//
//*****************************************************************************
void ReleaseCPU(void)
{
    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2401);           // Clear the HALT_JTAG bit
    IR_Shift(IR_ADDR_CAPTURE);
    SetTCLK();
}

//*****************************************************************************
//
// Write blocks to flash
//
//*****************************************************************************
void WriteFLASH(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray)
{
    uint16_t i;                     // Loop counter
    uint16_t addr = StartAddr;      // Address counter
    uint16_t FCTL3_val = 0xA500;    // ok for all devices; if Info-Seg. A on F2xxx should not be programmed

    HaltCPU();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2408);         // Set RW to write
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x0128);         // FCTL1 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA540);         // Enable FLASH write
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012A);         // FCTL2 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA540);         // Select MCLK as source, DIV=1
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Clear FCTL3; F2xxx: Unlock Info-Seg.
                                // A by toggling LOCKA-Bit if required,
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);

    for (i = 0; i < Length; i++, addr += 2)
    {
        DR_Shift16(0x2408);             // Set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(addr);               // Set address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16((uint16_t)DataArray[i]);       // Set data
        SetTCLK();
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2409);             // Set RW to read

        TCLKstrobes(35);        // Provide TCLKs, min. 33 for F149 and F449
                                // F2xxx: 29 are ok
    }

    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2408);         // Set RW to write
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x0128);         // FCTL1 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA500);         // Disable FLASH write
    SetTCLK();

    // set LOCK-Bits again
    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 address
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Lock Inf-Seg. A by toggling LOCKA and set LOCK again
    SetTCLK();

    ReleaseCPU();
}


//*****************************************************************************
//
// Write blocks to flash INFO A section
//
//*****************************************************************************

void WriteFLASH_infoA(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray)
{
    uint16_t i;                     // Loop counter
    uint16_t addr = StartAddr;      // Address counter
    uint16_t FCTL3_val = 0xA540;    // only if Info-Seg. A on F2xxx should be programmed

    HaltCPU();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2408);         // Set RW to write
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x0128);         // FCTL1 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA540);         // Enable FLASH write
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012A);         // FCTL2 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA540);         // Select MCLK as source, DIV=1
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Clear FCTL3; F2xxx: Unlock Info-Seg.
                                // A by toggling LOCKA-Bit if required,
    SetTCLK();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);

    for (i = 0; i < Length; i++, addr += 2)
    {
        DR_Shift16(0x2408);             // Set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(addr);               // Set address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16((uint16_t)DataArray[i]);       // Set data
        SetTCLK();
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2409);             // Set RW to read

        TCLKstrobes(35);        // Provide TCLKs, min. 33 for F149 and F449
                                // F2xxx: 29 are ok
    }

    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2408);         // Set RW to write
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x0128);         // FCTL1 register
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(0xA500);         // Disable FLASH write
    SetTCLK();

    // set LOCK-Bits again
    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 address
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Lock Inf-Seg. A by toggling LOCKA and set LOCK again
    SetTCLK();

    ReleaseCPU();
}

//*****************************************************************************
//
// Stop SBW
//
//*****************************************************************************

void StopSBW(void)
{
    // release JTAG/TEST signals
    IO_3state();
    MsDelay(10);             // delay 10ms
}

//*****************************************************************************
//
// Read CPU memory
//
//*****************************************************************************
uint16_t ReadMem(uint16_t Format, uint16_t Addr)
{
    uint16_t TDOword;

    HaltCPU();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    if  (Format == F_WORD)
    {
        DR_Shift16(0x2409);         // Set word read
    }
    else
    {
        DR_Shift16(0x2419);         // Set byte read
    }
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(Addr);               // Set address
    IR_Shift(IR_DATA_TO_ADDR);
    SetTCLK();

    ClrTCLK();
    TDOword = DR_Shift16(0x0000);   // Shift out 16 bits

    ReleaseCPU();
    return(Format == F_WORD ? TDOword : TDOword & 0x00FF);
}

//*****************************************************************************
//
// Write CPU memory
//
//*****************************************************************************
void WriteMem(uint16_t Format, uint16_t Addr, uint16_t Data)
{
    HaltCPU();

    ClrTCLK();
    IR_Shift(IR_CNTRL_SIG_16BIT);
    if  (Format == F_WORD)
    {
        DR_Shift16(0x2408);     // Set word write
    }
    else
    {
        DR_Shift16(0x2418);     // Set byte write
    }
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(Addr);           // Set addr
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(Data);           // Shift in 16 bits
    SetTCLK();

    ReleaseCPU();
}

//*****************************************************************************
//
// Execute Power On Reset
//
//*****************************************************************************
int ExecutePOR(void)
{
    uint16_t JtagVersion;

    // Perform Reset
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2C01);                 // Apply Reset
    DR_Shift16(0x2401);                 // Remove Reset

    ClrTCLK();
    SetTCLK();
    ClrTCLK();
    SetTCLK();
    ClrTCLK();
    JtagVersion = IR_Shift(IR_ADDR_CAPTURE); // read JTAG ID, checked at function end
    SetTCLK();

    WriteMem(F_WORD, 0x0120, 0x5A80);   // Disable Watchdog on target device

    if (JtagVersion != JTAG_ID)
    {
        return SC_ERR_ET_DCDC_IDCODE;
    }

    return SC_ERR_NONE;
}

//*****************************************************************************
//
// Get Device
// Start SBW, get the device id, synchronize CPU, execute POR
//
//*****************************************************************************
int GetDevice(void)
{
    uint16_t i;

    for (i = 0; i < MAX_ENTRY_TRY; i++)
    {
      JtagId = 0;    // initialize JtagId with an invalid value
      StopSBW();    // release JTAG/TEST signals
      StartSBW();   // establish the physical connection to the JTAG interface
      if(JtagId == JTAG_ID) // break if a valid JTAG ID is being returned
      {
          break;
      }
    }


    if(i >= MAX_ENTRY_TRY)
    {
      return SC_ERR_ET_DCDC_IDCODE;
    }

    ResetTAP();                          // Reset JTAG state machine, check fuse HW

    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2401);                  // Set device into JTAG mode + read
    int ir = IR_Shift(IR_CNTRL_SIG_CAPTURE);
    if (ir != JTAG_ID)
    {
        return SC_ERR_ET_DCDC_IDCODE;
    }

    // Wait until CPU is synchronized, timeout after a limited # of attempts
    for (i = 50; i > 0; i--)
    {
        if (DR_Shift16(0x0000) & 0x0200)
        {
            uint16_t DeviceId;
            DeviceId = ReadMem(F_WORD, 0x0FF0);// Get target device type
                                               //(bytes are interchanged)
            DeviceId = (DeviceId << 8) + (DeviceId >> 8); // swop bytes
            SetDevice(DeviceId);
            break;
        }
        else
        {
            if (i == 1)
            {
                return SC_ERR_ET_DCDC_DEVID;      // Timeout reached, return false
            }
        }
    }
    return ExecutePOR();                     // Perform PUC, Includes
}

//*****************************************************************************
//
// Set Program Counter
// Sets the program counter to the specified address
//
//*****************************************************************************
void SetPC(uint16_t Addr)
{
    SetInstrFetch();              // Set CPU into instruction fetch mode, TCLK=1

    // Load PC with address
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x3401);           // CPU has control of RW & BYTE.
    IR_Shift(IR_DATA_16BIT);
    DR_Shift16(0x4030);           // "mov #addr,PC" instruction
    ClrTCLK();
    SetTCLK();                    // F2xxx
    DR_Shift16(Addr);             // "mov #addr,PC" instruction
    ClrTCLK();
    IR_Shift(IR_ADDR_CAPTURE);
    SetTCLK();
    ClrTCLK();                    // Now the PC should be on Addr
    IR_Shift(IR_CNTRL_SIG_16BIT);
    DR_Shift16(0x2401);           // JTAG has control of RW & BYTE.
}

//*****************************************************************************
//
// Erase Flash
//
//*****************************************************************************
void EraseFLASH(uint16_t EraseMode, uint16_t EraseAddr)
{
    uint16_t StrobeAmount = 4820;       // default for Segment Erase
    uint16_t i, loopcount = 1;          // erase cycle repeating for Mass Erase
    uint16_t FCTL3_val = 0xA500;        // ok for all devices; if Info-Seg. A on F2xxx should not be erased

    if ((EraseMode == ERASE_MASS) || (EraseMode == ERASE_MAIN))
    {
        if(DeviceHas_FastFlash())
        {
            StrobeAmount = 10600;        // Larger Flash memories require
        }
        else
        {
            StrobeAmount = 5300;        // Larger Flash memories require
            loopcount = 19;             // additional cycles for erase.
        }
    }
    HaltCPU();

    for (i = loopcount; i > 0; i--)
    {
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2408);         // set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x0128);         // FCTL1 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(EraseMode);      // Enable erase mode
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x012A);         // FCTL2 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0xA540);         // MCLK is source, DIV=1
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x012C);         // FCTL3 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(FCTL3_val);      // Clear FCTL3; F2xxx: Unlock Info-Seg. A by toggling LOCKA-Bit if required,
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(EraseAddr);      // Set erase address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0x55AA);         // Dummy write to start erase
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2409);         // Set RW to read
        TCLKstrobes(StrobeAmount);  // Provide TCLKs
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2408);         // Set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x0128);         // FCTL1 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0xA500);         // Disable erase
        SetTCLK();
    }
    // set LOCK-Bits again
    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 address
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Lock Inf-Seg. A by toggling LOCKA (F2xxx) and set LOCK again
    SetTCLK();

    ReleaseCPU();
}

//*****************************************************************************
//
// Erase Flash infoA memory
//
//*****************************************************************************
void EraseFLASH_infoA(uint16_t EraseMode, uint16_t EraseAddr)
{
    uint16_t StrobeAmount = 4820;       // default for Segment Erase
    uint16_t i, loopcount = 1;          // erase cycle repeating for Mass Erase
    uint16_t FCTL3_val = 0xA540;        // only if Info-Seg. A on F2xxx should be erased

    if ((EraseMode == ERASE_MASS) || (EraseMode == ERASE_MAIN))
    {
        if(DeviceHas_FastFlash())
        {
            StrobeAmount = 10600;        // Larger Flash memories require
        }
        else
        {
            StrobeAmount = 5300;        // Larger Flash memories require
            loopcount = 19;             // additional cycles for erase.
        }
    }
    HaltCPU();

    for (i = loopcount; i > 0; i--)
    {
        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2408);         // set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x0128);         // FCTL1 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(EraseMode);      // Enable erase mode
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x012A);         // FCTL2 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0xA540);         // MCLK is source, DIV=1
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x012C);         // FCTL3 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(FCTL3_val);      // Clear FCTL3; F2xxx: Unlock Info-Seg. A by toggling LOCKA-Bit if required,
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(EraseAddr);      // Set erase address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0x55AA);         // Dummy write to start erase
        SetTCLK();

        ClrTCLK();
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2409);         // Set RW to read
        TCLKstrobes(StrobeAmount);  // Provide TCLKs
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2408);         // Set RW to write
        IR_Shift(IR_ADDR_16BIT);
        DR_Shift16(0x0128);         // FCTL1 address
        IR_Shift(IR_DATA_TO_ADDR);
        DR_Shift16(0xA500);         // Disable erase
        SetTCLK();
    }
    // set LOCK-Bits again
    ClrTCLK();
    IR_Shift(IR_ADDR_16BIT);
    DR_Shift16(0x012C);         // FCTL3 address
    IR_Shift(IR_DATA_TO_ADDR);
    DR_Shift16(FCTL3_val);      // Lock Inf-Seg. A by toggling LOCKA (F2xxx) and set LOCK again
    SetTCLK();

    ReleaseCPU();
}

//*****************************************************************************
//
// Verify flash
//
//*****************************************************************************
int VerifyPSA(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray)
{
    uint16_t TDOword, i;
    uint16_t POLY = 0x0805;           // Polynom value for PSA calculation
    uint16_t PSA_CRC = StartAddr-2;   // Start value for PSA calculation

    ExecutePOR();

    if(DeviceHas_EnhVerify())
    {
        SetPC(StartAddr-4);
        HaltCPU();
        ClrTCLK();
        IR_Shift(IR_DATA_16BIT);
        DR_Shift16(StartAddr-2);
    }
    else
    {
        SetPC(StartAddr-2);
        SetTCLK();
        ClrTCLK();
    }
    IR_Shift(IR_DATA_PSA);

    for (i = 0; i < Length; i++)
    {
        // Calculate the PSA (Pseudo Signature Analysis) value
        if ((PSA_CRC & 0x8000) == 0x8000)
        {
            PSA_CRC ^= POLY;
            PSA_CRC <<= 1;
            PSA_CRC |= 0x0001;
        }
        else
        {
            PSA_CRC <<= 1;
        }
        // if pointer is 0 then use erase check mask, otherwise data
        if (DataArray == 0)
        {
            PSA_CRC ^= 0xFFFF;
        }
        else
        {
            PSA_CRC ^= DataArray[i];
        }
        // Clock through the PSA
        SetTCLK();

        TMSH_TDIH();
        TMSL_TDIH();
        TMSL_TDIH();
        TMSH_TDIH();
        TMSH_TDIH();
        TMSL_TDIH();

        ClrTCLK();
    }
    IR_Shift(IR_SHIFT_OUT_PSA);
    TDOword = DR_Shift16(0x0000);   // Read out the PSA value
    SetTCLK();

    if(DeviceHas_EnhVerify())
    {
       ReleaseCPU();
    }

    ExecutePOR();

    return((TDOword == PSA_CRC) ? 0 : -1);
}

//*****************************************************************************
//
// Perform blank check
//
//*****************************************************************************
int EraseCheck(uint16_t StartAddr, uint16_t Length)
{
    return (VerifyPSA(StartAddr, Length, 0));
}

//*****************************************************************************
//
// Release device from JTAG, set PC to reset vector
//
//*****************************************************************************
int ReleaseDevice(uint16_t Addr)
{
    if (Addr == V_RESET)
    {
        IR_Shift(IR_CNTRL_SIG_16BIT);
        DR_Shift16(0x2C01);         // Perform a reset
        DR_Shift16(0x2401);
    }
    else
    {
        SetPC(Addr);                // Set target CPU's PC
    }
    JtagId = IR_Shift(IR_CNTRL_SIG_RELEASE);

    if (JtagId != JTAG_ID)
    {
        return SC_ERR_ET_DCDC_IDCODE;
    }

    return SC_ERR_NONE;
}

//*****************************************************************************
//
// Verify memory using PSA
//
//*****************************************************************************
int VerifyMem(uint16_t StartAddr, uint16_t Length, const uint16_t *DataArray)
{
    return (VerifyPSA(StartAddr, Length, DataArray));
}

//*****************************************************************************
//
// Program MSP430
//
//*****************************************************************************
int Program_MSP(void)
{
    int error = 0;
    uint32_t check = 0;
    uint16_t attempt = 0;
    uint16_t loopi = 0;
    bool eraseCompleted = false;
    bool writeCompleted = false;
    uint16_t * epromStartOfData = 0;
    unsigned int section = 0;

    for (section = 0; section < eprom_sections; section++)
    {
        Debug_UART_printStringWithNumber("\r\n\r\n****Section: %d****\r\n\r\n", section);
        Debug_UART_printStringWithNumber("Address: 0x%X\r\n", eprom_address[section]);
        writeCompleted = false;
        if (section == 0)
        {
            epromStartOfData = eprom;
        }
        else
        {
            epromStartOfData = epromStartOfData + eprom_length_of_sections[section - 1];
        }

        for (attempt = 0; attempt < 100; attempt++)
        {
#if DEBUG
            Debug_UART_printStringWithNumber("Attempt: %d\r\n", attempt);
#endif
            //Erase
            if (!eraseCompleted)
            {
                error = GetDevice();
                RETRY_ON_ERROR(error);

                check = IR_Shift(IR_BYPASS);
#if DEBUG
                Debug_UART_printStringWithNumber("IR SHIFT: %x\r\n", check);
#endif
                EraseFLASH(ERASE_MAIN, eprom_address[section]);
                check = IR_Shift(IR_BYPASS);
#if DEBUG
                Debug_UART_printStringWithNumber("IR SHIFT after erase: %x\r\n", check);

                for (loopi = 0; loopi < eprom_length_of_sections[section]; loopi++)
                {
                    Debug_UART_printStringWithNumber("0x%X, ", ReadMem(F_WORD, eprom_address[section] + 2*loopi));
                }
                Debug_UART_printString("\r\n");
#endif

                error = EraseCheck(eprom_address[section], eprom_length_of_sections[section]);
                RETRY_ON_ERROR(error);
#if DEBUG
                Debug_UART_printStringWithNumber("EraseCheck: %d\r\n", error);
#endif
                error = ReleaseDevice(V_RESET);
#if DEBUG
                Debug_UART_printStringWithNumber("Release Device: %d\r\n", error);// Perform Reset, release CPU from JTAG control
#endif
                IO_3state();
                RETRY_ON_ERROR(error);
                eraseCompleted = true;
            }

            //Write Program
            if (!writeCompleted)
            {
                error = GetDevice();
                RETRY_ON_ERROR(error);
                check = IR_Shift(IR_BYPASS);
#if DEBUG
                Debug_UART_printStringWithNumber("IR SHIFT: %x\r\n", check);
#endif
                WriteFLASH(eprom_address[section], eprom_length_of_sections[section], epromStartOfData);
#if DEBUG
                for (loopi = 0; loopi < eprom_length_of_sections[section]; loopi++)
                {
                    Debug_UART_printStringWithNumber("0x%X, ", ReadMem(F_WORD, eprom_address[section] + 2*loopi));
                }
                Debug_UART_printString("\r\n");
#endif
                error = VerifyMem(eprom_address[section], eprom_length_of_sections[section], epromStartOfData);
                RETRY_ON_ERROR(error);
                error = ReleaseDevice(V_RESET);
#if DEBUG
                Debug_UART_printStringWithNumber("Release Device: %d\r\n", error);
#endif
                IO_3state();
                RETRY_ON_ERROR(error);
                writeCompleted = true;
                break;
            }
        }

        if (attempt >= 100)
        {
            if (!eraseCompleted)
            {
                Debug_UART_printString("Erase was not successful\r\n");
                return SC_ERR_ET_DCDC_FLASH_VERI;
            }
            if (!writeCompleted)
            {
                Debug_UART_printString("Write was not successful\r\n");
                return SC_ERR_ET_DCDC_FLASH_VERI;
            }
        }
        else
        {
            Debug_UART_printStringWithNumber("Completed on attempt: %d\r\n", attempt);
        }
    }

    return SC_ERR_NONE;
}
