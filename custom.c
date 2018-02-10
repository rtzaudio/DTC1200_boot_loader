//*****************************************************************************
//
// DTC-1200 Digital Transport Controller Boot Loader for Ampex MM-1200
//
// Copyright (C) 2016-2018, RTZ Professional Audio, LLC
// All Rights Reserved
//
// RTZ is registered trademark of RTZ Professional Audio, LLC
//
//*****************************************************************************
//
// bl_main.c - The file holds the main control loop of the boot loader.
//
// Copyright (c) 2006-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_flash.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
//#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "bl_config.h"
#include "boot_loader/bl_commands.h"
#include "boot_loader/bl_decrypt.h"
#include "boot_loader/bl_flash.h"
#include "boot_loader/bl_hooks.h"
#include "boot_loader/bl_i2c.h"
#include "boot_loader/bl_packet.h"
#include "boot_loader/bl_ssi.h"
#include "boot_loader/bl_uart.h"
#ifdef CHECK_CRC
#include "boot_loader/bl_crc32.h"
#endif

/*** Static Function Prototypes ***/

void MotorDAC_init(void);
void MotorDAC_write(uint32_t supply, uint32_t takeup);

void U5_MCP23S17_init(void);
void U5_MCP23S17_write(uint32_t uRegAddr, uint32_t uData);

void U8_MCP23S17_init(void);
void U8_MCP23S17_write(uint32_t uRegAddr, uint32_t uData);

/*** Global Data ***/

static uint32_t lampmask;

//*****************************************************************************
//
// Performs application-specific low level hardware initialization on system
// reset.
//
// If hooked, this function will be called immediately after the boot loader
// code relocation completes.  An application may perform any required low
// hardware initialization during this function.  Note that the system clock
// has not been set when this function is called.  Initialization that assumes
// the system clock is set may be performed in the BL_INIT_FN_HOOK function
// instead.
//
// void MyHwInitFunc(void);
//
//*****************************************************************************

void MyHwInitFunc(void)
{
    //ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

//*****************************************************************************
//
// Performs application-specific initialization on system reset.
//
// This function will be called immediately after the boot loader sets the 
// system clock. Here we perform hardware initialization specific to the 
// DTC-1200 digital transport controller and the MM-1200 hardware.
//
// void MyInitFunc(void);
//
//*****************************************************************************

void MyInitFunc(void)
{
    lampmask = L_STAT1|L_STAT2|L_STAT3;

    // Initialize DAC U13 (SSI0)
    MotorDAC_init();
    // Set the motor DAC's to zero torque at power-up!
    MotorDAC_write(0, 0);
    
    // Initialize I/O Expander U5 (SSI1)
    U5_MCP23S17_init();
    // Set the LED states
    U5_MCP23S17_write(MCP_GPIOB, L_STAT1|L_STAT2|L_STAT3);
    
    // Initialize I/O Expander U8 (SSI2)
    U8_MCP23S17_init();
    // Engage the brakes
    U8_MCP23S17_write(MCP_GPIOA, T_BRAKE);        
}

//*****************************************************************************
//
// Informs an application that a download is starting.
//
// If hooked, this function will be called when a new firmware download is
// about to start.  The application may use this signal to initialize any
// progress display.
//
// void MyStartFunc(void);
//
//*****************************************************************************

void MyStartFunc(void)
{
    lampmask = L_STAT1|L_STAT2;
    U5_MCP23S17_write(MCP_GPIOB, lampmask);
}

//*****************************************************************************
//
// Informs an application of download progress.
//
// If hooked, this function will be called periodically during firmware
// download.  The application may use this to update its user interface.
// When using a protocol which does not inform the client of the final size of
// the download in advance (e.g. TFTP), the ulTotal parameter will be 0,
// otherwise it indicates the expected size of the complete download.
//
// void MyProgressFunc(uint32_t ulCompleted, uint32_t ulTotal);
//
// where:
//
// - ulCompleted indicates the number of bytes already downloaded.
// - ulTotal indicates the number of bytes expected or 0 if this is not known.
//
//*****************************************************************************

void MyProgressFunc(uint32_t ulCompleted, uint32_t ulTotal)
{
    // Toggle status-1 led
    lampmask ^= L_STAT2;
    U5_MCP23S17_write(MCP_GPIOB, lampmask);
}

//*****************************************************************************
//
// Informs an application that a download has completed.
//
// If hooked, this function will be called when a firmware download ends.
// The application may use this signal to update its user interface.  Typically
// a system reset will occur shortly after this function returns as the boot
// loader attempts to boot the new image.
//
// void MyEndFunc(void);
//
//*****************************************************************************

void MyEndFunc(void)
{
    // Status 3 led indicates download complete
    lampmask = L_STAT3;
    U5_MCP23S17_write(MCP_GPIOB, lampmask);
}

//*****************************************************************************
// THE FOLLOWING FUNCTIONS ARE SPECIFIC TO THE DTC-1200 CONTROLLER HARDWARE.
//*****************************************************************************

void MotorDAC_init(void)
{
    // Enable the SSI0 and GPIO-A peripherals.
    
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.

    ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
    ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin. The pins are as follows:
    //
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK

    ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.

    ROM_SSIConfigSetExpClk(
            SSI0_BASE,
            ROM_SysCtlClockGet(),
            SSI_FRF_MOTO_MODE_2,
            SSI_MODE_MASTER,
            1000000,
            16);

    // Enable the SSI0 module.

    ROM_SSIEnable(SSI0_BASE);

    // Read any residual data from the SSI port.

    uint32_t ui32DataRx[4];

    while(ROM_SSIDataGetNonBlocking(SSI0_BASE, &ui32DataRx[0]))
    {
    }
}

//*****************************************************************************
// This function writes the takeup and supply motor DAC values controlling
// the motor drive amp. The TLV5637 is a dual 10-bit, single supply DAC,
// based on a resistor string architecture. The output voltage (full scale
// determined by reference) is given by:
//
//      Vout = (2 REF) * (CODE/0x1000)
//
// Where REF is the reference voltage and CODE is the digital
// input value in the range 0x000 to 0xFFF. Because it is a
// 10-bit DAC, only D11 to D2 are used. D0 and D1 are ignored.
// A power-on reset initially puts the internal latches to a
// defined state (all bits zero).
//
// The motor current amp delivers full torque at 1mA and
// zero torque at 5.1mA.
//
//      DAC A - is the SUPPLY motor torque level
//      DAC B - is the TAKEUP motor torque level
//
//*****************************************************************************

void MotorDAC_write(uint32_t supply, uint32_t takeup)
{
    uint32_t ulWord;
    uint32_t ulDac;

    takeup = DAC_MAX - takeup;
    supply = DAC_MAX - supply;

    // (1) Set reference voltage to 1.024 V (CONTROL register)
    ulWord = (1 << 15) | (1 << 12) | 0x01;
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    ROM_SSIDataPut(SSI0_BASE, ulWord);
    while(ROM_SSIBusy(SSI0_BASE));
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // (2) Write data for DAC B to BUFFER
    ulDac  = (takeup & 0x3FF) << 2;
    ulWord = (1 << 12) | (uint16_t)ulDac;
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    ROM_SSIDataPut(SSI0_BASE, ulWord);
    while(ROM_SSIBusy(SSI0_BASE));
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // (3) Write DAC A value and update DAC A & B simultaneously
    ulDac  = (supply & 0x3FF) << 2;
    ulWord = (1 << 15) | (uint16_t)ulDac;
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    ROM_SSIDataPut(SSI0_BASE, ulWord);
    while(ROM_SSIBusy(SSI0_BASE));
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

//*****************************************************************************
//
// MCP23S17 I/O Expander
//
// U5 (SSI1) : MCP23S17SO TRANSPORT SWITCHES & LAMPS
//
//*****************************************************************************

void U5_MCP23S17_init(void)
{
    // Enable the SSI1 and GPIO-D peripherals.
    
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        
    // We're controlling the slave select manually on PD1.
    // Configure pin PD1 for standard GPIO use.

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);

    // Configure the pin muxing for SSI1 functions on port D0, D1, D2, and D3.
    // This step is not necessary if your part does not support pin muxing.

    ROM_GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    ROM_GPIOPinConfigure(GPIO_PD2_SSI1RX);
    ROM_GPIOPinConfigure(GPIO_PD3_SSI1TX);

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    //
    //      PD0 - SSI1CLK
    //      PD1 - SSI1Fss
    //      PD2 - SSI1Rx
    //      PD3 - SSI1Tx
    //

    ROM_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.

    ROM_SSIConfigSetExpClk(
            SSI1_BASE,
            ROM_SysCtlClockGet(),
            SSI_FRF_MOTO_MODE_0,
            SSI_MODE_MASTER,
            1000000,
            8);

    // Enable the SSI1 module.

    ROM_SSIEnable(SSI1_BASE);

    // Read any residual data from the SSI port.

    uint32_t ui32DataRx[4];
    while(ROM_SSIDataGetNonBlocking(SSI1_BASE, &ui32DataRx[0]))
    {
    }

    // U5 (SSI1) : MCP23S17SO TRANSPORT SWITCHES & LAMPS
    //
    // Configure the I/O Expander

    // Configure for byte mode, INT active low
    U5_MCP23S17_write(MCP_IOCONA, C_SEQOP | C_ODR);

    // Configure for byte mode, INT active low
    U5_MCP23S17_write(MCP_IOCONB, C_SEQOP);

    // Port A - all inputs from transport switches
    U5_MCP23S17_write(MCP_IODIRA, 0xFF);

    // Port B - all outputs to lamp/led drivers
    U5_MCP23S17_write(MCP_IODIRB, 0x00);

    // Invert input polarity of tape-out switch
    U5_MCP23S17_write(MCP_IOPOLA, 0x40);
}

//*****************************************************************************
// Set lamp & led indicators.
//
// The following bit masks are valid:
//
// L_FWD   - forward indicator lamp
// L_REW   - rewind indicator lamp
// L_PLAY  - play indicator lamp
// L_REC   - record indicator lamp
// L_STOP  - stop indicator lamp
// L_LED3  - diagnostic led3
// L_LED2  - diagnostic led2
// L_LED1  - diagnostic led1
//
//*****************************************************************************

void U5_MCP23S17_write(uint32_t uRegAddr, uint32_t uData)
{
    // Select SPI chip select low PD1
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);

    // 1) Write the op-code
    ROM_SSIDataPut(SSI1_BASE, 0x40);
    while(ROM_SSIBusy(SSI1_BASE));

    // 2) Write the register address
    ROM_SSIDataPut(SSI1_BASE, uRegAddr);
    while(ROM_SSIBusy(SSI1_BASE));

    // 2) Write the register address
    ROM_SSIDataPut(SSI1_BASE, uData);
    while(ROM_SSIBusy(SSI1_BASE));

    // Release SPI chip select high PD1
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

//*****************************************************************************
//
// MCP23S17 I/O Expander
//
// U8 (SSI2) : MCP23S17SO SOLENOID, CONFIG DIP SWITCH & TAPE SPEED
//
//*****************************************************************************
    
void U8_MCP23S17_init(void)
{
    // Enable the SSI0 and GPIO-B peripherals.  

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    // We're controlling the slave select manually on PB5.
    // Configure pin PB5 for standard GPIO use.

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);

    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.

    ROM_GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    //ROM_GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    ROM_GPIOPinConfigure(GPIO_PB6_SSI2RX);
    ROM_GPIOPinConfigure(GPIO_PB7_SSI2TX);

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin. The pins are as follows:
    //
    //      PB4 - SSI2CLK
    //      PB5 - SSI0Fss
    //      PB6 - SSI0Rx
    //      PB7 - SSI0Tx

    ROM_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

    // Configure and enable the SSI port for SPI master mode.  Use SSI2,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.

    ROM_SSIConfigSetExpClk(
            SSI2_BASE,
            ROM_SysCtlClockGet(),
            SSI_FRF_MOTO_MODE_0,
            SSI_MODE_MASTER,
            1000000,
            8);

    // Enable the SSI0 module.

    ROM_SSIEnable(SSI2_BASE);

    // Read any residual data from the SSI port.

    uint32_t ui32DataRx[4];
    while(ROM_SSIDataGetNonBlocking(SSI2_BASE, &ui32DataRx[0]))
    {
    }

    // U8 (SSI2) : MCP23S17SO SOLENOID, CONFIG DIP SWITCH & TAPE SPEED
    //
    // Configure the I/O Expander

    // Configure for byte mode, INT active low
    U8_MCP23S17_write(MCP_IOCONA, C_SEQOP);

    // Configure for byte mode, INT active low
    U8_MCP23S17_write(MCP_IOCONB, C_SEQOP | C_ODR);

    // Port A - solenoid and other drivers, all outputs
    U8_MCP23S17_write(MCP_IODIRA, 0);

    // Port B - DIP switches and tape-speed switch, all inputs
    U8_MCP23S17_write(MCP_IODIRB, 0xFF);

    //Invert input polarity of DIP switches and tape-speed switch
    U8_MCP23S17_write(MCP_IOPOLB, 0x8F);
}

//*****************************************************************************
// Set transport solenoids and/or record pulse/hold output drivers.
//
// The following bit masks are valid:
//
// T_SERVO     - capstan servo enable
// T_BRAKE     - engage brakes (release solenoid)
// T_TLIFT     - engage tape lifter solenoid
// T_PROL      - engage pinch roller solenoid
// T_RECP      - record pulse toggle bit
// T_RECH      - record hold bit
//
//*****************************************************************************

void U8_MCP23S17_write(uint32_t uRegAddr, uint32_t uData)
{
    // Select SPI chip select low PB5
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);

    // 1) Write the op-code
    ROM_SSIDataPut(SSI2_BASE, 0x40);
    while(ROM_SSIBusy(SSI2_BASE));

    // 2) Write the register address
    ROM_SSIDataPut(SSI2_BASE, uRegAddr);
    while(ROM_SSIBusy(SSI2_BASE));

    // 2) Write the register address
    ROM_SSIDataPut(SSI2_BASE, uData);
    while(ROM_SSIBusy(SSI2_BASE));

    // Release SPI chip select high PB5
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
}

// END-OF-FILE
