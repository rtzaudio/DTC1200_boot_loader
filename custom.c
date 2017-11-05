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
#include "inc/hw_nvic.h"
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
#define DAC_MAX         0x03FF      /* 10-bit full scale DAC   */


void MotorDAC_init(void);
void MotorDAC_write(unsigned supply, unsigned takeup);

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

}

//*****************************************************************************
//
// Performs application-specific initialization on system reset.
//
// If hooked, this function will be called immediately after the boot loader
// sets the system clock.  An application may perform any additional
// initialization during this function.
//
// void MyInitFunc(void);
//
//*****************************************************************************

void MyInitFunc(void)
{
    //ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Configure SPI-0 that drives the reel motor DAC's
	MotorDAC_init();

	// Set the motor DAC's to zero torque at power-up!
	MotorDAC_write(0, 0);
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

}

//*****************************************************************************
//
// These Functions are specific to the DTC-1200 hardware
//
//*****************************************************************************

void MotorDAC_init(void)
{
	// The SSI0 peripheral must be enabled for use.

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	// For this example SSI0 is used with PortA[5:2].  The actual port and pins
	// used may be different on your part, consult the data sheet for more
	// information.  GPIO port A needs to be enabled so these pins can be used.

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
	// This step is not necessary if your part does not support pin muxing.

	ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
	ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);

	// Configure the GPIO settings for the SSI pins.  This function also gives
	// control of these pins to the SSI hardware.  Consult the data sheet to
	// see which functions are allocated per pin.
	// The pins are assigned as follows:
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

	//
	// Read any residual data from the SSI port.  This makes sure the receive
	// FIFOs are empty, so we don't read any unwanted junk.  This is done here
	// because the SPI SSI mode is full-duplex, which allows you to send and
	// receive at the same time.  The SSIDataGetNonBlocking function returns
	// "true" when data was returned, and "false" when no data was returned.
	// The "non-blocking" function checks if there is any data in the receive
	// FIFO and does not "hang" if there isn't.
	//

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

void MotorDAC_write(unsigned supply, unsigned takeup)
{
    unsigned ulWord;
    unsigned ulDac;

    takeup = DAC_MAX - takeup;
    supply = DAC_MAX - supply;

    /* (1) Set reference voltage to 1.024 V (CONTROL register) */
    ulWord = (1 << 15) | (1 << 12) | 0x01;
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    ROM_SSIDataPut(SSI0_BASE, ulWord);
    while(ROM_SSIBusy(SSI0_BASE));
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    /* (2) Write data for DAC B to BUFFER */
    ulDac  = (takeup & 0x3FF) << 2;
    ulWord = (1 << 12) | (uint16_t)ulDac;
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    ROM_SSIDataPut(SSI0_BASE, ulWord);
    while(ROM_SSIBusy(SSI0_BASE));
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    /* (3) Write DAC A value and update DAC A & B simultaneously */
    ulDac  = (supply & 0x3FF) << 2;
    ulWord = (1 << 15) | (uint16_t)ulDac;
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    ROM_SSIDataPut(SSI0_BASE, ulWord);
    while(ROM_SSIBusy(SSI0_BASE));
    //GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

