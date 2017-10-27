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

#define DAC_MAX			0x03FF      /* 10-bit full scale DAC   */

//*****************************************************************************
//
// Writes data out the SPI port in master mode
//
//*****************************************************************************

void SPISend16(unsigned ui16Data)
{
    //
    // Wait until there is space in the SSI FIFO.
    //
    while(!(HWREG(SSIx_BASE + SSI_O_SR) & SSI_SR_TNF))
    {
    }

    //
    // Write the 16 bit word to the SSI port.
    //
    HWREG(SSIx_BASE + SSI_O_DR) = ui16Data;

    //
    // Empty the receive FIFO.
    //
    while(HWREG(SSIx_BASE + SSI_O_SR) & SSI_SR_RNE)
    {
        HWREG(SSIx_BASE + SSI_O_DR);
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
	//GPIO_write(Board_CS_SPI0, PIN_LOW);
	SPISend16(ulWord);
	//GPIO_write(Board_CS_SPI0, PIN_HIGH);

	/* (2) Write data for DAC B to BUFFER */
	ulDac  = (takeup & 0x3FF) << 2;
	ulWord = (1 << 12) | (uint16_t)ulDac;
	//GPIO_write(Board_CS_SPI0, PIN_LOW);
	SPISend16(ulWord);
	//GPIO_write(Board_CS_SPI0, PIN_HIGH);

	/* (3) Write DAC A value and update DAC A & B simultaneously */
	ulDac  = (supply & 0x3FF) << 2;
	ulWord = (1 << 15) | (uint16_t)ulDac;
	//GPIO_write(Board_CS_SPI0, PIN_LOW);
	SPISend16(ulWord);
	//GPIO_write(Board_CS_SPI0, PIN_HIGH);
}

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
#if 0
    //
    // Enable the clocks to the SSI and GPIO modules.
    //
    HWREG(SYSCTL_RCGCGPIO) |= (SSI_CLKPIN_CLOCK_ENABLE |
                               SSI_FSSPIN_CLOCK_ENABLE |
                               /*SSI_MISOPIN_CLOCK_ENABLE |*/
                               SSI_MOSIPIN_CLOCK_ENABLE);
    HWREG(SYSCTL_RCGCSSI) |= SSI_CLOCK_ENABLE;

    //
    // Make the pin be peripheral controlled.
    //
    HWREG(SSI_CLKPIN_BASE + GPIO_O_AFSEL) |= SSI_CLK;
    HWREG(SSI_CLKPIN_BASE + GPIO_O_PCTL) |= SSI_CLK_PCTL;
    HWREG(SSI_CLKPIN_BASE + GPIO_O_DEN) |= SSI_CLK;
    HWREG(SSI_CLKPIN_BASE + GPIO_O_ODR) &= ~(SSI_CLK);

    HWREG(SSI_FSSPIN_BASE + GPIO_O_AFSEL) |= SSI_CS;
    HWREG(SSI_FSSPIN_BASE + GPIO_O_PCTL) |= SSI_CS_PCTL;
    HWREG(SSI_FSSPIN_BASE + GPIO_O_DEN) |= SSI_CS;
    HWREG(SSI_FSSPIN_BASE + GPIO_O_ODR) &= ~(SSI_CS);

    HWREG(SSI_MISOPIN_BASE + GPIO_O_AFSEL) |= SSI_TX;
    HWREG(SSI_MISOPIN_BASE + GPIO_O_PCTL) |= SSI_TX_PCTL;
    HWREG(SSI_MISOPIN_BASE + GPIO_O_DEN) |= SSI_TX;
    HWREG(SSI_MISOPIN_BASE + GPIO_O_ODR) &= ~(SSI_TX);

    //HWREG(SSI_MOSIPIN_BASE + GPIO_O_AFSEL) |= SSI_RX;
    //HWREG(SSI_MOSIPIN_BASE + GPIO_O_PCTL) |= SSI_RX_PCTL;
    //HWREG(SSI_MOSIPIN_BASE + GPIO_O_DEN) |= SSI_RX;
    //HWREG(SSI_MOSIPIN_BASE + GPIO_O_ODR) &= ~(SSI_RX);

    //
    // Set the SSI protocol to Motorola POL=1, PHA=0, 16-bit
    //
    HWREG(SSIx_BASE + SSI_O_CR0) = (SSI_CR0_SPO | SSI_CR0_DSS_16);

    //
    // Enable the SSI interface in master mode.
    //
    HWREG(SSIx_BASE + SSI_O_CR1) = SSI_CR1_SSE;

	//
	// Now set the reel motors to have zero torque while booting!
	//
	MotorDAC_write(0, 0);

#endif
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
