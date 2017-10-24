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
