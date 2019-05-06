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

#ifndef __CUSTOM_H
#define __CUSTOM_H

/*** DTC-1200 Hardware Constants ***********************************************/

#define HW_REV			3			/* 0=A, 1=B, 2=C, 3=D etc  */

#define DAC_MIN			0           /* zero scale DAC setting  */
#define DAC_MAX			0x03FF      /* 10-bit full scale DAC   */

#define ADC_MIN			0           /* zero scale ADC input    */
#define ADC_MAX			0x0FFF      /* 12-bit full scale ADC   */

/*******************************************************************************
 * MCP23017 I/O Expander U5 Button/Switch and Lamp Definitions
 ******************************************************************************/

/* Transport Switch Inputs */
#define S_STOP          0x01        // stop button
#define S_PLAY          0x02        // play button
#define S_REC           0x04        // record button
#define S_REW           0x08        // rewind button
#define S_FWD           0x10        // fast fwd button
#define S_LDEF          0x20        // lift defeat button
#define S_TAPEOUT       0x40      	// tape out switch
#define S_TAPEIN        0x80      	// tape detect (dummy bit)

#define S_BUTTON_MASK   (S_STOP | S_PLAY | S_REC | S_LDEF | S_FWD | S_REW)
#define S_SWITCH_MASK   (S_TAPEOUT)

/* Lamp Driver Outputs */
#define L_REC           0x01      	// record indicator lamp
#define L_PLAY          0x02      	// play indicator lamp
#define L_STOP          0x04      	// stop indicator lamp
#define L_FWD           0x08      	// forward indicator lamp
#define L_REW           0x10      	// rewind indicator lamp
#define L_STAT1         0x20      	// diagnostic led1
#define L_STAT2         0x40      	// diagnostic led2
#define L_STAT3         0x80   		// diagnostic led3

#define L_LED_MASK      (L_STAT3 | L_STAT2 | L_STAT1)
#define L_LAMP_MASK     (L_FWD | L_REW | L_PLAY | L_REC | L_STOP)

/*******************************************************************************
 * MCP23017 I/O Expander U8 Solenoid Drivers and Mode Configuration Switches
 ******************************************************************************/

/* Transport Solenoid and Record drivers */
#define T_BRAKE      	0x01        // engage reel motor brakes
#define T_TLIFT         0x02        // engage tape lifter solenoid
#define T_PROL          0x04        // engage pinch roller solenoid
#define T_RECP          0x08        // record pulse toggle bit
#define T_RECH          0x10        // record hold bit
#define T_SERVO         0x20        // capstan servo enable

#define T_REC_MASK      (T_RECP | T_RECH)

/* Config DIP Switches & speed select */
#define M_DIPSW1        0x01        // config DIP switch 1
#define M_DIPSW2        0x02        // config DIP switch 2
#define M_DIPSW3        0x04        // config DIP switch 3
#define M_DIPSW4        0x08        // config DIP switch 4
#define M_HISPEED       0x80        // remote speed select switch

#define M_DIPSW_MASK    (M_DIPSW1 | M_DIPSW2 | M_DIPSW3 | M_DIPSW4)

/*******************************************************************************
 * MCP23017 Register Addresses (IOCON.BANK = 0)
 ******************************************************************************/

#define MCP_IODIRA      0x00		// I/O DIRECTION REGISTER
#define MCP_IODIRB      0x01		// I/O DIRECTION REGISTER
#define MCP_IOPOLA      0x02		// INPUT POLARITY REGISTER
#define MCP_IOPOLB      0x03		// INPUT POLARITY REGISTER
#define MCP_GPINTENA    0x04		// INTERRUPT-ON-CHANGE CONTROL REGISTER
#define MCP_GPINTENB    0x05		// INTERRUPT-ON-CHANGE CONTROL REGISTER
#define MCP_DEFVALA     0x06		// DEFAULT COMPARE REGISTER FOR INT-ON-CHANGE
#define MCP_DEFVALB     0x07		// DEFAULT COMPARE REGISTER FOR INT-ON-CHANGE
#define MCP_INTCONA     0x08		// INTERRUPT CONTROL REGISTER
#define MCP_INTCONB     0x09		// INTERRUPT CONTROL REGISTER
#define MCP_IOCONA      0x0A		// I/O EXPANDER CONFIGURATION REGISTER
#define MCP_IOCONB      0x0B		// I/O EXPANDER CONFIGURATION REGISTER
#define MCP_GPPUA       0x0C		// GPIO PULL-UP RESISTOR REGISTER
#define MCP_GPPUB       0x0D		// GPIO PULL-UP RESISTOR REGISTER
#define MCP_INTFA       0x0E		// INTERRUPT FLAG REGISTER
#define MCP_INTFB       0x0F		// INTERRUPT FLAG REGISTER
#define MCP_INTCAPA     0x10		// INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define MCP_INTCAPB     0x11		// INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define MCP_GPIOA       0x12		// GENERAL PURPOSE I/O PORT REGISTER
#define MCP_GPIOB       0x13		// GENERAL PURPOSE I/O PORT REGISTER
#define MCP_OLATA       0x14		// OUTPUT LATCH REGISTER
#define MCP_OLATB       0x15		// OUTPUT LATCH REGISTER

/* IOCON Configuration Register Bits */
#define C_INTPOL        0x02	/* INT output 1=Active-high, 0=Active-low. */
#define C_ODR           0x04	/* INT pin as an open-drain output         */
#define C_HAEN          0x08	/* Hardware address enable (N/A for I2C)   */
#define C_DISSLW        0x10	/* Slew rate disable bit                   */
#define C_SEQOP         0x20	/* Disable address pointer auto-increment  */
#define C_MIRROR        0x40	/* INT A/B pins mirrored                   */
#define C_BANK          0x80	/* port registers are in different banks   */

#endif /* __CUSTOM_H */
