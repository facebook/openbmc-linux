/* arch/arm/mach-aspeed/include/mach/regs-ast1070-scu.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2013/05/15 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __AST1070_SCU_H
#define __AST1070_SCU_H                     1

/*
 *  Register for SCU
 *  */
#define AST1070_SCU_PROTECT			0x00		/*	protection key register	*/
#define AST1070_SCU_RESET			0x04		/*	system reset control register */
#define AST1070_SCU_MISC_CTRL		0x08		/*	misc control register	*/
#define AST1070_SCU_UART_MUX		0x0C		/*	UART Mux control register	*/
#define AST1070_SCU_SPI_USB_MUX		0x10		/*	SPI/USB Mux control register	*/
#define AST1070_SCU_IO_DRIVING		0x14		/*	I/O Driving Strength control register	*/
#define AST1070_SCU_IO_PULL			0x18		/*	I/O Internal Pull control register	*/
#define AST1070_SCU_IO_SLEW			0x1C		/*	I/O Slew Rate control register	*/
#define AST1070_SCU_IO_SCHMITT		0x20		/*	I/O Schmitt Trigger Control register	*/
#define AST1070_SCU_IO_SELECT		0x24		/*	I/O Port Selection register	*/
#define AST1070_SCU_TRAP			0x30		/*	HW TRAPPING register	*/
#define AST1070_SCU_CHIP_ID			0x34		/*	CHIP ID register	*/


/*	AST1070_SCU_PROTECT: 0x00  - protection key register */
#define AST1070_SCU_PROTECT_UNLOCK			0x16881A78

/*	AST1070_SCU_RESET :0x04	 - system reset control register */
#define SCU_RESET_DMA				(0x1 << 11)
#define SCU_RESET_SPI_M				(0x1 << 10)
#define SCU_RESET_SPI_S				(0x1 << 9)
#define SCU_RESET_N4_LPC			(0x1 << 8)
#define SCU_RESET_N3_LPC			(0x1 << 7)
#define SCU_RESET_N2_LPC			(0x1 << 6)
#define SCU_RESET_N1_LPC			(0x1 << 5)
#define SCU_RESET_I2C				(0x1 << 4)
#define SCU_RESET_N4_UART			(0x1 << 3)
#define SCU_RESET_N3_UART			(0x1 << 2)
#define SCU_RESET_N2_UART			(0x1 << 1)
#define SCU_RESET_N1_UART			(0x1 << 0)

/*	AST1070_SCU_MISC_CTRL		0x08		misc control register	*/
#define SCU_DMA_M_S_MASK			(0x3 << 9)

#define SCU_DMA_SLAVE_EN			(0x1 << 10)
#define SCU_DMA_MASTER_EN			(0x1 << 9)

/*	AST1070_SCU_UART_MUX		0x0C	UART Mux control register	*/
#define UART_MUX_MASK(x)			(0xff << (x*8))

#define BMC_UART_CTRL(x)			(6 + (x*8))
#define BMC_UART_CTRL_MASK(x)		(0x3 << (6 + (x*8)))
#define SET_BMC_UART_CTRL(x,v)		((v) << (6 + (x*8)))
#define BMC_UART_FROM_N1			0
#define BMC_UART_FROM_PAD1			1
#define BMC_UART_FROM_NONE			2

#define NODE_UART_CTRL(x)			(3 + (x*8))
#define NODE_UART_CTRL_MASK(x)		(0x7 << (3 + (x*8)))
#define SET_NODE_UART_CTRL(x,v)		((v) << (3 + (x*8)))
#define NODE_UART_FROM_BMC			0
#define NODE_UART_FROM_PAD1			1
#define NODE_UART_FROM_PAD2			2
#define NODE_UART_FROM_PAD3			3
#define NODE_UART_FROM_PAD4			4
#define NODE_UART_FROM_NONE			5
#define NODE_UART_FROM_N2			6
#define NODE_UART_FROM_N3			7


#define SCU_UART_IO_PAD(x)			(x*8)
#define UART_IO_PAD_MASK(x)			(0x7 << (x*8))
#define SET_UART_IO_PAD(x,v)		((v) << (x*8))
#define PAD_FROM_NONE				0
#define PAD_FROM_N1_UART			1
#define PAD_FROM_N2_UART			2
#define PAD_FROM_N3_UART			3
#define PAD_FROM_N4_UART			4
#define PAD_FROM_BMC				5

/*	AST1070_SCU_TRAP			0x30		HW TRAPPING register	*/
#define TRAP_DEVICE_SLAVE			(0x1 << 2)
#define TRAP_MULTI_MASTER			(0x1 << 1)
#define TRAP_LPC_PLUS_MODE			(0x1 << 0)

#endif

