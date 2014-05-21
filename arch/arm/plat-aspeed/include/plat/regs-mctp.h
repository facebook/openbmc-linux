/* arch/arm/mach-aspeed/include/mach/regs-ast1010-scu.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2012/12/29 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __AST_MCTP_H
#define __AST_MCTP_H                     1

/*
 *  Register for MCTP 
 *  */
#define AST_MCTP_CTRL			0x00		/*	Engine Status and Engine Control	*/
#define AST_MCTP_INT			0x04		/*	Interrupt Enable and Status Register */
#define AST_MCTP_ID				0x08		/*	Target ID and Mask */
#define AST_MCTP_TX_DESC3		0x10		/*	Sending Descriptor [127:96] */
#define AST_MCTP_TX_DESC2		0x14		/*	Sending Descriptor [95:64] */
#define AST_MCTP_TX_DESC1		0x18		/*	Sending Descriptor [63:32] */
#define AST_MCTP_TX_DESC0		0x1C		/*	Sending Descriptor [31:0] */
#define AST_MCTP_TX_DATA		0x20		/*	Sending Data Port */
#define AST_MCTP_RX_DESC3		0x40		/*	Received Descriptor [127:96] */
#define AST_MCTP_RX_DESC2		0x44		/*	Received Descriptor [95:64] */
#define AST_MCTP_RX_DESC1		0x48		/*	Received Descriptor [63:32] */
#define AST_MCTP_RX_DESC0		0x4C		/*	Received Descriptor [31:0] */
#define AST_MCTP_RX_DATA		0x50		/*	Received Data Port */

#define AST_MCTP_DEC_ADDR		0x80		/*	ADDR */
#define AST_MCTP_DEC_MASK		0x84		/*	MASK */
#define AST_MCTP_DEC_TAG		0x88		/*	TAG */

/*	AST_MCTP_CTRL			0x00		Engine Status and Engine Control	*/

/*	AST_MCTP_INT			0x04		Interrupt Enable and Status Register */
#define MCTP_RX_INT_EN			(1 << 17)
#define MCTP_TX_INT_EN			(1 << 16)

#define MCTP_RX_COMPLETE		(1 << 1)
#define MCTP_TX_COMPLETE		(1)

#endif

