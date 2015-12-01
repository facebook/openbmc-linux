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
#ifndef __AST_P2X_H
#define __AST_P2X_H                     1

/*
 *  Register for MCTP 
 *  */
#define AST_P2X_CTRL			0x00		/*	Engine Status and Engine Control	*/
#define AST_P2X_INT				0x04		/*	Interrupt Enable and Status Register */
#define AST_P2X_ID				0x08		/*	Target ID and Mask */
#define AST_P2X_TX_DESC3		0x10		/*	Sending Descriptor [127:96] */
#define AST_P2X_TX_DESC2		0x14		/*	Sending Descriptor [95:64] */
#define AST_P2X_TX_DESC1		0x18		/*	Sending Descriptor [63:32] */
#define AST_P2X_TX_DESC0		0x1C		/*	Sending Descriptor [31:0] */
#define AST_P2X_TX_DATA		0x20		/*	Sending Data Port */
#define AST_P2X_RX_DESC3		0x40		/*	Received Descriptor [127:96] */
#define AST_P2X_RX_DESC2		0x44		/*	Received Descriptor [95:64] */
#define AST_P2X_RX_DESC1		0x48		/*	Received Descriptor [63:32] */
#define AST_P2X_RX_DESC0		0x4C		/*	Received Descriptor [31:0] */
#define AST_P2X_RX_DATA		0x50		/*	Received Data Port */

#define AST_P2X_MSI_IER			0x70		/*	MSI interrupt enalbe */
#define AST_P2X_MSI_ISR			0x74		/*	MSI interrupt sts */

#define AST_P2X_DEC_ADDR		0x80		/*	ADDR */
#define AST_P2X_DEC_MASK		0x84		/*	MASK */
#define AST_P2X_DEC_TAG		0x88		/*	TAG */

/*	AST_P2X_CTRL			0x00		Engine Status and Engine Control	*/
#define P2X_CTRL_GET_RX_LEN(x)		(((x >> 18) & 0xf) * 4)
#define P2X_CTRL_RX_IDLE			(1 << 17)
#define P2X_CTRL_TX_IDLE			(1 << 16)

#define P2X_CTRL_RX_MSI_EN			(1 << 5)
#define P2X_CTRL_UNLOCK_RX_BUFF	(1 << 4)
#define P2X_CTRL_RX_MATCH_EN		(1 << 3)
#define P2X_CTRL_DROP_DIS			(1 << 2)
#define P2X_CTRL_TX_TRIGGER		(1 << 1)
#define P2X_CTRL_RX_EN				(1)

/*	AST_P2X_INT			0x04		Interrupt Enable and Status Register */
#define P2X_INTD_EN				(1 << 21)
#define P2X_INTC_EN				(1 << 20)
#define P2X_INTB_EN				(1 << 19)
#define P2X_INTA_EN				(1 << 18)
#define P2X_RX_INT_EN			(1 << 17)
#define P2X_TX_INT_EN			(1 << 16)

#define P2X_PCIE_MSI				(1 << 6)
#define P2X_PCIE_INTD				(1 << 5)
#define P2X_PCIE_INTC				(1 << 4)
#define P2X_PCIE_INTB				(1 << 3)
#define P2X_PCIE_INTA				(1 << 2)
#define P2X_RX_COMPLETE			(1 << 1)
#define P2X_TX_COMPLETE			(1)

#endif
