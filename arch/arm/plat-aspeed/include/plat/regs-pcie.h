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
#ifndef __AST_PCIE_H
#define __AST_PCIE_H                     1

/*
 *  Register for PCIE 
 *  */
#define AST_PCIE_CFG2			0x04
#define AST_PCIE_GLOBAL			0x30
#define AST_PCIE_CFG_DIN			0x50	
#define AST_PCIE_CFG3			0x58
#define AST_PCIE_LOCK			0x7C

#define AST_PCIE_LINK			0xC0
#define AST_PCIE_INT				0xC4

/* 	AST_PCIE_CFG2			0x04		*/
#define PCIE_CFG_CLASS_CODE(x)	(x << 8)
#define PCIE_CFG_REV_ID(x)		(x)


/*SSID: 1E6ED028h[19:4]*/
/*SSVID: 1E6ED028h[3:0], 1E6ED024h[31:20]*/

/* AST_PCIE_SSID_A			0x24		*/
/* 31:20 */
#define PCIE_SSVID_H(x)			(x)

/* AST_PCIE_SSID_B			0x28		*/
/* 19:14 */
#define PCIE_SSID(x)			(x << 4)
/* 3:0 */
#define PCIE_SSVID_L(x)			(x)


/* 	AST_PCIE_GLOBAL			0x30 	*/
#define ROOT_COMPLEX_ID(x)		(x << 4)

/* AST_PCIE_CFG3			0x58	*/
#define PCIE_CFG_ADDR(x)			(x & 0xfff)
#define PCIE_CFG_ADDR_MASK		(0xfff)
#define PCIE_CFG_READ			(0x1 << 12)
#define PCIE_CFG_WRITE			(0x1 << 13)
#define PCIE_CFG_ACK				(0x1 << 14)
#define PCIE_MSI_ACK				(0x1 << 15)

/* 	AST_PCIE_LOCK			0x7C	*/
#define PCIE_UNLOCK				0xa8

/*	AST_PCIE_LINK			0xC0	*/
#define PCIE_LINK_STS			(1 << 5)

/*	AST_PCIE_INT			0xC4	*/
#define PCIE_INTD				(1 << 16)
#define PCIE_INTC				(1 << 15)
#define PCIE_INTB				(1 << 14)
#define PCIE_INTA				(1 << 13)

#define AST_PCIE_NONP_MEM_BASE		AST_PCIE0_WIN_BASE0
#define AST_PCIE_NONP_MEM_SIZE		AST_PCIE0_WIN_SIZE0
#define AST_PCIE_PREF_MEM_BASE		0x0
#define AST_PCIE_PREF_MEM_SIZE		0x0

#endif

