/* arch/arm/plat-aspeed/include/mach/regs-1070_lpc.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED AST1070 LPC Controller
*/

#ifndef __ASM_ARCH_REGS_LPC_H
#define __ASM_ARCH_REGS_LPC_H __FILE__

#define AST1070_LPC_HICR0			0x00
#define AST1070_LPC_HICR1			0x04
#define AST1070_LPC_HICR2			0x08
#define AST1070_LPC_HICR3			0x0c
#define AST1070_LPC_HICR4			0x10

//for snoop driver 
#define AST1070_LPC_L_80H_ADDR		0x220
#define AST1070_LPC_H_80H_ADDR		0x224
#define AST1070_LPC_80H_DATA		0x228
#define AST1070_LPC_80H_CTRL		0x22c


#define AST1070_LPC_80H_CLR			(0x1 << 4)

#define AST1070_LPC_80H_EN			0x1
#endif /* __ASM_ARCH_REGS_LPC_H */
