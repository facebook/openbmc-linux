/* arch/arm/plat-aspeed/include/mach/regs-lpc.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED LPC Controller
*/

#ifndef __AST_MBX_H_
#define __AST_MBX_H_

#define AST_MBX_DAT0				0x00
#define AST_MBX_DAT1				0x04
#define AST_MBX_DAT2				0x08
#define AST_MBX_DAT3				0x0C
#define AST_MBX_DAT4				0x10
#define AST_MBX_DAT5				0x14
#define AST_MBX_DAT6				0x18
#define AST_MBX_DAT7				0x1C
#define AST_MBX_DAT8				0x20
#define AST_MBX_DAT9				0x24
#define AST_MBX_DATA				0x28
#define AST_MBX_DATB				0x2C
#define AST_MBX_DATC				0x30
#define AST_MBX_DATD				0x34
#define AST_MBX_DATE				0x38
#define AST_MBX_DATF				0x3C
#define AST_MBX_STS0				0x40
#define AST_MBX_STS1				0x44
#define AST_MBX_BCR					0x48
#define AST_MBX_HCR					0x4C
#define AST_MBX_BIE0				0x50
#define AST_MBX_BIE1				0x54
#define AST_MBX_HIE0				0x58
#define AST_MBX_HIE1				0x5C

/* AST_MBX_BCR					0x48 	*/
#define	MBHIST			(1 << 7)
#define	MBHMK			(1 << 1)
#define	MBBINT			(1)



#endif 
