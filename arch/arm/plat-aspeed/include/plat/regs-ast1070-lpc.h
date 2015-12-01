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

#ifndef __AST1070_LPC_H_
#define __AST1070_LPC_H_

#define AST_LPC_HICR0				0x000
#define	AST_LPC_HICR1				0x004
#define AST_LPC_HICR2				0x008
#define AST_LPC_HICR3				0x00C
#define AST_LPC_HICR4				0x010
#define AST_LPC_LADR3H				0x014
#define AST_LPC_LADR3L				0x018
#define AST_LPC_LADR12H				0x01C
#define AST_LPC_LADR12L				0x020
#define AST_LPC_IDR1				0x024
#define AST_LPC_IDR2				0x028
#define AST_LPC_IDR3				0x02C
#define AST_LPC_ODR1				0x030
#define AST_LPC_ODR2				0x034
#define AST_LPC_ODR3				0x038
#define AST_LPC_STR1				0x03C
#define AST_LPC_STR2				0x040
#define AST_LPC_STR3				0x044
////
#define AST_LPC_SIRQCR0				0x048
#define AST_LPC_SIRQCR1				0x04C
#define AST_LPC_SIRQCR2				0x050
#define AST_LPC_SIRQCR3				0x06C
////
#define AST_LPC_ADR1				0x070
#define AST_LPC_IRQ1				0x074
#define AST_LPC_ADR2				0x078
#define AST_LPC_IRQ2				0x07C
#define AST_LPC_ADR3				0x080
#define AST_LPC_IRQ3				0x084

#define AST_LPC_DEV_ADDRM0			0x100
#define AST_LPC_DEV_ADDRM1			0x104
#define AST_LPC_DEV_ADDRM2			0x108
#define AST_LPC_DEV_ADDRM3			0x10C
#define AST_LPC_DEV_ADDR0			0x110
#define AST_LPC_DEV_ADDR1			0x114
#define AST_LPC_DEV_ADDR2			0x118
#define AST_LPC_DEV_ADDR3			0x11C
#define AST_LPC_DEC_ADDR0			0x120
#define AST_LPC_DEC_ADDR1			0x124
#define AST_LPC_DEC_RANGE0			0x128
#define AST_LPC_DEC_RANGE1			0x12C

#define AST_LPC_MBXDAT0				0x180
#define AST_LPC_MBXDAT1				0x184
#define AST_LPC_MBXDAT2				0x188
#define AST_LPC_MBXDAT3				0x18C
#define AST_LPC_MBXDAT4				0x190
#define AST_LPC_MBXDAT5				0x194
#define AST_LPC_MBXDAT6				0x198
#define AST_LPC_MBXDAT7				0x19C
#define AST_LPC_MBXDAT8				0x1A0
#define AST_LPC_MBXDAT9				0x1A4
#define AST_LPC_MBXDATA				0x1A8
#define AST_LPC_MBXDATB				0x1AC
#define AST_LPC_MBXDATC				0x1B0
#define AST_LPC_MBXDATD				0x1B4
#define AST_LPC_MBXDATE				0x1B8
#define AST_LPC_MBXDATF				0x1BC

#define AST_LPC_MBXSTS0				0x1C0
#define AST_LPC_MBXSTS1				0x1C4

#define AST_LPC_MBXBICR				0x1C8
#define AST_LPC_MBXHICR				0x1CC
#define AST_LPC_MBXBIE0				0x1D0
#define AST_LPC_MBXBIE1				0x1D4
#define AST_LPC_MBXHIE0				0x1D8
#define AST_LPC_MBXHIE1				0x1DC

#define AST_LPC_PIN_MON				0x200
#define AST_LPC_SIRQ_CTRL			0x208
#define AST_LPC_INT_STS				0x20C
#define AST_LPC_CTRL_STS			0x210
#define AST_LPC_PME					0x218
#define AST_LPC_SMI					0x21C
#define AST_LPC_80H_ADDR0			0x220
#define AST_LPC_80H_ADDR1			0x224
#define AST_LPC_80H_DATA			0x228
#define AST_LPC_80H_CTRL			0x22C

#define AST_LPC_CHIP_VER			0x240
#define AST_LPC_CHIP_REVER			0x244
#define AST_LPC_BMC_SCH0			0x248
#define AST_LPC_BMC_SCH1			0x24C
#define AST_LPC_NODE_SCH0			0x250
#define AST_LPC_NODE_SCH1			0x254
#define AST_LPC_NODE_SCH2			0x258
#define AST_LPC_NODE_SCH3			0x25C











#endif 
