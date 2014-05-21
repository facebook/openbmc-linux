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

#ifndef __AST_LPC_H_
#define __AST_LPC_H_

#define AST_LPC_HICR0				0x000
#define	AST_LPC_HICR1				0x004
#define AST_LPC_HICR2				0x008	/* Host Interface Control Register 2 */
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
#define AST_LPC_BTR0				0x048
#define AST_LPC_BTR1				0x04C
#define AST_LPC_BTCSR0				0x050
#define AST_LPC_BTCSR1				0x054
#define AST_LPC_BTCR				0x058
#define AST_LPC_BTDTR				0x05C
#define AST_LPC_BTIMSR				0x060
#define AST_LPC_BTFVSR0				0x064
#define AST_LPC_BTFVSR1				0x068
#define AST_LPC_SIRQCR0				0x06C
#define AST_LPC_SIRQCR1				0x070
#define AST_LPC_SIRQCR2				0x074
#define AST_LPC_SIRQCR3				0x078

//////
#define AST_LPC_HICR5				0x080	/* LPC Host interface Control Register 5 */
#define AST_LPC_HICR6				0x084	/* LPC Host Interface Control Register 6 */
#define AST_LPC_HICR7				0x088
#define AST_LPC_HICR8				0x08C
#define AST_LPC_SNPWADR				0x090	/* LPC Snoop Address Register */
#define AST_LPC_SNPWDR				0x094	/* LPC SNoop Data Register */
#define AST_LPC_HICR9				0x098
#define AST_LPC_HICRA				0x09C
#define AST_LPC_LHCR0				0x0A0
#define AST_LPC_LHCR1				0x0A4
#define AST_LPC_LHCR2				0x0A8
#define AST_LPC_LHCR3				0x0AC
#define AST_LPC_LHCR4				0x0B0
#define AST_LPC_LHCR5				0x0B4
#define AST_LPC_LHCR6				0x0B8
#define AST_LPC_LHCR7				0x0BC
#define AST_LPC_LHCR8				0x0C0
#define AST_LPC_PCCR6				0x0C4
#define AST_LPC_LHCRA				0x0C8
#define AST_LPC_LHCRB				0x0CC


#define AST_LPC_PCCR4				0x0D0	/* Post Code Control Regiter 4 */
#define AST_LPC_PCCR5				0x0D4	/* Post Code Control Regiter 5 */

#define AST_LPC_HICRB				0x0D8
#define AST_LPC_HICRC				0x0DC
#define AST_LPC_HISR0				0x0E0
#define AST_LPC_HISR1				0x0E4
#define AST_LPC_LADR4				0x0E8
#define AST_LPC_IDR4				0x0EC
#define AST_LPC_ODR4				0x0F0
#define AST_LPC_STR4				0x0F4
#define AST_LPC_LSADR12				0x0F8
#define AST_LPC_IDR5				0x0FC
#define AST_LPC_ODR5				0x100
#define AST_LPC_STR5				0x104



#define AST_LPC_PCCR0				0x130	/*Post Code Contol Register 0 */
#define AST_LPC_PCCR1				0x134	/*Post Code Contol Register 1 */
#define AST_LPC_PCCR2				0x138	/*Post Code Contol Register 2 */
#define AST_LPC_PCCR3				0x13C	/*Post Code Contol Register 3 */


#define AST_LPC_IBTCR0				0x140
#define AST_LPC_IBTCR1				0x144
#define AST_LPC_IBTCR2				0x148
#define AST_LPC_IBTCR3				0x14C
#define AST_LPC_IBTCR4				0x150
#define AST_LPC_IBTCR5				0x154
#define AST_LPC_IBTCR6				0x158
#define AST_LPC_SRUART1				0x15C
#define AST_LPC_SRUART2				0x160
#define AST_LPC_SRUART3				0x164
#define AST_LPC_SRUART4				0x168
#define AST_LPC_SCR0SIO				0x16C
#define AST_LPC_SCR0SI1				0x170
#define AST_LPC_SCR0SI2				0x174
#define AST_LPC_SCR0SI3				0x17C

#define AST_LPC_SWCR0300			0x180
#define AST_LPC_SWCR0704			0x184
#define AST_LPC_SWCR0B08			0x188
#define AST_LPC_SWCR0F0C			0x18C
#define AST_LPC_SWCR1310			0x190
#define AST_LPC_SWCR1714			0x194
#define AST_LPC_SWCR1B18			0x198
#define AST_LPC_SWCR1F1C			0x19C
#define AST_LPC_ACPIE3E0			0x1A0
#define AST_LPC_ACPIC1C0			0x1A4
#define AST_LPC_ACPIB3B0			0x1A8
#define AST_LPC_ACPIB7B4			0x1AC

/* AST_LPC_HICR0				0x000 		*/
#define LPC_LPC3_EN					(1 << 7)
#define LPC_LPC2_EN					(1 << 6)
#define LPC_LPC1_EN					(1 << 5)

#define LPC_SDWNE					(1 << 3)
#define LPC_PMEE					(1 << 2)

/* AST_LPC_HICR2				0x008		*/
#define LPC_LRST					(1 << 6)
#define LPC_SDWN					(1 << 5)
#define LPC_ABRT					(1 << 4)
#define LPC_IBFIF3					(1 << 3)
#define LPC_IBFIF2					(1 << 2)
#define LPC_IBFIF1					(1 << 1)
#define LPC_EERIE					(1)







/*	AST_LPC_HICR4				0x010		*/
#define LPC_HICS_LADR12AS			(1 << 7)
#define LPC_HICS_CLRINTLRSTR		(1 << 6)
#define LPC_HICS_STSINTLRSTR		(1 << 5)
#define LPC_HICS_ENINTLRSTR			(1 << 4)
/* bit 3 reserved */
#define LPC_HICS_KCSENBL			(1 << 2)
/* bit 1 reserved */
#define LPC_HICS_BTENBL				(1)


/*	AST_LPC_STR1				0: 0x03C, 1: 0x40, 2 : 0x44, 3: 4:		*/
#define LPC_STR_DBU4				(1 << 7)		
#define LPC_STR_DBU3				(1 << 6)
#define LPC_STR_DBU2				(1 << 5)
#define LPC_STR_DBU1				(1 << 4)
#define LPC_STR_CMD_DAT				(1 << 3)
#define LPC_STR_DBU0				(1 << 2)
#define LPC_STR_IBF					(1 << 1)
#define LPC_STR_OBF					(1)


/* AST_LPC_HICR5				0x080	- LPC Host interface Control Register */
#define LPC_HICR5_SNP1INT_EN		(1 << 3)
#define LPC_HICR5_SNP1W_EN			(1 << 2)
#define LPC_HICR5_SNP0INT_EN		(1 << 1)
#define LPC_HICR5_SNP0W_EN			(1)

/* AST_LPC_HICR6				0x084	- LPC Host Interface Control Register 6 */
#define LPC_HICR6_STR_BAUD			(1 << 3)
#define LPC_HICR6_STR_PME			(1 << 2)
#define LPC_HICR6_STR_SNP1W			(1 << 1)
#define LPC_HICR6_STR_SNP0W			(1)

/* AST_LPC_SNPWADR				0x090	- LPC Snoop Address Register*/
#define LPC_SNOOP_ADDR1_MASK		(0xffff << 16)
#define LPC_SNOOP_ADDR0_MASK		(0xffff)

/* AST_LPC_SNPWDR				0x094	- LPC SNoop Data Register */
#define GET_LPC_SNPD1(x)			((x >> 7) & 0xff)
#define GET_LPC_SNPD0(x)			(x & 0xff)

/*AST_LPC_PCCR0				0x130	- Post Code Contol Register 0 */
#define LPC_POST_DMA_INT_EN			(1 << 31)
#define LPC_POST_DMA_MODE_EN		(1 << 14)
#define LPC_RX_FIFO_CLR				(1 << 7)
#define LPC_POST_
#define LPC_POST_CODE_MODE_MASK		(0x3 << 4)
#define LPC_POST_CODE_MODE(x)		(x << 4)
#define BYTE_MODE	0
#define WORD_MODE	1
#define DWORD_MODE	2
#define FULL_MODE	3

#define LPC_POST_CODE_RXOVR			(1 << 3)
#define LPC_POST_CODE_RXTO			(1 << 2)
#define LPC_POST_CODE_RXAVA			(1 << 1)
#define LPC_POST_CODE_EN			(1)

/*AST_LPC_PCCR1				0x134	Post Code Contol Register 1 */
#define LPC_POST_ADDR_MASK		0x3fffff
#define LPC_CAPTURE_ADDR_MASK(x)	(x << 16)
#define LPC_CAPTURE_BASE_ADDR(x)	(x)

/*AST_LPC_PCCR2				0x138	Post Code Contol Register 2 */
#define LPC_POST_CODE_DMA_RDY		(1 << 4)
#define LPC_POST_CODE_STS			(1)

#endif 
