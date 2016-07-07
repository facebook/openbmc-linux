/* arch/arm/plat-aspeed/include/mach/regs-adc.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED ADC Controller
*/

#ifndef __ASM_ARCH_REGS_ADC_H
#define __ASM_ARCH_REGS_ADC_H __FILE__

#if defined(AST_SOC_G3)
#define MAX_CH_NO		12
#elif defined(AST_SOC_G4) || defined(AST_SOC_G5)
#define MAX_CH_NO		16
#elif defined(CONFIG_ARCH_AST1010)
#define MAX_CH_NO		8
#else
#err "ADC NO define MAX CHANNEL NO"
#endif

#if defined(AST_SOC_G5)
#define TEMPER_CH_NO	2
#endif

/*AST ADC Register Definition */
#define AST_ADC_CTRL			0x00
#define AST_ADC_IER				0x04
#define AST_ADC_VGA				0x08
#if defined(CONFIG_ARCH_AST1010)
#define AST_ADC_TRIM			0x08
#endif
#define AST_ADC_CLK				0x0c
#define AST_ADC_CH0_1			0x10
#define AST_ADC_CH2_3			0x14
#define AST_ADC_CH4_5			0x18
#define AST_ADC_CH6_7			0x1c
#define AST_ADC_CH8_9			0x20
#define AST_ADC_CH10_11			0x24
#define AST_ADC_CH12_13			0x28
#define AST_ADC_CH14_15			0x2c
#define AST_ADC_BOUND0			0x30
#define AST_ADC_BOUND1			0x34
#define AST_ADC_BOUND2			0x38
#define AST_ADC_BOUND3			0x3c
#define AST_ADC_BOUND4			0x40
#define AST_ADC_BOUND5			0x44
#define AST_ADC_BOUND6			0x48
#define AST_ADC_BOUND7			0x4c
#define AST_ADC_BOUND8			0x50
#define AST_ADC_BOUND9			0x54
#define AST_ADC_BOUND10			0x58
#define AST_ADC_BOUND11			0x5c
#define AST_ADC_BOUND12			0x60
#define AST_ADC_BOUND13			0x64
#define AST_ADC_BOUND14			0x68
#define AST_ADC_BOUND15			0x6c
#define AST_ADC_HYSTER0			0x70
#define AST_ADC_HYSTER1			0x74
#define AST_ADC_HYSTER2			0x78
#define AST_ADC_HYSTER3			0x7c
#define AST_ADC_HYSTER4			0x80
#define AST_ADC_HYSTER5			0x84
#define AST_ADC_HYSTER6			0x88
#define AST_ADC_HYSTER7			0x8c
#define AST_ADC_HYSTER8			0x90
#define AST_ADC_HYSTER9			0x94
#define AST_ADC_HYSTER10		0x98
#define AST_ADC_HYSTER11		0x9c
#define AST_ADC_HYSTER12		0xa0
#define AST_ADC_HYSTER13		0xa4
#define AST_ADC_HYSTER14		0xa8
#define AST_ADC_HYSTER15		0xac
#define AST_ADC_INTR_SEL		0xC0
#if defined(AST_SOC_G5)
#define AST_ADC_CH16			0xD0
#define AST_ADC_CH17			0xD4
#define AST_ADC_COMP_TRIM		0xC4
#endif

// AST_ADC_CTRL:0x00 - ADC Engine Control Register 
#define AST_ADC_CTRL_CH15_EN		(0x1 << 31)
#define AST_ADC_CTRL_CH14_EN		(0x1 << 30)
#define AST_ADC_CTRL_CH13_EN		(0x1 << 29)
#define AST_ADC_CTRL_CH12_EN		(0x1 << 28)
#define AST_ADC_CTRL_CH11_EN		(0x1 << 27)
#define AST_ADC_CTRL_CH10_EN		(0x1 << 26)
#define AST_ADC_CTRL_CH9_EN			(0x1 << 25)
#define AST_ADC_CTRL_CH8_EN			(0x1 << 24)
#define AST_ADC_CTRL_CH7_EN			(0x1 << 23)
#define AST_ADC_CTRL_CH6_EN			(0x1 << 22)
#define AST_ADC_CTRL_CH5_EN			(0x1 << 21)
#define AST_ADC_CTRL_CH4_EN			(0x1 << 20)
#define AST_ADC_CTRL_CH3_EN			(0x1 << 19)
#define AST_ADC_CTRL_CH2_EN			(0x1 << 18)
#define AST_ADC_CTRL_CH1_EN			(0x1 << 17)
#define AST_ADC_CTRL_CH0_EN			(0x1 << 16)

#if defined(AST_SOC_G3)
#define AST_ADC_CTRL_COMPEN_CLR		(0x1 << 6)
#define AST_ADC_CTRL_COMPEN			(0x1 << 5)
#elif defined(AST_SOC_G4)
#define AST_ADC_CTRL_COMPEN			(0x1 << 4)
#elif defined(AST_SOC_G5)
#define AST_ADC_CTRL_INIT_RDY		(0x1 << 8)
#define AST_ADC_CTRL_COMPEN			(0x1 << 5)
#else
#err "ERROR define COMPEN ADC"
#endif

#if defined(CONFIG_ARCH_AST1010)
#define AST_ADC_CTRL_OTP			(0x1 << 3)
#define AST_ADC_CTRL_PWR_DWN		(0x1 << 2)
#define AST_ADC_CTRL_TEST			(0x1 << 1)
#endif

#define AST_ADC_CTRL_NORMAL			(0x7 << 1)

#define AST_ADC_CTRL_EN				(0x1)


/* AST_ADC_IER : 0x04 - Interrupt Enable and Interrupt status	*/
#define AST_ADC_IER_CH15			(0x1 << 31)
#define AST_ADC_IER_CH14			(0x1 << 30)
#define AST_ADC_IER_CH13			(0x1 << 29)
#define AST_ADC_IER_CH12			(0x1 << 28)
#define AST_ADC_IER_CH11			(0x1 << 27)
#define AST_ADC_IER_CH10			(0x1 << 26)
#define AST_ADC_IER_CH9				(0x1 << 25)
#define AST_ADC_IER_CH8				(0x1 << 24)
#define AST_ADC_IER_CH7				(0x1 << 23)
#define AST_ADC_IER_CH6				(0x1 << 22)
#define AST_ADC_IER_CH5				(0x1 << 21)
#define AST_ADC_IER_CH4				(0x1 << 20)
#define AST_ADC_IER_CH3				(0x1 << 19)
#define AST_ADC_IER_CH2				(0x1 << 18)
#define AST_ADC_IER_CH1				(0x1 << 17)
#define AST_ADC_IER_CH0				(0x1 << 16)
#define AST_ADC_STS_CH15			(0x1 << 15)
#define AST_ADC_STS_CH14			(0x1 << 14)
#define AST_ADC_STS_CH13			(0x1 << 13)
#define AST_ADC_STS_CH12			(0x1 << 12)
#define AST_ADC_STS_CH11			(0x1 << 11)
#define AST_ADC_STS_CH10			(0x1 << 10)
#define AST_ADC_STS_CH9				(0x1 << 9)
#define AST_ADC_STS_CH8				(0x1 << 8)
#define AST_ADC_STS_CH7				(0x1 << 7)
#define AST_ADC_STS_CH6				(0x1 << 6)
#define AST_ADC_STS_CH5				(0x1 << 5)
#define AST_ADC_STS_CH4				(0x1 << 4)
#define AST_ADC_STS_CH3				(0x1 << 3)
#define AST_ADC_STS_CH2				(0x1 << 2)
#define AST_ADC_STS_CH1				(0x1 << 1)
#define AST_ADC_STS_CH0				(0x1)

/* AST_ADC_VGA	: 0x08 - VGA Detect Control */
#define AST_ADC_VGA_EN				(0x1 << 16)
#define AST_ADC_VGA_DIV_MASK		(0x3ff)

/* AST_ADC_CLK : 0x0c - ADC CLK Control */
#define AST_ADC_CLK_PRE_DIV_MASK	(0x7fff << 17)
#define AST_ADC_CLK_PRE_DIV			(0x1 << 17)
#define AST_ADC_CLK_INVERT			(0x1 << 16)		//only for ast2300
#define AST_ADC_CLK_DIV_MASK		(0x3ff)

#define AST_ADC_H_CH_MASK			(0x3ff << 16)
#define AST_ADC_L_CH_MASK			(0x3ff)

#define AST_ADC_H_BOUND				(0x3ff << 16)
#define AST_ADC_L_BOUND				(0x3ff)

#define AST_ADC_HYSTER_EN			(0x1 << 31)

#if defined(AST_SOC_G5)
/* AST_ADC_CH16	 : 0xD0 - */
/* AST_ADC_CH17	 : 0xD4 - */
#define AST_TEMP_CH_RDY				(0x1 << 31)
#define AST_GET_TEMP_A_MASK(x)		((x >>16) & 0xfff)
#define AST_TEMP_CH_EN				(0x1 << 15)		
#define AST_GET_TEMP_B_MASK(x)		(x & 0xfff)
	

#endif

#endif /* __ASM_ARCH_REGS_ADC_H */
