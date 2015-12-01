/* arch/arm/plat-aspeed/include/mach/regs-smc.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED Static memory ctrol 
*/

#ifndef __ASM_ARCH_REGS_FMC_H
#define __ASM_ARCH_REGS_FMC_H __FILE__

#define FMC_CE_TYPE				0x00
#define FMC_CE_CTRL				0x04
#define FMC_INTR_CTRL			0x08
#define FMC_CE0_CTRL			0x10
#define FMC_CE1_CTRL			0x14
#define FMC_CE2_CTRL			0x18
#define FMC_CE3_CTRL			0x1c
#define FMC_CE4_CTRL			0x20

#define FMC_CE0_ADDR			0x30
#define FMC_CE1_ADDR			0x34
#define FMC_CE2_ADDR			0x38
#define FMC_CE3_ADDR			0x3c
#define FMC_CE4_ADDR			0x40

#define FMC_MISC_CTRL1			0x50
#define FMC_MISC_CTRL2			0x54
#define FMC_NAND_CTRL			0x58
#define FMC_NAND_ECC			0x5c
#define FMC_NAND_ECC_CK1		0x60
#define FMC_NAND_ECC_CK2		0x64
#define FMC_NAND_ECC_CK3		0x68
#define FMC_NAND_ECC_GEN1		0x6c
#define FMC_NAND_ECC_GEN2		0x70
#define FMC_NAND_ECC_GEN3		0x74
#define FMC_NAND_ECC_CK_R1		0x78
#define FMC_NAND_ECC_CK_R2		0x7c
#define FMC_DMA_CTRL			0x80
#define FMC_DMA_FLASH_ADDR		0x84
#define FMC_DMA_DRAM_ADDR		0x88
#define FMC_DMA_LEN				0x8C
#define FMC_CHECK_SUM			0x90
#define FMC_SPI_TIMING			0x94





#endif /* __ASM_ARCH_REGS_FMC_H */
