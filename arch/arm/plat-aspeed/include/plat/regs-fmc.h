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

/* FMC_CE_TYPE				0x00 */
#define FMC_SET_WRITE_CS(x)		(0x1 << (x+16))
#define FMC_MASK_TYPE_CS(x)		(~(0x3 << (2*x)))
#define FMC_SET_TYPE_NAND_CS(x)	(0x1 << (2*x))
#define FMC_SET_TYPE_SPI_CS(x)	(0x2 << (2*x))	

#define FMC_TYPE_NOR 			0
#define FMC_TYPE_NAND 			1
#define FMC_TYPE_SPI 			2


/* FMC_CE0_CTRL	for NAND 0x10, 0x14, 0x18, 0x1c, 0x20 */
#define NAND_T_WEH(x)			(x << 28)
#define NAND_T_WEL(x)			(x << 24)
#define NAND_T_REH(x)			(x << 20)
#define NAND_T_REL(x)			(x << 16)
#define NAND_T_CESH(x)			(x << 12)
#define NAND_T_WTR(x)			(x << 10)
#define NAND_T_R(x)				(x << 4)
#define NAND_ADDR_CYCLE			(1 << 3)
#define NAND_CE_ACTIVE			(1 << 2)
#define NAND_OP_MODE				(1 << 0)

/* FMC_CE0_CTRL	for SPI 0x10, 0x14, 0x18, 0x1c, 0x20 */
#define SPI_IO_MODE(x)			(x << 28)
#define SPI_CE_WIDTH(x)			(x << 24)
#define SPI_CMD_DATA(x)			(x << 16)
#define SPI_DUMMY_CMD			(1 << 15)
#define SPI_DUMMY_HIGH			(1 << 14)
#define SPI_CLK_DIV				(1 << 13)
#define SPI_ADDR_CYCLE			(1 << 13)
#define SPI_CMD_MERGE_DIS		(1 << 12)
#define SPI_T_CLK				(x << 8)
#define SPI_DUMMY_LOW			(x << 6)
#define SPI_LSB_FIRST_CTRL		(1 << 5)
#define SPI_CPOL_1				(1 << 4)
#define SPI_DUAL_DATA			(1 << 3)
#define SPI_CE_INACTIVE			(1 << 2)
#define SPI_CMD_MODE			(x)
#define SPI_CMD_NOR_R_MODE		0
#define SPI_CMD_FAST_R_MODE		1
#define SPI_CMD_NOR_W_MODE		2
#define SPI_CMD_USER_MODE		3


/* FMC_CE0_ADDR			0x30 0x34 0x38 0x3c 0x40*/
#define FMC_END_ADDR(x)			(x << 24)
#define FMC_START_ADDR(x)		(x << 16)


/* FMC_MISC_CTRL1			0x50 */
#define READ_BUSY_PIN_STS		(1 << 3)

/* FMC_NAND_ECC			0x5c */
#define NAND_ECC_RESET			(1 << 3)
#define NAND_ECC_ENABLE			(1 << 2)
#define NAND_ECC_DATA_BLK_512	2
#define NAND_ECC_DATA_BLK_256	1
#define NAND_ECC_DATA_BLK_128	0



#endif /* __ASM_ARCH_REGS_FMC_H */
