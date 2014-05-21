/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-nand.c
* Author        : Ryan chen
* Description   : ASPEED NAND Device
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/10/15 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/regs-fmc.h>
#include <asm/io.h>

#include <linux/mtd/nand.h>

#include <plat/ast-scu.h>
#include <linux/mtd/mtd.h>



/* --------------------------------------------------------------------
 *   NAND Flash
 * -------------------------------------------------------------------- */

#if defined(CONFIG_MTD_NAND_AST) || defined(CONFIG_MTD_NAND_AST_MODULE)
static void __iomem *fmc_regs;

/* returns 0 if the nand is busy, returns 1 if the nand is ready */
static int 
ast_nand_dev_ready(struct mtd_info *mtd)
{
	int status;
	status = (readl(fmc_regs + FMC_MISC_CTRL1) & READ_BUSY_PIN_STS) >> 3;
	return status;
}

/* We use 2 256bytes as ECC's data length in sample code */
static void 
ast_enable_hwecc(struct mtd_info *mtd, int cmd)
{
	writel(NAND_ECC_DATA_BLK_256 | NAND_ECC_ENABLE , fmc_regs + FMC_NAND_ECC);

	writel(NAND_ECC_RESET , fmc_regs + FMC_NAND_ECC);
	
	writel(NAND_ECC_DATA_BLK_256 | NAND_ECC_ENABLE , fmc_regs + FMC_NAND_ECC);
}

static int 
ast_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	uint32_t ecc_1, ecc_2, ecc_3;

	ecc_1 = readl(fmc_regs + FMC_NAND_ECC_GEN1);
	ecc_2 = readl(fmc_regs + FMC_NAND_ECC_GEN2);	
	ecc_3 = readl(fmc_regs + FMC_NAND_ECC_GEN3);	

	ecc_code[0] = ecc_1;
	ecc_code[1] = ecc_1 >> 8;
	ecc_code[2] = ecc_1 >> 16;
	ecc_code[3] = ecc_1 >> 24;
	ecc_code[4] = ecc_2;
	ecc_code[5] = (((ecc_2 >> 8) & 0x0F) | 0xF0);	//Becase flash's data value will be 0xff after flash is erased. The 256bytes mode will use 44bits to do ECC, the software needs to add 0xF0 for the last 4 bits.

	return 0;
}

static int 
ast_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	unsigned int dw_read_data[3], dw_calc_data[3];
	unsigned int data1_check_status, data2_check_status;
	unsigned int i, ecc_position, ecc_bit;

	for (i = 0; i < 3; i++) {
		dw_read_data[i] = 0;
		dw_calc_data[i] = 0;
	}
	memcpy (dw_read_data, read_ecc, 6);
	memcpy (dw_calc_data, calc_ecc, 6);
	for (i = 0; i < 2; i++) {
		writel(dw_read_data[i], fmc_regs + FMC_NAND_ECC_CK1 + (i*4));
		writel(dw_calc_data[i], fmc_regs + FMC_NAND_ECC_GEN1 + (i*4));
	}

	data1_check_status = readl(fmc_regs + FMC_NAND_ECC_CK_R1) & 0xffff;
	data2_check_status = (readl(fmc_regs + FMC_NAND_ECC_CK_R1) & 0xffff0000) >> 16;

	if ((data1_check_status & 0x1000) && (data2_check_status & 0x1000)) {
		return 0;
	}

	if ((data1_check_status & 0x8000) || (data2_check_status & 0x8000)) {
		printk(KERN_ERR "uncorrectable error : ");
		return -1;
	}

	if ((data1_check_status & 0x4000) || (data2_check_status & 0x4000)) {
		printk ("error in ecc data\n");
		return 1;	/* error in ecc data; no action needed */
	}

//Correctable
	if (data1_check_status & 0x2000) {
		printk ("correctable in data area 1\n");
		ecc_position = (data1_check_status & 0xff8) >> 3;
		ecc_bit = (data1_check_status & 0x07);
		dat[ecc_position] ^= (1 << ecc_bit);
	}
	if (data2_check_status & 0x2000) {
		printk ("correctable in data area 2\n");
		ecc_position = (data2_check_status & 0xff8) >> 3;
		ecc_bit = (data2_check_status & 0x07);
		dat[128 + ecc_position] ^= (1 << ecc_bit);
	}

	return 1;
}

/*---------------------------------------------------------
 * AST2300 1 NAND * 128MB
 *--------------------------------------------------------*/

static struct mtd_partition ast_nand_partition_info[] = {
	[0] = {
		.name	= "ASPEED NAND Flash 0",
		.offset	= 0,
		.size	= SZ_64M,
	},
	[1] = {
		.name	= "ASPEED NAND Flash 1",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL
	},
};

static const char *ast_nand_part_probes[] = { "cmdlinepart", NULL };

struct platform_nand_data ast_nand_platdata = {
	.chip = {
		.nr_chips = 1,
		.chip_offset = 0,
		.nr_partitions = ARRAY_SIZE(ast_nand_partition_info),
		.partitions = ast_nand_partition_info,
		/* 50 us command delay time */
		.chip_delay = 50,
		.options = NAND_NO_AUTOINCR,
		.part_probe_types = ast_nand_part_probes,
	},
	.ctrl = {
		.hwcontrol = ast_enable_hwecc,
		.dev_ready = ast_nand_dev_ready,
		.select_chip = 0,
		.calculate = ast_calculate_ecc,
		.correct = ast_nand_correct_data,
	},
};

#if defined(CONFIG_AST_CS0_NAND)
static struct resource ast_nand_resource0[] = {
	{
		.start			= AST_CS0_DEF_BASE,
		.end			= AST_CS0_DEF_BASE + 0x10000,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nand_device0 = {
        .name           = "ast-nand",
        .id             = 0,
        .dev            = {
                .platform_data  = &ast_nand_platdata,
        },
        .num_resources  = ARRAY_SIZE(ast_nand_resource0),
        .resource       = ast_nand_resource0,
};
#endif

#if defined(CONFIG_AST_CS1_NAND)
static struct resource ast_nand_resource1[] = {
	{
		.start			= AST_CS1_DEF_BASE,
		.end			= AST_CS1_DEF_BASE + 0x10000,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nand_device1 = {
        .name           = "ast-nand",
        .id             = 1,
        .dev            = {
                .platform_data  = &ast_nand_platdata,
        },
        .num_resources  = ARRAY_SIZE(ast_nand_resource1),
        .resource       = ast_nand_resource1,
};
#endif

#if defined(CONFIG_AST_CS2_NAND)
static struct resource ast_nand_resource2[] = {
	{
		.start			= AST_CS2_DEF_BASE,
		.end			= AST_CS2_DEF_BASE + 0x10000,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nand_device2 = {
        .name           = "ast-nand",
        .id             = 2,
        .dev            = {
                .platform_data  = &ast_nand_platdata,
        },
        .num_resources  = ARRAY_SIZE(ast_nand_resource2),
        .resource       = ast_nand_resource2,
};
#endif

#if defined(CONFIG_AST_CS3_NAND)
static struct resource ast_nand_resource3[] = {
	{
		.start			= AST_CS3_DEF_BASE,
		.end			= AST_CS3_DEF_BASE + 0x10000,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nand_device3 = {
        .name           = "ast-nand",
        .id             = 3,
        .dev            = {
                .platform_data  = &ast_nand_platdata,
        },
        .num_resources  = ARRAY_SIZE(ast_nand_resource3),
        .resource       = ast_nand_resource3,
};
#endif

#if defined(CONFIG_AST_CS4_NAND)
static struct resource ast_nand_resource4[] = {
	{
		.start			= AST_CS4_DEF_BASE,
		.end			= AST_CS4_DEF_BASE + 0x10000,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nand_device4 = {
        .name           = "ast-nand",
        .id             = 4,
        .dev            = {
                .platform_data  = &ast_nand_platdata,
        },
        .num_resources  = ARRAY_SIZE(ast_nand_resource4),
        .resource       = ast_nand_resource4,
};
#endif

/*-------------------------------------*/
void __init ast_add_device_nand(void)
{
	u32 tmp;
	fmc_regs = ioremap(AST_FMC_BASE, 4*SZ_16);

	ast_scu_multi_func_nand();
	writel(0x31 , fmc_regs + FMC_MISC_CTRL1);

#if defined(CONFIG_AST_CS0_NAND)
	platform_device_register(&ast_nand_device0);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(0)) & FMC_MASK_TYPE_CS(0);
	writel( tmp | FMC_SET_TYPE_NAND_CS(0), fmc_regs);
	writel(0x9 , fmc_regs + FMC_CE0_CTRL);	
#endif

#if defined(CONFIG_AST_CS1_NAND)
	ast_scu_multi_func_romcs(1);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(1)) & FMC_MASK_TYPE_CS(1);
	writel( tmp | FMC_SET_TYPE_NAND_CS(1), fmc_regs);
	writel(0x9 , fmc_regs + FMC_CE1_CTRL);	
	platform_device_register(&ast_nand_device1);
#endif
#if defined(CONFIG_AST_CS2_NAND)
	ast_scu_multi_func_romcs(2);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(2)) & FMC_MASK_TYPE_CS(2);
	writel( tmp | FMC_SET_TYPE_NAND_CS(2), fmc_regs);
	writel(0x9 , fmc_regs + FMC_CE2_CTRL);	
	platform_device_register(&ast_nand_device2);
#endif
#if defined(CONFIG_AST_CS3_NAND)
	ast_scu_multi_func_romcs(3);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(3)) & FMC_MASK_TYPE_CS(3);
	writel( tmp | FMC_SET_TYPE_NAND_CS(3), fmc_regs);
	writel(0x9 , fmc_regs + FMC_CE3_CTRL);
	platform_device_register(&ast_nand_device3);
#endif
#if defined(CONFIG_AST_CS4_NAND)
	ast_scu_multi_func_romcs(4);
	tmp = (readl(fmc_regs) | FMC_SET_WRITE_CS(4)) & FMC_MASK_TYPE_CS(4);
	writel( tmp | FMC_SET_TYPE_NAND_CS(4), fmc_regs);
	writel(0x9 , fmc_regs + FMC_CE4_CTRL);	
	platform_device_register(&ast_nand_device4);
#endif
	iounmap(fmc_regs);

}
#else
void __init ast_add_device_nand(void) {}
#endif
