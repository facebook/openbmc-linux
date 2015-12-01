/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-nor.c
* Author        : Ryan chen
* Description   : ASPEED NOR Device
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
*    1. 2012/08/01 ryan chen create this file
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
#include <plat/ast-scu.h>


/* --------------------------------------------------------------------
 *   NOR Flash
 * -------------------------------------------------------------------- */

#if defined(CONFIG_MTD_AST) || defined(CONFIG_MTD_AST_MODULE)
/*---------------------------------------------------------
 * AST2300 1 NOR * 16MB
 *--------------------------------------------------------*/
static struct mtd_partition nor_partitions[] = {
		/* bootloader (U-Boot, etc) in first sector */
		{
			  .name 			= "u-boot",
			  .offset			= 0,
			  .size 			= 0x80000,
			  .mask_flags		= MTD_WRITEABLE, /* force read-only */
		},
		/* kernel */
		{
			  .name 			= "kernel",
			  .offset			= MTDPART_OFS_APPEND,
			  .size 			= 0x300000,
		},
		/* file system */
		{
			  .name 			= "ramdisk",
			  .offset			= MTDPART_OFS_APPEND,
			  .size 			= 0x500000,
		},
		{
			  .name 			= "data",
			  .offset			= MTDPART_OFS_APPEND,
			  .size 			= MTDPART_SIZ_FULL,
		}
};

static struct flash_platform_data ast_nor_data = {
		.map_name		= "cfi_probe",
		.width			= 2,
		.parts			= nor_partitions,
		.nr_parts		= ARRAY_SIZE(nor_partitions),
};

#if defined(CONFIG_AST_CS0_NOR)
static struct resource ast_nor_resource0[] = {
	{
		.start			= AST_FMC_CS0_BASE,
		.end			= AST_FMC_CS0_BASE + AST_NOR_SIZE- 1,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nor_device0 = {
        .name           = "ast-nor",
        .id             = 0,
        .dev            = {
                .platform_data  = &ast_nor_data,
        },
        .num_resources  = ARRAY_SIZE(ast_nor_resource0),
        .resource       = ast_nor_resource0,
};
#endif

#if defined(CONFIG_AST_CS1_NOR)
static struct resource ast_nor_resource1[] = {
	{
		.start			= AST_FMC_CS1_BASE,
		.end			= AST_FMC_CS1_BASE + AST_NOR_SIZE- 1,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nor_device1 = {
        .name           = "ast-nor",
        .id             = 1,
        .dev            = {
                .platform_data  = &ast_nor_data,
        },
        .num_resources  = ARRAY_SIZE(ast_nor_resource1),
        .resource       = ast_nor_resource1,
};
#endif

#if defined(CONFIG_AST_CS2_NOR)
static struct resource ast_nor_resource2[] = {
	{
		.start			= AST_FMC_CS2_BASE,
		.end			= AST_FMC_CS2_BASE + AST_NOR_SIZE- 1,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nor_device2 = {
        .name           = "ast-nor",
        .id             = 2,
        .dev            = {
                .platform_data  = &ast_nor_data,
        },
        .num_resources  = ARRAY_SIZE(ast_nor_resource2),
        .resource       = ast_nor_resource2,
};
#endif

#if defined(CONFIG_AST_CS3_NOR)
static struct resource ast_nor_resource3[] = {
	{
		.start			= AST_FMC_CS3_BASE,
		.end			= AST_FMC_CS3_BASE + AST_NOR_SIZE- 1,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nor_device3 = {
        .name           = "ast-nor",
        .id             = 3,
        .dev            = {
                .platform_data  = &ast_nor_data,
        },
        .num_resources  = ARRAY_SIZE(ast_nor_resource3),
        .resource       = ast_nor_resource3,
};
#endif

#if defined(CONFIG_AST_CS4_NOR)
static struct resource ast_nor_resource4[] = {
	{
		.start			= AST_FMC_CS4_BASE,
		.end			= AST_FMC_CS4_BASE + AST_NOR_SIZE- 1,
		.flags			= IORESOURCE_MEM,
	},
};

static struct platform_device ast_nor_device4 = {
        .name           = "ast-nor",
        .id             = 4,
        .dev            = {
                .platform_data  = &ast_nor_data,
        },
        .num_resources  = ARRAY_SIZE(ast_nor_resource4),
        .resource       = ast_nor_resource4,
};
#endif

/*-------------------------------------*/
void __init ast_add_device_flash(void)
{
	void __iomem *fmc_regs = ioremap(AST_FMC_BASE, 4*SZ_16);

	ast_scu_multi_func_nor();

#if defined(CONFIG_AST_CS0_NOR)
	//Enable NOR ACK
	ast_scu_multi_func_romcs(0);
	platform_device_register(&ast_nor_device0);
	writel((readl(fmc_regs) | FMC_SET_WRITE_CS(0)) & FMC_MASK_TYPE_CS(0), fmc_regs);
#endif
#if defined(CONFIG_AST_CS1_NOR)
	ast_scu_multi_func_romcs(1);
	writel((readl(fmc_regs) | FMC_SET_WRITE_CS(1)) & FMC_MASK_TYPE_CS(1), fmc_regs);
	platform_device_register(&ast_nor_device1);
#endif
#if defined(CONFIG_AST_CS2_NOR)
	ast_scu_multi_func_romcs(2);
	writel((readl(fmc_regs) | FMC_SET_WRITE_CS(2)) & FMC_MASK_TYPE_CS(2), fmc_regs);
	platform_device_register(&ast_nor_device2);
#endif
#if defined(CONFIG_AST_CS3_NOR)
	ast_scu_multi_func_romcs(3);
	writel((readl(fmc_regs) | FMC_SET_WRITE_CS(3)) & FMC_MASK_TYPE_CS(3), fmc_regs);
	platform_device_register(&ast_nor_device3);
#endif
#if defined(CONFIG_AST_CS4_NOR)
	ast_scu_multi_func_romcs(4);
	writel((readl(fmc_regs) | FMC_SET_WRITE_CS(4)) & FMC_MASK_TYPE_CS(4), fmc_regs);
	platform_device_register(&ast_nor_device4);
#endif
	iounmap(fmc_regs);

}
#else
void __init ast_add_device_flash(void) {}
#endif
