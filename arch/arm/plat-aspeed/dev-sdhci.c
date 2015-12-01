/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-sdhc.c
* Author        : Ryan chen
* Description   : ASPEED SDHC Device
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
*    1. 2012/07/30 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast_sdhci.h>
#include <plat/ast-scu.h>

extern void ast_sd_set_8bit_mode(u8 mode);

/* --------------------------------------------------------------------
 *  SDHC
 * -------------------------------------------------------------------- */
#if defined(CONFIG_MMC_AST) || defined(CONFIG_MMC_AST_MODULE)
static struct ast_sdhc_platform_data ast_sdhc_info = {
        .sd_clock_src_get = ast_get_sd_clock_src,
        .sd_set_8bit_mode = ast_sd_set_8bit_mode,
};

static struct resource ast_sdhci0_resource[] = {
	[0] = {
		.start = AST_SDHC0_BASE,
		.end = AST_SDHC0_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDHCI_SLOT0,
		.end = IRQ_SDHCI_SLOT0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource ast_sdhci1_resource[] = {
	[0] = {
		.start = AST_SDHC1_BASE,
		.end = AST_SDHC1_BASE + SZ_256 - 1 ,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDHCI_SLOT1,
		.end = IRQ_SDHCI_SLOT1,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_sdhc_dma_mask = 0xffffffffUL;

static struct platform_device ast_sdhci_device0 = {
	.name	= "ast_sdhci",
    .id = 0,
    .dev = {
            .dma_mask = &ast_sdhc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
            .platform_data = &ast_sdhc_info,
    },
	.resource = ast_sdhci0_resource,
	.num_resources = ARRAY_SIZE(ast_sdhci0_resource),
};

static struct platform_device ast_sdhci_device1 = {
	.name	= "ast_sdhci",
    .id = 1,
    .dev = {
            .dma_mask = &ast_sdhc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
            .platform_data = &ast_sdhc_info,
    },
	.resource = ast_sdhci1_resource,
	.num_resources = ARRAY_SIZE(ast_sdhci1_resource),
};

void __init ast_add_device_sdhci(void)
{
	//multipin. Remind: AST3200FPGA only supports one port at a time
#ifdef CONFIG_8BIT_MODE
	ast_scu_multi_func_sdhc_8bit_mode();
	platform_device_register(&ast_sdhci_device0);
#else
	platform_device_register(&ast_sdhci_device0);
	platform_device_register(&ast_sdhci_device1);
#endif	
	ast_scu_multi_func_sdhc_slot(3);
}
#else
void __init ast_add_device_sdhci(void) {}
#endif
