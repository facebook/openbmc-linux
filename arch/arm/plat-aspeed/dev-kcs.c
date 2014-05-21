/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-kcs.c
* Author        : Ryan chen
* Description   : ASPEED KCS
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
*    1. 2012/11/29 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>

/* --------------------------------------------------------------------
 *  KCS
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_KCS) || defined(CONFIG_AST_KCS_MODULE)
static u64 ast_kcs_dma_mask = 0xffffffffUL;

static struct resource ast_kcs0_resource[] = {
	[0] = {
		.start = AST_LPC_BASE,
		.end = AST_LPC_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LPC,
		.end = IRQ_LPC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_kcs0_device = {
	.name	= "ast-kcs",
    .id = 0,
    .dev = {
            .dma_mask = &ast_kcs_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_kcs0_resource,
	.num_resources = ARRAY_SIZE(ast_kcs0_resource),
};

static struct resource ast_kcs1_resource[] = {
	[0] = {
		.start = AST_LPC_BASE,
		.end = AST_LPC_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LPC,
		.end = IRQ_LPC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_kcs1_device = {
	.name	= "ast-kcs",
    .id = 1,
    .dev = {
            .dma_mask = &ast_kcs_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_kcs1_resource,
	.num_resources = ARRAY_SIZE(ast_kcs1_resource),
};

static struct resource ast_kcs2_resource[] = {
	[0] = {
		.start = AST_LPC_BASE,
		.end = AST_LPC_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LPC,
		.end = IRQ_LPC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_kcs2_device = {
	.name	= "ast-kcs",
    .id = 2,
    .dev = {
            .dma_mask = &ast_kcs_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_kcs2_resource,
	.num_resources = ARRAY_SIZE(ast_kcs2_resource),
};

void __init ast_add_device_kcs(void)
{
//	platform_device_register(&ast_kcs0_device);
//	platform_device_register(&ast_kcs1_device);
	platform_device_register(&ast_kcs2_device);
}
#else
void __init ast_add_device_kcs(void) {}
#endif

