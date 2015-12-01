/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-xdma.c
* Author        : Ryan chen
* Description   : ASPEED XDMA
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

/* --------------------------------------------------------------------
 *  XDMA
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_XDMA) || defined(CONFIG_AST_XDMA_MODULE)
static u64 ast_xdma_dma_mask = 0xffffffffUL;

static struct resource ast_xdma_resource[] = {
	[0] = {
		.start = AST_XDMA_BASE,
		.end = AST_XDMA_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_XDMA,
		.end = IRQ_XDMA,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_xdma_device = {
	.name	= "ast-xdma",
	.id = 0,
	.dev = {
		.dma_mask = &ast_xdma_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource = ast_xdma_resource,
	.num_resources = ARRAY_SIZE(ast_xdma_resource),
};

void __init ast_add_device_xdma(void)
{
	platform_device_register(&ast_xdma_device);
}
#else
void __init ast_add_device_xdma(void) {}
#endif

