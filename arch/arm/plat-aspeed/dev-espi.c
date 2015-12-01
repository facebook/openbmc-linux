/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-espi.c
* Author        : Ryan chen
* Description   : ASPEED eSPI Controller
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
//#include <plat/ast-espi.h>

/* --------------------------------------------------------------------
 *  eSPI
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_ESPI) || defined(CONFIG_AST_ESPI_MODULE)
static struct resource ast_espi_resource[] = {
	[0] = {
		.start = AST_ESPI_BASE,
		.end = AST_ESPI_BASE + SZ_256 -1,
		.flags = IORESOURCE_MEM,
	},
#ifdef CONFIG_COLDFIRE_ESPI	
	[1] = {
		.start = IRQ_CPU,
		.end = IRQ_CPU, 
		.flags = IORESOURCE_IRQ,
	},
#else
	[1] = {
		.start = IRQ_ESPI,
		.end = IRQ_ESPI,
		.flags = IORESOURCE_IRQ,
	},
#endif
};

static u64 ast_espi_dma_mask = 0xffffffffUL;

static struct platform_device ast_espi_device = {
	.name	= "ast-espi",
	.id = 0,
	.dev = {
		.dma_mask = &ast_espi_dma_mask,
		.coherent_dma_mask = 0xffffffff,
//		.platform_data = &ast_espi_info,
	},
	.resource = ast_espi_resource,
	.num_resources = ARRAY_SIZE(ast_espi_resource),
};

void __init ast_add_device_espi(void)
{
	platform_device_register(&ast_espi_device);
}
#else
void __init ast_add_device_espi(void) {}
#endif
