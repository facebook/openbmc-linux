/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-mctp.c
* Author        : Ryan chen
* Description   : ASPEED MCTP
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
 *  MCTP
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_MCTP) || defined(CONFIG_AST_MCTP_MODULE)
static u64 ast_mctp_dma_mask = 0xffffffffUL;

static struct resource ast_mctp_resource[] = {
	[0] = {
		.start = AST_MCTP_BASE,
		.end = AST_MCTP_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MCTP,
		.end = IRQ_MCTP,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = AST_DRAM_BASE,
		.end = AST_DRAM_BASE,
		.flags = IORESOURCE_BUS,
	},	
};

static struct platform_device ast_mctp_device = {
	.name	= "ast-mctp",
	.id = 0,
	.dev = {
		.dma_mask = &ast_mctp_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource = ast_mctp_resource,
	.num_resources = ARRAY_SIZE(ast_mctp_resource),
};

void __init ast_add_device_mctp(void)
{
	platform_device_register(&ast_mctp_device);
}
#else
void __init ast_add_device_mctp(void) {}
#endif

