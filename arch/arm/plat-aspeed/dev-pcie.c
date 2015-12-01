/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-pcie.c
* Author        : Ryan chen
* Description   : ASPEED PCIE
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
*    1. 2013/07/30 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>


#include <linux/platform_device.h>

/* --------------------------------------------------------------------
 *  PCEI
 * -------------------------------------------------------------------- */
#if defined(CONFIG_PCIE_AST)
static struct resource ast_pcie_resources[] = {
	{
		/* Register space */
		.name		= "pcie-regs",
		.start		= AST_PCIE_PLDA_BASE,
		.end			= AST_PCIE_PLDA_BASE + SZ_16K - 1,
		.flags		= IORESOURCE_MEM,
	},
#if 0	
	{
		/* Non-prefetch memory */
		.name		= "pcie-nonprefetch",
		.start		= AST_PCIE_WIN_BASE,
		.end			= AST_PCIE_WIN_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
#endif	
#if 0	
	{
		/* IO window */
		.name		= "pcie-io",
		.start		= AST_PCIE_IO_BASE,
		.end		= AST_PCIE_IO_BASE + SZ_2M + SZ_1M - 1,
		.flags		= IORESOURCE_IO,
	},
#endif
#if 0
	{
		/* Inbound memory window */
		.name		= "pcie-inbound0",
		.start		= AST_DRAM_BASE,
		.end			= AST_DRAM_BASE + SZ_2G - 1,
		.flags		= IORESOURCE_MEM,
	},
#endif	
};
	
static struct platform_device ast_pcie_device = {
	.name			= "ast-pcie",
	.id 				= 0,
	.resource			= ast_pcie_resources,
	.num_resources	= ARRAY_SIZE(ast_pcie_resources),	
};

void __init ast_add_device_pcie(void)
{
	platform_device_register(&ast_pcie_device);
}
#else
void __init ast_add_device_pcie(void) {}
#endif
