/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-lpc.c
* Author        : Ryan chen
* Description   : ASPEED LPC Controller
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

#include <asm/io.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>
#include <plat/regs-lpc.h>

static u32 ast_lpc_base = IO_ADDRESS(AST_LPC_BASE);

/* --------------------------------------------------------------------
 *  LPC
 * -------------------------------------------------------------------- */
#if defined(CONFIG_LPC) || defined(CONFIG_LPC_MODULE)
static struct resource ast_lpc_resource[] = {
	[0] = {
		.start = AST_LPC_BASE,
		.end = AST_LPC_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LPC,
		.end = IRQ_LPC,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_lpc_dma_mask = 0xffffffffUL;

static struct platform_device ast_lpc_device = {
	.name	= "ast_lpc",
    .id = 0,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_lpc_resource,
	.num_resources = ARRAY_SIZE(ast_lpc_resource),
};

static struct resource ast_lpc_plus_resource[] = {
	[0] = {
		.start = AST_LPC_PLUS_BASE,
		.end = AST_LPC_PLUS_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ast_lpc_plus_device = {
	.name	= "ast_lpc_plus",
    .id = 1,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_lpc_plus_resource,
	.num_resources = ARRAY_SIZE(ast_lpc_plus_resource),
};

void __init ast_add_device_lpc(void)
{
//	it should enable at u-boot
//	ast_scu_init_lpc();

	platform_device_register(&ast_lpc_device);
	platform_device_register(&ast_lpc_plus_device);
}
#else
void __init ast_add_device_lpc(void) {
  // Since we disable LPC, bring the UART1 and UART2 out from LPC control
  writel((readl(ast_lpc_base + AST_LPC_HICR9)
	  & ~(LPC_HICR9_SOURCE_UART1|LPC_HICR9_SOURCE_UART2
	      |LPC_HICR9_SOURCE_UART3|LPC_HICR9_SOURCE_UART4)),
	 ast_lpc_base + AST_LPC_HICR9);
}
#endif

