/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-formatter.c
* Author        : Ryan chen
* Description   : 
*
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
*    1. 2014/09/15 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>

/* --------------------------------------------------------------------
 *  RFX
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_FORMATTER) || defined(CONFIG_AST_FORMATTER_MODULE)
static struct resource ast_formatter_resource[] = {
	[0] = {
		.start = AST_FORMATTER_BASE,
		.end   = AST_FORMATTER_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_FORMATTER,
		.end   = IRQ_FORMATTER,
		.flags = IORESOURCE_IRQ,
	},	
	[2] = {
		.start = AST_FORMATTER_MEM_BASE,
		.end = AST_FORMATTER_MEM_BASE + AST_FORMATTER_MEM_SIZE - 1,
		.flags = IORESOURCE_DMA,
	},	
};

static u64 ast_device_formatter_dmamask = 0xffffffffUL;
struct platform_device ast_device_formatter = {
	.name		 	= "ast-formatter",
	.dev = {
		.dma_mask			= &ast_device_formatter_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_formatter_resource),
	.resource			= ast_formatter_resource,
};

void __init ast_add_device_formatter(void)
{
//	ast_scu_init_formatter();
	platform_device_register(&ast_device_formatter);
}
#else
void __init ast_add_device_formatter(void) {}
#endif
