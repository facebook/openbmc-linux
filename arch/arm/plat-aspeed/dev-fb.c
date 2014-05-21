/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-fb.c
* Author        : Ryan Chen
* Description   : ASPEED Frambuffer Driver
*
* Copyright (C) ASPEED Tech. Inc.
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

*   History      :
*    1. 2012/12/15 Ryan Chen initial
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>

#include <plat/devs.h>
#include <plat/ast-scu.h>

#include <mach/ast_lcd.h>

/* --------------------------------------------------------------------
 *  ASPEED FB
 * -------------------------------------------------------------------- */

#if defined(CONFIG_FB_AST) || defined(CONFIG_FB_AST_MODULE)
static struct ast_fb_plat_data fb_plat_data = {
	.get_clk = ast_get_d2_pll_clk,
};


static struct resource ast_fb_resources[] = {
	[0] = {
		.start = AST_GRAPHIC_BASE,
		.end = AST_GRAPHIC_BASE + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CRT,
		.end = IRQ_CRT,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_device_fb_dmamask = 0xffffffffUL;
struct platform_device ast_fb_device = {
	.name		  = "ast-fb",
	.id		  = 0,
	.dev		= {
		.dma_mask		= &ast_device_fb_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
		.platform_data= &fb_plat_data,
	},
	.resource	= ast_fb_resources,
	.num_resources	= ARRAY_SIZE(ast_fb_resources),
};

void __init ast_add_device_fb(void)
{
	ast_scu_multi_func_crt();

	ast_scu_init_crt();

	platform_device_register(&ast_fb_device);
}
#else
void __init ast_add_device_fb(void) {}
#endif
