/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-udc20.c
* Author        : Ryan chen
* Description   : ASPEED USB Device 2.0
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
#include <linux/usb/g_hid.h>

/* --------------------------------------------------------------------
 *  VHUB
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_VHUB)
static struct resource ast_vhub_resource[] = {
	[0] = {
		.start = AST_VHUB_BASE,
		.end = AST_VHUB_BASE + SZ_1K,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VHUB,
		.end = IRQ_VHUB,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_vhub_dma_mask = 0xffffffffUL;

static struct platform_device ast_vhub_device = {
	.name	= "ast-vhub",
	.id = 0,
	.dev = {
		.dma_mask = &ast_vhub_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	},
	.resource = ast_vhub_resource,
	.num_resources = ARRAY_SIZE(ast_vhub_resource),
};

void __init ast_add_device_vhub(void)
{
	ast_scu_multi_func_usb_port1_mode(0);
	ast_scu_init_usb_port1();

	platform_device_register(&ast_vhub_device);
}
#else
void __init ast_add_device_vhub(void) {}
#endif
