/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-hid.c
* Author        : Ryan chen
* Description   : ASPEED USB Device 1.1
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
 *  UDC 1.1
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_HID) || defined(CONFIG_AST_HID_MODULE)
/* hid descriptor for a keyboard */

static struct resource ast_hid_resource[] = {
	[0] = {
		.start = AST_UDC11_BASE,
		.end = AST_UDC11_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UDC11,
		.end = IRQ_UDC11,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_hid_dma_mask = 0xffffffffUL;

static struct platform_device ast_hid_device = {
	.name	= "ast-hid",
    .id = 0,
    .dev = {
            .dma_mask = &ast_hid_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_hid_resource,
	.num_resources = ARRAY_SIZE(ast_hid_resource),
};

void __init ast_add_device_hid(void)
{
	ast_scu_multi_func_usb_port2_mode(0);

//	ast_scu_init_usb_port1();

	platform_device_register(&ast_hid_device);
}
#else
void __init ast_add_device_hid(void) {}
#endif
