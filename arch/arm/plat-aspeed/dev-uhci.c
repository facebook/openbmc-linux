/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-uhci.c
* Author        : Ryan chen
* Description   : ASPEED EHCI Device
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
*    1. 2012/07/30 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>

/* --------------------------------------------------------------------
 *  UHCI
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_USB_UHCI_HCD) || defined(CONFIG_AST_USB_UHCI_HCD_MODULE)
static struct resource ast_uchi_resource[] = {
	[0] = {
		.start = AST_UHCI_BASE,
		.end = AST_UHCI_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_UHCI,
		.end = IRQ_UHCI,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_uhci_dma_mask = 0xffffffffUL;

static struct platform_device ast_uhci_device = {
	.name	= "ast_uhci",
    .id = 0,
    .dev = {
            .dma_mask = &ast_uhci_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_uchi_resource,
	.num_resources = ARRAY_SIZE(ast_uchi_resource),
};

void __init ast_add_device_uhci(void)
{

#if defined (CONFIG_AST_USB_UHCI_MULTIPORT_2)
	ast_scu_multi_func_usb11_host_port2(1);
#elif defined (CONFIG_AST_USB_UHCI_MULTIPORT_4)
	ast_scu_multi_func_usb11_host_port2(1);
	ast_scu_multi_func_usb11_host_port4(1);
#else
	ast_scu_multi_func_usb11_host_port2(0);
	ast_scu_multi_func_usb11_host_port4(0);
#endif

	ast_scu_init_uhci();

	platform_device_register(&ast_uhci_device);
}
#else
void __init ast_add_device_uhci(void) {}
#endif
