/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-ehci.c
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
 *  EHCI
 * -------------------------------------------------------------------- */
#if defined(CONFIG_USB_EHCI_AST) || defined(CONFIG_USB_EHCI_AST_MODULE)
static struct resource ast_echi_resource[] = {
	[0] = {
		.start = AST_EHCI_BASE,
		.end = AST_EHCI_BASE + SZ_256,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EHCI,
		.end = IRQ_EHCI,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_ehci_dma_mask = 0xffffffffUL;

static struct platform_device ast_ehci_device = {
	.name	= "ehci-ast",
    .id = 0,
    .dev = {
            .dma_mask = &ast_ehci_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_echi_resource,
	.num_resources = ARRAY_SIZE(ast_echi_resource),
};

void __init ast_add_device_ehci(void)
{
	ast_scu_multi_func_usb20_host_hub(1);
	ast_scu_init_usb20();

	platform_device_register(&ast_ehci_device);
}
#else
void __init ast_add_device_ehci(void) {}
#endif
