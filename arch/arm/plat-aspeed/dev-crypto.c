/********************************************************************************
* File Name     : dev-crypto.c
* Author        : Ryan Chen
* Description   : AST CRYPTO Device
*
* Copyright (C)  ASPEED Technology Inc.
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
*    1. 2014/12/15 Ryan Chen initial
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/platform_device.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/sizes.h>

#include <asm/arch/irqs.h>
#include <asm/arch/platform.h>
#include <asm/arch/devs.h>
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#endif


/* --------------------------------------------------------------------
 *  Crypto
 * -------------------------------------------------------------------- */

#if defined(CONFIG_CRYPTO_DEV_AST) || defined(CONFIG_CRYPTO_DEV_AST_MODULE) || defined(CONFIG_AST_CRYPTO)

static u64 ast_crypto_dma_mask = 0xffffffffUL;

static struct resource ast_crypto_resource[] = {
	[0] = {
		.start	= AST_CRYPTO_BASE,
		.end	= AST_CRYPTO_BASE + SZ_128 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
			.start = IRQ_CRYPTO,
			.end = IRQ_CRYPTO,
			.flags = IORESOURCE_IRQ,
	},
	
};

static struct platform_device ast_device_crypto = {
	.name		= "ast-crypto",
	.id		= 0,
	.dev = {
			.dma_mask = &ast_crypto_dma_mask,
			.coherent_dma_mask = 0xffffffff,
	},
	.resource       = ast_crypto_resource,
	.num_resources  = ARRAY_SIZE(ast_crypto_resource),
};

void __init ast_add_device_crypto(void)
{
	platform_device_register(&ast_device_crypto);
}
#else
void __init ast_add_device_crypto(void) {}
#endif
