/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-clpc.c
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
 *  LPC
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_CLPC) || defined(CONFIG_AST_CLPC_MODULE)
static u64 ast_lpc_dma_mask = 0xffffffffUL;

static struct resource ast_clpc0_resource[] = {
	[0] = {
		.start = AST_CLPC1_BASE,
		.end = AST_CLPC1_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_N1_KCS,
		.end = IRQ_N1_KCS,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_N1_UART,
		.end = IRQ_N1_UART,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_N1_MAILBOX,
		.end = IRQ_N1_MAILBOX,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N1_PORT80,
		.end = IRQ_N1_PORT80,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N1_RESET,
		.end = IRQ_N1_RESET,
		.flags = IORESOURCE_IRQ,
	},	
};

static struct platform_device ast_clpc0_device = {
	.name	= "ast_lpc",
    .id = 0,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_clpc0_resource,
	.num_resources = ARRAY_SIZE(ast_clpc0_resource),
};

static struct resource ast_clpc1_resource[] = {
	[0] = {
		.start = AST_CLPC2_BASE,
		.end = AST_CLPC2_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_N2_KCS,
		.end = IRQ_N2_KCS,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_N2_UART,
		.end = IRQ_N2_UART,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_N2_MAILBOX,
		.end = IRQ_N2_MAILBOX,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N2_PORT80,
		.end = IRQ_N2_PORT80,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N2_RESET,
		.end = IRQ_N2_RESET,
		.flags = IORESOURCE_IRQ,
	},	
};

static struct platform_device ast_clpc1_device = {
	.name	= "ast_lpc",
    .id = 1,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_clpc1_resource,
	.num_resources = ARRAY_SIZE(ast_clpc1_resource),
};

static struct resource ast_clpc2_resource[] = {
	[0] = {
		.start = AST_CLPC3_BASE,
		.end = AST_CLPC3_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_N3_KCS,
		.end = IRQ_N3_KCS,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_N3_UART,
		.end = IRQ_N3_UART,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_N3_MAILBOX,
		.end = IRQ_N3_MAILBOX,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N3_PORT80,
		.end = IRQ_N3_PORT80,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N3_RESET,
		.end = IRQ_N3_RESET,
		.flags = IORESOURCE_IRQ,
	},	
};

static struct platform_device ast_clpc2_device = {
	.name	= "ast_lpc",
    .id = 2,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_clpc2_resource,
	.num_resources = ARRAY_SIZE(ast_clpc2_resource),
};

static struct resource ast_clpc3_resource[] = {
	[0] = {
		.start = AST_CLPC4_BASE,
		.end = AST_CLPC4_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_N4_KCS,
		.end = IRQ_N4_KCS,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_N4_UART,
		.end = IRQ_N4_UART,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_N4_MAILBOX,
		.end = IRQ_N4_MAILBOX,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N4_PORT80,
		.end = IRQ_N4_PORT80,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_N4_RESET,
		.end = IRQ_N4_RESET,
		.flags = IORESOURCE_IRQ,
	},	
};

static struct platform_device ast_clpc3_device = {
	.name	= "ast_lpc",
    .id = 3,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_clpc3_resource,
	.num_resources = ARRAY_SIZE(ast_clpc3_resource),
};

void __init ast_add_device_clpc(void)
{
//	it should enable at u-boot
//	ast_scu_init_lpc();

	platform_device_register(&ast_clpc0_device);
	platform_device_register(&ast_clpc1_device);
	platform_device_register(&ast_clpc2_device);
	platform_device_register(&ast_clpc3_device);	
}
#else
void __init ast_add_device_clpc(void) {}
#endif

