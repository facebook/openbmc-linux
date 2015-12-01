/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-rfx.c
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

#include <plat/ast-scu.h>

/* --------------------------------------------------------------------
 *  REMOTEFX
 * -------------------------------------------------------------------- */
 static u64 ast_device_rfx_dmamask = 0xffffffffUL;
#if defined(CONFIG_AST_RFX)

#if defined(CONFIG_AST_ENTROPY) || defined(CONFIG_AST_ENTROPY_MODULE)
static struct resource ast_entropy_resource[] = {
	[0] = {
		.start = AST_ENTROPY_BASE,
		.end   = AST_ENTROPY_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},	
	[1] = {
		.start = IRQ_EGFX,
		.end   = IRQ_EGFX,
		.flags = IORESOURCE_IRQ,
	},	
	[2] = {
		.start = AST_ENTROPY_MEM_BASE,
		.end = AST_ENTROPY_MEM_BASE + AST_ENTROPY_MEM_SIZE - 1,
		.flags = IORESOURCE_DMA,
	},	
};

struct platform_device ast_device_entropy = {
	.name		 	= "ast-entropy",
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_entropy_resource),
	.resource			= ast_entropy_resource,
};
#endif

#if defined(CONFIG_AST_BULK) || defined(CONFIG_AST_BULK_MODULE)
static struct resource ast_bulk_resource[] = {
	[0] = {
		.start = AST_BULK_BASE,
		.end   = AST_BULK_BASE + SZ_128 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_BULK_INT0,
		.end   = IRQ_BULK_INT0,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = AST_BULK_STREAM_MEM_BASE,
		.end = AST_BULK_STREAM_MEM_BASE + AST_BULK_STREAM_MEM_SIZE - 1,
		.flags = IORESOURCE_DMA,
	},		
};

struct platform_device ast_device_bulk = {
	.name		 	= "ast-bulk",
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_bulk_resource),
	.resource			= ast_bulk_resource,
};
#endif

/* --------------------------------------------------------------------
 *  cmdq
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_CMDQ) || defined(CONFIG_AST_CMDQ_MODULE)
static struct resource ast_cmdq_resource[] = {
	[0] = {
		.start = AST_CMDQ_BASE,
		.end   = AST_CMDQ_BASE + SZ_128 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CMDQ_DONE,
		.end   = IRQ_CMDQ_DONE,
		.flags = IORESOURCE_IRQ,
	},	
	[2] = {
		.start = IRQ_CMDQ_ENG_DONE,
		.end   = IRQ_CMDQ_ENG_DONE,
		.flags = IORESOURCE_IRQ,
	},	
	[3] = {
		.start = IRQ_CMDQ_SW_CMD,
		.end   = IRQ_CMDQ_SW_CMD,
		.flags = IORESOURCE_IRQ,
	},		
	[2] = {
		.start = AST_CMDQ_MEM_BASE,
		.end = AST_CMDQ_MEM_BASE + AST_CMDQ_MEM_SIZE - 1,
		.flags = IORESOURCE_DMA,
	},	
	
};

struct platform_device ast_device_cmdq = {
	.name		 	= "ast-cmdq",
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_cmdq_resource),
	.resource			= ast_cmdq_resource,
};
#endif

#if defined(CONFIG_AST_BITBLT) || defined(CONFIG_AST_BITBLT_MODULE)
static struct resource ast_bitblt_resource[] = {
	[0] = {
		.start = AST_BITBLT_BASE,
		.end   = AST_BITBLT_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_BB_DONE,
		.end   = IRQ_BB_DONE,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_BB_ERROR,
		.end   = IRQ_BB_ERROR,
		.flags = IORESOURCE_IRQ,
	},
	
};

struct platform_device ast_device_bitblt = {
	.name		 	= "ast-bitblt",
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_bitblt_resource),
	.resource			= ast_bitblt_resource,
};
#endif

#if defined(CONFIG_AST_RLE) || defined(CONFIG_AST_RLE_MODULE)
static struct resource ast_rle_resource[] = {
	[0] = {
		.start = AST_RLE_BASE,
		.end   = AST_RLE_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_RLE_INT1,
		.end   = IRQ_RLE_INT1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device ast_device_rle = {
	.name		 	= "ast-rle",
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_rle_resource),
	.resource			= ast_rle_resource,
};
#endif

#if defined(CONFIG_AST_EGFX) || defined(CONFIG_AST_EGFX_MODULE)
static struct resource ast_egfx_resource[] = {
	[0] = {
		.start = AST_EGFX_BASE,
		.end   = AST_EGFX_BASE + SZ_256 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EGFX_DEC_DONE,
		.end   = IRQ_EGFX_DEC_DONE,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_EGFX_DEC_EXCPT,
		.end   = IRQ_EGFX_DEC_EXCPT,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_EGFX_DEC_HANG,
		.end   = IRQ_EGFX_DEC_HANG,
		.flags = IORESOURCE_IRQ,
	},	
};

struct platform_device ast_device_egfx = {
	.name		 	= "ast-egfx",
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_egfx_resource),
	.resource			= ast_egfx_resource,
};
#endif

#if defined(CONFIG_AST_MASK) || defined(CONFIG_AST_MASK_MODULE)
static struct resource ast_mask0_resource[] = {
	[0] = {
		.start = AST_VMASK_BASE,
		.end   = AST_VMASK_BASE + SZ_128 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VMASK_DONE,
		.end   = IRQ_VMASK_DONE,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = AST_VMASK_MEM_BASE,
		.end   = AST_VMASK_MEM_BASE + AST_VMASK_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},	
};

struct platform_device ast_device_mask0 = {
	.name		 	= "ast-mask",
	.id				= 0,
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_mask0_resource),
	.resource			= ast_mask0_resource,
};

static struct resource ast_mask1_resource[] = {
	[0] = {
		.start = AST_GMASK_BASE,
		.end   = AST_GMASK_BASE + SZ_128 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_GMASK_DONE,
		.end   = IRQ_GMASK_DONE,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = AST_GMASK_MEM_BASE,
		.end   = AST_GMASK_MEM_BASE + AST_GMASK_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},	
};

struct platform_device ast_device_mask1 = {
	.name		 	= "ast-mask",
	.id				= 1,
	.dev = {
		.dma_mask			= &ast_device_rfx_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(ast_mask1_resource),
	.resource			= ast_mask1_resource,
};

#endif

void __init ast_add_device_rfx(void)
{
	//bitblt scu reset is depend on rfx ast_scu_init_rfx	
	ast_scu_init_rfx();

#if defined(CONFIG_AST_ENTROPY) || defined(CONFIG_AST_ENTROPY_MODULE)
	platform_device_register(&ast_device_entropy);
#endif

#if defined(CONFIG_AST_BULK) || defined(CONFIG_AST_BULK_MODULE)
	platform_device_register(&ast_device_bulk);
#endif

#if defined(CONFIG_AST_CMDQ) || defined(CONFIG_AST_CMDQ_MODULE)
	platform_device_register(&ast_device_cmdq);
#endif

#if defined(CONFIG_AST_BITBLT) || defined(CONFIG_AST_BITBLT_MODULE)	
	platform_device_register(&ast_device_bitblt);
#endif

#if defined(CONFIG_AST_RLE) || defined(CONFIG_AST_RLE_MODULE)
	platform_device_register(&ast_device_rle);
#endif

#if defined(CONFIG_AST_EGFX) || defined(CONFIG_AST_EGFX_MODULE)
	platform_device_register(&ast_device_egfx);
#endif

#if defined(CONFIG_AST_MASK) || defined(CONFIG_AST_MASK_MODULE)
	platform_device_register(&ast_device_mask0);
	platform_device_register(&ast_device_mask1);
#endif

}
#else	//#if defined(CONFIG_AST_RFX)
void __init ast_add_device_rfx(void) {}
#endif
