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
 *  H264
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_H264) || defined(CONFIG_AST_H264_MODULE)
#if 0
static struct ast_h264_plat_data h264_plat_data = {
	.get_clk = ast_get_m_pll_clk,
	.ctrl_reset = ast_scu_reset_video,
};
#endif
static struct resource ast_h264_resource[] = {
	[0] = {
		.start = AST_H264_BASE,
		.end   = AST_H264_BASE + 0x00002100 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_H264,
		.end   = IRQ_H264,
		.flags = IORESOURCE_IRQ,
	},	
	[2] = {
		.start = AST_H264_MEM_BASE,
		.end = AST_H264_MEM_BASE + AST_H264_MEM_SIZE - 1,
		.flags = IORESOURCE_DMA,
	},	
};

static u64 ast_device_h264_dmamask = 0xffffffffUL;
struct platform_device ast_device_h264 = {
	.name		 	= "ast-h264",
	.dev = {
		.dma_mask			= &ast_device_h264_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
//		.platform_data 		= &h264_plat_data,
	},
	.num_resources	= ARRAY_SIZE(ast_h264_resource),
	.resource			= ast_h264_resource,
};

void __init ast_add_device_h264(void)
{
	ast_scu_init_h264();
	platform_device_register(&ast_device_h264);
}
#else
void __init ast_add_device_h264(void) {}
#endif
