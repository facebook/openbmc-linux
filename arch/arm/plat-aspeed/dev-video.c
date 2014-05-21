/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-video.c
* Author        : Ryan Chen
* Description   : ASPEED Video Driver
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
*    1. 2013/04/15 Ryan Chen initial
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>

#include <plat/devs.h>
#include <plat/ast-scu.h>
#include <plat/ast-sdmc.h>

#include <mach/ast_video.h>

/* --------------------------------------------------------------------
 *  AST VIDEO
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_VIDEO) || defined(CONFIG_AST_VIDEO_MODULE)

#define ASR_VIDEO_MEM				AST_DRAM_BASE + SZ_8M*10
static u32 get_vga_mem_base(void)
{
	u32 vga_mem_size, mem_size;
	mem_size = ast_sdmc_get_mem_size();
	vga_mem_size = ast_scu_get_vga_memsize();
	printk("VGA Info : MEM Size %d, VGA Mem Size %d \n",mem_size, vga_mem_size);
	return (mem_size - vga_mem_size);
}

static struct ast_video_plat_data video_plat_data = {
	.get_clk = ast_get_m_pll_clk,
	.ctrl_reset = ast_scu_reset_video,
	.vga_display = ast_scu_set_vga_display,
	.get_vga_base = get_vga_mem_base,
	.input_source = VIDEO_SOURCE_INTERNAL,
	.mode = VIDEO_FRAME_MODE,
	.rc4_enable = 0,
	.compress = VIDEO_YUV444,
	.scaling = 0,
};


static struct resource ast_video_resources[] = {
	[0] = {
		.start = AST_VIDEO_BASE,
		.end = AST_VIDEO_BASE + SZ_1K*2 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VIDEO,
		.end = IRQ_VIDEO,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = ASR_VIDEO_MEM,
		.end = ASR_VIDEO_MEM + SZ_32M,
		.flags = IORESOURCE_DMA,
	},	
};

static u64 ast_device_video_dmamask = 0xffffffffUL;
struct platform_device ast_video_device = {
	.name		  = "ast-video",
	.id		  = 0,
	.dev		= {
		.dma_mask		= &ast_device_video_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
		.platform_data= &video_plat_data,
	},
	.resource	= ast_video_resources,
	.num_resources	= ARRAY_SIZE(ast_video_resources),
};

void __init ast_add_device_video(void)
{
	ast_scu_init_video(0);
	ast_scu_multi_func_video();
	platform_device_register(&ast_video_device);
}
#else
void __init ast_add_device_video(void) {}
#endif
