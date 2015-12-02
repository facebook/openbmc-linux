/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-lpc.c
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
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>
#include <plat/ast-lpc.h>
#include <plat/regs-lpc.h>
#include <mach/gpio.h>

/* --------------------------------------------------------------------
 *  LPC
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_LPC) || defined(CONFIG_AST_LPC_MODULE)
static struct ast_lpc_bus_info ast_lpc_info = {
#ifdef CONFIG_AST_MASTER
	.lpc_bus_mode = 1,	//0: slave , 1: master
#else
	.lpc_bus_mode = 0,	//0: slave , 1: master
#endif	
	.lpc_mode = 0,
#ifdef CONFIG_AST_LPC_SNOOP	
	.snoop_enable = 1,
#else	
	.snoop_enable = 0,	
#endif
#ifdef CONFIG_AST_IPMI_KCS
	.ipmi_kcs_enable = 1,
#else	
	.ipmi_kcs_enable = 0,	
#endif
#ifdef CONFIG_AST_IPMI_BT
	.ipmi_bt_enable = 1,
#else	
	.ipmi_bt_enable = 0,	
#endif
	.bridge_phy_addr = AST_LPC_BRIDGE,
};

static struct resource ast_lpc_resource[] = {
	[0] = {
		.start = AST_LPC_BASE,
		.end = AST_LPC_BASE + SZ_512 -1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LPC,
		.end = IRQ_LPC,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_lpc_dma_mask = 0xffffffffUL;

static struct platform_device ast_lpc_device = {
	.name	= "ast-lpc",
    .id = 0,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
			.platform_data = &ast_lpc_info,
    },
	.resource = ast_lpc_resource,
	.num_resources = ARRAY_SIZE(ast_lpc_resource),
};
#ifdef AST_LPC_PLUS_BASE
static struct ast_lpc_bus_info ast_lpc_plus_info = {
#ifdef CONFIG_AST_MASTER
	.lpc_bus_mode = 1,	//0: slave , 1: master
#else
	.lpc_bus_mode = 0,	//0: slave , 1: master
#endif	
	.lpc_mode = 1,
	.bridge_phy_addr = AST_LPC_PLUS_BRIDGE,
};

static struct resource ast_lpc_plus_resource[] = {
	[0] = {
		.start = AST_LPC_PLUS_BASE,
		.end = AST_LPC_PLUS_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ast_lpc_plus_device = {
	.name	= "ast-lpc_plus",
    .id = 0,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
			.platform_data = &ast_lpc_plus_info,
    },
	.resource = ast_lpc_plus_resource,
	.num_resources = ARRAY_SIZE(ast_lpc_plus_resource),
};
#endif
void __init ast_add_device_lpc(void)
{
#if 0	
	//due to at init reset state is correct . 
	if(gpio_get_value(PIN_GPIOI1))
		printk("Use LPC+ Bus Access \n");
	else
		printk("Use LPC Bus Access \n");		
#endif

#ifdef CONFIG_ARCH_AST1070


#ifdef AST_LPC_PLUS_BASE	
	int cc_num;
	if(gpio_get_value(PIN_GPIOI2))
		cc_num = 2; //dual 1070
	else	
		cc_num = 1; //single 1070	

	if(ast_scu_get_lpc_plus_enable()) {
		ast_lpc_plus_info.scan_node = cc_num;
	} else {
		ast_lpc_info.lpc_bus_mode = 1;
		ast_lpc_info.scan_node = cc_num;
	}
#else
	ast_lpc_info.scan_node = 1;
#endif
	
	
#endif	//End AST1070

	platform_device_register(&ast_lpc_device);
#ifdef AST_LPC_PLUS_BASE	
	if(ast_scu_get_lpc_plus_enable())
		platform_device_register(&ast_lpc_plus_device);
#endif
}
#else
void __init ast_add_device_lpc(void) {
    // Since we disable LPC, bring the UART1 and UART2 out from LPC control

    void __iomem *reg_base;

    reg_base = ioremap(AST_LPC_BASE, SZ_256);
    writel(readl(reg_base + AST_LPC_HICR9)
           & ~(LPC_HICR9_SOURCE_UART1|LPC_HICR9_SOURCE_UART2
               |LPC_HICR9_SOURCE_UART3|LPC_HICR9_SOURCE_UART4),
           reg_base + AST_LPC_HICR9);
}
#endif
