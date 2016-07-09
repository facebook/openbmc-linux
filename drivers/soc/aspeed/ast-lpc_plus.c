/********************************************************************************
* File Name     : arch/arm/mach-aspeed/ast-lpc_plus.c 
* Author         : Ryan Chen
* Description   : AST LPC
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 

*   History      : 
*    1. 2013/05/15 Ryan Chen Create
* 
********************************************************************************/
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>

#include <plat/regs-lpc.h>
#include <plat/ast-lpc.h>
#ifdef CONFIG_ARCH_AST1070
#include <plat/ast1070-scu.h>
#include <plat/ast1070-devs.h>
#include <plat/regs-ast1070-intc.h>
#include <plat/ast-uart-dma.h>
#endif

//#define AST_LPCP_DEBUG

#ifdef AST_LPCP_DEBUG
#define LPCP_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define LPCP_DBUG(fmt, args...)
#endif

#if 0
static inline u32 
ast_lpc_plus_write(u32 reg)
{
	u32 val;
		
	val = readl(ast_lpc_base + reg);
	
	LPCDBUG("ast_lpc_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_lpc_plus_write(u32 val, u32 reg) 
{
	LPCDBUG("ast_lpc_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_lpc_base + reg);
}
#endif 

static int __init ast_lpc_plus_probe(struct platform_device *pdev)
{
	static struct ast_lpc_data *ast_lpc_plus;
//	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct resource *res;
	int ret = 0;

	ast_lpc_plus = kzalloc(sizeof(struct ast_lpc_data), GFP_KERNEL);
	if (ast_lpc_plus == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ast_lpc_plus->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free;
	}

	ast_lpc_plus->reg_base = ioremap(res->start, resource_size(res));
	if (ast_lpc_plus->reg_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

#ifdef CONFIG_ARCH_AST1070
	printk("LPC PLUS Scan Device...  ");
	for(i=0;i<ast_lpc_plus->bus_info->scan_node;i++) {
		ast1070_scu_init(i ,AST_LPC_PLUS_BRIDGE + i*0x10000);
		printk("C%d-[%x] ", i, ast1070_revision_id_info(i));
		ast1070_vic_init(i, (AST_LPC_PLUS_BRIDGE + i*0x10000), IRQ_C0_VIC_CHAIN + i, IRQ_C0_VIC_CHAIN_START + (i*AST_CVIC_NUM));
		ast1070_scu_dma_init(i);
		ast1070_uart_dma_init(i, AST_LPC_PLUS_BRIDGE);
		ast_add_device_cuart(i,AST_LPC_PLUS_BRIDGE + i*0x10000);
		ast_add_device_ci2c(i,AST_LPC_PLUS_BRIDGE + i*0x10000);
	}
	printk("\n");	
#endif

	platform_set_drvdata(pdev, ast_lpc_plus);
	return 0;

err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free:
	kfree(ast_lpc_plus);

	return ret;
}

static int __exit ast_lpc_plus_remove(struct platform_device *pdev)
{
	struct ast_lpc_data *ast_lpc_plus;
	struct resource *res;

	ast_lpc_plus = platform_get_drvdata(pdev);
	if (ast_lpc_plus == NULL)
		return -ENODEV;

	iounmap(ast_lpc_plus->reg_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(ast_lpc_plus);

	return 0;
}


#if 0
static struct ast_lpc_data ast_lpc_data = {
	.counter_width = 16,
};
#endif

static const struct platform_device_id ast_lpc_plus_idtable[] = {
	{
		.name = "ast-lpc_plus",
//		.driver_data = ast_video_data,
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, ast_lpc_plus_idtable);

static struct platform_driver ast_lpc_plus_driver = {
	.driver		= {
		.name	= "ast-lpc_plus",
		.owner	= THIS_MODULE,
	},
	.remove		= ast_lpc_plus_remove,
	.id_table		= ast_lpc_plus_idtable,	
};

static int __init ast_lpc_plus_init(void)
{
	return platform_driver_probe(&ast_lpc_plus_driver, ast_lpc_plus_probe);
}
core_initcall(ast_lpc_plus_init);
