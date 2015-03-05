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
#include <plat/ast-snoop.h>
#include <plat/ast-lpc.h>
#ifdef CONFIG_ARCH_AST1070
#include <plat/ast1070-scu.h>
#include <plat/ast1070-devs.h>
#include <plat/regs-ast1070-intc.h>
#include <plat/ast1070-uart-dma.h>
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

static int __devinit ast_lpc_plus_probe(struct platform_device *pdev)
{
	static struct ast_lpc_driver_data *lpc_plus_driver_data;
//	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct resource *res;
	int ret = 0;
	int i;

	lpc_plus_driver_data = kzalloc(sizeof(struct ast_lpc_driver_data), GFP_KERNEL);
	if (lpc_plus_driver_data == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	lpc_plus_driver_data->pdev = pdev;

	lpc_plus_driver_data->bus_info = pdev->dev.platform_data;

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

	lpc_plus_driver_data->reg_base = ioremap(res->start, resource_size(res));
	if (lpc_plus_driver_data->reg_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

#ifdef CONFIG_ARCH_AST1070
	if(lpc_plus_driver_data->bus_info->bus_scan) {
		printk("LPC PLUS Scan Device...  ");
		for(i=0;i<lpc_plus_driver_data->bus_info->scan_node;i++) {
			ast1070_scu_init(i ,lpc_plus_driver_data->bus_info->bridge_phy_addr + i*0x10000);
			printk("C%d-[%x] ", i, ast1070_revision_id_info(i));
			ast1070_vic_init(i, (lpc_plus_driver_data->bus_info->bridge_phy_addr + i*0x10000), IRQ_C0_VIC_CHAIN + i, IRQ_C0_VIC_CHAIN_START + (i*AST_CVIC_NUM));
			ast1070_scu_dma_init(i);
			ast1070_uart_dma_init(i, lpc_plus_driver_data->bus_info->bridge_phy_addr);
			ast_add_device_cuart(i,lpc_plus_driver_data->bus_info->bridge_phy_addr + i*0x10000);
			ast_add_device_ci2c(i,lpc_plus_driver_data->bus_info->bridge_phy_addr + i*0x10000);
		}
		printk("\n");

	}
	
#endif

	platform_set_drvdata(pdev, lpc_plus_driver_data);
	return 0;

err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free:
	kfree(lpc_plus_driver_data);

	return ret;
}

static int __devexit ast_lpc_plus_remove(struct platform_device *pdev)
{
	struct ast_lpc_driver_data *lpc_plus_driver_data;
	struct resource *res;

	lpc_plus_driver_data = platform_get_drvdata(pdev);
	if (lpc_plus_driver_data == NULL)
		return -ENODEV;

	iounmap(lpc_plus_driver_data->reg_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(lpc_plus_driver_data);

	return 0;
}

static struct platform_driver ast_lpc_plus_driver = {
	.driver		= {
		.name	= "ast_lpc_plus",
		.owner	= THIS_MODULE,
	},
	.probe		= ast_lpc_plus_probe,
	.remove		= __devexit_p(ast_lpc_plus_remove),
//	.id_table	= pwm_id_table,
};

static int __init ast_lpc_plus_init(void)
{
	return platform_driver_register(&ast_lpc_plus_driver);
}
arch_initcall(ast_lpc_plus_init);

static void __exit ast_lpc_plus_exit(void)
{
	platform_driver_unregister(&ast_lpc_plus_driver);
}
module_exit(ast_lpc_plus_exit);

MODULE_LICENSE("GPL v2");
