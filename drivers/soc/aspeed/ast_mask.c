/********************************************************************************
* File Name     : ast_vmask.c
* Author         : Ryan Chen
* Description   : AST vmask Controller
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
*
*   Version      : 1.0
*   History      : 
*      1. 2015/06/30 Ryan Chen create this file 
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/sched.h>  
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

/***********************************************************************/


/***********************************************************************/

//#define CONFIG_AST_VMASK_DEBUG

#ifdef CONFIG_AST_VMASK_DEBUG
	#define VMASK_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define VMASK_DBG(fmt, args...)
#endif

/***********************************************************************/
static struct ast_vmask_data {
	void __iomem		*reg_base;			/* virtual */
	int 				irq;	
	phys_addr_t		*mem_phy;				/* mem size*/			
	char				*mem_virt;	
	u32				mem_size;			/* mem size*/
	bool 				is_open;	
	spinlock_t 		lock;		
};

static inline void
ast_vmask_write(struct ast_vmask_data *ast_vmask, u32 val, u32 reg)
{
//	vmask_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_vmask->reg_base + reg);
}

static inline u32
ast_vmask_read(struct ast_vmask_data *ast_vmask, u32 reg)
{
	u32 val = readl(ast_vmask->reg_base + reg);
//	vmask_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

/***************************************************************************/
static void ast_vmask_init(struct ast_vmask_data *ast_vmask)
{
	int i;
	printk("phy : %x\n", ast_vmask->mem_phy);

	
}

/***************************************************************************/
static irqreturn_t ast_vmask_handler(int this_irq, void *dev_id)
{
	struct ast_vmask_data *ast_vmask = dev_id;

//	u32 sts = ast_vmask_read(ast_vmask, AST_VMASK_ISR);
	u32 idx = 0;
	

	return IRQ_HANDLED;
}

struct miscdevice ast_vmask_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-vmask",
//	.fops = &ast_vmask_fops,
};

struct ast_vmask_data *ast_vmask;

static int ast_vmask_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret=0;

	ast_vmask = kzalloc(sizeof(struct ast_vmask_data), GFP_KERNEL);
	if(ast_vmask == NULL) {
		printk("memalloc error");
		goto out;
	}
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	ast_vmask->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_vmask->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_DMA\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	ast_vmask->mem_phy = res->start;
	ast_vmask->mem_size = resource_size(res);

	printk("ast_vmask->mem_phy:  %x , each size %x \n", ast_vmask->mem_phy);

	ast_vmask->mem_virt = ioremap(res->start, resource_size(res));
	if (!ast_vmask->mem_virt) {
		ret = -EIO;
		goto out_region;
	}

	ast_vmask->irq = platform_get_irq(pdev, 0);
	if (ast_vmask->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_vmask->irq, ast_vmask_handler, IRQF_SHARED,
			  "ast-vmask", ast_vmask);
	
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_vmask->irq);
		goto out_region;
	}

	ret = misc_register(&ast_vmask_misc);
	if (ret){		
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}
	spin_lock_init(&ast_vmask->lock);

	platform_set_drvdata(pdev, ast_vmask);

	ast_vmask_init(ast_vmask);
	
	printk(KERN_INFO "ast_vmask: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_vmask->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_vmask_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_vmask_data *ast_vmask = platform_get_drvdata(pdev);

	free_irq(ast_vmask->irq, &ast_vmask);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_vmask->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

static struct platform_driver ast_vmask_driver = {
	.driver         = {
		.name   = "ast-vmask",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_vmask_driver, ast_vmask_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST vmask driver");
MODULE_LICENSE("GPL");
