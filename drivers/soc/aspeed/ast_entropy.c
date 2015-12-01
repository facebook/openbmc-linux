/********************************************************************************
* File Name     : ast_entropy.c
* Author         : Ryan Chen
* Description   : AST entropy Controller
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
#define AST_ENTROPY_STS					0x00
#define AST_ENTROPY_RING_START_ADDR		0x04
#define AST_ENTROPY_RING_END_ADDR			0x08
#define AST_ENTROPY_START_ADDR			0x0C
#define AST_ENTROPY_BITPOST_START_ADDR	0x10
#define AST_ENTROPY_SIGNPOST_START_ADDR	0x14
#define AST_ENTROPY_TILE_START_ADDR		0x18

#define AST_ENTROPY_IDWT_FIFO_CTRL		0x2C

#define AST_ENTROPY_CTRL					0x34

#define AST_ENTROPY_IER						0x3C


/***********************************************************************/

//#define CONFIG_AST_ENTROPY_DEBUG

#ifdef CONFIG_AST_ENTROPY_DEBUG
	#define ENTROPY_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define ENTROPY_DBG(fmt, args...)
#endif

/***********************************************************************/
static struct ast_entropy_data {
	void __iomem		*reg_base;			/* virtual */
	int 				irq;	
	phys_addr_t		*graphic_stream_phy;
	char				*graphic_stream_virt;		
	u32				graphic_stream_size;	
	bool 				is_open;	
	spinlock_t 		lock;		
};

static inline void
ast_entropy_write(struct ast_entropy_data *ast_entropy, u32 val, u32 reg)
{
//	entropy_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_entropy->reg_base + reg);
}

static inline u32
ast_entropy_read(struct ast_entropy_data *ast_entropy, u32 reg)
{
	u32 val = readl(ast_entropy->reg_base + reg);
//	entropy_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

/***************************************************************************/
static void ast_entropy_init(struct ast_entropy_data *ast_entropy)
{
	int i;
	ast_entropy_write(ast_entropy, ast_entropy->graphic_stream_phy, AST_ENTROPY_RING_START_ADDR);
	ast_entropy_write(ast_entropy, ast_entropy->graphic_stream_phy + ast_entropy->graphic_stream_size, AST_ENTROPY_RING_END_ADDR);

//	ast_entropy_write(ast_entropy, 0x18103000, AST_ENTROPY_IDWT_FIFO_CTRL);
	ast_entropy_write(ast_entropy, 0x17103000, AST_ENTROPY_IDWT_FIFO_CTRL);
	ast_entropy_write(ast_entropy, 0xFFFF8013, AST_ENTROPY_CTRL);
	ast_entropy_write(ast_entropy, 0x0408003F, AST_ENTROPY_IER);		
}

/***************************************************************************/
static irqreturn_t ast_entropy_handler(int this_irq, void *dev_id)
{
	struct ast_entropy_data *ast_entropy = dev_id;

	u32 sts = ast_entropy_read(ast_entropy, AST_ENTROPY_ISR);
	u32 idx = 0;
	

	return IRQ_HANDLED;
}

struct miscdevice ast_entropy_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-entropy",
//	.fops = &ast_entropy_fops,
};

struct ast_entropy_data *ast_entropy;

static int ast_entropy_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret=0;

	ast_entropy = kzalloc(sizeof(struct ast_entropy_data), GFP_KERNEL);
	if(ast_entropy == NULL) {
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

	ast_entropy->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_entropy->reg_base) {
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

	ast_entropy->mem_phy = res->start;
	ast_entropy->mem_size = resource_size(res);

	printk("ast_entropy->mem_phy:  %x , each size %x \n", ast_entropy->mem_phy, FORMATTER_DEST_SIZE);

	ast_entropy->mem_virt = ioremap(res->start, resource_size(res));
	if (!ast_entropy->mem_virt) {
		ret = -EIO;
		goto out_region;
	}

	ast_entropy->irq = platform_get_irq(pdev, 0);
	if (ast_entropy->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_entropy->irq, ast_entropy_handler, IRQF_SHARED,
			  "ast-entropy", ast_entropy);
	
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_entropy->irq);
		goto out_region;
	}

	ret = misc_register(&ast_entropy_misc);
	if (ret){		
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}
	spin_lock_init(&ast_entropy->lock);

	platform_set_drvdata(pdev, ast_entropy);

	ast_entropy_ctrl_init(ast_entropy);
	
	printk(KERN_INFO "ast_entropy: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_entropy->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_entropy_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_entropy_data *ast_entropy = platform_get_drvdata(pdev);

	free_irq(ast_entropy->irq, &ast_entropy);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_entropy->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

static struct platform_driver ast_entropy_driver = {
	.driver         = {
		.name   = "ast-entropy",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_entropy_driver, ast_entropy_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST entropy driver");
MODULE_LICENSE("GPL");
