/********************************************************************************
* File Name     : ast_bulk.c
* Author         : Ryan Chen
* Description   : AST formatter Controller
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
*      1. 2013/01/30 Ryan Chen create this file 
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
#define AST_BULK_STS					0x00
#define AST_BULK_RING_START_ADDR		0x04
#define AST_BULK_RING_END_ADDR		0x08
#define AST_BULK_BS_START_ADDR		0x0C
#define AST_BULK_BS_SIZE				0x10
#define AST_BULK_HRING_START_ADDR		0x14
#define AST_BULK_HRING_END_ADDR		0x18
#define AST_BULK_ENG_RDY				0x1C
#define AST_BULK_HBUFF_READP			0x20
#define AST_BULK_CLR_ISR				0x24
#define AST_BULK_ISR					0x28
#define AST_BULK_IER					0x28

/***********************************************************************/

//#define CONFIG_AST_BULK_DEBUG

#ifdef CONFIG_AST_BULK_DEBUG
	#define BULK_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define BULK_DBG(fmt, args...)
#endif

#define EGFX_BULK_NO_HEADER_MODE		1

/***********************************************************************/
static struct ast_bulk_data {
	void __iomem		*reg_base;			/* virtual */
	int 				irq;	
	phys_addr_t		*bulk_stream_phy;				
	phys_addr_t		*history_phy;					
	char				*stream_virt;	
	char				*history_virt;		
	u32				stream_size;			/* mem size*/
	u32				history_size;			/* mem size*/	
	bool 			is_open;	
	spinlock_t 		lock;		
};

static inline void
ast_bulk_write(struct ast_bulk_data *ast_bulk, u32 val, u32 reg)
{
//	BULK_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_bulk->reg_base + reg);
}

static inline u32
ast_bulk_read(struct ast_bulk_data *ast_bulk, u32 reg)
{
	u32 val = readl(ast_bulk->reg_base + reg);
//	BULK_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

//better for performance 
struct ast_bulk_data *ast_bulk;

/***************************************************************************/
static void ast_bulk_init(struct ast_bulk_data *ast_bulk)
{
	ast_bulk_write(ast_bulk, ast_bulk->bulk_stream_phy, AST_BULK_RING_START_ADDR);
	ast_bulk_write(ast_bulk, ast_bulk->bulk_stream_phy + ast_bulk->stream_size, AST_BULK_RING_END_ADDR);

	ast_bulk_write(ast_bulk, ast_bulk->history_phy, AST_BULK_HRING_START_ADDR);
	ast_bulk_write(ast_bulk, ast_bulk->history_phy + ast_bulk->history_size, AST_BULK_HRING_END_ADDR);

#if EGFX_BULK_NO_HEADER_MODE
	ast_bulk_write(ast_bulk, 3, AST_BULK_HBUFF_READP);		//noheader mode & history pointer reset
#else
	ast_bulk_write(ast_bulk, 1, AST_BULK_HBUFF_READP); 	//history pointer reset
#endif
	
}

/***************************************************************************/
static irqreturn_t ast_bulk_handler(int this_irq, void *dev_id)
{
	struct ast_bulk_data *ast_bulk = dev_id;

	u32 sts = ast_bulk_read(ast_bulk, AST_BULK_ISR);
	u32 idx = 0;
	
	
	return IRQ_HANDLED;
}

#if 0
static const struct file_operations ast_bulk_fops = {
        .owner  		= THIS_MODULE,
	.llseek 		= no_llseek,			
//        .unlocked_ioctl = ast_h264_ioctl,
	.read		= formatter_read,
	.write 		= formatter_write,
        .open 		= ast_bulk_open,
        .mmap		= formatter_mmap,
        .release 		= ast_bulk_release,
};
#endif

struct miscdevice ast_bulk_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-bulk",
//	.fops = &ast_bulk_fops,
};

static int ast_bulk_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret=0;

	ast_bulk = kzalloc(sizeof(struct ast_bulk_data), GFP_KERNEL);
	if(ast_bulk == NULL) {
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
		dev_err(&pdev->dev, "cannot reserved io region\n");
		ret = -ENXIO;
		goto out;
	}

	ast_bulk->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_bulk->reg_base) {
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
		dev_err(&pdev->dev, "cannot reserved dma region\n");
		ret = -ENXIO;
		goto out;
	}


	ast_bulk->bulk_stream_phy = res->start;
	ast_bulk->stream_size = resource_size(res)/2;

	ast_bulk->history_phy = ast_bulk->bulk_stream_phy + ast_bulk->stream_size;		
	ast_bulk->history_size = ast_bulk->stream_size;	

	printk("ast_bulk->bulk_stream_phy:  %x \n", ast_bulk->bulk_stream_phy);

	ast_bulk->stream_virt = ioremap(res->start, resource_size(res));
	if (!ast_bulk->stream_virt) {
		ret = -EIO;
		goto out_region;
	}
	ast_bulk->history_virt = ast_bulk->stream_virt + ast_bulk->stream_size;		

	ast_bulk->irq = platform_get_irq(pdev, 0);
	if (ast_bulk->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_bulk->irq, ast_bulk_handler, IRQF_SHARED,
			  "ast-bulk", ast_bulk);
	
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_bulk->irq);
		goto out_region;
	}

	ret = misc_register(&ast_bulk_misc);
	if (ret){		
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}
	spin_lock_init(&ast_bulk->lock);

	platform_set_drvdata(pdev, ast_bulk);

	ast_bulk_init(ast_bulk);
	
	printk(KERN_INFO "ast_bulk: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_bulk->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_bulk_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_bulk_data *ast_bulk = platform_get_drvdata(pdev);

	free_irq(ast_bulk->irq, &ast_bulk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_bulk->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

static struct platform_driver ast_bulk_driver = {
	.driver         = {
		.name   = "ast-bulk",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_bulk_driver, ast_bulk_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST bulk driver");
MODULE_LICENSE("GPL");
