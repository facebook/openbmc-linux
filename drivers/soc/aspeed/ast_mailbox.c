/********************************************************************************
* File Name     : ast_mbx.c
* Author         : Ryan Chen
* Description   : AST Mailbox Controller
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

#include <linux/completion.h>
#include <linux/slab.h>

#include <linux/sched.h>  
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_COLDFIRE
#include <asm/arch/regs-mbx.h>
#else
#include <plat/regs-mbx.h>
#endif

//#define CONFIG_AST_MBX_DEBUG

#ifdef CONFIG_AST_MBX_DEBUG
	#define MBX_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define MBX_DBG(fmt, args...)
#endif

/***********************************************************************/
struct mailbox_info {
	u8	ch_enable;
	u8	ch_type;	
};

//IOCTL ..
#define MBXIOC_BASE       'M'

#define AST_MBX_IOCRMAILBOX		_IOR(MBXIOC_BASE, 0, struct mailbox_info*)
#define AST_MBX_IOCWMAILBOX		_IOW(MBXIOC_BASE, 1, struct mailbox_info*)

static struct ast_mailbox_data {
	struct device		*misc_dev;
	void __iomem		*reg_base;			/* virtual */
	int 				irq;				
	bool is_open;	
	spinlock_t lock;		
	u32		sts0;
	u32		sts1;
	u32		bcr;
	struct completion		xfer_complete;	
};

static inline void
ast_mbx_write(struct ast_mailbox_data *ast_mbx, u32 val, u32 reg)
{
//	MBX_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_mbx->reg_base + reg);
}

static inline u32
ast_mbx_read(struct ast_mailbox_data *ast_mbx, u32 reg)
{
	u32 val = readl(ast_mbx->reg_base + reg);
//	MBX_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

static void ast_mailbox_host_mask_int(struct ast_mailbox_data *ast_mbx, u8 enable)
{
	if(enable)
		ast_mbx_write(ast_mbx, ast_mbx_read(ast_mbx, AST_MBX_BCR) | MBHMK, AST_MBX_BCR);
	else
		ast_mbx_write(ast_mbx, ast_mbx_read(ast_mbx, AST_MBX_BCR) & ~MBHMK, AST_MBX_BCR);
}

static void ast_mailbox_ctrl_init(struct ast_mailbox_data *ast_mbx)
{
	//per byte moniter ....
//	ast_mbx_write(ast_mbx, 0xff, AST_MBX_BIE0);
//	ast_mbx_write(ast_mbx, 0xff, AST_MBX_BIE1);	
}

/***************************************************************************/

static irqreturn_t ast_mailbox_handler(int this_irq, void *dev_id)
{
	struct ast_mailbox_data *ast_mbx = dev_id;

//	ast_mbx->sts0 = ast_mbx_read(ast_mbx, AST_MBX_STS0);
//	ast_mbx->sts1 = ast_mbx_read(ast_mbx, AST_MBX_STS1);
	ast_mbx->bcr = ast_mbx_read(ast_mbx, AST_MBX_BCR);

	if((ast_mbx->bcr & MBHIST) == 0)
		return IRQ_NONE;
	

	MBX_DBG("ast_mailbox_handler sts0 = %x, sts1 = %x, bcr = %x \n", ast_mbx->sts0, ast_mbx->sts1, ast_mbx->bcr);	
//	ast_mbx_write(ast_mbx, ast_mbx->sts0, AST_MBX_STS0);
//	ast_mbx_write(ast_mbx, ast_mbx->sts1, AST_MBX_STS1);
	ast_mbx_write(ast_mbx, ast_mbx->bcr, AST_MBX_BCR);

	printk("Data: %x %x %x %x \n", 
		ast_mbx_read(ast_mbx, AST_MBX_DAT0), 
		ast_mbx_read(ast_mbx, AST_MBX_DAT4),
		ast_mbx_read(ast_mbx, AST_MBX_DAT8),
		ast_mbx_read(ast_mbx, AST_MBX_DATC));
//	complete(&ast_mbx->xfer_complete);
	
	return IRQ_HANDLED;

}

static long ast_mailbox_ioctl(struct file *fp,
			     unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	void __user *argp = (void __user *)arg;


	MBX_DBG("ast_mailbox_ioctl cmd %x \n", cmd);

	switch(cmd) {
		case AST_MBX_IOCRMAILBOX:
			break;
			
		case AST_MBX_IOCWMAILBOX:	
			break;
			
		default:			
			printk("ast_mailbox_ioctl command fail\n");
			ret = -ENOTTY;
			break;			
	}

	return ret;
}

static int ast_mailbox_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_mailbox_data *ast_mbx = dev_get_drvdata(c->this_device);

	MBX_DBG("\n");
	spin_lock(&ast_mbx->lock);

	if(ast_mbx->is_open) {
		spin_unlock(&ast_mbx->lock);
		return -1;
	}
	ast_mbx->is_open = true;

	spin_unlock(&ast_mbx->lock);

	return 0;	
}

static int ast_mailbox_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_mailbox_data *ast_mbx = dev_get_drvdata(c->this_device);

	MBX_DBG("\n");

	spin_lock(&ast_mbx->lock);
	ast_mbx->is_open = false;
	spin_unlock(&ast_mbx->lock);


	return 0;}

static const struct file_operations ast_mailbox_fops = {
        .owner  =     	THIS_MODULE,
        .unlocked_ioctl =	ast_mailbox_ioctl,
        .open =         	ast_mailbox_open,
        .release =      	ast_mailbox_release,
};

struct miscdevice ast_mailbox_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-mailbox",
	.fops = &ast_mailbox_fops,
};

static int ast_mailbox_probe(struct platform_device *pdev)
{
	static struct ast_mailbox_data *ast_mbx;
	struct resource *res;
	int ret=0;

	MBX_DBG(" \n");	
	
	ast_mbx = kzalloc(sizeof(struct ast_mailbox_data), GFP_KERNEL);
	if(ast_mbx == NULL) {
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

	ast_mbx->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_mbx->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_mbx->irq = platform_get_irq(pdev, 0);
	if (ast_mbx->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_mbx->irq, ast_mailbox_handler, IRQF_SHARED,
			  "ast-mailbox", ast_mbx);
	
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_mbx->irq);
		goto out_region;
	}

	ret = misc_register(&ast_mailbox_misc);
	if (ret){		
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}

	spin_lock_init(&ast_mbx->lock);
	platform_set_drvdata(pdev, ast_mbx);
	dev_set_drvdata(ast_mailbox_misc.this_device, ast_mbx);

	ast_mailbox_ctrl_init(ast_mbx);
	
	printk(KERN_INFO "ast_mbx: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_mbx->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_mailbox_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_mailbox_data *ast_mbx = platform_get_drvdata(pdev);

	MBX_DBG("ast_mailbox_remove\n");

	misc_deregister(&ast_mailbox_misc);

	free_irq(ast_mbx->irq, &ast_mbx);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_mbx->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

#ifdef CONFIG_PM
static int 
ast_mailbox_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_mailbox_suspend : TODO \n");
	return 0;
}

static int 
ast_mailbox_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_mailbox_suspend        NULL
#define ast_mailbox_resume         NULL
#endif

static struct platform_driver ast_mailbox_driver = {
	.remove 		= ast_mailbox_remove,
	.suspend        = ast_mailbox_suspend,
	.resume         = ast_mailbox_resume,
	.driver         = {
		.name   = "ast-mailbox",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_mailbox_driver, ast_mailbox_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Mailbox driver");
MODULE_LICENSE("GPL");
