/********************************************************************************
* File Name     : ast_formatter.c
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

#include <plat/ast_formatter.h>

#include <plat/ast_bitblt.h>

#include <mach/ast_lcd.h>

#include <linux/fb.h>

/***********************************************************************/
#define AST_FORMATTER_SRC				0x00
#define AST_FORMATTER_CTRL				0x04
#define AST_FORMATTER_ISR				0x08
#define AST_FORMATTER_DEST1				0x10
#define AST_FORMATTER_DEST2				0x14
#define AST_FORMATTER_H_W				0x18
#define AST_FORMATTER_DEST3				0x1C
/***********************************************************************/
#define AST_FORMATTER_IDLE				(0x1 << 16)

#define AST_FORMATTER_SW_REST			(0x1 << 8)

#define AST_FORMATTER_TRIGGER			0x1
#define AST_GET_CUR_IDX(x)				((x >> 17) & 0x3)
#define AST_GET_VALID_IDX(x)			((x >> 19) & 0x3)
#define AST_GET_BIBLT_IDX(x)			((x >> 21) & 0x3)
/***********************************************************************/
#define FORMATTER_COMPLETE_INT			0x1
#define FORMATTER_COMPLETE_INT_EN		(0x1 << 16)

/***********************************************************************/
#define FORMATTER_DEST_NO				3
#define FORMATTER_DEST_SIZE				0x400000	//4MB<-(1920*1080*2)

/***********************************************************************/

//#define CONFIG_AST_FORMATTER_DEBUG

#ifdef CONFIG_AST_FORMATTER_DEBUG
	#define FORMATTER_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define FORMATTER_DBG(fmt, args...)
#endif

/***********************************************************************/
static struct ast_formatter_data {
	void __iomem		*reg_base;			/* virtual */
	int 				irq;	
	phys_addr_t		*dest[FORMATTER_DEST_NO];				/* mem size*/	
	phys_addr_t		*mem_phy;				/* mem size*/			
	char				*mem_virt;	
	u32				mem_size;			/* mem size*/
	bool 				is_open;	
	spinlock_t 		lock;		
};

static inline void
ast_formatter_write(struct ast_formatter_data *ast_formatter, u32 val, u32 reg)
{
//	formatter_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_formatter->reg_base + reg);
}

static inline u32
ast_formatter_read(struct ast_formatter_data *ast_formatter, u32 reg)
{
	u32 val = readl(ast_formatter->reg_base + reg);
//	formatter_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

//better for performance 
struct ast_formatter_data *ast_formatter;

/***************************************************************************/
#if 1
void formatter_yuv420_to_422(struct formatter_yuv420_to_422 *formatter)
{
	//check engine idle TODO ...
	FORMATTER_DBG("src : %x, x : %d, y : %d \n", formatter->src, formatter->width, formatter->height);
init:	
	if(AST_FORMATTER_IDLE &  ast_formatter_read(ast_formatter, AST_FORMATTER_CTRL)) {
		ast_formatter_write(ast_formatter, formatter->src, AST_FORMATTER_SRC);							// 420 to 422 Source address

		ast_formatter_write(ast_formatter, formatter->height << 16 | formatter->width, AST_FORMATTER_H_W);	// 420 to 422 Height/Width
		//Trigger 	
		ast_formatter_write(ast_formatter, AST_FORMATTER_TRIGGER, AST_FORMATTER_CTRL);					// 420 to 422 Fire
	} else {
		printk("Formatter Engine Busy ~~~~~~~~~~~~~~!! \n");
		goto init;
	}

}
#else
void formatter_yuv420_to_422(struct formatter_yuv420_to_422 *formatter)
{
	//check engine idle TODO ...
	int i, j =0;
	struct fb_info *astfb_info = astfb_get_crt_screen(0);	
	u32 *src = formatter->src_virt;	
	u32 *dest = ast_formatter->mem_virt;
	unsigned int temp0, temp1, temp2, temp3, temp4, temp5;	
	unsigned int width = formatter->width;
	unsigned int height = formatter->height;
	unsigned int blk_num = (width * height) / 16;
	struct ast_bitblt_basic bitblt;

	FORMATTER_DBG("src : %x, x : %d, y : %d \n", formatter->src, formatter->width, formatter->height);
	
	for(i=height-4;i>=0;i-=4)
	{
		for(j=width/2; j>0;j-=2)
		{
			temp0 = src[(blk_num * 24) / 4 - 6];
			temp1 = src[(blk_num * 24) / 4 - 5];
			temp2 = src[(blk_num * 24) / 4 - 4];
			temp3 = src[(blk_num * 24) / 4 - 3];
			temp4 = src[(blk_num * 24) / 4 - 2];
			temp5 = src[(blk_num * 24) / 4 - 1];
			dest[width*2*(i+0)/4 + j - 1] = ((temp4 & 0x0000ff00) << 16 |
											(temp0 & 0xff000000) >>  8 |
											(temp4 & 0xff000000) >> 16 |
											(temp0 & 0x00ff0000) >> 16 );
			dest[width*2*(i+0)/4 + j - 2] = ((temp4 & 0x000000ff) << 24 |
											(temp0 & 0x0000ff00) <<  8 |
											(temp4 & 0x00ff0000) >>  8 |
											(temp0 & 0x000000ff)       );
			dest[width*2*(i+1)/4 + j - 1] = ((temp4 & 0x0000ff00) << 16 |
											(temp1 & 0xff000000) >>  8 |
											(temp4 & 0xff000000) >> 16 |
											(temp1 & 0x00ff0000) >> 16 );
			dest[width*2*(i+1)/4 + j - 2] = ((temp4 & 0x000000ff) << 24 |
											(temp1 & 0x0000ff00) <<  8 |
											(temp4 & 0x00ff0000) >>  8 |
											(temp1 & 0x000000ff)       );
			dest[width*2*(i+2)/4 + j - 1] = ((temp5 & 0x0000ff00) << 16 |
											(temp2 & 0xff000000) >>  8 |
											(temp5 & 0xff000000) >> 16 |
											(temp2 & 0x00ff0000) >> 16 );
			dest[width*2*(i+2)/4 + j - 2] = ((temp5 & 0x000000ff) << 24 |
											(temp2 & 0x0000ff00) <<  8 |
											(temp5 & 0x00ff0000) >>  8 |
											(temp2 & 0x000000ff)       );
			dest[width*2*(i+3)/4 + j - 1] = ((temp5 & 0x0000ff00) << 16 |
											(temp3 & 0xff000000) >>  8 |
											(temp5 & 0xff000000) >> 16 |
											(temp3 & 0x00ff0000) >> 16 );
			dest[width*2*(i+3)/4 + j - 2] = ((temp5 & 0x000000ff) << 24 |
											(temp3 & 0x0000ff00) <<  8 |
											(temp5 & 0x00ff0000) >>  8 |
											(temp3 & 0x000000ff)       );
			blk_num --;
			
		}
	}

bitblt.src_base = ast_formatter->mem_phy;		
bitblt.dest_base = astfb_info->fix.smem_start;

FORMATTER_DBG("src addr : %x , dest : %x \n", bitblt.src_base, bitblt.dest_base);

bitblt.src_format = ASTFB_COLOR_YUV422;
bitblt.dest_format = ASTFB_COLOR_RGB888;

bitblt.src_pitch = (ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff) * 2;


FORMATTER_DBG("src: w : %d , h : %d \n", ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff, (ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) >> 16));

if((ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff) > astfb_info->var.xres)
	bitblt.src_w = astfb_info->var.xres;
else
	bitblt.src_w = ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff;

if((ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) >> 16) > astfb_info->var.yres)
	bitblt.src_h = astfb_info->var.yres;
else
	bitblt.src_h = ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) >> 16;

FORMATTER_DBG("cut w : %d , h : %d \n", bitblt.src_w, bitblt.src_h);
bitblt.src_x = 0;
bitblt.src_y = 0;


bitblt.dest_pitch = (astfb_info->var.xres) * 4;

bitblt.dest_w = astfb_info->var.xres;
bitblt.dest_h = astfb_info->var.yres;

FORMATTER_DBG("fb x : %d , y : %d \n", astfb_info->var.xres, astfb_info->var.yres);

bitblt.dest_x = 0;
bitblt.dest_y = 0;

ast_h264_bitblt(&bitblt);

	
}

#endif 
EXPORT_SYMBOL(formatter_yuv420_to_422);

void ast_formatter_reset(void)
{
	ast_formatter_write(ast_formatter, AST_FORMATTER_SW_REST, AST_FORMATTER_CTRL);
	mdelay(1);
	ast_formatter_write(ast_formatter, 0, AST_FORMATTER_CTRL);
}
EXPORT_SYMBOL(ast_formatter_reset);

static void ast_formatter_ctrl_init(struct ast_formatter_data *ast_formatter)
{
	int i;
	printk("phy : %x\n", ast_formatter->mem_phy);
	ast_formatter->dest[0] = ast_formatter->mem_phy;
	ast_formatter_write(ast_formatter, ast_formatter->dest[0], AST_FORMATTER_DEST1);
	ast_formatter->dest[1] = ast_formatter->dest[0] + FORMATTER_DEST_SIZE;	
	ast_formatter_write(ast_formatter, ast_formatter->dest[1], AST_FORMATTER_DEST2);	
	ast_formatter->dest[2] = ast_formatter->dest[1] + FORMATTER_DEST_SIZE;		
	ast_formatter_write(ast_formatter, ast_formatter->dest[2], AST_FORMATTER_DEST3);
	
	ast_formatter_write(ast_formatter, FORMATTER_COMPLETE_INT | FORMATTER_COMPLETE_INT_EN, AST_FORMATTER_ISR);

	//buffer reset to black
	for (i = 0; i < (ast_formatter->mem_size >> 2); i++)
		*(ast_formatter->mem_virt + i) = 0x80108010;
	
}

/***************************************************************************/
static irqreturn_t ast_formatter_handler(int this_irq, void *dev_id)
{
	struct ast_formatter_data *ast_formatter = dev_id;
	struct ast_bitblt_basic bitblt;
	struct fb_info *astfb_info = astfb_get_crt_screen(0);

	u32 sts = ast_formatter_read(ast_formatter, AST_FORMATTER_ISR);
	u32 idx = 0;
	
	if(sts & FORMATTER_COMPLETE_INT) {
		//get current idx 
		idx = ast_formatter_read(ast_formatter, AST_FORMATTER_CTRL);

		FORMATTER_DBG("idx %x, %x , %x : %d \n", AST_GET_CUR_IDX(idx), AST_GET_VALID_IDX(idx), AST_GET_BIBLT_IDX(idx), jiffies/HZ);

		bitblt.src_base = ast_formatter->dest[AST_GET_VALID_IDX(idx)];		
		bitblt.dest_base = astfb_info->fix.smem_start;

		FORMATTER_DBG("src addr : %x , dest : %x \n", bitblt.src_base, bitblt.dest_base);
		
		bitblt.src_format = ASTFB_COLOR_YUV422;
		bitblt.dest_format = ASTFB_COLOR_RGB888;

		bitblt.src_pitch = (ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff) * 2;


		FORMATTER_DBG("src: w : %d , h : %d \n", ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff, (ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) >> 16));
		
		if((ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff) > astfb_info->var.xres)
			bitblt.src_w = astfb_info->var.xres;
		else
			bitblt.src_w = ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) & 0xfff;

		if((ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) >> 16) > astfb_info->var.yres)
			bitblt.src_h = astfb_info->var.yres;
		else
			bitblt.src_h = ast_formatter_read(ast_formatter, AST_FORMATTER_H_W) >> 16;

		FORMATTER_DBG("cut w : %d , h : %d \n", bitblt.src_w, bitblt.src_h);
		bitblt.src_x = 0;
		bitblt.src_y = 0;


		bitblt.dest_pitch = (astfb_info->var.xres) * 4;
		
		bitblt.dest_w = astfb_info->var.xres;
		bitblt.dest_h = astfb_info->var.yres;

		FORMATTER_DBG("fb x : %d , y : %d \n", astfb_info->var.xres, astfb_info->var.yres);

		bitblt.dest_x = 0;
		bitblt.dest_y = 0;
#ifdef CONFIG_AST_BITBLT	
		ast_h264_bitblt(&bitblt);
#endif
		//clr 
		ast_formatter_write(ast_formatter, FORMATTER_COMPLETE_INT | FORMATTER_COMPLETE_INT_EN, AST_FORMATTER_ISR);
	} else {
		printk("ast_formatter_handler ERROR \n");
	}
	
	return IRQ_HANDLED;
}

ssize_t formatter_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	u32 mem = ast_formatter->mem_virt + *f_pos;

	if (*f_pos > FORMATTER_DEST_SIZE) 
		goto nothing;
	if (*f_pos + count > FORMATTER_DEST_SIZE)
		count = FORMATTER_DEST_SIZE - *f_pos;

	if(count > FORMATTER_DEST_SIZE)
		return -EFAULT;

	if (copy_to_user(buf, mem, count))
		return -EFAULT;

	FORMATTER_DBG("count %d, f_pos %x \n",count , *f_pos);
	*f_pos += count;
	return count;

nothing:
	return 0;
	
}

ssize_t formatter_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	u32 mem = ast_formatter->mem_virt + 2 *FORMATTER_DEST_SIZE + *f_pos;

	if (*f_pos > FORMATTER_DEST_SIZE) 
		goto nothing;
	if (*f_pos + count > FORMATTER_DEST_SIZE)
		count = FORMATTER_DEST_SIZE - *f_pos;

	if(count > FORMATTER_DEST_SIZE)
		return -EFAULT;

	if (copy_from_user(mem, buf, count))
		return -EFAULT;

	FORMATTER_DBG("count %d, f_pos %x \n",count , *f_pos);

	*f_pos += count;
	return count;

nothing:
	return 0;
}

static long formatter_ioctl(struct file *fp,
			     unsigned int cmd, unsigned long arg)
{
	long ret = 0;
//	struct miscdevice *c = fp->private_data;
	int intr = 0;
	struct formatter_yuv420_to_422 *formatter = (void __user *)arg;

	switch(cmd) {
		case 0:
			formatter->src = ast_formatter->dest[0];
			formatter_yuv420_to_422(formatter);

			break;
			
		default:			
			printk("formatter_ioctl command fail\n");
			ret = -ENOTTY;
			break;			
	}

	return ret;

}

static int ast_formatter_open(struct inode *inode, struct file *file)
{
//	struct miscdevice *c = file->private_data;
//	struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);

	spin_lock(&ast_formatter->lock);

	if(ast_formatter->is_open) {
		spin_unlock(&ast_formatter->lock);
		return -1;
	}
	ast_formatter->is_open = true;

	spin_unlock(&ast_formatter->lock);

	return 0;	
}

static int ast_formatter_release(struct inode *inode, struct file *file)
{
//	struct miscdevice *c = file->private_data;
//	struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);

	spin_lock(&ast_formatter->lock);
	ast_formatter->is_open = false;
	spin_unlock(&ast_formatter->lock);


	return 0;
}

static int formatter_mmap(struct file * file, struct vm_area_struct * vma)
{
//       struct miscdevice *c = file->private_data;
//        struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);
        size_t size = vma->vm_end - vma->vm_start;

        vma->vm_flags |= VM_IO;
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
printk("~~~ mmap address %x ~~~\n", ast_formatter->mem_phy);
        if (io_remap_pfn_range(vma, vma->vm_start, ((u32)ast_formatter->mem_phy >> PAGE_SHIFT),
                        size,
                        vma->vm_page_prot)) {
                printk(KERN_ERR "remap_pfn_range faile at %s()\n", __func__);
                return -EAGAIN;
        }

        return 0;
}

static const struct file_operations ast_formatter_fops = {
        .owner  		= THIS_MODULE,
	.llseek 		= no_llseek,			
//        .unlocked_ioctl = ast_h264_ioctl,
	.read		= formatter_read,
	.write 		= formatter_write,
        .open 		= ast_formatter_open,
        .mmap		= formatter_mmap,
        .release 		= ast_formatter_release,
};

struct miscdevice ast_formatter_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-formatter",
	.fops = &ast_formatter_fops,
};

static int ast_formatter_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret=0;

	ast_formatter = kzalloc(sizeof(struct ast_formatter_data), GFP_KERNEL);
	if(ast_formatter == NULL) {
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

	ast_formatter->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_formatter->reg_base) {
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


	ast_formatter->mem_phy = res->start;
	ast_formatter->mem_size = resource_size(res);
	if(ast_formatter->mem_size < (FORMATTER_DEST_NO * FORMATTER_DEST_SIZE))
		printk("memory not enough ~~~~\n");

	printk("ast_formatter->mem_phy:  %x , each size %x \n", ast_formatter->mem_phy, FORMATTER_DEST_SIZE);

	ast_formatter->mem_virt = ioremap(res->start, resource_size(res));
	if (!ast_formatter->mem_virt) {
		ret = -EIO;
		goto out_region;
	}

	ast_formatter->irq = platform_get_irq(pdev, 0);
	if (ast_formatter->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_formatter->irq, ast_formatter_handler, IRQF_SHARED,
			  "ast-formatter", ast_formatter);
	
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_formatter->irq);
		goto out_region;
	}

	ret = misc_register(&ast_formatter_misc);
	if (ret){		
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}
	spin_lock_init(&ast_formatter->lock);

	platform_set_drvdata(pdev, ast_formatter);

	ast_formatter_ctrl_init(ast_formatter);
	
	printk(KERN_INFO "ast_formatter: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_formatter->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_formatter_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_formatter_data *ast_formatter = platform_get_drvdata(pdev);

	free_irq(ast_formatter->irq, &ast_formatter);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_formatter->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

static struct platform_driver ast_formatter_driver = {
	.driver         = {
		.name   = "ast-formatter",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_formatter_driver, ast_formatter_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST formatter driver");
MODULE_LICENSE("GPL");
