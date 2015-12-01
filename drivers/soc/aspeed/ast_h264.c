/********************************************************************************
* File Name     : ast_h264.c
* Author         : Ryan Chen
* Description   : AST H264 Controller
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
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/time.h>
#include <linux/jiffies.h>

#include <linux/sched.h>  
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <plat/ast_formatter.h>

/***********************************************************************/
#define AST_H264_CMD					0x00
#define AST_H264_STS					0x20
#define AST_H264_ISR1					0x24
#define AST_H264_ISR2					0x28
#define AST_H264_ISR3					0x2C
#define AST_H264_MODE					0x30
#define AST_H264_DECODE_P1				0x100
#define AST_H264_DECODE_P2				0x104
#define AST_H264_DECODE_P3				0x108
#define AST_H264_DECODE_P4				0x10C
#define AST_H264_DECODE_P5				0x110
#define AST_H264_DECODE_P6				0x114
#define AST_H264_DECODE_P7				0x118
#define AST_H264_DECODE_P8				0x11C
#define AST_H264_DECODE_P9				0x120
#define AST_H264_DECODE_P10			0x124
#define AST_H264_DECODE_P11			0x128
#define AST_H264_DECODE_P12			0x12C
#define AST_H264_HEADER_P1				0x500
#define AST_H264_HEADER_P2				0x504
#define AST_H264_HEADER_P3				0x508
#define AST_H264_HEADER_P4				0x50C
#define AST_H264_HEADER_P5				0x510
#define AST_H264_HEADER_P6				0x514
#define AST_H264_HEADER_P7				0x518
#define AST_H264_HEADER_P8				0x51C
#define AST_H264_HEADER_P9				0x520
#define AST_H264_HEADER_P11			0x528
#define AST_H264_HEADER_P13			0x530
#define AST_H264_HEADER_P49			0x5C0

#define AST_H264_BASE1					0x800
#define AST_H264_BASE2					0x804
#define AST_H264_BASE3					0x808
#define AST_H264_BASE4					0x80C
#define AST_H264_BASE5					0x810
#define AST_H264_BASE6					0x814
#define AST_H264_BASE7					0x818
#define AST_H264_BASE8					0x81C
#define AST_H264_BASE9					0x820
#define AST_H264_BASE10					0x824
#define AST_H264_BASE11					0x828
#define AST_H264_BASE12					0x82C
#define AST_H264_BASE13					0x830
#define AST_H264_BASE14					0x834
#define AST_H264_BASE15					0x838
#define AST_H264_BASE16					0x83C
#define AST_H264_AXI_CONF				0x840
#define AST_H264_ISR						0x1004
#define AST_H264_IER						0x1008

#define AST_H264_BS_BASE				0x2000
#define AST_H264_BS_EP					0x2004
#define AST_H264_BS_WP					0x2008
#define AST_H264_BS_RP					0x200C
#define AST_H264_BS_CTRL				0x2010

/*************************************************************************************/
#define AST_H264_TRIGGER_DECODE		(0x1 << 14)

/* define AST_H264_ISR1					0x24 */
#define H264_AXI_BUS_ERROR				(0x1 << 6)
#define H264_DECODE_ERROR				(0x1 << 5)
#define H264_PIC_DECODE_END			(0x1 << 3)
#define H264_SLICE_DECODE_END			(0x1)

#define AST_H264_INT_MASK				(0x7)

#define AST_H264_BS_ERR_INT				(0x1 << 2)
#define AST_H264_AXIFREE_INT			(0x1 << 1)
#define AST_H264_DECODE_INT				(0x1)

#define AST_H264_GET_WORK_PLANE(x)		(x & 0xf)


#define AST_H264_GET_WIDTH(x)			((((x >> 8) & 0x7f) + 1) << 4)
#define AST_H264_GET_HEIGHT(x)			(((x & 0x7f) + 1) << 4)

/*************************************************************************************/


struct ast_h264_mem_info {
	unsigned int h264_mem_phy;
	unsigned int h264_mem_size;
	unsigned int h264_bs_size;
	unsigned int h264_pic_size;
	unsigned int h264_pic_number;
};

/***********************************************************************/

//IOCTL ..
#define H264IOC_BASE       'H'

//1. h264 reg base , 2. interrupt enable/disable
#define AST_H264_IOCGBASE		_IOR(H264IOC_BASE, 0, int)
#define AST_H264_IOSINT			_IOW(H264IOC_BASE, 1, int)
#define AST_H264_IOCGMEM		_IOR(H264IOC_BASE, 2, struct ast_h264_mem_info)

/***********************************************************************/

//#define CONFIG_AST_H264_DEBUG

#ifdef CONFIG_AST_H264_DEBUG
	#define H264_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define H264_DBG(fmt, args...)
#endif

/***********************************************************************/
#define BITSTREAM_SIZE				0x800000		//8MB
//#define DECODE_PICTURE_NUM			16
#define DECODE_PICTURE_NUM			3
#define DECODE_PICTURE_SIZE			0x2FD000		//1920 * 1088 * 1.5  = 0x2FD000 ~~ 3MB

static struct ast_h264_data {
	struct device		*misc_dev;
	void __iomem		*reg_base;			/* virtual */
	int 				irq;	
	phys_addr_t		*mem_phy;			/* mem size*/	
	u32				mem_virt;	
	u32				mem_size;			/* mem size*/
	void __iomem		*bs_phy;				/* bitstream src*/
	phys_addr_t		pic_phy[DECODE_PICTURE_NUM];			/* decode picture src*/
	u32				pic_virt[DECODE_PICTURE_NUM];			/* decode picture src*/	
	u32				bs_size;				/* bitstream size*/		
	bool is_open;	
	spinlock_t lock;		
	struct fasync_struct *async_queue;	
	int 				frame_count;
	int 				stop_trigger;	
}__attribute__ ((aligned));

static inline void
ast_h264_write(struct ast_h264_data *ast_h264, u32 val, u32 reg)
{
//	H264_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_h264->reg_base + reg);
}

static inline u32
ast_h264_read(struct ast_h264_data *ast_h264, u32 reg)
{
	u32 val = readl(ast_h264->reg_base + reg);
//	H264_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

static void ast_h264_ctrl_init(struct ast_h264_data *ast_h264)
{
	int i = 0;

	ast_h264->bs_phy = ast_h264->mem_phy; 
	ast_h264->bs_size = BITSTREAM_SIZE;

	H264_DBG("MEM : %x, Size : %x , Pic : %x, no %d \n", (u32)ast_h264->mem_phy, ast_h264->mem_size, (u32)ast_h264->pic_phy, DECODE_PICTURE_NUM);		

	ast_h264_write(ast_h264, (u32)ast_h264->bs_phy, AST_H264_BS_BASE);
	ast_h264_write(ast_h264, BITSTREAM_SIZE, AST_H264_BS_EP);
	ast_h264_write(ast_h264, 0, AST_H264_BS_WP);	

	for(i = 0; i < DECODE_PICTURE_NUM; i++) {
		ast_h264->pic_phy[i] = ast_h264->bs_phy + BITSTREAM_SIZE + (i * DECODE_PICTURE_SIZE); 
		ast_h264->pic_virt[i] = ast_h264->mem_virt + BITSTREAM_SIZE + (i * DECODE_PICTURE_SIZE); 
		ast_h264_write(ast_h264, (u32)ast_h264->pic_phy[i], AST_H264_BASE1 + (i * 4));
	}

	// interrupt enable
	ast_h264_write(ast_h264, 0x8000000d, AST_H264_IER);				//b31 interrupt for global
	ast_h264_write(ast_h264, 0x00000068, AST_H264_ISR3);	
	ast_h264_write(ast_h264, 0x00000001, AST_H264_MODE);				//b0 frame base decode end interrupt
	ast_h264_write(ast_h264, 0x0000f7ff, AST_H264_HEADER_P6);		//default error status 
	ast_h264_write(ast_h264, 0x0000d9ff, AST_H264_HEADER_P7);		//default error status 
	ast_h264_write(ast_h264, 0x00000206, AST_H264_DECODE_P1);		//b1 b2 for hw decode sps header	

}

/***************************************************************************/
static irqreturn_t ast_h264_handler(int this_irq, void *dev_id)
{
	struct ast_h264_data *ast_h264 = dev_id;
	u32 sts = ast_h264_read(ast_h264, AST_H264_ISR);
	u32 sts1 = ast_h264_read(ast_h264, AST_H264_ISR1);
	u32 pic_no = 0;
	u32 xy_infor = 0;
	struct formatter_yuv420_to_422 yuv420_to_422;

	if(sts & AST_H264_INT_MASK) {
		//check intrerrupt 
		if(sts & AST_H264_AXIFREE_INT) {
			printk("axi free int \n");
		}

		if(sts & AST_H264_BS_ERR_INT) {
			printk("bs error int ERROR \n");		
		}

		if(sts & AST_H264_DECODE_INT) {	
			if(sts1 & H264_DECODE_ERROR) {
				printk("decode error int %x , %x \n", ast_h264_read(ast_h264, AST_H264_HEADER_P1), ast_h264_read(ast_h264, AST_H264_HEADER_P2));
				ast_h264_write(ast_h264, ast_h264_read(ast_h264, AST_H264_HEADER_P1), AST_H264_HEADER_P1);
				ast_h264_write(ast_h264, ast_h264_read(ast_h264, AST_H264_HEADER_P2), AST_H264_HEADER_P2);								
				ast_h264_write(ast_h264, H264_DECODE_ERROR, AST_H264_ISR1);				
				ast_h264->stop_trigger = 1;
			} else {
				if(sts1 & H264_PIC_DECODE_END) {
	//				ast_h264_write(ast_h264, H264_PIC_DECODE_END, AST_H264_ISR1);
					ast_h264_write(ast_h264, H264_PIC_DECODE_END | H264_SLICE_DECODE_END, AST_H264_ISR1);
					pic_no = AST_H264_GET_WORK_PLANE(ast_h264_read(ast_h264, AST_H264_HEADER_P49));
					xy_infor = ast_h264_read(ast_h264, AST_H264_DECODE_P3);

					//trigger decode 
					if(!ast_h264->stop_trigger) {
						H264_DBG("[%d] : addr : %x , pic : %d : %d \n", ast_h264->frame_count, ast_h264->pic_phy[pic_no], pic_no, jiffies);
						yuv420_to_422.src = ast_h264->pic_phy[pic_no];
						yuv420_to_422.src_virt = ast_h264->pic_virt[pic_no];						
						yuv420_to_422.width = AST_H264_GET_WIDTH(xy_infor);
						yuv420_to_422.height = AST_H264_GET_HEIGHT(xy_infor); 	
						formatter_yuv420_to_422(&yuv420_to_422);
						ast_h264_write(ast_h264, AST_H264_TRIGGER_DECODE, AST_H264_CMD);
						ast_h264->frame_count++;						
					}
				}
				printk("[%d ] 264 %d \n", ast_h264->frame_count, jiffies * 1000/HZ);

				if(sts1 & H264_SLICE_DECODE_END) {
					printk("slice decode end TODO ...\n");
					ast_h264_write(ast_h264, H264_SLICE_DECODE_END, AST_H264_ISR1);
				}
			}
		}
	}else {
		printk("ast_h264_handler INT ERROR \n");
	}
	return IRQ_HANDLED;
}

struct ast_h264_data *ast_h264;

static long ast_h264_ioctl(struct file *fp,
			     unsigned int cmd, unsigned long arg)
{
	long ret = 0;
//	struct miscdevice *c = fp->private_data;
//	struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);
	int intr = 0;
	struct ast_h264_mem_info mem_info;

	switch(cmd) {
		case AST_H264_IOCGBASE:
			ret = __put_user(ast_h264->reg_base, (int __user *)arg);
			break;
			
		case AST_H264_IOSINT:	
			ret = __get_user(intr, (int __user *)arg);
			break;

		case AST_H264_IOCGMEM:	
			mem_info.h264_mem_phy = ast_h264->mem_phy;			
			mem_info.h264_mem_size = ast_h264->mem_size;			
			mem_info.h264_bs_size = BITSTREAM_SIZE;
			mem_info.h264_pic_size = DECODE_PICTURE_SIZE;
			mem_info.h264_pic_number = DECODE_PICTURE_NUM;
			ret =copy_to_user(arg, &mem_info, sizeof(struct ast_h264_mem_info));
			break;
			
		default:			
			printk("ast_h264_ioctl command fail\n");
			ret = -ENOTTY;
			break;			
	}

	return ret;
}

static int ast_h264_fasync(int fd, struct file *file, int mode)
{
//	struct miscdevice *c = file->private_data;
//	struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);
	return fasync_helper(fd, file, mode, &ast_h264->async_queue);
}

/** @note munmap handler is done by vma close handler */
static int ast_h264_mmap(struct file * file, struct vm_area_struct * vma)
{
//       struct miscdevice *c = file->private_data;
//        struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);
        size_t size = vma->vm_end - vma->vm_start;
        vma->vm_private_data = ast_h264;

        if (PAGE_ALIGN(size) > ast_h264->mem_size) {
                        printk(KERN_ERR "required length exceed the size "
                                   "of physical sram (%x)\n", ast_h264->mem_size);
                        return -EAGAIN;
        }

        vma->vm_flags |= VM_IO;
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
        if (io_remap_pfn_range(vma, vma->vm_start, 
				((u32)0x8d800000 >> PAGE_SHIFT),
                        	size,
                        	vma->vm_page_prot)) {
                printk(KERN_ERR "remap_pfn_range faile at %s()\n", __func__);
                return -EAGAIN;
        }

        return 0;
}

static int ast_h264_open(struct inode *inode, struct file *file)
{
//	struct miscdevice *c = file->private_data;
//	struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);

	H264_DBG("\n");
	spin_lock(&ast_h264->lock);

	if(ast_h264->is_open) {
		spin_unlock(&ast_h264->lock);
		return -1;
	}
	ast_h264->is_open = true;
	ast_h264->frame_count = 0;

	spin_unlock(&ast_h264->lock);

	return 0;	
}

static int ast_h264_release(struct inode *inode, struct file *file)
{
//	struct miscdevice *c = file->private_data;
//	struct ast_h264_data *ast_h264 = dev_get_drvdata(c->this_device);

	H264_DBG("\n");

	spin_lock(&ast_h264->lock);
	ast_h264->is_open = false;
	spin_unlock(&ast_h264->lock);


	return 0;
}

static const struct file_operations ast_h264_fops = {
        .owner  		= THIS_MODULE,
	.llseek 		= no_llseek,			
        .unlocked_ioctl = ast_h264_ioctl,
        .open 		= ast_h264_open,
        .release 		= ast_h264_release,
	.mmap		= ast_h264_mmap,
	.fasync 		= ast_h264_fasync,
};

struct miscdevice ast_h264_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-h264",
	.fops = &ast_h264_fops,
};

static int ast_h264_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret=0;

	H264_DBG(" \n");	

	ast_h264 = kzalloc(sizeof(struct ast_h264_data), GFP_KERNEL);
	if(ast_h264 == NULL) {
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

	ast_h264->reg_base = ioremap(res->start, resource_size(res));

	if (!ast_h264->reg_base) {
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

	ast_h264->mem_phy = res->start;
	ast_h264->mem_size = resource_size(res);
	printk("ast_h264->mem_phy = %x, mem_size = %x \n",ast_h264->mem_phy, ast_h264->mem_size);

	if(ast_h264->mem_size < (BITSTREAM_SIZE + (DECODE_PICTURE_NUM * DECODE_PICTURE_SIZE))) {
		printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		goto out_region;
	}

	ast_h264->mem_virt = ioremap(res->start, resource_size(res));
	if (!ast_h264->mem_virt) {
		ret = -EIO;
		goto out_region;
	}

	ast_h264->irq = platform_get_irq(pdev, 0);
	if (ast_h264->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_h264->irq, ast_h264_handler, IRQF_SHARED,
			  "ast-h264", ast_h264);
	
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_h264->irq);
		goto out_region;
	}

	ret = misc_register(&ast_h264_misc);
	if (ret){		
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}
	spin_lock_init(&ast_h264->lock);

	platform_set_drvdata(pdev, ast_h264);

//	dev_set_drvdata(ast_h264_misc.this_device, ast_h264);

	ast_h264_ctrl_init(ast_h264);		
	printk(KERN_INFO "ast_h264: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_h264->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_h264_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_h264_data *ast_h264 = platform_get_drvdata(pdev);

	H264_DBG("ast_h264_remove\n");

	misc_deregister(&ast_h264_misc);

	free_irq(ast_h264->irq, &ast_h264);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_h264->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

static struct platform_driver ast_h264_driver = {
	.remove 		= ast_h264_remove,
	.driver         = {
		.name   = "ast-h264",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_h264_driver, ast_h264_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST H264 driver");
MODULE_LICENSE("GPL");
