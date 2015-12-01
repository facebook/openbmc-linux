/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-i2c.c
* Author        : Ryan chen
* Description   : ASPEED I2C Device
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
*    1. 2012/07/30 ryan chen create this file
*
********************************************************************************/

#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/sizes.h>
#include <asm/arch/devs.h>
#include <asm/arch/platform.h>
#include <asm/arch/irqs.h>
#include <asm/arch/aspeed.h>
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/ast_i2c.h>
#include <plat/ast-scu.h>
#endif

//#define AST_I2C_IRQ_DEBUG

#ifdef AST_I2C_IRQ_DEBUG
#define DEBUG
#define I2C_IRQ_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define I2C_IRQ_DBUG(fmt, args...)
#endif

/*******************************************************************/
#define AST_I2CG_ISR				0x00

#define AST_I2C_DEV0_IRQ		0x1 
#define AST_I2C_DEV1_IRQ		(0x1 << 1)
#define AST_I2C_DEV2_IRQ		(0x1 << 2)
#define AST_I2C_DEV3_IRQ		(0x1 << 3)
#define AST_I2C_DEV4_IRQ		(0x1 << 4)
#define AST_I2C_DEV5_IRQ		(0x1 << 5)
#define AST_I2C_DEV6_IRQ		(0x1 << 6)
#define AST_I2C_DEV7_IRQ		(0x1 << 7)
#define AST_I2C_DEV8_IRQ		(0x1 << 8)
#define AST_I2C_DEV9_IRQ		(0x1 << 9)
#define AST_I2C_DEV10_IRQ		(0x1 << 10)
#define AST_I2C_DEV11_IRQ		(0x1 << 11)
#define AST_I2C_DEV12_IRQ		(0x1 << 12)
#define AST_I2C_DEV13_IRQ		(0x1 << 13)
#define AST_I2C_DEV14_IRQ		(0x1 << 14)

#define AST_I2CG_ISR_TARGET		0x08
#ifdef AST_SOC_G5
#define AST_I2CG_CTRL			0x0C
#define I2C_SRAM_BUFF_EN	0x1
#endif



/*******************************************************************/

#if defined (AST_SOC_G5)
struct buf_page page_info;

static void pool_buff_page_init(u32 buf_pool_addr) 
{
	page_info.flag = 0;
	page_info.page_size = AST_I2C_PAGE_SIZE;
	page_info.page_addr_point = 0;
	page_info.page_addr = buf_pool_addr;

}

extern u8 request_pool_buff_page(struct buf_page **req_page)
{
	*req_page = &page_info;
	return 0;

}

//TODO check free ?
extern void free_pool_buff_page(struct buf_page *req_page)
{
	return;
}

#elif defined(AST_SOC_G4)
#define I2C_PAGE_SIZE 8
struct buf_page page_info[I2C_PAGE_SIZE] = 
{  
	[0] = {
		.flag = 0,
		.page_no = 0,
		.page_size = 256,
		.page_addr_point = 0,	
	},
	[1] = {
		.flag = 0,
		.page_no = 1,			
		.page_size = 256,
		.page_addr_point = 0,		
	},
	[2] = {
		.flag = 0,
		.page_no = 2,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[3] = {
		.flag = 0,
		.page_no = 3,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[4] = {
		.flag = 0,
		.page_no = 4,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[5] = {
		.flag = 0,
		.page_no = 5,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[6] = {
		.flag = 0,
		.page_no = 6,			
		.page_size = 256,
		.page_addr_point = 0,
	},
	[7] = {
		.flag = 0,
		.page_no = 7,			
		.page_size = 256,
		.page_addr_point = 0,
	},
};

static void pool_buff_page_init(u32 buf_pool_addr) 
{
	u32 offset;
	int i ,j;

	for(i=0;i<I2C_PAGE_SIZE;i++) {
		offset = 0;
		for(j=0;j<i;j++)
			offset += page_info[i].page_size;
		
		page_info[i].page_addr = buf_pool_addr + offset;
//		I2CDBUG( "page[%d],addr :%x \n", i, page_info[i].page_addr);
	}

}

extern u8 request_pool_buff_page(struct buf_page **req_page)
{
	int i;
	//TODO
	spinlock_t	lock;
	spin_lock(&lock);	
	for(i=0;i<I2C_PAGE_SIZE;i++) {
		if(page_info[i].flag ==0) {
			page_info[i].flag = 1;
			*req_page = &page_info[i];
//			I2CDBUG( "request page addr %x \n", page_info[i].page_addr);
			break;
		}
	}
	spin_unlock(&lock);	
	return 0;
}

extern void free_pool_buff_page(struct buf_page *req_page)
{
	req_page->flag = 0;
//	I2CDBUG( "free page addr %x \n", req_page->page_addr);	
	req_page = NULL;
}

#elif defined (AST_SOC_G3)
#define I2C_PAGE_SIZE 5

struct buf_page page_info[I2C_PAGE_SIZE] = 
{  
	[0] = {
		.flag = 0,
		.page_size = 128,
	},
	[1] = {
		.flag = 0,
		.page_size = 32,
	},
	[2] = {
		.flag = 0,
		.page_size = 32,
	},
	[3] = {
		.flag = 0,
		.page_size = 32,
	},
	[4] = {
		.flag = 0,
		.page_size = 32,
	},
};

static void pool_buff_page_init(u32 buf_pool_addr) 
{

	u32 offset;
	int i ,j;

	for(i=0;i<I2C_PAGE_SIZE;i++) {
		offset = 0;
		for(j=0;j<i;j++)
			offset += page_info[i].page_size;
		
		page_info[i].page_addr = buf_pool_addr + offset;
		page_info[i].page_addr_point = page_info[i].page_addr/4;
//		I2C_IRQ_DBUG("page[%d],addr :%x , point : %d\n", i, page_info[i].page_addr, page_info[i].page_addr_point);
	}
}

extern u8 request_pool_buff_page(struct buf_page **req_page)
{
	int i;
	//TODO
	spinlock_t	lock;
	spin_lock(&lock);	
	for(i=0;i<I2C_PAGE_SIZE;i++) {
		if(page_info[i].flag ==0) {
			page_info[i].flag = 1;
			*req_page = &page_info[i];
			spin_unlock(&lock);
			return 1;
		}
	}
	spin_unlock(&lock);	
	return 0;

}

//TODO check free ?
extern void free_pool_buff_page(struct buf_page *req_page)
{
	req_page->flag = 0;
	req_page = NULL;
}

#else 
//DO nothing
static void pool_buff_page_init(void) {}
extern u8 request_pool_buff_page(struct buf_page **req_page) {return 0;}
extern void free_pool_buff_page(struct buf_page *req_page) {}
#endif

void __iomem	*i2c_reg_base;	

static void ast_i2c_global_interrupt(unsigned int irq, struct irq_desc *desc)
{
	u32 i = 0;
	u32 sts = readl(i2c_reg_base + AST_I2CG_ISR);
	I2C_IRQ_DBUG("ast_i2c_global_interrupt %x \n",sts);
	//should use run-roubin i2c
#if 1	
	for (i = 0; sts != 0; i++, sts >>= 1) {
		if (sts & 1) {
			I2C_IRQ_DBUG("gen irq %d\n", i);
			generic_handle_irq(IRQ_I2C_CHAIN_START + i);
			I2C_IRQ_DBUG("gen irq %d Exit \n", i);
		}
	}	

#else
	if(sts & AST_I2C_DEV0_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE1\n");
		generic_handle_irq(IRQ_I2C_DEV0);
	}

	if(sts & AST_I2C_DEV1_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE2\n");
		generic_handle_irq(IRQ_I2C_DEV1);
	}

	if(sts & AST_I2C_DEV2_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE3\n");
		generic_handle_irq(IRQ_I2C_DEV2);
	}

	if(sts & AST_I2C_DEV3_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE4\n");
		generic_handle_irq(IRQ_I2C_DEV3);
	}

	if(sts & AST_I2C_DEV4_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE5\n");
		generic_handle_irq(IRQ_I2C_DEV4);
	}

	if(sts & AST_I2C_DEV5_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE6\n");
		generic_handle_irq(IRQ_I2C_DEV5);
	}

	if(sts & AST_I2C_DEV6_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE7\n");
		generic_handle_irq(IRQ_I2C_DEV6);
	}

	if(sts & AST_I2C_DEV7_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE8\n");
		generic_handle_irq(IRQ_I2C_DEV7);
	}

	if(sts & AST_I2C_DEV8_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE9\n");
		generic_handle_irq(IRQ_I2C_DEV8);
	}

	if(sts & AST_I2C_DEV9_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE10\n");
		generic_handle_irq(IRQ_I2C_DEV9);
	}

	if(sts & AST_I2C_DEV10_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE11\n");
		generic_handle_irq(IRQ_I2C_DEV10);
	}

	if(sts & AST_I2C_DEV11_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE12\n");
		generic_handle_irq(IRQ_I2C_DEV11);
	}

	if(sts & AST_I2C_DEV12_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE13\n");
		generic_handle_irq(IRQ_I2C_DEV12);
	}

	if(sts & AST_I2C_DEV13_IRQ) {
//		I2C_IRQ_DBUG("ISR_DEVICE14\n");
		generic_handle_irq(IRQ_I2C_DEV13);
	}
#endif

}		


static void ast_i2c_ack_irq(struct irq_data *d)
{
//	I2C_IRQ_DBUG("ack irq[%d]\n",d->irq);
}

static void ast_i2c_mask_irq(struct irq_data *d)
{
//	I2C_IRQ_DBUG("mask irq[%d]\n",d->irq);
}

static void ast_i2c_unmask_irq(struct irq_data *d)
{
//	I2C_IRQ_DBUG("unmask irq[%d]\n",d->irq);
}

static struct irq_chip ast_i2c_irq_chip = {
	.name		= "i2c_irq_chip",
	.irq_ack		= ast_i2c_ack_irq,
	.irq_mask		= ast_i2c_mask_irq,
	.irq_unmask	= ast_i2c_unmask_irq,
};

static int __init ast_i2c_irq_init(void)
{
	int irq = 0;
	u8 *buf_pool;

	I2C_IRQ_DBUG("ast_i2c_irq_init \n");

#if defined(CONFIG_I2C_AST) || defined(CONFIG_I2C_AST_MODULE)
	//SCU I2C Reset 
	ast_scu_init_i2c();

	i2c_reg_base = ioremap(AST_I2C_BASE, SZ_16);
	if (!i2c_reg_base) {
		printk("ast_i2c_irq_init ERROR \n");
		return -1;
	}
	

#if defined (AST_I2C_POOL_BUFF_2048)
	buf_pool= ioremap(AST_I2C_BASE+0x800, 2048);
	if (!buf_pool) {
		printk("buf_pool ERROR \n");
		return -1;
	}
	pool_buff_page_init((u32)buf_pool); 
	
#elif defined (AST_I2C_POOL_BUFF_256)
	buf_pool = ioremap(AST_I2C_BASE+0x200, 256);
	if (!buf_pool) {
		printk("buf_pool ERROR \n");
		return -1;
	}
	pool_buff_page_init((u32)buf_pool); 

#elif defined (AST_I2C_POOL_BUFF_16)
	buf_pool = ioremap(AST_I2C_BASE+0x200, 224);
	if (!buf_pool) {
		printk("buf_pool ERROR \n");
		return -1;
	}
	pool_buff_page_init((u32)buf_pool); 

#ifdef AST_SOC_G5	
	if((MASTER_XFER_MODE == BUFF_MODE) || (SLAVE_XFER_MODE == BUFF_MODE) ||
		(MASTER_XFER_MODE == INC_DMA_MODE) || (SLAVE_XFER_MODE == INC_DMA_MODE))
		writel(I2C_SRAM_BUFF_EN, i2c_reg_base + AST_I2CG_CTRL);
#endif
	
#else
	buf_pool = 0;
#endif
	
	for (irq = 0; irq < ARCH_NR_I2C; irq++) {
		irq_set_chip_and_handler(irq + IRQ_I2C_CHAIN_START, &ast_i2c_irq_chip,
					 handle_level_irq);
		set_irq_flags(irq + IRQ_I2C_CHAIN_START, IRQF_VALID);
	}

	irq_set_chained_handler(IRQ_I2C, ast_i2c_global_interrupt);

#endif

	return 0;
}
core_initcall(ast_i2c_irq_init);

