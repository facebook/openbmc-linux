/********************************************************************************
* File Name     : arch/arm/mach-aspeed/ast-lpc.c 
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>

#include <plat/regs-lpc.h>
#include <plat/ast-snoop.h>


#define AST_LPC_DEBUG

#ifdef AST_LPC_DEBUG
#define LPCDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define LPCDBUG(fmt, args...)
#endif

#define LPCMSG(fmt, args...) printk(fmt, ## args)

static u32 ast_lpc_base;

static inline u32 
ast_lpc_read(u32 reg)
{
	u32 val;
		
	val = readl(ast_lpc_base + reg);
	
	LPCDBUG("ast_lpc_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_lpc_write(u32 val, u32 reg) 
{
	LPCDBUG("ast_lpc_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_lpc_base + reg);
}

/******************************************************************************/

//Suppose you are going to snoop 0x80 ~ 0x87
//snoop_init(0x80, 0x7, WORD_MODE, buf_dma, (SNOOP_DMA_BOUNDARY / 4)); //register in unit of DWORD
#if 0
extern void
ast_lpc_snoop_dma_enable(u16 port_number, u8 port_mask, u8 mode, dma_addr_t dma_base, u16 size)
{
	write_register(0x1e789134, (port_mask << 16) + port_number);
	write_register(0x1e7890d0, dma_base);
	write_register(0x1e7890d4, (size - 1));
	write_register(0x1e789130, (mode << 4) | ENABLE_DMA_INTERRUPT | ENABLE_POST_CODE_FUNCTION | ENABLE_SNOOP_DMA_MODE);

	//Enable error interrupt to check LPC reset
	write_register_or(0x1e789008, 1);

}

EXPORT_SYMBOL(ast_lpc_snoop_dma_init);
#endif

extern irqreturn_t ast_snoop_handler(int this_irq, void *dev_id)
{
	u32 snoop_sts;
	struct ast_snoop *snoop = dev_id;
	
	snoop_sts = ast_lpc_read(AST_LPC_HICR6);
	if((snoop_sts & (LPC_HICR6_STR_SNP1W | LPC_HICR6_STR_SNP0W)) == 0)
		return IRQ_NONE;

	if(snoop_sts & LPC_HICR6_STR_SNP0W) {
		snoop->snoop_ch0->snoop_data = GET_LPC_SNPD0(ast_lpc_read(AST_LPC_SNPWDR)); 
		//clear 
		ast_lpc_write(LPC_HICR6_STR_SNP0W, AST_LPC_HICR6);
	}

	if(snoop_sts & LPC_HICR6_STR_SNP1W) {
		snoop->snoop_ch1->snoop_data = GET_LPC_SNPD1(ast_lpc_read(AST_LPC_SNPWDR)); 
		//clear 
		ast_lpc_write(LPC_HICR6_STR_SNP1W, AST_LPC_HICR6);
		
	}

	return IRQ_HANDLED;

}
EXPORT_SYMBOL(ast_snoop_handler);

extern irqreturn_t ast_snoop_dma_handler(int this_irq, void *dev_id)
{
	u32 snoop_dma_sts, lpc_sts;
	struct ast_snoop_dma_channel *snoop_dma_ch = dev_id;

	snoop_dma_sts = ast_lpc_read(AST_LPC_PCCR2);

	lpc_sts = ast_lpc_read(AST_LPC_HICR2);
	
	printk("ISR : snoop_dma_sts = %x , lpc_sts = %x \n",snoop_dma_sts, lpc_sts);

	if(lpc_sts & LPC_LRST) {
		printk("LPC RST === > \n");
		//clear fifo ??		
		ast_lpc_write(ast_lpc_read(AST_LPC_PCCR0) | LPC_RX_FIFO_CLR, AST_LPC_PCCR0);
		//clear 
		ast_lpc_write(lpc_sts & ~LPC_LRST, AST_LPC_HICR2);
		
	}

	if(snoop_dma_sts & LPC_POST_CODE_DMA_RDY) {
		
		
	}


	return IRQ_HANDLED;

}
EXPORT_SYMBOL(ast_snoop_dma_handler);

extern void ast_snoop_channel_int_enable(struct ast_snoop_channel	*ast_ch, u8 enable)
{
	printk("ch[%d]int : %s , snoop port : %x",ast_ch->snoop_ch, enable? "Enable":"Disable", ast_ch->snoop_port);

	if(enable) {
		switch(ast_ch->snoop_ch) {
			case 0:
				//enable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) | LPC_HICR5_SNP0INT_EN, 
							AST_LPC_HICR5);				
				break;
			case 1:
				//enable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) | LPC_HICR5_SNP1INT_EN, 
							AST_LPC_HICR5);								
				break;
		};
		
	} else {
		switch(ast_ch->snoop_ch) {
			case 0:
				//disable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) & ~LPC_HICR5_SNP0INT_EN, 
							AST_LPC_HICR5);
		
				break;
			case 1:
				//disable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) & ~LPC_HICR5_SNP1INT_EN, 
							AST_LPC_HICR5);
		};
	
	}

}
EXPORT_SYMBOL(ast_snoop_channel_int_enable);

extern void ast_snoop_channel_enable(struct ast_snoop_channel *ast_ch, u8 enable)
{
	printk("ch[%d] : %s , snoop port : %x",ast_ch->snoop_ch, enable? "Enable":"Disable", ast_ch->snoop_port);

	if(enable) {
		switch(ast_ch->snoop_ch) {
			case 0:
				//disable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) & ~LPC_HICR5_SNP0W_EN, 
							AST_LPC_HICR5);

				//set port address 				
				ast_lpc_write((ast_lpc_read(AST_LPC_SNPWADR) & ~LPC_SNOOP_ADDR0_MASK) | 
							ast_ch->snoop_port, 
							AST_LPC_SNPWADR);
				//enable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) | LPC_HICR5_SNP0W_EN, 
							AST_LPC_HICR5);				
				break;
			case 1:
				//disable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) & ~LPC_HICR5_SNP1W_EN, 
							AST_LPC_HICR5);

				//set port address				
				ast_lpc_write((ast_lpc_read(AST_LPC_SNPWADR) & ~LPC_SNOOP_ADDR1_MASK) | 
							ast_ch->snoop_port, 
							AST_LPC_SNPWADR);
				//enable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) | LPC_HICR5_SNP1W_EN, 
							AST_LPC_HICR5);								
				break;
		};
		
	} else {
		switch(ast_ch->snoop_ch) {
			case 0:
				//disable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) & ~LPC_HICR5_SNP0W_EN, 
							AST_LPC_HICR5);
		
				break;
			case 1:
				//disable
				ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) & ~LPC_HICR5_SNP1W_EN, 
							AST_LPC_HICR5);
		
		};
	
	}

}
EXPORT_SYMBOL(ast_snoop_channel_enable);

extern void ast_snoop_dma_ch_enable(struct ast_snoop_dma_channel *ast_dma_ch, u8 enable)
{
	printk("ch[%d] : %s , snoop port : %x",ast_dma_ch->snoop_ch, enable? "Enable":"Disable", ast_dma_ch->snoop_port);

	if(enable) {
		//disable
		ast_lpc_write(ast_lpc_read(AST_LPC_PCCR0) & ~LPC_POST_CODE_EN, 
					AST_LPC_PCCR0);

		//set port address 				
		ast_lpc_write((ast_lpc_read(AST_LPC_PCCR0) & ~LPC_POST_ADDR_MASK) |
							LPC_CAPTURE_ADDR_MASK(ast_dma_ch->snoop_mask) | 
							LPC_CAPTURE_BASE_ADDR(ast_dma_ch->snoop_port),
							AST_LPC_PCCR0);

		ast_lpc_write(ast_dma_ch->dma_addr,	AST_LPC_PCCR4);
		ast_lpc_write(ast_dma_ch->dma_size - 1 , AST_LPC_PCCR5);

		//enable
		ast_lpc_write((ast_lpc_read(AST_LPC_PCCR0) & ~LPC_POST_CODE_MODE_MASK) |
					LPC_POST_CODE_MODE(ast_dma_ch->snoop_mode) |
					LPC_POST_DMA_MODE_EN |
					LPC_POST_CODE_EN, 
					AST_LPC_PCCR0);				
	
	} else {
		//disable
		ast_lpc_write(ast_lpc_read(AST_LPC_PCCR0) & ~LPC_POST_CODE_EN, 
					AST_LPC_PCCR0);
	}

}
EXPORT_SYMBOL(ast_snoop_dma_ch_enable);

extern int ast_snoop_init(struct ast_snoop *snoop)
{
	int ret=0;

	ast_snoop_channel_enable(snoop->snoop_ch0, 1);
	ast_snoop_channel_enable(snoop->snoop_ch1, 1);
	//request irq
	ret = request_irq(IRQ_LPC, ast_snoop_handler, IRQF_SHARED,
			  "ast-snoop", snoop);
	
	//enable irq
	ast_lpc_write(ast_lpc_read(AST_LPC_HICR5) | LPC_HICR5_SNP0INT_EN | LPC_HICR5_SNP1INT_EN, 
				AST_LPC_HICR5); 			
	return ret;
}
EXPORT_SYMBOL(ast_snoop_init);

extern void ast_snoop_dma_init(struct ast_snoop_dma_channel *ast_dma_ch)
{
	int ret=0;

	ast_snoop_dma_ch_enable(ast_dma_ch, 1);

	//request irq
	ret = request_irq(IRQ_LPC, ast_snoop_dma_handler, IRQF_SHARED,
			  "ast-snoop", ast_dma_ch);
	
	//enable irq
	ast_lpc_write(ast_lpc_read(AST_LPC_PCCR0) |
				LPC_POST_DMA_INT_EN,
				AST_LPC_PCCR0); 

	return ret;

}
EXPORT_SYMBOL(ast_snoop_dma_init);

static int __init ast_lpc_init_lpc(void)
{
	LPCMSG("AST LPC INIT \n");
	ast_lpc_base = ioremap(AST_LPC_BASE, SZ_256);	
	return 0;
}

arch_initcall(ast_lpc_init_lpc);


