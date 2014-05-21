/*
 *  ast1070-uart-dma.c
 *
 *  UART DMA for the AST1070 UART access.
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2012.05.26: Initial version [Ryan Chen]
 */

#include <linux/sysdev.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/ast-uart-dma.h>
#include <plat/regs-uart-dma.h>

//#define AST_UART_DMA_DEBUG

#ifdef AST_UART_DMA_DEBUG
#define DMADUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define DMADUG(fmt, args...)
#endif

//#define AST1070_FPGA			1

struct ast1070_dma uart_dma[CONFIG_AST1070_NR];

static inline void
ast1070_uart_dma_write(struct ast1070_dma *dma, u32 val, u32 reg)
{
	//printk("uart dma write : val: %x , reg : %x \n",val,reg);	
	writel(val, dma->reg_base+ reg);
}

static inline u32
ast1070_uart_dma_read(struct ast1070_dma *dma, u32 reg)
{
#if 0
	u32 val = readl(i2c_dev->reg_base + reg);
	printk("R : reg %x , val: %x \n",reg, val);
	return val;
#else	
	return readl(dma->reg_base + reg);
#endif
}

/* *****************************************************************************/
int ast_uart_rx_dma_enqueue(u8 node, u8 ch, dma_addr_t rx_buff, u16 len)
{
	unsigned long flags;
	struct ast1070_dma *dma = &uart_dma[node];
	struct ast1070_dma_ch *dma_ch = &(dma->dma_rx_ch[ch]);
	struct uart_dma_desc *rx_desc = dma_ch->desc;

	if(len > 4096)
		printk("ERROR !!!  Please Check ...\n");
	
	local_irq_save(flags);

	//fill to rx desc -->
	rx_desc->desc0 = DESC0_END | DESC0_INT_EN | DESC0_HW_OWN;
	rx_desc->desc1 = DESC1_LEN(len);
	rx_desc->desc2 = rx_buff;
	rx_desc->desc3 = 0;

	DMADUG("[c%d]: ch = %d, rx buff = %x, len = %d \n",node, ch, rx_buff, len);

	//fill in tx descriptor base register
	DMADUG("desc_addr : %x, reg offset %x \n",dma_ch->desc_dma_addr, dma_ch->desc_offset);	
	ast1070_uart_dma_write(dma, dma_ch->desc_dma_addr, dma_ch->desc_offset);

	local_irq_restore(flags);

	return 0;
}

EXPORT_SYMBOL(ast_uart_rx_dma_enqueue);

int ast_uart_tx_dma_enqueue(u8 node, u8 ch, dma_addr_t tx_buff, u16 len)
{
	unsigned long flags;
	struct ast1070_dma *dma = &uart_dma[node];
	struct ast1070_dma_ch *dma_ch = &(dma->dma_tx_ch[ch]);
	struct uart_dma_desc *tx_desc = dma_ch->desc;

	DMADUG("[c%d]: ch = %d, tx buff = %x, len = %d \n",node, ch, tx_buff, len);

	local_irq_save(flags);

	//fill to rx desc -->
	tx_desc->desc0 = DESC0_END | DESC0_INT_EN | DESC0_HW_OWN;
	tx_desc->desc1 = DESC1_LEN(len);
	tx_desc->desc2 = tx_buff;
	tx_desc->desc3 = 0;

//	DMADUG("desc vir = %x, tx desc = %x, %x, %x, %x  ===\n",tx_desc, tx_desc->desc0 ,tx_desc->desc1,tx_desc->desc2,tx_desc->desc3);
	//fill in tx descriptor base register
	DMADUG("desc_addr : %x, in offset %x \n",dma_ch->desc_dma_addr, dma_ch->desc_offset);
	ast1070_uart_dma_write(dma, dma_ch->desc_dma_addr, dma_ch->desc_offset);

	local_irq_restore(flags);

	return 0;
}

EXPORT_SYMBOL(ast_uart_tx_dma_enqueue);

int ast_uart_rx_dma_ctrl(u8 node, u8 ch, enum ast_uart_chan_op op)
{
	unsigned long flags;
	struct ast1070_dma *dma = &uart_dma[node];
	struct ast1070_dma_ch *dma_ch = &(dma->dma_rx_ch[ch]);
	DMADUG("[c%d]: ch = %d \n",node, ch);

	local_irq_save(flags);

	switch (op) {
		case AST_UART_DMAOP_TRIGGER:
			//trigger 
			DMADUG("Trigger \n");
			dma_ch->enable = 1;	
//			ast1070_uart_dma_write(dma, DMA_ENABLE, dma_ch->ctrl_offset);
			ast1070_uart_dma_write(dma, DMA_TRIGGER | DMA_ENABLE, dma_ch->ctrl_offset);
			break;

		case AST_UART_DMAOP_STOP:
			//disable engine 
			DMADUG("Stop \n");			
			dma_ch->enable = 0;
			ast1070_uart_dma_write(dma, 0, dma_ch->ctrl_offset);
			break;
	}


	return 0;	
}
EXPORT_SYMBOL(ast_uart_rx_dma_ctrl);

int ast_uart_tx_dma_ctrl(u8 node, u8 ch, enum ast_uart_chan_op op)
{
	unsigned long flags;
	struct ast1070_dma *dma = &uart_dma[node];
	struct ast1070_dma_ch *dma_ch = &(dma->dma_tx_ch[ch]);
	DMADUG("TX DMA CTRL [c%d]: ch = %d \n",node, ch);

	local_irq_save(flags);

	switch (op) {
		case AST_UART_DMAOP_TRIGGER:
			//trigger 
			DMADUG("Trigger \n");			
			ast1070_uart_dma_write(dma, DMA_ENABLE, dma_ch->ctrl_offset);
			ast1070_uart_dma_write(dma, DMA_TRIGGER | DMA_ENABLE, dma_ch->ctrl_offset);
			break;

		case AST_UART_DMAOP_STOP:
			//disable engine 
			DMADUG("STOP \n");			
			ast1070_uart_dma_write(dma, 0, dma_ch->ctrl_offset);
			break;
	}


	return 0;	
}
EXPORT_SYMBOL(ast_uart_tx_dma_ctrl);

int ast_uart_tx_dma_request(u8 node, u8 ch, ast_uart_dma_cbfn_t rtn, void *id)
{
	unsigned long flags;
	struct ast1070_dma *dma = &uart_dma[node];
	struct ast1070_dma_ch *dma_ch = &(dma->dma_tx_ch[ch]);

	DMADUG("TX DMA REQUEST [c%d]: ch = %d \n",node, ch);

	local_irq_save(flags);

	if (dma_ch->enable) {
		local_irq_restore(flags);
		return -EBUSY;
	}

	dma_ch->priv = id;
	dma_ch->enable = 1;
	dma_ch->callback_fn = rtn;
	//DMA IRQ En
	ast1070_uart_dma_write(dma, 
				ast1070_uart_dma_read(dma, UART_DMA_IER) |
				(1 << ch)
				, UART_DMA_IER);

	//enable engine 
//	ast1070_uart_dma_write(dma, DMA_ENABLE, dma_ch->ctrl_offset);
	local_irq_restore(flags);

	return 0;

}

EXPORT_SYMBOL(ast_uart_tx_dma_request);

int ast_uart_rx_dma_request(u8 node, u8 ch, ast_uart_dma_cbfn_t rtn, void *id)
{
	unsigned long flags;
	struct ast1070_dma *dma = &uart_dma[node];
	struct ast1070_dma_ch *dma_ch = &(dma->dma_rx_ch[ch]);

	DMADUG("RX DMA REQUEST [c%d] : ch = %d \n",node, ch);

	local_irq_save(flags);

	if (dma->dma_rx_ch[ch].enable) {
		local_irq_restore(flags);
		return -EBUSY;
	}
	dma_ch->priv = id;	
//	dma_ch->enable = 1;
	dma_ch->callback_fn = rtn;
//	dma_ch->name
	//DMA IRQ En
	ast1070_uart_dma_write(dma, 
				ast1070_uart_dma_read(dma, UART_DMA_IER) |
				(1 << (4+ch))
				, UART_DMA_IER);

	//enable engine 
//	ast1070_uart_dma_write(dma, DMA_ENABLE, dma_ch->ctrl_offset);
	local_irq_restore(flags);

	return 0;

}

EXPORT_SYMBOL(ast_uart_rx_dma_request);
/* *****************************************************************************/
static inline void ast_dma_bufffdone(struct ast1070_dma_ch	 *dma_ch)
{
	////TODO desc -- remove ......
	//workaround : Issue RX dma can;t be stoped , close open close  
	if(dma_ch->enable == 0) {
//		printk("workaround \n");
		return;
	}
		
//	u32 sts = ast1070_uart_dma_read(dma, dma_ch->ctrl_offset);	
	DMADUG("dma dwn : ch[%d] : %s ,len : %d \n", dma_ch->ch_no, dma_ch->direction ? "tx" : "rx", DESC3_GET_LEN(dma_ch->desc->desc3));

	DMADUG(" == desc = %x, %x, %x, %x  ===\n",dma_ch->desc->desc0,dma_ch->desc->desc1,dma_ch->desc->desc2,dma_ch->desc->desc3);


	if(dma_ch->desc->desc0 & DESC0_HW_OWN) 
		printk("ERROR ..... \n");
	
	if (dma_ch->callback_fn != NULL)
		(dma_ch->callback_fn)(dma_ch, dma_ch->priv, DESC3_GET_LEN(dma_ch->desc->desc3));
}


static irqreturn_t 
ast1070_c0_uart_dma_irq(int irq, void *dev_id)
{
//	struct ast1070_dma	*dma = dev_id;
	int i;
	struct ast1070_dma *dma = &uart_dma[0];	
	u32 sts = ast1070_uart_dma_read(dma, UART_DMA_ISR);
	DMADUG("C0 int -- > \n");
	DMADUG("isr sts = %x\n", sts);

	for(i = 0;i < 17 ; i++)
		DMADUG("offset : %x , val %x \n",i*4, ast1070_uart_dma_read(dma, i*4));
	
	if (sts & UART_DMA3_RX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[3]));
	} else if (sts & UART_DMA2_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA2_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[2]));
	} else if (sts & UART_DMA1_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA1_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[1]));
	} else if (sts & UART_DMA0_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA0_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[0]));
	} else if (sts & UART_DMA3_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[3]));
	} else if (sts & UART_DMA2_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA2_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[2]));
	} else if (sts & UART_DMA1_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA1_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[1]));
	} else if (sts & UART_DMA0_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA0_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[0]));
	} else {
		printk("No body .. !!! \n");
	}

	return IRQ_HANDLED;
}

#if (CONFIG_AST1070_NR >=2)
static irqreturn_t 
ast1070_c1_uart_dma_irq(int irq, void *dev_id)
{
//	struct ast1070_dma	*dma = dev_id;
	struct ast1070_dma *dma = &uart_dma[1];
	u32 sts = ast1070_uart_dma_read(dma, UART_DMA_ISR);
	DMADUG("C1 int -- > \n");

//	DMADUG("isr sts = %x\n", sts);
	if (sts & UART_DMA3_RX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[3]));
	} else if (sts & UART_DMA2_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA2_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[2]));
	} else if (sts & UART_DMA1_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA1_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[1]));
	} else if (sts & UART_DMA0_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA0_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[0]));
	} else if (sts & UART_DMA3_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[3]));
	} else if (sts & UART_DMA2_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA2_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[2]));
	} else if (sts & UART_DMA1_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA1_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[1]));
	} else if (sts & UART_DMA0_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA0_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[0]));
	} else {
		printk("No body .. !!! \n");
	}

	return IRQ_HANDLED;
}
#endif

#if (CONFIG_AST1070_NR >=3)
static irqreturn_t 
ast1070_c2_uart_dma_irq(int irq, void *dev_id)
{
	struct ast1070_dma	*dma = dev_id;
	u32 sts = ast1070_uart_dma_read(dma, UART_DMA_ISR);

//	DMADUG("isr sts = %x\n", sts);
	if (sts & UART_DMA3_RX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[3]));
	} else if (sts & UART_DMA2_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA2_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[2]));
	} else if (sts & UART_DMA1_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA1_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[1]));
	} else if (sts & UART_DMA0_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA0_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[0]));
	} else if (sts & UART_DMA3_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[3]));
	} else if (sts & UART_DMA2_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA2_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[2]));
	} else if (sts & UART_DMA1_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA1_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[1]));
	} else if (sts & UART_DMA0_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA0_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[0]));
	} else {
		printk("No body .. !!! \n");
	}

	return IRQ_HANDLED;
}
#endif

#if (CONFIG_AST1070_NR >=4)
static irqreturn_t 
ast1070_c3_uart_dma_irq(int irq, void *dev_id)
{
	struct ast1070_dma	*dma = dev_id;
	u32 sts = ast1070_uart_dma_read(dma, UART_DMA_ISR);

//	DMADUG("isr sts = %x\n", sts);
	if (sts & UART_DMA3_RX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[3]));
	} else if (sts & UART_DMA2_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA2_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[2]));
	} else if (sts & UART_DMA1_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA1_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[1]));
	} else if (sts & UART_DMA0_RX_INT) {		
		ast1070_uart_dma_write(dma, UART_DMA0_RX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_rx_ch[0]));
	} else if (sts & UART_DMA3_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA3_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[3]));
	} else if (sts & UART_DMA2_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA2_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[2]));
	} else if (sts & UART_DMA1_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA1_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[1]));
	} else if (sts & UART_DMA0_TX_INT) {
		ast1070_uart_dma_write(dma, UART_DMA0_TX_INT, UART_DMA_ISR);
		ast_dma_bufffdone(&(dma->dma_tx_ch[0]));
	} else {
		printk("No body .. !!! \n");
	}

	return IRQ_HANDLED;
}
#endif

extern int
ast1070_uart_dma_init(u8 node)
{
    int ret,i;
	struct ast1070_dma *dma = &uart_dma[node];

	DMADUG("ast1070 uart_dma_init [c%d]\n", node);

	if(node == 0) {	
		dma->reg_base = ioremap(AST_C0_UART_DMA_BASE, 0x100);
#if (CONFIG_AST1070_NR >=2)		
	} else if (node == 1) {
		dma->reg_base = ioremap(AST_C1_UART_DMA_BASE, 0x100);
#endif		
#if (CONFIG_AST1070_NR >=3)		
	} else if (node == 2) {
		dma->reg_base = ioremap(AST_C2_UART_DMA_BASE, 0x100);
#endif		
#if (CONFIG_AST1070_NR >=4)		
	} else if (node == 3) {
		dma->reg_base = ioremap(AST_C3_UART_DMA_BASE, 0x100);
#endif		
	} else {
		printk("node out of range !! \n");
		return 1;
	}
	
	if (!dma->reg_base) {
			printk(KERN_ERR "%s: failed to ioremap()\n", __func__);
			return -ENXIO;
	}

	ast1070_uart_dma_write(dma, 0xff, UART_DMA_ISR);
	ast1070_uart_dma_write(dma, 0, UART_DMA_IER);

	for(i=0;i<4;i++) {
		//TX ------------------------
		dma->dma_tx_ch[i].enable = 0;
		dma->dma_tx_ch[i].ch_no = i;
		dma->dma_tx_ch[i].direction = 1;
		//tx descriptor allocation
		dma->dma_tx_ch[i].desc = dma_alloc_coherent(NULL, sizeof(struct uart_dma_desc), &(dma->dma_tx_ch[i].desc_dma_addr), GFP_KERNEL);
		if (dma->dma_tx_ch[i].desc == NULL) {
			DMADUG("Can't allocate tx descriptor\n");
			return 0;
		}
		memset(dma->dma_tx_ch[i].desc, 0, sizeof(struct uart_dma_desc));
		DMADUG("tx_desc [%d] virt = %x, dma = %x\n", i, (u32)dma->dma_tx_ch[i].desc, dma->dma_tx_ch[i].desc_dma_addr);

		ast1070_uart_dma_write(dma, 0, UART_DMA0_TX_CTRL + (i*8));
		dma->dma_tx_ch[i].ctrl_offset = UART_DMA0_TX_CTRL + (i*8);
		dma->dma_tx_ch[i].desc_offset = UART_DMA0_TX_DESCPT + (i*8);

		//RX ------------------------
		dma->dma_rx_ch[i].enable = 0;
		dma->dma_rx_ch[i].ch_no = i;	
		dma->dma_rx_ch[i].direction = 0;
		//rx descriptor allocation
		dma->dma_rx_ch[i].desc = dma_alloc_coherent(NULL, sizeof(struct uart_dma_desc), &(dma->dma_rx_ch[i].desc_dma_addr), GFP_KERNEL);
		if (dma->dma_rx_ch[i].desc == NULL) {
			DMADUG("Can't allocate tx descriptor\n");
			return 0;
		}
		memset(dma->dma_rx_ch[i].desc, 0, sizeof(struct uart_dma_desc));
		DMADUG("rx_desc [%d] virt = %x, dma = %x\n", i, (u32)dma->dma_rx_ch[i].desc, dma->dma_rx_ch[i].desc_dma_addr);
		ast1070_uart_dma_write(dma, 0, UART_DMA0_RX_CTRL + (i*8));
		dma->dma_rx_ch[i].ctrl_offset = UART_DMA0_RX_CTRL + (i*8);
		dma->dma_rx_ch[i].desc_offset = UART_DMA0_RX_DESCPT + (i*8);
	}

	DMADUG("reg base = %x \n", (u32)dma->reg_base);

	if(node == 0) {
		for(i=0;i<4;i++) {
			ret = request_irq(IRQ_C0_N1_UART_DMA + i, 
									ast1070_c0_uart_dma_irq, IRQF_SHARED, 
									"ast1070_n1_uart_dma", dma);
			if (ret)
				printk ("Unable to get UART DMA IRQ !!!!!!!!!!!!!!!!!!!!\n");
		}
#if (CONFIG_AST1070_NR >=2)
	} else if (node == 1) {
		for(i=0;i<4;i++) {
			ret = request_irq(IRQ_C1_N1_UART_DMA + i, 
									ast1070_c1_uart_dma_irq, IRQF_SHARED, 
									"ast1070_n1_uart_dma", dma);
			if (ret)
				printk ("Unable to get UART DMA IRQ !!!!!!!!!!!!!!!!!!!!\n");
		}
#endif
#if (CONFIG_AST1070_NR >=3)
	} else if (node == 2) {
		for(i=0;i<4;i++) {
			ret = request_irq(IRQ_C2_N1_UART_DMA + i, 
									ast1070_c2_uart_dma_irq, IRQF_SHARED, 
									"ast1070_n1_uart_dma", dma);
			if (ret)
				printk ("Unable to get UART DMA IRQ !!!!!!!!!!!!!!!!!!!!\n");
	}
#endif
#if (CONFIG_AST1070_NR >=4)
	} else if (node == 3) {
		for(i=0;i<4;i++) {
			ret = request_irq(IRQ_C3_N1_UART_DMA + i, 
									ast1070_c3_uart_dma_irq, IRQF_SHARED, 
									"ast1070_n1_uart_dma", dma);
			if (ret)
				printk ("Unable to get UART DMA IRQ !!!!!!!!!!!!!!!!!!!!\n");
		}
#endif	
	} else {
		printk("ERROR !! \n");
	}

	//Limit : AST1070 4* SPI CLK < AST2400 HCLK

#ifdef AST1070_FPGA
	//Low SPI clk setting  == PCLK/8 , set 11
	ast1070_uart_dma_write(dma, 
					(ast1070_uart_dma_read(dma, UART_DMA_CTRL) & ~SPI_CLK_MASK) |
					SPI_CLK_SET(0x3) |
					DMA_RX_TIMEOUT(0xfff) |
					TXDESC_AUTO_POLLING |
					RXDESC_AUTO_POLLING
					, UART_DMA_CTRL);
#else
	ast1070_uart_dma_write(dma, 
					(ast1070_uart_dma_read(dma, UART_DMA_CTRL) & 
					~DMA_BURST_MASK) |
					DMA_RX_TIMEOUT(0xfff) |
					TXDESC_AUTO_POLLING |
					RXDESC_AUTO_POLLING
					, UART_DMA_CTRL);
#endif	

    return 0;
}                                                                              

EXPORT_SYMBOL(ast1070_uart_dma_init);
