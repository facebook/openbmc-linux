/*
 *  ast-uart-sdma.c
 *
 *  UART SDMA for the AST UART access.
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2013.12.26: Initial version [Ryan Chen]
 */

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/ast-uart-dma.h>
#include <plat/regs-uart-sdma.h>

//#define AST_UART_SDMA_DEBUG

#ifdef AST_UART_SDMA_DEBUG
#define SDMADUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SDMADUG(fmt, args...)
#endif

static inline void
ast_uart_sdma_write(struct ast_sdma *sdma, u32 val, u32 reg)
{
//	printk("uart dma write : val: %x , reg : %x \n",val,reg);	
	writel(val, sdma->reg_base+ reg);
}

static inline u32
ast_uart_sdma_read(struct ast_sdma *sdma, u32 reg)
{
#if 0
	u32 val = readl(sdma->reg_base + reg);
	printk("R : reg %x , val: %x \n",reg, val);
	return val;
#else	
	return readl(sdma->reg_base + reg);
#endif
}

struct ast_sdma ast_uart_sdma;

/* *****************************************************************************/
int ast_uart_rx_sdma_enqueue(u8 ch, dma_addr_t rx_buff)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;

	SDMADUG("ch = %d, rx buff = %x, len = %d \n",ch, rx_buff);

	local_irq_save(flags);
	ast_uart_sdma_write(sdma, rx_buff, UART_RX_SDMA_ADDR(ch));
	local_irq_restore(flags);

	return 0;
}

EXPORT_SYMBOL(ast_uart_rx_sdma_enqueue);

int ast_uart_tx_sdma_enqueue(u8 ch, dma_addr_t tx_buff)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;

	SDMADUG("ch = %d, tx buff = %x \n",ch, tx_buff);

	local_irq_save(flags);
	ast_uart_sdma_write(sdma, tx_buff, UART_TX_SDMA_ADDR(ch));
	local_irq_restore(flags);

	return 0;
}

EXPORT_SYMBOL(ast_uart_tx_sdma_enqueue);

int ast_uart_rx_sdma_ctrl(u8 ch, enum ast_uart_chan_op op)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;
	struct ast_sdma_info *dma_ch = &(sdma->dma_ch->rx_dma_info[ch]);
	SDMADUG("RX DMA CTRL [ch %d] \n", ch);

	local_irq_save(flags);

	switch (op) {
		case AST_UART_DMAOP_TRIGGER:
			SDMADUG("Trigger \n");
			dma_ch->enable = 1;
#ifdef SDDMA_RX_FIX
#else
			ast_uart_set_sdma_time_out(0xffff);	
#endif
			//set enable 
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_RX_SDMA_EN) | (0x1 << ch), UART_RX_SDMA_EN);
			break;
		case AST_UART_DMAOP_STOP:
			//disable engine 
			SDMADUG("STOP \n");			
			dma_ch->enable = 0;			
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_RX_SDMA_EN) & ~(0x1 << ch), UART_RX_SDMA_EN);
			//set reset 
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_RX_SDMA_REST) | (0x1 << ch), UART_RX_SDMA_REST);
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_RX_SDMA_REST) & ~(0x1 << ch), UART_RX_SDMA_REST);			

			ast_uart_sdma_write(sdma, 0, UART_RX_R_POINT(ch));	
			//Addddd 
			ast_uart_sdma_write(sdma, dma_ch->dma_phy_addr, UART_RX_SDMA_ADDR(ch));
			break;
		case AST_UART_DMAOP_PAUSE:
			//disable engine 
			dma_ch->enable = 0;
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_RX_SDMA_EN) & ~(0x1 << ch), UART_RX_SDMA_EN);
			break;
	}

	local_irq_restore(flags);
	return 0;	
}
EXPORT_SYMBOL(ast_uart_rx_sdma_ctrl);

int ast_uart_tx_sdma_ctrl(u8 ch, enum ast_uart_chan_op op)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;
	struct ast_sdma_info *dma_ch = &(sdma->dma_ch->tx_dma_info[ch]);	
	SDMADUG("TX DMA CTRL [ch %d] \n", ch);

	local_irq_save(flags);

	switch (op) {
		case AST_UART_DMAOP_TRIGGER:
			SDMADUG("TRIGGER : Enable \n");	
			dma_ch->enable = 1;
			//set enable 
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_TX_SDMA_EN) | (0x1 << ch), UART_TX_SDMA_EN);
			break;
		case AST_UART_DMAOP_STOP:
			SDMADUG("STOP : DISABLE & RESET\n");	
			dma_ch->enable = 0;
			//disable engine 
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_TX_SDMA_EN) & ~(0x1 << ch), UART_TX_SDMA_EN);			
			//set reset 
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_TX_SDMA_REST) | (0x1 << ch), UART_TX_SDMA_REST);
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_TX_SDMA_REST) & ~(0x1 << ch), UART_TX_SDMA_REST);

			ast_uart_sdma_write(sdma, 0, UART_TX_W_POINT(ch));	
			break;
		case AST_UART_DMAOP_PAUSE:
			SDMADUG("PAUSE : DISABLE \n");	
			dma_ch->enable = 0;
			//disable engine 
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_TX_SDMA_EN) & ~(0x1 << ch), UART_TX_SDMA_EN);						
	}

	local_irq_restore(flags);
	return 0;	
}
EXPORT_SYMBOL(ast_uart_tx_sdma_ctrl);

u32 ast_uart_get_tx_sdma_pt(u8 ch)
{
	struct ast_sdma *sdma = &ast_uart_sdma;
	return ast_uart_sdma_read(sdma, UART_TX_R_POINT(ch));
}

EXPORT_SYMBOL(ast_uart_get_tx_sdma_pt);

int ast_uart_tx_sdma_update(u8 ch, u16 point)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;
//	struct ast_sdma_info *dma_ch = &(sdma->dma_ch->tx_dma_info[ch]);	
	SDMADUG("TX DMA CTRL [ch %d] point %d \n", ch, point);

	local_irq_save(flags);
	ast_uart_sdma_write(sdma, point, UART_TX_W_POINT(ch));
	local_irq_restore(flags);
	return 0;	
}

EXPORT_SYMBOL(ast_uart_tx_sdma_update);

int ast_uart_tx_sdma_request(u8 ch, ast_uart_dma_cbfn_t rtn, void *id)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;
	struct ast_sdma_info *dma_ch = &(sdma->dma_ch->tx_dma_info[ch]);

	SDMADUG("TX DMA REQUEST ch = %d \n",ch);

	local_irq_save(flags);

	if (dma_ch->enable) {
		local_irq_restore(flags);
		return -EBUSY;
	}
	dma_ch->priv = id;	
	dma_ch->callback_fn = rtn;

	//DMA IRQ En
	ast_uart_sdma_write(sdma, 
				ast_uart_sdma_read(sdma, UART_TX_SDMA_IER) |
				(1 << ch)
				, UART_TX_SDMA_IER);

	local_irq_restore(flags);

	return 0;

}

EXPORT_SYMBOL(ast_uart_tx_sdma_request);

int ast_uart_rx_sdma_update(u8 ch, u16 point)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;
	SDMADUG("RX DMA CTRL [ch %d] point %x \n", ch, point);

	local_irq_save(flags);
	ast_uart_sdma_write(sdma, point, UART_RX_R_POINT(ch)) ;
	local_irq_restore(flags);
	return 0;	
}

EXPORT_SYMBOL(ast_uart_rx_sdma_update);

#ifdef SDDMA_RX_FIX
char *ast_uart_rx_sdma_request(u8 ch, ast_uart_dma_cbfn_t rtn, void *id)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;
	struct ast_sdma_info *dma_ch = &(sdma->dma_ch->rx_dma_info[ch]);

	SDMADUG("RX DMA REQUEST ch = %d \n",ch);

	local_irq_save(flags);

	if (dma_ch->enable) {
		local_irq_restore(flags);
		return 0;
	}
	dma_ch->priv = id;	

	dma_ch->callback_fn = rtn;

	//DMA IRQ En
	ast_uart_sdma_write(sdma, 
				ast_uart_sdma_read(sdma, UART_RX_SDMA_IER) |
				(1 << ch)
				, UART_RX_SDMA_IER);

	local_irq_restore(flags);

	return dma_ch->sdma_virt_addr;

}

#else
char *ast_uart_rx_sdma_request(u8 ch, void *id)
{
	unsigned long flags;
	struct ast_sdma *sdma = &ast_uart_sdma;
	struct ast_sdma_info *dma_ch = &(sdma->dma_ch->rx_dma_info[ch]);

	SDMADUG("RX DMA REQUEST ch = %d \n",ch);

	local_irq_save(flags);

	if (dma_ch->enable) {
		local_irq_restore(flags);
		return -EBUSY;
	}
	dma_ch->priv = id;	

	local_irq_restore(flags);

	return dma_ch->sdma_virt_addr;

}
#endif

EXPORT_SYMBOL(ast_uart_rx_sdma_request);

u16 ast_uart_get_rx_sdma_pt(u8 ch)
{
	struct ast_sdma *sdma = &ast_uart_sdma;
	return ast_uart_sdma_read(sdma, UART_RX_W_POINT(ch)) ;
}

EXPORT_SYMBOL(ast_uart_get_rx_sdma_pt);

void ast_uart_set_sdma_time_out(u16 val)
{
	struct ast_sdma *sdma = &ast_uart_sdma;
	ast_uart_sdma_write(sdma, val, UART_SDMA_TIMER);
}

EXPORT_SYMBOL(ast_uart_set_sdma_time_out);

/* *****************************************************************************/
static inline void ast_sdma_bufffdone(struct ast_sdma_info *sdma_ch)
{
	u32 len;
	struct ast_sdma *sdma = &ast_uart_sdma;

	if(sdma_ch->enable == 0) {
		printk("sdma Please check ch_no %x %s!!!!!\n",sdma_ch->ch_no, sdma_ch->direction ? "TX": "RX");
		if(sdma_ch->direction) 	
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_TX_SDMA_EN) & ~(0x1 << sdma_ch->ch_no), UART_TX_SDMA_EN);
		else {
			ast_uart_sdma_write(sdma, ast_uart_sdma_read(sdma, UART_RX_SDMA_EN) & ~(0x1 << sdma_ch->ch_no), UART_RX_SDMA_EN);
			ast_uart_rx_sdma_update(sdma_ch->ch_no, ast_uart_get_rx_sdma_pt(sdma_ch->ch_no));
			printk("OFFSET : UART_RX_SDMA_EN = %x  \n ", ast_uart_sdma_read(sdma, UART_RX_SDMA_EN));	
		}
		return;
	}

	if(sdma_ch->direction) {
		len = ast_uart_sdma_read(sdma, UART_TX_R_POINT(sdma_ch->ch_no)) ;
		SDMADUG("tx rp %x , wp %x \n", ast_uart_sdma_read(sdma, UART_TX_R_POINT(sdma_ch->ch_no)), ast_uart_sdma_read(sdma, UART_TX_W_POINT(sdma_ch->ch_no)));
		
	} else {
		SDMADUG("rx rp %x , wp %x \n", ast_uart_sdma_read(sdma, UART_RX_R_POINT(sdma_ch->ch_no)), ast_uart_sdma_read(sdma, UART_RX_W_POINT(sdma_ch->ch_no)));	
		len = ast_uart_sdma_read(sdma, UART_RX_W_POINT(sdma_ch->ch_no)) ;
	}
	
	SDMADUG("<dma dwn>: ch[%d] : %s ,len : %d \n", 
			sdma_ch->ch_no, sdma_ch->direction ? "tx" : "rx", len);

	if (sdma_ch->callback_fn != NULL)
		(sdma_ch->callback_fn)(sdma_ch->priv, len);
}


static irqreturn_t 
ast_uart_sdma_irq(int irq, void *dev_id)
{
	struct ast_sdma	*sdma = (struct ast_sdma *)dev_id;

	u32 tx_sts = ast_uart_sdma_read(sdma, UART_TX_SDMA_ISR);
	u32 rx_sts = ast_uart_sdma_read(sdma, UART_RX_SDMA_ISR);

	
	SDMADUG("tx sts : %x, rx sts : %x \n",tx_sts, rx_sts);

	if((tx_sts == 0) && (rx_sts == 0)) {
		printk("SDMA IRQ ERROR !!!\n");
		return IRQ_HANDLED;		
	}

	if(rx_sts & UART_SDMA0_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA0_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[0]));
	} else if (rx_sts & UART_SDMA1_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA1_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[1]));	
	} else if (rx_sts & UART_SDMA2_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA2_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[2]));	
	} else if (rx_sts & UART_SDMA3_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA3_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[3]));	
	} else if (rx_sts & UART_SDMA4_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA4_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[4]));
	} else if (rx_sts & UART_SDMA5_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA5_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[5]));
	} else if (rx_sts & UART_SDMA6_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA6_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[6]));	
	} else if (rx_sts & UART_SDMA7_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA7_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[7]));
	} else if (rx_sts & UART_SDMA8_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA8_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[8]));	
	} else if (rx_sts & UART_SDMA9_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA9_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[9]));	
	} else if (rx_sts & UART_SDMA10_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA10_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[10]));	
	} else if (rx_sts & UART_SDMA11_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA11_INT, UART_RX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->rx_dma_info[11]));	
	} else {
	}

	if(tx_sts & UART_SDMA0_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA0_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[0]));
	} else if (tx_sts & UART_SDMA1_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA1_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[1]));
	} else if (tx_sts & UART_SDMA2_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA2_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[2]));
	} else if (tx_sts & UART_SDMA3_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA3_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[3]));	
	} else if (tx_sts & UART_SDMA4_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA4_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[4]));	
	} else if (tx_sts & UART_SDMA5_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA5_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[5]));	
	} else if (tx_sts & UART_SDMA6_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA6_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[6]));	
	} else if (tx_sts & UART_SDMA7_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA7_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[7]));	
	} else if (tx_sts & UART_SDMA8_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA8_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[8]));	
	} else if (tx_sts & UART_SDMA9_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA9_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[9]));	
	} else if (tx_sts & UART_SDMA10_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA10_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[10]));	
	} else if (tx_sts & UART_SDMA11_INT) {
		ast_uart_sdma_write(sdma, UART_SDMA11_INT, UART_TX_SDMA_ISR);
		ast_sdma_bufffdone(&(sdma->dma_ch->tx_dma_info[11]));	
	} else {
	}

	return IRQ_HANDLED;
}

struct ast_sdma sdma;

extern int
ast_uart_sdma_init(void)
{
    	int i, ret;
	struct ast_sdma *sdma = &ast_uart_sdma;	
//	struct ast_sdma_ch *sdma_ch;
	char	*rx_dma_virt_addr;
	dma_addr_t	rx_dma_phy_addr;

	sdma->dma_ch = kzalloc(sizeof(struct ast_sdma_ch), GFP_KERNEL);

	if (!sdma->dma_ch) {
		printk(KERN_ERR "%s: failed to ioremap()\n", __func__);
		return -ENOMEM;
	}

	sdma->reg_base = ioremap(AST_UART_SDMA_BASE, 0x100);
	
	if (!sdma->reg_base) {
		printk(KERN_ERR "%s: failed to ioremap()\n", __func__);
		return -ENXIO;
	}

	rx_dma_virt_addr = (unsigned char *)dma_alloc_coherent(NULL, 
								SDMA_RX_BUFF_SIZE * AST_UART_SDMA_CH, &rx_dma_phy_addr, GFP_KERNEL);
	
	for(i=0; i<AST_UART_SDMA_CH; i++) {
		//TX ------------------------
		sdma->dma_ch->tx_dma_info[i].enable = 0;
		sdma->dma_ch->tx_dma_info[i].ch_no = i;
		sdma->dma_ch->tx_dma_info[i].direction = 1;
		ast_uart_sdma_write(sdma, 0, UART_TX_W_POINT(i));
		//RX ------------------------
		sdma->dma_ch->rx_dma_info[i].enable = 0;
		sdma->dma_ch->rx_dma_info[i].ch_no = i;	
		sdma->dma_ch->rx_dma_info[i].direction = 0;
		sdma->dma_ch->rx_dma_info[i].sdma_virt_addr = rx_dma_virt_addr + (SDMA_RX_BUFF_SIZE * i);
		sdma->dma_ch->rx_dma_info[i].dma_phy_addr = rx_dma_phy_addr + (SDMA_RX_BUFF_SIZE * i);
		ast_uart_sdma_write(sdma, sdma->dma_ch->rx_dma_info[i].dma_phy_addr, UART_RX_SDMA_ADDR(i));		
		ast_uart_sdma_write(sdma, 0, UART_RX_R_POINT(i)) ;			
	}

	ast_uart_sdma_write(sdma, 0xffffffff, UART_TX_SDMA_REST);
	ast_uart_sdma_write(sdma, 0x0, UART_TX_SDMA_REST);		

	ast_uart_sdma_write(sdma, 0xffffffff, UART_RX_SDMA_REST);
	ast_uart_sdma_write(sdma, 0x0, UART_RX_SDMA_REST);		

	ast_uart_sdma_write(sdma, 0, UART_TX_SDMA_EN);
	ast_uart_sdma_write(sdma, 0, UART_RX_SDMA_EN);	

#ifdef SDDMA_RX_FIX
	ast_uart_sdma_write(sdma, 0x200, UART_SDMA_TIMER);
#else
	ast_uart_sdma_write(sdma, 0xffff, UART_SDMA_TIMER);
#endif

	//TX
	ast_uart_sdma_write(sdma, 0xfff, UART_TX_SDMA_ISR);
	ast_uart_sdma_write(sdma, 0, UART_TX_SDMA_IER);

	//RX
	ast_uart_sdma_write(sdma, 0xfff, UART_RX_SDMA_ISR);
	ast_uart_sdma_write(sdma, 0, UART_RX_SDMA_IER);

	ret = request_irq(IRQ_UART_SDMA, 
							ast_uart_sdma_irq, 0, 
							"ast_sdma_uart", sdma);
	if (ret) {
		printk ("Unable to get UART SDMA IRQ !!!!!!!!!!!!!!!!!!!!\n");
		return -1;
	}


	ast_uart_sdma_write(sdma, SDMA_SET_TX_BUFF_SIZE(SDMA_BUFF_SIZE_4KB) | SDMA_SET_RX_BUFF_SIZE(SDMA_BUFF_SIZE_64KB)
					, UART_SDMA_CONF);

    return 0;
}                                                                              

EXPORT_SYMBOL(ast_uart_sdma_init);
