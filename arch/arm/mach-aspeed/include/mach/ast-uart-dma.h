/********************************************************************************
* File Name     : ast-uart-dma.h
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
********************************************************************************/
#ifndef AST_UART_DMA_H_INCLUDED
#define AST_UART_DMA_H_INCLUDED

#define	DMA_BUFF_SIZE				0x1000	//4096
#define	SDMA_RX_BUFF_SIZE				0x10000	//65536

#define 	SDDMA_RX_FIX	1
/* enum ast_uart_chan_op
 *
 * operation codes passed to the DMA code by the user, and also used
 * to inform the current channel owner of any changes to the system state
*/

enum ast_uart_chan_op {
	AST_UART_DMAOP_TRIGGER,
	AST_UART_DMAOP_STOP,
	AST_UART_DMAOP_PAUSE,	
};

/* ast_uart_dma_cbfn_t *  * buffer callback routine type */
typedef void (*ast_uart_dma_cbfn_t)(void *dev_id, u16 len);

#ifdef CONFIG_AST1070_UART_DMA
struct ast_uart_dma_data {
		u8		chip_no;		//campain chip number 
		u8		dma_ch;		//dma channel number 
};

struct uart_dma_desc {
	u32	desc0;
	u32	desc1;
	u32	desc2;	
	u32	desc3;	
} __attribute__ ((aligned(16)));

struct ast1070_dma_info {
	u8	ch_no;
	u8 	direction;	
	u8	enable;
	u32	ctrl_offset;
	u32	desc_offset;	
	void	*priv;
	struct uart_dma_desc 	*desc;
	dma_addr_t	desc_dma_addr;	/* Mapped descr. table */
	/* cdriver callbacks */
	ast_uart_dma_cbfn_t	 callback_fn;	/* buffer done callback */
};

#define AST1070_UART_DMA_CH 		4

struct ast1070_dma_ch {
	struct ast1070_dma_info tx_dma_info[AST1070_UART_DMA_CH];
	struct ast1070_dma_info rx_dma_info[AST1070_UART_DMA_CH];
};

struct ast1070_dma {
	void __iomem		*reg_base;	
	struct ast1070_dma_ch	*dma_ch;
};

/* ast_uart_dma_request  *  * request a dma channel exclusivley */
extern int ast_uart_rx_dma_request(u8 node, u8 channel, ast_uart_dma_cbfn_t rtn, void *id);
extern int ast_uart_tx_dma_request(u8 node, u8 channel, ast_uart_dma_cbfn_t rtn, void *id);

/* ast_uart_dma_ctrl  *  * change the state of the dma channel */
extern int ast_uart_rx_dma_ctrl(u8 node, u8 ch, enum ast_uart_chan_op op);
extern int ast_uart_tx_dma_ctrl(u8 node, u8 ch, enum ast_uart_chan_op op);

extern int ast_uart_rx_dma_enqueue(u8 node, u8 ch, dma_addr_t rx_buff, u16 len);
extern int ast_uart_tx_dma_enqueue(u8 node, u8 ch, dma_addr_t tx_buff, u16 len);

extern int ast1070_uart_dma_init(u8 chip, u32 lpc_base);

#endif

#ifdef CONFIG_AST_UART_SDMA
struct ast_uart_sdma_data {
		u8		dma_ch;		//dma channel number 
#ifdef SDDMA_RX_FIX		
#else
		u8		workaround;	
#endif
};

struct ast_sdma_info {
	u8	ch_no;
	u8 	direction;	
	u8	enable;
	void	*priv;
	char	*sdma_virt_addr;
	dma_addr_t	dma_phy_addr;
	/* cdriver callbacks */
	ast_uart_dma_cbfn_t	 callback_fn;	/* buffer done callback */
};

#define AST_UART_SDMA_CH 		12

struct ast_sdma_ch {
	struct ast_sdma_info tx_dma_info[AST_UART_SDMA_CH];
	struct ast_sdma_info rx_dma_info[AST_UART_SDMA_CH];
};

struct ast_sdma {
	void __iomem		*reg_base;	
	struct ast_sdma_ch	*dma_ch;
};


/* ast_uart_dma_request  *  * request a dma channel exclusivley */
#ifdef SDDMA_RX_FIX
extern char *ast_uart_rx_sdma_request(u8 ch, ast_uart_dma_cbfn_t rtn, void *id);
#else
extern char *ast_uart_rx_sdma_request(u8 ch, void *id);
#endif
extern int ast_uart_tx_sdma_request(u8 ch, ast_uart_dma_cbfn_t rtn, void *id);

/* ast_uart_dma_ctrl  *  * change the state of the dma channel */
extern int ast_uart_rx_sdma_ctrl(u8 ch, enum ast_uart_chan_op op);
extern int ast_uart_tx_sdma_ctrl(u8 ch, enum ast_uart_chan_op op);

extern int ast_uart_rx_sdma_enqueue(u8 ch, dma_addr_t rx_buff);
extern int ast_uart_tx_sdma_enqueue(u8 ch, dma_addr_t tx_buff);
extern int ast_uart_tx_sdma_update(u8 ch, u16 point);
extern int ast_uart_rx_sdma_update(u8 ch, u16 point);
extern u32 ast_uart_get_tx_sdma_pt(u8 ch);
extern u16 ast_uart_get_rx_sdma_pt(u8 ch);
extern void ast_uart_set_sdma_time_out(u16 val);

extern int ast_uart_sdma_init(void);
#endif


#endif

