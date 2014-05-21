#ifndef AST_UART_DMA_H_INCLUDED
#define AST_UART_DMA_H_INCLUDED


#define	DMA_BUFF_SIZE				0x1000	//4096

struct ast_uart_dma_data {
		u8		chip_no;		//campain chip number 
		u8		dma_ch;		//dma channel number 
};


/* enum ast_uart_chan_op
 *
 * operation codes passed to the DMA code by the user, and also used
 * to inform the current channel owner of any changes to the system state
*/

enum ast_uart_chan_op {
	AST_UART_DMAOP_TRIGGER,
	AST_UART_DMAOP_STOP,
};

struct ast1070_dma_ch;

/* ast_uart_dma_cbfn_t *  * buffer callback routine type */
typedef void (*ast_uart_dma_cbfn_t)(struct ast1070_dma_ch *,void *dev_id, u16 len);

struct uart_dma_desc {
	u32	desc0;
	u32	desc1;
	u32	desc2;	
	u32	desc3;	
} __attribute__ ((aligned(16)));

struct ast1070_dma_ch {
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

struct ast1070_dma {
	void __iomem		*reg_base;	
	struct ast1070_dma_ch	dma_tx_ch[AST1070_UART_DMA_CH];
	struct ast1070_dma_ch	dma_rx_ch[AST1070_UART_DMA_CH];	
};


/* ast_uart_dma_request  *  * request a dma channel exclusivley */
extern int ast_uart_rx_dma_request(u8 node, u8 channel, ast_uart_dma_cbfn_t rtn, void *id);
extern int ast_uart_tx_dma_request(u8 node, u8 channel, ast_uart_dma_cbfn_t rtn, void *id);

/* ast_uart_dma_ctrl  *  * change the state of the dma channel */
extern int ast_uart_rx_dma_ctrl(u8 node, u8 ch, enum ast_uart_chan_op op);
extern int ast_uart_tx_dma_ctrl(u8 node, u8 ch, enum ast_uart_chan_op op);

extern int ast_uart_rx_dma_enqueue(u8 node, u8 ch, dma_addr_t rx_buff, u16 len);
extern int ast_uart_tx_dma_enqueue(u8 node, u8 ch, dma_addr_t tx_buff, u16 len);



#endif

