/*
 * Platform data for AST LPC .
 *
 * Copyright (C) ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _AST_LPC_H_
#define _AST_LPC_H_

#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>


/************************  SNOOP  ******************************************/
#define AST_SNOOP_NUM		2

/************************  IPMI  ******************************************/
#define AST_IPMI_PKT_SIZE		(512)
typedef void (*ast_ipmi_irq)(void *data);

/************************  IPMI BT ****************************************/
#define BT_NORMAL_TIMEOUT		5	/* seconds */
#define BT_NORMAL_RETRY_LIMIT	2
#define BT_RESET_DELAY			6	/* seconds after warm reset */

#define AST_BT_NUM		2

enum bt_states {
	BT_STATE_IDLE = 0,	/* Order is critical in this list */
	BT_STATE_XACTION_START,
	BT_STATE_WRITE_BYTES,
	BT_STATE_WRITE_CONSUME,
	BT_STATE_READ_WAIT,
	BT_STATE_CLEAR_B2H,
	BT_STATE_CLEAR_H2B,
	BT_STATE_READ_BYTES,
	BT_STATE_RESET1,	/* These must come last */
	BT_STATE_RESET2,
	BT_STATE_RESET3,
	BT_STATE_RESTART,
	BT_STATE_PRINTME,
	BT_STATE_CAPABILITIES_BEGIN,
	BT_STATE_CAPABILITIES_END,
	BT_STATE_LONG_BUSY	/* BT doesn't get hosed :-) */
};

struct ast_bt_data {
	//driver information
	struct platform_device *pdev;
	int 			open_count;
	struct miscdevice	miscdev;
	u8				bt_reg;				/* register info */
	struct ast_lpc_data	*ast_lpc;

	//
	void __iomem			*isr;
	void __iomem			*str;
	void __iomem			*fifo;
	void __iomem			*fifo_sts;
	struct tasklet_struct	bt_tasklet;
	enum bt_states	state;
	unsigned char	seq;		/* BT sequence number */
	unsigned char	write_data[AST_IPMI_PKT_SIZE];
	int		write_count;
	unsigned char	read_data[AST_IPMI_PKT_SIZE];
	int		read_count;
	int		truncated;
	long		timeout;	/* microseconds countdown */
	int		error_retries;	/* end of "common" fields */
	u8 		BTPktRdy;
	enum bt_states	complete;	/* to divert the state machine */
	int		BT_CAP_outreqs;
	long		BT_CAP_req2rsp;
	int		BT_CAP_retries;	/* Recommended retries */
	unsigned char	completion_code[AST_IPMI_PKT_SIZE];
	unsigned int	completion_code_length;
	u8	regspacings;
	/* cdriver callbacks */
	ast_ipmi_irq	 bt_irq_hander;	/* buffer done callback */
};



/************************  IPMI KCS ****************************************/
#define AST_KCS_NUM	4

#if defined(AST_SOC_G5) || defined(AST_SOC_G4)
#define AST_IPMI_KCS0_REGSPACINGS			4	//any
#define AST_IPMI_KCS1_REGSPACINGS			4	//any
#define AST_IPMI_KCS2_REGSPACINGS			1	//fix
#define AST_IPMI_KCS3_REGSPACINGS			1	//any
#elif defined(AST_SOC_G3)
#define AST_IPMI_KCS0_REGSPACINGS			4	//fix
#define AST_IPMI_KCS1_REGSPACINGS			4	//fix
#define AST_IPMI_KCS2_REGSPACINGS			1	//fix
#define AST_IPMI_KCS3_REGSPACINGS			1	//any
#else
#error "Not define SoC generation"
#endif

enum kcs_states {
	KCS_PHASE_IDLE = 0,
	KCS_PHASE_WRITE,
	KCS_PHASE_WRITE_END,
	KCS_PHASE_READ,
	KCS_PHASE_ERROR,
	KCS_PHASE_ERROR1,
	KCS_PHASE_ERROR2,
	/* The hardware failed to follow the state machine. */
	KCS_PHASE_HOSED
};

struct ast_kcs_data {
	//driver information
	struct platform_device *pdev;
	int 			open_count;
	struct miscdevice	miscdev;
	u8				kcs_reg;				/* register info */
	struct ast_lpc_data	*ast_lpc;

	//-> phase state
	enum kcs_states		KCSPhase;
	//RX
	u32	KCSRcvPktIx;
	u8	*pKCSRcvPkt;
	u8 	KCSPktRdy;
	//Tx
	u8 	KCSSendWait;
	u16	KCSSendPktIx;
	u16	KCSSendPktLen;
	u8	*pKCSSendPkt;
	//register offset -- >
	void __iomem			*str;
	void __iomem			*idr;
	void __iomem			*odr;
	u8	regspacings;
	/* cdriver callbacks */
	ast_ipmi_irq	 kcs_irq_hander;	/* buffer done callback */
};

/************************  LPC Snoop dma **************************************/
#define SNOOP_DMA_SIZE 	0x1000	//4K

struct ast_snoop_dma_data {
	//driver information
	struct platform_device *pdev;
	struct miscdevice	miscdev;
	u8				snoop_dma_reg;				/* register info */
	struct ast_lpc_data	*ast_lpc;
	ast_ipmi_irq	 snoop_irq_hander;	/* buffer done callback */

	void __iomem			*pccr0;
	void __iomem			*pccr1;
	void __iomem			*pccr2;
	void __iomem			*pccr3;
	void __iomem			*pccr4;
	void __iomem			*pccr5;
	void __iomem			*pccr6;

	u8	port0_data[SNOOP_DMA_SIZE];
	u16	port0_rd_idx;
	u16	port0_wr_idx;
	u8	port1_data[SNOOP_DMA_SIZE];
	u16	port1_rd_idx;
	u16	port1_wr_idx;
	u8	port2_data[SNOOP_DMA_SIZE];
	u16	port2_rd_idx;
	u16	port2_wr_idx;
	u8	port3_data[SNOOP_DMA_SIZE];
	u16	port3_rd_idx;
	u16	port3_wr_idx;


	u8	snoop_port;
	u8	snoop_mode;
	u16	snoop_index;	//sw idx
	u8	*dma_virt;
	dma_addr_t dma_addr;
	u16 dma_size;
	struct tasklet_struct	snoop_tasklet;
        spinlock_t snoop_lock;
};

/************************  LPC Snoop **************************************/

#define SNOOP_FIFO_SIZE	16

struct ast_snoop_data {
	//driver information
	struct platform_device *pdev;
	struct miscdevice	miscdev;
	u8				snoop_reg;				/* register info */
	struct ast_lpc_data	*ast_lpc;
	ast_ipmi_irq	 snoop_irq_hander;	/* buffer done callback */

	u8 read_idx;
	u8 write_idx;
	u8 fifo[SNOOP_FIFO_SIZE];
};

/************************  AST LPC  ****************************************/
struct ast_lpc_data {
	struct platform_device *pdev;
	void __iomem			*reg_base;			/* virtual */
	int 					irq;					//LPC IRQ number
	//Slave mode
	struct ast_snoop_data 			*ast_snoop;
	struct ast_snoop_dma_data 	*ast_snoop_dma;
	struct ast_kcs_data 			*ast_kcs;
	struct ast_bt_data 			*ast_bt;
};

/************************  LPC API ***********************************************/

/* common function */
extern u32 StrToHex(char *p);

/******************************** IPMI KCS API ************************************/
extern struct ast_kcs_data *register_ipmi_kcs_drv(u8 kcs_no);
extern void request_ipmi_kcs_irq(u8 kcs_no, ast_ipmi_irq handler);

extern u16 ast_get_ipmi_kcs_addr(struct ast_lpc_data *ast_lpc, u8 kcs_ch);
extern void ast_set_ipmi_kcs_addr(struct ast_lpc_data *ast_lpc, u8 kcs_ch, u16 kcs_addr);
extern u8 ast_get_ipmi_kcs_en(struct ast_lpc_data *ast_lpc, u8 kcs_ch);
extern void ast_set_ipmi_kcs_en(struct ast_lpc_data *ast_lpc, u8 kcs_ch, u8 enable);

/******************************** IPMI BT API ************************************/
extern struct ast_bt_data *register_ipmi_bt_drv(u8 bt_no);
extern void request_ipmi_bt_irq(u8 bt_no, ast_ipmi_irq handler);
extern u16 ast_get_ipmi_bt_irq(struct ast_lpc_data *ast_lpc, u8 bt_ch);
extern void ast_set_ipmi_bt_irq(struct ast_lpc_data *ast_lpc, u8 bt_ch, u16 bt_addr);
extern u16 ast_get_ipmi_bt_addr(struct ast_lpc_data *ast_lpc, u8 bt_ch);
extern void ast_set_ipmi_bt_addr(struct ast_lpc_data *ast_lpc, u8 bt_ch, u16 bt_addr);
extern u8 ast_get_ipmi_bt_en(struct ast_lpc_data *ast_lpc, u8 bt_ch);
extern void ast_set_ipmi_bt_en(struct ast_lpc_data *ast_lpc, u8 bt_ch, u8 enable);

/******************************** SNOOP API ************************************/

extern void ast_set_snoop_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u8 enable);
extern u8 ast_get_snoop_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch);
extern void ast_set_snoop_port(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u16 snoop_port);
extern u16 ast_get_snoop_port(struct ast_lpc_data *ast_lpc, u8 snoop_ch);
extern u8 ast_get_snoop_data(struct ast_lpc_data *ast_lpc, u8 snoop_ch);
extern void ast_set_lpc2gpio_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u8 enable);
extern u16 ast_get_lpc2gpio_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch);
extern void ast_set_lpc2gpio(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u16 sel80hgio);
extern u16 ast_get_lpc2gpio(struct ast_lpc_data *ast_lpc, u8 snoop_ch);
extern void request_snoop_irq(u8 snoop_no, ast_ipmi_irq handler);
extern struct ast_snoop_data *register_snoop_drv(u8 snoop_no);

/******************************** SNOOP DMA API ************************************/

extern void request_snoop_dma_irq(ast_ipmi_irq handler);
extern struct ast_snoop_dma_data *register_snoop_dma_drv(void);

#endif
