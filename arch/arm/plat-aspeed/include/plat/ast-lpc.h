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
struct ast_lpc_bus_info {
	u8 lpc_bus_mode; /* 1: host mode , 0: dev mode*/	
	u8 scan_node;	
	u8 lpc_mode; /* 0: lpc , 1: lpc+ */
	u8 snoop_enable;
	u8 ipmi_kcs_enable;	
	u8 ipmi_bt_enable;		
	u32 bridge_phy_addr;
};

/************************  LPC Snoop **************************************/
struct ast_lpc_snoop_dma_channel {
	u8	snoop_ch;	
	u8	snoop_port;
	u8	snoop_mask;
	u8	snoop_mode;
	u8	snoop_index;
	u32	dma_virt;	
	dma_addr_t dma_addr;	
	u16 dma_size;
};

#define SNOOP_FIFO_SIZE	16

struct ast_snoop_data {
	u8 read_idx;
	u8 write_idx;
	u8 fifo[SNOOP_FIFO_SIZE];
};

/************************  LPC KCS ****************************************/
#define IPMI_MAX_MSG_LENGTH	(272	)

typedef enum kcs_states {
	KCS_PHASE_IDLE = 0,
	KCS_PHASE_WRITE,
	KCS_PHASE_WRITE_END,
	KCS_PHASE_READ,
	KCS_PHASE_ERROR1,
	KCS_PHASE_ERROR2,
	KCS_PHASE_ERROR3,
	/* The hardware failed to follow the state machine. */
	KCS_PHASE_HOSED
} kcs_phase;

struct ast_kcs_data {
	//-> phase state 
	enum kcs_states		KCSPhase;	
	// --->>>>
	wait_queue_head_t read_q, write_q;       /* read and write queues */	
	//RX 
	u32	KCSRcvPktIx;
	u8	*pKCSRcvPkt;
	//Tx 
	u8	KCSSendPktIx;
	u8	KCSSendPktLen;
	u8	*pKCSSendPkt;
	//register offset -- > 
	u32	str;
	u32	idr;
	u32	odr;
};

#define BT_DEBUG_OFF	0	/* Used in production */
#define BT_DEBUG_ENABLE	1	/* Generic messages */
#define BT_DEBUG_MSG	2	/* Prints all request/response buffers */
#define BT_DEBUG_STATES	4	/* Verbose look at state changes */

/* Results of SMI events. */
enum si_sm_result {
	SI_SM_CALL_WITHOUT_DELAY, /* Call the driver again immediately */
	SI_SM_CALL_WITH_DELAY,	/* Delay some before calling again. */
	SI_SM_CALL_WITH_TICK_DELAY,/* Delay >=1 tick before calling again. */
	SI_SM_TRANSACTION_COMPLETE, /* A transaction is finished. */
	SI_SM_IDLE,		/* The SM is in idle state. */
	SI_SM_HOSED,		/* The hardware violated the state machine. */

	/*
	 * The hardware is asserting attn and the state machine is
	 * idle.
	 */
	SI_SM_ATTN,
	SI_SM_READ_COMPLETE
};

/*
 * Typical "Get BT Capabilities" values are 2-3 retries, 5-10 seconds,
 * and 64 byte buffers.  However, one HP implementation wants 255 bytes of
 * buffer (with a documented message of 160 bytes) so go for the max.
 * Since the Open IPMI architecture is single-message oriented at this
 * stage, the queue depth of BT is of no concern.
 */

#define BT_NORMAL_TIMEOUT	5	/* seconds */
#define BT_NORMAL_RETRY_LIMIT	2
#define BT_RESET_DELAY		6	/* seconds after warm reset */


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
	u32	str;
	u32	fifo;	
	enum bt_states	state;
	unsigned char	seq;		/* BT sequence number */
	unsigned char	write_data[IPMI_MAX_MSG_LENGTH];
	int		write_count;
	unsigned char	read_data[IPMI_MAX_MSG_LENGTH];
	int		read_count;
	int		truncated;
	long		timeout;	/* microseconds countdown */
	int		error_retries;	/* end of "common" fields */
	int		nonzero_status;	/* hung BMCs stay all 0 */
	enum bt_states	complete;	/* to divert the state machine */
	int		BT_CAP_outreqs;
	long		BT_CAP_req2rsp;
	int		BT_CAP_retries;	/* Recommended retries */
	unsigned char	completion_code[IPMI_MAX_MSG_LENGTH];
	unsigned int	completion_code_length;
};

struct ast_lpc_driver_data {
	struct platform_device *pdev;
	void __iomem		*reg_base;			/* virtual */	
	int 				irq;					//LPC IRQ number 
	u32					bus_id;			//for LPC dev# 
	struct ast_lpc_bus_info *bus_info;
	//Slave mode
	struct ast_snoop_data *snoop_data;
	void				*ast_lpc_snoop_dma_virt;
	dma_addr_t		ast_lpc_snoop_dma;
	
	struct ast_kcs_data *kcs_data;
	struct ast_bt_data *bt_data;
};

/************************  LPC KCS **************************************/

struct kcs_parameter {
	u8	ch_enable;
	u8	ch_type;	
};

/* Different Phases of the KCS Module */
#define KCS_PHASE_IDLE			0x00
#define KCS_PHASE_WRITE			0x01
#define KCS_PHASE_WRITE_END		0x02
#define KCS_PHASE_READ			0x03
#define KCS_PHASE_ABORT			0x04
#define KCS_PHASE_ERROR			0x05

////////////////////////////////////////////
/* KCS Command Control codes. */
#define KCS_GET_STATUS_ABORT	0x60
#define KCS_WRITE_START			0x61
#define KCS_WRITE_END			0x62
#define KCS_READ_BYTE			0x68

/* Status bits. */
#define GET_STATUS_STATE(status) (((status) >> 6) & 0x03)
#define KCS_IDLE_STATE	0
#define KCS_READ_STATE	1
#define KCS_WRITE_STATE	2
#define KCS_ERROR_STATE	3
#define GET_STATUS_ATN(status) ((status) & 0x04)
#define GET_STATUS_IBF(status) ((status) & 0x02)
#define GET_STATUS_OBF(status) ((status) & 0x01)

/* KCS Error Codes */
#define KCS_NO_ERROR			0x00
#define KCS_ABORTED_BY_COMMAND		0x01
#define KCS_ILLEGAL_CONTROL_CODE	0x02
#define KCS_LENGTH_ERROR		0x06
#define KCS_UNSPECIFIED_ERROR		0xff

////////////////////////////////////////////////////////////////

#define MAX_KCS_PKT_LEN		(272)

