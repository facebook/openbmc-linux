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
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <linux/dma-mapping.h>

#include <plat/regs-lpc.h>
#include <plat/ast-lpc.h>
#ifdef CONFIG_ARCH_AST1070
#include <plat/ast-uart-dma.h>
#include <plat/ast1070-scu.h>
#include <plat/ast1070-devs.h>
#include <plat/regs-ast1070-intc.h>
#endif
#include <linux/miscdevice.h>
#include <linux/hwmon-sysfs.h>


//#include <linux/ipmi.h>

//#define AST_LPC_DEBUG

#ifdef AST_LPC_DEBUG
#define LPC_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define LPC_DBUG(fmt, args...)
#endif

static inline u32 
ast_lpc_read(struct ast_lpc_driver_data *ast_lpc, u32 reg)
{
#if 0
	u32 val;
	val = readl(ast_lpc->reg_base + reg);
	LPC_DBUG("ast_lpc_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_lpc->reg_base + reg);
#endif
}

static inline void
ast_lpc_write(struct ast_lpc_driver_data *ast_lpc, u32 val, u32 reg) 
{
//	LPC_DBUG("ast_lpc_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_lpc->reg_base + reg);
}

static u32 StrToHex(char *p)
{
	int i, sum;
	int temp, length;
	char c;
	sum = 0;
	length = strlen(p);
	for( i = 0; i < (length - 1) ; i++ )
	{
		c = *p;
		if( c >= 'a' && c <= 'z') {
			temp = c - 87;
			sum += ((temp) << (4*(length - i - 2)));
		} else if( c >= 'A' && c <= 'Z') {
			temp = c - 55;
			sum += ((temp) << (4*(length - i - 2)));
		} else {
			temp = c - 48;
			sum = sum + ((temp) << (4*(length - i - 2)));
		}
		p = p + 1;
	}
	return sum;
}

/***************** BT Handle ******************************************************/
static int bt_debug; /* 0 == BT_DEBUG_OFF */

static void 
write_all_bytes(struct ast_lpc_driver_data *ast_lpc, struct ast_bt_data *bt)
{
	int i;

	if (bt_debug & BT_DEBUG_MSG) {
		printk(KERN_WARNING "BT: write %d bytes seq=0x%02X",
			bt->write_count, bt->seq);
		for (i = 0; i < bt->write_count; i++)
			printk(" %02x", bt->write_data[i]);
		printk("\n");
	}
	for (i = 0; i < bt->write_count; i++)
		ast_lpc_write(ast_lpc, bt->write_data[i], bt->fifo);
}

static int 
read_all_bytes(struct ast_lpc_driver_data *ast_lpc, struct ast_bt_data *bt)
{
	unsigned char i;

	/*
	 * length is "framing info", minimum = 3: NetFn, Seq, Cmd
	 * Keep layout of first four bytes aligned with write_data[]
	 */

	bt->read_data[0] = ast_lpc_read(ast_lpc, bt->fifo);
	bt->read_count = bt->read_data[0];
	printk ("in read_all_bytes bt->read_count = %x\n", bt->read_count);

	if (bt->read_count < 3 || bt->read_count >= IPMI_MAX_MSG_LENGTH) {
		if (bt_debug & BT_DEBUG_MSG)
			printk(KERN_WARNING "BT: bad raw rsp len=%d\n",
				bt->read_count);
		bt->truncated = 1;
		return 1;	/* let next XACTION START clean it up */
	}
	for (i = 1; i <= bt->read_count; i++)
		bt->read_data[i] = ast_lpc_read(ast_lpc, bt->fifo);
	bt->read_count++;	/* Account internally for length byte */

	printk ("bt->read_data = ");
	for (i = 0; i < bt->read_count; i++) {
		printk ("%x ", bt->read_data[i]);
	}
	if (bt_debug & BT_DEBUG_MSG) {
		int max = bt->read_count;

		printk(KERN_WARNING "BT: got %d bytes seq=0x%02X",
			max, bt->read_data[2]);
		if (max > 16)
			max = 16;
		for (i = 0; i < max; i++)
			printk(KERN_CONT " %02x", bt->read_data[i]);
		printk(KERN_CONT "%s\n", bt->read_count == max ? "" : " ...");
	}

	return 0;
}

#define BT_STATE_CHANGE(X, Y) { bt->state = X; return Y; }
#define BT_SI_SM_RETURN(Y)   { last_printed = BT_STATE_PRINTME; return Y; }
/* Check status and (usually) take action and change this state machine. */
static enum si_sm_result ibt_event(struct ast_lpc_driver_data *ast_lpc, struct ast_bt_data *bt)
{
	unsigned char status;
	static enum bt_states last_printed = BT_STATE_PRINTME;
	int i;

	status = ast_lpc_read(ast_lpc, bt->str);
	bt->nonzero_status |= status;

	if ((bt->state < BT_STATE_WRITE_BYTES) && (status & BT_B2H_ATN)) {
//		drain_BMC2HOST(bt);
//		BT_SI_SM_RETURN(SI_SM_CALL_WITH_DELAY);
//yriver
		printk ("Buffer was not empty !!\n");
	}
	
	switch (bt->state) {
	/* 
	 * Idle state first checks for asynchronous messages from another
	 * channel, then does some opportunistic housekeeping.
	 */

	case BT_STATE_IDLE:
		if (status & BT_B_BUSY)		/* clear a leftover B_BUSY */
			ast_lpc_write(ast_lpc, BT_B_BUSY, bt->str);

		BT_STATE_CHANGE(BT_STATE_READ_WAIT,
				SI_SM_CALL_WITHOUT_DELAY);
		
	case BT_STATE_XACTION_START:
		if (status & (BT_H_BUSY | BT_B2H_ATN))
			BT_SI_SM_RETURN(SI_SM_CALL_WITH_DELAY);
		if (ast_lpc_read(ast_lpc, bt->str) & BT_B_BUSY)
			ast_lpc_write(ast_lpc, BT_B_BUSY, bt->str);	/* force clear */
		BT_STATE_CHANGE(BT_STATE_WRITE_BYTES,
				SI_SM_CALL_WITHOUT_DELAY);

	case BT_STATE_WRITE_BYTES:
		if (status & BT_B_BUSY)
			ast_lpc_write(ast_lpc, BT_B_BUSY, bt->str);	/* clear */
//yriver
//		BT_CONTROL(BT_CLR_WR_PTR);
		write_all_bytes(ast_lpc, bt);
		ast_lpc_write(ast_lpc, BT_B2H_ATN, bt->str);

		bt->state = bt->complete;
		return bt->state == BT_STATE_IDLE ?	/* where to next? */
			SI_SM_TRANSACTION_COMPLETE :	/* normal */
			SI_SM_CALL_WITHOUT_DELAY;	/* Startup magic */

	case BT_STATE_READ_WAIT:
		if (!(status & BT_H2B_ATN))
			BT_SI_SM_RETURN(SI_SM_CALL_WITH_DELAY);
		ast_lpc_write(ast_lpc, BT_B_BUSY, bt->str);		/* set */
		ast_lpc_write(ast_lpc, BT_B2H_ATN, bt->str);		/* clear it to ACK the host */
		BT_STATE_CHANGE(BT_STATE_CLEAR_H2B,
				SI_SM_CALL_WITHOUT_DELAY);

	case BT_STATE_CLEAR_H2B:
		if (status & BT_H2B_ATN) {
			/* keep hitting it */
			ast_lpc_write(ast_lpc, BT_B2H_ATN, bt->str);
			BT_SI_SM_RETURN(SI_SM_CALL_WITH_DELAY);
		}
		BT_STATE_CHANGE(BT_STATE_READ_BYTES,
				SI_SM_CALL_WITHOUT_DELAY);

	case BT_STATE_READ_BYTES:
		if (!(status & BT_B_BUSY))
			/* check in case of retry */
			ast_lpc_write(ast_lpc, BT_B_BUSY, bt->str);	
//yriver
//		BT_CONTROL(BT_CLR_RD_PTR);	/* start of BMC2HOST buffer */
		i = read_all_bytes(ast_lpc, bt);		/* true == packet seq match */
		ast_lpc_write(ast_lpc, BT_B_BUSY, bt->str);			/* NOW clear */
		BT_STATE_CHANGE(BT_STATE_IDLE,
				SI_SM_READ_COMPLETE);

	default:
		return SI_SM_CALL_WITH_DELAY;
	}
	return SI_SM_CALL_WITH_DELAY;
}

static void ast_ipmi_bt_handle(struct ast_lpc_driver_data *ast_lpc, struct ast_bt_data *bt_data) 
{
	u8 result;
	unsigned int count = 0;	
	
	do {
		result = ibt_event(ast_lpc, bt_data);
		count++;
	} while (((result != SI_SM_READ_COMPLETE) && (count <= 100000)));
	if (count >= 100000) printk ("SI_SM_READ_NOT_COMPLETE\n");

	count = 0;
	printk ("driver finished read\n");
//	flag = 1;
//	wake_up_interruptible (&my_queue);

	return;
}

static u16 
ast_get_ipmi_bt_addr(struct ast_lpc_driver_data *ast_lpc, u8 bt_ch)
{
	u16 tmp = 0;
	switch(bt_ch) {
		case 0:	//0xca2, 0xca3 is kcs , ca4,ca5,ca6 is bt
			tmp = ((ast_lpc_read(ast_lpc, AST_LPC_LADR3H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR3L)) + 2;
			break;
		case 1:	//ibt
			tmp = LPC_iBT_GET_ADDR(ast_lpc_read(ast_lpc, AST_LPC_IBTCR0));
			break;
		default:
			break;
	}
	return tmp;
}

static void 
ast_set_ipmi_bt_addr(struct ast_lpc_driver_data *ast_lpc, u8 bt_ch, u16 bt_addr)
{
	LPC_DBUG("set ch %d, addr %x \n", bt_ch, bt_addr);
	switch(bt_ch) {
		case 0:	//0xca2, 0xca3 is kcs , ca4,ca5,ca6 is bt
			ast_lpc_write(ast_lpc, (bt_addr - 2) >> 8,AST_LPC_LADR3H);
			ast_lpc_write(ast_lpc, (bt_addr - 2) & 0xff, AST_LPC_LADR3L);
			break;
		case 1:	//ibt [31:16] ex  == 0x00e4 --> BTCR : 0xE4 , BTDTR : 0xE4 + 1, BTIMSR : 0xE4 + 2
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) & ~LPC_iBT_ADDR_MASK) | LPC_iBT_SET_ADDR(bt_addr), AST_LPC_IBTCR0);
			break;
		default:
			break;
	}
}

static u8 
ast_get_ipmi_bt_en(struct ast_lpc_driver_data *ast_lpc, u8 bt_ch)
{
	u8 tmp = 0;
	switch(bt_ch) {
		case 0:
			if((ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC3_EN) && (ast_lpc_read(ast_lpc, AST_LPC_HICR4) & LPC_HICS_BTENBL))
				tmp = 1;
			else
				tmp = 0;
			break;
		case 1:	//ibt
			if(ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) & LPC_iBT_ENABLE)
				tmp = 1;
			else
				tmp = 0;
			break;
			
		default:
			printk("Error Ch no !!\n");
			break;
	}
		
	return tmp;

}

static void 
ast_set_ipmi_bt_en(struct ast_lpc_driver_data *ast_lpc, u8 bt_ch, u8 enable)
{
	if(enable) {
		switch(bt_ch) {
			case 0: //bt		--> fix 0xca4 / 0xca5 / 0xca6
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC3_EN, AST_LPC_HICR0);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_BTENBL, AST_LPC_HICR4);
				//TODO .....
				break;
			case 1:	//ibt 
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR1) | LPC_iBT_H2B_RISING_ISR, AST_LPC_IBTCR1);	//Enable BT H2B interrupt Interrupt
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) | LPC_iBT_ENABLE | LPC_iBT_ClrSvRdP_EN | LPC_iBT_ClrSvWrP_EN, AST_LPC_IBTCR0);	//Enable BT Interface
				break;
		}
	} else {
		switch(bt_ch) {
			case 0:	//bt 
				//TODO
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_BTENBL, AST_LPC_HICR4);
				break;
			case 1:	//ibt
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR1) & ~LPC_iBT_H2B_RISING_ISR, AST_LPC_IBTCR1);	//Enable BT H2B interrupt Interrupt
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) & ~(LPC_iBT_ENABLE | LPC_iBT_ClrSvRdP_EN | LPC_iBT_ClrSvWrP_EN), AST_LPC_IBTCR0);	//Enable BT Interface
				break;
		}	

	}	
}

/**************************   LPC  BT SYSFS  **********************************************************/
static ssize_t 
ast_store_ipmi_bt(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

//	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	input_val = StrToHex(sysfsbuf);
	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable
			ast_set_ipmi_bt_en(ast_lpc, sensor_attr->index, input_val);
			break;
		case 1: //addr 
			ast_set_ipmi_bt_addr(ast_lpc, sensor_attr->index, input_val);
			break;
		default:
			return -EINVAL;
			break;
	}

	return count;
}

static ssize_t 
ast_show_ipmi_bt(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{

	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //channel enable, disable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_ipmi_bt_en(ast_lpc, sensor_attr->index),ast_get_ipmi_bt_en(ast_lpc,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 1: //addr
			return sprintf(sysfsbuf, "%x \n", ast_get_ipmi_bt_addr(ast_lpc, sensor_attr->index));
			break;			
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
}

#define sysfs_ipmi_bt_ch(index) \
static SENSOR_DEVICE_ATTR_2(ipmi_bt##index##_en, S_IRUGO | S_IWUSR, \
	ast_show_ipmi_bt, ast_store_ipmi_bt, 0, index); \
static SENSOR_DEVICE_ATTR_2(ipmi_bt##index##_addr, S_IRUGO | S_IWUSR, \
	ast_show_ipmi_bt, ast_store_ipmi_bt, 1, index); \
\
static struct attribute *ipmi_bt##index##_attributes[] = { \
	&sensor_dev_attr_ipmi_bt##index##_en.dev_attr.attr, \
	&sensor_dev_attr_ipmi_bt##index##_addr.dev_attr.attr, \
	NULL \
};

sysfs_ipmi_bt_ch(0);
sysfs_ipmi_bt_ch(1);

static const struct attribute_group ipmi_bt_attribute_groups[] = {
	{ .attrs = ipmi_bt0_attributes },
	{ .attrs = ipmi_bt1_attributes },	
};

/***************** KCS Handle ******************************************************/
static void ast_ipmi_kcs_tx(struct ast_lpc_driver_data *ast_lpc, struct ast_kcs_data *kcs_data)
{
	LPC_DBUG("\n");
	/* Send the first byte */
	kcs_data->KCSSendPktIx = 0;
	ast_lpc_write(ast_lpc, kcs_data->pKCSSendPkt[0], kcs_data->odr);
	kcs_data->KCSSendPktIx++;
	//SA Set OBF Byte
}

static void ast_ipmi_kcs_rx(struct ast_lpc_driver_data *ast_lpc, struct ast_kcs_data *kcs_data)
{
	u8 b;
	int i;
	switch (kcs_data->KCSPhase) {
	case KCS_PHASE_WRITE:
		LPC_DBUG("KCS_PHASE_WRITE \n");
		/* Set the state to write state */
		ast_lpc_write(ast_lpc, ((ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_WRITE_STATE << 6)), kcs_data->str);
		/* Read the BYTE from the data register */
		kcs_data->pKCSRcvPkt[kcs_data->KCSRcvPktIx] = ast_lpc_read(ast_lpc, kcs_data->idr);
		LPC_DBUG("rx data = [%x] \n", kcs_data->pKCSRcvPkt[kcs_data->KCSRcvPktIx]);
		kcs_data->KCSRcvPktIx++;
		if(kcs_data->KCSRcvPktIx > MAX_KCS_PKT_LEN)
			printk("ERROR ---> TODO ... \n");
		break;
		
	case KCS_PHASE_WRITE_END :
		LPC_DBUG("KCS_PHASE_WRITE_END \n");
		/* Set the state to READ_STATE */
		ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_READ_STATE << 6), kcs_data->str);
		/* Read the BYTE from the data register */
		kcs_data->pKCSRcvPkt[kcs_data->KCSRcvPktIx] = ast_lpc_read(ast_lpc, kcs_data->idr);
		kcs_data->KCSRcvPktIx++;

		//SA lets print all data received from SMS
		LPC_DBUG("rx data = [%x] \n", kcs_data->pKCSRcvPkt[kcs_data->KCSRcvPktIx]);
		
		/* Move to READ Phase */
		kcs_data->KCSPhase = KCS_PHASE_READ;
		/* Signal receive data ready */
#if 1		
		printk("Total Rx Data : [");
		for(i=0; i <kcs_data->KCSRcvPktIx; i++)
			printk("%x ", kcs_data->pKCSRcvPkt[i]);
		printk("] \n");
#endif		

#if 1
		//TODO ...... send message to ipmi handle....
		/* Send the first byte */
		kcs_data->KCSSendPktIx = 0;
		printk("send %x \n",kcs_data->pKCSSendPkt[kcs_data->KCSSendPktIx]);
		ast_lpc_write(ast_lpc, kcs_data->pKCSSendPkt[0], kcs_data->odr);
		kcs_data->KCSSendPktIx++;
#else
		ast_ipmi_kcs_message_rcv(ast_lpc, kcs_data);
#endif
		break;
	case KCS_PHASE_READ:
		LPC_DBUG("KCS_PHASE_READ \n");

		/* If we have reached the end of the packet move to idle state */
		if (kcs_data->KCSSendPktIx == kcs_data->KCSSendPktLen)
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_IDLE_STATE << 6), kcs_data->str);

		/* Read the byte returned by the SMS */
		b = ast_lpc_read(ast_lpc, kcs_data->idr);
		//SA Need to clear IBF
		//sa_0111 CLEAR_IBF_STATUS(ChannelNum);

		if (b != KCS_READ_BYTE)
		{
			LPC_DBUG("KCS_PHASE_READ : Set Error State\n");
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_ERROR_STATE << 6), kcs_data->str);
			ast_lpc_write(ast_lpc, 0, kcs_data->odr);
			//SA Set OBF Byte
			break;
		}

		/* If we are finished transmitting, send the dummy byte */
		if (kcs_data->KCSSendPktIx == kcs_data->KCSSendPktLen)
		{
			LPC_DBUG("KCS_PHASE_READ : finished transmitting\n");
			kcs_data->KCSPhase = KCS_PHASE_IDLE;
			ast_lpc_write(ast_lpc, 0, kcs_data->odr);
			//SA Set OBF Byte

			/* Set Transmission Complete */
			break;
		}
		/* Transmit the next byte from the send buffer */
		printk("send idx %d : %x \n",kcs_data->KCSSendPktIx, kcs_data->pKCSSendPkt[kcs_data->KCSSendPktIx]);
		ast_lpc_write(ast_lpc, kcs_data->pKCSSendPkt[kcs_data->KCSSendPktIx], kcs_data->odr);
		kcs_data->KCSSendPktIx++;

		break;
		
	case KCS_PHASE_ERROR1:
		LPC_DBUG("KCS_PHASE_ERROR1 \n");
		/* Set the KCS State to READ_STATE */
		ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_READ_STATE << 6), kcs_data->str);
		/* Read the Dummy byte  */
		ast_lpc_read(ast_lpc, kcs_data->idr);
		/* Write the error code to Data out register */
		printk("TODO ...... \n");
		ast_lpc_write(ast_lpc, KCS_ABORTED_BY_COMMAND, kcs_data->odr);
		
		//SA Set OBF Byte

		/* Set the abort phase to be error2 */
		kcs_data->KCSPhase = KCS_PHASE_ERROR2;
//		pKCSBuf->AbortPhase = ABORT_PHASE_ERROR2;
		break;
	case KCS_PHASE_ERROR2:
		LPC_DBUG("ABORT_PHASE_ERROR2 \n");
		/**  * The system software has read the error code. Go to idle  * state. 	**/
		ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_IDLE_STATE << 6), kcs_data->str);

		/* Read the Dummy byte  */
		ast_lpc_read(ast_lpc, kcs_data->idr);

		kcs_data->KCSPhase = KCS_PHASE_IDLE;
//		pKCSBuf->AbortPhase = 0;
		/* Send the dummy byte  */
		ast_lpc_write(ast_lpc, 0, kcs_data->odr);
		
		break;
	default:
		LPC_DBUG("rx default == > TODO .. \n");		
		/* Read the Dummy byte  */
		ast_lpc_read(ast_lpc, kcs_data->idr);
	}

}

static void ast_ipmi_kcs_cmd_dat(struct ast_lpc_driver_data *ast_lpc, struct ast_kcs_data *kcs_data)
{
	u8 cmd;

	/* Set the status to WRITE_STATE */
	ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_WRITE_STATE << 6), kcs_data->str);
	/* Read the command */
	cmd = ast_lpc_read(ast_lpc, kcs_data->idr);
	LPC_DBUG("cmd = %x \n", cmd);

	switch (cmd) {
		case KCS_WRITE_START:
			LPC_DBUG("KCS_WRITE_START \n");
			/* Set the Index to 0 */
			kcs_data->KCSRcvPktIx = 0;
			/* Set the phase to WRITE */
			kcs_data->KCSPhase = KCS_PHASE_WRITE;
			break;
		case KCS_WRITE_END:
			/* Set the phase to write end */
			LPC_DBUG("KCS_WRITE_END \n");
			kcs_data->KCSPhase = KCS_PHASE_WRITE_END;
			break;
	        case KCS_GET_STATUS_ABORT:
			/* Set the error code */
			LPC_DBUG("KCS_GET_STATUS_ABORT TODO ...\n");
	//			kcs_data->KCSError = KCS_ABORTED_BY_COMMAND;

			/* Set the phase to write end */
			/* Set the abort phase to be error1 */
			kcs_data->KCSPhase = KCS_PHASE_ERROR1;

			/* Send the dummy byte  */
			ast_lpc_write(ast_lpc, 0, kcs_data->odr);
			break;
		default:
			LPC_DBUG("default -- > TODO .... \n");

			/* Set the error code */
	//			kcs_data->KCSError = KCS_ILLEGAL_CONTROL_CODE;
			/* Invalid command code - Set an error state */
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, kcs_data->str) & (~0xC0)) | (KCS_ERROR_STATE << 6), kcs_data->str);
			/* Set the phase to error phase */
			kcs_data->KCSPhase = KCS_PHASE_ERROR;
			break;
	}
}

static void ast_snoop_dma_handler(struct ast_lpc_driver_data *ast_lpc)
{
	u32 snoop_dma_sts, lpc_sts;

	snoop_dma_sts = ast_lpc_read(ast_lpc, AST_LPC_PCCR2);
	lpc_sts = ast_lpc_read(ast_lpc, AST_LPC_HICR2);
	
	printk("ISR : snoop_dma_sts = %x , lpc_sts = %x \n",snoop_dma_sts, lpc_sts);

	if(lpc_sts & LPC_LRST) {
		printk("LPC RST === >TODO .... \n");
		//clear fifo ??		
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_PCCR0) | LPC_RX_FIFO_CLR, AST_LPC_PCCR0);
		//clear 
		ast_lpc_write(ast_lpc, lpc_sts & ~LPC_LRST, AST_LPC_HICR2);
		
	}

	if(snoop_dma_sts & LPC_POST_CODE_DMA_RDY) {
		
		
	}
}

static void ast_ipmi_kcs_handle(struct ast_lpc_driver_data *ast_lpc, struct ast_kcs_data *kcs_data) 
{
	u32 str= ast_lpc_read(ast_lpc, kcs_data->str);
	if (str) {
		str &= 0xb;
		switch(str) {
			case (LPC_STR_CMD_DAT | LPC_STR_IBF):
				LPC_DBUG("LPC_STR_CMD_DAT | LPC_STR_IBF \n");
				ast_ipmi_kcs_cmd_dat(ast_lpc, kcs_data);
				break;

			case LPC_STR_OBF:
				LPC_DBUG("LPC_STR_OBF -- TODO ..\n");
				ast_ipmi_kcs_cmd_dat(ast_lpc, kcs_data);
				break;
			case LPC_STR_IBF:
				LPC_DBUG("LPC_STR_IBF \n");
				ast_ipmi_kcs_rx(ast_lpc, kcs_data);
				break;
			default:
				printk("ERROR 1111 \n");
				break;

		}
				
	}

}

static irqreturn_t ast_lpc_isr (int this_irq, void *dev_id)
{
	struct ast_lpc_driver_data *ast_lpc = dev_id;
	LPC_DBUG("\n");
	
	//SNOOP 
	if(ast_lpc->bus_info->snoop_enable) {
		if(ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & LPC_POST_CODE_EN)
			ast_snoop_dma_handler(ast_lpc);
		
		if(!(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & (LPC_HICR5_SNP0INT_EN | LPC_HICR5_SNP1INT_EN))) {
//			printk("SNOOP not enable \n");
			goto next;
		}

		//sts
		if(!(ast_lpc_read(ast_lpc, AST_LPC_HICR6) & (LPC_HICR6_STR_SNP1W | LPC_HICR6_STR_SNP0W))) {
//			printk("SNOOP sts none \n");			
			goto next;
		}
		
		//ch1	
		if(ast_lpc_read(ast_lpc, AST_LPC_HICR6) & LPC_HICR6_STR_SNP1W) {
			ast_lpc->snoop_data[1].fifo[ast_lpc->snoop_data[1].write_idx] = GET_LPC_SNPD1(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
//			printk("Ch1 data %x \n", GET_LPC_SNPD1(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR)));
			printk("Ch1 data in fifo widx %d : %x \n", ast_lpc->snoop_data[1].write_idx, ast_lpc->snoop_data[1].fifo[ast_lpc->snoop_data[1].write_idx]);
			ast_lpc->snoop_data[1].write_idx++;
			ast_lpc->snoop_data[1].write_idx %= SNOOP_FIFO_SIZE;
			if(ast_lpc->snoop_data[1].write_idx == ast_lpc->snoop_data[1].read_idx) {
				ast_lpc->snoop_data[1].read_idx++;
				ast_lpc->snoop_data[1].read_idx %= SNOOP_FIFO_SIZE;
			}
			ast_lpc_write(ast_lpc, LPC_HICR6_STR_SNP1W,  AST_LPC_HICR6);
		}

		//ch0
		if(ast_lpc_read(ast_lpc, AST_LPC_HICR6) & LPC_HICR6_STR_SNP0W) {
			ast_lpc->snoop_data[0].fifo[ast_lpc->snoop_data[0].write_idx] = GET_LPC_SNPD0(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
//			printk("Ch0 data %x \n", GET_LPC_SNPD0(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR)));
			printk("Ch0 data in fifo widx %d : %x \n", ast_lpc->snoop_data[0].write_idx, ast_lpc->snoop_data[0].fifo[ast_lpc->snoop_data[0].write_idx]);
			ast_lpc->snoop_data[0].write_idx++;
			ast_lpc->snoop_data[0].write_idx %= SNOOP_FIFO_SIZE;
			if(ast_lpc->snoop_data[0].write_idx == ast_lpc->snoop_data[0].read_idx) {
				ast_lpc->snoop_data[0].read_idx++;
				ast_lpc->snoop_data[0].read_idx %= SNOOP_FIFO_SIZE;
			}
			ast_lpc_write(ast_lpc, LPC_HICR6_STR_SNP0W,  AST_LPC_HICR6);
		}

		
	}

next:

	if(ast_lpc->bus_info->ipmi_kcs_enable) {
		//kcs 1: 
		if(ast_lpc_read(ast_lpc, AST_LPC_HICR2) & LPC_IBFIF1) {
			ast_ipmi_kcs_handle(ast_lpc, &ast_lpc->kcs_data[0]);
		}
		//kcs 2
		if(ast_lpc_read(ast_lpc, AST_LPC_HICR2) & LPC_IBFIF2) {
			ast_ipmi_kcs_handle(ast_lpc, &ast_lpc->kcs_data[1]);
		}
		//kcs 3
		if(ast_lpc_read(ast_lpc, AST_LPC_HICR2) & LPC_IBFIF3) {
			ast_ipmi_kcs_handle(ast_lpc, &ast_lpc->kcs_data[2]);
		}
		//kcs 4
		if(ast_lpc_read(ast_lpc, AST_LPC_HICRB) & LPC_KCS4_RCV_INTR) {
			ast_ipmi_kcs_handle(ast_lpc, &ast_lpc->kcs_data[3]);
		}
	}

	if(ast_lpc->bus_info->ipmi_bt_enable) {
		//bt 0
		if(ast_lpc_read(ast_lpc, AST_LPC_HICR4) & LPC_HICS_BTENBL) {
			ast_ipmi_bt_handle(ast_lpc, &ast_lpc->bt_data[0]);
		}
		//ibt 1
		if(ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) & LPC_iBT_ENABLE) {
			if(ast_lpc_read(ast_lpc, AST_LPC_IBTCR2) & LPC_iBT_H2B_RISING_ISR) {
				ast_lpc_write(ast_lpc, LPC_iBT_H2B_RISING_ISR, AST_LPC_IBTCR2);
				ast_ipmi_bt_handle(ast_lpc, &ast_lpc->bt_data[1]);
			}	
			
		}		
	}

	return IRQ_HANDLED;
}		

/**************************   LPC  KCS Function  **********************************************************/
////////////
static u16 
ast_get_ipmi_kcs_addr(struct ast_lpc_driver_data *ast_lpc, u8 kcs_ch)
{
	u16 tmp = 0;
	switch(kcs_ch) {
		case 0:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_LADR12AS, AST_LPC_HICR4);
			tmp = (ast_lpc_read(ast_lpc, AST_LPC_LADR12H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR12L);
			break;
		case 1:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_LADR12AS, AST_LPC_HICR4);
			tmp = (ast_lpc_read(ast_lpc, AST_LPC_LADR12H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR12L);
			break;
		case 2:
			tmp = (ast_lpc_read(ast_lpc, AST_LPC_LADR3H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR3L);
			break;
		case 3:
			break;
		case 4:
			break;
		default:
			break;
	}
	return tmp;
}

static void 
ast_set_ipmi_kcs_addr(struct ast_lpc_driver_data *ast_lpc, u8 kcs_ch, u16 kcs_addr)
{
	LPC_DBUG("set ch %d, addr %x \n", kcs_ch, kcs_addr);
	switch(kcs_ch) {
		case 0:	//0xca0, 0xca4
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_LADR12AS, AST_LPC_HICR4);
			ast_lpc_write(ast_lpc, kcs_addr >> 8, AST_LPC_LADR12H);
			ast_lpc_write(ast_lpc, kcs_addr & 0xff, AST_LPC_LADR12L);
			break;
		case 1:	//0xca2, 0xca6
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_LADR12AS, AST_LPC_HICR4);
			ast_lpc_write(ast_lpc, kcs_addr >> 8, AST_LPC_LADR12H);
			ast_lpc_write(ast_lpc, kcs_addr & 0xff, AST_LPC_LADR12L);
			break;
		case 2:	//0xcb2, 0xcb3
			ast_lpc_write(ast_lpc, kcs_addr >> 8,AST_LPC_LADR3H);
			ast_lpc_write(ast_lpc, kcs_addr & 0xff, AST_LPC_LADR3L);
			break;
		case 3:	//0xcax~+4 +1
			ast_lpc_write(ast_lpc, ((kcs_addr + 1) << 16) | kcs_addr, AST_LPC_LADR4);
			break;
		case 4:	//bt +1
		
			break;
		default:
			break;
	}
}

static u8 
ast_get_ipmi_kcs_en(struct ast_lpc_driver_data *ast_lpc, u8 kcs_ch)
{
	u8 tmp = 0;
	switch(kcs_ch) {
		case 0:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC1_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 1:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC2_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 2:	//kcs 3
			if((ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC3_EN) && (ast_lpc_read(ast_lpc, AST_LPC_HICR4) & LPC_HICS_KCSENBL))
				tmp = 1;
			else
				tmp = 0;
			break;
		case 3:	//kcs4
			if(ast_lpc_read(ast_lpc, AST_LPC_HICRB) & LPC_KCS4_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
		
	return tmp;

}

static void 
ast_set_ipmi_kcs_en(struct ast_lpc_driver_data *ast_lpc, u8 kcs_ch, u8 enable)
{
	if(enable) {
		switch(kcs_ch) {
			case 0: //kcs1
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC1_EN, AST_LPC_HICR0);					
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) | LPC_IBFIF1, AST_LPC_HICR2);	
				break;
			case 1:	//kcs2
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC2_EN, AST_LPC_HICR0);					
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) | LPC_IBFIF2, AST_LPC_HICR2);	
				break;
			case 2: //kcs3	--> fix 0xca2 / 0xca3
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC3_EN, AST_LPC_HICR0);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_KCSENBL, AST_LPC_HICR4);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR3) | LPC_IBFIF1, AST_LPC_HICR2);	
				break;
			case 3: //kcs4	
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) | LPC_KCS4_EN, AST_LPC_HICRB);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) | LPC_KCS4_RCV_INTR, AST_LPC_HICRB);	
				break;
		}
	} else {
		switch(kcs_ch) {
			case 0:	//kcs 1
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) & ~LPC_LPC1_EN, AST_LPC_HICR0);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) & ~LPC_IBFIF1, AST_LPC_HICR2);					
				break;
			case 1:	//kcs 2
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) & ~LPC_LPC2_EN, AST_LPC_HICR0);					
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) & ~LPC_IBFIF2, AST_LPC_HICR2);								
				break;
			case 2:	//kcs 3
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_KCSENBL, AST_LPC_HICR4);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) & ~LPC_IBFIF2, AST_LPC_HICR2);	
				break;
			case 3:	//kcs 4
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) & ~LPC_KCS4_RCV_INTR, AST_LPC_HICRB);	
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) & ~LPC_KCS4_EN, AST_LPC_HICRB);
				break;
		}	

	}	
}

/**************************   LPC  KCS SYSFS  **********************************************************/
static ssize_t 
ast_store_ipmi_kcs(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

//	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	input_val = StrToHex(sysfsbuf);		
	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable
			ast_set_ipmi_kcs_en(ast_lpc, sensor_attr->index, input_val);
			break;
		case 1: //addr 
			ast_set_ipmi_kcs_addr(ast_lpc, sensor_attr->index, input_val);
			break;
		default:
			return -EINVAL;
			break;
	}

	return count;
}

static ssize_t 
ast_show_ipmi_kcs(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //channel enable, disable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_ipmi_kcs_en(ast_lpc, sensor_attr->index),ast_get_ipmi_kcs_en(ast_lpc,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 1: //addr
			return sprintf(sysfsbuf, "%x \n", ast_get_ipmi_kcs_addr(ast_lpc, sensor_attr->index));
			break;			
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
}

#define sysfs_ipmi_kcs_ch(index) \
static SENSOR_DEVICE_ATTR_2(ipmi_kcs##index##_en, S_IRUGO | S_IWUSR, \
	ast_show_ipmi_kcs, ast_store_ipmi_kcs, 0, index); \
static SENSOR_DEVICE_ATTR_2(ipmi_kcs##index##_addr, S_IRUGO | S_IWUSR, \
	ast_show_ipmi_kcs, ast_store_ipmi_kcs, 1, index); \
\
static struct attribute *ipmi_kcs##index##_attributes[] = { \
	&sensor_dev_attr_ipmi_kcs##index##_en.dev_attr.attr, \
	&sensor_dev_attr_ipmi_kcs##index##_addr.dev_attr.attr, \
	NULL \
};

sysfs_ipmi_kcs_ch(0);
sysfs_ipmi_kcs_ch(1);
sysfs_ipmi_kcs_ch(2);
sysfs_ipmi_kcs_ch(3);

static const struct attribute_group ipmi_kcs_attribute_groups[] = {
	{ .attrs = ipmi_kcs0_attributes },
	{ .attrs = ipmi_kcs1_attributes },	
	{ .attrs = ipmi_kcs2_attributes },	
	{ .attrs = ipmi_kcs3_attributes },	
};

/**************************   LPC  Snoop Function  **********************************************************/
static u16 
ast_get_lpc2gpio(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch)
{
	u16 tmp;
	switch(snoop_ch) {
		case 0:
			tmp = LPC_HICR5_GET_SEL80HGIO(ast_lpc_read(ast_lpc, AST_LPC_HICR5));
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
	return tmp;
}

static void 
ast_set_lpc2gpio(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch, u16 sel80hgio)
{
	switch(snoop_ch) {
		case 0:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_HICR5) & (~(0x1f << 24))) | (sel80hgio << 24), 
					AST_LPC_HICR5);
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
}

static u16 
ast_get_lpc2gpio_en(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch)
{
	u16 tmp = 0;
	switch(snoop_ch) {
		case 0:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & (LPC_HICR5_ENSIOGIO | LPC_HICR5_EN80HGIO | LPC_HICR5_ENINVGIO))
				tmp = 1;
			else
				tmp = 0;
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
	return tmp;
}

static void 
ast_set_lpc2gpio_en(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch, u8 enable)
{
	switch(snoop_ch) {
		case 0:
			if(enable)
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_ENSIOGIO | LPC_HICR5_EN80HGIO | LPC_HICR5_ENINVGIO, 
						AST_LPC_HICR5);
			else
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~(LPC_HICR5_ENSIOGIO | LPC_HICR5_EN80HGIO | LPC_HICR5_ENINVGIO), 
						AST_LPC_HICR5);
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
}

static u8 
ast_get_snoop_data(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch)
{
	u8 tmp;
	
	LPC_DBUG("Get CH%d data %d:[%x]",snoop_ch,ast_lpc->snoop_data[snoop_ch].read_idx,ast_lpc->snoop_data[snoop_ch].fifo[ast_lpc->snoop_data[snoop_ch].read_idx]);
//	if(!((ast_lpc->snoop_data[snoop_ch].read_idx + 1) >= ast_lpc->snoop_data[snoop_ch].write_idx)) {
	if(ast_lpc->snoop_data[snoop_ch].write_idx > (ast_lpc->snoop_data[snoop_ch].read_idx)) {
		tmp = ast_lpc->snoop_data[snoop_ch].fifo[ast_lpc->snoop_data[snoop_ch].read_idx];		
		ast_lpc->snoop_data[snoop_ch].read_idx ++;
		ast_lpc->snoop_data[snoop_ch].read_idx %= SNOOP_FIFO_SIZE;
	} else {
		if(ast_lpc->snoop_data[snoop_ch].read_idx == 0)
			tmp = ast_lpc->snoop_data[snoop_ch].fifo[SNOOP_FIFO_SIZE - 1];	
		else
			tmp = ast_lpc->snoop_data[snoop_ch].fifo[ast_lpc->snoop_data[snoop_ch].read_idx -1];	
	}
	//tmp = GET_LPC_SNPD0(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
	//tmp = GET_LPC_SNPD1(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
	return tmp;
}

static u16 
ast_get_snoop_port(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch)
{
	u16 tmp;
	switch(snoop_ch) {
		case 0:
			tmp = LPC_GET_SNOOP_ADDR0(ast_lpc_read(ast_lpc, AST_LPC_SNPWADR));
			break;
		case 1:
			tmp = LPC_GET_SNOOP_ADDR1(ast_lpc_read(ast_lpc, AST_LPC_SNPWADR));
			break;
		case 2:
			tmp = LPC_GET_CAPTURE_ADDR(ast_lpc_read(ast_lpc, AST_LPC_PCCR1));
		default:
			printk("Error Ch no !!\n");
			break;
	}
	return tmp;
}

static void 
ast_set_snoop_port(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch, u16 snoop_port)
{
	switch(snoop_ch) {
		case 0:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_SNPWADR) & ~LPC_SNOOP_ADDR0_MASK) | snoop_port, 
					AST_LPC_SNPWADR);
			break;

		case 1:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_SNPWADR) & ~LPC_SNOOP_ADDR1_MASK) | (snoop_port << 16), 
					AST_LPC_SNPWADR);
			break;
		case 2:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_PCCR1) & ~LPC_POST_ADDR_MASK) |
								LPC_CAPTURE_BASE_ADDR(snoop_port),
								AST_LPC_PCCR1);
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
}

static void 
ast_set_snoop_en(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch, u8 enable)
{
	switch(snoop_ch) {
		case 0:
			if(enable)
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_SNP0W_EN, AST_LPC_HICR5);								
			else
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_SNP0W_EN, AST_LPC_HICR5);
			break;

		case 1:
			if(enable)
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_SNP1W_EN, AST_LPC_HICR5);								
			else
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_SNP1W_EN, AST_LPC_HICR5);
			break;
		case 2:
			if(enable)
				ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & ~LPC_POST_CODE_MODE_MASK) |
							LPC_POST_CODE_MODE(BYTE_MODE) | LPC_POST_DMA_MODE_EN |LPC_POST_DMA_INT_EN |LPC_POST_CODE_EN,
							AST_LPC_PCCR0);	
			else 
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & ~(LPC_POST_DMA_MODE_EN | LPC_POST_DMA_INT_EN | LPC_POST_CODE_EN), AST_LPC_PCCR0);

		default:
			printk("Error Ch no !!\n");
			break;
	}

}
static u8 
ast_get_snoop_en(struct ast_lpc_driver_data *ast_lpc, u8 snoop_ch)
{
	u8 tmp = 0;
	switch(snoop_ch) {
		case 0:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_SNP0W_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 1:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_SNP1W_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 2:
			if(ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & LPC_POST_CODE_EN)
				tmp = 1;
			else 
				tmp = 0;
		default:
			printk("Error Ch no !!\n");
			break;
	}
		
	return tmp;

}
/**************************   LPC  Snoop Function  End **********************************************************/

/**************************   LPC  Snoop Sys fs   **********************************************************/
static ssize_t 
ast_store_snoop(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//input_val = simple_strtoul(sysfsbuf, NULL, 10);
	input_val = StrToHex(sysfsbuf);
	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable
			ast_set_snoop_en(ast_lpc, sensor_attr->index, input_val);
			break;
		case 1: //port
			ast_set_snoop_port(ast_lpc, sensor_attr->index, input_val);
			break;
		case 2: 
			break;			
		case 3: 
			ast_set_lpc2gpio_en(ast_lpc, sensor_attr->index, input_val);
			break;
		case 4: 
			ast_set_lpc2gpio(ast_lpc, sensor_attr->index, input_val);
			break;

		default:
			return -EINVAL;
			break;
	}

	return count;
}

static ssize_t 
ast_show_snoop(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{

	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //channel enable, disable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_snoop_en(ast_lpc, sensor_attr->index),ast_get_snoop_en(ast_lpc,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 1: //port
			return sprintf(sysfsbuf, "%x\n",ast_get_snoop_port(ast_lpc, sensor_attr->index));
			break;			
		case 2: //data
			return sprintf(sysfsbuf, "%x\n",ast_get_snoop_data(ast_lpc, sensor_attr->index));
			break;			
		case 3: //LPC port to GPIO Enable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_lpc2gpio_en(ast_lpc, sensor_attr->index),ast_get_lpc2gpio_en(ast_lpc,sensor_attr->index) ? "Enable":"Disable");
			break;			
		case 4: //LPC port to GPIO Port source 
			return sprintf(sysfsbuf, "%x\n",ast_get_lpc2gpio(ast_lpc, sensor_attr->index));
			break;			
		default:
			return -EINVAL;
			break;
	}
}

/* attr SNOOP sysfs 0~max snoop channel 
*	 0 - show 80h data
*/

#define sysfs_snoop_ch(index) \
static SENSOR_DEVICE_ATTR_2(snoop##index##_en, S_IRUGO | S_IWUSR, \
	ast_show_snoop, ast_store_snoop, 0, index); \
static SENSOR_DEVICE_ATTR_2(snoop##index##_port, S_IRUGO | S_IWUSR, \
	ast_show_snoop, ast_store_snoop, 1, index); \
static SENSOR_DEVICE_ATTR_2(snoop##index##_data, S_IRUGO | S_IWUSR, \
	ast_show_snoop, NULL, 2, index); \
static SENSOR_DEVICE_ATTR_2(snoop##index##_p2gpio_en, S_IRUGO | S_IWUSR, \
	ast_show_snoop, ast_store_snoop, 3, index); \
static SENSOR_DEVICE_ATTR_2(snoop##index##_p2gpio, S_IRUGO | S_IWUSR, \
	ast_show_snoop, ast_store_snoop, 4, index); \	
\
static struct attribute *snoop##index##_attributes[] = { \
	&sensor_dev_attr_snoop##index##_en.dev_attr.attr, \
	&sensor_dev_attr_snoop##index##_port.dev_attr.attr, \
	&sensor_dev_attr_snoop##index##_data.dev_attr.attr, \
	&sensor_dev_attr_snoop##index##_p2gpio_en.dev_attr.attr, \
	&sensor_dev_attr_snoop##index##_p2gpio.dev_attr.attr, \	
	NULL \
};

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 snoop are supported)
 */
sysfs_snoop_ch(0);
sysfs_snoop_ch(1);


static const struct attribute_group snoop_attribute_groups[] = {
	{ .attrs = snoop0_attributes },
	{ .attrs = snoop1_attributes },	
};

/************************************************** SYS FS **************************************************************/
static ssize_t show_snoop_dma_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", ast_lpc_read(ast_lpc, AST_LPC_HICR7) );
}

static DEVICE_ATTR(snoop_dma_data, S_IRUGO | S_IWUSR, show_snoop_dma_data, NULL); 

static ssize_t show_snoop_dma_port_mask(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", (ast_lpc_read(ast_lpc, AST_LPC_PCCR1) & 0xff0000) >> 16);
}

static ssize_t store_snoop_dma_port_mask(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	val = StrToHex(buf);		

	ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_PCCR1) & 0xff00ffff) | val, AST_LPC_PCCR1);
	return count;
}

static DEVICE_ATTR(snoop_dma_port_mask, S_IRUGO | S_IWUSR, show_snoop_dma_port_mask, store_snoop_dma_port_mask); 

static ssize_t show_snoop_dma_port(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", ast_lpc_read(ast_lpc, AST_LPC_PCCR1) & 0xffff);
}

static ssize_t store_snoop_dma_port(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	val = StrToHex(buf);		

	ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_PCCR1) & 0xffff0000) | val, AST_LPC_PCCR1);
	return count;
}

static DEVICE_ATTR(snoop_dma_port, S_IRUGO | S_IWUSR, show_snoop_dma_port, store_snoop_dma_port); 

static ssize_t show_snoop_dma_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	if(ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & LPC_POST_CODE_EN)
		return sprintf(buf, "1: Enable\n");
	else
		return sprintf(buf, "0: Disable\n");	
}

static ssize_t store_snoop_dma_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);
	//use byte mode snoop 
	if(val)
		ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & ~LPC_POST_CODE_MODE_MASK) |
					LPC_POST_CODE_MODE(BYTE_MODE) | LPC_POST_DMA_MODE_EN |LPC_POST_DMA_INT_EN |LPC_POST_CODE_EN,
					AST_LPC_PCCR0); 
	else 
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & ~LPC_POST_CODE_EN, AST_LPC_PCCR0);

	return count;
}

static DEVICE_ATTR(snoop_dma_en, S_IRUGO | S_IWUSR, show_snoop_dma_en, store_snoop_dma_en); 

static struct attribute *snoop_dma_attributes[] = {
	&dev_attr_snoop_dma_en.attr,
	&dev_attr_snoop_dma_port.attr,
	&dev_attr_snoop_dma_port_mask.attr,	
	&dev_attr_snoop_dma_data.attr,
	NULL
};

static const struct attribute_group snoop_dma_attribute_group = {
	.attrs = snoop_dma_attributes
};


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

/************************************************** SYS FS **************************************************************/
static ssize_t store_lpc2ahb_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val) 
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_ENL2H, AST_LPC_HICR5);
	else
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_ENL2H, AST_LPC_HICR5);

	ast_lpc_write(ast_lpc, 0xffff0000, AST_LPC_HICR8);

	return count;
}

static ssize_t show_lpc2ahb_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_ENL2H)
		return sprintf(buf, "1: Enable\n");
	else
		return sprintf(buf, "0: Disable\n");
}

static DEVICE_ATTR(lpc2ahb_en, S_IRUGO | S_IWUSR, show_lpc2ahb_en, store_lpc2ahb_en); 

static ssize_t store_fwcycle_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val) 
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_ENFWH, AST_LPC_HICR5);
	else
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_ENFWH, AST_LPC_HICR5);

	return count;
}

static ssize_t show_fwcycle_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_ENFWH)
		return sprintf(buf, "1: Enable\n");
	else
		return sprintf(buf, "0: Disable\n");
}

static DEVICE_ATTR(fwcycle_en, S_IRUGO | S_IWUSR, show_fwcycle_en, store_fwcycle_en); 

static ssize_t show_fwcycle_addr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", ast_lpc_read(ast_lpc, AST_LPC_HICR7) );
}

static ssize_t store_fwcycle_addr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);

	val = StrToHex(buf);		

	ast_lpc_write(ast_lpc, val, AST_LPC_HICR7);
	return count;
}

static DEVICE_ATTR(fwcycle_addr, S_IRUGO | S_IWUSR, show_fwcycle_addr, store_fwcycle_addr); 

static struct attribute *ast_lpc_attributes[] = {
	&dev_attr_fwcycle_en.attr,
	&dev_attr_fwcycle_addr.attr,
	&dev_attr_lpc2ahb_en.attr,	
	NULL
};

static const struct attribute_group lpc_attribute_group = {
	.attrs = ast_lpc_attributes
};

/**************************   LPC  Snoop Sys fs   End **********************************************************/
static int
ast_get_route_selio(struct ast_lpc_driver_data *ast_lpc, u8 io_ch, char *sysfsbuf)
{
	u8 tmp = 0;
	switch(io_ch) {
		case 0:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL5IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART0->IO5, 1:UART1->IO5, 2:UART2->IO5, 3:UART3->IO5, 4:UART4->IO5");
			break;
		case 1:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL1IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART1->IO1, 1:UART2->IO1, 2:UART3->IO1, 3:UART4->IO1, 4:UART0->IO1");
			break;
		case 2:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL2IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART2->IO2, 1:UART3->IO2, 2:UART4->IO2, 3:UART0->IO2, 4:UART1->IO2");
			break;
		case 3:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL3IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART3->IO3, 1:UART4->IO3, 2:UART5->IO3, 3:UART1->IO3, 4:UART2->IO3");
			break;
		case 4:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL4IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART4->IO4, 1:UART0->IO4, 2:UART1->IO4, 3:UART2->IO4, 4:UART3->IO4");
			break;
		case 6:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL6IO(ast_lpc_read(ast_lpc, AST_LPC_HICR9)), "0: UART1->IO6, 1:UART2->IO6, 2:UART3->IO6, 3:UART4->IO6, 4:UART0->IO6");
			break;

		default:
			printk("Error Ch no !!\n");
			break;
	}
		
	return tmp;

}

static int 
ast_get_route_seldw(struct ast_lpc_driver_data *ast_lpc, u8 io_ch, char *sysfsbuf)
{
	u8 tmp = 0;
	switch(io_ch) {
		case 0:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL5DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO0->UART0, 1:IO1->UART0, 2:IO2->UART0, 3:IO3->UART0, 4:IO4->UART0, 5:UART1->UART0, 6:UART2->UART0, 7:UART3->UART0, 8:UART4->UART0, 9:IO6->UART0");
			break;
		case 1:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL1DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO1->UART1, 1:IO2->UART1, 2:IO3->UART1, 3:IO4->UART1, 4:UART2->UART1, 5:UART3->UART1, 6:UART4->UART1, 7:IO6->UART1");
			break;
		case 2:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL2DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO2->UART2, 1:IO3->UART2, 2:IO4->UART2, 3:IO1->UART2, 4:UART3->UART2, 5:UART4->UART2, 6:UART1->UART2, 7:IO6->UART2");
			break;
		case 3:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL3DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO3->UART3, 1:IO4->UART3, 2:IO1->UART3, 3:IO2->UART3, 4:UART4->UART3, 5:UART1->UART3, 6:UART2->UART3, 7:IO6->UART3");
			break;
		case 4:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL4DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO4->UART4, 1:IO1->UART4, 2:IO2->UART4, 3:IO3->UART4, 4:UART1->UART4, 5:UART2->UART4, 6:UART3->UART4, 7:IO6->UART4");
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
		
	return tmp;

}

static void
ast_set_route_selio(struct ast_lpc_driver_data *ast_lpc, u8 io_ch, u8 value)
{
	switch(io_ch) {
		case 0:
			ast_lpc_write(ast_lpc, SET_LPC_SEL5IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 	
			break;
		case 1:
			ast_lpc_write(ast_lpc, SET_LPC_SEL1IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 				
			break;
		case 2:
			ast_lpc_write(ast_lpc, SET_LPC_SEL2IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 							
			break;
		case 3:
			ast_lpc_write(ast_lpc, SET_LPC_SEL3IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 							
			break;
		case 4:
			ast_lpc_write(ast_lpc, SET_LPC_SEL4IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 							
			break;
		case 6:
			ast_lpc_write(ast_lpc, SET_LPC_SEL6IO(ast_lpc_read(ast_lpc, AST_LPC_HICR9), value), 
						AST_LPC_HICR9); 							
			break;
			
		default:
			printk("Error Ch no !!\n");
			break;
	}

}

static void
ast_set_route_seldw(struct ast_lpc_driver_data *ast_lpc, u8 io_ch, u8 value)
{
	switch(io_ch) {
		case 0:
			ast_lpc_write(ast_lpc, SET_LPC_SEL5DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 	
			break;
		case 1:
			ast_lpc_write(ast_lpc, SET_LPC_SEL1DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 				
			break;
		case 2:
			ast_lpc_write(ast_lpc, SET_LPC_SEL2DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 				
			break;
		case 3:
			ast_lpc_write(ast_lpc, SET_LPC_SEL3DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 				
			break;
		case 4:
			ast_lpc_write(ast_lpc, SET_LPC_SEL4DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value), 
						AST_LPC_HICRA); 	
			
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
}

static ssize_t 
ast_store_route(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //selio
			ast_set_route_selio(ast_lpc, sensor_attr->index, input_val);	
			break;
		case 1: //seldw
			ast_set_route_seldw(ast_lpc, sensor_attr->index, input_val);
			break;
		default:
			return -EINVAL;
			break;
	}

	return count;
}

static ssize_t 
ast_show_route(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{

	struct ast_lpc_driver_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //selio 
			return ast_get_route_selio(ast_lpc, sensor_attr->index, sysfsbuf);
			break;
		case 1: //seldw
			return ast_get_route_seldw(ast_lpc, sensor_attr->index, sysfsbuf);
			break;			
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
}

#define sysfs_route_ch(index) \
static SENSOR_DEVICE_ATTR_2(route##index##_selio, S_IRUGO | S_IWUSR, \
	ast_show_route, ast_store_route, 0, index); \
static SENSOR_DEVICE_ATTR_2(route##index##_seldw, S_IRUGO | S_IWUSR, \
	ast_show_route, ast_store_route, 1, index); \
\
static struct attribute *route##index##_attributes[] = { \
	&sensor_dev_attr_route##index##_selio.dev_attr.attr, \
	&sensor_dev_attr_route##index##_seldw.dev_attr.attr, \
	NULL \
};

sysfs_route_ch(0);
sysfs_route_ch(1);
sysfs_route_ch(2);
sysfs_route_ch(3);
sysfs_route_ch(4);
sysfs_route_ch(5);
sysfs_route_ch(6);


static const struct attribute_group route_attribute_groups[] = {
	{ .attrs = route0_attributes },
	{ .attrs = route1_attributes },
	{ .attrs = route2_attributes },
	{ .attrs = route3_attributes },
	{ .attrs = route4_attributes },
	{ .attrs = route5_attributes },
	{ .attrs = route6_attributes },	
};

/**************************   LPC  route Sys fs   End **********************************************************/
u8 sbuf[18] = {0x1c, 0x01, 0x00, 0x20, 0x01, 0x01, 0x01, 0x02,
                 0xBF, 0x00, 0x00, 0x00, 0xBB, 0xAA, 0x00, 0x00,
                 0x00, 0x00};
///////////////////////////////////

#define SNOOP_DMA_BOUNDARY 0x1000

static int ast_lpc_probe(struct platform_device *pdev)
{
//	const struct platform_device_id *id = platform_get_device_id(pdev);
	static struct ast_lpc_driver_data *ast_lpc;
	struct resource *res;
	int ret = 0;
	int i = 0;
	LPC_DBUG("\n");	

	ast_lpc = kzalloc(sizeof(struct ast_lpc_driver_data), GFP_KERNEL);
	if (ast_lpc == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ast_lpc->pdev = pdev;

	ast_lpc->bus_info = pdev->dev.platform_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free;
	}

	ast_lpc->reg_base = ioremap(res->start, resource_size(res));
	if (ast_lpc->reg_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	ast_lpc->irq = platform_get_irq(pdev, 0);
	if (ast_lpc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto err_free_mem;
	}

	ret = request_irq(ast_lpc->irq, ast_lpc_isr, IRQF_SHARED, "ast-lpc", ast_lpc);
	if (ret) {
		printk("AST LPC Unable to get IRQ");
		goto err_free_mem;
	}

	if(ast_lpc->bus_info->lpc_bus_mode) {
		printk("LPC Scan Device... \n");
#ifdef CONFIG_ARCH_AST1070			
		for(i=0;i<ast_lpc->bus_info->scan_node;i++) {
			ast1070_scu_init(i ,ast_lpc->bus_info->bridge_phy_addr + i*0x10000);
			printk("C%d-[%x] ", i, ast1070_revision_id_info(i));
			ast1070_vic_init(i, (ast_lpc->bus_info->bridge_phy_addr + i*0x10000), IRQ_C0_VIC_CHAIN + i, IRQ_C0_VIC_CHAIN_START + (i*AST_CVIC_NUM));
			ast1070_i2c_irq_init(i, (ast_lpc->bus_info->bridge_phy_addr + i*0x10000), IRQ_C0_VIC_CHAIN + i, IRQ_C0_VIC_CHAIN_START + (i*AST_CVIC_NUM));
			ast1070_scu_dma_init(i);
			ast1070_uart_dma_init(i, ast_lpc->bus_info->bridge_phy_addr);
			ast_add_device_cuart(i,ast_lpc->bus_info->bridge_phy_addr + i*0x10000);
			ast_add_device_ci2c(i,ast_lpc->bus_info->bridge_phy_addr + i*0x10000);
		}
#endif			
		printk("\n");
		platform_set_drvdata(pdev, ast_lpc);
	} else {
		platform_set_drvdata(pdev, ast_lpc);

		dev_set_drvdata(&pdev->dev, ast_lpc);

		ret = sysfs_create_group(&pdev->dev.kobj, &lpc_attribute_group);
		if (ret)
			goto err_free_mem;

		for(i=0; i< 7; i++) {
			ret = sysfs_create_group(&pdev->dev.kobj, &route_attribute_groups[i]);
			if (ret)
				goto err_free_mem;
		}

		if(ast_lpc->bus_info->snoop_enable) {
			LPC_DBUG("SNOOP Init \n");
			ast_lpc->snoop_data = kzalloc(sizeof(struct ast_snoop_data) * 2, GFP_KERNEL);
			//enable irq
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_SNP0INT_EN | LPC_HICR5_SNP1INT_EN, 
						AST_LPC_HICR5); 	

			for(i=0; i< 2; i++) {
				ret = sysfs_create_group(&pdev->dev.kobj, &snoop_attribute_groups[i]);
				if (ret)
					goto err_free_mem;
			}
			//snoop dma
			ast_lpc->ast_lpc_snoop_dma_virt = (unsigned char *)dma_alloc_coherent(NULL, 
										SNOOP_DMA_BOUNDARY, &ast_lpc->ast_lpc_snoop_dma, GFP_KERNEL);

			ast_lpc_write(ast_lpc, ast_lpc->ast_lpc_snoop_dma, AST_LPC_PCCR4);
			//dma size
			ast_lpc_write(ast_lpc, SNOOP_DMA_BOUNDARY/4, AST_LPC_PCCR5);
			ret = sysfs_create_group(&pdev->dev.kobj, &snoop_dma_attribute_group);
			if (ret)
				goto err_free_mem;
			
		}

		//snoop dma mode
//		ret = sysfs_create_file(&pdev->dev.kobj, &snoop_dma_attribute);
//		if (ret)
//			goto err_free_mem;
		
		if(ast_lpc->bus_info->ipmi_kcs_enable) {
			LPC_DBUG("IPMI KCS Init \n");
			ast_lpc->kcs_data = kzalloc(sizeof(struct ast_kcs_data) * 4, GFP_KERNEL);
			
			ast_lpc->kcs_data[0].str = AST_LPC_STR1;
			ast_lpc->kcs_data[0].idr = AST_LPC_IDR1;
			ast_lpc->kcs_data[0].odr = AST_LPC_ODR1;
			ast_lpc->kcs_data[0].pKCSRcvPkt = kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
			ast_lpc->kcs_data[0].pKCSSendPkt = sbuf; //kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);

			ast_lpc->kcs_data[1].str = AST_LPC_STR2;
			ast_lpc->kcs_data[1].idr = AST_LPC_IDR2;
			ast_lpc->kcs_data[1].odr = AST_LPC_ODR2;
			ast_lpc->kcs_data[1].pKCSRcvPkt = kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
			ast_lpc->kcs_data[1].pKCSSendPkt = sbuf; //kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
			
			ast_lpc->kcs_data[2].str = AST_LPC_STR3;
			ast_lpc->kcs_data[2].idr = AST_LPC_IDR3;
			ast_lpc->kcs_data[2].odr = AST_LPC_ODR3;
			ast_lpc->kcs_data[2].pKCSRcvPkt = kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
			ast_lpc->kcs_data[2].pKCSSendPkt = sbuf; //kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
			ast_lpc->kcs_data[2].KCSSendPktLen = 18;
			
			ast_lpc->kcs_data[3].str = AST_LPC_STR4;
			ast_lpc->kcs_data[3].idr = AST_LPC_IDR4;
			ast_lpc->kcs_data[3].odr = AST_LPC_ODR4;
			ast_lpc->kcs_data[3].pKCSRcvPkt = kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);
			ast_lpc->kcs_data[3].pKCSSendPkt = sbuf; //kmalloc(IPMI_MAX_MSG_LENGTH, GFP_KERNEL);

			for(i=0; i< 4; i++) {
				ret = sysfs_create_group(&pdev->dev.kobj, &ipmi_kcs_attribute_groups[i]);
				if (ret)
					goto err_free_mem;
			}
		}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////					
		if(ast_lpc->bus_info->ipmi_bt_enable) {
			LPC_DBUG("IPMI BT Init \n");
			ast_lpc->bt_data = kzalloc(sizeof(struct ast_bt_data) * 2, GFP_KERNEL);
			ast_lpc->bt_data[0].str = AST_LPC_BTDTR;
			ast_lpc->bt_data[0].fifo = AST_LPC_BTR1;
			ast_lpc->bt_data[0].seq = 0;
			ast_lpc->bt_data[0].state = BT_STATE_IDLE;	/* start here */
			ast_lpc->bt_data[0].complete = BT_STATE_IDLE;	/* end here */
			ast_lpc->bt_data[0].BT_CAP_req2rsp = BT_NORMAL_TIMEOUT * 1000000;
			ast_lpc->bt_data[0].BT_CAP_retries = BT_NORMAL_RETRY_LIMIT;

			ast_lpc->bt_data[1].str = AST_LPC_IBTCR4;
			ast_lpc->bt_data[1].fifo = AST_LPC_IBTCR5;
			ast_lpc->bt_data[1].seq = 0;
			ast_lpc->bt_data[1].state = BT_STATE_IDLE;	/* start here */
			ast_lpc->bt_data[1].complete = BT_STATE_IDLE;	/* end here */
			ast_lpc->bt_data[1].BT_CAP_req2rsp = BT_NORMAL_TIMEOUT * 1000000;
			ast_lpc->bt_data[1].BT_CAP_retries = BT_NORMAL_RETRY_LIMIT;

			for(i=0; i< 2; i++) {
				ret = sysfs_create_group(&pdev->dev.kobj, &ipmi_bt_attribute_groups[i]);
				if (ret)
					goto err_free_mem;
			}
		}	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////						
	}
	
	return 0;

err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free:
	kfree(ast_lpc);

	return ret;
}

#if 0
static struct ast_lpc_data ast_lpc_data = {
	.counter_width = 16,
};
#endif

static struct platform_driver ast_lpc_driver = {
	.driver		= {
		.name			= "ast-lpc",
		.owner			= THIS_MODULE,
	},
	.probe 		= ast_lpc_probe,
//	.remove		= ast_lpc_remove,	
//	.id_table		= ast_lpc_idtable,	
};

static int __init ast_lpc_init(void)
{
	return platform_driver_register(&ast_lpc_driver);
}
arch_initcall(ast_lpc_init);
