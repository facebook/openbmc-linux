/********************************************************************************
* File Name     : ast_bt.c
* Author         : Ryan Chen
* Description   : AST BT Controller
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
*      1. 2016/01/30 Ryan Chen create this file 
*    
********************************************************************************/
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>
	
#include <plat/ast-lpc.h>
#include <plat/regs-lpc.h>

//#define CONFIG_AST_BT_DEBUG

#ifdef CONFIG_AST_BT_DEBUG
//#define BT_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#define BT_DBG(fmt, args...) printk(fmt, ## args)

#else
#define BT_DBG(fmt, args...)
#endif

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

/***************** BT Handle ******************************************************/
static void 
write_all_bytes(struct ast_bt_data *ast_bt)
{
	int i;

	BT_DBG("write_all_bytes host wp %d \n", LPC_iBT_GET_HOST_WP(readl(ast_bt->fifo_sts)));
	for (i = 0; i < ast_bt->write_count; i++) {
		BT_DBG("%x ", ast_bt->write_data[i]);
		writel(ast_bt->write_data[i], ast_bt->fifo);
		if(LPC_iBT_GET_HOST_WP(readl(ast_bt->fifo_sts)) == 64) {
			writel(BT_B2H_ATN, ast_bt->str);
		}
		while(readl(ast_bt->str) & (BT_H_BUSY | BT_B2H_ATN));
	}

	BT_DBG("\n");
}

static int 
read_all_bytes(struct ast_bt_data *ast_bt)
{
	unsigned char i;
	BT_DBG("\n");

	/*
	 * length is "framing info", minimum = 3: NetFn, Seq, Cmd
	 * Keep layout of first four bytes aligned with write_data[]
	 */

	ast_bt->read_data[0] = readl(ast_bt->fifo);
	ast_bt->read_count = ast_bt->read_data[0] + 1;
	
	BT_DBG("bt->read_count = %x\n", ast_bt->read_count);

	if (ast_bt->read_count < 3 || ast_bt->read_count >= AST_IPMI_PKT_SIZE) {
		printk("BT: bad raw rsp len=%d\n", ast_bt->read_count);
		ast_bt->truncated = 1;
		return 1;	/* let next XACTION START clean it up */
	}
	for (i = 1; i < ast_bt->read_count; i++)
		ast_bt->read_data[i] = readl(ast_bt->fifo);

	BT_DBG ("bt->read_data = ");
	for (i = 0; i < ast_bt->read_count; i++) {
		BT_DBG ("%x ", ast_bt->read_data[i]);
	}
	BT_DBG ("\n");

	return 0;
}

#define BT_STATE_CHANGE(X, Y) { ast_bt->state = X; return Y; }
#define BT_SI_SM_RETURN(Y)   { last_printed = BT_STATE_PRINTME; return Y; }
/* Check status and (usually) take action and change this state machine. */
static enum si_sm_result ibt_event(struct ast_bt_data *ast_bt)
{
	unsigned char status;
	static enum bt_states last_printed = BT_STATE_PRINTME;
	int i;

	status = readl(ast_bt->str);
	
	BT_DBG("sts : %x state : %d\n", status, ast_bt->state);

	switch (ast_bt->state) {
	/* 
	 * Idle state first checks for asynchronous messages from another
	 * channel, then does some opportunistic housekeeping.
	 */

	case BT_STATE_IDLE:
		if (status & BT_B_BUSY)		/* clear a leftover B_BUSY */
			writel(BT_B_BUSY, ast_bt->str);

		BT_STATE_CHANGE(BT_STATE_READ_WAIT,
				SI_SM_CALL_WITHOUT_DELAY);
		
	case BT_STATE_XACTION_START:
		if (status & (BT_H_BUSY | BT_B2H_ATN))
			BT_SI_SM_RETURN(SI_SM_CALL_WITH_DELAY);
		if (readl(ast_bt->str) & BT_B_BUSY)
			writel(BT_B_BUSY, ast_bt->str);	/* force clear */
		BT_STATE_CHANGE(BT_STATE_WRITE_BYTES,
				SI_SM_CALL_WITHOUT_DELAY);

	case BT_STATE_WRITE_BYTES:
		if (status & BT_B_BUSY)
			writel(BT_B_BUSY, ast_bt->str);	/* clear */
//yriver
//		BT_CONTROL(BT_CLR_WR_PTR);
		write_all_bytes(ast_bt);
		writel(BT_B2H_ATN, ast_bt->str);

		ast_bt->state = BT_STATE_IDLE;
		return SI_SM_TRANSACTION_COMPLETE;

	case BT_STATE_READ_WAIT:
		if (!(status & BT_H2B_ATN))
			BT_SI_SM_RETURN(SI_SM_CALL_WITH_DELAY);
		writel(BT_B_BUSY, ast_bt->str);		/* set */
		writel(BT_B2H_ATN, ast_bt->str);		/* clear it to ACK the host */
		BT_STATE_CHANGE(BT_STATE_CLEAR_H2B,
				SI_SM_CALL_WITHOUT_DELAY);

	case BT_STATE_CLEAR_H2B:
		BT_DBG("BT_STATE_CLEAR_H2B status %x \n", readl(ast_bt->str));
		if (status & BT_H2B_ATN) {
			/* keep hitting it */
			writel(BT_H2B_ATN, ast_bt->str);
			BT_SI_SM_RETURN(SI_SM_CALL_WITH_DELAY);
		}
		BT_STATE_CHANGE(BT_STATE_READ_BYTES,
				SI_SM_CALL_WITHOUT_DELAY);

	case BT_STATE_READ_BYTES:
		if (!(status & BT_B_BUSY))
			/* check in case of retry */
			writel(BT_B_BUSY, ast_bt->str);	
//yriver
//		BT_CONTROL(BT_CLR_RD_PTR);	/* start of BMC2HOST buffer */
		i = read_all_bytes(ast_bt);		/* true == packet seq match */
		writel(BT_B_BUSY, ast_bt->str);			/* NOW clear */
		BT_STATE_CHANGE(BT_STATE_IDLE,
				SI_SM_READ_COMPLETE);

	default:
		return SI_SM_CALL_WITH_DELAY;
	}
	return SI_SM_CALL_WITH_DELAY;
}

static void ast_bt_tasklet_func(unsigned long data)
{
	u8 result;
	unsigned int count = 0;	

	struct ast_bt_data *ast_bt = (struct ast_bt_data *)data;

	do {
		result = ibt_event(ast_bt);
		count++;
	} while (((result != SI_SM_READ_COMPLETE) && (count <= 100000)));
	if (count >= 100000) printk ("SI_SM_READ_NOT_COMPLETE\n");

	ast_bt->BTPktRdy = 1;
}

static void ast_ipmi_bt_handle(struct ast_bt_data *ast_bt) 
{
	u32 isr = readl(ast_bt->isr);

	if(isr & LPC_iBT_H2B_RISING_ISR) {
		BT_DBG("LPC_iBT_H2B_RISING_ISR \n");
		writel(LPC_iBT_H2B_RISING_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_BMC_HWRST_ISR) {
		BT_DBG("LPC_iBT_BMC_HWRST_ISR \n");
		writel(LPC_iBT_BMC_HWRST_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_OEM3_RISING_ISR) {
		BT_DBG("LPC_iBT_OEM3_RISING_ISR \n");
		writel(LPC_iBT_OEM3_RISING_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_OEM2_RISING_ISR) {
		BT_DBG("LPC_iBT_OEM2_RISING_ISR \n");
		writel(LPC_iBT_OEM2_RISING_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_OEM1_RISING_ISR) {
		BT_DBG("LPC_iBT_OEM1_RISING_ISR \n");
		writel(LPC_iBT_OEM1_RISING_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_SREAD_OV_ISR) {
		BT_DBG("LPC_iBT_SREAD_OV_ISR \n");
		writel(LPC_iBT_SREAD_OV_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_SWRITE_OV_ISR) {
		BT_DBG("LPC_iBT_SWRITE_OV_ISR \n");
		writel(LPC_iBT_SWRITE_OV_ISR, ast_bt->isr);
	}
		
	if(isr & LPC_iBT_HREAD_OV_ISR) {
		BT_DBG("LPC_iBT_HREAD_OV_ISR \n");
		writel(LPC_iBT_HREAD_OV_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_HWRITE_OV_ISR) {
		BT_DBG("LPC_iBT_HWRITE_OV_ISR \n");
		writel(LPC_iBT_HWRITE_OV_ISR, ast_bt->isr);
	}

	if(isr & LPC_iBT_HBUSY_FALLING_ISR) {
		BT_DBG("LPC_iBT_HBUSY_FALLING_ISR \n");
		writel(LPC_iBT_HBUSY_FALLING_ISR, ast_bt->isr);
	}
	
	if(isr & LPC_iBT_SMS_FALLING_ISR) {
		BT_DBG("LPC_iBT_SMS_FALLING_ISR \n");
		writel(LPC_iBT_SMS_FALLING_ISR, ast_bt->isr);
	}
	
	if(isr & LPC_iBT_B2H_FALLING_ISR) {
		BT_DBG("LPC_iBT_B2H_FALLING_ISR \n");
		writel(LPC_iBT_B2H_FALLING_ISR, ast_bt->isr);
	}
	
	if(isr & LPC_iBT_OEM0_RISING_ISR) {
		BT_DBG("LPC_iBT_OEM0_RISING_ISR \n");
		writel(LPC_iBT_OEM0_RISING_ISR, ast_bt->isr);
	}
		
	tasklet_schedule(&ast_bt->bt_tasklet);

}

/**************************   LPC  BT SYSFS  **********************************************************/
static ssize_t 
store_ipmi_bt_irq(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_bt_data *ast_bt = dev_get_drvdata(dev);
	input_val = StrToHex(sysfsbuf);
	ast_set_ipmi_bt_irq(ast_bt->ast_lpc, ast_bt->pdev->id, input_val);
	return count;
}

static ssize_t 
show_ipmi_bt_irq(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_bt_data *ast_bt = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n", ast_get_ipmi_bt_irq(ast_bt->ast_lpc, ast_bt->pdev->id));
}
static DEVICE_ATTR(irq, S_IRUGO | S_IWUSR, show_ipmi_bt_irq, store_ipmi_bt_irq);

static ssize_t 
store_ipmi_bt_addr(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_bt_data *ast_bt = dev_get_drvdata(dev);
	input_val = StrToHex(sysfsbuf);
	ast_set_ipmi_bt_addr(ast_bt->ast_lpc, ast_bt->pdev->id, input_val);
	return count;
}

static ssize_t 
show_ipmi_bt_addr(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_bt_data *ast_bt = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n", ast_get_ipmi_bt_addr(ast_bt->ast_lpc, ast_bt->pdev->id));
}
static DEVICE_ATTR(addr, S_IRUGO | S_IWUSR, show_ipmi_bt_addr, store_ipmi_bt_addr);

static ssize_t 
store_ipmi_bt_en(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_bt_data *ast_bt = dev_get_drvdata(dev);
	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	ast_set_ipmi_bt_en(ast_bt->ast_lpc, ast_bt->pdev->id, input_val);
	return count;
}

static ssize_t 
show_ipmi_bt_en(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_bt_data *ast_bt = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%d : %s\n", ast_get_ipmi_bt_en(ast_bt->ast_lpc, ast_bt->pdev->id),ast_get_ipmi_bt_en(ast_bt->ast_lpc, ast_bt->pdev->id) ? "Enable":"Disable");
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, show_ipmi_bt_en, store_ipmi_bt_en);

static struct attribute *ast_ipmi_bt_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_addr.attr,
	&dev_attr_irq.attr,		
	NULL
};

static const struct attribute_group ipmi_bt_attribute_group = {
	.attrs = ast_ipmi_bt_attributes,
//	.is_visible = ast_ipmi_bt_attrs_visible,
};

/***********************************************************************/
static ssize_t ast_bt_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	struct miscdevice *c = file->private_data;
	struct ast_bt_data *ast_bt = dev_get_drvdata(c->this_device);
	
	if(ast_bt->BTPktRdy) {
		BT_DBG("ioctl : ast_bt_read\n");
		if(copy_to_user(buf, ast_bt->read_data, ast_bt->read_count)) {
			dev_err(&(ast_bt->pdev->dev), "read fail \n");
			return -EINVAL;			
		}
		return ast_bt->read_count;
	} else {
		return 0;
	}
}

ssize_t ast_bt_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	struct miscdevice *c = file->private_data;
	struct ast_bt_data *ast_bt = dev_get_drvdata(c->this_device);
	u8 result;
	unsigned int timeout = 0;	

	if (count > AST_IPMI_PKT_SIZE) {
		dev_err(&(ast_bt->pdev->dev), "write too long, len = %d\n", (int)count);
		return -EINVAL;
	}

	if (count < 2) {
		printk("IPMI_REQ_LEN_INVALID_ERR \n");
		return -EINVAL;
	}
	
	if (ast_bt->state == BT_STATE_LONG_BUSY) {
		printk("IPMI_NODE_BUSY_ERR \n");
		return -EINVAL;
	}

	if (ast_bt->state != BT_STATE_IDLE) {
		printk("IPMI_NOT_IN_MY_STATE_ERR \n");
		return -EINVAL;
	}

	if(ast_bt->BTPktRdy) {		
		BT_DBG("ioctl : ast_bt_write %d \n", count);

		ast_bt->BTPktRdy = 0;
		if(copy_from_user(ast_bt->write_data, buf, count)) {
			dev_err(&(ast_bt->pdev->dev), "copy fail = %d, \n", (int)count);
			return -EINVAL;
		}
		ast_bt->completion_code_length = 1;	
		ast_bt->completion_code[0] = 0;	

		ast_bt->write_count = count;
		ast_bt->error_retries = 0;
		ast_bt->truncated = 0;
		
		/* Send the first byte */
		ast_bt->state = BT_STATE_XACTION_START;
		ast_bt->timeout = ast_bt->BT_CAP_req2rsp;
		
		do {
			result = ibt_event(ast_bt);
			timeout++;
		} while ((result != SI_SM_TRANSACTION_COMPLETE) && (timeout <= 100000));
		if (timeout >= 100000) printk("SI_SM_TRANSACTION_NOT_COMPLETE\n");
				
		return count;
	} else {
		return -EINVAL;
	}
	
}

static int ast_bt_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_bt_data *ast_bt = dev_get_drvdata(c->this_device);

	BT_DBG("ast_bt_open\n");

	/* Flush input queue on first open */
	if (ast_bt->open_count)
		return -1;
	
	ast_bt->open_count++;
	return 0;
}

static int ast_bt_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_bt_data *ast_bt = dev_get_drvdata(c->this_device);

	BT_DBG("ast_bt_release\n");
	ast_bt->open_count--;
	
	return 0;	
}

static const struct file_operations ast_bt_fops = {
	.owner 			=  	THIS_MODULE,
	.llseek 			= 	no_llseek,
	.read			=	ast_bt_read,
	.write			=	ast_bt_write,	
	.open			=	ast_bt_open,
	.release 			=    	ast_bt_release,
};

static void ast_bt_ctrl_init(void)
{
}

static int ast_bt_probe(struct platform_device *pdev)
{
	int ret=0;

	char bt_name[10];
	struct ast_bt_data *ast_bt = register_ipmi_bt_drv(pdev->id);

	BT_DBG("ast_bt_probe\n");	
	
	ret = sysfs_create_group(&pdev->dev.kobj, &ipmi_bt_attribute_group);
	if (ret)
		goto out;

	ast_bt->pdev = pdev;
	ast_bt->miscdev.minor = MISC_DYNAMIC_MINOR;

	snprintf(bt_name, sizeof(bt_name), "ast-bt.%d", pdev->id);

	ast_bt->miscdev.fops = &ast_bt_fops;
	ast_bt->miscdev.parent = &pdev->dev;
	ast_bt->miscdev.name = bt_name;
	ret = misc_register(&ast_bt->miscdev);
	if (ret){		
		printk(KERN_ERR "BT : failed to request interrupt\n");
		goto out;
	}

	tasklet_init(&ast_bt->bt_tasklet, ast_bt_tasklet_func,
			(unsigned long)ast_bt);

	request_ipmi_bt_irq(pdev->id, ast_ipmi_bt_handle);

	platform_set_drvdata(pdev, ast_bt);		

	dev_set_drvdata(ast_bt->miscdev.this_device, ast_bt);	

	printk(KERN_INFO "ast_bt.%d: driver successfully loaded.\n",pdev->id);

	return 0;

out:
	printk(KERN_WARNING "ast_bt: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_bt_remove(struct platform_device *pdev)
{
	struct ast_bt_data *ast_bt = platform_get_drvdata(pdev);
	
	BT_DBG("ast_bt_remove\n");

	misc_deregister(&ast_bt->miscdev);
	kfree(ast_bt);

	return 0;	
}

static struct platform_driver ast_bt_driver = {
	.remove 		= ast_bt_remove,
	.driver         = {
		.name   = "ast-bt",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_bt_driver, ast_bt_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST BT driver");
MODULE_LICENSE("GPL");
