/********************************************************************************
* File Name     : ast_kcs.c
* Author         : Ryan Chen
* Description   : AST IPMI KCS
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
# Linux x86 host 
# modprobe ipmi_si type=kcs ports=0xca2 regspacings=1
# modprobe ipmi_si type=kcs ports=0xca8 regspacings=4
# x86 KCS Port : CA2/CA3, CB2/CB3, CA0/CA4, CA8/CAC
# Set KCS default address : KCS0 : CA0/CA4, KCS1 : CA8/CAC, KCS3 : CA2/CA3, KCS4 : CB2/CB3

*test in Dos [Notice : ipmitool only use fix in ca2/ca3 ]
# IPMITOOL 20 18 01
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

//#define CONFIG_KCS_DEBUG

#ifdef CONFIG_KCS_DEBUG
#define KCS_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define KCS_DBG(fmt, args...)
#endif

struct kcs_parameter {
	u8	ch_enable;
	u8	ch_type;	
};


#define KCS_CMD_DAT	0x8
#define KCS_IBF 			0x2
#define KCS_OBF			0x1
////////////////////////////////////////////
/* KCS Command Control codes. */
#define KCS_GET_STATUS_ABORT	0x60
#define KCS_WRITE_START			0x61
#define KCS_WRITE_END			0x62
#define KCS_READ_BYTE			0x68

/* Status bits. */
#define IPMI_KCS_IDLE_STATE		(0 << 6)
#define IPMI_KCS_READ_STATE	(1 << 6)
#define IPMI_KCS_WRITE_STATE	(2 << 6)
#define IPMI_KCS_ERROR_STATE	(3 << 6)


#define GET_STATUS_STATE(status) (((status) >> 6) & 0x03)
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
//read id 
u8 sbuf[18] = {0x1c, 0x01, 0x00, 0x20, 0x01, 0x01, 0x01, 0x02,
                 0xBF, 0x00, 0x00, 0x00, 0xBB, 0xAA, 0x00, 0x00,
                 0x00, 0x00};



/************************  IPMI KCS  ****************************************************************/
static inline unsigned char read_kcs_status(struct ast_kcs_data *ast_kcs)
{
	return readl(ast_kcs->str);
}

static inline unsigned char read_kcs_data(struct ast_kcs_data *ast_kcs)
{
	return readl(ast_kcs->idr);
}

static inline unsigned char read_kcs_cmd(struct ast_kcs_data *ast_kcs)
{
	return readl(ast_kcs->idr);
}

static inline void write_kcs_data(struct ast_kcs_data *ast_kcs, unsigned char data)
{
	writel(data, ast_kcs->odr);
}

static inline void write_kcs_status(struct ast_kcs_data *ast_kcs, unsigned char str)
{
	u32 sts = readl(ast_kcs->str) & (~0xC0);	
	writel( sts | str, ast_kcs->str);
}

/**************************   LPC  KCS SYSFS  **********************************************************/
static ssize_t 
store_ipmi_kcs_sms_atn(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	if(input_val) 
		writel(readl(ast_kcs->str) | (0x1 << 2), ast_kcs->str);
	else
		writel(readl(ast_kcs->str) & ~(0x1 << 2), ast_kcs->str);
	return count;
}

static ssize_t 
show_ipmi_kcs_sms_atn(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%d\n", (readl(ast_kcs->str) & 0x4) >> 2);
}
static DEVICE_ATTR(sms_atn, S_IRUGO | S_IWUSR, show_ipmi_kcs_sms_atn, store_ipmi_kcs_sms_atn);

static ssize_t 
store_ipmi_kcs_oem(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	input_val &= 0x3;
	writel((readl(ast_kcs->str) & 0xcf) | (input_val << 4), ast_kcs->str);
	return count;
}

static ssize_t 
show_ipmi_kcs_oem(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n", (readl(ast_kcs->str) & 0xcf) >> 4);
}
static DEVICE_ATTR(oem, S_IRUGO | S_IWUSR, show_ipmi_kcs_oem, store_ipmi_kcs_oem);

static ssize_t 
store_ipmi_kcs_str(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	input_val = StrToHex(sysfsbuf);
	writel(input_val, ast_kcs->str);
	return count;
}

static ssize_t 
show_ipmi_kcs_str(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n", readl(ast_kcs->str));
}
static DEVICE_ATTR(str, S_IRUGO | S_IWUSR, show_ipmi_kcs_str, store_ipmi_kcs_str);

static ssize_t 
store_ipmi_kcs_addr(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	input_val = StrToHex(sysfsbuf);
	ast_set_ipmi_kcs_addr(ast_kcs->ast_lpc, ast_kcs->pdev->id, input_val);
	return count;
}

static ssize_t 
show_ipmi_kcs_addr(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n", ast_get_ipmi_kcs_addr(ast_kcs->ast_lpc, ast_kcs->pdev->id));
}
static DEVICE_ATTR(addr, S_IRUGO | S_IWUSR, show_ipmi_kcs_addr, store_ipmi_kcs_addr);

static ssize_t 
store_ipmi_kcs_en(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	ast_set_ipmi_kcs_en(ast_kcs->ast_lpc, ast_kcs->pdev->id, input_val);
	return count;
}

static ssize_t 
show_ipmi_kcs_en(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%d : %s\n", ast_get_ipmi_kcs_en(ast_kcs->ast_lpc, ast_kcs->pdev->id),ast_get_ipmi_kcs_en(ast_kcs->ast_lpc, ast_kcs->pdev->id) ? "Enable":"Disable");
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, show_ipmi_kcs_en, store_ipmi_kcs_en);


static struct attribute *ast_ipmi_kcs_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_addr.attr,
	&dev_attr_str.attr,	
	&dev_attr_sms_atn.attr,	
	&dev_attr_oem.attr,		
//	&dev_attr_buff.attr,		
	NULL
};

static const struct attribute_group ipmi_kcs_attribute_group = {
	.attrs = ast_ipmi_kcs_attributes,
//	.is_visible = ast_ipmi_kcs_attrs_visible,
};

/***********************************************************************/

static int ast_kcs_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(c->this_device);

	KCS_DBG("ast_kcs_open\n");

	/* Flush input queue on first open */
	if (ast_kcs->open_count)
		return -1;
	
	ast_kcs->open_count++;
	return 0;
}

static int ast_kcs_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(c->this_device);

	KCS_DBG("ast_kcs_release\n");
	ast_kcs->open_count--;
	
	return 0;	
}

static ssize_t ast_kcs_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	struct miscdevice *c = file->private_data;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(c->this_device);
	

	if(ast_kcs->KCSPktRdy) {
		if(copy_to_user(buf, ast_kcs->pKCSRcvPkt, ast_kcs->KCSRcvPktIx)) {
			dev_err(&(ast_kcs->pdev->dev), "read fail \n");
			return -EINVAL;			
		}
		ast_kcs->KCSSendWait = 1;		
		ast_kcs->KCSPktRdy = 0;
		return ast_kcs->KCSRcvPktIx;
	} else {
		return 0;
	}
}

ssize_t ast_kcs_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	struct miscdevice *c = file->private_data;
	struct ast_kcs_data *ast_kcs = dev_get_drvdata(c->this_device);

	if (count > AST_IPMI_PKT_SIZE) {
		dev_err(&(ast_kcs->pdev->dev), "write too long, len = %d\n", (int)count);
		return -EINVAL;
	}

	if(ast_kcs->KCSSendWait) {
		KCS_DBG("count %d \n", count);
		if(count) {
			if(copy_from_user(ast_kcs->pKCSSendPkt, buf, count)) {
				dev_err(&(ast_kcs->pdev->dev), "copy fail = %d, \n", (int)count);
				return -EINVAL;
			}
			ast_kcs->KCSSendWait = 0;		
			/* Send the first byte */
			ast_kcs->KCSSendPktIx = 0;
			ast_kcs->KCSSendPktLen = count;
			KCS_DBG("send idx %d : %x \n",ast_kcs->KCSSendPktIx, ast_kcs->pKCSSendPkt[ast_kcs->KCSSendPktIx]);
			if(ast_kcs->KCSPhase == KCS_PHASE_READ) {
				write_kcs_data(ast_kcs, ast_kcs->pKCSSendPkt[ast_kcs->KCSSendPktIx]);
				ast_kcs->KCSSendPktIx++;
			} else {
				printk("KCS resp error \n");
				return -EINVAL;
			}
		} else {
			ast_kcs->KCSSendWait = 0;		
			/* Send the first byte */
			ast_kcs->KCSSendPktIx = 0;
			ast_kcs->KCSSendPktLen = 0;
			write_kcs_status(ast_kcs, IPMI_KCS_ERROR_STATE);
		}
		
		return count;
	} else {
		return -EINVAL;
	}
	
}

static const struct file_operations ast_kcs_fops = {
	.owner 			=	THIS_MODULE,
	.llseek 			=	no_llseek,
	.read 			= 	ast_kcs_read,
	.write 			= 	ast_kcs_write,	
	.open 			=	ast_kcs_open,
	.release 		=	ast_kcs_release,
};

/***********************************************************************/
static void ast_ipmi_kcs_rx(struct ast_kcs_data *ast_kcs)
{
	u8 b;
	int i;
	switch (ast_kcs->KCSPhase) {
		case KCS_PHASE_WRITE:
			KCS_DBG("KCS_PHASE_WRITE \n");
			/* Set the state to write state */
			write_kcs_status(ast_kcs, IPMI_KCS_WRITE_STATE);
			/* Read the BYTE from the data register */
			ast_kcs->pKCSRcvPkt[ast_kcs->KCSRcvPktIx] = read_kcs_data(ast_kcs);
			KCS_DBG("rx data = [%x] \n", ast_kcs->pKCSRcvPkt[ast_kcs->KCSRcvPktIx]);
			ast_kcs->KCSRcvPktIx++;
			if(ast_kcs->KCSRcvPktIx > 272)
				printk("ERROR ---> TODO ... \n");
			break;
			
		case KCS_PHASE_WRITE_END :
			KCS_DBG("KCS_PHASE_WRITE_END \n");
			/* Set the state to READ_STATE */
			write_kcs_status(ast_kcs, IPMI_KCS_READ_STATE);
			/* Read the BYTE from the data register */
			ast_kcs->pKCSRcvPkt[ast_kcs->KCSRcvPktIx] = read_kcs_data(ast_kcs);
			KCS_DBG("rx data = [%x] \n", ast_kcs->pKCSRcvPkt[ast_kcs->KCSRcvPktIx]);
			ast_kcs->KCSRcvPktIx++;

			/* Move to READ Phase */
			ast_kcs->KCSPhase = KCS_PHASE_READ;
			/* Signal receive data ready */
			ast_kcs->KCSPktRdy = 1;
#if 1		
			printk(KERN_DEBUG "Total Rx Data : [");
			for(i=0; i < ast_kcs->KCSRcvPktIx; i++)
				printk(KERN_DEBUG " %x", ast_kcs->pKCSRcvPkt[i]);
			printk(KERN_DEBUG "] \n");
#endif		
			//trigger timeout --> TODO ~~~~
			break;
		case KCS_PHASE_READ:
			KCS_DBG("KCS_PHASE_READ \n");

			/* If we have reached the end of the packet move to idle state */
			if (ast_kcs->KCSSendPktIx == ast_kcs->KCSSendPktLen)
				write_kcs_status(ast_kcs, IPMI_KCS_IDLE_STATE);

			/* Read the byte returned by the SMS */
			b = read_kcs_cmd(ast_kcs);
			//SA Need to clear IBF
			//sa_0111 CLEAR_IBF_STATUS(ChannelNum);

			if (b != KCS_READ_BYTE)
			{
				KCS_DBG("KCS_PHASE_READ : Set Error State TODO ~~~\n");
				write_kcs_status(ast_kcs, IPMI_KCS_READ_STATE);
				write_kcs_data(ast_kcs, 0);
				//SA Set OBF Byte
				break;
			}

			/* If we are finished transmitting, send the dummy byte */
			if (ast_kcs->KCSSendPktIx == ast_kcs->KCSSendPktLen)
			{
				KCS_DBG("KCS_PHASE_READ : finished transmitting\n");
				ast_kcs->KCSPhase = KCS_PHASE_IDLE;
				write_kcs_data(ast_kcs, 0);
				//SA Set OBF Byte

				/* Set Transmission Complete */
				break;
			}
			/* Transmit the next byte from the send buffer */
			KCS_DBG("send idx %d : %x \n",ast_kcs->KCSSendPktIx, ast_kcs->pKCSSendPkt[ast_kcs->KCSSendPktIx]);
			write_kcs_data(ast_kcs, ast_kcs->pKCSSendPkt[ast_kcs->KCSSendPktIx]);
			ast_kcs->KCSSendPktIx++;

			break;
			
		case KCS_PHASE_ERROR1:
			KCS_DBG("KCS_PHASE_ERROR1 \n");
			/* Set the KCS State to READ_STATE */
			write_kcs_status(ast_kcs, IPMI_KCS_READ_STATE);
			/* Read the Dummy byte  */
			read_kcs_data(ast_kcs);
			/* Write the error code to Data out register */
			printk("TODO ...... \n");
			write_kcs_data(ast_kcs, KCS_ABORTED_BY_COMMAND);

			//SA Set OBF Byte

			/* Set the abort phase to be error2 */
			ast_kcs->KCSPhase = KCS_PHASE_ERROR2;
	//		pKCSBuf->AbortPhase = ABORT_PHASE_ERROR2;
			break;
		case KCS_PHASE_ERROR2:
			KCS_DBG("ABORT_PHASE_ERROR2 \n");
			/**  * The system software has read the error code. Go to idle  * state. 	**/
			write_kcs_status(ast_kcs, IPMI_KCS_IDLE_STATE);

			/* Read the Dummy byte  */
			read_kcs_data(ast_kcs);

			ast_kcs->KCSPhase = KCS_PHASE_IDLE;
	//		pKCSBuf->AbortPhase = 0;
			/* Send the dummy byte  */
			write_kcs_data(ast_kcs, 0);			
			break;
		default:
			KCS_DBG("rx default == > TODO .. \n");		
			/* Read the Dummy byte  */
			read_kcs_data(ast_kcs);
			break;
	}
}

static void ast_ipmi_kcs_cmd_dat(struct ast_kcs_data *ast_kcs)
{
	u8 cmd;

	if((ast_kcs->KCSPhase == KCS_PHASE_IDLE) || (ast_kcs->KCSPhase == KCS_PHASE_WRITE)) {
		/* Set the status to WRITE_STATE */	
		write_kcs_status(ast_kcs, IPMI_KCS_WRITE_STATE);
	} else if(ast_kcs->KCSPhase == KCS_PHASE_READ) {
		//recovery 
		cmd = read_kcs_cmd(ast_kcs);
//		printk("Err STAT %d, cmd %x \n", ast_kcs->KCSPhase, cmd);		
		write_kcs_status(ast_kcs, IPMI_KCS_ERROR_STATE);
		ast_kcs->KCSPktRdy = 0;
		ast_kcs->KCSPhase = KCS_PHASE_IDLE;
		write_kcs_status(ast_kcs, IPMI_KCS_IDLE_STATE);
		return;
	} else {
		printk("Err STAT %x \n", ast_kcs->KCSPhase );
		ast_kcs->KCSPktRdy = 0;	
		write_kcs_status(ast_kcs, IPMI_KCS_ERROR_STATE);		
	}
	
	/* Read the command */
	cmd = read_kcs_cmd(ast_kcs);
	switch (cmd) {
		case KCS_WRITE_START:
			KCS_DBG("KCS_WRITE_START \n");
			/* Set the Index to 0 */
			ast_kcs->KCSRcvPktIx = 0;
			/* Set the phase to WRITE */
			ast_kcs->KCSPhase = KCS_PHASE_WRITE;
			break;
		case KCS_WRITE_END:
			/* Set the phase to write end */
			KCS_DBG("KCS_WRITE_END \n");
			ast_kcs->KCSPhase = KCS_PHASE_WRITE_END;
//			mod_timer(&ast_kcs->kcs_timer, jiffies + 5 * HZ);
			break;
	        case KCS_GET_STATUS_ABORT:
			/* Set the error code */
			KCS_DBG("KCS_GET_STATUS_ABORT TODO ...\n");
//			ast_kcs->KCSError = KCS_ABORTED_BY_COMMAND;

			/* Set the phase to write end */
			/* Set the abort phase to be error1 */
			ast_kcs->KCSPhase = KCS_PHASE_ERROR1;
			/* Send the dummy byte  */
			write_kcs_data(ast_kcs, 0);
			break;
		default:
			KCS_DBG("undefine cmd %x \n", cmd);
			/* Invalid command code - Set an error state */
			write_kcs_status(ast_kcs, IPMI_KCS_ERROR_STATE);
			/* Set the phase to error phase */
			ast_kcs->KCSPhase = KCS_PHASE_ERROR;
			break;
	}
}

static void ast_ipmi_kcs_handle(void *data)
{
	struct ast_kcs_data *ast_kcs = (struct ast_kcs_data *)data;
	u32 str= read_kcs_status(ast_kcs) & (KCS_CMD_DAT | KCS_IBF | KCS_OBF);
	switch(str) {
		case (KCS_CMD_DAT | KCS_IBF):
			KCS_DBG("%d-LPC_STR_CMD_DAT | LPC_STR_IBF \n", ast_kcs->pdev->id);
			ast_ipmi_kcs_cmd_dat(ast_kcs);
			break;
		case KCS_OBF:
			KCS_DBG("%d-LPC_STR_OBF ~~~~TOTO \n", ast_kcs->pdev->id);
			ast_ipmi_kcs_cmd_dat(ast_kcs);
			break;
		case KCS_IBF:
			KCS_DBG("%d-LPC_STR_IBF \n", ast_kcs->pdev->id);
//			mod_timer(&ast_kcs->kcs_timer, jiffies + 5 * HZ);
			ast_ipmi_kcs_rx(ast_kcs);
			break;
		default:
			printk("%d-ERROR TODO \n", ast_kcs->pdev->id);
			break;
	}
}
/***********************************************************************/
static void ast_kcs_timeout(unsigned long data)
{
	struct ast_kcs_data *ast_kcs = (struct ast_kcs_data *) data;
	printk("kcs_timout \n");
	write_kcs_status(ast_kcs, IPMI_KCS_ERROR_STATE);
	ast_kcs->KCSPktRdy = 0;
	ast_kcs->KCSPhase = KCS_PHASE_ERROR;
	
}

static int ast_kcs_probe(struct platform_device *pdev)
{
	int ret=0;
	char kcs_name[10];
	struct ast_kcs_data *ast_kcs = register_ipmi_kcs_drv(pdev->id);

	KCS_DBG("ast_kcs_probe\n");	

	ast_kcs->pKCSRcvPkt = kmalloc(AST_IPMI_PKT_SIZE * 2, GFP_KERNEL);
	ast_kcs->KCSRcvPktIx = 0;	
	ast_kcs->KCSPktRdy = 0;
	ast_kcs->pKCSSendPkt = ast_kcs->pKCSRcvPkt + AST_IPMI_PKT_SIZE;
	ast_kcs->KCSPhase = KCS_PHASE_IDLE;

#if 0
	init_timer(&(ast_kcs->kcs_timer));
//	ast_kcs->kcs_timer.data = (unsigned long) ast_kcs;
//	ast_kcs->kcs_timer.function = ast_kcs_timeout;
//	ast_kcs->kcs_timer.expires = jiffies + 5 * HZ;	//5 second
#endif 

	ret = sysfs_create_group(&pdev->dev.kobj, &ipmi_kcs_attribute_group);
	if (ret)
		goto err_free_mem;

	ast_kcs->pdev = pdev;
	ast_kcs->miscdev.minor = MISC_DYNAMIC_MINOR;

	snprintf(kcs_name, sizeof(kcs_name), "ast-kcs.%d", pdev->id);

	ast_kcs->miscdev.fops = &ast_kcs_fops;
	ast_kcs->miscdev.parent = &pdev->dev;
	ast_kcs->miscdev.name = kcs_name;
	ret = misc_register(&ast_kcs->miscdev);
	if (ret){		
		printk(KERN_ERR "KCS : failed to request interrupt\n");
		goto err_free_mem;
	}

	request_ipmi_kcs_irq(pdev->id, ast_ipmi_kcs_handle);

	platform_set_drvdata(pdev, ast_kcs);		
	
	dev_set_drvdata(ast_kcs->miscdev.this_device, ast_kcs);	

	printk(KERN_INFO "ast_kcs.%d: driver successfully loaded.\n",pdev->id);

	return 0;


err_free_mem:
	kfree(ast_kcs->pKCSRcvPkt);

	printk(KERN_WARNING "ast_kcs: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_kcs_remove(struct platform_device *pdev)
{
	struct ast_kcs_data *ast_kcs = platform_get_drvdata(pdev);
	KCS_DBG("ast_kcs_remove\n");	
	misc_deregister(&ast_kcs->miscdev);
	ast_kcs->kcs_reg = 0;
	kfree(ast_kcs->pKCSRcvPkt);

	return 0;	
}

static struct platform_driver ast_kcs_driver = {
	.remove 		= ast_kcs_remove,
	.driver         = {
		.name   = "ast-kcs",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_kcs_driver, ast_kcs_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST KCS driver");
MODULE_LICENSE("GPL");
