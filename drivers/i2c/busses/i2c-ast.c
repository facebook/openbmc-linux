/*
 *  i2c_adap_ast.c
 *
 *  I2C adapter for the ASPEED I2C bus access.
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2012.07.26: Initial version [Ryan Chen]
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/slab.h>

#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <linux/dma-mapping.h>

#include <asm/irq.h>
#include <asm/io.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/arch/regs-iic.h>
#include <asm/arch/ast_i2c.h>
#else
#include <plat/regs-iic.h>
#include <plat/ast_i2c.h>
#endif

//AST2400 buffer mode issue , force I2C slave write use byte mode , read use buffer mode
/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
//static int clock = 100; /* Default: 100 kHz */


/***************************************************************************/

#ifdef CONFIG_AST_I2C_SLAVE_RDWR
#define I2C_S_BUF_SIZE 		256
#define I2C_S_RX_BUF_NUM 		20
#define BUFF_FULL		0xff00
#define BUFF_ONGOING	1
#endif

#define AST_LOCKUP_DETECTED (0x1 << 15)

// Enable SCL/SDA pull LOW detection for Yosemite platform
#ifdef CONFIG_YOSEMITE
#define AST_I2C_LOW_TIMEOUT 0x07
#else
#define AST_I2C_LOW_TIMEOUT 0x00
#endif //CONFIG_YOSEMITE


struct ast_i2c_dev {
	struct ast_i2c_driver_data *ast_i2c_data;
	struct device		*dev;
	void __iomem		*reg_base;			/* virtual */
	int 				irq;				//I2C IRQ number
	u32					bus_id;				//for i2c dev# IRQ number check
	u32					state;				//I2C xfer mode state matchine
	struct i2c_adapter	adap;
	struct buf_page		*req_page;
//dma or buff mode needed
	unsigned char		*dma_buf;
	dma_addr_t			dma_addr;

//master
	int					xfer_last;			//cur xfer is last msgs for stop msgs
	struct i2c_msg 		*master_msgs;		//cur xfer msgs
	int					master_xfer_len;			//cur xfer len
	int					master_xfer_cnt;			//total xfer count
	u32					master_xfer_mode;			//cur xfer mode ... 0 : no_op , master: 1 byte , 2 : buffer , 3: dma , slave : xxxx
	struct completion	cmd_complete;
	int					cmd_err;
	u8 					blk_r_flag; 		//for smbus block read
	void 				(*do_master_xfer)(struct ast_i2c_dev *i2c_dev);
  spinlock_t  master_lock;
//Slave structure
	u8					slave_operation;
	u8					slave_event;
	struct i2c_msg		*slave_msgs; 		//cur slave xfer msgs
	int 				slave_xfer_len;
	int 				slave_xfer_cnt;
	u32					slave_xfer_mode;			//cur xfer mode ... 0 : no_op , master: 1 byte , 2 : buffer , 3: dma , slave : xxxx
	void				(*do_slave_xfer)(struct ast_i2c_dev *i2c_dev);
#ifdef CONFIG_AST_I2C_SLAVE_RDWR
  struct i2c_msg		slave_rx_msg[I2C_S_RX_BUF_NUM + 1];
  struct i2c_msg		slave_tx_msg;
  spinlock_t	slave_rx_lock;
#endif
};



static inline void
ast_i2c_write(struct ast_i2c_dev *i2c_dev, u32 val, u32 reg)
{
//	dev_dbg(i2c_dev->dev, "ast_i2c_write : val: %x , reg : %x \n",val,reg);
	writel(val, i2c_dev->reg_base+ reg);
}

static inline u32
ast_i2c_read(struct ast_i2c_dev *i2c_dev, u32 reg)
{
#if 0
	u32 val = readl(i2c_dev->reg_base + reg);
	printk("R : reg %x , val: %x \n",reg, val);
	return val;
#else
	return readl(i2c_dev->reg_base + reg);
#endif
}

static u32 select_i2c_clock(struct ast_i2c_dev *i2c_dev)
{

	unsigned int clk, inc = 0, div, divider_ratio;
	u32 SCL_Low, SCL_High, data;

  // hack: The calculated value for 1MHz does not match with measured value, so override
  if (i2c_dev->ast_i2c_data->bus_clk == 1000000) {
    data = 0x77744302;
    return data;
  }

	clk = i2c_dev->ast_i2c_data->get_i2c_clock();
//	printk("pclk = %d \n",clk);
	divider_ratio = clk / i2c_dev->ast_i2c_data->bus_clk;
	for (div = 0; divider_ratio >= 16; div++)
	{
		inc |= (divider_ratio & 1);
		divider_ratio >>= 1;
	}
	divider_ratio += inc;
	SCL_Low = (divider_ratio >> 1) - 1;
	SCL_High = divider_ratio - SCL_Low - 2;
	data = 0x77700300 | (SCL_High << 16) | (SCL_Low << 12) | div;
//	printk("I2CD04 for %d = %08X\n", target_speed, data);
	return data;
}

#ifdef CONFIG_AST_I2C_SLAVE_MODE
/* AST I2C Slave mode  */
static void ast_slave_issue_alert(struct ast_i2c_dev *i2c_dev, u8 enable)
{
	//only support dev0~3
	if(i2c_dev->bus_id > 3)
		return;
	else {
		if(enable)
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_CMD_REG) | AST_I2CD_S_ALT_EN, I2C_CMD_REG);
		else
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_CMD_REG) & ~AST_I2CD_S_ALT_EN, I2C_CMD_REG);
	}
}

static void ast_slave_mode_enable(struct ast_i2c_dev *i2c_dev, struct i2c_msg *msgs)
{
	if(msgs->buf[0] == 1) {
		ast_i2c_write(i2c_dev, msgs->addr, I2C_DEV_ADDR_REG);
		ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
	} else
		ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_FUN_CTRL_REG) & ~AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
}

#endif

static void ast_i2c_dev_init(struct ast_i2c_dev *i2c_dev)
{
	//I2CG Reset
	ast_i2c_write(i2c_dev, 0, I2C_FUN_CTRL_REG);

#ifdef CONFIG_AST_I2C_SLAVE_EEPROM
	i2c_dev->ast_i2c_data->slave_init(&(i2c_dev->slave_msgs));
	ast_slave_mode_enable(i2c_dev, i2c_dev->slave_msgs);
#elif defined(CONFIG_AST_I2C_SLAVE_RDWR)
	i2c_dev->slave_msgs = i2c_dev->slave_rx_msg;
#else
	i2c_dev->slave_msgs = NULL;
#endif

	//Enable Master Mode
	ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);


	/* Set AC Timing */
#if defined(CONFIG_ARCH_AST2400)
	if(i2c_dev->ast_i2c_data->bus_clk/1000 > 400) {
		printk("high speed mode enable clk [%dkhz]\n",i2c_dev->ast_i2c_data->bus_clk/1000);		
		ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) |
							AST_I2CD_M_HIGH_SPEED_EN |
							AST_I2CD_M_SDA_DRIVE_1T_EN |
							AST_I2CD_SDA_DRIVE_1T_EN 
							, I2C_FUN_CTRL_REG);
		
		/* Set AC Timing */
		ast_i2c_write(i2c_dev, 0x3, I2C_AC_TIMING_REG2);
		ast_i2c_write(i2c_dev, select_i2c_clock(i2c_dev), I2C_AC_TIMING_REG1);		
	}else {
		/* target apeed is xxKhz*/	
		ast_i2c_write(i2c_dev, select_i2c_clock(i2c_dev), I2C_AC_TIMING_REG1);
		ast_i2c_write(i2c_dev, AST_NO_TIMEOUT_CTRL, I2C_AC_TIMING_REG2);	
	}
#else
	/* target apeed is xxKhz*/
	ast_i2c_write(i2c_dev, select_i2c_clock(i2c_dev), I2C_AC_TIMING_REG1);
	ast_i2c_write(i2c_dev, AST_NO_TIMEOUT_CTRL, I2C_AC_TIMING_REG2);
#endif
//	ast_i2c_write(i2c_dev, 0x77743335, I2C_AC_TIMING_REG1);
/////


	//Clear Interrupt
	ast_i2c_write(i2c_dev, 0xfffffff, I2C_INTR_STS_REG);

	//TODO
//	ast_i2c_write(i2c_dev, 0xAF, I2C_INTR_CTRL_REG);
	//Enable Interrupt, STOP Interrupt has bug in AST2000

	/* Set interrupt generation of I2C controller */
	ast_i2c_write(i2c_dev,
				AST_I2CD_SDA_DL_TO_INTR_EN |
				AST_I2CD_BUS_RECOVER_INTR_EN |
				AST_I2CD_SMBUS_ALT_INTR_EN |
//				AST_I2CD_SLAVE_MATCH_INTR_EN |
				AST_I2CD_SCL_TO_INTR_EN |
				AST_I2CD_ABNORMAL_INTR_EN |
				AST_I2CD_NORMAL_STOP_INTR_EN |
				AST_I2CD_ARBIT_LOSS_INTR_EN |
				AST_I2CD_RX_DOWN_INTR_EN |
				AST_I2CD_TX_NAK_INTR_EN |
				AST_I2CD_TX_ACK_INTR_EN,
				I2C_INTR_CTRL_REG);

				// Initialize completion structure
				init_completion(&i2c_dev->cmd_complete);
}

#ifdef CONFIG_AST_I2C_SLAVE_RDWR
//for memory buffer initial
static void ast_i2c_slave_buff_init(struct ast_i2c_dev *i2c_dev)
{
	int i;
	//Tx buf  1
	i2c_dev->slave_tx_msg.len = I2C_S_BUF_SIZE;
	i2c_dev->slave_tx_msg.buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	//Rx buf 4
	for(i=0; i<I2C_S_RX_BUF_NUM+1; i++) {
		i2c_dev->slave_rx_msg[i].addr = ~BUFF_ONGOING;
		i2c_dev->slave_rx_msg[i].flags = 0;	//mean empty buffer
		i2c_dev->slave_rx_msg[i].len = I2C_S_BUF_SIZE;
		i2c_dev->slave_rx_msg[i].buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	}
}

static void ast_i2c_slave_rdwr_xfer(struct ast_i2c_dev *i2c_dev)
{
	int i;
  int count = 0;
	unsigned long flags;

	spin_lock_irqsave(&i2c_dev->slave_rx_lock, flags);

	switch(i2c_dev->slave_event) {
		case I2C_SLAVE_EVENT_START_WRITE:
			for(i=0; i<I2C_S_RX_BUF_NUM; i++) {
				if((i2c_dev->slave_rx_msg[i].flags == 0) && (i2c_dev->slave_rx_msg[i].addr != BUFF_ONGOING)) {
					i2c_dev->slave_rx_msg[i].addr = BUFF_ONGOING;
					break;
				}
			}
			if(i == I2C_S_RX_BUF_NUM) {
        // dev_err(i2c_dev->dev, "RX buffer full ........use tmp msgs buff \n");
        for(i=0; i<I2C_S_RX_BUF_NUM; i++) {
          if((i2c_dev->slave_rx_msg[i].flags == 0) && (i2c_dev->slave_rx_msg[i].addr == BUFF_ONGOING)) {
            count++;
            i2c_dev->slave_rx_msg[i].addr = 0;
          }
        }

        if (count) {
          dev_err(i2c_dev->dev, "Cleared slave ongoing buffers of count: %d\n", count);
        }

        for(i=0; i<I2C_S_RX_BUF_NUM; i++) {
          if((i2c_dev->slave_rx_msg[i].flags == 0) && (i2c_dev->slave_rx_msg[i].addr != BUFF_ONGOING)) {
            i2c_dev->slave_rx_msg[i].addr = BUFF_ONGOING;
            break;
          }
        }
			}
			//printk("I2C_SLAVE_EVENT_START_WRITE ... %d \n", i);

			i2c_dev->slave_msgs = &i2c_dev->slave_rx_msg[i];
			break;
		case I2C_SLAVE_EVENT_START_READ:
			// printk("I2C_SLAVE_EVENT_START_READ ERROR .. not imple \n");
			i2c_dev->slave_msgs = &i2c_dev->slave_tx_msg;
			break;
		case I2C_SLAVE_EVENT_WRITE:
			//printk("I2C_SLAVE_EVENT_WRITE next write ERROR ...\n");
			i2c_dev->slave_msgs = &i2c_dev->slave_tx_msg;
			break;
		case I2C_SLAVE_EVENT_READ:
			printk("I2C_SLAVE_EVENT_READ ERROR ... \n");
			i2c_dev->slave_msgs = &i2c_dev->slave_tx_msg;
			break;
		case I2C_SLAVE_EVENT_NACK:
			//printk("I2C_SLAVE_EVENT_NACK ERROR ... \n");
			i2c_dev->slave_msgs = &i2c_dev->slave_tx_msg;
			break;
		case I2C_SLAVE_EVENT_STOP:
			//printk("I2C_SLAVE_EVENT_STOP \n");
			for(i=0; i<I2C_S_RX_BUF_NUM; i++) {
				if(i2c_dev->slave_rx_msg[i].addr == BUFF_ONGOING) {
					i2c_dev->slave_rx_msg[i].flags = BUFF_FULL;
					i2c_dev->slave_rx_msg[i].addr = 0;
					break;
				}
			}

			i2c_dev->slave_msgs = &i2c_dev->slave_tx_msg;
			break;
	}
	spin_unlock_irqrestore(&i2c_dev->slave_rx_lock, flags);

}

static int ast_i2c_slave_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs)
{
	struct ast_i2c_dev *i2c_dev = adap->algo_data;
	int ret=0, i;
	unsigned long flags;

	switch(msgs->flags) {
		case 0:
//			printk("slave read \n");
			//cur_msg = get_free_msg;
			spin_lock_irqsave(&i2c_dev->slave_rx_lock, flags);

			for(i=0; i<I2C_S_RX_BUF_NUM; i++) {
				if((i2c_dev->slave_rx_msg[i].addr == 0) && (i2c_dev->slave_rx_msg[i].flags == BUFF_FULL)) {
					memcpy(msgs->buf, i2c_dev->slave_rx_msg[i].buf, i2c_dev->slave_rx_msg[i].len);
					msgs->len = i2c_dev->slave_rx_msg[i].len;
					i2c_dev->slave_rx_msg[i].flags = 0;
					i2c_dev->slave_rx_msg[i].len = 0;
					break;
				}
			}
			spin_unlock_irqrestore(&i2c_dev->slave_rx_lock, flags);

			if(i == I2C_S_RX_BUF_NUM) {
				//printk("No buffer ........ \n");
				msgs->len = 0;
				ret = -1;
			}
			break;
		case I2C_M_RD:	//slave write
//			printk("slave write \n");
			memcpy(msgs->buf, i2c_dev->slave_tx_msg.buf, I2C_S_BUF_SIZE);
			break;
		case I2C_S_EN:
			if((msgs->addr < 0x1) || (msgs->addr > 0xff)) {
				ret = -1;
				printk("addrsss not correct !! \n");
				return ret;
			}
			if(msgs->len != 1) printk("ERROR \n");
			ast_slave_mode_enable(i2c_dev, msgs);
			break;
		case I2C_S_ALT:
//			printk("slave issue alt\n");
			if(msgs->len != 1) printk("ERROR \n");
			if(msgs->buf[0]==1)
				ast_slave_issue_alert(i2c_dev, 1);
			else
				ast_slave_issue_alert(i2c_dev, 0);
			break;

		default:
			printk("slave xfer error \n");
			break;

	}
	return ret;
}


#endif

static u8
ast_i2c_bus_reset(struct ast_i2c_dev *i2c_dev)
{
  u32 temp;

  // Reset i2c controller
  temp = ast_i2c_read(i2c_dev,I2C_FUN_CTRL_REG);

  ast_i2c_write(i2c_dev, temp & ~(AST_I2CD_SLAVE_EN | AST_I2CD_MASTER_EN), I2C_FUN_CTRL_REG);

  ast_i2c_write(i2c_dev, temp, I2C_FUN_CTRL_REG);
}

static u8
ast_i2c_bus_error_recover(struct ast_i2c_dev *i2c_dev)
{
	u32 sts;
	int r;
	u32 i = 0;

	//Check 0x14's SDA and SCL status
	sts = ast_i2c_read(i2c_dev,I2C_CMD_REG);

	if ((sts & AST_I2CD_SDA_LINE_STS) && (sts & AST_I2CD_SCL_LINE_STS)) {
		//Means bus is idle.
		dev_err(i2c_dev->dev, "I2C bus (%d) is idle. I2C slave doesn't exist?!\n", i2c_dev->bus_id);
		return -1;
	}

	dev_err(i2c_dev->dev, "ERROR!! I2C(%d) bus hanged, try to recovery it!\n", i2c_dev->bus_id);


	if ((sts & AST_I2CD_SDA_LINE_STS) && !(sts & AST_I2CD_SCL_LINE_STS)) {
		//if SDA == 1 and SCL == 0, it means the master is locking the bus.
		//Send a stop command to unlock the bus.
		dev_err(i2c_dev->dev, "I2C's master is locking the bus, try to stop it.\n");
//
		init_completion(&i2c_dev->cmd_complete);
		i2c_dev->cmd_err = 0;

		ast_i2c_write(i2c_dev, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);

		r = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
													   i2c_dev->adap.timeout*HZ);

		if(i2c_dev->cmd_err &&
		   i2c_dev->cmd_err != AST_I2CD_INTR_STS_NORMAL_STOP) {
			dev_err(i2c_dev->dev, "recovery error \n");
			return -1;
		}

		if (r == 0) {
			 dev_err(i2c_dev->dev, "recovery timed out\n");
			 return -1;
		} else {
			dev_err(i2c_dev->dev, "Recovery successfully\n");
			return 0;
		}


	} else if (!(sts & AST_I2CD_SDA_LINE_STS)) {
		//else if SDA == 0, the device is dead. We need to reset the bus
		//And do the recovery command.
		dev_err(i2c_dev->dev, "I2C's slave is dead, try to recover it\n");
		//Let's retry 10 times
		for (i = 0; i < 10; i++) {
			ast_i2c_dev_init(i2c_dev);
			//Do the recovery command BIT11
			init_completion(&i2c_dev->cmd_complete);
      i2c_dev->cmd_err = 0;
			ast_i2c_write(i2c_dev, AST_I2CD_BUS_RECOVER_CMD_EN, I2C_CMD_REG);

			r = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
														   i2c_dev->adap.timeout*HZ);
			if (i2c_dev->cmd_err != 0 &&
			   i2c_dev->cmd_err != AST_I2CD_INTR_STS_NORMAL_STOP) {
				dev_err(i2c_dev->dev, "ERROR!! Failed to do recovery command(0x%08x)\n", i2c_dev->cmd_err);
				return -1;
			}
			//Check 0x14's SDA and SCL status
			sts = ast_i2c_read(i2c_dev,I2C_CMD_REG);
			if (sts & AST_I2CD_SDA_LINE_STS) //Recover OK
				break;
		}
		if (i == 10) {
			dev_err(i2c_dev->dev, "ERROR!! recover failed\n");
			return -1;
		}
	} else {
		dev_err(i2c_dev->dev, "Don't know how to handle this case?!\n");
		return -1;
	}
  dev_err(i2c_dev->dev, "Recovery successfully\n");
	return 0;
}

static void ast_master_alert_recv(struct ast_i2c_dev *i2c_dev)
{
	printk("ast_master_alert_recv bus id %d, Disable Alt, Please Imple \n",i2c_dev->bus_id);
}

static int ast_i2c_wait_bus_not_busy(struct ast_i2c_dev *i2c_dev)
{
	int timeout = 10; //TODO number
  volatile u8 mode = 0;
//	printk("ast_i2c_wait_bus_not_busy \n");

  // Wait for slave transfer to finish
  mode = i2c_dev->slave_operation;
  while (mode == 1) {
    if (timeout <= 0) {
      break;
    }
    mode = i2c_dev->slave_operation;
    timeout--;
    msleep(1);
  }

  if (timeout <= 0) {
    //TODO: sometimes the slave operation flag is not reset properly so go ahead with checking bus busy signal
    dev_err(i2c_dev->dev, "slave operation set, check busy status line\n");
  }

  // Wait for Bus to go IDLE
  timeout = 10;
	while (ast_i2c_read(i2c_dev,I2C_CMD_REG) & AST_I2CD_BUS_BUSY_STS) {
		if(timeout<=0) {
			break;
    }

		timeout--;
		msleep(1);
	}

  if (timeout <=0) {
    //ast_i2c_bus_error_recover(i2c_dev);
    ast_i2c_bus_reset(i2c_dev);
    return 0;
  }

  // TODO: hack to reset slave operation flag manually
  i2c_dev->slave_operation = 0;
  return 0;
}

//ast1070, ast1010 dma
static void ast_i2c_do_dec_dma_xfer(struct ast_i2c_dev *i2c_dev)	
{
	u32 cmd = 0;
	int i;

	i2c_dev->master_xfer_mode = DEC_DMA_XFER;
	i2c_dev->slave_xfer_mode = DEC_DMA_XFER;
	dev_dbg(i2c_dev->dev, "ast_i2c_do_dec_dma_xfer \n");
	if(i2c_dev->slave_operation == 1) { 
		if(i2c_dev->slave_msgs->flags & I2C_M_RD) {
			//DMA tx mode
			if(i2c_dev->slave_msgs->len > AST_I2C_DMA_SIZE)
				i2c_dev->slave_xfer_len = AST_I2C_DMA_SIZE;
			else 
				i2c_dev->slave_xfer_len = i2c_dev->slave_msgs->len;
			
			dev_dbg(i2c_dev->dev, "(<--) slave tx DMA \n");
			for(i=0; i<i2c_dev->slave_xfer_len; i++)
				i2c_dev->dma_buf[i] = i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt + i];
			
			ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
			ast_i2c_write(i2c_dev, AST_I2C_DMA_SIZE-1, I2C_DMA_LEN_REG);
			ast_i2c_write(i2c_dev, AST_I2CD_TX_DMA_ENABLE | AST_I2CD_S_TX_CMD,I2C_CMD_REG);	
		} else {
			//DMA prepare rx
			dev_dbg(i2c_dev->dev, "(-->) slave rx DMA \n");
			ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
			ast_i2c_write(i2c_dev, (AST_I2C_DMA_SIZE-1), I2C_DMA_LEN_REG);
			ast_i2c_write(i2c_dev, AST_I2CD_RX_DMA_ENABLE, I2C_CMD_REG);		
		}
	} else {
		dev_dbg(i2c_dev->dev,"M cnt %d, xf len %d \n",i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);
		if(i2c_dev->master_xfer_cnt == -1) {
			//send start 
			dev_dbg(i2c_dev->dev, " %sing %d byte%s %s 0x%02x\n",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "read" : "write",
							i2c_dev->master_msgs->len, i2c_dev->master_msgs->len > 1 ? "s" : "",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "from" : "to", i2c_dev->master_msgs->addr);

			if(i2c_dev->master_msgs->flags & I2C_M_RD) {
				//workaround .. HW can;t send start read addr with buff mode 
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD;
				ast_i2c_write(i2c_dev, (i2c_dev->master_msgs->addr <<1) |0x1, I2C_BYTE_BUF_REG);
//				tx_buf[0] = (i2c_dev->master_msgs->addr <<1); //+1
				i2c_dev->master_xfer_len = 1;
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
									AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);	
			} else {
				//tx
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD | AST_I2CD_TX_DMA_ENABLE;		

				i2c_dev->dma_buf[0] = (i2c_dev->master_msgs->addr <<1);	//+1
				//next data write
				if((i2c_dev->master_msgs->len + 1) > AST_I2C_DMA_SIZE)
					i2c_dev->master_xfer_len = AST_I2C_DMA_SIZE;
				else
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len + 1;
				
				for(i = 1; i < i2c_dev->master_xfer_len; i++)
					i2c_dev->dma_buf[i] = i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt+i];
				
				if (i2c_dev->xfer_last == 1) {
					dev_dbg(i2c_dev->dev, "last stop \n");
					cmd |= AST_I2CD_M_STOP_CMD;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
										~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					dev_dbg(i2c_dev->dev, "intr en %x \n",ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG));
				} else {
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
				}
				ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
				ast_i2c_write(i2c_dev, (i2c_dev->master_xfer_len-1), I2C_DMA_LEN_REG);
			}
			ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);			
			dev_dbg(i2c_dev->dev, "txfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);

		} else if (i2c_dev->master_xfer_cnt < i2c_dev->master_msgs->len){
			//Next send 
			if(i2c_dev->master_msgs->flags & I2C_M_RD) {
				//Rx data
				cmd = AST_I2CD_M_RX_CMD | AST_I2CD_RX_DMA_ENABLE;
															
				if((i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt) > AST_I2C_DMA_SIZE) {
					i2c_dev->master_xfer_len = AST_I2C_DMA_SIZE;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
					
				} else {
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt;
					if((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->blk_r_flag == 0)) {
						dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN \n");
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);						
					} else {
#ifdef CONFIG_AST1010
						//Workaround for ast1010 can't send NACK
						if((i2c_dev->master_xfer_len == 1) && (i2c_dev->xfer_last == 1)) {
							//change to byte mode
							cmd |= AST_I2CD_M_STOP_CMD | AST_I2CD_M_S_RX_CMD_LAST;
							cmd &= ~AST_I2CD_RX_DMA_ENABLE;
							i2c_dev->master_xfer_mode = BYTE_XFER;
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
												~AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
							
						} else if (i2c_dev->master_xfer_len > 1) {
							i2c_dev->master_xfer_len -=1;
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
						} else {
							printk(" Fix Me !! \n");
						}
#else
						if(i2c_dev->xfer_last == 1) {
							dev_dbg(i2c_dev->dev, "last stop \n");							
							cmd |= AST_I2CD_M_STOP_CMD; 
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
											~AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
							dev_dbg(i2c_dev->dev, "intr en %x \n",ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG));
						} else {
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
						}
						//TODO check....
						cmd |= AST_I2CD_M_S_RX_CMD_LAST;
#endif						
					}
					
				}
				ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
				ast_i2c_write(i2c_dev, i2c_dev->master_xfer_len-1, I2C_DMA_LEN_REG);
				ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);
				dev_dbg(i2c_dev->dev, "rxfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);
			} else {
				//Tx data
				//next data write
				cmd = AST_I2CD_M_TX_CMD | AST_I2CD_TX_DMA_ENABLE;
				if((i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt) > AST_I2C_DMA_SIZE) {
					i2c_dev->master_xfer_len = AST_I2C_DMA_SIZE;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);			
				
				} else {
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt;
					if(i2c_dev->xfer_last == 1) {
						dev_dbg(i2c_dev->dev, "last stop \n");
						cmd |= AST_I2CD_M_STOP_CMD;
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
											~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
						dev_dbg(i2c_dev->dev, "intr en %x \n",ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG));
					} else {
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);			
					}
				}

				for(i = 0; i < i2c_dev->master_xfer_len; i++) 
					i2c_dev->dma_buf[i] = i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i];

				ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
				ast_i2c_write(i2c_dev, (i2c_dev->master_xfer_len-1), I2C_DMA_LEN_REG);
				ast_i2c_write(i2c_dev, cmd , I2C_CMD_REG);
				dev_dbg(i2c_dev->dev, "txfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);

			}		
		}else {
			//should send next msg 
			 if(i2c_dev->master_xfer_cnt != i2c_dev->master_msgs->len)
					 printk("complete rx ... ERROR \n");
			
			 dev_dbg(i2c_dev->dev, "ast_i2c_do_byte_xfer complete \n");
			 i2c_dev->cmd_err = 0;
			 complete(&i2c_dev->cmd_complete);		
		}
	
	}

	
}

static void ast_i2c_do_inc_dma_xfer(struct ast_i2c_dev *i2c_dev)	
{
	u32 cmd = 0;
	int i;

	i2c_dev->master_xfer_mode = INC_DMA_XFER;
	i2c_dev->slave_xfer_mode = INC_DMA_XFER;
	dev_dbg(i2c_dev->dev, "ast_i2c_do_inc_dma_xfer \n");
	if(i2c_dev->slave_operation == 1) { 
		dev_dbg(i2c_dev->dev,"S cnt %d, xf len %d \n",i2c_dev->slave_xfer_cnt, i2c_dev->slave_msgs->len);
		if(i2c_dev->slave_msgs->flags & I2C_M_RD) {
			//DMA tx mode
			if(i2c_dev->slave_msgs->len > AST_I2C_DMA_SIZE)
				i2c_dev->slave_xfer_len = AST_I2C_DMA_SIZE;
			else 
				i2c_dev->slave_xfer_len = i2c_dev->slave_msgs->len;
			
			dev_dbg(i2c_dev->dev, "(<--) slave tx DMA len %d \n",i2c_dev->slave_xfer_len);
			for(i=0; i<i2c_dev->slave_xfer_len; i++)
				i2c_dev->dma_buf[i] = i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt + i];
			
			ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
			ast_i2c_write(i2c_dev, i2c_dev->slave_xfer_len, I2C_DMA_LEN_REG);
			ast_i2c_write(i2c_dev, AST_I2CD_TX_DMA_ENABLE | AST_I2CD_S_TX_CMD,I2C_CMD_REG);	
		} else {
			//DMA prepare rx
			if(i2c_dev->slave_msgs->len > AST_I2C_DMA_SIZE)
				i2c_dev->slave_xfer_len = AST_I2C_DMA_SIZE;
			else 
				i2c_dev->slave_xfer_len = i2c_dev->slave_msgs->len;
		
			dev_dbg(i2c_dev->dev, "(-->) slave rx DMA len %d \n", i2c_dev->slave_xfer_len);
			ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
			ast_i2c_write(i2c_dev, i2c_dev->slave_xfer_len, I2C_DMA_LEN_REG);
			ast_i2c_write(i2c_dev, AST_I2CD_RX_DMA_ENABLE, I2C_CMD_REG);		
		}
	} else {
		dev_dbg(i2c_dev->dev,"M cnt %d, xf len %d \n",i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);
		if(i2c_dev->master_xfer_cnt == -1) {
			//send start 
			dev_dbg(i2c_dev->dev, " %sing %d byte%s %s 0x%02x\n",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "read" : "write",
							i2c_dev->master_msgs->len, i2c_dev->master_msgs->len > 1 ? "s" : "",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "from" : "to", i2c_dev->master_msgs->addr);

			if(i2c_dev->master_msgs->flags & I2C_M_RD) {
				//workaround .. HW can;t send start read addr with buff mode 
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD;
				ast_i2c_write(i2c_dev, (i2c_dev->master_msgs->addr <<1) |0x1, I2C_BYTE_BUF_REG);
//				tx_buf[0] = (i2c_dev->master_msgs->addr <<1); //+1
				i2c_dev->master_xfer_len = 1;
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
									AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);	
			} else {
				//tx
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD | AST_I2CD_TX_DMA_ENABLE;		

				i2c_dev->dma_buf[0] = (i2c_dev->master_msgs->addr <<1);	//+1
				//next data write
				if((i2c_dev->master_msgs->len + 1) > AST_I2C_DMA_SIZE)
					i2c_dev->master_xfer_len = AST_I2C_DMA_SIZE;
				else
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len + 1;
				
				for(i = 1; i < i2c_dev->master_xfer_len; i++)
					i2c_dev->dma_buf[i] = i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt+i];
				
				if (i2c_dev->xfer_last == 1) {
					dev_dbg(i2c_dev->dev, "last stop \n");
					cmd |= AST_I2CD_M_STOP_CMD;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
										~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					dev_dbg(i2c_dev->dev, "intr en %x \n",ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG));
				} else {
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
				}
				ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
				ast_i2c_write(i2c_dev, (i2c_dev->master_xfer_len), I2C_DMA_LEN_REG);
			}
			ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);			
			dev_dbg(i2c_dev->dev, "txfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);

		} else if (i2c_dev->master_xfer_cnt < i2c_dev->master_msgs->len) {
			//Next send 
			if(i2c_dev->master_msgs->flags & I2C_M_RD) {
				//Rx data
				cmd = AST_I2CD_M_RX_CMD | AST_I2CD_RX_DMA_ENABLE;
															
				if((i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt) > AST_I2C_DMA_SIZE) {
					i2c_dev->master_xfer_len = AST_I2C_DMA_SIZE;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
					
				} else {
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt;
					if((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->blk_r_flag == 0)) {
						dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN \n");
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);						
					} else {
						if(i2c_dev->xfer_last == 1) {
							dev_dbg(i2c_dev->dev, "last stop \n");							
							cmd |= AST_I2CD_M_STOP_CMD; 
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
											~AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
							dev_dbg(i2c_dev->dev, "intr en %x \n",ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG));
						} else {
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
						}
						//TODO check....
						cmd |= AST_I2CD_M_S_RX_CMD_LAST;
					}
					
				}
				ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
				ast_i2c_write(i2c_dev, (i2c_dev->master_xfer_len), I2C_DMA_LEN_REG);
				ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);
				dev_dbg(i2c_dev->dev, "rxfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);
			} else {
				//Tx data
				//next data write
				cmd = AST_I2CD_M_TX_CMD | AST_I2CD_TX_DMA_ENABLE;
				if((i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt) > AST_I2C_DMA_SIZE) {
					i2c_dev->master_xfer_len = AST_I2C_DMA_SIZE;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);			
				
				} else {
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt;
					if(i2c_dev->xfer_last == 1) {
						dev_dbg(i2c_dev->dev, "last stop \n");
						cmd |= AST_I2CD_M_STOP_CMD;
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
											~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
						dev_dbg(i2c_dev->dev, "intr en %x \n",ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG));
					} else {
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);			
					}
				}

				for(i = 0; i < i2c_dev->master_xfer_len; i++) 
					i2c_dev->dma_buf[i] = i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i];

				ast_i2c_write(i2c_dev, i2c_dev->dma_addr, I2C_DMA_BASE_REG);
				ast_i2c_write(i2c_dev, (i2c_dev->master_xfer_len), I2C_DMA_LEN_REG);
				ast_i2c_write(i2c_dev, cmd , I2C_CMD_REG);
				dev_dbg(i2c_dev->dev, "txfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);

			}		
		}else {
			//should send next msg 
			 if(i2c_dev->master_xfer_cnt != i2c_dev->master_msgs->len)
					 printk("complete rx ... bus=%d addr=0x%x (%d vs. %d) ERROR\n",
					 i2c_dev->bus_id, i2c_dev->master_msgs->addr,
					 i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);
			
			 dev_dbg(i2c_dev->dev, "ast_i2c_do_byte_xfer complete \n");
			 i2c_dev->cmd_err = 0;
			 complete(&i2c_dev->cmd_complete);		
		}
	
	}

	
}

static void ast_i2c_do_pool_xfer(struct ast_i2c_dev *i2c_dev)
{
	u32 cmd = 0;
	int i;
	u32 *tx_buf;

#if defined(AST_SOC_G4)
	ast_i2c_write(i2c_dev,
					(ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) &
					~AST_I2CD_BUFF_SEL_MASK) |
					AST_I2CD_BUFF_SEL(i2c_dev->req_page->page_no),
					I2C_FUN_CTRL_REG);
#endif

#if defined(AST_SOC_G5)
	dev_dbg(i2c_dev->dev, "offset buffer = %x \n",i2c_dev->bus_id * 0x10);

	tx_buf = (u32*)(i2c_dev->req_page->page_addr + i2c_dev->bus_id * 0x10);
#else
	tx_buf = (u32 *) i2c_dev->req_page->page_addr;
#endif

	if(i2c_dev->slave_operation == 1) {
		if(i2c_dev->slave_msgs->flags & I2C_M_RD) {
			dev_dbg(i2c_dev->dev, "(<--) slave tx buf \n");

			if(i2c_dev->slave_msgs->len > i2c_dev->req_page->page_size)
				i2c_dev->slave_xfer_len = i2c_dev->req_page->page_size;
			else
				i2c_dev->slave_xfer_len = i2c_dev->slave_msgs->len;

			for(i = 0; i< i2c_dev->slave_xfer_len; i++) {
				if(i%4 == 0)
					tx_buf[i/4] = 0;
				tx_buf[i/4] |= (i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt + i] << ((i%4)*8)) ;
				dev_dbg(i2c_dev->dev, "[%x] ",tx_buf[i/4]);
			}
			dev_dbg(i2c_dev->dev, "\n");

			ast_i2c_write(i2c_dev, AST_I2CD_TX_DATA_BUF_END_SET((i2c_dev->slave_xfer_len-1)) |
						AST_I2CD_BUF_BASE_ADDR_SET((i2c_dev->req_page->page_addr_point)),
						I2C_BUF_CTRL_REG);

			ast_i2c_write(i2c_dev, AST_I2CD_TX_BUFF_ENABLE | AST_I2CD_S_TX_CMD, I2C_CMD_REG);
		} else {
			//prepare for new rx
			dev_dbg(i2c_dev->dev, "(-->) slave prepare rx buf \n");
			ast_i2c_write(i2c_dev,
						AST_I2CD_RX_BUF_END_ADDR_SET((i2c_dev->req_page->page_size-1)) |
						AST_I2CD_BUF_BASE_ADDR_SET((i2c_dev->req_page->page_addr_point)),
						I2C_BUF_CTRL_REG);

			ast_i2c_write(i2c_dev, AST_I2CD_RX_BUFF_ENABLE, I2C_CMD_REG);

		}
	} else {
		dev_dbg(i2c_dev->dev,"M cnt %d, xf len %d \n",i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);
		if(i2c_dev->master_xfer_cnt == -1) {
			//send start
			dev_dbg(i2c_dev->dev, " %sing %d byte%s %s 0x%02x\n",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "read" : "write",
							i2c_dev->master_msgs->len, i2c_dev->master_msgs->len > 1 ? "s" : "",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "from" : "to", i2c_dev->master_msgs->addr);

			if(i2c_dev->master_msgs->flags & I2C_M_RD) {
//workaround .. HW can;t send start read addr with buff mode
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD;
				ast_i2c_write(i2c_dev, (i2c_dev->master_msgs->addr <<1) |0x1, I2C_BYTE_BUF_REG);

//				tx_buf[0] = (i2c_dev->master_msgs->addr <<1); //+1
				i2c_dev->master_xfer_len = 1;
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
									AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
			} else {
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD | AST_I2CD_TX_BUFF_ENABLE;
				tx_buf[0] = (i2c_dev->master_msgs->addr <<1);	//+1
				//next data write
				if((i2c_dev->master_msgs->len + 1) > i2c_dev->req_page->page_size)
					i2c_dev->master_xfer_len = i2c_dev->req_page->page_size;
				else
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len + 1;

				for(i = 1; i < i2c_dev->master_xfer_len; i++) {
					if(i%4 == 0)
						tx_buf[i/4] = 0;
					tx_buf[i/4] |= (i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i] << ((i%4)*8)) ;
				}

				if (i2c_dev->xfer_last == 1) {
					dev_dbg(i2c_dev->dev, "last stop \n");
					cmd |= AST_I2CD_M_STOP_CMD;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
										~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);

				} else {
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
				}
				ast_i2c_write(i2c_dev,
							AST_I2CD_TX_DATA_BUF_END_SET((i2c_dev->master_xfer_len - 1)) |
							AST_I2CD_BUF_BASE_ADDR_SET(i2c_dev->req_page->page_addr_point),
							I2C_BUF_CTRL_REG);
			}
			ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);
			dev_dbg(i2c_dev->dev, "txfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);

		} else if (i2c_dev->master_xfer_cnt < i2c_dev->master_msgs->len){
			//Next send
			if(i2c_dev->master_msgs->flags & I2C_M_RD) {
				//Rx data
				cmd = AST_I2CD_M_RX_CMD | AST_I2CD_RX_BUFF_ENABLE;

				if((i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt) > i2c_dev->req_page->page_size) {
					i2c_dev->master_xfer_len = i2c_dev->req_page->page_size;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				} else {
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt;
					if((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->blk_r_flag == 0)) {
						dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN \n");
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
					} else {
						if(i2c_dev->xfer_last == 1) {
							dev_dbg(i2c_dev->dev, "last stop \n");
							cmd |= AST_I2CD_M_STOP_CMD;
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
											~AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
						} else {
							ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
						}
						cmd |= AST_I2CD_M_S_RX_CMD_LAST;
					}
				}
				ast_i2c_write(i2c_dev,
									AST_I2CD_RX_BUF_END_ADDR_SET((i2c_dev->master_xfer_len-1))|
									AST_I2CD_BUF_BASE_ADDR_SET((i2c_dev->req_page->page_addr_point)),
									I2C_BUF_CTRL_REG);
				ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);
				dev_dbg(i2c_dev->dev, "rxfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);
			} else {
				//Tx data
				//next data write
				cmd = AST_I2CD_M_TX_CMD | AST_I2CD_TX_BUFF_ENABLE;
				if((i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt) > i2c_dev->req_page->page_size) {
					i2c_dev->master_xfer_len = i2c_dev->req_page->page_size;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);

				} else {
					i2c_dev->master_xfer_len = i2c_dev->master_msgs->len - i2c_dev->master_xfer_cnt;
					if(i2c_dev->xfer_last == 1) {
						dev_dbg(i2c_dev->dev, "last stop \n");
						cmd |= AST_I2CD_M_STOP_CMD;
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
											~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);

					} else {
						ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
											AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					}
				}

				for(i = 0; i < i2c_dev->master_xfer_len; i++) {
					if(i%4 == 0)
						tx_buf[i/4] = 0;
					tx_buf[i/4] |= (i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i] << ((i%4)*8)) ;
				}
//				printk("count %x \n",ast_i2c_read(i2c_dev,I2C_CMD_REG));
				ast_i2c_write(i2c_dev,
							AST_I2CD_TX_DATA_BUF_END_SET((i2c_dev->master_xfer_len - 1)) |
							AST_I2CD_BUF_BASE_ADDR_SET(i2c_dev->req_page->page_addr_point),
							I2C_BUF_CTRL_REG);

				ast_i2c_write(i2c_dev, cmd , I2C_CMD_REG);
				dev_dbg(i2c_dev->dev, "txfer size %d , cmd = %x \n",i2c_dev->master_xfer_len, cmd);
			}
		} else {
			//should send next msg
			if(i2c_dev->master_xfer_cnt != i2c_dev->master_msgs->len)
				printk("complete rx ... bus=%d addr=0x%x (%d vs. %d) ERROR\n",
				i2c_dev->bus_id, i2c_dev->master_msgs->addr,
				i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);

			dev_dbg(i2c_dev->dev, "ast_i2c_do_byte_xfer complete \n");
			i2c_dev->cmd_err = 0;
			complete(&i2c_dev->cmd_complete);
		}

	}
}
static void ast_i2c_do_byte_xfer(struct ast_i2c_dev *i2c_dev)
{
	u8 *xfer_buf;
	u32 cmd = 0;

	if(i2c_dev->slave_operation == 1) {
		dev_dbg(i2c_dev->dev,"S cnt %d, xf len %d \n",i2c_dev->slave_xfer_cnt, i2c_dev->slave_msgs->len);
		if(i2c_dev->slave_msgs->flags & I2C_M_RD) {
			//READ <-- TX
			dev_dbg(i2c_dev->dev, "(<--) slave(tx) buf %d [%x]\n", i2c_dev->slave_xfer_cnt, i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt]);
			ast_i2c_write(i2c_dev, i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt], I2C_BYTE_BUF_REG);
			ast_i2c_write(i2c_dev, AST_I2CD_S_TX_CMD, I2C_CMD_REG);
		} else {
			// Write -->Rx
			//no need to handle in byte mode
			dev_dbg(i2c_dev->dev, "(-->) slave(rx) BYTE do nothing\n");

		}
	} else {
		dev_dbg(i2c_dev->dev,"M cnt %d, xf len %d \n",i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);
		if(i2c_dev->master_xfer_cnt == -1) {
			//first start
			dev_dbg(i2c_dev->dev, " %sing %d byte%s %s 0x%02x\n",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "read" : "write",
							i2c_dev->master_msgs->len, i2c_dev->master_msgs->len > 1 ? "s" : "",
							i2c_dev->master_msgs->flags & I2C_M_RD ? "from" : "to", i2c_dev->master_msgs->addr);


			if(i2c_dev->master_msgs->flags & I2C_M_RD)
				ast_i2c_write(i2c_dev, (i2c_dev->master_msgs->addr <<1) |0x1, I2C_BYTE_BUF_REG);
			else
				ast_i2c_write(i2c_dev, (i2c_dev->master_msgs->addr <<1), I2C_BYTE_BUF_REG);

			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
								AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);

			ast_i2c_write(i2c_dev, AST_I2CD_M_TX_CMD | AST_I2CD_M_START_CMD, I2C_CMD_REG);


		} else if (i2c_dev->master_xfer_cnt < i2c_dev->master_msgs->len){
			xfer_buf = i2c_dev->master_msgs->buf;
			if(i2c_dev->master_msgs->flags & I2C_M_RD) {
				//Rx data
				cmd = AST_I2CD_M_RX_CMD;
				if((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->master_xfer_cnt == 0)) {
					dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN \n");
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);

				} else if((i2c_dev->xfer_last == 1) && (i2c_dev->master_xfer_cnt + 1 == i2c_dev->master_msgs->len)) {
					cmd |= AST_I2CD_M_S_RX_CMD_LAST | AST_I2CD_M_STOP_CMD;
	//				disable rx_dwn isr
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
										~AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				} else {
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				}

				dev_dbg(i2c_dev->dev, "(<--) rx byte, cmd = %x \n",cmd);

				ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);


			} else {
				//Tx data
				dev_dbg(i2c_dev->dev, "(-->) xfer byte data index[%02x]:%02x  \n",i2c_dev->master_xfer_cnt, *(xfer_buf + i2c_dev->master_xfer_cnt));
				ast_i2c_write(i2c_dev, *(xfer_buf + i2c_dev->master_xfer_cnt), I2C_BYTE_BUF_REG);
				if((i2c_dev->xfer_last == 1) && (i2c_dev->master_xfer_cnt + 1 == i2c_dev->master_msgs->len)) {
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) &
										~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					ast_i2c_write(i2c_dev, AST_I2CD_M_TX_CMD | AST_I2CD_M_STOP_CMD, I2C_CMD_REG);
				} else {
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					ast_i2c_write(i2c_dev, AST_I2CD_M_TX_CMD, I2C_CMD_REG);
				}
			}

		} else {
			//should send next msg
			if(i2c_dev->master_xfer_cnt != i2c_dev->master_msgs->len)
				printk("CNT ERROR bus=%d addr=0x%x (%d vs. %d)\n",
				       i2c_dev->bus_id, i2c_dev->master_msgs->addr,
				       i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);

			dev_dbg(i2c_dev->dev, "ast_i2c_do_byte_xfer complete \n");
			i2c_dev->cmd_err = 0;
			complete(&i2c_dev->cmd_complete);

		}
	}

}

static void ast_i2c_slave_xfer_done(struct ast_i2c_dev *i2c_dev)
{
	u32 xfer_len;
	int i;
	u8 *rx_buf;
  unsigned long flags;

	dev_dbg(i2c_dev->dev, "ast_i2c_slave_xfer_done [%d]\n",i2c_dev->slave_xfer_mode);


	if (i2c_dev->slave_msgs->flags & I2C_M_RD) {
		//tx done , only check tx count ...
		if(i2c_dev->slave_xfer_mode == BYTE_XFER) {
			xfer_len = 1;
		} else if (i2c_dev->slave_xfer_mode == BUFF_XFER) {
			xfer_len = AST_I2CD_TX_DATA_BUF_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
			xfer_len++;
			dev_dbg(i2c_dev->dev,"S tx buff done len %d \n",xfer_len);
		} else if (i2c_dev->slave_xfer_mode == DEC_DMA_XFER) {
			//DMA mode
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			if(xfer_len == 0)
				xfer_len = i2c_dev->slave_xfer_len;
			else
				xfer_len = i2c_dev->slave_xfer_len - xfer_len - 1;
			dev_dbg(i2c_dev->dev,"S tx tx dma done len %d \n",xfer_len);
		} else if (i2c_dev->slave_xfer_mode == INC_DMA_XFER) {
			//DMA mode
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			xfer_len = i2c_dev->slave_xfer_len - xfer_len;
			dev_dbg(i2c_dev->dev,"S tx tx dma done len %d \n",xfer_len);

		} else {
			printk("ERROR type !! \n");
		}

	} else {
		//rx done
		if(i2c_dev->slave_xfer_mode == BYTE_XFER) {
			//TODO
			xfer_len = 1;
			if(i2c_dev->slave_event == I2C_SLAVE_EVENT_STOP) {
				i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt] = 0;
				i2c_dev->slave_msgs->len = i2c_dev->slave_xfer_cnt;
			} else {
        if (i2c_dev->slave_xfer_cnt == 0)
          dev_err(i2c_dev->dev,"Possible first byte failure issue\n");
				i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt] = ast_i2c_read(i2c_dev,I2C_BYTE_BUF_REG) >> 8;
			}
			dev_dbg(i2c_dev->dev,"rx buff %d, [%x] \n",i2c_dev->slave_xfer_cnt ,i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt]);
		} else if (i2c_dev->slave_xfer_mode == BUFF_XFER) {
			xfer_len = AST_I2CD_RX_BUF_ADDR_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
#if defined(AST_SOC_G5)
#else
			if(xfer_len == 0)
				xfer_len = AST_I2C_PAGE_SIZE;
#endif
			i2c_dev->slave_xfer_len = xfer_len;
			dev_dbg(i2c_dev->dev,"rx buff done len %d \n",xfer_len);

#if defined(AST_SOC_G5)
			dev_dbg(i2c_dev->dev, "offset buffer = %x \n",i2c_dev->bus_id * 0x10);

			rx_buf = (u8 *) (i2c_dev->req_page->page_addr + i2c_dev->bus_id * 0x10);
#else
			rx_buf = (u8 *)i2c_dev->req_page->page_addr;
#endif

			for(i=0;i<xfer_len;i++) {
				i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt+i] = rx_buf[i];
				dev_dbg(i2c_dev->dev,"%d, [%x] \n",i2c_dev->slave_xfer_cnt+i ,i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt+i]);
			}

		} else if (i2c_dev->slave_xfer_mode == DEC_DMA_XFER) {
			//RX DMA DOWN
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			if(xfer_len == 0)
				xfer_len = i2c_dev->slave_xfer_len;
			else
				xfer_len = i2c_dev->slave_xfer_len - xfer_len - 1;
			dev_dbg(i2c_dev->dev, " S rx dma done len %d \n", xfer_len);
			
			for(i=0;i<xfer_len;i++) {
				i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt+i] = i2c_dev->dma_buf[i];
				dev_dbg(i2c_dev->dev,"%d, [%x] \n",i2c_dev->slave_xfer_cnt+i ,i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt+i]);
			}
		} else if (i2c_dev->slave_xfer_mode == INC_DMA_XFER) {
			//RX DMA DOWN
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			if(xfer_len == 0)
				xfer_len = i2c_dev->slave_xfer_len;
			else
				xfer_len = i2c_dev->slave_xfer_len - xfer_len;

			dev_dbg(i2c_dev->dev, " S rx dma done len %d \n", xfer_len);
			for(i=0;i<xfer_len;i++) {
				i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt+i] = i2c_dev->dma_buf[i];
				dev_dbg(i2c_dev->dev,"%d, [%x] \n",i2c_dev->slave_xfer_cnt+i ,i2c_dev->slave_msgs->buf[i2c_dev->slave_xfer_cnt+i]);
			}
		} else {
			printk("ERROR !! XFER Type \n");
		}
	}

	if(xfer_len !=i2c_dev->slave_xfer_len) {
		//TODO..
		printk(" **slave xfer error ====\n");
		//should goto stop....
	} else
		i2c_dev->slave_xfer_cnt += i2c_dev->slave_xfer_len;


	if((i2c_dev->slave_event == I2C_SLAVE_EVENT_NACK) || (i2c_dev->slave_event == I2C_SLAVE_EVENT_STOP)) {
#ifdef CONFIG_AST_I2C_SLAVE_RDWR
			ast_i2c_slave_rdwr_xfer(i2c_dev);
#else
			i2c_dev->ast_i2c_data->slave_xfer(i2c_dev->slave_event, &(i2c_dev->slave_msgs));
#endif
		i2c_dev->slave_xfer_cnt = 0;
	} else {
		if(i2c_dev->slave_xfer_cnt == i2c_dev->slave_msgs->len)	{
			dev_err(i2c_dev->dev,"slave next msgs with len %d\n", i2c_dev->slave_xfer_cnt);
#ifdef CONFIG_AST_I2C_SLAVE_RDWR
			ast_i2c_slave_rdwr_xfer(i2c_dev);
#else
			i2c_dev->ast_i2c_data->slave_xfer(i2c_dev->slave_event, &(i2c_dev->slave_msgs));
#endif

			i2c_dev->slave_xfer_cnt = 0;
		}
		i2c_dev->do_slave_xfer(i2c_dev);
	}

  // Read the current state for clearing up the slave mode
  i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;

	if(AST_I2CD_IDLE == i2c_dev->state) {
		dev_dbg(i2c_dev->dev,"** Slave go IDLE **\n");
		i2c_dev->slave_operation = 0;

		if(i2c_dev->slave_xfer_mode == BUFF_XFER) {
			i2c_dev->ast_i2c_data->free_pool_buff_page(i2c_dev->req_page);
		}

	} else if (i2c_dev->slave_event == I2C_SLAVE_EVENT_STOP) {
    // TODO: hack to reset slave operation flag in case the stop is received
		i2c_dev->slave_operation = 0;
  }

}

//TX/Rx Done
static void ast_i2c_master_xfer_done(struct ast_i2c_dev *i2c_dev)
{
	u32 xfer_len;
	int i;
	u8 *pool_buf;
	unsigned long flags;

	spin_lock_irqsave(&i2c_dev->master_lock, flags);

	/*
	 * This function shall be involked during interrupt handling.
	 * Since the interrupt could be fired at anytime, we will need to make sure
	 * we have the buffer (i2c_dev->master_msgs) to handle the results.
	 */
	if (!i2c_dev->master_msgs) {
		goto unlock_out;
	}

	dev_dbg(i2c_dev->dev, "ast_i2c_master_xfer_done mode[%d]\n",i2c_dev->master_xfer_mode);

	if (i2c_dev->master_msgs->flags & I2C_M_RD) {
		if(i2c_dev->master_xfer_cnt == -1) {
			xfer_len = 1;
			goto next_xfer;
		}
		if(i2c_dev->master_xfer_mode == BYTE_XFER) {
			if ((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->blk_r_flag == 0)) {
				i2c_dev->master_msgs->len += (ast_i2c_read(i2c_dev,I2C_BYTE_BUF_REG) & AST_I2CD_RX_BYTE_BUFFER) >> 8;
				i2c_dev->blk_r_flag = 1;
				dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN %d \n", i2c_dev->master_msgs->len -1);
			}
			xfer_len = 1;
			i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt] = (ast_i2c_read(i2c_dev,I2C_BYTE_BUF_REG) & AST_I2CD_RX_BYTE_BUFFER) >> 8;
		} else if (i2c_dev->master_xfer_mode == BUFF_XFER) {
#if defined(AST_SOC_G5)
			dev_dbg(i2c_dev->dev, "offset buffer = %x \n",i2c_dev->bus_id * 0x10);

			pool_buf = (u8 *) (i2c_dev->req_page->page_addr + i2c_dev->bus_id * 0x10);
#else
			pool_buf = (u8 *)i2c_dev->req_page->page_addr;
#endif
			xfer_len = AST_I2CD_RX_BUF_ADDR_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
#if defined(AST_SOC_G5)
#else
			if(xfer_len == 0)
				xfer_len = AST_I2C_PAGE_SIZE;
#endif

			for(i = 0; i< xfer_len; i++) {
				i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i] = pool_buf[i];
				dev_dbg(i2c_dev->dev, "rx %d buff[%x]\n",i2c_dev->master_xfer_cnt+i, i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt+i]);
			}

			if ((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->blk_r_flag == 0)) {
				i2c_dev->master_msgs->len += pool_buf[0];
				i2c_dev->blk_r_flag = 1;
				dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN %d \n", i2c_dev->master_msgs->len -1);
			}
		} else if (i2c_dev->master_xfer_mode == DEC_DMA_XFER) {
			//DMA Mode
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			if(xfer_len == 0)
				xfer_len = i2c_dev->master_xfer_len;
			else
				xfer_len = i2c_dev->master_xfer_len - xfer_len - 1;
			for(i = 0; i < xfer_len; i++) {
				i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i] = i2c_dev->dma_buf[i];
				dev_dbg(i2c_dev->dev, "buf[%x] \n", i2c_dev->dma_buf[i]);
				dev_dbg(i2c_dev->dev, "buf[%x] \n", i2c_dev->dma_buf[i+1]);
			}
	
			if ((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->blk_r_flag == 0)) {
					i2c_dev->master_msgs->len += i2c_dev->dma_buf[0];
					i2c_dev->blk_r_flag = 1;
					dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN %d \n", i2c_dev->master_msgs->len -1);			
			}
			
		} else if (i2c_dev->master_xfer_mode == INC_DMA_XFER) {
			//DMA Mode
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			if(xfer_len == 0)
				xfer_len = i2c_dev->master_xfer_len;
			else
				xfer_len = i2c_dev->master_xfer_len - xfer_len;
			
			for(i = 0; i < xfer_len; i++) {
				i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i] = i2c_dev->dma_buf[i];
				dev_dbg(i2c_dev->dev, "buf[%x] \n", i2c_dev->dma_buf[i]);
				dev_dbg(i2c_dev->dev, "buf[%x] \n", i2c_dev->dma_buf[i+1]);
			}
	
			if ((i2c_dev->master_msgs->flags & I2C_M_RECV_LEN) && (i2c_dev->blk_r_flag == 0)) {
					i2c_dev->master_msgs->len += i2c_dev->dma_buf[0];
					i2c_dev->blk_r_flag = 1;
					dev_dbg(i2c_dev->dev, "I2C_M_RECV_LEN %d \n", i2c_dev->master_msgs->len -1);			
			}
			
		} else {
			printk("ERROR xfer type \n");
		}

	}else {
		if(i2c_dev->master_xfer_mode == BYTE_XFER) {
			xfer_len = 1;
		} else if(i2c_dev->master_xfer_mode == BUFF_XFER) {
			xfer_len = AST_I2CD_TX_DATA_BUF_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
			xfer_len++;
			dev_dbg(i2c_dev->dev,"tx buff done len %d \n",xfer_len);
		} else if(i2c_dev->master_xfer_mode == DEC_DMA_XFER) {
			//DMA
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			if(xfer_len == 0)
				xfer_len = i2c_dev->master_xfer_len;
			else
				xfer_len = i2c_dev->master_xfer_len - xfer_len - 1;
			dev_dbg(i2c_dev->dev,"tx dma done len %d \n",xfer_len);
		} else if(i2c_dev->master_xfer_mode == INC_DMA_XFER) {
			//DMA
			xfer_len = ast_i2c_read(i2c_dev, I2C_DMA_LEN_REG);
			xfer_len = i2c_dev->master_xfer_len - xfer_len;
			dev_dbg(i2c_dev->dev,"tx dma done len %d \n",xfer_len);
		} else {
			printk("ERROR xfer type \n");
		}
	}

next_xfer:

	if(xfer_len !=i2c_dev->master_xfer_len) {
		//TODO..
		printk(" ** xfer error bus=%d addr=0x%x (%d vs. %d)\n",
		       i2c_dev->bus_id, i2c_dev->master_msgs->addr,
		       xfer_len, i2c_dev->master_xfer_len);
		//should goto stop....
		i2c_dev->cmd_err = 1;
		goto done_out;
	} else
		i2c_dev->master_xfer_cnt += i2c_dev->master_xfer_len;

	if(i2c_dev->master_xfer_cnt != i2c_dev->master_msgs->len) {
		dev_dbg(i2c_dev->dev,"do next cnt \n");
		i2c_dev->do_master_xfer(i2c_dev);
	} else {
#if 0
		int i;
		printk(" ===== \n");
		for(i=0;i<i2c_dev->master_msgs->len;i++)
			printk("rx buf i,[%x]\n",i,i2c_dev->master_msgs->buf[i]);
		printk(" ===== \n");
#endif
		i2c_dev->cmd_err = 0;

done_out:
		dev_dbg(i2c_dev->dev,"msgs complete \n");
		complete(&i2c_dev->cmd_complete);
	}

unlock_out:
	spin_unlock_irqrestore(&i2c_dev->master_lock, flags);
}

static void ast_i2c_slave_addr_match(struct ast_i2c_dev *i2c_dev)
{
	u8 match;
  unsigned long flags;


	i2c_dev->slave_operation = 1;
	i2c_dev->slave_xfer_cnt = 0;
	match = ast_i2c_read(i2c_dev,I2C_BYTE_BUF_REG) >> 8;
	i2c_dev->slave_msgs->buf[0] = match;
	dev_dbg(i2c_dev->dev, "S Start Addr match [%x] \n",match);

	if(match & 1) {
		i2c_dev->slave_event = I2C_SLAVE_EVENT_START_READ;
	} else {
		i2c_dev->slave_event = I2C_SLAVE_EVENT_START_WRITE;
	}

#ifdef CONFIG_AST_I2C_SLAVE_RDWR
	ast_i2c_slave_rdwr_xfer(i2c_dev);
  i2c_dev->slave_msgs->buf[0] = match;
  i2c_dev->slave_xfer_cnt = 1;
  // Reset the length field as we have received new slave address match
	i2c_dev->slave_msgs->len = 0x0;
#else
	i2c_dev->ast_i2c_data->slave_xfer(i2c_dev->slave_event, &(i2c_dev->slave_msgs));
	i2c_dev->slave_xfer_cnt = 0;
#endif

	//request: set slave_xfer_mode properly based on slave_dma mode
	if(i2c_dev->ast_i2c_data->slave_dma == BYTE_MODE) {
		i2c_dev->do_slave_xfer = ast_i2c_do_byte_xfer;
    i2c_dev->slave_xfer_mode = BYTE_XFER;
    i2c_dev->slave_xfer_len = 1;
	} else if (i2c_dev->ast_i2c_data->slave_dma == DEC_DMA_MODE) {
		i2c_dev->do_slave_xfer = ast_i2c_do_dec_dma_xfer;
    i2c_dev->slave_xfer_mode = DEC_DMA_XFER;
	} else if (i2c_dev->ast_i2c_data->slave_dma == INC_DMA_MODE) {
		i2c_dev->do_slave_xfer = ast_i2c_do_inc_dma_xfer;
    i2c_dev->slave_xfer_mode = INC_DMA_XFER;
	} else {
		if(i2c_dev->ast_i2c_data->request_pool_buff_page(&(i2c_dev->req_page)) == 0) {
			i2c_dev->do_slave_xfer = ast_i2c_do_pool_xfer;
      i2c_dev->slave_xfer_mode = BUFF_XFER;
		} else {
			i2c_dev->do_slave_xfer = ast_i2c_do_byte_xfer;
      dev_err(i2c_dev->dev,"i2cdriver: pool request failed for slave\n");
      i2c_dev->slave_xfer_mode = BYTE_XFER;
      i2c_dev->slave_xfer_len = 1;
    }
	}

	i2c_dev->do_slave_xfer(i2c_dev);

}

static irqreturn_t i2c_ast_handler(int this_irq, void *dev_id)
{
	u32 sts;
  u32 temp;

	struct ast_i2c_dev *i2c_dev = dev_id;
	u32 isr_sts = readl(i2c_dev->ast_i2c_data->reg_gr);

	if(!(isr_sts & (1<< i2c_dev->bus_id)))
		return IRQ_NONE;

	i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;
	sts = ast_i2c_read(i2c_dev,I2C_INTR_STS_REG);
//	printk("ISR : %x , sts [%x]\n",sts , xfer_sts);
//	dev_dbg(i2c_dev->dev,"ISR : %x , sts [%x]\n",sts , xfer_sts);

//	dev_dbg(i2c_dev->dev,"sts machine %x, slave_op %d \n", xfer_sts,i2c_dev->slave_operation);

	if(AST_I2CD_INTR_STS_SMBUS_ALT & sts) {
		dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_SMBUS_ALT= %x\n",sts);
		//Disable ALT INT
		ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_INTR_CTRL_REG) &
					~AST_I2CD_SMBUS_ALT_INTR_EN,
					I2C_INTR_CTRL_REG);
		ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SMBUS_ALT, I2C_INTR_STS_REG);
		ast_master_alert_recv(i2c_dev);
		sts &= ~AST_I2CD_SMBUS_ALT_INTR_EN;
	}

	if(AST_I2CD_INTR_STS_ABNORMAL & sts) {
    // TODO: observed abnormal interrupt happening when the bus is stressed with traffic
    dev_err(i2c_dev->dev, "abnormal interrupt happens with status: %x, slave mode: %d\n", sts, i2c_dev->slave_operation);
    // Need to clear the interrupt
    ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_ABNORMAL, I2C_INTR_STS_REG);

		i2c_dev->cmd_err |= AST_I2CD_INTR_STS_ABNORMAL;
		complete(&i2c_dev->cmd_complete);

    return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_SCL_TO & sts) {
    dev_err(i2c_dev->dev, "SCL LOW detected with sts = %x, slave mode: %x\n",sts, i2c_dev->slave_operation);
    ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SCL_TO, I2C_INTR_STS_REG);
    i2c_dev->cmd_err |= AST_I2CD_INTR_STS_SCL_TO;
    complete(&i2c_dev->cmd_complete);

    // Reset i2c controller
    ast_i2c_bus_reset(i2c_dev);

    return IRQ_HANDLED;
	}

	switch(sts) {
		case AST_I2CD_INTR_STS_TX_ACK:
			if(i2c_dev->slave_operation == 1) {
				i2c_dev->slave_event = I2C_SLAVE_EVENT_READ;
				ast_i2c_slave_xfer_done(i2c_dev);
				dev_dbg(i2c_dev->dev, "S clear isr: AST_I2CD_INTR_STS_TX_ACK = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK, I2C_INTR_STS_REG);
			} else {
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK, I2C_INTR_STS_REG);
				ast_i2c_master_xfer_done(i2c_dev);
			}
			break;
		case AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP:
				if((i2c_dev->xfer_last == 1) && (i2c_dev->slave_operation == 0)) {
					dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP= %x\n",sts);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
					//take care
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					ast_i2c_master_xfer_done(i2c_dev);

				} else {
					dev_err(i2c_dev->dev,"ast_i2c:  TX_ACK | NORMAL_STOP;  xfer_last %d\n", i2c_dev->xfer_last);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
					uint32_t new_val = ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
						        	AST_I2CD_NORMAL_STOP_INTR_EN |
								AST_I2CD_TX_ACK_INTR_EN;
					ast_i2c_write(i2c_dev, new_val, I2C_INTR_CTRL_REG);
					//take care
					i2c_dev->cmd_err |= AST_LOCKUP_DETECTED;
					complete(&i2c_dev->cmd_complete);
				}
			break;
		case AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH :
				if((i2c_dev->xfer_last == 1) && (i2c_dev->slave_operation == 0)) {
					dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH= %x\n",sts);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
					//take care
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					ast_i2c_master_xfer_done(i2c_dev);

          // Handle the new slave match interrupt
    			ast_i2c_slave_addr_match(i2c_dev);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
				} else {
					dev_err(i2c_dev->dev, "Slave  TX_ACK | NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH;  xfer_last %d\n", i2c_dev->xfer_last);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
					uint32_t new_val = ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
						        	AST_I2CD_NORMAL_STOP_INTR_EN |
								AST_I2CD_TX_ACK_INTR_EN;
					ast_i2c_write(i2c_dev, new_val, I2C_INTR_CTRL_REG);
					//take care
					i2c_dev->cmd_err |= AST_LOCKUP_DETECTED;
					complete(&i2c_dev->cmd_complete);

          // stop previous slave transaction
          i2c_dev->slave_event = I2C_SLAVE_EVENT_STOP;
          ast_i2c_slave_xfer_done(i2c_dev);

          // Handle the new slave match interrupt
    			ast_i2c_slave_addr_match(i2c_dev);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
				}
			break;

		case AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH :
				if((i2c_dev->xfer_last == 1) && (i2c_dev->slave_operation == 0)) {
					dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH= %x\n",sts);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
					//take care
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
										AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
					ast_i2c_master_xfer_done(i2c_dev);

          // Handle the new slave match interrupt
    			ast_i2c_slave_addr_match(i2c_dev);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
				} else {
					dev_err(i2c_dev->dev, "Slave  TX_NAK | NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH;  xfer_last %d\n", i2c_dev->xfer_last);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
					uint32_t new_val = ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
						        	AST_I2CD_NORMAL_STOP_INTR_EN |
								AST_I2CD_TX_ACK_INTR_EN;
					ast_i2c_write(i2c_dev, new_val, I2C_INTR_CTRL_REG);
					//take care
					i2c_dev->cmd_err |= AST_LOCKUP_DETECTED;
					complete(&i2c_dev->cmd_complete);

          // stop previous slave transaction
          i2c_dev->slave_event = I2C_SLAVE_EVENT_STOP;
          ast_i2c_slave_xfer_done(i2c_dev);

          // Handle the new slave match interrupt
    			ast_i2c_slave_addr_match(i2c_dev);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
				}
			break;

		case AST_I2CD_INTR_STS_TX_NAK:
			if(i2c_dev->slave_operation == 1) {
				i2c_dev->slave_event = I2C_SLAVE_EVENT_NACK;
				ast_i2c_slave_xfer_done(i2c_dev);
				dev_err(i2c_dev->dev, "S clear isr: AST_I2CD_INTR_STS_TX_NAK = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK, I2C_INTR_STS_REG);
			} else {
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_NAK = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK, I2C_INTR_STS_REG);
				if (i2c_dev->master_msgs
				    && i2c_dev->master_msgs->flags & I2C_M_IGNORE_NAK) {
					dev_dbg(i2c_dev->dev, "I2C_M_IGNORE_NAK next send\n");
				} else {
					dev_dbg(i2c_dev->dev, "NAK error\n");
					i2c_dev->cmd_err |= AST_I2CD_INTR_STS_TX_NAK;
				}
				complete(&i2c_dev->cmd_complete);
			}
			break;

		case AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP:
			if(i2c_dev->slave_operation == 1) {
				printk("SLAVE TODO .... \n");
        i2c_dev->slave_operation = 0;
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				i2c_dev->cmd_err |= AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP;
				complete(&i2c_dev->cmd_complete);
			} else {
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_NAK| AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				dev_dbg(i2c_dev->dev, "M TX NAK | NORMAL STOP \n");
				i2c_dev->cmd_err |= AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP;
				complete(&i2c_dev->cmd_complete);
			}
			break;

		//Issue : Workaround for I2C slave mode
		case AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_SLAVE_MATCH:
			if(i2c_dev->slave_operation == 1) {
				dev_err(i2c_dev->dev, "i2cdriver: TX_NAK and Slave match sts = %x\n",sts);
				ast_i2c_slave_addr_match(i2c_dev);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_SLAVE_MATCH , I2C_INTR_STS_REG);
			} else {
				printk("ERROR !!!!\n");
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK, I2C_INTR_STS_REG);
				i2c_dev->cmd_err |= AST_I2CD_INTR_STS_TX_NAK;
				complete(&i2c_dev->cmd_complete);

				ast_i2c_slave_addr_match(i2c_dev);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SLAVE_MATCH , I2C_INTR_STS_REG);
			}
			break;
		case AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH:
			ast_i2c_slave_addr_match(i2c_dev);
			dev_dbg(i2c_dev->dev, "S clear isr: AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH = %x\n",sts);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
			break;
		case AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH | AST_I2CD_INTR_STS_ARBIT_LOSS:
			dev_err(i2c_dev->dev, "M clear isr: sts = %x\n",sts);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_ARBIT_LOSS, I2C_INTR_STS_REG);
			i2c_dev->cmd_err |= AST_I2CD_INTR_STS_ARBIT_LOSS;
      ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
                AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
			complete(&i2c_dev->cmd_complete);

			ast_i2c_slave_addr_match(i2c_dev);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
			break;
		case AST_I2CD_INTR_STS_RX_DOWN:
			if(i2c_dev->slave_operation == 1) {
				i2c_dev->slave_event = I2C_SLAVE_EVENT_WRITE;
				ast_i2c_slave_xfer_done(i2c_dev);
				dev_dbg(i2c_dev->dev, "S clear isr: AST_I2CD_INTR_STS_RX_DOWN = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN, I2C_INTR_STS_REG);
			} else {
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_RX_DOWN = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN, I2C_INTR_STS_REG);
				ast_i2c_master_xfer_done(i2c_dev);
			}
			break;

		case AST_I2CD_INTR_STS_NORMAL_STOP:
			if(i2c_dev->slave_operation == 1) {
				i2c_dev->slave_event = I2C_SLAVE_EVENT_STOP;
				ast_i2c_slave_xfer_done(i2c_dev);
				dev_dbg(i2c_dev->dev, "S clear isr: AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				dev_dbg(i2c_dev->dev, "state [%x] \n",i2c_dev->state);
			} else {
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				i2c_dev->cmd_err |= AST_I2CD_INTR_STS_NORMAL_STOP;
				complete(&i2c_dev->cmd_complete);
			}
			break;
		case AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP:
			/* Whether or not we're done, the hardware thinks we're done, so bail. */
			if(i2c_dev->slave_operation == 0) {
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				//take care
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
									AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				ast_i2c_master_xfer_done(i2c_dev);
			} else {
				dev_err(i2c_dev->dev, "S clear isr: AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				i2c_dev->slave_event = I2C_SLAVE_EVENT_STOP;
				ast_i2c_slave_xfer_done(i2c_dev);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
									AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				ast_i2c_master_xfer_done(i2c_dev);
      }
			break;
		case AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH:
			if(i2c_dev->slave_operation == 0) {
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				//take care
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) |
									AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
				ast_i2c_master_xfer_done(i2c_dev);

				// Handle the new slave match interrupt
				ast_i2c_slave_addr_match(i2c_dev);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
			} else {
				dev_err(i2c_dev->dev, "S clear isr: AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP |"
                              "AST_I2CD_INTR_STS_SLAVE_MATCH = %x\n",sts);
				i2c_dev->slave_event = I2C_SLAVE_EVENT_STOP;
				ast_i2c_slave_xfer_done(i2c_dev);

				// Handle the new slave match interrupt
				ast_i2c_slave_addr_match(i2c_dev);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP |
                                AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
      }
			break;
		case AST_I2CD_INTR_STS_ARBIT_LOSS:
			dev_err(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_ARBIT_LOSS = %x\n",sts);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_ARBIT_LOSS, I2C_INTR_STS_REG);
			i2c_dev->cmd_err |= AST_I2CD_INTR_STS_ARBIT_LOSS;
			complete(&i2c_dev->cmd_complete);
			break;
 		case AST_I2CD_INTR_STS_GCALL_ADDR:
			i2c_dev->cmd_err |= AST_I2CD_INTR_STS_GCALL_ADDR;
			complete(&i2c_dev->cmd_complete);

			break;
		case AST_I2CD_INTR_STS_SMBUS_DEF_ADDR:
			break;
		case AST_I2CD_INTR_STS_SMBUS_DEV_ALT:
			break;

		case AST_I2CD_INTR_STS_SMBUS_ARP_ADDR:
			break;
		case AST_I2CD_INTR_STS_SDA_DL_TO:
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SDA_DL_TO, I2C_INTR_STS_REG);
			i2c_dev->cmd_err |= AST_I2CD_INTR_STS_SDA_DL_TO;
			complete(&i2c_dev->cmd_complete);
			break;
		case AST_I2CD_INTR_STS_BUS_RECOVER:
			dev_err(i2c_dev->dev, "Bus recover with sts= %x, slave mode: %x\n",sts, i2c_dev->slave_operation);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_BUS_RECOVER, I2C_INTR_STS_REG);
			complete(&i2c_dev->cmd_complete);
			break;
		default:
				printk("GR %x : Status : %x, bus_id %d\n",i2c_dev->ast_i2c_data->reg_gr, sts, i2c_dev->bus_id);

      // Handle Arbitration Loss
      if (sts & AST_I2CD_INTR_STS_ARBIT_LOSS) {
			  ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_ARBIT_LOSS, I2C_INTR_STS_REG);
        i2c_dev->cmd_err |= AST_I2CD_INTR_STS_ARBIT_LOSS;
        complete(&i2c_dev->cmd_complete);
        sts &= (~AST_I2CD_INTR_STS_ARBIT_LOSS);
      }

      // Handle the write transaction ACK
      if (sts & AST_I2CD_INTR_STS_TX_ACK) {
			  ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK, I2C_INTR_STS_REG);
        ast_i2c_master_xfer_done(i2c_dev);
        sts &= (~AST_I2CD_INTR_STS_TX_ACK);
      }

      // Handle Normal Stop conditon
      if (sts & AST_I2CD_INTR_STS_NORMAL_STOP) {
			  ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK, I2C_INTR_STS_REG);
        sts &= (~AST_I2CD_INTR_STS_NORMAL_STOP);
				i2c_dev->cmd_err |= AST_I2CD_INTR_STS_NORMAL_STOP;
				complete(&i2c_dev->cmd_complete);
      }

      // Handle the Slave address match
      if (sts & AST_I2CD_INTR_STS_SLAVE_MATCH) {
        ast_i2c_slave_addr_match(i2c_dev);
        sts &= (~AST_I2CD_INTR_STS_SLAVE_MATCH);
			  ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
      }


      // TODO: Debug print for any unhandled condition
			if(sts) {
				printk("GR %x : Status : %x, bus_id %d\n",i2c_dev->ast_i2c_data->reg_gr, sts, i2c_dev->bus_id);
      }

      //TODO: Clearing this interrupt for now, but needs to cleanup this ISR function
      ast_i2c_write(i2c_dev, sts, I2C_INTR_STS_REG);

			return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

static int ast_i2c_do_msgs_xfer(struct ast_i2c_dev *i2c_dev, struct i2c_msg *msgs, int num)
{
	int i;
	int ret = 1;
	unsigned long flags;

	spin_lock_irqsave(&i2c_dev->master_lock, flags);

	//request: update master_xfer_mode based on master_dma selection
	if(i2c_dev->ast_i2c_data->master_dma == BYTE_MODE) {
		i2c_dev->do_master_xfer = ast_i2c_do_byte_xfer;
    i2c_dev->master_xfer_mode = BYTE_XFER;
    i2c_dev->master_xfer_len = 1;
	} else if (i2c_dev->ast_i2c_data->master_dma == DEC_DMA_MODE) {
		i2c_dev->do_master_xfer = ast_i2c_do_dec_dma_xfer;
    i2c_dev->master_xfer_mode = DEC_DMA_XFER;
	} else if (i2c_dev->ast_i2c_data->master_dma == INC_DMA_MODE) {
		i2c_dev->do_master_xfer = ast_i2c_do_inc_dma_xfer;
    i2c_dev->master_xfer_mode = INC_DMA_XFER;
	} else {
		if(i2c_dev->ast_i2c_data->request_pool_buff_page(&(i2c_dev->req_page)) == 0) {
			i2c_dev->do_master_xfer = ast_i2c_do_pool_xfer;
      i2c_dev->master_xfer_mode = BUFF_XFER;
		} else {
			i2c_dev->do_master_xfer = ast_i2c_do_byte_xfer;
      dev_err(i2c_dev->dev, "i2cdriver: pool request failed for master\n");
      i2c_dev->master_xfer_mode = BYTE_XFER;
      i2c_dev->master_xfer_len = 1;

    }
	}

//	printk("start xfer ret = %d \n",ret);

	for (i=0; i < num; i++) {
		i2c_dev->blk_r_flag = 0;
		i2c_dev->master_msgs = &msgs[i];
		if(num == i+1)
			i2c_dev->xfer_last = 1;
		else
			i2c_dev->xfer_last = 0;

		i2c_dev->blk_r_flag = 0;
		init_completion(&i2c_dev->cmd_complete);
		i2c_dev->cmd_err = 0;

		if(i2c_dev->master_msgs->flags & I2C_M_NOSTART)
			i2c_dev->master_xfer_cnt = 0;
		else
			i2c_dev->master_xfer_cnt = -1;

		i2c_dev->do_master_xfer(i2c_dev);

		spin_unlock_irqrestore(&i2c_dev->master_lock, flags);

		ret = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
													   i2c_dev->adap.timeout*HZ);

		spin_lock_irqsave(&i2c_dev->master_lock, flags);
		i2c_dev->master_msgs = NULL;

		if (ret == 0) {
			dev_err(i2c_dev->dev, "controller timed out\n");
			i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;
//			printk("sts [%x], isr sts [%x] \n",i2c_dev->state, ast_i2c_read(i2c_dev,I2C_INTR_STS_REG));
			ast_i2c_bus_reset(i2c_dev);
			ret = -ETIMEDOUT;
			spin_unlock_irqrestore(&i2c_dev->master_lock, flags);
			goto stop;
		}

		if(i2c_dev->cmd_err != 0 &&
		   i2c_dev->cmd_err != AST_I2CD_INTR_STS_NORMAL_STOP) {
			if (i2c_dev->cmd_err & AST_LOCKUP_DETECTED) {
				printk("ast-i2c:  error got unexpected STOP\n");
				// reset the bus
				//ast_i2c_bus_error_recover(i2c_dev);
        ast_i2c_bus_reset(i2c_dev);
			}
			ret = -EAGAIN;
			spin_unlock_irqrestore(&i2c_dev->master_lock, flags);
			goto stop;
		}
	}

	spin_unlock_irqrestore(&i2c_dev->master_lock, flags);

	if(i2c_dev->cmd_err == 0 ||
	   i2c_dev->cmd_err == AST_I2CD_INTR_STS_NORMAL_STOP) {
		ret = num;
		goto out;

	}
stop:
	if(i2c_dev->cmd_err & AST_I2CD_INTR_STS_NORMAL_STOP)
		goto out;
	init_completion(&i2c_dev->cmd_complete);
  i2c_dev->cmd_err = 0;
	ast_i2c_write(i2c_dev, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);
	wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
											   i2c_dev->adap.timeout*HZ);

out:
	//Free ..
	if(i2c_dev->master_xfer_mode == BUFF_XFER) {
		i2c_dev->ast_i2c_data->free_pool_buff_page(i2c_dev->req_page);

	}
	dev_dbg(i2c_dev->dev, "end xfer ret = %d, xfer mode[%d]\n",ret, i2c_dev->master_xfer_mode);
	return ret;

}

static int ast_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct ast_i2c_dev *i2c_dev = adap->algo_data;
	int ret, i;
	int sts;

	sts = ast_i2c_read(i2c_dev,I2C_CMD_REG);
	dev_dbg(i2c_dev->dev, "state[%x],SCL[%d],SDA[%d],BUS[%d]\n", (sts >> 19) & 0xf, (sts >> 18) & 0x1,(sts >> 17) & 0x1,(sts >> 16) & 1);
	/*
	 * Wait for the bus to become free.
	 */

	ret = ast_i2c_wait_bus_not_busy(i2c_dev);
	if (ret) {
		dev_err(&i2c_dev->adap.dev, "i2c_ast: timeout waiting for bus free\n");
		goto out;
	}

	for (i = adap->retries; i >= 0; i--) {

		ret = ast_i2c_do_msgs_xfer(i2c_dev, msgs, num);
		if (ret != -EAGAIN)
			goto out;
		dev_dbg(&i2c_dev->adap.dev, "Retrying transmission [%d]\n",i);
		udelay(100);
	}

	ret = -EREMOTEIO;
out:

	return ret;
}

static u32 ast_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_ast_algorithm = {
	.master_xfer	= ast_i2c_xfer,
#ifdef CONFIG_AST_I2C_SLAVE_RDWR
	.slave_xfer		= ast_i2c_slave_xfer,
#endif
	.functionality	= ast_i2c_functionality,
};

static int ast_i2c_probe(struct platform_device *pdev)
{
	struct ast_i2c_dev *i2c_dev;
	struct resource *res;
	int ret;

	dev_dbg(&pdev->dev, "ast_i2c_probe \n");

	i2c_dev = kzalloc(sizeof(struct ast_i2c_dev), GFP_KERNEL);
	if (!i2c_dev) {
		ret = -ENOMEM;
		goto err_no_mem;
	}

	i2c_dev->ast_i2c_data = pdev->dev.platform_data;
	if(i2c_dev->ast_i2c_data->master_dma == BUFF_MODE) {
		dev_dbg(&pdev->dev, "use buffer pool mode 256\n");

	} else if ((i2c_dev->ast_i2c_data->master_dma >= DEC_DMA_MODE) || (i2c_dev->ast_i2c_data->slave_dma >= DEC_DMA_MODE)) {
		dev_dbg(&pdev->dev, "use dma mode \n");
		if (!i2c_dev->dma_buf) {
			i2c_dev->dma_buf = dma_alloc_coherent(NULL, AST_I2C_DMA_SIZE, &i2c_dev->dma_addr, GFP_KERNEL);
			if (!i2c_dev->dma_buf) {
				printk("unable to allocate tx Buffer memory\n");
				ret = -ENOMEM;
				goto err_no_dma;
			}
			if(i2c_dev->dma_addr%4 !=0) {
				printk("not 4 byte boundary \n");
				ret = -ENOMEM;
				goto err_no_dma;
			}
//			printk("dma_buf = [0x%x] dma_addr = [0x%x], please check 4byte boundary \n",i2c_dev->dma_buf,i2c_dev->dma_addr);
			memset (i2c_dev->dma_buf, 0, AST_I2C_DMA_SIZE);
		}

	} else {
		//master_mode 0: use byte mode
		dev_dbg(&pdev->dev, "use default byte mode \n");
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto err_no_io_res;
	}
	if (!request_mem_region(res->start, resource_size(res), res->name)) {
        dev_err(&pdev->dev, "cannot reserved region\n");
        ret = -ENXIO;
        goto err_no_io_res;
	}

	i2c_dev->reg_base = ioremap(res->start, resource_size(res));
	if (!i2c_dev->reg_base) {
		ret = -EIO;
		goto release_mem;
	}

	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto ereqirq;
	}

	i2c_dev->dev = &pdev->dev;

#if defined (CONFIG_ARCH_AST1070)
	if(i2c_dev->irq == IRQ_C0_I2C) {
		i2c_dev->bus_id = pdev->id - NUM_BUS;
		dev_dbg(&pdev->dev, "C0 :: pdev->id %d , i2c_dev->bus_id = %d, i2c_dev->irq =%d\n",pdev->id, i2c_dev->bus_id,i2c_dev->irq);
#if (CONFIG_AST1070_NR >= 2)
	} else if(i2c_dev->irq == IRQ_C1_I2C) {
		i2c_dev->bus_id = pdev->id - (NUM_BUS + 8);
		dev_dbg(&pdev->dev, "C1 :: pdev->id %d , i2c_dev->bus_id = %d, i2c_dev->irq =%d\n",pdev->id, i2c_dev->bus_id,i2c_dev->irq);
#endif
	} else {
		i2c_dev->bus_id = pdev->id;
		dev_dbg(&pdev->dev, "AST pdev->id %d , i2c_dev->bus_id = %d, i2c_dev->irq =%d\n",pdev->id, i2c_dev->bus_id,i2c_dev->irq);
	}
#else
	i2c_dev->bus_id = pdev->id;
#endif

 	/* Initialize the I2C adapter */
	i2c_dev->adap.owner   = THIS_MODULE;
//TODO
	i2c_dev->adap.retries = 0;

//	i2c_dev->adap.retries = 3;

	i2c_dev->adap.timeout = 5;

	i2c_dev->master_xfer_mode = BYTE_XFER;

	/*
	 * If "pdev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c_dev->adap.nr = pdev->id != -1 ? pdev->id : 0;
	snprintf(i2c_dev->adap.name, sizeof(i2c_dev->adap.name), "ast_i2c.%u",
		 i2c_dev->adap.nr);

	i2c_dev->slave_operation = 0;
	i2c_dev->blk_r_flag = 0;
	i2c_dev->adap.algo = &i2c_ast_algorithm;
	i2c_dev->adap.algo_data = i2c_dev;
	i2c_dev->adap.dev.parent = &pdev->dev;

	ast_i2c_dev_init(i2c_dev);

	ret = request_irq(i2c_dev->irq, i2c_ast_handler, IRQF_SHARED,
			  i2c_dev->adap.name, i2c_dev);
	if (ret) {
		printk(KERN_INFO "I2C: Failed request irq %d\n", i2c_dev->irq);
		goto ereqirq;
	}

  spin_lock_init(&i2c_dev->master_lock);

#ifdef CONFIG_AST_I2C_SLAVE_RDWR
	ast_i2c_slave_buff_init(i2c_dev);
  spin_lock_init(&i2c_dev->slave_rx_lock);
#endif

	ret = i2c_add_numbered_adapter(&i2c_dev->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}

	platform_set_drvdata(pdev, i2c_dev);

	printk(KERN_INFO "I2C: %d: AST I2C adapter [%d khz]\n",
	       i2c_dev->bus_id,i2c_dev->ast_i2c_data->bus_clk/1000);

	return 0;

eadapt:
	free_irq(i2c_dev->irq, i2c_dev);
ereqirq:
	iounmap(i2c_dev->reg_base);

release_mem:
	release_mem_region(res->start, resource_size(res));
err_no_io_res:
err_no_dma:
	kfree(i2c_dev);

err_no_mem:
	return ret;
}

static int ast_i2c_remove(struct platform_device *pdev)
{
	struct ast_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	struct resource *res;

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c_dev->adap);

	free_irq(i2c_dev->irq, i2c_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(i2c_dev->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);

	kfree(i2c_dev);

	return 0;
}

#ifdef CONFIG_PM
static int ast_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TODO
//	struct ast_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	return 0;
}

static int ast_i2c_resume(struct platform_device *pdev)
{
	//TODO
//	struct ast_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	//Should reset i2c ???
	return 0;
}
#else
#define ast_i2c_suspend        NULL
#define ast_i2c_resume         NULL
#endif

static struct platform_driver i2c_ast_driver = {
	.probe			= ast_i2c_probe,
	.remove 		= ast_i2c_remove,
    .suspend        = ast_i2c_suspend,
    .resume         = ast_i2c_resume,
    .driver         = {
            .name   = "ast-i2c",
            .owner  = THIS_MODULE,
    },
};

static int __init ast_i2c_init(void)
{
	return platform_driver_register(&i2c_ast_driver);
}

static void __exit ast_i2c_exit(void)
{
	platform_driver_unregister(&i2c_ast_driver);
}
//TODO : check module init sequence
module_init(ast_i2c_init);
module_exit(ast_i2c_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED AST I2C Bus Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ast_i2c");
