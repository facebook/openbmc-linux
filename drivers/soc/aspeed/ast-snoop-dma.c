/********************************************************************************
* File Name     : ast_snoop_dma.c
* Author         : Ryan Chen
* Description   : AST SNOOP DMA Controller
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
#include <linux/dma-mapping.h>


#include <plat/ast-lpc.h>
#include <plat/regs-lpc.h>

//#define CONFIG_AST_SNOOP_DMA_DEBUG

#ifdef CONFIG_AST_SNOOP_DMA_DEBUG
	#define SNOOP_DMA_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define SNOOP_DMA_DBG(fmt, args...)
#endif
/************************************************** SYS FS **************************************************************/
static ssize_t show_snoop_dma_data3(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);
	u16 rd_data;

	spin_lock(&ast_snoop_dma->snoop_lock);

	ast_snoop_dma->port3_rd_idx %= SNOOP_DMA_SIZE;

	if(ast_snoop_dma->port3_rd_idx != ast_snoop_dma->port3_wr_idx) {
		rd_data = ast_snoop_dma->port3_data[ast_snoop_dma->port3_rd_idx++];
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return sprintf(buf, "%02x\n", rd_data);
	} else {
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return 0;
	}
}

static DEVICE_ATTR(data3, S_IRUGO, show_snoop_dma_data3, NULL);

static ssize_t show_snoop_dma_data2(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);
	u16 rd_data;

	spin_lock(&ast_snoop_dma->snoop_lock);

	ast_snoop_dma->port2_rd_idx %= SNOOP_DMA_SIZE;

	if(ast_snoop_dma->port2_rd_idx != ast_snoop_dma->port2_wr_idx) {
		rd_data = ast_snoop_dma->port2_data[ast_snoop_dma->port2_rd_idx++];
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return sprintf(buf, "%02x\n", rd_data);
	} else {
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return 0;
	}
}

static DEVICE_ATTR(data2, S_IRUGO, show_snoop_dma_data2, NULL);

static ssize_t show_snoop_dma_data1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);
	u16 rd_data;

	spin_lock(&ast_snoop_dma->snoop_lock);

	ast_snoop_dma->port1_rd_idx %= SNOOP_DMA_SIZE;

	if(ast_snoop_dma->port1_rd_idx != ast_snoop_dma->port1_wr_idx) {
		rd_data = ast_snoop_dma->port1_data[ast_snoop_dma->port1_rd_idx++];
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return sprintf(buf, "%02x\n", rd_data);
	} else {
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return 0;
	}
}

static DEVICE_ATTR(data1, S_IRUGO, show_snoop_dma_data1, NULL);

static ssize_t show_snoop_dma_data0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);
	u16 rd_data;

	spin_lock(&ast_snoop_dma->snoop_lock);

	if(ast_snoop_dma->port0_rd_idx != ast_snoop_dma->port0_wr_idx) {
		rd_data = ast_snoop_dma->port0_data[ast_snoop_dma->port0_rd_idx++];
		ast_snoop_dma->port0_rd_idx %= SNOOP_DMA_SIZE;
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return sprintf(buf, "%02x\n", rd_data);
	} else {
		spin_unlock(&ast_snoop_dma->snoop_lock);
		return 0;
	}
}

static DEVICE_ATTR(data0, S_IRUGO, show_snoop_dma_data0, NULL);

static ssize_t show_snoop_dma_data_history(struct device *dev,
  struct device_attribute *attr, char *buf)
{
  struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);
  u16 i;
  char *ptr;

  spin_lock(&ast_snoop_dma->snoop_lock);

  ptr = buf;
  if (((ast_snoop_dma->port0_wr_idx - ast_snoop_dma->port0_rd_idx) &
        (SNOOP_DMA_SIZE - 1)) > 256)
    i = (ast_snoop_dma->port0_wr_idx - 256) & (SNOOP_DMA_SIZE - 1);
  else
    i = ast_snoop_dma->port0_rd_idx;

  while (i != ast_snoop_dma->port0_wr_idx ) {
    ptr += sprintf(ptr, "%02x ", ast_snoop_dma->dma_virt[i]);
    i=(i+1)%SNOOP_DMA_SIZE;
  }
  // remove last space
  if (ptr != buf) {
    ptr--;
    *ptr = '\0';
  }

  spin_unlock(&ast_snoop_dma->snoop_lock);
  return (ssize_t)(ptr - buf);
}

static DEVICE_ATTR(data_history, S_IRUGO, show_snoop_dma_data_history, NULL);

static ssize_t show_snoop_dma_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);

	return sprintf(buf, "%d [0: 1 Byte mode/1: 2 Byte mode/2: 4 Byte mode/3: Full mode]\n", ast_snoop_dma->snoop_mode);
}

static ssize_t store_snoop_dma_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	u8 enable = 0;
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);
	u32 mask = readl(ast_snoop_dma->pccr0) & ~LPC_POST_CODE_MODE_MASK;
	u32 dontcare_mask = readl(ast_snoop_dma->pccr1) & ~LPC_DONT_CARE_MASK;

	val = simple_strtoul(buf, NULL, 10);

	ast_snoop_dma->snoop_mode = val;

	SNOOP_DMA_DBG("mode %x \n", ast_snoop_dma->snoop_mode);

	if(readl(ast_snoop_dma->pccr0) & LPC_POST_CODE_EN) {
		writel(readl(ast_snoop_dma->pccr0) & ~(LPC_POST_CODE_EN | LPC_POST_DMA_MODE_EN), ast_snoop_dma->pccr0);
		writel(readl(ast_snoop_dma->pccr0) | LPC_RX_FIFO_CLR, ast_snoop_dma->pccr0);
		ast_snoop_dma->snoop_mode = 0;
		ast_snoop_dma->port0_rd_idx = 0;
		ast_snoop_dma->port0_wr_idx = 0;
		ast_snoop_dma->port1_rd_idx = 0;
		ast_snoop_dma->port1_wr_idx = 0;
		ast_snoop_dma->port2_rd_idx = 0;
		ast_snoop_dma->port2_wr_idx = 0;
		ast_snoop_dma->port3_rd_idx = 0;
		ast_snoop_dma->port3_wr_idx = 0;
		ast_snoop_dma->snoop_index = 0;
		enable = 1;
	}

	switch(ast_snoop_dma->snoop_mode) {
		case 0:	//port
			writel(dontcare_mask, ast_snoop_dma->pccr1);
			break;
		case 1:	//prot. port+1
			writel(mask | LPC_POST_CODE_MODE(WORD_MODE), ast_snoop_dma->pccr0);
			writel(dontcare_mask | (1 << 16), ast_snoop_dma->pccr1);
			break;
		case 2: //port ~ port +3
			writel(mask | LPC_POST_CODE_MODE(WORD_MODE), ast_snoop_dma->pccr0);
			writel(dontcare_mask | (3 << 16), ast_snoop_dma->pccr1);
			break;
		case 3:	//TODO ~~
			writel(mask, ast_snoop_dma->pccr0);
			break;
		default:
			count = 0;
			break;

	}

	if(enable)
		writel(readl(ast_snoop_dma->pccr0) | (LPC_POST_CODE_EN | LPC_POST_DMA_MODE_EN), ast_snoop_dma->pccr0);

	return count;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_snoop_dma_mode, store_snoop_dma_mode);

static ssize_t show_snoop_dma_port(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);


	return sprintf(buf, "0x%x\n", readl(ast_snoop_dma->pccr1) & 0xffff);
}

static ssize_t store_snoop_dma_port(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);

	val = StrToHex((char *)buf) & 0xffff;

	writel((readl(ast_snoop_dma->pccr1) & 0xffff0000) | val, ast_snoop_dma->pccr1);
	return count;
}

static DEVICE_ATTR(port, S_IRUGO | S_IWUSR, show_snoop_dma_port, store_snoop_dma_port);

static ssize_t show_snoop_dma_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);

	if(readl(ast_snoop_dma->pccr0) & LPC_POST_CODE_EN)
		return sprintf(buf, "1: Enable\n");
	else
		return sprintf(buf, "0: Disable\n");
}

static ssize_t store_snoop_dma_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_snoop_dma_data *ast_snoop_dma = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);
	//use byte mode snoop
	if(val) {
		writel(readl(ast_snoop_dma->pccr0) | LPC_POST_CODE_EN,	ast_snoop_dma->pccr0);
	} else {
		writel(readl(ast_snoop_dma->pccr0) & ~LPC_POST_CODE_EN, ast_snoop_dma->pccr0);
	}
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, show_snoop_dma_en, store_snoop_dma_en);

static struct attribute *snoop_dma_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_port.attr,
	&dev_attr_mode.attr,
	&dev_attr_data0.attr,
	&dev_attr_data1.attr,
	&dev_attr_data2.attr,
	&dev_attr_data3.attr,
  &dev_attr_data_history.attr,
	NULL
};

static const struct attribute_group snoop_dma_attribute_group = {
	.attrs = snoop_dma_attributes
};

static void ast_snoop_dma_tasklet_func(unsigned long data)
{
	struct ast_snoop_dma_data *ast_snoop_dma =  (struct ast_snoop_dma_data *)data;
	u32 cur_pt = readl(ast_snoop_dma->pccr6) & (SNOOP_DMA_SIZE - 1);
	u8 info_addr = 0;
	u8 port_data = 0;

	SNOOP_DMA_DBG("pt : %x \n", cur_pt);

	spin_lock(&ast_snoop_dma->snoop_lock);

	while(ast_snoop_dma->snoop_index != cur_pt) {
		port_data = ast_snoop_dma->dma_virt[ast_snoop_dma->snoop_index++];
		if(ast_snoop_dma->snoop_mode) {
			// multi byte write mode
			//confirm port no and read or write
			info_addr = ast_snoop_dma->dma_virt[ast_snoop_dma->snoop_index++];
			SNOOP_DMA_DBG("data : %x, info : %x \n", port_data, info_addr);
			if(info_addr & 0x40) {
				//write
				switch(info_addr & 0xf) {
					case 0: //port 0
						if((ast_snoop_dma->port0_wr_idx + 1) % SNOOP_DMA_SIZE == ast_snoop_dma->port0_rd_idx) {
							ast_snoop_dma->port0_data[ast_snoop_dma->port0_wr_idx++] = port_data;
							ast_snoop_dma->port0_rd_idx++;
						} else {
							ast_snoop_dma->port0_data[ast_snoop_dma->port0_wr_idx++] = port_data;
						}
						SNOOP_DMA_DBG("port 0 : %x \n", port_data);
						break;
					case 1: //port 1
						if((ast_snoop_dma->port1_wr_idx + 1) % SNOOP_DMA_SIZE == ast_snoop_dma->port1_rd_idx) {
							ast_snoop_dma->port1_data[ast_snoop_dma->port1_wr_idx++] = port_data;
						} else {
							ast_snoop_dma->port1_data[ast_snoop_dma->port1_wr_idx++] = port_data;
						}
						SNOOP_DMA_DBG("port 1 : %x \n", port_data);
						break;
					case 2: //port 2
						if((ast_snoop_dma->port2_wr_idx + 1) % SNOOP_DMA_SIZE == ast_snoop_dma->port2_rd_idx) {
							ast_snoop_dma->port2_data[ast_snoop_dma->port2_wr_idx++] = port_data;
						} else {
							ast_snoop_dma->port2_data[ast_snoop_dma->port2_wr_idx++] = port_data;
						}
						SNOOP_DMA_DBG("port 2 : %x \n", port_data);
						break;
					case 3: //port 3
						if((ast_snoop_dma->port3_wr_idx + 1) % SNOOP_DMA_SIZE == ast_snoop_dma->port3_rd_idx) {
							ast_snoop_dma->port3_data[ast_snoop_dma->port3_wr_idx++] = port_data;
						} else {
							ast_snoop_dma->port3_data[ast_snoop_dma->port3_wr_idx++] = port_data;
						}
						SNOOP_DMA_DBG("port 3 : %x \n", port_data);
						break;
				}

			} else {
				//read
				printk("read info : %x, data %x \n", info_addr, port_data);
			}
		} else {
			// 1 byte write mode
			if((ast_snoop_dma->port0_wr_idx + 1) % SNOOP_DMA_SIZE == ast_snoop_dma->port0_rd_idx) {
				ast_snoop_dma->port0_data[ast_snoop_dma->port0_wr_idx++] = port_data;
				ast_snoop_dma->port0_rd_idx++;
			} else {
				ast_snoop_dma->port0_data[ast_snoop_dma->port0_wr_idx++] = port_data;
			}
      SNOOP_DMA_DBG("%x \n", ast_snoop_dma->port0_data[ast_snoop_dma->port0_wr_idx-1]);
		}

		ast_snoop_dma->port0_rd_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->port0_wr_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->port1_rd_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->port1_wr_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->port2_rd_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->port2_wr_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->port3_rd_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->port3_wr_idx %= SNOOP_DMA_SIZE;
		ast_snoop_dma->snoop_index %= SNOOP_DMA_SIZE;
		cur_pt = readl(ast_snoop_dma->pccr6) & (SNOOP_DMA_SIZE - 1);
	}

	spin_unlock(&ast_snoop_dma->snoop_lock);
#if 0
	do {
		//1st byte: D[7:0] data
		//2nd byte: abort/write/irq/irq D[3:0] address
		//index at 0x1e7890c4 D[27:0]
		data = buffer[last_index++ % SNOOP_DMA_BOUNDARY];
		address = buffer[last_index++ % SNOOP_DMA_BOUNDARY];
		if (address & 0x40) {
			printk ("W %x %x = %x\n", (address & 0xf0) >> 4, b_address + (address & 0x0f), data);
		} else {
			printk ("R %x %x\n", (address & 0xf0) >> 4, b_address + (address & 0x0f));
		}
	} while ((last_index % SNOOP_DMA_BOUNDARY) != (read_register(0x1e7890c4) & (SNOOP_DMA_BOUNDARY - 1)));
#endif
}

static void ast_snoop_dma_handler(void *data)
{
	u32 snoop_dma_sts;
	struct ast_snoop_dma_data *ast_snoop_dma =  (struct ast_snoop_dma_data *)data;
	SNOOP_DMA_DBG("\n");

	snoop_dma_sts = readl(ast_snoop_dma->pccr2);

	if(snoop_dma_sts & LPC_POST_CODE_DMA_RDY) {
		//clear isr
		writel(LPC_POST_CODE_DMA_RDY, ast_snoop_dma->pccr2);
		//trigger snoop dma data
		tasklet_schedule(&ast_snoop_dma->snoop_tasklet);

	} else {
		printk("ast_snoop_dma_handler TODO ~~ \n");
	}
}
/***********************************************************************/
static int ast_snoop_dma_probe(struct platform_device *pdev)
{

	int ret=0;
	struct ast_snoop_dma_data *ast_snoop_dma = register_snoop_dma_drv();

	ast_snoop_dma->pccr0 = ast_snoop_dma->ast_lpc->reg_base + 0x130;
	ast_snoop_dma->pccr1 = ast_snoop_dma->ast_lpc->reg_base + 0x134;
	ast_snoop_dma->pccr2 = ast_snoop_dma->ast_lpc->reg_base + 0x138;
	ast_snoop_dma->pccr3 = ast_snoop_dma->ast_lpc->reg_base + 0x13C;
	ast_snoop_dma->pccr4 = ast_snoop_dma->ast_lpc->reg_base + 0xD0;
	ast_snoop_dma->pccr5 = ast_snoop_dma->ast_lpc->reg_base + 0xD4;
	ast_snoop_dma->pccr6 = ast_snoop_dma->ast_lpc->reg_base + 0xC4;

	SNOOP_DMA_DBG("ast_snoop_dma_probe\n");

	ast_snoop_dma->pdev = pdev;
	ast_snoop_dma->miscdev.minor = MISC_DYNAMIC_MINOR;

	writel(0, ast_snoop_dma->pccr0);

	ast_snoop_dma->dma_virt = dma_alloc_coherent(NULL,
								SNOOP_DMA_SIZE, &ast_snoop_dma->dma_addr, GFP_KERNEL);

	writel(ast_snoop_dma->dma_addr, ast_snoop_dma->pccr4);
	SNOOP_DMA_DBG("dma addr %x \n", ast_snoop_dma->dma_addr);
	//dma size //design bug : could not equal to 8K/4 length
	writel(SNOOP_DMA_SIZE/4, ast_snoop_dma->pccr5);
	ret = sysfs_create_group(&pdev->dev.kobj, &snoop_dma_attribute_group);
	if (ret)
		goto err_free_mem;

	ast_snoop_dma->miscdev.parent = &pdev->dev;
	ast_snoop_dma->miscdev.name = "ast-snoop-dma";
	ret = misc_register(&ast_snoop_dma->miscdev);
	if (ret){
		printk(KERN_ERR "SNOOP DMA : failed to request interrupt\n");
		goto out;
	}

	tasklet_init(&ast_snoop_dma->snoop_tasklet, ast_snoop_dma_tasklet_func,
			(unsigned long)ast_snoop_dma);

	request_snoop_dma_irq(ast_snoop_dma_handler);

	//default port is 0x80
	writel(0x80, ast_snoop_dma->pccr1);

	//init snoop dma - default 1-byte mode
	ast_snoop_dma->snoop_mode = 0;
	ast_snoop_dma->port0_rd_idx = 0;
	ast_snoop_dma->port0_wr_idx = 0;
	ast_snoop_dma->port1_rd_idx = 0;
	ast_snoop_dma->port1_wr_idx = 0;
	ast_snoop_dma->port2_rd_idx = 0;
	ast_snoop_dma->port2_wr_idx = 0;
	ast_snoop_dma->port3_rd_idx = 0;
	ast_snoop_dma->port3_wr_idx = 0;

	ast_snoop_dma->snoop_index = 0;

	spin_lock_init(&ast_snoop_dma->snoop_lock);

//	writel((readl(ast_snoop_dma->pccr0) & ~LPC_POST_CODE_MODE_MASK) | LPC_POST_CODE_MODE(BYTE_MODE) |
//			LPC_POST_DMA_MODE_EN |LPC_POST_DMA_INT_EN, ast_snoop_dma->pccr0);

	writel(LPC_RX_FIFO_CLR | LPC_POST_DMA_MODE_EN |LPC_POST_DMA_INT_EN, ast_snoop_dma->pccr0);


	platform_set_drvdata(pdev, ast_snoop_dma);
	dev_set_drvdata(ast_snoop_dma->miscdev.this_device, ast_snoop_dma);

	printk(KERN_INFO "ast_snoop_dma.%d: driver successfully loaded.\n",pdev->id);

	return 0;

err_free_mem:
//	dma_free_coherent

out:
	printk(KERN_WARNING "ast_snoop_dma: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_snoop_dma_remove(struct platform_device *pdev)
{
	struct ast_snoop_dma_data *ast_snoop_dma = platform_get_drvdata(pdev);
	SNOOP_DMA_DBG("ast_snoop_dma_remove\n");

	misc_deregister(&ast_snoop_dma->miscdev);
	kfree(ast_snoop_dma);

	return 0;
}

static struct platform_driver ast_snoop_dma_driver = {
	.remove 		= ast_snoop_dma_remove,
	.driver         = {
		.name   = "ast-snoop-dma",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_snoop_dma_driver, ast_snoop_dma_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST SNOOP DMA driver");
MODULE_LICENSE("GPL");
