/********************************************************************************
* File Name     : ast_snoop.c
* Author         : Ryan Chen
* Description   : AST SNOOP Controller
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
* Example for GPIO LED 
GPIO LED
1. LPC old snoop function #0
1a. 1e789090 D[15:0] = 80, port 80
1b. 1e789080 D[0] = 1, enable
2. GPIO for LPC
2a. We use GPIOC0~GPIOC7 which multipined with SD1 controller
2b. 1e6e2090 D[0] = 0, disable SD1
2c. 1e6e2090 D[26:23] = 0, disable I2C 10~13
2d. 1e780004 D[23:16] = ff, for GPIOC output pin
2e. 1e78005c D[2] = 1, GPIOC for LPC(A0), 1e780060 = 0x10000, 1e780064 = 0 for A1
3. GPIO for 80h
3a. 1e789080 D[31] = 1
3b. 1e789080 D[30] = 1
3c. 1e789080 D[29] = 0, non-inverse
3d. 1e789080 D[28:24] = 2, GPIOC

3. Super I/O (use the debug.exe DOS utility)
o 2e a5
o 2e a5
o 2e 7
o 2f 7 --> Device 7
o 2e 30
o 2f 80 --> D[7] = 1, Enable port 80h
o 2e 38
o 2f 2 --> D[4:0] select GPIO group, GPIOC = 2 and D[7] = inverse bit

then you can use debug utility to test the 80 port data
o 80 ff
o 80 0

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

//#define CONFIG_AST_SNOOP_DEBUG

#ifdef CONFIG_AST_SNOOP_DEBUG
	#define SNOOP_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define SNOOP_DBG(fmt, args...)
#endif

/**************************   LPC  Snoop Sys fs   **********************************************************/
//LPC port to GPIO Port source 
static ssize_t 
store_snoop_p2gpio_source(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	input_val = StrToHex(sysfsbuf);
	ast_set_lpc2gpio(ast_snoop->ast_lpc, ast_snoop->pdev->id, input_val);
	return count;
}

static ssize_t 
show_snoop_p2gpio_source(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n",ast_get_lpc2gpio(ast_snoop->ast_lpc, ast_snoop->pdev->id));
}
static DEVICE_ATTR(p2gpio_source, S_IRUGO | S_IWUSR, show_snoop_p2gpio_source, store_snoop_p2gpio_source);

//LPC port to GPIO Enable
static ssize_t 
store_snoop_p2gpio_enable(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	ast_set_lpc2gpio_en(ast_snoop->ast_lpc, ast_snoop->pdev->id, input_val);
	return count;
}

static ssize_t 
show_snoop_p2gpio_enable(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%d : %s\n", ast_get_lpc2gpio_en(ast_snoop->ast_lpc, ast_snoop->pdev->id),ast_get_lpc2gpio_en(ast_snoop->ast_lpc,ast_snoop->pdev->id) ? "Enable":"Disable");
}
static DEVICE_ATTR(p2gpio_enable, S_IRUGO | S_IWUSR, show_snoop_p2gpio_enable, store_snoop_p2gpio_enable);

static ssize_t 
show_snoop_data(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n",ast_get_snoop_data(ast_snoop->ast_lpc, ast_snoop->pdev->id));
}
static DEVICE_ATTR(data, S_IRUGO, show_snoop_data, NULL);

static ssize_t 
store_snoop_port(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	input_val = StrToHex(sysfsbuf);
	ast_set_snoop_port(ast_snoop->ast_lpc, ast_snoop->pdev->id, input_val);
	return count;
}

static ssize_t 
show_snoop_port(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%x\n",ast_get_snoop_port(ast_snoop->ast_lpc, ast_snoop->pdev->id));
}
static DEVICE_ATTR(port, S_IRUGO | S_IWUSR, show_snoop_port, store_snoop_port);

static ssize_t 
store_snoop_en(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	input_val = simple_strtoul(sysfsbuf, NULL, 10);
	ast_set_snoop_en(ast_snoop->ast_lpc, ast_snoop->pdev->id, input_val);
	return count;
}

static ssize_t 
show_snoop_en(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_snoop_data *ast_snoop = dev_get_drvdata(dev);
	return sprintf(sysfsbuf, "%d : %s\n", ast_get_snoop_en(ast_snoop->ast_lpc, ast_snoop->pdev->id),ast_get_snoop_en(ast_snoop->ast_lpc, ast_snoop->pdev->id) ? "Enable":"Disable");
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, show_snoop_en, store_snoop_en);


static struct attribute *ast_snoop_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_port.attr,
	&dev_attr_data.attr,	
	&dev_attr_p2gpio_enable.attr,	
	&dev_attr_p2gpio_source.attr,		
//	&dev_attr_buff.attr,		
	NULL
};

static const struct attribute_group snoop_attribute_group = {
	.attrs = ast_snoop_attributes,
};

/**************************   LPC  Snoop Sys fs END  **********************************************************/
static void ast_snoop_handler(void *data)
{
	struct ast_snoop_data *ast_snoop = (struct ast_snoop_data *)data;

	ast_snoop->write_idx++;
	ast_snoop->write_idx %= SNOOP_FIFO_SIZE;
	if(ast_snoop->write_idx == ast_snoop->read_idx) {
		ast_snoop->read_idx++;
		ast_snoop->read_idx %= SNOOP_FIFO_SIZE;
	}
}

static int ast_snoop_probe(struct platform_device *pdev)
{
	int ret=0;
	char snoop_name[15];
	struct ast_snoop_data *ast_snoop = register_snoop_drv(pdev->id);

	SNOOP_DBG("ast_snoop_probe\n");	

	ast_snoop->pdev = pdev;
	ast_snoop->miscdev.minor = MISC_DYNAMIC_MINOR;

	snprintf(snoop_name, sizeof(snoop_name), "ast-snoop.%d", pdev->id);

	ret = sysfs_create_group(&pdev->dev.kobj, &snoop_attribute_group);
	if (ret)
		goto out;

	ast_snoop->miscdev.parent = &pdev->dev;
	ast_snoop->miscdev.name = snoop_name;
	ret = misc_register(&ast_snoop->miscdev);
	if (ret){		
		printk(KERN_ERR "KCS : failed to request interrupt\n");
		goto out;
	}

	request_snoop_irq(pdev->id, ast_snoop_handler);

	platform_set_drvdata(pdev, ast_snoop);		

	dev_set_drvdata(ast_snoop->miscdev.this_device, ast_snoop);	

	printk(KERN_INFO "ast_snoop.%d: driver successfully loaded.\n",pdev->id);

	return 0;

out:
	printk(KERN_WARNING "ast_snoop: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_snoop_remove(struct platform_device *pdev)
{
	struct ast_snoop_data *ast_snoop = platform_get_drvdata(pdev);

	SNOOP_DBG("ast_snoop_remove\n");

	misc_deregister(&ast_snoop->miscdev);
	kfree(ast_snoop);

	return 0;	
}

static struct platform_driver ast_snoop_driver = {
	.remove 		= ast_snoop_remove,
	.driver         = {
		.name   = "ast-snoop",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_snoop_driver, ast_snoop_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST SNOOP driver");
MODULE_LICENSE("GPL");
