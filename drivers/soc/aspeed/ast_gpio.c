/********************************************************************************
* File Name     : driver/char/asped/ast_gpio.c 
* Author         : Ryan Chen
* Description   : AST GPIO IRQ test driver 
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
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <asm/io.h>

#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/interrupt.h>
#include <mach/irqs.h>

static irqreturn_t gpio_handler ( int irq, void *dev_instance)
{
	printk("GPIO IRQ .........\n");


	return (IRQ_HANDLED);
}

int __init gpio_module_init(void)
{
	int		retVal;

	printk("gpio irq init \n ");

	retVal = request_irq(IRQ_GPIOB1, gpio_handler, IRQF_SHARED, "gpioB1", 0x30); 

	if (retVal < 0) {
	        printk ("Unable to get GPIO IRQ\n");
	}


	return 0;       /* Return 1 to not load the module */
}

void __exit gpio_module_exit(void)
{
	return 0;
}


static int gpio_module_open(struct inode *inode, struct file *filp)
{

	return 0;
}



/* ------------------------- Module Information Follows -------------------- */
module_init (gpio_module_init);
module_exit (gpio_module_exit);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("GPIO IRQ test driver for BMC appliance");
MODULE_LICENSE("GPL");

