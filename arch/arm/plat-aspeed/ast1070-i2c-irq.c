/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-i2c.c
* Author        : Ryan chen
* Description   : ASPEED I2C Device
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/07/30 ryan chen create this file
*
********************************************************************************/

#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/sizes.h>
#include <asm/arch/devs.h>
#include <asm/arch/platform.h>
#include <asm/arch/irqs.h>
#include <asm/arch/aspeed.h>
#else
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/ast_i2c.h>
#include <plat/regs-iic.h>
#include <plat/ast-scu.h>
#endif

static void __iomem ci2c_reg_base[MAX_AST1070_NR];	

static void ast_ci2c_global_interrupt(unsigned int irq, struct irq_desc *desc)
{
//	u32 i = 0;
	u32 sts = readl(i2c_reg_base[i] + AST_I2CG_ISR);
	printk("ast_i2c_global_interrupt %x \n",sts);
	//should use run-roubin i2c
#if 0	
	for (i = 0; sts != 0; i++, stat >>= 1) {
		if (stat & 1) {
			printk("gen irq %d\n", i);
			generic_handle_irq(IRQ_I2C_CHAIN_START + i);
		}
	}	
#endif

	if(sts & AST_I2C_DEV0_IRQ) {
		printk("ISR_DEVICE1\n");
		generic_handle_irq(IRQ_I2C_DEV0);
	}

	if(sts & AST_I2C_DEV1_IRQ) {
		printk("ISR_DEVICE2\n");
		generic_handle_irq(IRQ_I2C_DEV1);
	}

	if(sts & AST_I2C_DEV2_IRQ) {
		printk("ISR_DEVICE3\n");
		generic_handle_irq(IRQ_I2C_DEV2);
	}

	if(sts & AST_I2C_DEV3_IRQ) {
		printk("ISR_DEVICE4\n");
		generic_handle_irq(IRQ_I2C_DEV3);
	}

	if(sts & AST_I2C_DEV4_IRQ) {
		printk("ISR_DEVICE5\n");
		generic_handle_irq(IRQ_I2C_DEV4);
	}

	if(sts & AST_I2C_DEV5_IRQ) {
		printk("ISR_DEVICE6\n");
		generic_handle_irq(IRQ_I2C_DEV5);
	}

	if(sts & AST_I2C_DEV6_IRQ) {
		printk("ISR_DEVICE7\n");
		generic_handle_irq(IRQ_I2C_DEV6);
	}

	if(sts & AST_I2C_DEV7_IRQ) {
		printk("ISR_DEVICE8\n");
		generic_handle_irq(IRQ_I2C_DEV7);
	}

	if(sts & AST_I2C_DEV8_IRQ) {
		printk("ISR_DEVICE9\n");
		generic_handle_irq(IRQ_I2C_DEV8);
	}

	if(sts & AST_I2C_DEV9_IRQ) {
		printk("ISR_DEVICE10\n");
		generic_handle_irq(IRQ_I2C_DEV9);
	}

	if(sts & AST_I2C_DEV10_IRQ) {
		printk("ISR_DEVICE11\n");
		generic_handle_irq(IRQ_I2C_DEV10);
	}

	if(sts & AST_I2C_DEV11_IRQ) {
		printk("ISR_DEVICE12\n");
		generic_handle_irq(IRQ_I2C_DEV11);
	}

	if(sts & AST_I2C_DEV12_IRQ) {
		printk("ISR_DEVICE13\n");
		generic_handle_irq(IRQ_I2C_DEV12);
	}

	if(sts & AST_I2C_DEV13_IRQ) {
		printk("ISR_DEVICE14\n");
		generic_handle_irq(IRQ_I2C_DEV13);
	}

}		


static void ast_ci2c_ack_irq(struct irq_data *d)
{
	printk("ack irq[%d]\n",d->irq);
}

static void ast_ci2c_mask_irq(struct irq_data *d)
{
	printk("mask irq[%d]\n",d->irq);
}

static void ast_ci2c_unmask_irq(struct irq_data *d)
{
	printk("unmask irq[%d]\n",d->irq);
}

static struct irq_chip ast_ci2c_irq_chip = {
	.name		= "ci2c_irq_chip",
	.irq_ack		= ast_ci2c_ack_irq,
	.irq_mask		= ast_ci2c_mask_irq,
	.irq_unmask	= ast_ci2c_unmask_irq,
};

void __init ast1070_i2c_irq_init(u8 nr, u32 base, unsigned int irq_chain, unsigned int irq_chain_start)
{
	int irq = 0;

	printk("ast_i2c_irq_init \n");
	//SCU I2C Reset 
	ast1070_scu_init_i2c(chip);

	ci2c_reg_base[nr] = ioremap(base, SZ_16);
	if (!ci2c_reg_base[nr]) {
		printk("ast_i2c_irq_init ERROR \n");
		return -1;
	}
	
	for (irq = 0; irq < ARCH_NR_AST1070_I2C; irq++) {
		irq_set_chip(irq + irq_chain, &ast_ci2c_irq_chip);
		irq_set_handler(irq + irq_chain, &handle_simple_irq);
		set_irq_flags(irq + irq_chain, IRQF_VALID);
	}

	irq_set_chained_handler(irq_chain, ast_ci2c_global_interrupt);

	return 0;
}
