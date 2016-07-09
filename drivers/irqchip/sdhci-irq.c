/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/sdhci-irq.c
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
#include <plat/ast-scu.h>
#endif

//#define AST_SDHCI_IRQ_DEBUG

#ifdef AST_SDHCI_IRQ_DEBUG
#define SDHCI_IRQ_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define SDHCI_IRQ_DBUG(fmt, args...)
#endif

/*******************************************************************/
#define AST_SDHCI_INFO				0x00
#define AST_SDHCI_BLOCK			0x04
#define AST_SDHCI_ISR				0xFC

/* #define AST_SDHCI_INFO				0x00*/
#define AST_SDHCI_S1MMC8			(1 << 25)
#define AST_SDHCI_S0MMC8			(1 << 24)

/*******************************************************************/

void __iomem	*sdhci_reg_base;	

void ast_sd_set_8bit_mode(u8 mode)
{
	if(mode)
		writel( (1 << 24) | readl(sdhci_reg_base), sdhci_reg_base);
	else
		writel( ~(1 << 24) & readl(sdhci_reg_base), sdhci_reg_base);
}

EXPORT_SYMBOL(ast_sd_set_8bit_mode);

static void ast_sdhci_isr(unsigned int irq, struct irq_desc *desc)
{
	u32 sts = readl(sdhci_reg_base + AST_SDHCI_ISR);
	SDHCI_IRQ_DBUG("ast_sdhci_isr %x \n",sts);
	//should use run-roubin sdhci

	if(sts & 1)
		generic_handle_irq(IRQ_SDHCI_SLOT0);

	if(sts & 2)
		generic_handle_irq(IRQ_SDHCI_SLOT1);
	
}		

static void ast_sdhci_ack_irq(struct irq_data *d)
{
	SDHCI_IRQ_DBUG("ack irq[%d]\n",d->irq);
}

static void ast_sdhci_mask_irq(struct irq_data *d)
{
	SDHCI_IRQ_DBUG("mask irq[%d]\n",d->irq);
}

static void ast_sdhci_unmask_irq(struct irq_data *d)
{
	SDHCI_IRQ_DBUG("unmask irq[%d]\n",d->irq);
}

static struct irq_chip ast_sdhci_irq_chip = {
	.name		= "sdhci_irq_chip",
	.irq_ack		= ast_sdhci_ack_irq,
	.irq_mask		= ast_sdhci_mask_irq,
	.irq_unmask	= ast_sdhci_unmask_irq,
};

static int __init ast_sdhci_irq_init(void)
{
	int irq = 0;
	SDHCI_IRQ_DBUG("ast_sdhci_irq_init \n");

#if defined(CONFIG_MMC_AST) || defined(CONFIG_MMC_AST_MODULE)
	ast_scu_init_sdhci();

	sdhci_reg_base = ioremap(AST_SDHC_BASE, SZ_256);
	if (!sdhci_reg_base) {
		printk("ast_sdhci_irq_init ERROR \n");
		return -1;
	}
	
	for (irq = 0; irq < ARCH_NR_SDHCI; irq++) {
		irq_set_chip_and_handler(irq + IRQ_SDHCI_CHAIN_START, &ast_sdhci_irq_chip,
					 handle_level_irq);
		set_irq_flags(irq + IRQ_SDHCI_CHAIN_START, IRQF_VALID);
	}

	irq_set_chained_handler(IRQ_SDHC, ast_sdhci_isr);
#endif

	return 0;
}
core_initcall(ast_sdhci_irq_init);

