/*
 *  linux/arch/arm/plat-aspeed/ast1070_irq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/init.h>
#include <linux/stddef.h>
#include <linux/list.h>
#include <linux/timer.h>

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <asm/system.h>
#include <linux/irq.h>

#include <mach/hardware.h>
#include <asm/mach/irq.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <plat/regs-ast1070-intc.h>

#define irq_to_c0_vic(irq_no)	(irq_no-IRQ_C0_VIC_CHAIN_START)

static void ast1070_c0_mask_irq(unsigned int irq)
{
	u32 regVal;
//	printk("ast_c0_mask_irq %d\n",irq);
	irq = irq_to_c0_vic(irq);	
//	printk("ast_c0_mask_irq cvic %d\n",irq);
	regVal = readl(AST_INTR_DIS(0));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_DIS(0));

}

static void ast1070_c0_unmask_irq(unsigned int irq)
{
	u32 regVal;
//	printk("ast_c0_unmask_irq %d\n",irq);	
	irq = irq_to_c0_vic(irq);
//	printk("ast_c0_unmask_irq cvic %d\n",irq);	
	regVal = readl(AST_INTR_EN(0));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_EN(0));
}

static struct irq_chip ast1070_c0_irq_chip = {
	.name	= "ast1070_c0",
	.ack	= ast1070_c0_mask_irq,
	.mask	= ast1070_c0_mask_irq,
	.unmask = ast1070_c0_unmask_irq,
}; 

static void
ast1070_c0_handle_irq(unsigned int irq, struct irq_desc *desc)
{
	int i,cvic_irq=0;
    unsigned long sts = readl(AST_IRQ_STS(0));

	if(irq != IRQ_C0_VIC_CHAIN)
		BUG();

	desc->chip->ack(IRQ_C0_VIC_CHAIN);	

    if (sts == 0) {
            do_bad_IRQ(irq, desc);
            return;
    }

    do {
			for(i=0; i<AST_CVIC_NUM; i++) {
				if((1<<i)& readl(AST_IRQ_STS(0))) {
					cvic_irq =i;
					break;
				}
			}
            cvic_irq += IRQ_C0_VIC_CHAIN_START;
			//dispatch IRQ 
//			printk("dispatch ast1070 IRQ %d\n",cvic_irq);
            generic_handle_irq(cvic_irq);
			
    } while (readl(AST_IRQ_STS(0)));
	
	desc->chip->unmask(IRQ_C0_VIC_CHAIN);

}

static int __init ast1070_c0_init_irq(void)
{
	unsigned int i;
//	printk("ast1070_c0_init_irq **==== Start ---------------\n");	
	/* CVIC */
	writel(0, AST_INTR_EN(0));
	writel(0xFFFFFFFF, AST_INTR_DIS(0));

	//AST1070 total IRQ# 25 
	for (i = 0; i < AST_CVIC_NUM; i++) 
	{
		IRQ_SET_HIGH_LEVEL(0,i);
		IRQ_SET_LEVEL_TRIGGER(0,i);
		set_irq_chip(i + IRQ_C0_VIC_CHAIN_START, &ast1070_c0_irq_chip);
		set_irq_handler(i + IRQ_C0_VIC_CHAIN_START, handle_level_irq);
		set_irq_flags(i + IRQ_C0_VIC_CHAIN_START, IRQF_VALID);
	}
	set_irq_chained_handler(IRQ_C0_VIC_CHAIN, ast1070_c0_handle_irq);
//	printk("ast1070_init_irq **====  END ----------\n");	
	return 0;
}

arch_initcall(ast1070_c0_init_irq);

#if (CONFIG_AST1070_NR >= 2)
#define irq_to_c1_vic(irq_no)	(irq_no-IRQ_C1_VIC_CHAIN_START)

static void ast1070_c1_mask_irq(unsigned int irq)
{
	u32 regVal;
//	printk("ast_mask_irq %d\n",irq);
	irq = irq_to_c1_vic(irq);	
//	printk("ast_mask_irq cvic %d\n",irq);
	regVal = readl(AST_INTR_DIS(1));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_DIS(1));

}

static void ast1070_c1_unmask_irq(unsigned int irq)
{
	u32 regVal;
//	printk("ast_unmask_irq %d\n",irq);	
	irq = irq_to_c1_vic(irq);
//	printk("ast_unmask_irq cvic %d\n",irq);	
	regVal = readl(AST_INTR_EN(1));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_EN(1));
}

static struct irq_chip ast1070_c1_irq_chip = {
	.name	= "ast1070_c1",
	.ack	= ast1070_c1_mask_irq,
	.mask	= ast1070_c1_mask_irq,
	.unmask = ast1070_c1_unmask_irq,
}; 

static void
ast1070_c1_handle_irq(unsigned int irq, struct irq_desc *desc)
{
	int i,cvic_irq=0;
    unsigned long sts = readl(AST_IRQ_STS(1));

	if(irq != IRQ_C1_VIC_CHAIN)
		BUG();

	desc->chip->ack(IRQ_C1_VIC_CHAIN);	

    if (sts == 0) {
            do_bad_IRQ(irq, desc);
            return;
    }

    do {
			for(i=0; i<AST_CVIC_NUM; i++) {
				if((1<<i)& readl(AST_IRQ_STS(1))) {
					cvic_irq =i;
					break;
				}
			}
            cvic_irq += IRQ_C1_VIC_CHAIN_START;
			//dispatch IRQ 
//			printk("dispatch ast1070 IRQ %d\n",cvic_irq);
            generic_handle_irq(cvic_irq);
			
    } while (readl(AST_IRQ_STS(1)));

	desc->chip->unmask(IRQ_C1_VIC_CHAIN);

}

static int __init ast1070_c1_init_irq(void)
{
	unsigned int i;
//	printk("ast1070_c1_init_irq **==== Start ---------------\n");	
	/* CVIC */
	writel(0, AST_INTR_EN(1));
	writel(0xFFFFFFFF, AST_INTR_DIS(1));

	//AST1070 total IRQ# 25 
	for (i = 0; i < AST_CVIC_NUM; i++) 
	{
		IRQ_SET_HIGH_LEVEL(1,i);
		IRQ_SET_LEVEL_TRIGGER(1,i);
		set_irq_chip(i + IRQ_C1_VIC_CHAIN_START, &ast1070_c1_irq_chip);
		set_irq_handler(i + IRQ_C1_VIC_CHAIN_START, handle_level_irq);
		set_irq_flags(i + IRQ_C1_VIC_CHAIN_START, IRQF_VALID);
	}
	set_irq_chained_handler(IRQ_C1_VIC_CHAIN, ast1070_c1_handle_irq);
//	printk("ast1070_init_irq **====  END ----------\n");	
	return 0;
}

arch_initcall(ast1070_c1_init_irq);

#endif
