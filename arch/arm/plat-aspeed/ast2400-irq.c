/*
 *  linux/arch/arm/plat-aspeed/ast2400-irq.c
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
#include <linux/irq.h>

#include <mach/hardware.h>
#include <asm/mach/irq.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <plat/regs-ast2400-vic.h>

#define irq_to_bmc_vic(irq_no)	(irq_no-IRQ_EXT_CHAIN_START)

static u32 ast2400_vic_base;

static void ast2400_mask_irq(unsigned int irq)
{
	int i=0;
	u32 regVal;
//	printk("ast2400_mask_irq %d\n",irq);
	irq = irq_to_bmc_vic(irq);	

	if (irq > 32) {
		i=1;
		irq = irq - 32;
	} else
		i=0;
	
//	printk("ast2400_mask_irq 1 %d\n",irq);		
	regVal = readl(AST_INTR_DIS(ast2400_vic_base, i));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_DIS(ast2400_vic_base, i));

}

static void ast2400_unmask_irq(unsigned int irq)
{
	int i=0;
	u32 regVal;
//	printk("ast2400_unmask_irq 0 %d\n",irq);	
	irq = irq_to_bmc_vic(irq);
//	printk("ast2400_unmask_irq 1 %d\n",irq);	

	if (irq > 32) {
		i=1;
		irq = irq - 32;
	} else
		i=0;
	
	regVal = readl(AST_INTR_EN(ast2400_vic_base, i));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_EN(ast2400_vic_base, i));
}

static struct irq_chip ast2400_irq_chip = {
	.name	= "ast2400 irq",
	.ack	= ast2400_mask_irq,
	.mask	= ast2400_mask_irq,
	.unmask = ast2400_unmask_irq,
}; 

static void
ast2400_handle_irq(unsigned int irq, struct irq_desc *desc)
{
	int i,cvic_irq=0;
    unsigned long sts = readl(AST_IRQ_STS(ast2400_vic_base, 0));

	if(irq != IRQ_EXT_CHAIN)
		BUG();

	desc->chip->ack(IRQ_EXT_CHAIN);	

    if (sts == 0) {
            do_bad_IRQ(irq, desc);
            return;
    }

    do {
			for(i=0; i<ARCH_NR_EXT_BMC; i++) {
				if(i<32) {
					if((1<<i)& readl(AST_IRQ_STS(ast2400_vic_base, 0))) {
						cvic_irq =i;
						break;
					}
				} else {
					if((1<<(i-32))& readl(AST_IRQ_STS(ast2400_vic_base, 0))) {
						cvic_irq =i;
						break;
					}

				}
			}
            cvic_irq += IRQ_EXT_CHAIN_START;
			//dispatch IRQ 
//			printk("dispatch ast2400 IRQ %d\n",cvic_irq);
            generic_handle_irq(cvic_irq);
			
    } while (readl(AST_IRQ_STS(ast2400_vic_base, 0)));
//	printk("unmask \n");
	desc->chip->unmask(IRQ_EXT_CHAIN);

}

static int __init ast2400_init_irq(void)
{
	unsigned int i;
	ast2400_vic_base = ioremap(AST_PCI_EXT_VIC, SZ_4K);

	writel(0, AST_INTR_SEL(ast2400_vic_base, 0));
	writel(0, AST_INTR_EN(ast2400_vic_base, 0));
	writel(0xFFFFFFFF, AST_INTR_DIS(ast2400_vic_base, 0));
	writel(0xFFFFFFFF, AST_INTR_EDGE_CLR(ast2400_vic_base, 0));

	writel(0, AST_INTR_SEL(ast2400_vic_base, 1));
	writel(0, AST_INTR_EN(ast2400_vic_base, 1));
	writel(0xFFFFFFFF, AST_INTR_DIS(ast2400_vic_base, 1));
	writel(0xFFFFFFFF, AST_INTR_EDGE_CLR(ast2400_vic_base, 1));
	
	//AST1070 total IRQ# 25 
	for (i = 0; i < ARCH_NR_EXT_BMC; i++) 
	{
		if(i<32) {
			IRQ_SET_HIGH_LEVEL(ast2400_vic_base, 0,i);
			IRQ_SET_LEVEL_TRIGGER(ast2400_vic_base, 0,i);
		} else {
			IRQ_SET_HIGH_LEVEL(ast2400_vic_base, 1,i-32);
			IRQ_SET_LEVEL_TRIGGER(ast2400_vic_base, 1,i-32);
		}
	
		set_irq_chip(i + IRQ_EXT_CHAIN_START, &ast2400_irq_chip);
		set_irq_handler(i + IRQ_EXT_CHAIN_START, handle_level_irq);
		set_irq_flags(i + IRQ_EXT_CHAIN_START, IRQF_VALID);
	}
	set_irq_chained_handler(IRQ_EXT_CHAIN, ast2400_handle_irq);
	return 0;
}

arch_initcall(ast2400_init_irq);
