/*
 *  linux/arch/arm/plat-aspeed/irq.c
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
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>

#include <plat/regs-intr.h>

static void ast_mask_irq(unsigned int irq)
{
	int i=0;
	u32 regVal;
	u8 timer;

#ifdef IRQ_TIMER7
	if(((irq >= IRQ_TIMER0) && (irq <= IRQ_TIMER2)) || ((i >= IRQ_TIMER3) && (i <= IRQ_TIMER7)))
		timer = 1;

#else
	if((irq >= IRQ_TIMER0) && (irq <= IRQ_TIMER2))
		timer = 1;
#endif
	
	if (irq > 32) {
		i=1;
		irq = irq - 32;
	} else
		i=0;
	
	regVal = readl(AST_INTR_DIS(i));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_DIS(i));

	/*
	 * clear the interrupt
	 */
	if(timer)
		IRQ_EDGE_CLEAR(i,irq);	
	
}

static void ast_unmask_irq(unsigned int irq)
{
	int i;
	u32 regVal;

	if (irq > 32) {
		i=1;
		irq = irq - 32;
	} else
		i=0;
	
	regVal = readl(AST_INTR_EN(i));
	regVal |= (1 << irq);
	writel(regVal, AST_INTR_EN(i));
}

static struct irq_chip ast_irq_chip = {
	.name	= "ast_irq",
	.ack	= ast_mask_irq,
	.mask	= ast_mask_irq,
	.unmask = ast_unmask_irq,
}; 
 
void __init ast_init_irq(void)
{
	unsigned int i;

	/* VIC1 */
	writel(0, AST_INTR_SEL(0));
	writel(0, AST_INTR_EN(0));
	writel(0xFFFFFFFF, AST_INTR_DIS(0));
	writel(0xFFFFFFFF, AST_INTR_EDGE_CLR(0));

#if defined(NEW_VIC)
	writel(0, AST_INTR_SEL(1));
	writel(0, AST_INTR_EN(1));
	writel(0xFFFFFFFF, AST_INTR_DIS(1));
	writel(0xFFFFFFFF, AST_INTR_EDGE_CLR(1));
#endif

	//TOTAL IRQ NUM = 
	for (i = 0; i < AST_VIC_NUM; i++) 
	{
		if(i<32) {
			if((i >= IRQ_TIMER0) && (i <= IRQ_TIMER2)) //Timer0/1/2
				IRQ_SET_RISING_EDGE(0,i);
			else {
				IRQ_SET_HIGH_LEVEL(0,i);
				IRQ_SET_LEVEL_TRIGGER(0,i);
			}
#ifdef IRQ_TIMER7
		} else {
			if((i >= IRQ_TIMER3) && (i <= IRQ_TIMER7)) //Timer3/4/5/6/7
				IRQ_SET_RISING_EDGE(0,i-32);
			else {
				IRQ_SET_HIGH_LEVEL(1,i-32);
				IRQ_SET_LEVEL_TRIGGER(1,i-32);
			}
#endif			
		}
			
		set_irq_chip(i, &ast_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}

}
