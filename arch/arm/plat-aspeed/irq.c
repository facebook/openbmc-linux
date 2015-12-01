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
 
#include <linux/irq.h>
#include <linux/io.h>
#include <mach/hardware.h>
/***************************************************************************/
static void __iomem *ast_vic_base;

#define ast_vic_write(value, reg) \
	__raw_writel(value, ast_vic_base + (reg))
#define ast_vic_read(reg) \
	__raw_readl(ast_vic_base + (reg))
/***************************************************************************/
#define AST_IRQ_STS(x)				(ast_vic_base + 0x80 + (x*0x04))
#define AST_FIQ_STS(x)				(ast_vic_base + 0x88 + (x*0x04))
#define AST_RAW_STS(x)				(ast_vic_base + 0x90 + (x*0x04))
#define AST_INTR_SEL(x)				(ast_vic_base + 0x98 + (x*0x04))
#define AST_INTR_EN(x)				(ast_vic_base + 0xA0 + (x*0x04))
#define AST_INTR_DIS(x)				(ast_vic_base + 0xA8 + (x*0x04))
#define AST_INTR_SW_EN(x)			(ast_vic_base + 0xB0 + (x*0x04))
#define AST_INTR_SW_CLR(x)			(ast_vic_base + 0xB8 + (x*0x04))
#define AST_INTR_SENSE(x)			(ast_vic_base + 0xC0 + (x*0x04))
#define AST_INTR_BOTH_EDGE(x)		(ast_vic_base + 0xC8 + (x*0x04))
#define AST_INTR_EVENT(x)				(ast_vic_base + 0xD0 + (x*0x04))
#define AST_INTR_EDGE_CLR(x)			(ast_vic_base + 0xD8 + (x*0x04))
#define AST_INTR_EDGE_STS(x)			(ast_vic_base + 0xE0 + (x*0x04))

#define IRQ_SET_LEVEL_TRIGGER(x, irq_no)   *((volatile unsigned long*)AST_INTR_SENSE(x)) |= 1 << (irq_no)
#define IRQ_SET_EDGE_TRIGGER(x, irq_no)    *((volatile unsigned long*)AST_INTR_SENSE(x)) &= ~(1 << (irq_no))
#define IRQ_SET_RISING_EDGE(x, irq_no)     *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_FALLING_EDGE(x, irq_no)    *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_SET_HIGH_LEVEL(x,irq_no)      *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_LOW_LEVEL(x, irq_no)       *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_EDGE_CLEAR(x, irq_no)          *((volatile unsigned long*)AST_INTR_EDGE_CLR(x)) |= 1 << (irq_no)
#define IRQ_SW_CLEAR(x, irq_no)          *((volatile unsigned long*)AST_INTR_SW_CLR(x)) |= 1 << (irq_no)

/***************************************************************************/
static void ast_mask_irq(struct irq_data *d)
{
	u8 timer;
	u8 cpu;
	int idx = 0;
	u32 irq = d->irq;
	

	if(((d->irq >= IRQ_TIMER0) && (d->irq <= IRQ_TIMER2)) || ((d->irq >= IRQ_TIMER3) && (d->irq <= IRQ_TIMER7)))
		timer = 1;

	if(d->irq == IRQ_CPU)
		cpu = 1;
	
	if (d->irq > 31) {
		idx=1;
		irq = d->irq - 32;
	} 
	
	writel(readl(AST_INTR_DIS(idx)) | (1 << irq), AST_INTR_DIS(idx));

	/*
	 * clear the interrupt
	 */
	if(timer)
		IRQ_EDGE_CLEAR(idx, irq);	

	if(cpu)
		IRQ_SW_CLEAR(idx, irq);	
	
}

static void ast_unmask_irq(struct irq_data *d)
{
	int idx = 0;
	u32 irq = d->irq;

	if (d->irq > 31) {
		idx=1;
		irq = d->irq - 32;
	} 

	writel(readl(AST_INTR_EN(idx)) | (1 << irq), AST_INTR_EN(idx));
}


static struct irq_chip ast_irq_chip = {
	.name	= "ast_irq",
	.irq_ack	= ast_mask_irq,
	.irq_mask	= ast_mask_irq,
	.irq_unmask = ast_unmask_irq,
}; 
 
void __init ast_init_irq(void)
{
	unsigned int i;

	ast_vic_base = (void *)IO_ADDRESS(AST_VIC_BASE);
	
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
			irq_set_chip_and_handler(i, &ast_irq_chip, handle_level_irq);
//			irq_set_chip_data(irq, base);
		} else {
			if((i >= IRQ_TIMER3) && (i <= IRQ_TIMER7)) //Timer3/4/5/6/7
				IRQ_SET_RISING_EDGE(0,i-32);
			else {
				IRQ_SET_HIGH_LEVEL(1,i-32);
				IRQ_SET_LEVEL_TRIGGER(1,i-32);
			}
			irq_set_chip_and_handler(i, &ast_irq_chip, handle_level_irq);
//			irq_set_chip_data(irq, base);
		}
		set_irq_flags(i, IRQF_VALID);
	}
	
}
