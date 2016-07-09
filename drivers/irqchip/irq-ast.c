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
#include <asm/mach/irq.h>

#include <asm/irq.h>


#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/bitmap.h>
#include <linux/types.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>

#include <asm/exception.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>


/***************************************************************************/
static void __iomem *ast_vic_base;
static struct irq_domain *ast_vic_domain;
static struct device_node *ast_vic_np;

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
static void ast_mask_ack_irq(struct irq_data *d)
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

static void ast_ack_irq(struct irq_data *d)
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

	/*
	 * clear the interrupt
	 */
	if(timer)
		IRQ_EDGE_CLEAR(idx, irq);	

	if(cpu)
		IRQ_SW_CLEAR(idx, irq);	
	
}

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

static int ast_irq_type(struct irq_data *d, unsigned int type)
{
	int idx = 0;
	u32 irq = d->irq;

	if (d->irq > 31) {
		idx=1;
		irq = d->irq - 32;
	} 
//	printk("ast_irq_type irq %d , type %x\n", d->irq, type);
	
	switch (type) {
		case IRQ_TYPE_NONE:
			break;
		case IRQ_TYPE_EDGE_RISING:
			IRQ_SET_RISING_EDGE(idx,irq);
			irq_set_handler(d->irq, handle_edge_irq);
			break;
		case IRQ_TYPE_EDGE_FALLING:
			IRQ_SET_FALLING_EDGE(idx,irq);
			irq_set_handler(d->irq, handle_edge_irq);
			break;		
		case IRQ_TYPE_EDGE_BOTH:
	//		irq_set_handler(d->irq, handle_edge_irq);
			break;
		case IRQ_TYPE_LEVEL_LOW:
		case IRQ_TYPE_LEVEL_HIGH:
			irq_set_handler(d->irq, handle_level_irq);
			break;
		default:
			pr_err("No such irq type %d", type);
			return -EINVAL;
	}

	return 0;
}

static struct irq_chip ast_irq_chip = {
	.name	= "ast-vic",
	.irq_mask_ack = ast_mask_ack_irq,
	.irq_ack	= ast_ack_irq,
	.irq_mask	= ast_mask_irq,
	.irq_unmask = ast_unmask_irq,
	.irq_set_type	= ast_irq_type,
//		.irq_set_wake	= ast_irq_wake	
}; 

asmlinkage void __exception_irq_entry
ast_vic_handle_irq(struct pt_regs *regs)
{
	u32 irqstat0, irqstat1;
	u32 irq;	

	irqstat0 = readl(AST_IRQ_STS(0));
	irqstat1 = readl(AST_IRQ_STS(1));

	/*
	 * ISR value is 0 when there is no current interrupt or when there is
	 * a spurious interrupt
	 */
	if (irqstat0) {
		irq = ffs(irqstat0) - 1;
		handle_IRQ(irq, regs);
	}

	if(irqstat1) {
		irq = ffs(irqstat1) - 1 + 32;
		handle_IRQ(irq, regs);		
	}
}

void __init ast_init_irq(void)
{
	int i;
	int irq_base;	

	ast_vic_base = ioremap(AST_VIC_BASE, SZ_256);
	if (!ast_vic_base)
		panic("Unable to ioremap AIC registers\n");


	for(i = 0; i < 2; i++) {
		writel(0, AST_INTR_SEL(i));
		writel(0, AST_INTR_EN(i));
		writel(0xFFFFFFFF, AST_INTR_DIS(i));
		writel(0xFFFFFFFF, AST_INTR_EDGE_CLR(i));
	}

	/* Add irq domain for VIC */
	irq_base = irq_alloc_descs(-1, 0, AST_VIC_NUM, 0);
	if (irq_base < 0) {
		WARN(1, "Cannot allocate irq_descs, assuming pre-allocated\n");
		irq_base = 0;
	}
	ast_vic_domain = irq_domain_add_legacy(ast_vic_np, AST_VIC_NUM,
						irq_base, 0,
						&irq_domain_simple_ops, NULL);

	if (!ast_vic_domain)
		panic("Unable to add VIC irq domain\n");

	irq_set_default_host(ast_vic_domain);
	
	for (i = 0; i < AST_VIC_NUM; i++) 
	{
		irq_set_chip_and_handler(i, &ast_irq_chip, handle_level_irq);
		irq_set_chip_data(i, ast_vic_base);
		set_irq_flags(i, IRQF_VALID);
	}

	set_handle_irq(ast_vic_handle_irq);

}
