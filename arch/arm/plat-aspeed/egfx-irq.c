/*
 *  linux/arch/arm/plat-aspeed/egfx-irq.c
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

//#define EGFX_VIC_DEBUG

#ifdef EGFX_VIC_DEBUG
#define EGFX_DEBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define EGFX_DEBUG(fmt, args...)
#endif

/***************************************************************************/
static void __iomem *egfx_vic_base;

#define ast_egfx_write(value, reg) \
	__raw_writel(value, egfx_vic_base + (reg))
#define ast_egfx_read(reg) \
	__raw_readl(egfx_vic_base + (reg))
/***************************************************************************/
#define AST_EGFX_IER			0x00
#define AST_EGFX_ISR			0x04
/***************************************************************************/
static void egfx_vic_mask_irq(struct irq_data *data)
{
	u32 irq = data->irq - IRQ_EGFX_CHAIN_START;
	ast_egfx_write(ast_egfx_read(AST_EGFX_IER) & ~(1 << irq), AST_EGFX_IER);
}

static void egfx_vic_unmask_irq(struct irq_data *data)
{
	u32 irq = data->irq - IRQ_EGFX_CHAIN_START;
	ast_egfx_write(ast_egfx_read(AST_EGFX_IER) | (1 << irq), AST_EGFX_IER);
}


static void egfx_vic_handler(unsigned int irq, struct irq_desc *desc)
{
	int i;
//	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 sts = ast_egfx_read(AST_EGFX_ISR);
	EGFX_DEBUG("sts : %x : ier %x \n", sts, ast_egfx_read(AST_EGFX_IER));
	if (unlikely(sts == 0)) {
		/* ack if we get an irq with nothing (ie, startup) */
		printk("TODO ... ~~~~~~~~~~\n");
//		desc = irq_desc + BAST_IRQ_ISA;
//		desc->irq_data.chip->irq_ack(&desc->irq_data);
	} else {
		/* handle the IRQ */
		for (i = 0; sts != 0; i++, sts >>= 1) {
			if (sts & 1) {
				generic_handle_irq(IRQ_EGFX_CHAIN_START + i);
			}
		}
	}
}		

static struct irq_chip ast_egfx_chip = {
	.name		= "egfx-irq",
	.irq_ack		= egfx_vic_mask_irq,
	.irq_mask		= egfx_vic_mask_irq,
	.irq_unmask 	= egfx_vic_unmask_irq,
}; 

static __init int egfx_irq_init(void)
{
	int i;

	egfx_vic_base = ioremap(AST_EGFX_SYS_BASE, SZ_16);
	ast_egfx_write(0, AST_EGFX_ISR);

	for(i=0; i<ARCH_NR_EGFX; i++) {
		irq_set_chip_and_handler(i + IRQ_EGFX_CHAIN_START, &ast_egfx_chip,
					 handle_level_irq);
		set_irq_flags(i + IRQ_EGFX_CHAIN_START, IRQF_VALID);
	}
	irq_set_chained_handler(IRQ_EGFX, egfx_vic_handler);

	return 0;
}

arch_initcall(egfx_irq_init);
