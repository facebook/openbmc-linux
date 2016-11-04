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
#include <linux/irq.h>

#include <mach/hardware.h>
#include <asm/mach/irq.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <plat/regs-ast1070-intc.h>

//#define AST1070_IRQ_DEBUG

#ifdef AST1070_IRQ_DEBUG
#define CIRQ_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define CIRQ_DBUG(fmt, args...)
#endif

static DEFINE_SPINLOCK(ast1070_vic_lock);

struct ast1070_vic_data {
	unsigned char chip;
	unsigned int irq_chain;
	unsigned int irq_chain_start;		
	void __iomem *reg_base;
};

static struct ast1070_vic_data vic_data[MAX_AST1070_NR];

static void ast1070_vic_mask_irq(unsigned int irq)
{
	u32 regVal;
	u32 vic;
    struct ast1070_vic_data *vic_data = get_irq_chip_data(irq);	
	spin_lock(&ast1070_vic_lock);	
	vic = irq - vic_data->irq_chain_start;
	CIRQ_DBUG("c[%d] : irq %d, vic %d\n",vic_data->chip, irq, vic);
	regVal = readl(vic_data->reg_base + AST1070_INTR_DIS);
	regVal |= (1 << vic);
	writel(regVal, vic_data->reg_base + AST1070_INTR_DIS);
	spin_unlock(&ast1070_vic_lock);	
}

static void ast1070_vic_unmask_irq(unsigned int irq)
{
	u32 regVal;
	u32 vic;
	struct ast1070_vic_data *vic_data = get_irq_chip_data(irq);	
	spin_lock(&ast1070_vic_lock);		
	vic = irq - vic_data->irq_chain_start;
	CIRQ_DBUG("c[%d] : irq %d, vic %d\n",vic_data->chip, irq, vic);
	regVal = readl(vic_data->reg_base + AST1070_INTR_EN);
	regVal |= (1 << vic);
	writel(regVal, vic_data->reg_base + AST1070_INTR_EN);
	spin_unlock(&ast1070_vic_lock);	
}

static struct irq_chip ast1070_vic_chip = {
	.name	= "ast1070_vic",
	.ack	= ast1070_vic_mask_irq,
	.mask	= ast1070_vic_mask_irq,
	.unmask = ast1070_vic_unmask_irq,
}; 

static void
ast1070_vic_handle_irq(unsigned int irq, struct irq_desc *desc)
{
	int i,cvic_irq=0;
	struct ast1070_vic_data *vic_data = get_irq_data(irq);
//	struct irq_chip *chip = get_irq_chip(irq);

	spin_lock(&ast1070_vic_lock);		
    u32 sts = readl(vic_data->reg_base + AST1070_IRQ_STS);
	spin_unlock(&ast1070_vic_lock);		

	CIRQ_DBUG("c[%d], sts : %x \n",vic_data->chip, sts);

	if(irq != vic_data->irq_chain)
		BUG();

	CIRQ_DBUG("ack %d \n",vic_data->irq_chain);
	desc->chip->ack(vic_data->irq_chain);	

    if (sts == 0) {
        do_bad_IRQ(irq, desc);
        return;
    }

   do {
		for(i=0; i<AST_CVIC_NUM; i++) {
			if((1<<i)& readl(vic_data->reg_base + AST1070_IRQ_STS)) {
				cvic_irq =i;
				break;
			}
		}
        cvic_irq += vic_data->irq_chain_start;
		//dispatch IRQ 
		CIRQ_DBUG("dispatch ast1070 IRQ %d\n",cvic_irq);
        generic_handle_irq(cvic_irq);			
    } while (readl(vic_data->reg_base + AST1070_IRQ_STS));

	CIRQ_DBUG("unmask %d \n",vic_data->irq_chain);
	desc->chip->unmask(vic_data->irq_chain);

}

void __init ast1070_vic_init(u8 vic_nr, u32 base, unsigned int irq_chain, unsigned int irq_chain_start)
{
	unsigned int i;
	vic_data[vic_nr].chip = vic_nr;
	switch(vic_nr) {
		case 0:
			vic_data[0].reg_base = ioremap(AST1070_C0_VIC_BASE, SZ_256);
			break;
		case 1:
			vic_data[1].reg_base = ioremap(AST1070_C1_VIC_BASE, SZ_256);
			break;
	}
	
	vic_data[vic_nr].irq_chain = irq_chain;
	vic_data[vic_nr].irq_chain_start = irq_chain_start;

	CIRQ_DBUG("c[%d], chain %d, chain start : %d \n",vic_nr, irq_chain, irq_chain_start);

	/* CVIC */
	writel(0, vic_data[vic_nr].reg_base + AST1070_INTR_EN);
	writel(0xFFFFFFFF, vic_data[vic_nr].reg_base + AST1070_INTR_DIS);

	/*
	 * Setup the Linux IRQ subsystem.
	 */
	for (i = 0; i < AST_CVIC_NUM; i++) {
		writel(readl(vic_data[vic_nr].reg_base + AST1070_INTR_EVENT) | (1 << (i)), (vic_data[vic_nr].reg_base + AST1070_INTR_EVENT)); 
		writel(readl(vic_data[vic_nr].reg_base + AST1070_INTR_SENSE) | (1 << (i)), (vic_data[vic_nr].reg_base + AST1070_INTR_SENSE)); 
		set_irq_chip(i + vic_data[vic_nr].irq_chain_start, &ast1070_vic_chip);
		set_irq_chip_data(i + vic_data[vic_nr].irq_chain_start, &vic_data[vic_nr]);
		set_irq_handler(i + vic_data[vic_nr].irq_chain_start, handle_level_irq);
		set_irq_flags(i + vic_data[vic_nr].irq_chain_start, IRQF_VALID);
	}

	if (set_irq_data(vic_data[vic_nr].irq_chain, &vic_data[vic_nr]) != 0)
		BUG();

	set_irq_chained_handler(vic_data[vic_nr].irq_chain, ast1070_vic_handle_irq);
	
}
