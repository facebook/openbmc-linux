/*
 *  linux/drivers/gpio/ast-sgpio.c
 *
 * Support functions for ASPEED SGPIO
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 * Written by Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/io.h>

#include <asm/mach/irq.h>
#include <mach/hardware.h>

#include <linux/platform_device.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/arch/regs-gpio.h>
#include <asm/arch/gpio.h>
#else
#include <plat/regs-gpio.h>
#include <mach/gpio.h>
#endif

//#define AST_SGPIO_DEBUG

#ifdef AST_SGPIO_DEBUG
//#define SGPIODBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#define SGPIODBUG(fmt, args...) printk(fmt, ## args)

#else
#define SGPIODBUG(fmt, args...)
#endif

/*************************************************************/
//GPIO group structure 
struct ast_sgpio_bank {
    int 	irq;
	u32  	base;	
//TODO remove base	
	u32  	index;	
	u32		data_offset;
	u32		data_read_offset;	
	u32		int_en_offset;	
	u32		int_type_offset;		
	u32		int_sts_offset;	
	u32		rst_tol_offset;		
	struct gpio_chip chip;
	
};

int ast_sgpio_to_irq(unsigned gpio)
{
	return (gpio + IRQ_SGPIO_CHAIN_START);
}

EXPORT_SYMBOL(ast_sgpio_to_irq);

int ast_irq_to_sgpio(unsigned irq)
{
	return (irq - IRQ_SGPIO_CHAIN_START);
}

EXPORT_SYMBOL(ast_irq_to_sgpio);

static inline int
ast_sgpio_read(struct ast_sgpio_bank *ast_sgpio ,u32 offset)
{
#ifdef AST_SGPIO_DEBUG
	u32 v = readl((void *)(ast_sgpio->base + offset));
	SGPIODBUG("read base = 0x%08x, offset = 0x%08x, val = %x \n", ast_sgpio->base, offset, v);
	return v;
#else
	return readl((void *)(ast_sgpio->base + offset));
#endif
}

static inline void
ast_sgpio_write(struct ast_sgpio_bank *ast_sgpio , u32 val, u32 offset)
{
    SGPIODBUG("base = 0x%08x, offset = 0x%08x, val = 0x%08x\n", ast_sgpio->base, offset, val);
    writel(val, (void *)(ast_sgpio->base + offset));
}

/***************************************************************************************/
static int ast_sgpio_configuration(struct ast_sgpio_bank *ast_sgpio)
{
	ast_sgpio_write(ast_sgpio, (ast_sgpio_read(ast_sgpio, AST_SGPIO_CTRL) & 0xffff0000) | AST_SGPIO_PIN(10) | AST_SGPIO_ENABLE, AST_SGPIO_CTRL);
	return 0;
}

static int
ast_sgpio_get(struct gpio_chip *chip, unsigned offset)
{
	unsigned long flags;
	u32 v;

	struct ast_sgpio_bank *ast_sgpio = container_of(chip, struct ast_sgpio_bank, chip);

	SGPIODBUG("Get %s[%d] \n",chip->label, offset);

	local_irq_save(flags);

	v = ast_sgpio_read(ast_sgpio, ast_sgpio->data_offset);
	v &= (1 << (offset + (ast_sgpio->index * 8)));

	if(v)
		v = 1;
	else
		v = 0;

	local_irq_restore(flags);

	return v;
}

static void
ast_sgpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct ast_sgpio_bank *ast_sgpio = container_of(chip, struct ast_sgpio_bank, chip);
	unsigned long flags;
	u32 v;
	SGPIODBUG("Set %s[%d] = %d, ast_sgpio->index = %d \n",chip->label, offset, val, ast_sgpio->index);

	local_irq_save(flags);

	v = ast_sgpio_read(ast_sgpio, ast_sgpio->data_read_offset);

	if (val)
		v |= (1 << (offset + (ast_sgpio->index * 8)));
	else
		v &= ~(1 << (offset + (ast_sgpio->index * 8)));

	ast_sgpio_write(ast_sgpio, v, ast_sgpio->data_offset);

	local_irq_restore(flags);
}
	

#define AST_SGPIO_BANK(name, index_no, data, read_data, int_en, int_type, int_sts, rst_tol)	\
	{												\
		.index = index_no,								\
		.data_offset = data,						\
		.data_read_offset = read_data,			\
		.int_en_offset = int_en,					\
		.int_type_offset = int_type,				\
		.int_sts_offset = int_sts, 					\
		.rst_tol_offset = rst_tol,					\
		.chip = {                            							\
	        .label                  = name,  							\
	        .get            = ast_sgpio_get,  \
	        .set            = ast_sgpio_set,  \
	        .ngpio          = GPIO_PER_PORT_PIN_NUM, \
		}, \
	}

static struct ast_sgpio_bank ast_sgpio_gp[] = {
	AST_SGPIO_BANK("SGPIOA", 0, 0x000, 0x070, 0x004, 0x008, 0x014, 0x018),
	AST_SGPIO_BANK("SGPIOB", 1, 0x000, 0x070, 0x004, 0x008, 0x014, 0x018),
	AST_SGPIO_BANK("SGPIOC", 2, 0x000, 0x070, 0x004, 0x008, 0x014, 0x018),
	AST_SGPIO_BANK("SGPIOD", 3, 0x000, 0x070, 0x004, 0x008, 0x014, 0x018),
	AST_SGPIO_BANK("SGPIOE", 0, 0x01C, 0x074, 0x020, 0x024, 0x030, 0x034),
	AST_SGPIO_BANK("SGPIOF", 1, 0x01C, 0x074, 0x020, 0x024, 0x030, 0x034),
	AST_SGPIO_BANK("SGPIOG", 2, 0x01C, 0x074, 0x020, 0x024, 0x030, 0x034),
	AST_SGPIO_BANK("SGPIOH", 3, 0x01C, 0x074, 0x020, 0x024, 0x030, 0x034),
	AST_SGPIO_BANK("SGPIOI", 0, 0x038, 0x078, 0x03C, 0x040, 0x04C, 0x050),
	AST_SGPIO_BANK("SGPIOJ", 1, 0x038, 0x078, 0x03C, 0x040, 0x04C, 0x050),	
};


/***************************************************************************************/

/*
 * We need to unmask the GPIO bank interrupt as soon as possible to
 * avoid missing GPIO interrupts for other lines in the bank.
 * Then we need to mask-read-clear-unmask the triggered GPIO lines
 * in the bank to avoid missing nested interrupts for a GPIO line.
 * If we wait to unmask individual GPIO lines in the bank after the
 * line's interrupt handler has been run, we may miss some nested
 * interrupts.
 */
static void 
ast_sgpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	u32 isr;
	int i,j;
	struct ast_sgpio_bank *ast_sgpio;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	if(irq != IRQ_SGPIO)
		BUG();

	SGPIODBUG("ast_sgpio_irq_handler %d \n ", irq);

	
	chained_irq_enter(chip, desc);

	for (i = 0; i < GPIO_PORT_NUM; i++) {
		ast_sgpio = &ast_sgpio_gp[i];
		isr = ast_sgpio_read(ast_sgpio, ast_sgpio->int_sts_offset);
		SGPIODBUG("isr %x \n", isr);
		isr = (isr >> (8 * ast_sgpio->index)) & 0xff;
		SGPIODBUG("[%s] isr %x \n", ast_sgpio->chip.label, isr);
		if(isr != 0) {
			//get gpio isr and --> to IRQ number ....
			for (j=0; j<8;j++) {
				if((1<<j) & isr) {
					generic_handle_irq(j + IRQ_SGPIO_CHAIN_START + (8 * i));
				}
			}
		}		
	}

	chained_irq_exit(chip, desc);

}

static void ast_sgpio_ack_irq(struct irq_data *d)
{
	struct ast_sgpio_bank *ast_sgpio = irq_get_chip_data(d->irq);

	unsigned int sgpio_irq = (d->irq - IRQ_SGPIO_CHAIN_START) % 8;

	SGPIODBUG("irq [%d] : ast_sgpio_ack_irq [%s] pin %d\n ",d->irq, ast_sgpio->chip.label, sgpio_irq);

	SGPIODBUG("write clr [%x] %x\n ",ast_sgpio->int_sts_offset, 1<< (sgpio_irq + (ast_sgpio->index * 8)));
		
	ast_sgpio_write(ast_sgpio, 1<< (sgpio_irq + (ast_sgpio->index * 8)), ast_sgpio->int_sts_offset);

	SGPIODBUG("read sts %x\n ",ast_sgpio_read(ast_sgpio, ast_sgpio->int_sts_offset));

}

static void ast_sgpio_mask_irq(struct irq_data *d)
{
	struct ast_sgpio_bank *ast_sgpio = irq_get_chip_data(d->irq);
	unsigned int sgpio_irq = (d->irq - IRQ_SGPIO_CHAIN_START) % 8;

	SGPIODBUG("irq [%d] : ast_sgpio_mask_irq [%s] pin %d\n ",d->irq, ast_sgpio->chip.label, sgpio_irq);

	//disable irq
	ast_sgpio_write(ast_sgpio, ast_sgpio_read(ast_sgpio, ast_sgpio->int_en_offset) &
			~(1<< (sgpio_irq + (ast_sgpio->index * 8))), ast_sgpio->int_en_offset);
}

static void ast_sgpio_unmask_irq(struct irq_data *d)
{
	struct ast_sgpio_bank *ast_sgpio = get_irq_chip_data(irq);
	u32 sgpio_irq = (irq - IRQ_SGPIO_CHAIN_START) % 8;

	SGPIODBUG("irq[%d], [%s] pin %d\n",d->irq, ast_sgpio->chip.label, sgpio_irq);

	//Enable IRQ ..
	ast_sgpio_write(ast_sgpio, 1<< (sgpio_irq + (ast_sgpio->index * 8)), ast_sgpio->int_sts_offset);

	ast_sgpio_write(ast_sgpio, ast_sgpio_read(ast_sgpio, ast_sgpio->int_en_offset) |
			(1<< (sgpio_irq + (ast_sgpio->index * 8))), ast_sgpio->int_en_offset);

}

static int
ast_sgpio_irq_type(unsigned int irq, unsigned int type)
{
	u32 type0, type1, type2;
	struct ast_sgpio_bank *ast_sgpio = irq_get_chip_data(irq);
	u32 sgpio_irq = (irq - IRQ_SGPIO_CHAIN_START) %8;

	SGPIODBUG("irq %d,  sgpio_irq %d , irq_type %x \n",irq, sgpio_irq, type);
	
	if (type & ~IRQ_TYPE_SENSE_MASK)
		return -EINVAL;

	type0 = ast_sgpio_read(ast_sgpio, ast_sgpio->int_type_offset);
	type1 = ast_sgpio_read(ast_sgpio, ast_sgpio->int_type_offset + 0x04);
	type2 = ast_sgpio_read(ast_sgpio, ast_sgpio->int_type_offset + 0x08);

	switch(type) {
		/* Edge rising type */
		case IRQ_TYPE_EDGE_RISING:
			type0 |=(1 << sgpio_irq);
			type1 &=~(1 << sgpio_irq);
			type2 &=~(1 << sgpio_irq);
			break;
		/* Edge falling type */
		case IRQ_TYPE_EDGE_FALLING:
			type2 |=(1 << sgpio_irq);
			break;
		case IRQ_TYPE_EDGE_BOTH:
			type0 &=~(1 << sgpio_irq);
			type1 |=(1 << sgpio_irq);
			type2 &=~(1 << sgpio_irq);
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			type0 |=(1 << sgpio_irq);
			type1 |=(1 << sgpio_irq);
			type2 &=~(1 << sgpio_irq);
			break;
		case IRQ_TYPE_LEVEL_LOW:
			type0 &=~(1 << sgpio_irq);
			type1 |=(1 << sgpio_irq);
			type2 &=~(1 << sgpio_irq);
			break;
		default:
			SGPIODBUG("not support trigger");
			return -EINVAL;
			break;
	}

	ast_sgpio_write(ast_sgpio, type0, ast_sgpio->int_type_offset);
	ast_sgpio_write(ast_sgpio, type1, ast_sgpio->int_type_offset + 0x04);
	ast_sgpio_write(ast_sgpio, type2, ast_sgpio->int_type_offset + 0x08);

	return 0;

}

static struct irq_chip ast_sgpio_irq_chip = {
	.name		= "SGPIO",
	.irq_ack			= ast_sgpio_ack_irq,
	.irq_mask			= ast_sgpio_mask_irq,
	.irq_unmask			= ast_sgpio_unmask_irq,
	.irq_set_type		= ast_sgpio_irq_type,
};

/*---------------------------------------------------------------------*/
static int 
ast_sgpio_probe(struct platform_device *pdev)
{
	int i, j;
	struct resource *res;
	struct ast_sgpio_bank *ast_sgpio;
	u32 sgpio_base;

	printk("AST SGPIO Driver, (c) ASPEED Tech. Inc. no %d \n", SGPIO_CHAIN_CHIP_BASE);
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
			return -ENXIO;
	
	if (!request_mem_region(res->start, resource_size(res), res->name))
			return -EBUSY;
	
	sgpio_base = (u32) ioremap(res->start, resource_size(res));
	if (!sgpio_base)
			return -ENOMEM;
	
	SGPIODBUG("virt sgpio_base = %x \n", sgpio_base);
	SGPIODBUG("gpio port num %d, total gpio pin : %d\n",
			SGPIO_PORT_NUM, ARCH_NR_SGPIOS);
	
	SGPIODBUG("sgpio chain start %d \n",IRQ_SGPIO_CHAIN_START);

	for (i = 0; i < SGPIO_PORT_NUM; i++) {
		ast_sgpio = &ast_sgpio_gp[i];

		SGPIODBUG("add gpio_chip [%s] : %d\n",ast_sgpio->chip.label, i);

		ast_sgpio->base = sgpio_base;
		ast_sgpio->chip.base = SGPIO_CHAIN_CHIP_BASE + i*8;
		ast_sgpio->chip.ngpio = 8;

		gpiochip_add(&ast_sgpio->chip);

		//Set Level Trigger
		ast_sgpio_write(ast_sgpio, 0xffffffff, ast_sgpio->int_type_offset);
		ast_sgpio_write(ast_sgpio, 0xffffffff, ast_sgpio->int_type_offset + 0x04);
		ast_sgpio_write(ast_sgpio, 0, ast_sgpio->int_type_offset + 0x08);

		for(j=0;j<8;j++) {
			SGPIODBUG("inst chip data %d\n",i*8 + j + IRQ_SGPIO_CHAIN_START);	
			irq_set_chip_data(i*8 + j + IRQ_SGPIO_CHAIN_START, ast_sgpio);
			irq_set_chip_and_handler(i*8 + j + IRQ_SGPIO_CHAIN_START, &ast_sgpio_irq_chip,
						handle_level_irq);
			set_irq_flags(i*8 + j + IRQ_SGPIO_CHAIN_START, IRQF_VALID);
		}

		irq_set_chained_handler(IRQ_SGPIO, ast_sgpio_irq_handler);

	}

	ast_sgpio_configuration(ast_sgpio);

	return 0;
	
}

static struct platform_driver ast_sgpio_driver = {
        .probe          = ast_sgpio_probe,
//      .remove         = __exit_p(ast_sgpio_remove),
        .driver         = {
                .name   = "ast-sgpio",
                .owner  = THIS_MODULE,
        },
};

static int __init ast_sgpio_init(void)
{
        return platform_driver_register(&ast_sgpio_driver);
}


core_initcall(ast_sgpio_init);
