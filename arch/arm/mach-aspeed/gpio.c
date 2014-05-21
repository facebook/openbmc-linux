/*
 *  linux/arch/arm/plat-aspeed/gpio.c
 *
 * Support functions for ASPEED GPIO
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 * Written by Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <asm/mach/irq.h>

#include <plat/regs-gpio.h>
#include <asm-generic/gpio.h>

//#define AST_GPIO_DEBUG

#ifdef AST_GPIO_DEBUG
//#define GPIODBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#define GPIODBUG(fmt, args...) printk(fmt, ## args)

#else
#define GPIODBUG(fmt, args...)
#endif

/*************************************************************/
//GPIO group structure 
struct ast_gpio_bank {
    int 	irq;
	u32  	base;	
//TODO remove base	
	u32  	index;	
	u32		data_offset;
	u32		dir_offset;
	u32		int_en_offset;	
	u32		int_type_offset;		
	u32		int_sts_offset;	
	u32		rst_tol_offset;		
	u32		debounce_offset;	
	u32		cmd_source_offset;		
	struct gpio_chip chip;
	
//#ifdef CONFIG_PM
//#define NR_REGS         (7)
//			u32 regs[NR_REGS];
//#endif
};

int ast_gpio_to_irq(unsigned gpio)
{
	return (gpio + IRQ_GPIO_CHAIN_START);
}

EXPORT_SYMBOL(ast_gpio_to_irq);

int ast_irq_to_gpio(unsigned irq)
{
	return (irq - IRQ_GPIO_CHAIN_START);
}

EXPORT_SYMBOL(ast_irq_to_gpio);

static inline u32
ast_gpio_read(struct ast_gpio_bank *ast_gpio ,u32 offset)
{
	GPIODBUG("base = 0x%08x, offset = 0x%08x \n", ast_gpio->base, offset);

	return readl(ast_gpio->base + offset);
}

static inline void
ast_gpio_write(struct ast_gpio_bank *ast_gpio , u32 val, u32 offset)
{
    GPIODBUG("base = 0x%08x, offset = 0x%08x, val = 0x%08x\n", ast_gpio->base, offset, val);
    writel(val, ast_gpio->base + offset);
}

/***************************************************************************************/
static int
ast_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct ast_gpio_bank *ast_gpio = container_of(chip, struct ast_gpio_bank, chip);
	unsigned long flags;
	u32 v;
	int ret = -1;

	GPIODBUG("dir_in %s[%d] \n",chip->label, offset);

	local_irq_save(flags);

	v = ast_gpio_read(ast_gpio, ast_gpio->dir_offset);
	
	v &= ~(GPIO_OUTPUT_MODE << (offset + (ast_gpio->index * 8)));
	ast_gpio_write(ast_gpio, v, ast_gpio->dir_offset);

	ret = 0;

	local_irq_restore(flags);
	return ret;

}

static int
ast_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int val)
{
   	struct ast_gpio_bank *ast_gpio = container_of(chip, struct ast_gpio_bank, chip);
    unsigned long flags;
    u32 v;
    int ret = -1;
	GPIODBUG("dir_out %s[%d], val %d \n",chip->label, offset, val);

    local_irq_save(flags);

	/* Drive as an output */
	v = ast_gpio_read(ast_gpio, ast_gpio->dir_offset);

	v |= (GPIO_OUTPUT_MODE << (offset + (ast_gpio->index * 8)));

	ast_gpio_write(ast_gpio, v, ast_gpio->dir_offset);

	local_irq_restore(flags);
	
	ret = 0;	
	return ret;
}

static int
ast_gpio_get(struct gpio_chip *chip, unsigned offset)
{
   	struct ast_gpio_bank *ast_gpio = container_of(chip, struct ast_gpio_bank, chip);
    unsigned long flags;
    u32 v;

	GPIODBUG("Get %s[%d] \n",chip->label, offset);

    local_irq_save(flags);

    v = ast_gpio_read(ast_gpio, ast_gpio->data_offset);

    v &= (1 << (offset + (ast_gpio->index * 8)));
	
	if(v)
		v = 1;
	else
		v = 0;

    local_irq_restore(flags);
	
    return v;
}

static void
ast_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
   	struct ast_gpio_bank *ast_gpio = container_of(chip, struct ast_gpio_bank, chip);
    unsigned long flags;
    u32 v;
	GPIODBUG("Set %s[%d] = %d\n",chip->label, offset, val);

	local_irq_save(flags);

    /* Set the value */

    v = ast_gpio_read(ast_gpio, ast_gpio->data_offset);

	if (val)
		v |= (1 << (offset + (ast_gpio->index * 8)));
	else
		v &= ~(1 << (offset + (ast_gpio->index * 8)));

	ast_gpio_write(ast_gpio, v, ast_gpio->data_offset);

	local_irq_restore(flags);
}
	

#define AST_GPIO_BANK(name, gpio_base, index_no, data, dir, int_en, int_type, int_sts, rst_tol, debounce, cmd_s)	\
	{												\
		.base = gpio_base,							\
		.index = index_no,								\
		.data_offset = data,						\
		.dir_offset = dir, 							\
		.int_en_offset = int_en,					\
		.int_type_offset = int_type,				\
		.int_sts_offset = int_sts, 					\
		.rst_tol_offset = rst_tol,					\
		.debounce_offset = debounce,				\
		.cmd_source_offset = cmd_s,					\
		.chip = {                            							\
	        .label                  = name,  							\
	        .direction_input        = ast_gpio_direction_input,         \
	        .direction_output       = ast_gpio_direction_output,        \
	        .get            = ast_gpio_get,  \
	        .set            = ast_gpio_set,  \
	        .ngpio          = GPIO_PER_PORT_PIN_NUM, \
		}, \
	}

static struct ast_gpio_bank ast_gpio_gp[] = {
	AST_GPIO_BANK("GPIOA", IO_ADDRESS(AST_GPIO_BASE), 0, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOB", IO_ADDRESS(AST_GPIO_BASE), 1, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOC", IO_ADDRESS(AST_GPIO_BASE), 2, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOD", IO_ADDRESS(AST_GPIO_BASE), 3, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOE", IO_ADDRESS(AST_GPIO_BASE), 0, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOF", IO_ADDRESS(AST_GPIO_BASE), 1, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOG", IO_ADDRESS(AST_GPIO_BASE), 2, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOH", IO_ADDRESS(AST_GPIO_BASE), 3, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOI", IO_ADDRESS(AST_GPIO_BASE), 0, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOJ", IO_ADDRESS(AST_GPIO_BASE), 1, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOK", IO_ADDRESS(AST_GPIO_BASE), 2, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOL", IO_ADDRESS(AST_GPIO_BASE), 3, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),	
	AST_GPIO_BANK("GPIOM", IO_ADDRESS(AST_GPIO_BASE), 0, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),	
	AST_GPIO_BANK("GPION", IO_ADDRESS(AST_GPIO_BASE), 1, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOO", IO_ADDRESS(AST_GPIO_BASE), 2, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),	
	AST_GPIO_BANK("GPIOP", IO_ADDRESS(AST_GPIO_BASE), 3, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOQ", IO_ADDRESS(AST_GPIO_BASE), 0, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOR", IO_ADDRESS(AST_GPIO_BASE), 1, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),	
	AST_GPIO_BANK("GPIOS", IO_ADDRESS(AST_GPIO_BASE), 2, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
#if defined(CONFIG_ARCH_AST2400)		
	AST_GPIO_BANK("GPIOT", IO_ADDRESS(AST_GPIO_BASE), 4, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),	
	AST_GPIO_BANK("GPIOU", IO_ADDRESS(AST_GPIO_BASE), 0, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOV", IO_ADDRESS(AST_GPIO_BASE), 1, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOW", IO_ADDRESS(AST_GPIO_BASE), 2, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOX", IO_ADDRESS(AST_GPIO_BASE), 3, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOY", IO_ADDRESS(AST_GPIO_BASE), 0, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOZ", IO_ADDRESS(AST_GPIO_BASE), 1, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOAA", IO_ADDRESS(AST_GPIO_BASE), 2, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOAB", IO_ADDRESS(AST_GPIO_BASE), 3, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),	
#endif	
};


/***************************************************************************************/
/*
 * assuming the pin is muxed as a gpio output, set its value.
 */
int ast_set_gpio_value(unsigned gpio_pin, int value)
{
	u32 data;
	u32 gp, pin;
	gp = gpio_pin / 8;
	pin = gpio_pin % 32;
	data = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].data_offset);
	if(value)
		data |= (1 << pin);
	else
		data &= ~(1 << pin);
	
	GPIODBUG("gp : %d, pin %d, data = %x \n ", gp, pin, data);
	ast_gpio_write(&ast_gpio_gp[gp], data, ast_gpio_gp[gp].data_offset);

	return 0;
}
EXPORT_SYMBOL(ast_set_gpio_value);


/*
 * read the pin's value (works even if it's not muxed as a gpio).
 */
int ast_get_gpio_value(unsigned gpio_pin)
{
	u32 data;
	u32 gp, pin;
	gp = gpio_pin / 8;
	pin = gpio_pin % 32;
	data = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].data_offset);

	GPIODBUG("gp : %d, pin %d, data = %x, value = %d \n ", gp, pin, data, (data >> pin) & 1);

	return ((data >> pin) & 1);
}
EXPORT_SYMBOL(ast_get_gpio_value);

//timer 0/1/2
//Debounce time = PCLK * (val+1)
void ast_set_gpio_debounce_timer(int timer, int val)
{
	switch(timer) {
		case 0:
			writel(val, IO_ADDRESS(AST_GPIO_BASE) + 0x50);
			break;
		case 1:
			writel(val, IO_ADDRESS(AST_GPIO_BASE) + 0x54);
			break;
		case 2:
			writel(val, IO_ADDRESS(AST_GPIO_BASE) + 0x58);
			break;
	}
}

EXPORT_SYMBOL(ast_set_gpio_debounce_timer);

//TODO ......
//mode 0 : no debounce , 1: set  0x50, 2: 0x54, 3: 0x58
void ast_set_gpio_debounce(int gpio_port, int mode)
{
#if 0 
	u32 set0, set1;
	u16 gp, port;
	gp = gpio_port / 4;
	port = gpio_port % 4;
	set0 = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].debounce_offset);
	set1 = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].debounce_offset + 0x04);

	switch(port) {
		case 0:		//A , H , ......
			set0 = port 
			ast_gpio_write(ast_gpio, val, 0x50);
			break;
		case 1:
			ast_gpio_write(ast_gpio, val, 0x54);
			break;
		case 2:
			ast_gpio_write(ast_gpio, val, 0x58);
			break;
		case 3:
			ast_gpio_write(ast_gpio, val, 0x58);
		break;
		default:
			GPIODBUG("not support \n");
			return;
		break;

	}

	ast_gpio_write(&ast_gpio_gp[gp], set0, ast_gpio_gp[gp].debounce_offset);
	ast_gpio_write(&ast_gpio_gp[gp], set1, ast_gpio_gp[gp].debounce_offset + 0x04);
#endif
}

EXPORT_SYMBOL(ast_set_gpio_debounce);

//TODO ......
//
void ast_set_gpio_tolerant(int gpio_port, int mode)
{
#if 0 
	u32 set0, set1;
	u16 gp, port;
	gp = gpio_port / 4;
	port = gpio_port % 4;
	set0 = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].debounce_offset);
	set1 = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].debounce_offset + 0x04);

	switch(port) {
		case 0:		//A , H , ......
			set0 = port 
			ast_gpio_write(ast_gpio, val, 0x50);
			break;
		case 1:
			ast_gpio_write(ast_gpio, val, 0x54);
			break;
		case 2:
			ast_gpio_write(ast_gpio, val, 0x58);
			break;
		case 3:
			ast_gpio_write(ast_gpio, val, 0x58);
		break;
		default:
			GPIODBUG("not support \n");
			return;
		break;

	}

	ast_gpio_write(&ast_gpio_gp[gp], set0, ast_gpio_gp[gp].debounce_offset);
	ast_gpio_write(&ast_gpio_gp[gp], set1, ast_gpio_gp[gp].debounce_offset + 0x04);
#endif
}

EXPORT_SYMBOL(ast_set_gpio_tolerant);

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
ast_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	u32 isr;
	int i,j;
	struct ast_gpio_bank *ast_gpio;

	if(irq != IRQ_GPIO)
		BUG();

	GPIODBUG("ast_gpio_irq_handler %d \n ", irq);

//	ast_gpio = get_irq_data(irq);

//	GPIODBUG("[%s] ------\n ",ast_gpio->chip.label );
	
	desc->chip->ack(IRQ_GPIO);

	for (i = 0; i < GPIO_PORT_NUM; i++) {
		ast_gpio = &ast_gpio_gp[i];
		isr = ast_gpio_read(ast_gpio, ast_gpio->int_sts_offset);
		GPIODBUG("isr %x \n", isr);
		isr = (isr >> (8 * ast_gpio->index)) & 0xff;
		GPIODBUG("[%s] isr %x \n", ast_gpio->chip.label, isr);
		if(isr != 0) {
			//get gpio isr and --> to IRQ number ....
			for (j=0; j<8;j++) {
				if((1<<j) & isr) {
					// dispach interrupt
//					GPIODBUG("[%s] pin %d -> irq [%d]\n",ast_gpio->chip.label, j, j + IRQ_GPIO_CHAIN_START + (8 * i));
					generic_handle_irq(j + IRQ_GPIO_CHAIN_START + (8 * i));
				}
			}
//			GPIODBUG("isr -- ? %x \n",ast_gpio_read(ast_gpio, ast_gpio->int_sts_offset));
		}		
	}

#if 0
	while(1) {
		isr = ast_gpio_read(ast_gpio, ast_gpio->int_sts_offset);
		printk("isr %x \n", isr);
		isr = isr >> (8 * ast_gpio->index);
		//get gpio isr and --> to IRQ number ....
		for (i=0; i<8;i++) {
			if((1<<i) & isr) {
				// dispach interrupt
				printk("[%s] pin %d -> irq [%d]\n",ast_gpio->chip.label, i,i + IRQ_GPIO_CHAIN_START + (8 * ast_gpio->index));
				generic_handle_irq(i + IRQ_GPIO_CHAIN_START + (8 * ast_gpio->index));
			}
		}
		if(isr == 0)
			break;
	}
#endif		
	desc->chip->unmask(IRQ_GPIO);
	/* now it may re-trigger */

}

static void ast_gpio_ack_irq(unsigned int irq)
{
	struct ast_gpio_bank *ast_gpio = get_irq_chip_data(irq);

	unsigned int gpio_irq = irq - IRQ_GPIO_CHAIN_START;

	gpio_irq = gpio_irq % 8;

	GPIODBUG("irq [%d] : ast_gpio_ack_irq [%s] pin %d\n ",irq, ast_gpio->chip.label, gpio_irq);

	GPIODBUG("write clr [%x] %x\n ",ast_gpio->int_sts_offset, 1<< (gpio_irq + (ast_gpio->index * 8)));
		
	ast_gpio_write(ast_gpio, 1<< (gpio_irq + (ast_gpio->index * 8)), ast_gpio->int_sts_offset);

	GPIODBUG("read sts %x\n ",ast_gpio_read(ast_gpio, ast_gpio->int_sts_offset));

}

static void ast_gpio_mask_irq(unsigned int irq)
{
	struct ast_gpio_bank *ast_gpio = get_irq_chip_data(irq);
	unsigned int gpio_irq = irq - IRQ_GPIO_CHAIN_START;
	gpio_irq = gpio_irq%8;
	

	GPIODBUG("irq [%d] : ast_gpio_mask_irq [%s] pin %d\n ",irq, ast_gpio->chip.label, gpio_irq);

	//disable irq
	ast_gpio_write(ast_gpio, ast_gpio_read(ast_gpio, ast_gpio->int_en_offset) &
			~(1<< (gpio_irq + (ast_gpio->index * 8))), ast_gpio->int_en_offset);
}

static void ast_gpio_unmask_irq(unsigned int irq)
{
	struct ast_gpio_bank *ast_gpio = get_irq_chip_data(irq);
	unsigned int gpio_irq = irq - IRQ_GPIO_CHAIN_START;
	gpio_irq = gpio_irq%8;	


	GPIODBUG("irq[%d], [%s] pin %d\n",irq, ast_gpio->chip.label, gpio_irq);

	//Enable IRQ ..
	ast_gpio_write(ast_gpio, 1<< (gpio_irq + (ast_gpio->index * 8)), ast_gpio->int_sts_offset);

	ast_gpio_write(ast_gpio, ast_gpio_read(ast_gpio, ast_gpio->int_en_offset) |
			(1<< (gpio_irq + (ast_gpio->index * 8))), ast_gpio->int_en_offset);

}

static int
ast_gpio_irq_type(unsigned int irq, unsigned int type)
{
	u32 type0, type1, type2;
	struct ast_gpio_bank *ast_gpio;
	int retval = 0;	
	unsigned int gpio_irq = irq - IRQ_GPIO_CHAIN_START;
	gpio_irq = gpio_irq%32;



	GPIODBUG("ast_gpio_irq_type %d : %x \n",irq,type);
	if (type & ~IRQ_TYPE_SENSE_MASK)
		return -EINVAL;

	ast_gpio = get_irq_chip_data(irq);

	type0 = ast_gpio_read(ast_gpio, ast_gpio->int_type_offset);
	type1 = ast_gpio_read(ast_gpio, ast_gpio->int_type_offset + 0x04);
	type2 = ast_gpio_read(ast_gpio, ast_gpio->int_type_offset + 0x08);

	switch(type) {
		/* Edge rising type */
		case IRQ_TYPE_EDGE_RISING:
			type0 |=(1<<irq);
			type1 &=~(1<<irq);
			type2 &=~(1<<irq);
			break;
		/* Edge falling type */
		case IRQ_TYPE_EDGE_FALLING:
			type2 |=(1<<irq);
			break;
		case IRQ_TYPE_EDGE_BOTH:
			type0 &=~(1<<irq);
			type1 |=(1<<irq);
			type2 &=~(1<<irq);
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			type0 |=(1<<irq);
			type1 |=(1<<irq);
			type2 &=~(1<<irq);
			break;
		case IRQ_TYPE_LEVEL_LOW:
			type0 &=~(1<<irq);
			type1 |=(1<<irq);
			type2 &=~(1<<irq);
			break;
		default:
			GPIODBUG("not support trigger");
			return -EINVAL;
			break;
	}

	ast_gpio_write(ast_gpio, type0, ast_gpio->int_type_offset);
	ast_gpio_write(ast_gpio, type1, ast_gpio->int_type_offset + 0x04);
	ast_gpio_write(ast_gpio, type2, ast_gpio->int_type_offset + 0x08);

	return retval;

}

static struct irq_chip ast_gpio_irq_chip = {
	.name		= "GPIO",
	.ack		= ast_gpio_ack_irq,
	.mask		= ast_gpio_mask_irq,
	.unmask		= ast_gpio_unmask_irq,
	.set_type	= ast_gpio_irq_type,
};

/*---------------------------------------------------------------------*/
static int __init ast_gpio_init(void)
{
	int i,j;
	struct ast_gpio_bank *ast_gpio;

	GPIODBUG("gpio port num %d, total gpio pin : %d\n",
		GPIO_PORT_NUM, ARCH_NR_GPIOS);

	GPIODBUG("gpio chain start %d \n",IRQ_GPIO_CHAIN_START);
	for (i = 0; i < GPIO_PORT_NUM; i++) {
		ast_gpio = &ast_gpio_gp[i];

		GPIODBUG("add gpio_chip [%s] : %d\n",ast_gpio->chip.label, i);

#if 0		 
		bank->chip.direction_input = ast_gpio_direction_input;
		bank->chip.get = ast_gpio_get;
		bank->chip.direction_output = ast_gpio_direction_output;
		bank->chip.set = ast_gpio_set;

		bank->chip.label = "gpio";
#endif
		ast_gpio->chip.base = i*8;
		ast_gpio->chip.ngpio = 8;

		gpiochip_add(&ast_gpio->chip);

#if 1
		//Set Level Trigger
		ast_gpio_write(ast_gpio, 0xffffffff, ast_gpio->int_type_offset);
		ast_gpio_write(ast_gpio, 0xffffffff, ast_gpio->int_type_offset + 0x04);
		ast_gpio_write(ast_gpio, 0, ast_gpio->int_type_offset + 0x08);
		//remove clear direction for keep orignal state
//		ast_gpio_write(ast_gpio, 0, ast_gpio->dir_offset);
		//Enable IRQ 
//		ast_gpio_write(ast_gpio, 0xffffffff, ast_gpio->int_en_offset);
		
#endif

		for(j=0;j<8;j++) {
			GPIODBUG("inst chip data %d\n",i*8 + j + IRQ_GPIO_CHAIN_START);	
			set_irq_chip_data(i*8 + j + IRQ_GPIO_CHAIN_START, ast_gpio);
			set_irq_chip(i*8 + j + IRQ_GPIO_CHAIN_START, &ast_gpio_irq_chip);
			set_irq_handler(i*8 + j + IRQ_GPIO_CHAIN_START, handle_level_irq);
			set_irq_flags(i*8 + j + IRQ_GPIO_CHAIN_START, IRQF_VALID);
		}
		set_irq_chained_handler(IRQ_GPIO, ast_gpio_irq_handler);
//		set_irq_chip_data(IRQ_GPIO, ast_gpio);
//		set_irq_data(IRQ_GPIO, ast_gpio_gp[]);
		
		
	}

	return 0;
	
}

core_initcall(ast_gpio_init);

//arch_initcall(ast_gpio_init);

