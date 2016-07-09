/*
 *  linux/arch/arm/arch-ast2000/timer.c
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
 
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <linux/irq.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <plat/ast-scu.h>

#include <plat/core.h>

#include <asm/mach/time.h>

/***************************************************************************/
#define AST_TIMER_COUNT			0x00
#define AST_TIMER_RELOAD		0x04
#define AST_TIMER_MATCH1		0x08
#define AST_TIMER_MATCH2		0x0C
#define AST_TIMER_CTRL1			0x30
#define AST_TIMER_CTRL2			0x34
#define AST_TIMER_CTRL3			0x38
/***************************************************************************/
//Timer Ctrl 

#define TIMER_CTRL_T1_ENABLE	0x1
#define TIMER_CTRL_T1_EXT_REF	(0x1 << 1) 
#define TIMER_CTRL_T2_ENABLE	(0x1 << 4) 
#define TIMER_CTRL_T2_EXT_REF	(0x1 << 5) 
#define TIMER_CTRL_T3_ENABLE	(0x1 << 8) 
#define TIMER_CTRL_T3_EXT_REF	(0x1 << 9) 


/***************************************************************************/
#define AST_TIMER_EXT_CLK_1M		(1*1000*1000)				/* 1M */
#define TIMER_RELOAD					(AST_TIMER_EXT_CLK_1M / HZ)

/* Ticks */
#define TICKS_PER_uSEC                  		1		/* AST_TIMER_EXT_CLK_1M / 10 ^ 6 */

/* How long is the timer interval? */
#define TICKS2USECS(x)				((x) / TICKS_PER_uSEC)

/***************************************************************************/
static void __iomem *ast_timer_base;

#define ast_timer_write(value, reg) \
	__raw_writel(value, ast_timer_base + (reg))
#define ast_timer_read(reg) \
	__raw_readl(ast_timer_base + (reg))
/***************************************************************************/
#ifdef CONFIG_ARCH_USES_GETTIMEOFFSET 
static unsigned long ast_gettimeoffset(void)
{
	unsigned long ticks1, ticks2;//, status;


	ticks2 = ast_timer_read(AST_TIMER_COUNT);
	do {
		ticks1 = ticks2;
//		status = readl(AST_RAW_STS(0));// __raw_readl(IO_ADDRESS(ASPEED_VIC_BASE) + ASPEED_VIC_RAW_STATUS_OFFSET);
		ticks2 = ast_timer_read(AST_TIMER_COUNT);
	} while (ticks2 > ticks1);

	ticks1 = TIMER_RELOAD - ticks2;

	return TICKS2USECS(ticks1);
}
#endif

/*
 * IRQ handler for the timer
 */

#ifdef CONFIG_GENERIC_CLOCKEVENTS
static irqreturn_t ast_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static void ast_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evt)
{

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		ast_timer_write(TIMER_RELOAD - 1, AST_TIMER_RELOAD);
		ast_timer_write(TIMER_RELOAD - 1, AST_TIMER_COUNT);
		ast_timer_write(TIMER_CTRL_T1_ENABLE | TIMER_CTRL_T1_EXT_REF, AST_TIMER_CTRL1);
		break;

//	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
//		ctrl |= TIMER_CTRL_ONESHOT;
//		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		break;
	}
}

static int ast_set_next_event(unsigned long next,
	struct clock_event_device *evt)
{
	ast_timer_write(next, AST_TIMER_RELOAD);

	ast_timer_write(TIMER_CTRL_T1_ENABLE | ast_timer_read(AST_TIMER_CTRL1), AST_TIMER_CTRL1);

	return 0;
}

static struct clock_event_device ast_clockevent = {
	.name		= "timer0",
	.shift		= 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= ast_set_mode,
	.set_next_event	= ast_set_next_event,
	.rating		= 300,
	.cpumask	= cpu_all_mask,
};
#else
static irqreturn_t 
ast_timer_interrupt(int irq, void *dev_id)
{
	timer_tick();
	return IRQ_HANDLED;	
}

#endif

static struct irqaction ast_timer_irq = {
	.name		= "ast timer",
	.flags		= IRQF_TIMER | IRQF_TRIGGER_RISING,
	.handler		= ast_timer_interrupt,
	.dev_id		= &ast_clockevent,	
};


#define TIMER_FREQ_KHZ	(1000)

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init ast_init_timer(void)
{
	ast_timer_base = ioremap(AST_TIMER_BASE, 0x100);

	ast_timer_write(0, AST_TIMER_CTRL1);
	ast_timer_write(0, AST_TIMER_CTRL2);
	ast_timer_write(0, AST_TIMER_CTRL3);

	ast_timer_write(TIMER_RELOAD - 1, AST_TIMER_RELOAD);
	ast_timer_write(TIMER_RELOAD - 1, AST_TIMER_COUNT);

#ifdef CONFIG_GENERIC_CLOCKEVENTS

	ast_clockevent.irq = IRQ_TIMER0;
	ast_clockevent.mult = div_sc(TIMER_FREQ_KHZ, NSEC_PER_MSEC, ast_clockevent.shift);
	ast_clockevent.max_delta_ns = clockevent_delta2ns(0xffffffff, &ast_clockevent);
	ast_clockevent.min_delta_ns = clockevent_delta2ns(0xf, &ast_clockevent);

	setup_irq(IRQ_TIMER0, &ast_timer_irq);
	clockevents_register_device(&ast_clockevent);
#else	

	ast_timer_write(TIMER_CTRL_T1_ENABLE | TIMER_CTRL_T1_EXT_REF, AST_TIMER_CTRL1);

	/* 
	 * Make irqs happen for the system timer
	 */
	setup_irq(IRQ_TIMER0, &ast_timer_irq);

#endif


}

