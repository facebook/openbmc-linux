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
#include <linux/sched_clock.h>
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
#define AST_TIMER1_COUNT		0x00
#define AST_TIMER1_RELOAD		0x04
#define AST_TIMER1_MATCH1		0x08
#define AST_TIMER1_MATCH2		0x0C
#define AST_TIMER2_COUNT		0x10
#define AST_TIMER2_RELOAD		0x14
#define AST_TIMER2_MATCH1		0x18
#define AST_TIMER2_MATCH2		0x1C
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
#define AST_TIMER_EXT_CLK_1M	(1*1000*1000)	/* 1MHz */
#define AST_TIMER_RELOAD_HZ	(AST_TIMER_EXT_CLK_1M / HZ)
#define AST_TIMER_RELOAD_MIN	0x1
#define AST_TIMER_RELOAD_MAX	0xFFFFFFFF

/* Ticks */
#define TICKS_PER_uSEC		1	/* AST_TIMER_EXT_CLK_1M / 10 ^ 6 */

/* How long is the timer interval? */
#define TICKS2USECS(x)		((x) / TICKS_PER_uSEC)

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


	ticks2 = ast_timer_read(AST_TIMER1_COUNT);
	do {
		ticks1 = ticks2;
		ticks2 = ast_timer_read(AST_TIMER_COUNT);
	} while (ticks2 > ticks1);

	ticks1 = AST_TIMER_RELOAD_HZ - ticks2;

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
		ast_timer_write(AST_TIMER_RELOAD_HZ - 1, AST_TIMER1_RELOAD);
		ast_timer_write(AST_TIMER_RELOAD_HZ - 1, AST_TIMER1_COUNT);
		ast_timer_write(TIMER_CTRL_T1_ENABLE | TIMER_CTRL_T1_EXT_REF,
		                AST_TIMER_CTRL1);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		break;
	}
}

static int ast_set_next_event(unsigned long next,
                              struct clock_event_device *evt)
{
	u32 val;

	/* Stop the timer at first. */
	val = ast_timer_read(AST_TIMER_CTRL1) & (~TIMER_CTRL_T1_ENABLE);
	ast_timer_write(val, AST_TIMER_CTRL1);

	ast_timer_write(next, AST_TIMER1_RELOAD);

	/* Re-enable the timer. */
	val = ast_timer_read(AST_TIMER_CTRL1) | TIMER_CTRL_T1_ENABLE;
	ast_timer_write(val, AST_TIMER_CTRL1);

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

/*
 * Clocksource: ASPEED integrated 1MHz timer.
 */
static cycle_t ast_read_cycles(struct clocksource *cs)
{
	return (cycle_t)(~ast_timer_read(AST_TIMER2_COUNT));
}

static u64 ast_read_sched_clock(void)
{
	return (u64)(~ast_timer_read(AST_TIMER2_COUNT));
}

static struct clocksource ast_clocksource = {
	.name		= "ast_clocksource",
	.rating		= 300,
	.mask		= CLOCKSOURCE_MASK(32),
	.read		= ast_read_cycles,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
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

	ast_timer_write(AST_TIMER_RELOAD_HZ - 1, AST_TIMER1_RELOAD);
	ast_timer_write(AST_TIMER_RELOAD_HZ - 1, AST_TIMER1_COUNT);

#ifdef CONFIG_GENERIC_CLOCKEVENTS

	ast_clockevent.irq = IRQ_TIMER0;
	ast_clockevent.mult = div_sc(TIMER_FREQ_KHZ, NSEC_PER_MSEC,
	                             ast_clockevent.shift);
	ast_clockevent.max_delta_ns = clockevent_delta2ns(AST_TIMER_RELOAD_MAX,
	                                                  &ast_clockevent);
	ast_clockevent.min_delta_ns = clockevent_delta2ns(AST_TIMER_RELOAD_MIN,
	                                                  &ast_clockevent);

	setup_irq(IRQ_TIMER0, &ast_timer_irq);
	clockevents_register_device(&ast_clockevent);

	/* Set up free-running clocksource timer with interrupt disabled. */
	ast_timer_write(0, AST_TIMER2_COUNT);
	ast_timer_write(0, AST_TIMER2_MATCH1);
	ast_timer_write(0, AST_TIMER2_MATCH2);
	ast_timer_write(AST_TIMER_RELOAD_MAX, AST_TIMER2_RELOAD);
	ast_timer_write(TIMER_CTRL_T2_ENABLE | TIMER_CTRL_T2_EXT_REF |
	                ast_timer_read(AST_TIMER_CTRL1), AST_TIMER_CTRL1);
	clocksource_register_hz(&ast_clocksource, AST_TIMER_EXT_CLK_1M);
	sched_clock_register(ast_read_sched_clock, 32, AST_TIMER_EXT_CLK_1M);
#else
	ast_timer_write(TIMER_CTRL_T1_ENABLE | TIMER_CTRL_T1_EXT_REF,
	                AST_TIMER_CTRL1);

	/* 
	 * Make irqs happen for the system timer
	 */
	setup_irq(IRQ_TIMER0, &ast_timer_irq);
#endif
}
