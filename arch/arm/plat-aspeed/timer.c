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

#include <linux/irq.h>
#include <asm/system.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/time.h>
#include <plat/ast-scu.h>

#define ASPEED_TIMER0_VA_BASE		(IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMER0_OFFSET)
#define ASPEED_TIMER1_VA_BASE		(IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMER1_OFFSET)
#define ASPEED_TIMER2_VA_BASE		(IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMER2_OFFSET)
#define ASPEED_TIMERC_VA_BASE		(IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMERRC_OFFSET)

/*
 * Returns number of ms since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long ast_gettimeoffset(void)
{
	volatile TimerStruct_t *timer0 = (TimerStruct_t *) ASPEED_TIMER0_VA_BASE;
	unsigned long ticks1, ticks2;//, status;

	/*
	 * Get the current number of ticks.  Note that there is a race
	 * condition between us reading the timer and checking for
	 * an interrupt.  We get around this by ensuring that the
	 * counter has not reloaded between our two reads.
	 */
	ticks2 = timer0->TimerValue;
	do {
		ticks1 = ticks2;
//		status = readl(AST_RAW_STS(0));// __raw_readl(IO_ADDRESS(ASPEED_VIC_BASE) + ASPEED_VIC_RAW_STATUS_OFFSET);
		ticks2 = timer0->TimerValue;
	} while (ticks2 > ticks1);

	/*
	 * Number of ticks since last interrupt.
	 */
	ticks1 = TIMER_RELOAD - ticks2;

	/*
	 * Interrupt pending?  If so, we've reloaded once already.
	 */
//	if (status & (1 << IRQ_TIMER0))
//		ticks1 += TIMER_RELOAD;

	/*
	 * Convert the ticks to usecs
	 */
	return TICKS2USECS(ticks1);
}


/*
 * IRQ handler for the timer
 */
static irqreturn_t 
ast_timer_interrupt(int irq, void *dev_id)
{

//	write_seqlock(&xtime_lock);

	/*
	 * clear the interrupt in Irq.c
	 */
//	IRQ_EDGE_CLEAR(0,IRQ_TIMER0);

	timer_tick();


//	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;	
}

static struct irqaction ast_timer_irq = {
	.name		= "ast timer",
	.flags		= IRQF_DISABLED |  IRQF_TIMER,
	.handler	= ast_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init ast_setup_timer(void)
{
	volatile TimerStruct_t *timer0 = (volatile TimerStruct_t *) ASPEED_TIMER0_VA_BASE;
    volatile __u32         *timerc = (volatile __u32*) ASPEED_TIMERC_VA_BASE;

	/*
	 * Initialise to a known state (all timers off)
	 */
        *timerc = 0;

	timer0->TimerLoad    = TIMER_RELOAD - 1;
	timer0->TimerValue   = TIMER_RELOAD - 1;
	*timerc              = TIMER0_ENABLE | TIMER0_RefExt;

	/* 
	 * Make irqs happen for the system timer
	 */
	ast_scu_show_system_info();
	
	setup_irq(IRQ_TIMER0, &ast_timer_irq);

}

struct sys_timer ast_timer = {
	.init		= ast_setup_timer,
//	.offset		= ast_gettimeoffset,
};
