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

#include <asm/io.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/time.h>
#include <plat/ast-scu.h>

#define ASPEED_TIMER0_VA_BASE   (IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMER0_OFFSET)
#define ASPEED_TIMER1_VA_BASE   (IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMER1_OFFSET)
#define ASPEED_TIMER2_VA_BASE   (IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMER2_OFFSET)
#define ASPEED_TIMERC_VA_BASE   (IO_ADDRESS(AST_TIMER_BASE)+ASPEED_TIMERRC_OFFSET)

#define ASPEED_TIMER_RELOAD_MAX 0xFFFFFFFF
#define ASPEED_TIMER_RELOAD_MIN 1

static struct clock_event_device clockevent_ast;

static inline unsigned long ast_timer_read_count(void *base)
{
  volatile TimerStruct_t *timer = (volatile TimerStruct_t *)(base);
  return timer->TimerValue;
}

/* change the timer count and load value (if requeseted) */
static inline void ast_timer_set_count(void *base, unsigned long count,
                                       unsigned long reload)
{
  volatile TimerStruct_t *timer = (volatile TimerStruct_t *)(base);
  timer->TimerValue = count;
  timer->TimerLoad = reload;
}

#define AST_TIMER_DISABLE 0
#define AST_TIMER_ENABLE 1

static inline void ast_timer0_ctrl(int enable)
{
  volatile __u32 *timerc = (volatile __u32*) ASPEED_TIMERC_VA_BASE;
  if (enable == AST_TIMER_ENABLE) {
    *timerc |= TIMER0_ENABLE | TIMER0_RefExt;
  } else {
    *timerc &= ~TIMER0_ENABLE;
  }
}

static inline void ast_timer1_ctrl(int enable)
{
  volatile __u32 *timerc = (volatile __u32*) ASPEED_TIMERC_VA_BASE;
  if (enable == AST_TIMER_ENABLE) {
    *timerc |= TIMER1_ENABLE | TIMER1_RefExt;
  } else {
    *timerc &= ~TIMER1_ENABLE;
  }
}

static inline void ast_timer_disable_all()
{
  volatile __u32 *timerc = (volatile __u32*) ASPEED_TIMERC_VA_BASE;
  *timerc = 0;
}

/*
 * clocksource
 */
static irqreturn_t ast_clocksource_interrupt(int irq, void *dev_id)
{
  return IRQ_HANDLED;
}

static struct irqaction ast_clocksource_irq = {
  .name   = "ast-clocksource",
  .flags    = IRQF_DISABLED |  IRQF_TIMER,
  .handler  = ast_clocksource_interrupt,
};

static cycle_t read_cycles(void)
{
#if 1
	return (cycles_t)(ASPEED_TIMER_RELOAD_MAX
                    - ast_timer_read_count(ASPEED_TIMER1_VA_BASE));
#else
  return (cycles_t) ast_timer_read_count(ASPEED_TIMER1_VA_BASE);
#endif
}

static struct clocksource clocksource_ast = {
	.name		= "ast-clocksource",
	.rating		= 300,
	.read		= read_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * clockevent
 */
/* IRQ handler for the timer */
static irqreturn_t ast_clockevent_interrupt(int irq, void *dev_id)
{
  struct clock_event_device *evt = &clockevent_ast;
  evt->event_handler(evt);
  return IRQ_HANDLED;
}

static struct irqaction ast_clockevent_irq = {
  .name   = "ast-clockevent",
  .flags    = IRQF_DISABLED |  IRQF_TIMER,
  .handler  = ast_clockevent_interrupt,
};

static int ast_timer_set_next_event(unsigned long cycles,
                                    struct clock_event_device *evt)
{
  /* In this case, we shall not set the load value. */
  ast_timer_set_count(ASPEED_TIMER0_VA_BASE, cycles, 0);
  /* turn on the timer */
  ast_timer0_ctrl(AST_TIMER_ENABLE);
  return 0;
}

static void ast_timer_set_mode(enum clock_event_mode mode,
                               struct clock_event_device *evt)
{
  /* stop timer first */
  ast_timer0_ctrl(AST_TIMER_DISABLE);
  switch (mode) {
  case CLOCK_EVT_MODE_PERIODIC:
    ast_timer_set_count(ASPEED_TIMER0_VA_BASE,
                        TIMER_RELOAD - 1, TIMER_RELOAD - 1);
    ast_timer0_ctrl(AST_TIMER_ENABLE);
    break;
  case CLOCK_EVT_MODE_ONESHOT:
    /*
     * Leave the timer disabled, ast_timer_set_next_event() will
     * enable it later
     */
    break;
  case CLOCK_EVT_MODE_UNUSED:
  case CLOCK_EVT_MODE_SHUTDOWN:
  case CLOCK_EVT_MODE_RESUME:
    break;
  }
}

static struct clock_event_device clockevent_ast = {
  .name = "ast-clockevent",
  .features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
  .shift = 32,
  .set_next_event = ast_timer_set_next_event,
  .set_mode = ast_timer_set_mode,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init ast_setup_timer(void)
{
  /*
   * Initialise to a known state (all timers off)
   */
  ast_timer_disable_all();

  /*
   * For clock event, set the value and reload to 0, so that no interrupt even
   * after enabling timer.
   */
  ast_timer_set_count(ASPEED_TIMER0_VA_BASE, 0, 0);
  /*
   * For clock source, set the value and reload to the max
   */
  ast_timer_set_count(ASPEED_TIMER1_VA_BASE,
                      ASPEED_TIMER_RELOAD_MAX, ASPEED_TIMER_RELOAD_MAX);

  /* Enable timer */
  ast_timer0_ctrl(AST_TIMER_ENABLE);
  ast_timer1_ctrl(AST_TIMER_ENABLE);

  ast_scu_show_system_info();

  /* irqs happen for the system timer */
  setup_irq(IRQ_TIMER0, &ast_clockevent_irq);
  setup_irq(IRQ_TIMER1, &ast_clocksource_irq);

  /* setup clocksource */
	clocksource_ast.mult = clocksource_hz2mult(ASPEED_TIMER_CLKRATE,
                                             clocksource_ast.shift);
	if (clocksource_register(&clocksource_ast)) {
    printk(KERN_ERR "Failed to register clock source %s", clocksource_ast.name);
  }

  /* setup clockevent */
  clockevent_ast.mult = div_sc(ASPEED_TIMER_CLKRATE, NSEC_PER_SEC,
                               clockevent_ast.shift);
  clockevent_ast.max_delta_ns = clockevent_delta2ns(ASPEED_TIMER_RELOAD_MAX,
                                                    &clockevent_ast);
  clockevent_ast.min_delta_ns = clockevent_delta2ns(ASPEED_TIMER_RELOAD_MIN,
                                                    &clockevent_ast);
  clockevent_ast.cpumask = cpumask_of_cpu(0);
  clockevents_register_device(&clockevent_ast);
}

struct sys_timer ast_timer = {
  .init   = ast_setup_timer,
};
