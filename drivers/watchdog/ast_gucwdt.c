/*
 * linux/drivers/char/watdog/umvp2500_wdt.c
 * modified from drivers/char/watchdog/wdt.c
 *
 * Driver for GUC-UMVP2500 Watch Dog IP
 *
 * Copyright 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>

#include <linux/platform_device.h>
#include <asm/io.h>

#ifdef CONFIG_COLDFIRE
#include <asm/arch/irqs.h>
#include <asm/arch/ast_wdt.h>
#include <asm/arch/platform.h>
#else
#include <mach/irqs.h>
#include <mach/ast_wdt.h>
#include <mach/platform.h>
#endif

typedef unsigned char bool_T;

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif

#define TRUE	1
#define FALSE	0

#if defined(CONFIG_COLDFIRE)
#define WDT_BASE_VA		AST_WDT_BASE

#else
#define WDT_BASE_VA		AST_WDT_VA_BASE
#endif

#define WDT_CntSts              (WDT_BASE_VA+0x00)
#define WDT_Reload              (WDT_BASE_VA+0x04)
#define WDT_Restart             (WDT_BASE_VA+0x08)
#define WDT_Ctrl                (WDT_BASE_VA+0x0C)
#define WDT_TimeOut             (WDT_BASE_VA+0x10)
#define WDT_Clr                 (WDT_BASE_VA+0x14)
#define WDT_RstWd               (WDT_BASE_VA+0x18)

#define WDT_CTRL_B_SECOND_BOOT  (0x1 << 7)
#define WDT_CTRL_B_RESET_SOC (0x00 << 5) /* yes, 0x00 */
#define WDT_CTRL_B_RESET_FULL (0x01 << 5)
#define WDT_CTRL_B_RESET_ARM (0x2 << 5)
#define WDT_CTRL_B_RESET_MASK (0x3 << 5)
#define WDT_CTRL_B_1MCLK (0x1 << 4)
#define WDT_CTRL_B_EXT  (0x1 << 3)
#define WDT_CTRL_B_INTR  (0x1 << 2)
#define WDT_CTRL_B_CLEAR_AFTER  (0x1 << 1)
#define WDT_CTRL_B_ENABLE  (0x1 << 0)


#define UMVP_READ_REG(r)		(*((volatile unsigned int *) (r)))
#define UMVP_WRITE_REG(r,v)		(*((volatile unsigned int *) (r)) = ((unsigned int)   (v)))


#define WDT_CLK_SRC_EXT		0
#define WDT_CLK_SRC_PCLK	1

//Global Variables
#define WDT_TIMO 30             /* Default timeout, 30 seconds */
#define WDT_INITIAL_TIMO (8*60) /* Initial timeout, 8m */

#define TICKS_PER_uSEC 1        /* 1MHz clock */
#define WDT_TIMO2TICKS(t) (TICKS_PER_uSEC * 1000000 * (t))

static int heartbeat = WDT_TIMO;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds. (0<heartbeat<65536, default=" __MODULE_STRING(WDT_TIMO) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

static int force_disable = 0; // setting this to 1 will disable the wdt timer
module_param(force_disable, int, 0);
MODULE_PARM_DESC(force_disable, "Disable watchdog by default "
                 "(default=0, enable watchdog)");

static char expect_close;

//Function Declaration
int __init wdt_init(void);

static irqreturn_t wdt_isr(int irq, void *devid, struct pt_regs *regs)
{
	/* clear timeout */
	UMVP_WRITE_REG(WDT_Clr, 1);

	return (IRQ_HANDLED);
}

void wdt_disable(void)
{
    register unsigned int regVal;

    /* reset WDT_Ctrl[0] as 0 */
    regVal = UMVP_READ_REG(WDT_Ctrl);
    regVal &= ~(WDT_CTRL_B_ENABLE);
    UMVP_WRITE_REG(WDT_Ctrl, regVal);
}

void wdt_sel_clk_src(unsigned char sourceClk)
{
    register unsigned int regVal;

    regVal = UMVP_READ_REG(WDT_Ctrl);
    if (sourceClk == WDT_CLK_SRC_PCLK)
    {
        /* reset WDT_Ctrl[4] as 0 */
      regVal &= ~(WDT_CTRL_B_1MCLK);
    }
    else
    {
        /* set WDT_Ctrl[4] as 1 */
      regVal |= WDT_CTRL_B_1MCLK;
    }
    UMVP_WRITE_REG(WDT_Ctrl, regVal);
}

void wdt_set_timeout_action(bool_T bResetOut, bool_T bIntrSys,
                            bool_T bClrAfter, bool_T bResetARMOnly)
{
	register unsigned int regVal;

	regVal = UMVP_READ_REG(WDT_Ctrl);

	if (bResetOut)
	{
		/* set WDT_Ctrl[3] = 1 */
		regVal |= WDT_CTRL_B_EXT;
	}
	else
	{
		/* reset WDT_Ctrl[3] = 0 */
		regVal &= ~WDT_CTRL_B_EXT;
	}

	if (bIntrSys)
	{
		/* set WDT_Ctrl[2] = 1 */
		regVal |= WDT_CTRL_B_INTR;
	}
	else
	{
		/* reset WDT_Ctrl[2] = 0 */
		regVal &= ~WDT_CTRL_B_INTR;
	}

	if (bClrAfter)
	{
		/* set WDT_Ctrl[1] = 1 */
		regVal |= WDT_CTRL_B_CLEAR_AFTER;
	}
	else
	{
		/* reset WDT_Ctrl[1] = 0 */
		regVal &= ~WDT_CTRL_B_CLEAR_AFTER;
	}

  if (bResetARMOnly)
  {
    /* set WDT_Ctrl[6..5] = 10 ie, reset ARM only */
    regVal &= ~WDT_CTRL_B_RESET_MASK;
    regVal |= WDT_CTRL_B_RESET_ARM;
  }
  else
  {
    /* reset WDT_CTrl[6..5] = 01, full chip */
    regVal &= ~WDT_CTRL_B_RESET_MASK;
    regVal |= WDT_CTRL_B_RESET_FULL;
  }

	UMVP_WRITE_REG(WDT_Ctrl, regVal);
}

void wdt_enable(void)
{
  if (!force_disable) {
  	register unsigned int regVal;

  	/* set WDT_Ctrl[0] as 1 */
  	regVal = UMVP_READ_REG(WDT_Ctrl);
  	regVal |= WDT_CTRL_B_ENABLE;
  	UMVP_WRITE_REG(WDT_Ctrl, regVal);
  }
}

bool_T wdt_is_enabled(void)
{
  unsigned int reg;
  reg = UMVP_READ_REG(WDT_Ctrl);
  return reg & WDT_CTRL_B_ENABLE;
}

void wdt_restart_new(unsigned int nPeriod, int sourceClk, bool_T bResetOut,
                     bool_T bIntrSys, bool_T bClrAfter, bool_T bResetARMOnly)
{
  bool_T enabled = wdt_is_enabled();

  if (enabled) {
  	wdt_disable();
  }

  UMVP_WRITE_REG(WDT_Reload, nPeriod);

  wdt_sel_clk_src(sourceClk);

  wdt_set_timeout_action(bResetOut, bIntrSys, bClrAfter, bResetARMOnly);

  UMVP_WRITE_REG(WDT_Restart, 0x4755);	/* reload! */

  if (enabled) {
    wdt_enable();
  }
}

void wdt_restart(void)
{
  if (!force_disable) {
  	wdt_disable();
	  UMVP_WRITE_REG(WDT_Restart, 0x4755);	/* reload! */
  	wdt_enable();
  }
}


/**
 *	wdt_set_heartbeat:
 *	@t:		the new heartbeat value that needs to be set.
 *
 *	Set a new heartbeat value for the watchdog device. If the heartbeat value is
 *	incorrect we keep the old value and return -EINVAL. If successfull we
 *	return 0.
 */
static int wdt_set_heartbeat(int t)
{
  if ((t < 1) || (t > 1000))
      return -EINVAL;

  heartbeat=t;

  wdt_restart_new(WDT_TIMO2TICKS(t), WDT_CLK_SRC_EXT,
                  /* No Ext, No intr, Self clear, Full chip reset */
                  FALSE, FALSE, TRUE, FALSE);
  return 0;
}

/*
   Kernel Interfaces
*/

/**
 *	umvp2500_wdt_write:
 *	@file: file handle to the watchdog
 *	@buf: buffer to write (unused as data does not matter here
 *	@count: count of bytes
 *	@ppos: pointer to the position to write. No seeks allowed
 *
 *	A write to a watchdog device is defined as a keepalive signal. Any data will
 *	do, except for the reserved letters 'V' (to enable magic close), the
 *	letter 'X' (to override the current watchdog settings and disable it), or the
 *	letter 'x' (to turn off override and restore its old settings).
 *
 */

static ssize_t umvp2500_wdt_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
  if(count)
  {
    if (!nowayout)
    {
       size_t i;

       /* In case it was set long ago */
       expect_close = 0;

       for (i = 0; i != count; i++)
       {
         char c;
         if (get_user(c, buf + i))
           return -EFAULT;

         switch(c) {
           case 'V':
             expect_close = 42;
             break;
           case 'X':
             force_disable = 1;
             wdt_disable();
             break;
           case 'x':
             force_disable = 0;
             break;
           default:
             break;
         }
       }
     }
     wdt_restart();
   }
   return count;
 }

/**
 *	umvp2500_wdt_ioctl:
 *	@inode: inode of the device
 *	@file: file handle to the device
 *	@cmd: watchdog command
 *	@arg: argument pointer
 * *	The watchdog API defines a common set of functions for all watchdogs
 *	according to their available features. We only actually usefully support
 *	querying capabilities and current status.
 */

static int umvp2500_wdt_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
  void __user *argp = (void __user *)arg;
  int __user *p = argp;
  int new_heartbeat;

  static struct watchdog_info ident =
  {
    .options 		= WDIOF_SETTIMEOUT|
                          WDIOF_MAGICCLOSE|
                          WDIOF_KEEPALIVEPING,
    .firmware_version 	= 1,
    .identity 		= "UMVP2500 WDT",
  };

  switch(cmd)
  {
    default:
      return -ENOIOCTLCMD;
    case WDIOC_GETSUPPORT:
      return copy_to_user(argp, &ident, sizeof(ident))?-EFAULT:0;
    case WDIOC_GETSTATUS:
    case WDIOC_GETBOOTSTATUS:
      return put_user(0, p);
    case WDIOC_KEEPALIVE:
      wdt_restart();
      return 0;
    case WDIOC_SETTIMEOUT:
      if (get_user(new_heartbeat, p))
        return -EFAULT;

      if (wdt_set_heartbeat(new_heartbeat))
        return -EINVAL;

      /* Fall */
    case WDIOC_GETTIMEOUT:
      return put_user(heartbeat, p);
  }
}
/**
*	umvp2500_wdt_open:
*	@inode: inode of device
*	@file: file handle to device
*
*	The watchdog device has been opened. The watchdog device is single
*	open and on opening we load the counters. Counter zero is a 100Hz
*	cascade, into counter 1 which downcounts to reboot. When the counter
*	triggers counter 2 downcounts the length of the reset pulse which
*	set set to be as long as possible.
*/

static int umvp2500_wdt_open(struct inode *inode, struct file *file)
{
  /*
   *	Activate
   */
 // wdt_init();
  wdt_restart();
  return nonseekable_open(inode, file);
}

/**
*	umvp2500_wdt_release:
*	@inode: inode to board
*	@file: file handle to board
*
*	The watchdog has a configurable API. There is a religious dispute
*	between people who want their watchdog to be able to shut down and
*	those who want to be sure if the watchdog manager dies the machine
*	reboots. In the former case we disable the counters, in the latter
*	case you have to open it again very soon.
*/

static int umvp2500_wdt_release(struct inode *inode, struct file *file)
{
  if (expect_close != 42 && !nowayout)
  {
    /* handles the case where the device is closed without the "magic
     * close" character (anything that is not 'V' qualifies -- see the
     * original Linux watchdog spec for more about this). closing the
     * device in this case must disable the timer too, so automatic
     * restarts are inhibited.
     */
    wdt_disable();
  }
  else
  {
    /* handles the case where the kernel is compiled with nowayout, or
     * if the user specifies that the watchdog should continue ticking
     * after device closure (by writing a 'V' before closing the device)
     */
    wdt_restart();
  }

  expect_close = 0;
  return 0;
}

/**
*	notify_sys:
*	@this: our notifier block
*	@code: the event being reported
*	@unused: unused
*
*	Our notifier is called on system shutdowns. We want to turn the card
*	off at reboot otherwise the machine will reboot again during memory
*	test or worse yet during the following fsck. This would suck, in fact
*	trust me - if it happens it does suck.
*/

static int umvp2500_wdt_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
   if(code==SYS_DOWN || code==SYS_HALT)
   {
     /* Turn the WDT off */
     wdt_disable();
   }
   return NOTIFY_DONE;
}

extern void ast_wdt_reset_soc(void)
{
	writel(0x10 , WDT_Reload);
	writel(0x4755, WDT_Restart);
	writel(WDT_CTRL_B_RESET_SOC|WDT_CTRL_B_CLEAR_AFTER|WDT_CTRL_B_ENABLE,
         WDT_Ctrl);
}

EXPORT_SYMBOL(ast_wdt_reset_soc);

extern void ast_wdt_reset_full(void)
{
	writel(0x10 , WDT_Reload);
	writel(0x4755, WDT_Restart);
	writel(WDT_CTRL_B_RESET_FULL|WDT_CTRL_B_CLEAR_AFTER|WDT_CTRL_B_ENABLE,
         WDT_Ctrl);
}

EXPORT_SYMBOL(ast_wdt_reset_full);

static struct file_operations umvp2500_wdt_fops =
{
  .owner	= THIS_MODULE,
  .llseek	= no_llseek,
  .write	= umvp2500_wdt_write,
  .ioctl	= umvp2500_wdt_ioctl,
  .open		= umvp2500_wdt_open,
  .release	= umvp2500_wdt_release,
};

static struct miscdevice ast_wdt_miscdev =
{
   .minor	= WATCHDOG_MINOR,
   .name	= "watchdog",
   .fops	= &umvp2500_wdt_fops,
};

static struct notifier_block umvp2500_wdt_notifier =
{
   .notifier_call=umvp2500_wdt_notify_sys,
};

static int ast_wdt_probe(struct platform_device *pdev)
{
   int ret;

   /* register ISR */
   if (request_irq(IRQ_WDT, (void *)wdt_isr, IRQF_DISABLED, "WDT", NULL))
   {
     printk("unable to register interrupt INT_WDT = %d\n", IRQ_WDT);
     return (-1);
   }
   else
     printk("success to register interrupt for INT_WDT (%d)\n", IRQ_WDT);

   ret = register_reboot_notifier(&umvp2500_wdt_notifier);
   if(ret)
   {
     printk(KERN_ERR "wdt: cannot register reboot notifier (err=%d)\n", ret);
     free_irq(IRQ_WDT, NULL);
     return ret;
   }

   ret = misc_register(&ast_wdt_miscdev);
   if (ret)
   {
      printk(KERN_ERR "wdt: cannot register miscdev on minor=%d (err=%d)\n",WATCHDOG_MINOR, ret);
      unregister_reboot_notifier(&umvp2500_wdt_notifier);
      return ret;
   }

   /* interrupt the system while WDT timeout */
   wdt_restart_new(WDT_TIMO2TICKS(WDT_INITIAL_TIMO), WDT_CLK_SRC_EXT,
                   /* No Ext, No intr, Self clear, Full chip reset */
                   FALSE, FALSE, TRUE, FALSE);

   /* enable it by default */
   if (!force_disable) {
     wdt_enable();
   }

   /* change the reload value back to regular */
   UMVP_WRITE_REG(WDT_Reload, WDT_TIMO2TICKS(heartbeat));

   printk(KERN_INFO "UMVP2500 WDT is installed. (irq:%d, initial timeout:%ds, "
          "timeout:%ds nowayout:%d enabled:%s)\n",
          IRQ_WDT, WDT_INITIAL_TIMO, heartbeat, nowayout,
          wdt_is_enabled() ? "yes" : "no");

   return (0);
}

static int ast_wdt_remove(struct platform_device *dev)
{
	misc_deregister(&ast_wdt_miscdev);
	disable_irq(IRQ_WDT);
	free_irq(IRQ_WDT, NULL);
	return 0;
}

static void ast_wdt_shutdown(struct platform_device *dev)
{
	wdt_disable();
}

static struct platform_driver ast_wdt_driver = {
        .probe          = ast_wdt_probe,
        .remove         = ast_wdt_remove,
        .shutdown       = ast_wdt_shutdown,
#if 0
        .suspend                = ast_wdt_suspend,
        .resume         = ast_wdt_resume,
#endif
        .driver         = {
                .owner  = THIS_MODULE,
                .name   = "ast-wdt",
        },
};

static char banner[] __initdata = KERN_INFO "ASPEED Watchdog Timer, ASPEED Technology Inc.\n";

static int __init watchdog_init(void)
{
        printk(banner);

        return platform_driver_register(&ast_wdt_driver);
}

static void __exit watchdog_exit(void)
{
        platform_driver_unregister(&ast_wdt_driver);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Golbal Unichip Corp.");
MODULE_DESCRIPTION("Driver for UMVP-2500 Watch Dog");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_LICENSE("GPL");
