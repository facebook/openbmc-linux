/********************************************************************************
* File Name     : drivers/rtc/rtc-ast.c
* Author        : Ryan chen
* Description   : ASPEED Real Time Clock Driver (RTC)
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/09/21 ryan chen create this file
*
********************************************************************************/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/io.h>

#include <plat/regs-rtc.h>

struct ast_rtc {
        void __iomem *base;
        int irq;
        struct resource *res;
        struct rtc_device *rtc_dev;
        spinlock_t lock;
};

//static char banner[] = "ASPEED RTC, (C) ASPEED Technology Inc.\n";
//#define CONFIG_RTC_DEBUG


static inline u32
rtc_read(void __iomem *base, u32 reg)
{
#ifdef CONFIG_RTC_DEBUG
        int val = readl(base + reg);
        pr_debug("base = 0x%p, offset = 0x%08x, value = 0x%08x\n", base, reg, val);
        return val;
#else
        return readl(base + reg);
#endif
}

static inline void
rtc_write(void __iomem * base, u32 val, u32 reg)
{
        pr_debug("base = 0x%p, offset = 0x%08x, data = 0x%08x\n", base, reg, val);
        writel(val, base + reg);
}

static int
ast_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct ast_rtc *ast_rtc = dev_get_drvdata(dev);
	pr_debug("cmd = 0x%08x, arg = 0x%08lx\n", cmd, arg);
	
	switch (cmd) {
		case RTC_AIE_ON:	/* alarm on */
			{
				rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL) | ENABLE_ALL_ALARM, RTC_CONTROL);
				return 0;
			}

		case RTC_AIE_OFF:	/* alarm off */
			{
				rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL) &~ENABLE_ALL_ALARM, RTC_CONTROL);
				return 0;
			}
		case RTC_UIE_ON:	/* update on */
			{
				pr_debug("no such function \n");
				return 0;
			}
		case RTC_UIE_OFF:	/* update off */
			{
				pr_debug("no such function \n");
				return 0;
			}
		case RTC_PIE_OFF:	/* periodic off */
			{
				rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL) | ENABLE_SEC_INTERRUPT, RTC_CONTROL);

				return 0;
			}
		case RTC_PIE_ON:	/* periodic on */
			{
				rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL) & ~ENABLE_SEC_INTERRUPT, RTC_CONTROL);

				return 0;
			}
		default:
				return -ENOTTY;
	}
		
	return 0;
}


/* Time read/write */
static int
ast_rtc_get_time(struct device *dev, struct rtc_time *rtc_tm)
{
        struct ast_rtc *ast_rtc = dev_get_drvdata(dev);
        unsigned long flags;
        u32 reg_time, reg_date;

        spin_lock_irqsave(&ast_rtc->lock, flags);

		reg_time = rtc_read(ast_rtc->base, RTC_CNTR_STS_1);
		reg_date = rtc_read(ast_rtc->base, RTC_CNTR_STS_2);
		
		spin_unlock_irqrestore(&ast_rtc->lock, flags);

		rtc_tm->tm_year = GET_CENT_VAL(reg_date)*1000 | GET_YEAR_VAL(reg_date);
		rtc_tm->tm_mon = GET_MON_VAL(reg_date);

		rtc_tm->tm_mday = GET_DAY_VAL(reg_time);
		rtc_tm->tm_hour = GET_HOUR_VAL(reg_time);
		rtc_tm->tm_min = GET_MIN_VAL(reg_time);
		rtc_tm->tm_sec = GET_SEC_VAL(reg_time);
		
        pr_debug("read time %02x.%02x.%02x %02x/%02x/%02x\n",
                 rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday,
                 rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);
		return 0;

}

static int
ast_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
    struct ast_rtc *ast_rtc = dev_get_drvdata(dev);
    unsigned long flags;
    u32 reg_time, reg_date;

	pr_debug("set time %02d.%02d.%02d %02d/%02d/%02d\n",
			 tm->tm_year, tm->tm_mon, tm->tm_mday,
			 tm->tm_hour, tm->tm_min, tm->tm_sec);

    spin_lock_irqsave(&ast_rtc->lock, flags);

	/* set hours */
	reg_time = SET_DAY_VAL(tm->tm_mday) | SET_HOUR_VAL(tm->tm_hour) | SET_MIN_VAL(tm->tm_min) | SET_SEC_VAL(tm->tm_sec);
	
	/* set century */
	/* set mon */
	reg_date = SET_CENT_VAL(tm->tm_year / 1000) |  SET_YEAR_VAL(tm->tm_year % 1000) | SET_MON_VAL(tm->tm_mon);

	rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL) | RTC_LOCK, RTC_CONTROL);

	rtc_write(ast_rtc->base, reg_time, RTC_CNTR_STS_1);
	rtc_write(ast_rtc->base, reg_date, RTC_CNTR_STS_2);

	rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL) &~RTC_LOCK , RTC_CONTROL);
	
	spin_unlock_irqrestore(&ast_rtc->lock, flags);

	return 0;	
}
static int
ast_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
    struct ast_rtc *ast_rtc = dev_get_drvdata(dev);
    unsigned long flags;
    struct rtc_time *alm_tm = &alarm->time;
    u32 alarm_reg;

    spin_lock_irqsave(&ast_rtc->lock, flags);
	alarm_reg = rtc_read(ast_rtc->base, RTC_ALARM);
	spin_unlock_irqrestore(&ast_rtc->lock, flags);

//DAY	
	alm_tm->tm_mday = GET_DAY_VAL(alarm_reg);

//HR
	alm_tm->tm_hour = GET_HOUR_VAL(alarm_reg);

//MIN
	alm_tm->tm_min= GET_MIN_VAL(alarm_reg);

//SEC
	alm_tm->tm_sec= GET_SEC_VAL(alarm_reg);

    pr_debug("ast_rtc_read_alarm: %d, %02x %02x.%02x.%02x\n",
             alarm->enabled,
             alm_tm->tm_mday & 0xff, alm_tm->tm_hour & 0xff, alm_tm->tm_min & 0xff, alm_tm->tm_sec);

	return 0;


}

static int
ast_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
    struct ast_rtc *ast_rtc = dev_get_drvdata(dev);
    struct rtc_time *tm = &alarm->time;
    unsigned long flags;
    u32 reg_alarm = 0;

    pr_debug("ast_rtc_setalarm: %d, %02x %02x.%02x.%02x\n",
             alarm->enabled,
             tm->tm_mday & 0xff, tm->tm_hour & 0xff, tm->tm_min & 0xff, tm->tm_sec);

//DAY
    /* set day of week */
    if (tm->tm_mday <= 31 && tm->tm_mday >= 1) {
			reg_alarm |= SET_DAY_VAL(tm->tm_mday);
    }
    
//HR
    /* set ten hours */
    if (tm->tm_hour <= 23 && tm->tm_hour >= 0) {
            reg_alarm |= SET_HOUR_VAL(tm->tm_hour);
    }

//MIN
    /* set ten minutes */
    if (tm->tm_min <= 59 && tm->tm_min >= 0) {
            reg_alarm |= SET_MIN_VAL(tm->tm_min);
    }

//SEC
    /* set ten secondss */
    if (tm->tm_sec <= 59 && tm->tm_sec >= 0) {
            reg_alarm |= SET_SEC_VAL(tm->tm_sec);
    }

	pr_debug("ast_rtc_set alarm reg: %x \n", reg_alarm);

	spin_lock_irqsave(&ast_rtc->lock, flags);

	rtc_write(ast_rtc->base, reg_alarm, RTC_ALARM);
	
	if(alarm->enabled)	
		rtc_write(ast_rtc->base, reg_alarm, RTC_CONTROL);
	else
		rtc_write(ast_rtc->base, reg_alarm, RTC_CONTROL);

	spin_unlock_irqrestore(&ast_rtc->lock, flags);
	return 0;

}
static int
ast_rtc_proc(struct device *dev, struct seq_file *seq)
{
        struct ast_rtc *ast_rtc = dev_get_drvdata(dev);
        u32 ctrl_reg;

        ctrl_reg = rtc_read(ast_rtc->base, RTC_CONTROL);

        pr_debug("ctrl_reg = 0x%08x\n", ctrl_reg);

        seq_printf(seq, "periodic_IRQ\t: %s\n",
                     (ctrl_reg & ENABLE_SEC_INTERRUPT) ? "yes" : "no" );

        return 0;
}

static int
ast_rtc_irq_set_freq(struct device *dev, int freq)
{
	struct ast_rtc *ast_rtc = dev_get_drvdata(dev);
	pr_debug("freq = %d\n", freq);

	spin_lock_irq(&ast_rtc->lock);

	if(freq == 0)
		rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL)&~ENABLE_SEC_INTERRUPT, RTC_CONTROL);
	else
		rtc_write(ast_rtc->base, rtc_read(ast_rtc->base, RTC_CONTROL)|ENABLE_SEC_INTERRUPT, RTC_CONTROL);

	spin_unlock_irq(&ast_rtc->lock);

	return 0;
}

static irqreturn_t
ast_rtc_interrupt(int irq, void *dev_id)
{
	struct ast_rtc *ast_rtc = dev_id;

	unsigned int status = rtc_read(ast_rtc->base, RTC_ALARM_STS);
	rtc_write(ast_rtc->base, status, RTC_ALARM_STS);

	if (status & SEC_INTERRUPT_STATUS) {
        printk("RTC Alarm SEC_INTERRUPT_STATUS!!\n");
	}

	if (status & DAY_ALARM_STATUS) {
        printk("RTC Alarm DAY_ALARM_STATUS!!\n");
	}

	if (status & HOUR_ALARM_STATUS) {
        printk("RTC Alarm HOUR_ALARM_STATUS!!\n");
	}

	if (status & MIN_ALARM_STATUS) {
        printk("RTC Alarm MIN_ALARM_STATUS!!\n");
	}

	if (status & SEC_ALARM_STATUS) {
        printk("RTC Alarm SEC_ALARM_STATUS!!\n");
	}

	rtc_update_irq(ast_rtc->rtc_dev, 1, RTC_AF | RTC_IRQF);

	return (IRQ_HANDLED);
}

static struct rtc_class_ops ast_rtcops = {
        .ioctl            = ast_rtc_ioctl,
        .read_time        = ast_rtc_get_time,
        .set_time         = ast_rtc_set_time,
        .read_alarm       = ast_rtc_read_alarm,
        .set_alarm        = ast_rtc_set_alarm,
        .proc             = ast_rtc_proc,
        .irq_set_freq     = ast_rtc_irq_set_freq,
};

/*
 * Initialize and install RTC driver
 */
static int __init ast_rtc_probe(struct platform_device *pdev)
{
	struct ast_rtc *ast_rtc;
	struct rtc_device *rtc_dev;
	struct resource *res;
	int ret;

	pr_debug("%s: probe=%p\n", __func__, pdev);

	ast_rtc = kzalloc(sizeof *ast_rtc, GFP_KERNEL);
	if (!ast_rtc)
			return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
			dev_err(&pdev->dev, "register resources unusable\n");
			ret = -ENXIO;
			goto  free_rtc;
	}

	ast_rtc->irq = platform_get_irq(pdev, 0);
	if (ast_rtc->irq < 0) {
			dev_err(&pdev->dev, "unable to get irq\n");
			ret = -ENXIO;
			goto free_rtc;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
			ret = -EBUSY;
			goto free_rtc;
	}
	
	ast_rtc->base = ioremap(res->start, resource_size(res));
	if (!ast_rtc->base) {
			dev_err(&pdev->dev, "cannot map SocleDev registers\n");
			ret = -ENOMEM;
			goto release_mem;
	}
	
	pr_debug("base = 0x%p, irq = %d\n", ast_rtc->base, ast_rtc->irq);

	rtc_dev = rtc_device_register(pdev->name, &pdev->dev, &ast_rtcops, THIS_MODULE);
	if (IS_ERR(rtc_dev)) {
			ret = PTR_ERR(rtc_dev);
			goto unmap;
	}
	
	ast_rtc->res = res;
	ast_rtc->rtc_dev = rtc_dev;
	spin_lock_init(&ast_rtc->lock);

	platform_set_drvdata(pdev, ast_rtc);

//	ast_rtc_irq_set_freq(&pdev->dev, 1);

	/* start the RTC from dddd:hh:mm:ss = 0000:00:00:00 */
	spin_lock_irq(&ast_rtc->lock);
	if(!(rtc_read(ast_rtc->base, RTC_CONTROL) & RTC_ENABLE)) {
		//combination mode
		rtc_write(ast_rtc->base, ALARM_MODE_SELECT | RTC_LOCK | RTC_ENABLE, RTC_CONTROL);

		rtc_write(ast_rtc->base, 0, RTC_CNTR_STS_1);

		rtc_write(ast_rtc->base, 0, RTC_CNTR_STS_2);

		rtc_write(ast_rtc->base, 0, RTC_ALARM);
		rtc_write(ast_rtc->base, ~RTC_LOCK & rtc_read(ast_rtc->base, RTC_CONTROL), RTC_CONTROL);
	} else
		printk("no need to enable RTC \n");

	spin_unlock_irq(&ast_rtc->lock);
	
	/* register ISR */
	ret = request_irq(ast_rtc->irq, ast_rtc_interrupt, IRQF_DISABLED, dev_name(&rtc_dev->dev), ast_rtc);
	if (ret) {
		printk(KERN_ERR "ast_rtc: IRQ %d already in use.\n",
				ast_rtc->irq);
		goto unregister;
	}

	return 0;

unregister:
	rtc_device_unregister(rtc_dev);
	platform_set_drvdata(pdev, NULL);
unmap:
	iounmap(ast_rtc->base);
release_mem:
	release_mem_region(res->start, resource_size(res));
free_rtc:
	kfree(ast_rtc);
	return ret;

}

/*
 * Disable and remove the RTC driver
 */
static int __exit ast_rtc_remove(struct platform_device *pdev)
{
	struct ast_rtc *ast_rtc = platform_get_drvdata(pdev);

	free_irq(IRQ_RTC, pdev);
	rtc_device_unregister(ast_rtc->rtc_dev);
	platform_set_drvdata(pdev, NULL);
	iounmap(ast_rtc->base);
	release_resource(ast_rtc->res);
	kfree(ast_rtc);

	return 0;
}

#ifdef CONFIG_PM

/* ASPEED RTC Power management control */
static int ast_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int ast_rtc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define ast_rtc_suspend NULL
#define ast_rtc_resume  NULL
#endif

static struct platform_driver ast_rtc_driver = {
	.probe		= ast_rtc_probe,
	.remove		= __exit_p(ast_rtc_remove),
	.suspend	= ast_rtc_suspend,
	.resume		= ast_rtc_resume,
	.driver		= {
		.name	= "ast_rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init ast_rtc_init(void)
{
	return platform_driver_register(&ast_rtc_driver);
}

static void __exit ast_rtc_exit(void)
{
	platform_driver_unregister(&ast_rtc_driver);
}

module_init(ast_rtc_init);
module_exit(ast_rtc_exit);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("RTC driver for ASPEED AST ");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ast_rtc");

