/*
 *  ast_adc.c
 *
 *  ASPEED ADC controller driver
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2012.11.26: Initial version [Ryan Chen]
 */

/* attr ADC sysfs 0~max adc channel 
*	 0 - show/store enable
*	 3 - show value
*	 1 - show/store alarm_en set enable 
*	 2 - show alarm   get statuse
*	 4 - show/store upper
*	 5 - show/store lower  */


#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm/arch/regs-adc.h>
#include <asm/arch/ast-scu.h>

struct ast_adc_data {
	struct device			*hwmon_dev;
	void __iomem			*reg_base;			/* virtual */
	int 					irq;				//ADC IRQ number 
	s8						compen_value;		//Compensating value
	u8						trim;
};

struct ast_adc_data *ast_adc;

static u8 ast_get_adc_en(struct ast_adc_data *ast_adc, u8 adc_ch);


static inline void
ast_adc_write(struct ast_adc_data *ast_adc, u32 val, u32 reg)
{
//	printk("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_adc->reg_base+ reg);
}

static inline u32
ast_adc_read(struct ast_adc_data *ast_adc, u32 reg)
{
	u32 val = readl(ast_adc->reg_base + reg);
	//Workaround for command cycle response ..	
	val = readl(ast_adc->reg_base + reg);
	
//	printk("read offset: %x, val: %x \n",reg,val);
	return val;
}

static void ast1010_adc_ctrl_init(void)
{
	ast_adc->trim = ast_scu_otp_read(0);

	if((ast_adc->trim == 0x0))
		ast_adc->trim = 0x80;
	
	ast_adc->compen_value = ast_scu_otp_read(1);		
//	printk("trim = %x, cv = %d \n",ast_adc->trim, ast_adc->compen_value);

	ast_adc_write(ast_adc, ast_adc->trim, AST_ADC_TRIM);
	//TODO ..... clk
	ast_adc_write(ast_adc, 0x40, AST_ADC_CLK);

	ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CTRL) & ~AST_ADC_CTRL_PWR_DWN, AST_ADC_CTRL);	
	//wait for 50us
	udelay(1);
	ast_adc_write(ast_adc, AST_ADC_CTRL_CH0_EN | AST_ADC_CTRL_CH1_EN |
						AST_ADC_CTRL_CH2_EN | AST_ADC_CTRL_CH3_EN |
						AST_ADC_CTRL_CH4_EN | AST_ADC_CTRL_CH5_EN |
						AST_ADC_CTRL_CH6_EN | AST_ADC_CTRL_CH7_EN |
						AST_ADC_CTRL_REFV | AST_ADC_CTRL_EN, AST_ADC_CTRL);	

}

static u16
ast_get_adc_value(struct ast_adc_data *ast_adc, u32 adc_ch)
{
	u32 ub;
	u32 lb;
	u32 i;
	u32 bound;
	u32 pre_ub;
	u32 pre_lb;

    int tmp;

	
	ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CTRL) & ~(AST_ADC_CH_EN(adc_ch)), AST_ADC_CTRL);	
	ast_adc_write(ast_adc, 0x03ff0000, AST_ADC_BOUND0 + adc_ch*4);
	ast_adc_write(ast_adc, AST_ADC_IER_CH(adc_ch), AST_ADC_IER);
	

	pre_ub = 0x3ff;
	pre_lb = 0x000;
	ub = 0x3ff;
	lb = 0x1ff;
	for(i=0;i<10;i++)
	{
		bound = ((ub << 16) & 0x3ff0000) | (lb & 0x3ff);
//		printk("Upper Bound : %x\n", ub);
//		printk("Lower Bound : %x\n", lb);
		ast_adc_write(ast_adc, bound, AST_ADC_BOUND0 + adc_ch*4);
		ast_adc_write(ast_adc, AST_ADC_IER_CH(adc_ch), AST_ADC_IER);
		
		if((AST_ADC_STS_CH(adc_ch) & ast_adc_read(ast_adc, AST_ADC_IER)) == 0)
		{
			pre_ub = ub;
			pre_lb = lb;
			ub = ub;
			lb = (pre_ub + pre_lb) >> 1;
		}
		else
		{
			ub = lb;
			lb = (lb + pre_lb) >> 1;
		}
	}
//	printk("ub %x ,lb %x\n",ub,lb);
	bound = ((ub << 16) & 0x3ff0000) | (ub & 0x3ff);
	ast_adc_write(ast_adc, bound, AST_ADC_BOUND0 + adc_ch*4);
	ast_adc_write(ast_adc, AST_ADC_IER_CH(adc_ch), AST_ADC_IER);
	if((AST_ADC_STS_CH(adc_ch) & ast_adc_read(ast_adc, AST_ADC_IER)) == 0) {
//		printk("yes ub %x \n",ub);
		tmp = ub;
	} else {
//		printk("no lb %x \n",lb);	
		tmp = lb;
	}

	//clear
	ast_adc_write(ast_adc, 0, AST_ADC_IER);
	ast_adc_write(ast_adc, 0, AST_ADC_BOUND0 + adc_ch*4);	
	ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CTRL) | AST_ADC_CH_EN(adc_ch), AST_ADC_CTRL);	

	
//	printk("ub %x , lb %x, voltage = %x \n",ub, lb ,tmp);	

	tmp += ast_adc->compen_value;

	return tmp;

}

/* attr ADC sysfs 0~max adc channel 
*	 0 - show value
*/

static ssize_t 
ast_show_adc(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);
	u16 tmp;
	u32 voltage;

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //value
			tmp = ast_get_adc_value(ast_adc, sensor_attr->index);
			return sprintf(sysfsbuf, "%d \n", tmp);
			break;

		default:
			return -EINVAL;
			break;
	}
}

/* attr ADC sysfs 0~max adc channel 
*	 0 - show value 
*/

#define sysfs_adc_ch(index) \
static SENSOR_DEVICE_ATTR_2(adc##index##_value, S_IRUGO | S_IWUSR, \
	ast_show_adc, NULL, 0, index); \
\
static struct attribute *adc##index##_attributes[] = { \
	&sensor_dev_attr_adc##index##_value.dev_attr.attr, \
	NULL \
};

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_adc_ch(0);
sysfs_adc_ch(1);
sysfs_adc_ch(2);
sysfs_adc_ch(3);
sysfs_adc_ch(4);
sysfs_adc_ch(5);
sysfs_adc_ch(6);
sysfs_adc_ch(7);

static const struct attribute_group adc_attribute_groups[] = {
	{ .attrs = adc0_attributes },
	{ .attrs = adc1_attributes },
	{ .attrs = adc2_attributes },
	{ .attrs = adc3_attributes },
	{ .attrs = adc4_attributes },
	{ .attrs = adc5_attributes },
	{ .attrs = adc6_attributes },
	{ .attrs = adc7_attributes },
};


static int 
ast_adc_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;
	int ret=0;
	int i;

	dev_dbg(&pdev->dev, "ast_adc_probe \n");

	ast_adc = kzalloc(sizeof(struct ast_adc_data), GFP_KERNEL);
	if (!ast_adc) {
		ret = -ENOMEM;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_mem;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out_mem;
	}

	ast_adc->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_adc->reg_base) {
		ret = -EIO;
		goto out_region;
	}

/*
	ast_adc->irq = platform_get_irq(pdev, 0);
	if (ast_adc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}
*/

	/* Register sysfs hooks */
	ast_adc->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(ast_adc->hwmon_dev)) {
		ret = PTR_ERR(ast_adc->hwmon_dev);
		goto out_region;
	}

	for(i=0; i<MAX_CH_NO; i++) {
		err = sysfs_create_group(&pdev->dev.kobj, &adc_attribute_groups[i]);
		if (err)
			goto out_region;
	}

	ast1010_adc_ctrl_init();
	
	printk(KERN_INFO "ast_adc: driver successfully loaded.\n");

	return 0;


//out_irq:
//	free_irq(ast_adc->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out_mem:
	kfree(ast_adc);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int 
ast_adc_remove(struct platform_device *pdev)
{
	int i=0;
	struct ast_adc_data *ast_adc = platform_get_drvdata(pdev);
	struct resource *res;
	printk(KERN_INFO "ast_adc: driver unloaded.\n");

    hwmon_device_unregister(ast_adc->hwmon_dev);

	for(i=0; i<5; i++)
		sysfs_remove_group(&pdev->dev.kobj, &adc_attribute_groups[i]);

	platform_set_drvdata(pdev, NULL);
//	free_irq(ast_adc->irq, ast_adc);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ast_adc->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);
	kfree(ast_adc);
	return 0;
}

#ifdef CONFIG_PM
static int 
ast_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_adc_suspend : TODO \n");
	return 0;
}

static int 
ast_adc_resume(struct platform_device *pdev)
{
	ast_adc_ctrl_init();
	return 0;
}

#else
#define ast_adc_suspend        NULL
#define ast_adc_resume         NULL
#endif

static struct platform_driver ast_adc_driver = {
	.probe			= ast_adc_probe,
	.remove 		= __devexit_p(ast_adc_remove),
    .suspend        = ast_adc_suspend,
    .resume         = ast_adc_resume,
    .driver         = {
            .name   = "ast_adc",
            .owner  = THIS_MODULE,
    },
};

static int __init 
ast_adc_init(void)
{
	return platform_driver_register(&ast_adc_driver);
}

static void __exit 
ast_adc_exit(void)
{
	platform_driver_unregister(&ast_adc_driver);
}

module_init(ast_adc_init);
module_exit(ast_adc_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ADC driver");
MODULE_LICENSE("GPL");
