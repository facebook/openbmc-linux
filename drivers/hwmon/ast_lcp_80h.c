/*
 *  ast_lpc_snoop.c
 *
 *  ASPEED LPC Snoop controller driver
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

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <plat/regs-1070_lpc.h>

struct ast_clpc_data {
	struct device			*hwmon_dev;
	void __iomem			*reg_base;			/* virtual */
	int 					irq;				//ADC IRQ number 
	u8 						80h_data;			//80h_data
};

static inline void
ast_clpc_write(struct ast_clpc_data *ast_clpc, u32 val, u32 reg)
{
//	printk("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_clpc->reg_base+ reg);
}

static inline u32
ast_clpc_read(struct ast_adc_data *ast_clpc, u32 reg)
{
	u32 val = readl(ast_clpc->reg_base + reg);
//	printk("read offset: %x, val: %x \n",reg,val);
	return val;
}

static irqreturn_t ast_lpc_80h_handler(int irq, void *dev_id)
{
	struct ast_clpc_data *ast_clpc = dev_id;
	u32 sts = ast_clpc_read(ast_clpc, AST1070_LPC_80H_CTRL);

	if(isr_sts & AST1070_LPC_80H_CLR) {
		ast_clpc->80h_data = ast_clpc_read(ast_clpc, AST1070_LPC_80H_DATA);	
		ast_clpc_write(ast_clpc, AST1070_LPC_80H_CLR, AST1070_LPC_80H_DATA);	
	} else
		printk("IRQ ISSUE bug \n");

	return IRQ_HANDLED;

}

static void ast_clpc_80h_init(struct ast_clpc_data *ast_clpc, u16 addr)
{
	ast_clpc_write(ast_clpc, AST1070_LPC_80H_CLR, AST1070_LPC_80H_CTRL);
	
	//Snoop Port
	ast_clpc_write(ast_clpc, addr & 0xff, AST1070_LPC_L_80H_ADDR);
	ast_clpc_write(ast_clpc, (addr & 0xff) >> 8 , AST1070_LPC_H_80H_ADDR);
	//Clear Interrupt and Enable
	//AST1070 BUG :===: D[4] W1C	
	ast_clpc_write(ast_clpc, AST1070_LPC_80H_CLR, AST1070_LPC_80H_DATA);		
	ast_clpc_write(ast_clpc, AST1070_LPC_80H_CLR | AST1070_LPC_80H_EN, AST1070_LPC_80H_CTRL);
}

/* attr 80H sysfs 0~max adc channel 
*	 0 - show/store 80h addr
*	 1 - show 80h data
*/

static ssize_t 
ast_show_clpc(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);
	u16 tmp;
	u32 voltage,tmp1, tmp2,tmp3;

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //channel enable, disable
			return sprintf(sysfsbuf, "%d \n", ast_clpc->80h_data);
			break;

		default:
			return -EINVAL;
			break;
	}
}

static ssize_t 
ast_store_clpc(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable

			break;
		default:
			return -EINVAL;
			break;
	}

	return count;
}

/* attr ADC sysfs 0~max adc channel 
*	 0 - show 80h data
*/

#define sysfs_clpc(index) \
static SENSOR_DEVICE_ATTR_2(clpc##index##_en, S_IRUGO | S_IWUSR, \
	ast_show_clpc, NULL, 0, index); \
\
static struct attribute *clpc##index##_attributes[] = { \
	&sensor_dev_attr_clpc##index##_80h.dev_attr.attr, \
	NULL \
};

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_clpc(0);

static const struct attribute_group clpc_attribute_groups[] = {
	{ .attrs = clpc0_attributes },
};


static int 
ast_clpc_probe(struct platform_device *pdev)
{
	struct ast_clpc_data *ast_clpc;
	struct resource *res;
	int err;
	int ret=0;
	int i;

	dev_dbg(&pdev->dev, "ast_clpc_probe \n");

	ast_clpc = kzalloc(sizeof(struct ast_clpc_data), GFP_KERNEL);
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

	ast_clpc->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_clpc->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_clpc->irq = platform_get_irq(pdev, 3);
	if (ast_clpc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}


	/* Register sysfs hooks */
	ast_clpc->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(ast_clpc->hwmon_dev)) {
		ret = PTR_ERR(ast_clpc->hwmon_dev);
		goto out_region;
	}

	for(i=0; i< MAX_CH_NO; i++) {
		err = sysfs_create_group(&pdev->dev.kobj, &clpc_attribute_groups[i]);
		if (err)
			goto out_region;
	}

	ast_clpc_80h_init();

	ret = request_irq(ast_clpc->irq, ast_lpc_handler, IRQF_SHARED,
			  i2c_dev->adap.name, i2c_dev);
	if (ret) {
		printk(KERN_INFO "I2C: Failed request irq %d\n", i2c_dev->irq);
		goto out_region;
	}

	platform_set_drvdata(pdev, ast_clpc);
	
	printk(KERN_INFO "ast_adc: driver successfully loaded.\n");

	return 0;


//out_irq:
//	free_irq(ast_clpc->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out_mem:
	kfree(ast_clpc);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int 
ast_adc_remove(struct platform_device *pdev)
{
	int i=0;
	struct ast_adc_data *ast_clpc = platform_get_drvdata(pdev);
	struct resource *res;
	printk(KERN_INFO "ast_adc: driver unloaded.\n");

    hwmon_device_unregister(ast_clpc->hwmon_dev);

	for(i=0; i<5; i++)
		sysfs_remove_group(&pdev->dev.kobj, &clpc_attribute_groups[i]);

	platform_set_drvdata(pdev, NULL);
//	free_irq(ast_adc->irq, ast_adc);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ast_clpc->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);
	kfree(ast_clpc);
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
