// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * alert_l.c - Alert_l driver for AMD APML devices
 *
 * Copyright (C) 2023-2024 Advanced Micro Devices, Inc.
 */

#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i3c/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>

#include "sbrmi-common.h"
#include "apml_alertl.h"

#define DRIVER_NAME "apml_alertl"

#define RAS_STATUS_REG		0x4C
#define STATUS_REG		0x2
#define RAS_ALERT_STATUS	BIT(1)

MODULE_ALIAS("apml_alertl:" DRIVER_NAME);

#ifdef CONFIG_DEBUG_FS
struct dentry *amd_apml;
struct task_struct *p_task;
u64 proc_pid;
#endif

static irqreturn_t alert_l_irq_thread_handler(int irq, void *dev_id)
{
	struct apml_alertl_data *oob_adata = dev_id;
	struct apml_message msg = { 0 };
	struct kernel_siginfo info = {0};
	unsigned int status = 0;
	int ret, i;

	/*
	 * Read RAS Status register to identify the RAS error
	 * Currently only RAS fatal error is supported
	 */

	for (i = 0; i < oob_adata->num_of_rmi_devs; i++) {
		if (!oob_adata->rmi_dev[i] || !oob_adata->rmi_dev[i]->regmap)
			continue;
		msg.data_in.reg_in[REG_OFF_INDEX] = RAS_STATUS_REG;

		ret = regmap_read(oob_adata->rmi_dev[i]->regmap,
				  msg.data_in.reg_in[REG_OFF_INDEX],
				  &status);
		if (ret < 0)
			return ret;
		if (status)
			break;
	}

	if (!status)
		return IRQ_HANDLED;

	/* For RAS errors, signal the registered program*/
	info.si_signo = USR_SIGNAL;
	info.si_int = status | (oob_adata->rmi_dev[i]->dev_static_addr << 16);

	pr_debug("Sending signal to the process, RAS bit is set sigint is %x\n", info.si_int);
	p_task = pid_task(find_get_pid(proc_pid), PIDTYPE_PID);
	if (p_task) {
		ret = send_sig_info(USR_SIGNAL, &info, p_task);
		if (ret < 0)
			pr_err("Sending signal to the process, unsuccessful\n");
			/* RAS status(0x4c) and Status register(0x2) bits clear is
			 * required even if sending signal to user application
			 * fails.
			 * So no return even if signal send fails.
			 */
	}

	/* Clear the RAS Status register */
	if (!oob_adata->rmi_dev[i] || !oob_adata->rmi_dev[i]->regmap)
		return -ENODEV;

	mutex_lock(&oob_adata->rmi_dev[i]->lock);
	msg.data_in.reg_in[REG_OFF_INDEX] = RAS_STATUS_REG;
	ret = regmap_write(oob_adata->rmi_dev[i]->regmap,
			   msg.data_in.reg_in[REG_OFF_INDEX],
			   status);

	msg.data_in.reg_in[REG_OFF_INDEX] = STATUS_REG;
	ret = regmap_write(oob_adata->rmi_dev[i]->regmap,
			   msg.data_in.reg_in[REG_OFF_INDEX],
			   RAS_ALERT_STATUS);
	mutex_unlock(&oob_adata->rmi_dev[i]->lock);
	if (ret < 0)
		return ret;

	return IRQ_HANDLED;
}

#ifdef CONFIG_DEBUG_FS
static int proc_pid_store(void *data, u64 value)
{
	struct task_struct *ptask;
	int ret = 0;

	if (value == 0)
		return -EINVAL;
	/* The new value will override the previous value */
	proc_pid = value;
	ptask = pid_task(find_get_pid(proc_pid), PIDTYPE_PID);
	if (!ptask) {
		pr_err("PID not found\n");
		return -EINVAL;
	}

	return ret;
}

static int proc_pid_show(void *data, u64 *value)
{
	*value = proc_pid;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(proc_pid_fops, proc_pid_show, proc_pid_store, "%llu\n");
#endif

static void *get_apml_dev_byphandle(struct device_node *dnode,
				    const char *phandle_name,
				    int index)
{
	struct device_node *d_node;
	struct device *dev;
	void *apml_dev;

	if (!phandle_name || !dnode)
		return NULL;

	d_node = of_parse_phandle(dnode, phandle_name, index);
	if (IS_ERR_OR_NULL(d_node))
		return NULL;

	dev = bus_find_device(&i3c_bus_type, NULL, d_node, sbrmi_match_i3c);
	if (!dev) {
		dev = bus_find_device(&i2c_bus_type, NULL, d_node, sbrmi_match_i2c);
		if (IS_ERR_OR_NULL(dev)) {
			of_node_put(d_node);
			return NULL;
		}
	}

	of_node_put(d_node);
	apml_dev = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(apml_dev))
		return NULL;

	return apml_dev;
}

static int apml_alertl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dnode = dev->of_node;
	struct apml_sbrmi_device **rmi_dev;
	struct apml_alertl_data *oob_alert;
	struct gpio_desc *alertl_gpiod;
	u32 irq_num;
	u32 num_dev = 0;
	int ret = 0;
	int i = 0;

	/* Allocate memory to oob_alert_data structure */
	oob_alert = devm_kzalloc(dev, sizeof(struct apml_alertl_data),
				 GFP_KERNEL);
	if (!oob_alert)
		return -ENOMEM;
	/* identify the number of devices associated with each alert */
	num_dev = of_property_count_elems_of_size(dnode, "sbrmi",
						  sizeof(phandle));
	oob_alert->num_of_rmi_devs = num_dev;

	/* Allocate memory as per the number of rmi devices */
	rmi_dev = devm_kzalloc(dev, num_dev * sizeof(struct apml_sbrmi_device), GFP_KERNEL);
	if (!rmi_dev)
		return -ENOMEM;
	oob_alert->rmi_dev = rmi_dev;

	/*
	 * For each of the Alerts get the device associated
	 * Currently the ALert_L driver identification is only supported
	 * over I3C. We can add property in dts to identify the bus type
	 */

	for (i = 0; i < num_dev; i++) {
		rmi_dev[i] = get_apml_dev_byphandle(pdev->dev.of_node, "sbrmi", i);
		if (!rmi_dev[i]) {
			pr_err("Error getting APML SBRMI device. Exiting\n");
			return -EINVAL;
		}
	}

	/* Get the alert_l gpios, irq_number for the GPIO and register ISR*/
	alertl_gpiod = devm_gpiod_get(dev, NULL, GPIOD_IN);
	if (IS_ERR(alertl_gpiod)) {
		dev_err(&pdev->dev, "Unable to retrieve gpio\n");
		return PTR_ERR(alertl_gpiod);
	}

	ret = gpiod_to_irq(alertl_gpiod);
	if (ret < 0) {
		dev_err(dev, "No corresponding irq for gpio error: %d\n", ret);
		return ret;
	}
	irq_num = ret;
	pr_debug("Register IRQ:%u\n", irq_num);
	ret = devm_request_threaded_irq(dev, irq_num,
					NULL,
					(void *)alert_l_irq_thread_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"apml_irq", oob_alert);
	if (ret) {
		pr_err("Cannot register IRQ:%u\n", irq_num);
		return ret;
	}

	/* Set the platform data to pdev */
	platform_set_drvdata(pdev, oob_alert);

	/*
	 * Create a sys entry to register user application PID
	 * Only one debugfs entry is created for all apml alerts on
	 * the platform (the idea of debugfs entry is only for RAS consumers)
	 */
#ifdef CONFIG_DEBUG_FS
	pr_debug("Creating debugfs files");
	if (!amd_apml) {
		amd_apml = debugfs_create_dir("apml_alertl", NULL);
		if (IS_ERR_OR_NULL(amd_apml))
			return -ENOMEM;
		debugfs_create_file("ras_fatal_pid", 0600, amd_apml, oob_alert, &proc_pid_fops);
	}
#endif
	return 0;
}

static int alert_remove(struct platform_device *pdev)
{
#ifdef CONFIG_DEBUG_FS
	if (amd_apml) {
		debugfs_remove_recursive(amd_apml);
		amd_apml = NULL;
	}
#endif
	return 0;
}

static const struct of_device_id apml_alertl_dt_ids[] = {
	{ .compatible = "apml-alertl", },
	{},
};
MODULE_DEVICE_TABLE(of, apml_alertl_dt_ids);

static struct platform_driver apml_alertl_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(apml_alertl_dt_ids),
	},
	.probe		= apml_alertl_probe,
	.remove		= alert_remove,
};

module_platform_driver(apml_alertl_driver);
MODULE_AUTHOR("Akshay Gupta <akshay.gupta@amd.com>");
MODULE_AUTHOR("Naveenkrishna Chatradhi <naveenkrishna.chatradhi@amd.com>");
MODULE_DESCRIPTION("AMD APML ALERT_L Driver");
MODULE_LICENSE("GPL");
