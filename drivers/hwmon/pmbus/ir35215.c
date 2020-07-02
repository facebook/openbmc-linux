// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Infineon IR35215
 *
 * Copyright (c) 2020 Facebook Inc.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "pmbus.h"

static struct pmbus_driver_info ir35215_info = {
	.pages = 2,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_TEMP
		| PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT
		| PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT
		| PMBUS_HAVE_PIN | PMBUS_HAVE_POUT
		| PMBUS_HAVE_STATUS_VOUT | PMBUS_HAVE_STATUS_IOUT
		| PMBUS_HAVE_STATUS_INPUT | PMBUS_HAVE_STATUS_TEMP,
	.func[1] = PMBUS_HAVE_TEMP
		| PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT
		| PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT
		| PMBUS_HAVE_PIN | PMBUS_HAVE_POUT
		| PMBUS_HAVE_STATUS_VOUT | PMBUS_HAVE_STATUS_IOUT
		| PMBUS_HAVE_STATUS_INPUT | PMBUS_HAVE_STATUS_TEMP,
};

static int ir35215_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	return pmbus_do_probe(client, id, &ir35215_info);
}

static const struct i2c_device_id ir35215_id[] = {
	{"ir35215", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ir35215_id);

static struct i2c_driver ir35215_driver = {
	.driver = {
		   .name = "ir35215",
		   },
	.probe = ir35215_probe,
	.remove = pmbus_do_remove,
	.id_table = ir35215_id,
};

module_i2c_driver(ir35215_driver);

MODULE_AUTHOR("Tao Ren <rentao.bupt@gmail.com>");
MODULE_DESCRIPTION("PMBus driver for Infineon IR35215");
MODULE_LICENSE("GPL");
