// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware monitoring driver for PFE3000 and compatibles
 * Based on the zl6100 driver with the following copyright:
 *
 * Copyright (c) 2011 Ericsson AB.
 * Copyright (c) 2012 Guenter Roeck
 * Copyright 2004-present Facebook. All Rights Reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include "pmbus.h"

#define PFE3000_MFR_MODEL     "PFE3000-12-069RA"
#define PFE3000_OPERATION_ON  PB_OPERATION_CONTROL_ON
#define PFE3000_OPERATION_OFF 0x0
#define PFE3000_SYSFS_CMD_OFF 1
#define PFE3000_ACCESS_DELAY  1000 /* delay between chip accesses */

/*
 * TODO add multi-page support: PFE3000 supports page 0, 1, 2, 4, 5
 * and 6, but the driver only supports page 0 as of now.
 */
#define PFE3000_NUM_PAGES     1
#define PFE3000_VALID_PAGE(p) ((p) >= -1 && (p) < PFE3000_NUM_PAGES)

enum pfe3000_model {
	PFE3000_12_069RA,
};

struct pfe3000_data {
	int id;
	int shutdown_state;
	ktime_t access_time; /* time when the chip was accessed */
	int access_delay;    /* delay between chip access in usec */
	struct pmbus_driver_info info;
#define to_pfe3000_data(x) container_of(x, struct pfe3000_data, info)
};

/*
 * Some PFE3000 devices may fail to return MFR_MODEL, and "ignore_probe"
 * parameter allows users to ignore such failure and continue device
 * binding.
 */
static bool ignore_probe;
module_param(ignore_probe, bool, 0);
MODULE_PARM_DESC(ignore_probe, "Ignore probe failures.");

static void pfe3000_update_access_time(struct pfe3000_data *data)
{
	data->access_time = ktime_get();
}

static void pfe3000_delay_access(struct pfe3000_data *data)
{
	if (data->access_delay > 0) {
		s64 delta;

		delta = ktime_us_delta(ktime_get(), data->access_time);
		if (delta < data->access_delay)
			udelay(data->access_delay - delta);
	}
}

static int pfe3000_read_word_data(struct i2c_client *client,
				  int page, int reg)
{
	int ret = -ENXIO;
	struct pfe3000_data *data;
	const struct pmbus_driver_info *info;

	info = pmbus_get_driver_info(client);
	data = to_pfe3000_data(info);
	if (data->id != PFE3000_12_069RA || !PFE3000_VALID_PAGE(page))
		return -ENXIO;

	switch (reg) {
	case PMBUS_FAN_COMMAND_1:
	case PMBUS_STATUS_WORD:
	case PMBUS_READ_VIN:
	case PMBUS_READ_IIN:
	case PMBUS_READ_VOUT:
	case PMBUS_READ_IOUT:
	case PMBUS_READ_TEMPERATURE_1:
	case PMBUS_READ_TEMPERATURE_2:
	case PMBUS_READ_TEMPERATURE_3:
	case PMBUS_READ_FAN_SPEED_1:
	case PMBUS_READ_POUT:
	case PMBUS_READ_PIN:
		pfe3000_delay_access(data);
		ret = pmbus_read_word_data(client, page, reg);
		pfe3000_update_access_time(data);
		break;
	}

	return ret;
}

static int pfe3000_read_byte_data(struct i2c_client *client,
				  int page, int reg)
{
	int ret = -ENXIO;
	struct pfe3000_data *data;
	const struct pmbus_driver_info *info;

	info = pmbus_get_driver_info(client);
	data = to_pfe3000_data(info);
	if (data->id != PFE3000_12_069RA || !PFE3000_VALID_PAGE(page))
		return -ENXIO;

	switch (reg) {
	case PMBUS_PAGE:
	case PMBUS_OPERATION:
	case PMBUS_CLEAR_FAULTS:
	case PMBUS_CAPABILITY:
	case PMBUS_VOUT_MODE:
	case PMBUS_FAN_CONFIG_12:
	case PMBUS_STATUS_VOUT:
	case PMBUS_STATUS_IOUT:
	case PMBUS_STATUS_INPUT:
	case PMBUS_STATUS_TEMPERATURE:
	case PMBUS_STATUS_CML:
	case PMBUS_STATUS_OTHER:
	case PMBUS_STATUS_MFR_SPECIFIC:
	case PMBUS_STATUS_FAN_12:
		pfe3000_delay_access(data);
		ret = pmbus_read_byte_data(client, page, reg);
		pfe3000_update_access_time(data);
		break;
	}

	return ret;
}

static int pfe3000_write_word_data(struct i2c_client *client, int page,
				   int reg, u16 word)
{
	int ret = -ENXIO;
	struct pfe3000_data *data;
	const struct pmbus_driver_info *info;

	info = pmbus_get_driver_info(client);
	data = to_pfe3000_data(info);
	if (data->id != PFE3000_12_069RA || !PFE3000_VALID_PAGE(page))
		return -ENXIO;

	if (reg == PMBUS_FAN_COMMAND_1) {
		pfe3000_delay_access(data);
		ret = pmbus_write_word_data(client, page, reg, word);
		pfe3000_update_access_time(data);
	}

	return ret;
}

static int pfe3000_write_byte(struct i2c_client *client,
			      int page, u8 value)
{
	int ret = -ENXIO;
	struct pfe3000_data *data;
	const struct pmbus_driver_info *info;

	info = pmbus_get_driver_info(client);
	data = to_pfe3000_data(info);
	if (data->id != PFE3000_12_069RA || !PFE3000_VALID_PAGE(page))
		return -ENXIO;

	switch (value) {
	case PMBUS_PAGE:
	case PMBUS_OPERATION:
	case PMBUS_CLEAR_FAULTS:
		pfe3000_delay_access(data);
		ret = pmbus_write_byte(client, page, value);
		pfe3000_update_access_time(data);
		break;
	}

	return ret;
}

static int pfe3000_write_byte_data(struct i2c_client *client, int page,
				   int reg, u8 value)
{
	int ret = -ENXIO;
	struct pfe3000_data *data;
	const struct pmbus_driver_info *info;

	info = pmbus_get_driver_info(client);
	data = to_pfe3000_data(info);
	if (data->id != PFE3000_12_069RA || !PFE3000_VALID_PAGE(page))
		return -ENXIO;

	switch (value) {
	case PMBUS_PAGE:
	case PMBUS_OPERATION:
	case PMBUS_CLEAR_FAULTS:
		pfe3000_delay_access(data);
		ret = pmbus_write_byte_data(client, page, reg, value);
		pfe3000_update_access_time(data);
		break;
	}

	return ret;
}

static ssize_t shutdown_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	int ret, page = 0;
	struct pfe3000_data *data;
	const struct pmbus_driver_info *info;
	struct i2c_client *client = to_i2c_client(dev);

	info = pmbus_get_driver_info(client);
	data = to_pfe3000_data(info);

	/* Update shutdown state only if read was successful */
	ret = pfe3000_read_byte_data(client, page, PMBUS_OPERATION);
	if (ret == PFE3000_OPERATION_OFF)
		data->shutdown_state = 1;
	else if (ret > 0)
		data->shutdown_state = 0;

	return sprintf(buf, "%d\n\nSet to 1 for shutdown PFE3000 PSU\n",
		       data->shutdown_state);
}

static int shutdown_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	u8 cmd_args;
	long shutdown;
	int ret, new_state, page = 0;
	struct pfe3000_data *data;
	const struct pmbus_driver_info *info;
	struct i2c_client *client = to_i2c_client(dev);

	info = pmbus_get_driver_info(client);
	data = to_pfe3000_data(info);

	if (buf == NULL || count == 0)
		return -ENXIO;

	if (kstrtol(buf, 10, &shutdown) != 0) {
		/*
		 * Return "count" bytes processed even though kstrtol()
		 * returned failure.
		 */
		return count;
	}

	/* Shutdown PFE3000 only if the user input is exactly "1" */
	if (shutdown == PFE3000_SYSFS_CMD_OFF) {
		cmd_args = PFE3000_OPERATION_OFF;
		new_state = 1;
	} else {
		cmd_args = PFE3000_OPERATION_ON;
		new_state = 0;
	}
	ret = pfe3000_write_byte_data(client, page,
				      PMBUS_OPERATION, cmd_args);
	if (ret == 0)
		data->shutdown_state = new_state;

	/* Always returns the number of processed characters. */
	return count;
}

static DEVICE_ATTR_RW(shutdown);

static struct attribute *shutdown_attrs[] = {
	&dev_attr_shutdown.attr,
	NULL,
};
static struct attribute_group control_attr_group = {
	.name = "control",
	.attrs = shutdown_attrs,
};

static int pfe3000_sysfs_init(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	return sysfs_create_group(&client->dev.kobj, &control_attr_group);
}

static void pfe3000_sysfs_destroy(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &control_attr_group);
}

static int pfe3000_remove(struct i2c_client *client)
{
	pfe3000_sysfs_destroy(client);
	return pmbus_do_remove(client);
}

static int pfe3000_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct pfe3000_data *data;
	struct pmbus_driver_info *info;
	u8 dev_model[I2C_SMBUS_BLOCK_MAX + 1];

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA |
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return -ENODEV;

	udelay(PFE3000_ACCESS_DELAY);
	ret = i2c_smbus_read_block_data(client, PMBUS_MFR_MODEL,
					dev_model);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed to read manufacture model: errno=%d\n",
			-ret);
		if (!ignore_probe)
			return ret;
	} else {
		dev_model[ret] = '\0';
		/*
		 * Some PFE3000 devices return model string with trailing
		 * spaces: let's use strncmp() to just compare the model
		 * string and ignore trailing spaces.
		 */
		if (strncmp(PFE3000_MFR_MODEL, dev_model,
			    strlen(PFE3000_MFR_MODEL)) != 0) {
			dev_err(&client->dev,
				"unexpected manufacture model: [%s]\n",
				dev_model);
			return ret;
		}
		dev_notice(&client->dev, "MFR_MODEL is [%s]\n", dev_model);
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	data->id = PFE3000_12_069RA;
	data->shutdown_state = 0;
	data->access_time = ktime_get();
	data->access_delay = PFE3000_ACCESS_DELAY;
	info = &data->info;
	info->pages = PFE3000_NUM_PAGES;
	info->func[0] = PMBUS_HAVE_VIN |
			PMBUS_HAVE_VOUT |
			PMBUS_HAVE_IIN |
			PMBUS_HAVE_IOUT |
			PMBUS_HAVE_PIN |
			PMBUS_HAVE_POUT |
			PMBUS_HAVE_FAN12 |
			PMBUS_HAVE_TEMP |
			PMBUS_HAVE_TEMP2 |
			PMBUS_HAVE_TEMP3 |
			PMBUS_HAVE_STATUS_VOUT |
			PMBUS_HAVE_STATUS_IOUT |
			PMBUS_HAVE_STATUS_INPUT |
			PMBUS_HAVE_STATUS_TEMP |
			PMBUS_HAVE_STATUS_FAN12;
	info->read_word_data = pfe3000_read_word_data;
	info->read_byte_data = pfe3000_read_byte_data;
	info->write_word_data = pfe3000_write_word_data;
	info->write_byte = pfe3000_write_byte;

	ret = pfe3000_sysfs_init(client, id);
	if (ret != 0) {
		dev_err(&client->dev,
			"failed to initialize sysfs nodes\n");
		return ret;
	}

	udelay(PFE3000_ACCESS_DELAY);
	return pmbus_do_probe(client, id, info);
}

static const struct i2c_device_id pfe3000_ids[] = {
	{"pfe3000", PFE3000_12_069RA},
	{ }
};
MODULE_DEVICE_TABLE(i2c, pfe3000_ids);

static struct i2c_driver pfe3000_driver = {
	.driver = {
		   .name = "pfe3000",
	},
	.probe = pfe3000_probe,
	.remove = pfe3000_remove,
	.id_table = pfe3000_ids,
};

module_i2c_driver(pfe3000_driver);

MODULE_AUTHOR("Tao Ren, based on work by Guenter Roeck and Vineela Kukkadapu");
MODULE_DESCRIPTION("PMBus driver for pfe3000");
MODULE_LICENSE("GPL");
