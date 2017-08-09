/*
 * Hardware monitoring driver for PFE3000 and compatibles
 * Based on the zl6100 driver with the following copyright:
 *
 * Copyright (c) 2011 Ericsson AB.
 * Copyright (c) 2012 Guenter Roeck
 * Copyright 2004-present Facebook. All Rights Reserved.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/i2c/pmbus.h>
#include "pmbus.h"

enum chips { PFE3000_12_069RA };
struct pfe3000_data {
	int id;
	int shutdown_state;
	struct pmbus_driver_info info;
};

#define to_pfe3000_data(x)  container_of(x, struct pfe3000_data, info)

/*
 * Other PowerOne device require a wait time;  this is included in case
 * it is necessary for future debugging.
 */

#define PFE3000_WAIT_TIME       5000	/* uS	*/
#define PFE3000_PAGE0           0
#define PFE3000_OP_REG_ADDR     PMBUS_OPERATION
#define PFE3000_OP_SHUTDOWN_CMD 0x0
#define PFE3000_OP_POWERON_CMD  PB_OPERATION_CONTROL_ON
#define PFE3000_OP_ARG_SHUTDOWN 1

static ushort delay = PFE3000_WAIT_TIME;
module_param(delay, ushort, 0644);
MODULE_PARM_DESC(delay, "Delay between chip accesses in uS");

static const struct i2c_device_id pfe3000_id[] = {
	{"pfe3000dc", PFE3000_12_069RA },
	{"pfe3000", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, pfe3000_id);

static int pfe3000_read_word_data(struct i2c_client *client, int page, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe3000_data *data = to_pfe3000_data(info);
	int ret;

	if (data->id != PFE3000_12_069RA && page > 0)
		return -ENXIO;

	if (reg >= PMBUS_VIRT_BASE)
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
	//	case PMBUS_MFR_LOCATION:
			ret = pmbus_read_word_data(client, page, reg);
			return ret;
		default:
			return -ENXIO;
		}
}

static int pfe3000_read_byte_data(struct i2c_client *client, int page, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe3000_data *data = to_pfe3000_data(info);
	int ret;

	if (data->id != PFE3000_12_069RA && page > 0)
		return -ENXIO;

	switch (reg) {
		case PMBUS_PAGE:
		case PMBUS_OPERATION:
		case PMBUS_CLEAR_FAULTS:
		case PMBUS_CAPABILITY:
		case PMBUS_VOUT_MODE:
		case PMBUS_FAN_CONFIG_12:
		case PMBUS_STATUS_BYTE:
		case PMBUS_STATUS_VOUT:
		case PMBUS_STATUS_IOUT:
		case PMBUS_STATUS_INPUT:
		case PMBUS_STATUS_TEMPERATURE:
		case PMBUS_STATUS_CML:
		case PMBUS_STATUS_OTHER:
		case PMBUS_STATUS_MFR_SPECIFIC:
		case PMBUS_STATUS_FAN_12:
			ret = pmbus_read_byte_data(client, page, reg);
			return ret;
		default:
			return -ENXIO;
	}
}

static int pfe3000_write_word_data(struct i2c_client *client, int page, int reg,
				  u16 word)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe3000_data *data = to_pfe3000_data(info);
	int ret;

	if (data->id != PFE3000_12_069RA && page > 0)
		return -ENXIO;

	if (reg >= PMBUS_VIRT_BASE)
		return -ENXIO;

	if (reg == PMBUS_FAN_COMMAND_1)
		ret = pmbus_write_word_data(client, page, reg, word);
	else
		ret = -ENXIO;

	return ret;
}

static int pfe3000_write_byte(struct i2c_client *client, int page, u8 value)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe3000_data *data = to_pfe3000_data(info);
	int ret;

	if (data->id != PFE3000_12_069RA && page > 0)
		return -ENXIO;

	switch (value) {
		case PMBUS_PAGE:
		case PMBUS_OPERATION:
		case PMBUS_CLEAR_FAULTS:
			ret = pmbus_write_byte(client, page, value);
			return ret;
		default:
			return -ENXIO;
	}
}

static int pfe3000_write_byte_data(struct i2c_client *client, int page,
																		int reg, u8 value)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe3000_data *data = to_pfe3000_data(info);
	int ret;

	if (data->id != PFE3000_12_069RA && page > 0)
		return -ENXIO;

	switch (value) {
		case PMBUS_PAGE:
		case PMBUS_OPERATION:
		case PMBUS_CLEAR_FAULTS:
			ret = pmbus_write_byte_data(client, page, reg, value);
			return ret;
		default:
			return -ENXIO;
	}
}


static struct pmbus_platform_data platform_data = {
	.flags = PMBUS_SKIP_STATUS_CHECK
};

static ssize_t pfe3000_shutdown_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe3000_data *data = to_pfe3000_data(info);
	int len = 0;
	u8 read_val = 0;

	// Update shutdown state before printing out
	read_val = pfe3000_read_byte_data(client, PFE3000_PAGE0,
														        PFE3000_OP_REG_ADDR);
	// Only if the read was successful, update the status
	if (read_val >= 0)
	{
		if (read_val == PFE3000_OP_SHUTDOWN_CMD)
			data->shutdown_state = 1;
		else
			data->shutdown_state = 0;

	}

  len = sprintf(buf, "%d\n\nSet to 1 for shutdown PFE3000 PSU\n",
		              data->shutdown_state);
  return len;
}

static int pfe3000_shutdown_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct pfe3000_data *data = to_pfe3000_data(info);

	u8 write_value = 0;
	long shutdown = 0;
	int rc = 0;

	if (buf == NULL) {
		return -ENXIO;
	}

	rc = kstrtol(buf, 10, &shutdown);
	if (rc != 0)	{
		// Parsing was not successful. But will return as "count" bytes processed
		// without doing anything, in order to give the control back to
		// the caller / shell
		return count;
	}

	// We will shutdown PFE3000 only if the user input is exactly "1"
	if (shutdown == (long)PFE3000_OP_ARG_SHUTDOWN) {
		write_value = PFE3000_OP_SHUTDOWN_CMD;
	} else {
		write_value = PFE3000_OP_POWERON_CMD;
	}

	rc = pfe3000_write_byte_data(client, PFE3000_PAGE0,
																 PFE3000_OP_REG_ADDR, write_value);

  // Write successful. Update driver state
	if (rc == 0) {
		if (write_value == PFE3000_OP_SHUTDOWN_CMD)
			data->shutdown_state = 1;
		else
			data->shutdown_state = 0;
	}

	// No matter successful or failure, we processed all input characters
	// So, return the number of input chars processed to finish the sysfs
	// access.
	return count;
}

static DEVICE_ATTR(shutdown, S_IRUGO | S_IWUSR,
									 pfe3000_shutdown_show,
                   pfe3000_shutdown_store);

static struct attribute *shutdown_attrs[] = {
									     &dev_attr_shutdown.attr,
									     NULL
									 };
static struct attribute_group control_attr_group = {
									     .name = "control",
									     .attrs = shutdown_attrs,
									 };

static int pfe3000_register_shutdown(struct i2c_client *client,
			const struct i2c_device_id *id)
{
  return sysfs_create_group(&client->dev.kobj,
												 		&control_attr_group);
}

static void pfe3000_remove_shutdown(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj,
										 &control_attr_group);
	return;
}

static int pfe3000_remove(struct i2c_client *client)
{
	// First, remove sysfs stub for shutdown control
	pfe3000_remove_shutdown(client);
	// Finally, remove what pmbus_core has added
	return pmbus_do_remove(client);
}

static int pfe3000_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	int pmbus_ret;
	int addon_ret;
	int kind;
	struct device *dev = &client->dev;
	struct pfe3000_data *data;
	struct pmbus_driver_info *info;
	u8 device_id[I2C_SMBUS_BLOCK_MAX + 1];
	// This is a hack specific for CMM, i2c_smbus_read_block_data
	// causes a seg fault and thus reading MFR data is not possible.
	// MFR data is needed to determine the number pages which is 7
	// as per spec.
	strcpy(device_id, "PFE3000-12-069RA");
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA
				     | I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return -ENODEV;

	/* ret = i2c_smbus_read_block_data(client, PMBUS_MFR_MODEL, device_id);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read Manufacturer ID\n");
		kind = PFE3000_12_069RA;
	} else {
		device_id[ret] = 0;

		if (strncmp(device_id, "PFE3000-12-069RA", ret) != 0 )
			dev_notice(&client->dev, "Unexpected MFR_ID : [%s]\n", device_id);

    // Default to this kind
		kind = PFE3000_12_069RA;

		dev_notice(&client->dev, "MFR_ID is [%s]\n", device_id);
	}*/
	dev_notice(&client->dev, "MFR_ID is [%s]\n", device_id);

	data = devm_kzalloc(&client->dev, sizeof(struct pfe3000_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->id = kind;
	data->shutdown_state = 0;

	info = &data->info;
	info->pages = 2;
	info->delay = delay;

	/*
	 * Skip extra status checks;  this is required to read the
	 * VOUT_MODE register to determine the exponent to apply
	 * to the VOUT values.
	 */
	//dev->platform_data = &platform_data;

	/* Make sure that the page isn't pointing elsewhere! */

//	i2c_smbus_write_byte_data(client, PMBUS_PAGE, 0);

	/*
	 * It seems reasonable to just scan the device for supported
	 * values, but most drivers seem to jam these values in
	 * there, so that's what we'll do.
	 */

  // info->func[num] : num reflects the page number
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
									PMBUS_HAVE_STATUS_FAN12 ;
									//	PMBUS_HAVE_MFRDATA; Look at the comment above about MFR data
	info->func[1] = PMBUS_HAVE_VIN |
	 								PMBUS_HAVE_VOUT |
									PMBUS_HAVE_IIN |
									PMBUS_HAVE_IOUT |
									PMBUS_HAVE_PIN |
									PMBUS_HAVE_POUT |
									PMBUS_HAVE_STATUS_VOUT |
									PMBUS_HAVE_STATUS_IOUT |
									PMBUS_HAVE_STATUS_TEMP |
									PMBUS_HAVE_STATUS_FAN12 |
									PMBUS_HAVE_STATUS_INPUT;
//	info->func[2] = PMBUS_HAVE_VOUT;
//	info->func[4] = PMBUS_HAVE_VOUT;
//	info->func[5] = PMBUS_HAVE_VOUT;
//	info->func[6] = PMBUS_HAVE_VOUT;

	info->read_word_data = pfe3000_read_word_data;
	info->read_byte_data = pfe3000_read_byte_data;
	info->write_word_data = pfe3000_write_word_data;
	info->write_byte = pfe3000_write_byte;

	// Register the device through pmbus core routine
	pmbus_ret = pmbus_do_probe(client, id, info);
	// On top of this, install sysfs for shutdown control
	addon_ret = pfe3000_register_shutdown(client, id);

	if ((pmbus_ret == 0) && (addon_ret == 0)) {
		ret = 0;
	} else {
		// Something went wrong.
		if (pmbus_ret == 0)
		{
			// Rollback pmbus dev register before bail out
			pmbus_do_remove(client);
			// Return the errorcode of shutdown addon installation
			ret = addon_ret;
		} else {
			// Return the errorcode of pmbus device registration
			ret = pmbus_ret;
		}
	}

	return ret;
}

static struct i2c_driver pfe3000_driver = {
	.driver = {
		   .name = "pfe3000",
		   },
	.probe = pfe3000_probe,
	.remove = pfe3000_remove,
	.id_table = pfe3000_id,
};

module_i2c_driver(pfe3000_driver);

MODULE_AUTHOR("Vineela Kukkadapu, based on work by Guenter Roeck");
MODULE_DESCRIPTION("PMBus driver for pfe3000");
MODULE_LICENSE("GPL");
