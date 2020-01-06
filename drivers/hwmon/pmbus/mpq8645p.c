/*
 * Copyright 2015-present Facebook. All Rights Reserved.
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
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/pmbus.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include "pmbus.h"

#define MAX_DATA_SIZE                    8

#define MPQ8645P_REG_RESTORE_USER_ALL    0x16
#define MPQ8645P_REG_MFR_CTRL_COMP       0xD0
#define MPQ8645P_REG_MFR_CTRL_VOUT       0xD1
#define MPQ8645P_REG_MFR_CTRL_OPS        0xD2
#define MPQ8645P_REG_MFR_VOUT_OVP_FALUT_LIMIT 0xD4
#define MPQ8645P_REG_MFR_OVP_NOCP_SET    0xD5
#define MPQ8645P_REG_MFR_OT_OC_SET       0xD6
#define MPQ8645P_REG_MFR_OC_PHASE_LIMIT  0xD7
#define MPQ8645P_REG_MFR_HICCUP_ITV_SET  0xD8
#define MPQ8645P_REG_MFR_PGOOD_ON_OFF    0xD9
#define MPQ8645P_REG_MFR_VOUT_STEP       0xDA
#define MPQ8645P_REG_MFR_LOW_POWER       0xE5
#define MPQ8645P_REG_MFR_CTRL            0xEA

enum {
	MPQ8645P_REVISION = 0,
	MPQ8645P_RESTORE_USER_ALL,
	MPQ8645P_VOUT_COMMAND,
	MPQ8645P_IOUT_OC_FAULT_LIMIT,
	MPQ8645P_IOUT_OC_WARN_LIMIT,
	MPQ8645P_MFR_REVISION,
	MPQ8645P_MFR_CTRL_COMP,
	MPQ8645P_MFR_CTRL_VOUT,
	MPQ8645P_MFR_CTRL_OPS,
	MPQ8645P_MFR_OC_PHASE_LIMIT,
	MPQ8645P_NUM_ENTRIES
};

struct mpq8645p_pdata {
	struct i2c_client *client;
	struct mutex lock;
	u8 index[MPQ8645P_NUM_ENTRIES];
};

#define get_struct(x, y) container_of((x), struct mpq8645p_pdata, index[(y)])

static struct pmbus_driver_info mpq8645p_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_VOLTAGE_IN] = 4000,
	.b[PSC_VOLTAGE_IN] = 0,
	.R[PSC_VOLTAGE_IN] = -2,
	.m[PSC_VOLTAGE_OUT] = 8000,
	.b[PSC_VOLTAGE_OUT] = 0,
	.R[PSC_VOLTAGE_OUT] = -1,
	.m[PSC_CURRENT_OUT] = 16000,
	.b[PSC_CURRENT_OUT] = 0,
	.R[PSC_CURRENT_OUT] = -3,
	.m[PSC_TEMPERATURE] = 1,
	.b[PSC_TEMPERATURE] = 0,
	.R[PSC_TEMPERATURE] = 0,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT
		| PMBUS_HAVE_IOUT | PMBUS_HAVE_TEMP
		| PMBUS_HAVE_STATUS_VOUT | PMBUS_HAVE_STATUS_IOUT
		| PMBUS_HAVE_STATUS_INPUT | PMBUS_HAVE_STATUS_TEMP
};

static int mpq8645p_restore_user(struct i2c_client *client)
{
	int ret, i;
	struct mtp_cmd {
		uint8_t reg; uint16_t value; int delay;
	} cmds[6] = {
		{0xE6, 0x287C, 1},
		{0xE7, 0x0001, 2},
		{0xE7, 0x2001, 167},
		{0xE7, 0x1001, 1},
		{0xE7, 0x4001, 300},
		{0xE7, 0x0000, 10}
	};

	// Write to MTP
	for (i = 0; i < 6; i++) {
		if ((ret = i2c_smbus_write_word_data(client, cmds[i].reg, cmds[i].value)) < 0) {
			goto exit;
		}
		msleep(cmds[i].delay);
	}

	// MTP->RAM
	ret = i2c_smbus_write_byte(client, MPQ8645P_REG_RESTORE_USER_ALL);
	msleep(2);
exit:
	return ret;
}

static ssize_t mpq8645p_write_reg(struct i2c_client *client, u8 index, u16 val)
{
	switch (index) {
	case MPQ8645P_RESTORE_USER_ALL:
		return mpq8645p_restore_user(client);
	case MPQ8645P_MFR_CTRL_COMP:
		return i2c_smbus_write_byte_data(client, MPQ8645P_REG_MFR_CTRL_COMP, (u8)val);
	case MPQ8645P_MFR_CTRL_VOUT:
		return i2c_smbus_write_byte_data(client, MPQ8645P_REG_MFR_CTRL_VOUT, (u8)val);
	case MPQ8645P_MFR_CTRL_OPS:
		return i2c_smbus_write_byte_data(client, MPQ8645P_REG_MFR_CTRL_OPS, (u8)val);
	case MPQ8645P_MFR_OC_PHASE_LIMIT:
		return i2c_smbus_write_byte_data(client,
						 MPQ8645P_REG_MFR_OC_PHASE_LIMIT, (u8)val);
	default:
		return -EINVAL;
	}
}

static ssize_t mpq8645p_read_reg(struct i2c_client *client, u8 index)
{
	switch (index) {
	case MPQ8645P_VOUT_COMMAND:
		return i2c_smbus_read_word_data(client, PMBUS_VOUT_COMMAND);
	case MPQ8645P_IOUT_OC_FAULT_LIMIT:
		return i2c_smbus_read_word_data(client, PMBUS_IOUT_OC_FAULT_LIMIT);
	case MPQ8645P_IOUT_OC_WARN_LIMIT:
		return i2c_smbus_read_word_data(client, PMBUS_IOUT_OC_WARN_LIMIT);
	case MPQ8645P_MFR_CTRL_COMP:
		return i2c_smbus_read_byte_data(client, MPQ8645P_REG_MFR_CTRL_COMP);
	case MPQ8645P_MFR_CTRL_VOUT:
		return i2c_smbus_read_byte_data(client, MPQ8645P_REG_MFR_CTRL_VOUT);
	case MPQ8645P_MFR_CTRL_OPS:
		return i2c_smbus_read_byte_data(client, MPQ8645P_REG_MFR_CTRL_OPS);
	case MPQ8645P_MFR_OC_PHASE_LIMIT:
		return i2c_smbus_read_byte_data(client, MPQ8645P_REG_MFR_OC_PHASE_LIMIT);
	default:
		return -EINVAL;
	}
}

static ssize_t mpq8645p_debugfs_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	u8 *index_ptr = file->private_data;
	u8 index = *index_ptr;
	struct mpq8645p_pdata *pdata = get_struct(index_ptr, index);
	struct i2c_client *client = pdata->client;
	char data[MAX_DATA_SIZE] = {0};
	int ret;
	char end;
	u16 value;

	if (copy_from_user(data, buf, sizeof(data)))
		return -EFAULT;

	ret = sscanf(data, "%hi%c", &value, &end);
	if (ret < 1) {
		dev_err(&client->dev, "Can't parse value\n");
		return -EINVAL;
	}
	if (ret > 1  && end != '\n') {
		dev_err(&client->dev, "Detect extra parameters\n");
		return -EINVAL;
	}

	mutex_lock(&pdata->lock);
	ret = mpq8645p_write_reg(client, index, value);
	mutex_unlock(&pdata->lock);

	return ret < 0? -EFAULT: count;
}


static ssize_t mpq8645p_debugfs_read(struct file *file, char __user *buf,
				    size_t count, loff_t *ppos)
{
	u8 *index_ptr = file->private_data;
	u8 index = *index_ptr;
	struct mpq8645p_pdata *pdata = get_struct(index_ptr, index);
	struct i2c_client *client = pdata->client;
	char data[I2C_SMBUS_BLOCK_MAX] = {0};
	int rc;

	mutex_lock(&pdata->lock);
	switch (index) {
	case MPQ8645P_VOUT_COMMAND:
	case MPQ8645P_IOUT_OC_FAULT_LIMIT:
	case MPQ8645P_IOUT_OC_WARN_LIMIT:
		rc =  mpq8645p_read_reg(client, index);
		if (rc >= 0)
			rc = snprintf(data, 5, "%04X", rc);
		break;
	case MPQ8645P_MFR_CTRL_COMP:
	case MPQ8645P_MFR_CTRL_VOUT:
	case MPQ8645P_MFR_CTRL_OPS:
	case MPQ8645P_MFR_OC_PHASE_LIMIT:
		rc =  mpq8645p_read_reg(client, index);
		if (rc >= 0)
			rc = snprintf(data, 3, "%02X", rc);
		break;
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&pdata->lock);

	if (rc < 0)
		return rc;

	data[rc] = '\n';
	rc += 2;

	return simple_read_from_buffer(buf, count, ppos, data, rc);
}

static const struct file_operations mpq8645p_reg_ops = {
	.llseek = noop_llseek,
	.write = mpq8645p_debugfs_write,
	.read = mpq8645p_debugfs_read,
	.open = simple_open,
};

static ssize_t mpq8645p_debugfs_write_mfr_ver(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	u8 *index_ptr = file->private_data;
	u8 index = *index_ptr;
	struct mpq8645p_pdata *pdata = get_struct(index_ptr, index);
	struct i2c_client *client = pdata->client;
	char data[MAX_DATA_SIZE] = {0};
	int ret;
	u8 ver;

	if (copy_from_user(data, buf, sizeof(data)))
		return -EFAULT;

	if (sscanf(data, "%hhi", &ver) < 1) {
		dev_err(&client->dev, "Can't parse version\n");
		return -EINVAL;
	}

	mutex_lock(&pdata->lock);
	if ((ret = mpq8645p_restore_user(client)) < 0)
		goto exit;
	if ((ret = i2c_smbus_write_word_data(client, 0xE7, 0x0001)) < 0)
		goto exit;
	if ((ret = i2c_smbus_write_block_data(client, PMBUS_MFR_REVISION, 1, &ver)) < 0)
		goto exit;
	if ((ret = i2c_smbus_write_word_data(client, 0xE7, 0x0000)) < 0)
		goto exit;
exit:
	mutex_unlock(&pdata->lock);
	return ret < 0? -EFAULT: count;
}

static ssize_t mpq8645p_debugfs_read_mfr_ver(struct file *file, char __user *buf,
				    size_t count, loff_t *ppos)
{
	u8 *index_ptr = file->private_data;
	u8 index = *index_ptr;
	struct mpq8645p_pdata *pdata = get_struct(index_ptr, index);
	struct i2c_client *client = pdata->client;
	u8 buffer[I2C_SMBUS_BLOCK_MAX];
	char str[(I2C_SMBUS_BLOCK_MAX * 2) + 2];
	char *res;
	int rc;

	mutex_lock(&pdata->lock);
	rc = i2c_smbus_read_block_data(client, PMBUS_MFR_REVISION, buffer);
	mutex_unlock(&pdata->lock);

	if (rc < 0)
		return rc;

	res = bin2hex(str, buffer, min(rc, I2C_SMBUS_BLOCK_MAX));
	*res++ = '\n';
	*res = 0;

	return simple_read_from_buffer(buf, count, ppos, str, res - str);
}

static const struct file_operations mpq8645p_ver_ops = {
	.llseek = noop_llseek,
	.write = mpq8645p_debugfs_write_mfr_ver,
	.read = mpq8645p_debugfs_read_mfr_ver,
	.open = simple_open,
};

static int mpq8645p_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int i, ret;
	struct dentry *debugfs;
	struct dentry *mpq8645p_dir;
	struct mpq8645p_pdata *pdata;

	ret = pmbus_do_probe(client, id, &mpq8645p_info);
	if (ret)
		return ret;

	debugfs = pmbus_get_debugfs_dir(client);
	if (!debugfs)
		return -ENOENT;

	mpq8645p_dir = debugfs_create_dir(client->name, debugfs);
	if (!mpq8645p_dir)
		return -ENOENT;

	pdata = devm_kzalloc(&client->dev, sizeof(struct mpq8645p_pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->client = client;
	for (i = 0; i < MPQ8645P_NUM_ENTRIES; ++i) {
		pdata->index[i] = i;
	}

	debugfs_create_file("fw_version", 0444, mpq8645p_dir,
			    &pdata->index[MPQ8645P_REVISION], &mpq8645p_ver_ops);
	debugfs_create_file("restore", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_RESTORE_USER_ALL], &mpq8645p_reg_ops);
	debugfs_create_file("vout_command", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_VOUT_COMMAND], &mpq8645p_reg_ops);
	debugfs_create_file("oc_fault_limit", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_IOUT_OC_FAULT_LIMIT], &mpq8645p_reg_ops);
	debugfs_create_file("oc_warn_limit", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_IOUT_OC_WARN_LIMIT], &mpq8645p_reg_ops);
	debugfs_create_file("ctrl_comp", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_MFR_CTRL_COMP], &mpq8645p_reg_ops);
	debugfs_create_file("ctrl_vout", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_MFR_CTRL_VOUT], &mpq8645p_reg_ops);
	debugfs_create_file("ctrl_ops", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_MFR_CTRL_OPS], &mpq8645p_reg_ops);
	debugfs_create_file("oc_phase_limit", 0644, mpq8645p_dir,
			    &pdata->index[MPQ8645P_MFR_OC_PHASE_LIMIT], &mpq8645p_reg_ops);

	return 0;
}

static const struct of_device_id mpq8645p_of_match[] = {
	{ .compatible = "mps,mpq8645p" },
	{}
};

static const struct i2c_device_id mpq8645p_id[] = {
	{"mpq8645p", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mpq8645p_id);

/* This is the driver that will be inserted */
static struct i2c_driver mpq8645p_driver = {
	.driver = {
		.name = "mpq8645p",
		.of_match_table = mpq8645p_of_match,
	},
	.probe = mpq8645p_probe,
	.remove = pmbus_do_remove,
	.id_table = mpq8645p_id,
};

module_i2c_driver(mpq8645p_driver);

MODULE_AUTHOR("Howard Chiu <Howard.chiu@quantatw.com>");
MODULE_DESCRIPTION("PMBus driver for MPQ8645P");
MODULE_LICENSE("GPL");
