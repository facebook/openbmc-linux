/*
 * Copyright (C) 2011 Infineon Technologies
 *
 * Authors:
 * Peter Huewe <huewe.external@infineon.com>
 *
 * Device driver for TCG/TCPA TPM (trusted platform module).
 * Specifications at www.trustedcomputinggroup.org
 *
 * This device driver implements the TPM interface as defined in
 * the TCG TPM Interface Spec version 1.2, revision 1.0 and the
 * Infineon I2C Protocol Stack Specification v0.20.
 *
 * It is based on the original tpm_tis device driver from Leendert van
 * Dorn and Kyleen Hall.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2 of the
 * License.
 *
 * NOTE:
 * Suspend does currently not work Nvidias Tegra2 Platform
 * but works fine on Beagleboard (ARM OMAP).
 *
 */
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/wait.h>
#include "tpm.h"

/* max. buffer size supported by our tpm */
#define TPM_BUFSIZE 1260

/* max. number of iterations after i2c NAK */
#define MAX_COUNT 3

#define SLEEP_DURATION_LOW 55
#define SLEEP_DURATION_HI 65

/* max. number of iterations after i2c NAK for 'long' commands
 * we need this especially for sending TPM_READY, since the cleanup after the
 * transtion to the ready state may take some time, but it is unpredictable
 * how long it will take.
 */
#define MAX_COUNT_LONG 50

#define SLEEP_DURATION_LONG_LOW 200
#define SLEEP_DURATION_LONG_HI 220

/* expected value for DIDVID register */
#define TPM_TIS_I2C_DID_VID 0x000b15d1L
#define TPM_TIS_I2C_DID_VID_9635 0xd1150b00L
#define TPM_TIS_I2C_DID_VID_9645 0x001a15d1L

/* Structure to store I2C TPM specific stuff */
struct tpm_inf_dev {
	struct i2c_client *client;
	u8 buf[TPM_BUFSIZE+sizeof(u8)]; /* max. buffer size + addr */
	struct tpm_chip *chip;
};

static struct tpm_inf_dev tpm_dev;


/*
 * iic_tpm_read() - read from TPM register
 * @addr: register address to read from
 * @buffer: provided by caller
 * @len: number of bytes to read
 *
 * Read len bytes from TPM register and put them into
 * buffer (little-endian format, i.e. first byte is put into buffer[0]).
 *
 * NOTE: TPM is big-endian for multi-byte values. Multi-byte
 * values have to be swapped.
 *
 * Return -EIO on error, 0 on success.
 */
static int iic_tpm_read(u8 addr, u8 *buffer, size_t len)
{

	struct i2c_msg msg1 = { tpm_dev.client->addr, 0, 1, &addr };
	struct i2c_msg msg2 = { tpm_dev.client->addr, I2C_M_RD, len, buffer };

	int rc;
	int count;

	for (count = 0; count < MAX_COUNT; count++) {
		rc = i2c_transfer(tpm_dev.client->adapter, &msg1, 1);
		if (rc > 0)
			break; /* break here to skip sleep */

		//usleep_range(SLEEP_DURATION_LOW, SLEEP_DURATION_HI);
		udelay(SLEEP_DURATION_LOW);
	}

	if (rc <= 0)
		return -EIO;

	/* After the TPM has successfully received the register address it needs
	 * some time, thus we're sleeping here again, before retrieving the data
	 */
	for (count = 0; count < MAX_COUNT; count++) {
		//usleep_range(SLEEP_DURATION_LOW, SLEEP_DURATION_HI);
                udelay(SLEEP_DURATION_LOW);
		rc = i2c_transfer(tpm_dev.client->adapter, &msg2, 1);
		if (rc > 0)
			break;

	}

	if (rc <= 0)
		return -EIO;

	return 0;
}

static int iic_tpm_write_generic(u8 addr, u8 *buffer, size_t len,
				unsigned int sleep_low,
				unsigned int sleep_hi,
				u8 max_count)
{
	int rc = -1;
	int count;

	struct i2c_msg msg1 = { tpm_dev.client->addr, 0, len + 1, tpm_dev.buf };

	tpm_dev.buf[0] = addr;
	memcpy(&(tpm_dev.buf[1]), buffer, len);

	for (count = 0; count < max_count; count++) {
		rc = i2c_transfer(tpm_dev.client->adapter, &msg1, 1);
		if (rc > 0)
			break;

		//usleep_range(sleep_low, sleep_hi);
		udelay(sleep_low);
	}

	if (rc <= 0)
		return -EIO;

	return 0;
}

/*
 * iic_tpm_write() - write to TPM register
 * @addr: register address to write to
 * @buffer: containing data to be written
 * @len: number of bytes to write
 *
 * Write len bytes from provided buffer to TPM register (little
 * endian format, i.e. buffer[0] is written as first byte).
 *
 * NOTE: TPM is big-endian for multi-byte values. Multi-byte
 * values have to be swapped.
 *
 * NOTE: use this function instead of the iic_tpm_write_generic function.
 *
 * Return -EIO on error, 0 on success
 */
static int iic_tpm_write(u8 addr, u8 *buffer, size_t len)
{
	return iic_tpm_write_generic(addr, buffer, len, SLEEP_DURATION_LOW,
			SLEEP_DURATION_HI, MAX_COUNT);
}

/*
 * This function is needed especially for the cleanup situation after
 * sending TPM_READY
 * */
static int iic_tpm_write_long(u8 addr, u8 *buffer, size_t len)
{
	return iic_tpm_write_generic(addr, buffer, len, SLEEP_DURATION_LONG_LOW,
			SLEEP_DURATION_LONG_HI, MAX_COUNT_LONG);
}

#define TPM_HEADER_SIZE 10

enum tis_access {
	TPM_ACCESS_VALID = 0x80,
	TPM_ACCESS_ACTIVE_LOCALITY = 0x20,
	TPM_ACCESS_REQUEST_PENDING = 0x04,
	TPM_ACCESS_REQUEST_USE = 0x02,
};

enum tis_status {
	TPM_STS_VALID = 0x80,
	TPM_STS_COMMAND_READY = 0x40,
	TPM_STS_GO = 0x20,
	TPM_STS_DATA_AVAIL = 0x10,
	TPM_STS_DATA_EXPECT = 0x08,
};

enum tis_defaults {
	TIS_SHORT_TIMEOUT = 750,	/* ms */
	TIS_LONG_TIMEOUT = 2000,	/* 2 sec */
};

#define	TPM_ACCESS(l)			(0x0000 | ((l) << 4))
#define	TPM_STS(l)			(0x0001 | ((l) << 4))
#define	TPM_DATA_FIFO(l)		(0x0005 | ((l) << 4))
#define	TPM_DID_VID(l)			(0x0006 | ((l) << 4))

static int check_locality(struct tpm_chip *chip, int loc)
{
	u8 buf;
	int rc;

	rc = iic_tpm_read(TPM_ACCESS(loc), &buf, 1);
	if (rc < 0)
		return rc;

	if ((buf & (TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID)) ==
		(TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID)) {
		chip->vendor.locality = loc;
		return loc;
	}

	return -1;
}

static void release_locality(struct tpm_chip *chip, int loc, int force)
{
	u8 buf;
	if (iic_tpm_read(TPM_ACCESS(loc), &buf, 1) < 0)
		return;

	if (force || (buf & (TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID)) ==
			(TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID)) {
		buf = TPM_ACCESS_ACTIVE_LOCALITY;
		iic_tpm_write(TPM_ACCESS(loc), &buf, 1);
	}
}

static int request_locality(struct tpm_chip *chip, int loc)
{
	unsigned long stop;
	u8 buf = TPM_ACCESS_REQUEST_USE;

	if (check_locality(chip, loc) >= 0)
		return loc;

	iic_tpm_write(TPM_ACCESS(loc), &buf, 1);

	/* wait for burstcount */
	stop = jiffies + chip->vendor.timeout_a;
	do {
		if (check_locality(chip, loc) >= 0)
			return loc;
		msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));

	return -1;
}

static u8 tpm_tis_i2c_status(struct tpm_chip *chip)
{
	/* NOTE: since i2c read may fail, return 0 in this case --> time-out */
	u8 buf;
	if (iic_tpm_read(TPM_STS(chip->vendor.locality), &buf, 1) < 0)
		return 0;
	else
		return buf;
}

static void tpm_tis_i2c_ready(struct tpm_chip *chip)
{
	/* this causes the current command to be aborted */
	u8 buf = TPM_STS_COMMAND_READY;
	iic_tpm_write_long(TPM_STS(chip->vendor.locality), &buf, 1);
}

static ssize_t get_burstcount(struct tpm_chip *chip)
{
	unsigned long stop;
	ssize_t burstcnt;
	u8 buf[3];

	/* wait for burstcount */
	/* which timeout value, spec has 2 answers (c & d) */
	stop = jiffies + chip->vendor.timeout_d;
	do {
		/* Note: STS is little endian */
		if (iic_tpm_read(TPM_STS(chip->vendor.locality) + 1, buf, 3) < 0)
			burstcnt = 0;
		else
			burstcnt = (buf[2] << 16) + (buf[1] << 8) + buf[0];

		if (burstcnt)
			return burstcnt;

		msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));
	return -EBUSY;
}

static int wait_for_stat(struct tpm_chip *chip, u8 mask, unsigned long timeout,
			int *status)
{
	unsigned long stop;

	/* check current status */
	*status = tpm_tis_i2c_status(chip);
	if ((*status & mask) == mask)
		return 0;

	stop = jiffies + timeout;
	do {
		msleep(TPM_TIMEOUT);
		*status = tpm_tis_i2c_status(chip);
		if ((*status & mask) == mask)
			return 0;

	} while (time_before(jiffies, stop));

	return -ETIME;
}

static int recv_data(struct tpm_chip *chip, u8 *buf, size_t count)
{
	size_t size = 0;
	ssize_t burstcnt;
	int rc;

	while (size < count) {
		burstcnt = get_burstcount(chip);

		/* burstcnt < 0 = tpm is busy */
		if (burstcnt < 0)
			return burstcnt;

		/* limit received data to max. left */
		if (burstcnt > (count-size))
			burstcnt = count-size;

		rc = iic_tpm_read(TPM_DATA_FIFO(chip->vendor.locality),
				  &(buf[size]),
				  burstcnt);
		if (rc == 0)
			size += burstcnt;

	}
	return size;
}

static int tpm_tis_i2c_recv(struct tpm_chip *chip, u8 *buf, size_t count)
{
	int size = 0;
	int expected, status;

	if (count < TPM_HEADER_SIZE) {
		size = -EIO;
		goto out;
	}

	/* read first 10 bytes, including tag, paramsize, and result */
	size = recv_data(chip, buf, TPM_HEADER_SIZE);
	if (size < TPM_HEADER_SIZE) {
		dev_err(chip->dev, "Unable to read header\n");
		goto out;
	}

	expected = be32_to_cpu(*(__be32 *) (buf + 2));
	if ((size_t)expected > count) {
		size = -EIO;
		goto out;
	}

	size += recv_data(chip, &buf[TPM_HEADER_SIZE],
				expected - TPM_HEADER_SIZE);
	if (size < expected) {
		dev_err(chip->dev, "Unable to read remainder of result\n");
		size = -ETIME;
		goto out;
	}

	wait_for_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c, &status);
	if (status & TPM_STS_DATA_AVAIL) {	/* retry? */
		dev_err(chip->dev, "Error left over data\n");
		size = -EIO;
		goto out;
	}

out:
	tpm_tis_i2c_ready(chip);
	/* The TPM needs some time to clean up here,
	 * so we sleep rather than keeping the bus busy
	 */
	//usleep_range(2400, 2600);
	mdelay(2);
	release_locality(chip, chip->vendor.locality, 0);
	return size;
}

static int tpm_tis_i2c_send(struct tpm_chip *chip, u8 *buf, size_t len)
{
	int rc, status;
	ssize_t burstcnt;
	size_t count = 0;
	u8 sts = TPM_STS_GO;

	if (len > TPM_BUFSIZE)
		return -E2BIG; /* command is too long for our tpm, sorry */

	if (request_locality(chip, 0) < 0)
		return -EBUSY;

	status = tpm_tis_i2c_status(chip);
	if ((status & TPM_STS_COMMAND_READY) == 0) {
		tpm_tis_i2c_ready(chip);
		if (wait_for_stat
		    (chip, TPM_STS_COMMAND_READY,
		     chip->vendor.timeout_b, &status) < 0) {
			rc = -ETIME;
			goto out_err;
		}
	}

	while (count < len - 1) {
		burstcnt = get_burstcount(chip);
		dev_dbg(chip->dev,
			"send(): count=%zd, len=%zd, burstcount=%zd (plain)\n",
			count, len, burstcnt);

		/* burstcnt < 0 = tpm is busy */
		if (burstcnt < 0)
			return burstcnt;

		if (burstcnt > (len-1-count))
			burstcnt = len-1-count;

		dev_dbg(chip->dev, "send(): burstcount=%zd\n", burstcnt);

		rc = iic_tpm_write(TPM_DATA_FIFO(chip->vendor.locality),
				   &(buf[count]), burstcnt);
		if (rc == 0)
			count += burstcnt;

		wait_for_stat(chip, TPM_STS_VALID,
				chip->vendor.timeout_c, &status);

		if ((status & TPM_STS_DATA_EXPECT) == 0) {
			rc = -EIO;
			goto out_err;
		}

	}

	dev_dbg(chip->dev, "send(): last byte @ count=%zd\n", count);

	/* write last byte */
	iic_tpm_write(TPM_DATA_FIFO(chip->vendor.locality), &(buf[count]), 1);
	wait_for_stat(chip, TPM_STS_VALID, chip->vendor.timeout_c, &status);
	if ((status & TPM_STS_DATA_EXPECT) != 0) {
		rc = -EIO;
		goto out_err;
	}

	/* go and do it */
	iic_tpm_write(TPM_STS(chip->vendor.locality), &sts, 1);

	return len;
out_err:
	tpm_tis_i2c_ready(chip);
	/* The TPM needs some time to clean up here,
	 * so we sleep rather than keeping the bus busy
	 */
	//usleep_range(2400, 2600);
	mdelay(2);
	release_locality(chip, chip->vendor.locality, 0);
	return rc;
}

static const struct file_operations tis_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = tpm_open,
	.read = tpm_read,
	.write = tpm_write,
	.release = tpm_release,
};

static DEVICE_ATTR(pubek, S_IRUGO, tpm_show_pubek, NULL);
static DEVICE_ATTR(pcrs, S_IRUGO, tpm_show_pcrs, NULL);
static DEVICE_ATTR(enabled, S_IRUGO, tpm_show_enabled, NULL);
static DEVICE_ATTR(active, S_IRUGO, tpm_show_active, NULL);
static DEVICE_ATTR(owned, S_IRUGO, tpm_show_owned, NULL);
static DEVICE_ATTR(temp_deactivated, S_IRUGO, tpm_show_temp_deactivated, NULL);
static DEVICE_ATTR(caps, S_IRUGO, tpm_show_caps_1_2, NULL);
static DEVICE_ATTR(cancel, S_IWUSR | S_IWGRP, NULL, tpm_store_cancel);

static struct attribute *tis_attrs[] = {
	&dev_attr_pubek.attr,
	&dev_attr_pcrs.attr,
	&dev_attr_enabled.attr,
	&dev_attr_active.attr,
	&dev_attr_owned.attr,
	&dev_attr_temp_deactivated.attr,
	&dev_attr_caps.attr,
	&dev_attr_cancel.attr, NULL,
};

static struct attribute_group tis_attr_grp = {
	.attrs = tis_attrs
};

static struct tpm_vendor_specific tpm_tis_i2c = {
	.status = tpm_tis_i2c_status,
	.recv = tpm_tis_i2c_recv,
	.send = tpm_tis_i2c_send,
	.cancel = tpm_tis_i2c_ready,
	.req_complete_mask = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_complete_val = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_canceled = TPM_STS_COMMAND_READY,
	.attr_group = &tis_attr_grp,
	.miscdev = {
		    .fops = &tis_ops,},
};

static int tpm_tis_i2c_init(struct device *dev)
{
	u32 vendor;
	int rc = 0;
	struct tpm_chip *chip;
	chip = tpm_register_hardware(dev, &tpm_tis_i2c);
	if (!chip) {
		rc = -ENODEV;
		goto out_err;
	}

	/* Disable interrupts */
	chip->vendor.irq = 0;

	/* Default timeouts */
	chip->vendor.timeout_a = msecs_to_jiffies(TIS_SHORT_TIMEOUT);
	chip->vendor.timeout_b = msecs_to_jiffies(TIS_LONG_TIMEOUT);
	chip->vendor.timeout_c = msecs_to_jiffies(TIS_SHORT_TIMEOUT);
	chip->vendor.timeout_d = msecs_to_jiffies(TIS_SHORT_TIMEOUT);

	if (request_locality(chip, 0) != 0) {
		rc = -ENODEV;
		goto out_vendor;
	}

	/* read four bytes from DID_VID register */
	if (iic_tpm_read(TPM_DID_VID(0), (u8 *) &vendor, 4) < 0) {
		rc = -EIO;
		goto out_vendor;
	}

	/* create DID_VID register value, after swapping to little-endian */
	vendor = cpu_to_le32(vendor);

	if (vendor != TPM_TIS_I2C_DID_VID &&
		vendor != TPM_TIS_I2C_DID_VID_9635 &&
		vendor != TPM_TIS_I2C_DID_VID_9645) {
		rc = -ENODEV;
		goto out_release;
	}

	dev_info(dev, "1.2 TPM (device-id 0x%X)\n", vendor >> 16);

	INIT_LIST_HEAD(&chip->vendor.list);
	tpm_dev.chip = chip;

	tpm_get_timeouts(chip);
	tpm_continue_selftest(chip);

	return 0;

out_release:
	release_locality(chip, chip->vendor.locality, 1);

out_vendor:
	tpm_dev_vendor_release(chip);
	tpm_remove_hardware(chip->dev);

out_err:
	return rc;
}


static const struct i2c_device_id tpm_i2c_tis_table[] = {
	{ "tpm_tis_i2c", 0 },
	{ },
};

#ifdef CONFIG_PM
/* NOTE:
 * Suspend does currently not work Nvidias Tegra2 Platform
 * but works fine on Beagleboard (arm omap).
 *
 * This function will block System Suspend if TPM is not initialized,
 * however the TPM is usually initialized by BIOS/u-boot or by sending
 * a tpm startup command.
 */
/*
static int tpm_tis_i2c_suspend(struct device *dev)
{
	return tpm_pm_suspend(dev, dev->power.power_state);
}

static int tpm_tis_i2c_resume(struct device *dev)
{
	return tpm_pm_resume(dev);
}


static const struct dev_pm_ops tpm_tis_i2c_ops = {
	.suspend = tpm_tis_i2c_suspend,
	.resume = tpm_tis_i2c_resume,
};
*/
#endif


static int dummy_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	pr_err("%s\n", __func__);
	return 0;
}

static int dummy_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver tpm_i2c_tis_driver = {

	.id_table = tpm_i2c_tis_table,
	.probe = dummy_probe,
	.remove = dummy_remove,

	.suspend = tpm_pm_suspend,
	.resume = tpm_pm_resume,

	.driver	= {
		.name	= "tpm_tis_i2c",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
/*
		.pm     = &tpm_tis_i2c_ops,
*/

#endif
	},
};


static struct i2c_adapter *adap;
static struct i2c_client *client;
static	struct i2c_board_info info = {
	I2C_BOARD_INFO("tpm_tis_i2c", 0x20),
};


static int addr = 0x20;
module_param(addr, int, S_IRUGO);
MODULE_PARM_DESC(addr, "TPM I2C Device Address (default: 0x20)");

static int bus_id = 9;
module_param(bus_id, int, S_IRUGO);
MODULE_PARM_DESC(bus_id, "TPM I2C Bus Id (default: 9)");

static int __init init_tis_i2c(void)
{
	int rc = 0;
	info.addr = addr;

	if (tpm_dev.client != NULL)
		return -EBUSY;

	adap = i2c_get_adapter(bus_id);
	if (!adap)
		return -ENODEV;

	client = i2c_new_device(adap, &info);
	if (!client) {
		i2c_put_adapter(adap);
		return -ENODEV;
	}

	rc = i2c_add_driver(&tpm_i2c_tis_driver);
	if (rc != 0) {
		i2c_del_driver(&tpm_i2c_tis_driver);
		i2c_put_adapter(tpm_dev.client->adapter);
		return -ENODEV;
	}

	client->driver = &tpm_i2c_tis_driver;
	tpm_dev.client = client;
	rc = tpm_tis_i2c_init(&client->dev);
	if (rc < 0) {
		i2c_del_driver(&tpm_i2c_tis_driver);
		i2c_put_adapter(tpm_dev.client->adapter);
		device_del(&(tpm_dev.client->dev));
	}

	return rc;
}

static void __exit cleanup_tis_i2c(void)
{
	struct tpm_chip *chip = tpm_dev.chip;
	release_locality(chip, chip->vendor.locality, 1);

	tpm_dev_vendor_release(chip);
	tpm_remove_hardware(chip->dev);

	i2c_del_driver(&tpm_i2c_tis_driver);

	i2c_put_adapter(tpm_dev.client->adapter);

	/*
	 * taken from core.c as workaround since
	 * tpm_remove_hardware requires device structure
	 */
	pr_debug("device: '%s': %s\n",
		dev_name(&(tpm_dev.client->dev)), __func__);
	device_del(&(tpm_dev.client->dev));
}

module_init(init_tis_i2c);
module_exit(cleanup_tis_i2c);
MODULE_AUTHOR("Peter Huewe <huewe.external@infineon.com>");
MODULE_DESCRIPTION("TPM TIS I2C Driver");
MODULE_VERSION("2.1.2");
MODULE_LICENSE("GPL");
