// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2014-2019 Nuvoton Technology corporation
 *
 * TPM I2C PTP
 *
 * TPM I2C Device Driver Interface for devices that implement the TPM I2C
 * Interface defined by TCG PC Client Platform TPM Profile (PTP) Specification
 * Revision 01.03 v22 at www.trustedcomputinggroup.org
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/freezer.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/crc-ccitt.h>
#include "tpm.h"

/* I2C interface offsets */
#define TPM_LOC_SEL                     0x00
#define TPM_ACCESS                      0x04
#define TPM_INT_ENABLE                  0x08
#define TPM_INT_STATUS                  0x10
#define TPM_INT_CAPABILITY              0x14
#define TPM_STS                         0x18
#define TPM_STS_BURST_COUNT             0x19
#define TPM_DATA_FIFO                   0x24
#define TPM_I2C_INTERFACE_CAPABILITY    0x30
#define TPM_I2C_DEVICE_ADDRESS          0x38
#define TPM_DATA_CSUM_ENABLE            0x40
#define TPM_DATA_CSUM                   0x44
#define TPM_VID_DID                     0x48
#define TPM_RID                         0x4C

 /* max. command/response length */
#define TPM_I2C_BUFSIZE                 2048

/* I2C bus device maximum buffer size w/o counting I2C address or command */
#define TPM_I2C_MAX_BUF_SIZE            32

#define TPM_I2C_MAX_RETRY_CNT           5
#define TPM_I2C_RETRY_DELAY_SHORT_US    (2 * 1000)
#define TPM_I2C_RETRY_DELAY_LONG_US     (10 * 1000)
#define TPM_I2C_DELAY_RANGE_US          300

#define OF_IS_TPM2 ((void *)1)
#define I2C_IS_TPM2 1

struct tpm_tis_data {
	u16 manufacturer_id;
	int locality;
	int irq;
	bool irq_tested;
	unsigned int flags;
	wait_queue_head_t int_queue;
	wait_queue_head_t read_queue;
	const struct tpm_tis_phy_ops *phy_ops;
	unsigned int intrs;
	unsigned short rng_quality;
};

#define MAX_COUNT               3
#define SLEEP_DURATION_LOW      55
#define SLEEP_DURATION_HI       65

static int iic_tpm_read(struct i2c_client *client, u8 addr, u8 *buffer,
			size_t len)
{
	struct i2c_msg msg1 = {
		.addr = client->addr,
		.len = 1,
		.buf = &addr
	};
	struct i2c_msg msg2 = {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = buffer
	};
	int rc = 0;
	int count;

	if (!client->adapter->algo->master_xfer)
		return -EOPNOTSUPP;
	i2c_lock_bus(client->adapter, I2C_LOCK_ROOT_ADAPTER);
	for (count = 0; count < MAX_COUNT; count++) {
		rc = __i2c_transfer(client->adapter, &msg1, 1);
		if (rc > 0)
			break;	/* break here to skip sleep */
		usleep_range(SLEEP_DURATION_LOW, SLEEP_DURATION_HI);
	}
	if (rc <= 0)
		goto out;
	for (count = 0; count < MAX_COUNT; count++) {
		rc = __i2c_transfer(client->adapter, &msg2, 1);
		if (rc > 0)
			break;
		usleep_range(SLEEP_DURATION_LOW, SLEEP_DURATION_HI);
	}
out:
	i2c_unlock_bus(client->adapter, I2C_LOCK_ROOT_ADAPTER);
	usleep_range(SLEEP_DURATION_LOW, SLEEP_DURATION_HI);
	if (rc <= 0)
		return -EIO;
	return len;
}

static u8 tpm_dev_buf[TPM_I2C_BUFSIZE + sizeof(u8)]; /* max buff size + addr */

#ifdef CONFIG_TCG_TIS_I2C_PTP_MAX_SIZE
	static int i2c_max_size = CONFIG_TCG_TIS_I2C_PTP_MAX_SIZE;
#else
	static int i2c_max_size = TPM_I2C_MAX_BUF_SIZE;
#endif
module_param(i2c_max_size, int, 0660);

static int iic_tpm_write_generic(struct i2c_client *client,
				 u8 addr, u8 *buffer, size_t len,
				 unsigned int sleep_low,
				 unsigned int sleep_hi, u8 max_count)
{
	int rc = -EIO;
	int count;
	struct i2c_msg msg1 = {
		.addr = client->addr,
		.len = len + 1,
		.buf = tpm_dev_buf
	};

	if (len > TPM_BUFSIZE)
		return -EINVAL;
	if (!client->adapter->algo->master_xfer)
		return -EOPNOTSUPP;
	i2c_lock_bus(client->adapter, I2C_LOCK_ROOT_ADAPTER);
	tpm_dev_buf[0] = addr;
	memcpy(&tpm_dev_buf[1], buffer, len);
	for (count = 0; count < max_count; count++) {
		rc = __i2c_transfer(client->adapter, &msg1, 1);
		if (rc > 0)
			break;
		usleep_range(sleep_low, sleep_hi);
	}
	i2c_unlock_bus(client->adapter, I2C_LOCK_ROOT_ADAPTER);
	usleep_range(SLEEP_DURATION_LOW, SLEEP_DURATION_HI);
	if (rc <= 0)
		return -EIO;
	return 0;
}

static s32 i2c_ptp_read_buf(struct i2c_client *client, u8 offset, size_t size,
			    u8 *data)
{
	s32 status;

	status = iic_tpm_read(client, offset, data, size);
	dev_dbg(&client->dev,
		"%s(offset=%u size=%zu data=%*ph) -> sts=%d\n", __func__,
		offset, size, (int)size, data, status);
	return status;
}

static s32 i2c_ptp_write_buf(struct i2c_client *client, u8 offset, size_t size,
			     u8 *data)
{
	s32 status;

	status = iic_tpm_write_generic(client, offset, data, size,
				       SLEEP_DURATION_LOW, SLEEP_DURATION_HI,
				       MAX_COUNT);
	dev_dbg(&client->dev,
		"%s(offset=%u size=%zu data=%*ph) -> sts=%d\n", __func__,
		offset, size, (int)size, data, status);

	return status;
}

#define TPM_INT_DATA_AVAIL		(BIT(0))
#define TPM_INT_STS_VALID		(BIT(1))
#define TPM_INT_LOC_CHANGED		(BIT(2))
#define TPM_INT_CMD_READY		(BIT(7))
#define TPM_INT_GLOBAL			(BIT(31))

#define TPM_INT_TO_ACTIVATE		(TPM_INT_DATA_AVAIL)

static bool wait_for_tpm_stat_cond(struct tpm_chip *chip, u8 mask,
				   bool check_cancel, bool *canceled)
{
	u8 status = chip->ops->status(chip);

	*canceled = false;
	if (status != 0xff && (status & mask) == mask)
		return true;
	if (check_cancel && chip->ops->req_canceled(chip, status)) {
		*canceled = true;
		return true;
	}
	return false;
}

int i2c_ptp_wait_for_tpm_stat_l(struct tpm_chip *chip, u8 mask,
				unsigned long timeout, wait_queue_head_t *queue,
				bool check_cancel)
{
	struct tpm_tis_data *priv = dev_get_drvdata(&chip->dev);
	unsigned long stop, start, long_delay;
	long rc;
	u8 status;
	bool canceled = false;
	unsigned int cur_intrs;

	start = jiffies;
	stop = start + timeout;
	long_delay = start + usecs_to_jiffies(TPM_I2C_RETRY_DELAY_LONG_US);

	if (queue && chip->flags & TPM_CHIP_FLAG_IRQ) {
again:
		cur_intrs = priv->intrs;
		timeout = stop - jiffies;
		if ((long)timeout <= 0)
			return -ETIME;

		enable_irq(priv->irq);
		rc = wait_event_interruptible_timeout(*queue,
						      ((cur_intrs != priv->intrs) &&
						      wait_for_tpm_stat_cond(chip, mask, check_cancel, &canceled)),
						      timeout);
		if (rc > 0) {
			if (canceled)
				return -ECANCELED;
			return 0;
		}
		if (rc == -ERESTARTSYS && freezing(current)) {
			clear_thread_flag(TIF_SIGPENDING);
			goto again;
		}
	} else {
		do {
			status = chip->ops->status(chip);
			if (status != 0xff && (status & mask) == mask)
				return 0;

			if (time_before(jiffies, long_delay))
				usleep_range(TPM_I2C_RETRY_DELAY_SHORT_US,
					     TPM_I2C_RETRY_DELAY_SHORT_US
					     + TPM_I2C_DELAY_RANGE_US);
			else
				usleep_range(TPM_I2C_RETRY_DELAY_LONG_US,
					     TPM_I2C_RETRY_DELAY_LONG_US
					     + TPM_I2C_DELAY_RANGE_US);

		} while (time_before(jiffies, stop));
	}
	return -ETIME;
}

/* wrapper of wait_for_tpm_stat, since we can't do the int processing during
 * the int_handler in I2C, here we do it after the int_handler is done and
 * wait_for_tpm_stat retruns
 */
static int i2c_ptp_wait_for_tpm_stat(struct tpm_chip *chip, u8 mask,
				     unsigned long timeout,
				     wait_queue_head_t *queue,
				     bool check_cancel)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	u32 intsts;
	int rc;

	rc = i2c_ptp_wait_for_tpm_stat_l(chip, mask, timeout, queue,
					 check_cancel);
	if (rc < 0)
		return rc;

	if (chip->flags & TPM_CHIP_FLAG_IRQ) {
		rc = i2c_ptp_read_buf(client, TPM_INT_STATUS, 4, (u8 *)&intsts);
		if (rc < 0) {
			dev_err(&chip->dev,
				"%s() fail to read TPM_INT_STATUS\n", __func__);
			return -EIO;
		}

		/* Clear interrupts handled */
		rc = i2c_ptp_write_buf(client, TPM_INT_STATUS, 4,
				       (u8 *)&intsts);
		if (rc < 0) {
			dev_err(&chip->dev,
				"%s() fail to clear int status\n", __func__);
			return -EIO;
		}
	}

	return 0;
}

#define TPM_ACCESS_REQUEST_USE		(BIT(1))
#define TPM_ACCESS_REQUEST_PENDING	(BIT(2))
#define TPM_ACCESS_ACTIVE_LOCALITY	(BIT(5))
#define TPM_ACCESS_VALID_STS		(BIT(7))

/* Before we attempt to access the TPM we must see that the valid bit is set.
 * The specification says that this bit is 0 at reset and remains 0 until the
 * 'TPM has gone through its self test and initialization and has established
 * correct values in the other bits.'
 */
static int wait_startup(struct tpm_chip *chip, int l)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	unsigned long stop = jiffies + chip->timeout_a;

	do {
		int rc;
		u8 access;

		rc = i2c_ptp_read_buf(client, TPM_ACCESS, 1, &access);
		if (rc < 0)
			return rc;

		if (access != 0xff && (access & TPM_ACCESS_VALID_STS) != 0)
			return 0;
		tpm_msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));
	return -1;
}

static bool i2c_ptp_check_locality(struct tpm_chip *chip, int l)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	struct tpm_tis_data *priv = dev_get_drvdata(&chip->dev);
	int rc;
	u8 access;
	u8 data;

	data = (u8)l;
	rc = i2c_ptp_write_buf(client, TPM_LOC_SEL, 1, &data);
	if (rc < 0)
		return false;

	rc = i2c_ptp_read_buf(client, TPM_ACCESS, 1, &access);
	if (rc < 0)
		return false;

	if ((access & (TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID_STS)) ==
	    (TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID_STS)) {
		priv->locality = l;
		return true;
	}

	return false;
}

static bool i2c_ptp_locality_inactive(struct tpm_chip *chip, int l)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	int rc;
	u8 access;
	u8 data;

	data = (u8)l;
	rc = i2c_ptp_write_buf(client, TPM_LOC_SEL, 1, &data);
	if (rc < 0)
		return false;

	rc = i2c_ptp_read_buf(client, TPM_ACCESS, 1, &access);
	if (rc < 0)
		return false;

	if ((access & (TPM_ACCESS_VALID_STS | TPM_ACCESS_ACTIVE_LOCALITY))
	    == TPM_ACCESS_VALID_STS)
		return true;

	return false;
}

static int i2c_ptp_release_locality(struct tpm_chip *chip, int l)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	unsigned long stop;
	int rc;
	u8 data;

	if (i2c_ptp_locality_inactive(chip, l))
		return 0;

	data = TPM_ACCESS_ACTIVE_LOCALITY;
	rc = i2c_ptp_write_buf(client, TPM_ACCESS, 1, &data);
	if (rc < 0)
		return rc;

	stop = jiffies + chip->timeout_a;
	do {
		if (i2c_ptp_locality_inactive(chip, l))
			return 0;

		tpm_msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));

	return -1;
}

static int i2c_ptp_request_locality(struct tpm_chip *chip, int l)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	unsigned long stop;
	int rc;
	u8 data;

	if (i2c_ptp_check_locality(chip, l))
		return l;

	data = TPM_ACCESS_REQUEST_USE;
	rc = i2c_ptp_write_buf(client, TPM_ACCESS, 1, &data);
	if (rc < 0)
		return rc;

	stop = jiffies + chip->timeout_a;

	do {
		if (i2c_ptp_check_locality(chip, l))
			return l;

		rc = i2c_ptp_write_buf(client, TPM_ACCESS, 1, &data);
		if (rc < 0)
			return rc;

		tpm_msleep(TPM_TIMEOUT);
	} while (time_before(jiffies, stop));

	return -1;
}

#define TPM_STS_RESPONSE_RETRY (BIT(1))
#define TPM_STS_SELFTEST_DONE  (BIT(2))
#define TPM_STS_EXPECT         (BIT(3))
#define TPM_STS_DATA_AVAIL     (BIT(4))
#define TPM_STS_GO             (BIT(5))
#define TPM_STS_COMMAND_READY  (BIT(6))
#define TPM_STS_VALID          (BIT(7))
#define TPM_STS_COMMAND_CANCEL (BIT(24))

/* bit1...bit0 reads always 0 */
#define TPM_STS_ERR_VAL        (BIT(0) | BIT(1))

/* read TPM_STS register */
static u8 i2c_ptp_status(struct tpm_chip *chip)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	int rc;
	u8 status;

	rc = i2c_ptp_read_buf(client, TPM_STS, 1, &status);
	if (rc < 0)
		return 0;

	return status;
}

/* write commandReady to TPM_STS register */
static void i2c_ptp_ready(struct tpm_chip *chip)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	u8 data;

	data = TPM_STS_COMMAND_READY;
	i2c_ptp_write_buf(client, TPM_STS, 1, &data);
}

/* read Burst Count */
static int i2c_ptp_get_burstcount(struct tpm_chip *chip)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	unsigned long stop;
	int burstcnt = 0, rc;

	/* wait for burstcount */
	stop = jiffies + chip->timeout_a;

	do {
		rc = i2c_ptp_read_buf(client, TPM_STS_BURST_COUNT, 2,
				      (u8 *)&burstcnt);
		if (rc < 0)
			return rc;

		if (burstcnt)
			return burstcnt;

		usleep_range(TPM_TIMEOUT_USECS_MIN, TPM_TIMEOUT_USECS_MAX);
	} while (time_before(jiffies, stop));

	return -EBUSY;
}

/* write commandCancel to TPM_STS register */
static void i2c_ptp_cancel(struct tpm_chip *chip)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	u32 data;

	data = TPM_STS_COMMAND_CANCEL;
	i2c_ptp_write_buf(client, TPM_STS, 4, (u8 *)&data);
}

/* enable checksum */
static int i2c_ptp_enable_csum(struct tpm_chip *chip)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	s32 rc;
	u8 data = 1;

	/* Enable CSUM */
	rc = i2c_ptp_write_buf(client, TPM_DATA_CSUM_ENABLE, 1, &data);
	if (rc < 0) {
		dev_err(&chip->dev,
			"%s() fail to read to TPM_DATA_CSUM_ENABLE: %d\n",
			__func__, rc);
		return rc;
	}
	return 0;
}

/* Caclculate crc16 of the buffer and compares it to TPM_DATA_CSUM */
static int i2c_ptp_check_crc(struct tpm_chip *chip, u8 *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	s32 status;
	u16 tpm_csum = 0;
	u16 crc = 0;

	crc = crc_ccitt(crc, buf, len);
	crc = be16_to_cpu(crc);
	status = i2c_ptp_read_buf(client, TPM_DATA_CSUM, 2, (u8 *)&tpm_csum);
	if (status <= 0) {
		dev_err(&chip->dev, "%s() fail to read to TPM_DATA_CSUM: %d\n",
			__func__, status);
		return -EIO;
	}

	if (tpm_csum != crc) {
		dev_err(&chip->dev, "%s() bad data csum\n", __func__);
		return -EIO;
	}

	return 0;
}

static int i2c_ptp_recv_data(struct tpm_chip *chip, u8 *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	int size = 0, burstcnt, rc, bytes2read = count;
	bool paramsize_flag = false;

	while (size < bytes2read) {
		burstcnt = i2c_ptp_get_burstcount(chip);
		if (burstcnt <= 0)
			return burstcnt;

		burstcnt = min_t(int, (bytes2read - size), burstcnt);
		rc = i2c_ptp_read_buf(client, TPM_DATA_FIFO, burstcnt,
				      buf + size);
		if (rc < 0)
			return rc;

		size += burstcnt;

		if (size >= 6 && !paramsize_flag) {
			bytes2read = be32_to_cpu(*(__be32 *)(buf + 2));
			paramsize_flag = true;
		}
	}

	return size;
}

static int i2c_ptp_recv(struct tpm_chip *chip, u8 *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	int size = 0;
	int status, retries;

	if (count < TPM_HEADER_SIZE) {
		status = -EIO;
		goto out;
	}

	for (retries = 0; retries < TPM_I2C_MAX_RETRY_CNT; retries++) {
		size = 0;

		if (retries > 0) {
			/* if this is not the first trial, set responseRetry */
			u8 data = TPM_STS_RESPONSE_RETRY;

			i2c_ptp_write_buf(client, TPM_STS, 1, &data);
		}

		status = i2c_ptp_wait_for_tpm_stat(chip,
						   (TPM_STS_DATA_AVAIL | TPM_STS_VALID),
						   chip->timeout_b, NULL,
						   false);
		if (status < 0)
			return status;

		size = i2c_ptp_recv_data(chip, buf, TPM_HEADER_SIZE);
		if (size < TPM_HEADER_SIZE)
			continue;

		status = i2c_ptp_wait_for_tpm_stat(chip, TPM_STS_VALID,
						   chip->timeout_a, NULL,
						   false);
		if (status < 0)
			continue;

		/* check CRC */
		status = i2c_ptp_check_crc(chip, buf, size);
		if (status < 0)
			continue;

		status = size;
		break;
	}

out:
	i2c_ptp_ready(chip);
	return status;
}

/*
 * If interrupts are used (signaled by an irq set in the vendor structure)
 * tpm.c can skip polling for the data to be available as the interrupt is
 * waited for here
 */
static int i2c_ptp_send_data(struct tpm_chip *chip, u8 *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	int rc, status, burstcnt, retries;
	size_t count = 0, bytes2send;
	u32 ordinal = be32_to_cpu(*((__be32 *)(buf + 6)));
	u32 intmask;

	/* When NPCT7XX FU Mode command or startup (when TPM I2C may be has
	 * been reset), wait for STS_VALID and re-initialize I2C regs
	 */
	if (ordinal == 0x20000201 || ordinal == 0x144) {
		// wait for I2C to be ready
		if (wait_startup(chip, 0) != 0)
			return -ENODEV;

		rc = i2c_ptp_enable_csum(chip);
		if (rc < 0) {
			dev_err(&chip->dev, "%s() fail to enable checksum\n",
				__func__);
			return rc;
		}

		// in interrupt mode re-eanble and clear the interrupts
		if (chip->flags & TPM_CHIP_FLAG_IRQ) {
			intmask = (TPM_INT_TO_ACTIVATE | TPM_INT_GLOBAL);
			i2c_ptp_write_buf(client, TPM_INT_ENABLE, 4,
					  (u8 *)&intmask);
			i2c_ptp_write_buf(client, TPM_INT_STATUS, 4,
					  (u8 *)&intmask);
		}
	}

	for (retries = 0; retries < TPM_I2C_MAX_RETRY_CNT; retries++) {
		count = 0;
		rc = -1;

		if (retries > 0) {
			/* if this is not the first trial, abort the command */
			i2c_ptp_ready(chip);
		}

		status = i2c_ptp_status(chip);
		if ((status & TPM_STS_COMMAND_READY) == 0) {
			i2c_ptp_ready(chip);
			if (i2c_ptp_wait_for_tpm_stat
			    (chip, TPM_STS_COMMAND_READY, chip->timeout_b,
			     NULL, false) < 0) {
				rc = -ETIME;
				continue;
			}
		}

		while (count < len) {
			burstcnt = i2c_ptp_get_burstcount(chip);
			if (burstcnt < 0) {
				dev_err(&chip->dev, "Unable to read burstcount\n");
				rc = burstcnt;
				break;
			}

			bytes2send = min_t(int, i2c_max_size, burstcnt);
			bytes2send = min_t(int, bytes2send, (len - count));
			rc = i2c_ptp_write_buf(client, TPM_DATA_FIFO,
					       bytes2send, buf + count);
			if (rc < 0)
				break;

			count += bytes2send;
		}

		if (rc < 0)
			continue;

		if (i2c_ptp_wait_for_tpm_stat(chip, TPM_STS_VALID,
					      chip->timeout_a, NULL,
					      false) < 0) {
			rc = -ETIME;
			continue;
		}

		/* check CRC */
		rc = i2c_ptp_check_crc(chip, buf, len);
		if (rc < 0)
			continue;

		return 0;
	}

	i2c_ptp_ready(chip);
	return rc;
}

static void disable_interrupts(struct tpm_chip *chip)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	struct tpm_tis_data *priv = dev_get_drvdata(&chip->dev);
	u32 intmask;
	int rc;

	rc = i2c_ptp_read_buf(client, TPM_INT_ENABLE, 4, (u8 *)&intmask);
	if (rc < 0)
		intmask = 0;

	intmask &= ~TPM_INT_GLOBAL;
	rc = i2c_ptp_write_buf(client, TPM_INT_ENABLE, 4, (u8 *)&intmask);
	if (rc < 0)
		dev_err(&chip->dev, "%s() fail to write TPM_INT_ENABLE\n",
			__func__);

	devm_free_irq(chip->dev.parent, priv->irq, chip);
	priv->irq = 0;
	chip->flags &= ~TPM_CHIP_FLAG_IRQ;
}

/*
 * If interrupts are used (signaled by an irq set in the vendor structure)
 * tpm.c can skip polling for the data to be available as the interrupt is
 * waited for here
 */
static int i2c_ptp_send_main(struct tpm_chip *chip, u8 *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	struct tpm_tis_data *priv = dev_get_drvdata(&chip->dev);
	int rc;
	u32 ordinal;
	unsigned long dur;
	u8 data;

	rc = i2c_ptp_send_data(chip, buf, len);
	if (rc < 0)
		return rc;

	/* go and do it */
	data = TPM_STS_GO;
	rc = i2c_ptp_write_buf(client, TPM_STS, 1, &data);
	if (rc < 0)
		goto out_err;

	ordinal = be32_to_cpu(*((__be32 *)(buf + 6)));

	dur = tpm_calc_ordinal_duration(chip, ordinal);
	if (i2c_ptp_wait_for_tpm_stat
	    (chip, TPM_STS_DATA_AVAIL | TPM_STS_VALID, dur,
	     &priv->int_queue, false) < 0) {
		rc = -ETIME;
		goto out_err;
	}

	return 0;
out_err:
	i2c_ptp_ready(chip);
	return rc;
}

static int i2c_ptp_send(struct tpm_chip *chip, u8 *buf, size_t len)
{
	int rc, irq;
	struct tpm_tis_data *priv = dev_get_drvdata(&chip->dev);

	if (!(chip->flags & TPM_CHIP_FLAG_IRQ) || priv->irq_tested)
		return i2c_ptp_send_main(chip, buf, len);

	/* Verify receipt of the expected IRQ */
	irq = priv->irq;
	chip->flags &= ~TPM_CHIP_FLAG_IRQ;
	rc = i2c_ptp_send_main(chip, buf, len);
	priv->irq = irq;
	chip->flags |= TPM_CHIP_FLAG_IRQ;
	if (!priv->irq_tested)
		tpm_msleep(1);
	if (!priv->irq_tested)
		disable_interrupts(chip);
	priv->irq_tested = true;
	return rc;
}

static bool i2c_ptp_req_canceled(struct tpm_chip *chip, u8 status)
{
	return (status == TPM_STS_COMMAND_READY);
}

/* The only purpose for the handler is to signal to any waiting threads that
 * the interrupt is currently being asserted. The driver does not do any
 * processing triggered by interrupts, and the chip provides no way to mask at
 * the source (plus that would be slow over I2C). Run the IRQ as a one-shot,
 * this means it cannot be shared. The processing of the interrupt status
 * is done in i2c_ptp_wait_for_tpm_stat
 */
static irqreturn_t i2c_ptp_int_handler(int dummy, void *dev_id)
{
	struct tpm_chip *chip = dev_id;
	struct tpm_tis_data *priv = dev_get_drvdata(&chip->dev);

	priv->irq_tested = true;
	priv->intrs++;
	wake_up_interruptible(&priv->int_queue);
	disable_irq_nosync(priv->irq);
	return IRQ_HANDLED;
}

static int i2c_ptp_gen_interrupt(struct tpm_chip *chip)
{
	const char *desc = "attempting to generate an interrupt";
	u32 cap2;

	return tpm2_get_tpm_pt(chip, 0x100, &cap2, desc);
}

/* Register the IRQ and issue a command that will cause an interrupt. If an
 * irq is seen then leave the chip setup for IRQ operation, otherwise reverse
 * everything and leave in polling mode. Returns 0 on success.
 */
static int i2c_ptp_probe_irq_single(struct tpm_chip *chip, u32 intmask,
				    int flags, int irq)
{
	struct i2c_client *client = to_i2c_client(chip->dev.parent);
	struct tpm_tis_data *priv = dev_get_drvdata(&chip->dev);
	int rc;
	u32 int_support;

	if (devm_request_irq(chip->dev.parent, irq, i2c_ptp_int_handler, flags,
			     dev_name(&chip->dev), chip) != 0) {
		dev_info(&chip->dev, "Unable to request irq: %d for probe\n",
			 irq);
		return -1;
	}
	priv->irq = irq;

	chip->flags |= TPM_CHIP_FLAG_IRQ;

	/* check what are the interrupts that the TPM supports */
	rc = i2c_ptp_read_buf(client, TPM_INT_CAPABILITY, 4, (u8 *)
			   &int_support);
	if (rc < 0)
		return rc;

	/* Clear all existing */
	rc = i2c_ptp_write_buf(client, TPM_INT_STATUS, 4, (u8 *)&int_support);
	if (rc < 0)
		return rc;

	/* Turn on */
	int_support &= TPM_INT_TO_ACTIVATE;
	int_support |= TPM_INT_GLOBAL;
	rc = i2c_ptp_write_buf(client, TPM_INT_ENABLE, 4, (u8 *)&int_support);
	if (rc < 0)
		return rc;

	priv->irq_tested = false;

	/* Generate an interrupt by having the core call through to
	 * i2c_ptp_send
	 */
	rc = i2c_ptp_gen_interrupt(chip);
	if (rc < 0)
		return rc;

	return 0;
}

static int i2c_ptp_remove(struct i2c_client *client)
{
	u32 interrupt;
	int rc;

	rc = i2c_ptp_read_buf(client, TPM_INT_ENABLE, 4, (u8 *)&interrupt);
	if (rc < 0)
		interrupt = 0;

	// Clear and disable interrupts
	interrupt &= ~TPM_INT_GLOBAL;
	rc = i2c_ptp_write_buf(client, TPM_INT_STATUS, 4, (u8 *)&interrupt);
	rc = i2c_ptp_write_buf(client, TPM_INT_ENABLE, 4, (u8 *)&interrupt);

	return rc;
}
EXPORT_SYMBOL_GPL(i2c_ptp_remove);

static const struct tpm_class_ops tpm_i2c = {
	.flags = TPM_OPS_AUTO_STARTUP,
	.status = i2c_ptp_status,
	.recv = i2c_ptp_recv,
	.send = i2c_ptp_send,
	.cancel = i2c_ptp_cancel,
	.req_complete_mask = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_complete_val = TPM_STS_DATA_AVAIL | TPM_STS_VALID,
	.req_canceled = i2c_ptp_req_canceled,
	.request_locality = i2c_ptp_request_locality,
	.relinquish_locality = i2c_ptp_release_locality,
};

int i2c_ptp_core_init(struct i2c_client *client, struct tpm_tis_data *priv,
		      int irq, const struct tpm_tis_phy_ops *phy_ops,
		      acpi_handle acpi_dev_handle)
{
	u32 vendor;
	u32 intmask;
	u8 rid;
	int rc;
	struct tpm_chip *chip;
	struct device *dev = &client->dev;

	chip = tpmm_chip_alloc(dev, &tpm_i2c);
	if (IS_ERR(chip))
		return PTR_ERR(chip);

	chip->hwrng.quality = priv->rng_quality;

	/* Maximum timeouts */
	chip->timeout_a = msecs_to_jiffies(TPM2_TIMEOUT_A);
	chip->timeout_b = msecs_to_jiffies(TPM2_TIMEOUT_B);
	chip->timeout_c = msecs_to_jiffies(TPM2_TIMEOUT_C);
	chip->timeout_d = msecs_to_jiffies(TPM2_TIMEOUT_D);
	priv->phy_ops = phy_ops;
	priv->intrs = 0;
	dev_set_drvdata(&chip->dev, priv);

	if (wait_startup(chip, 0) != 0) {
		rc = -ENODEV;
		goto out_err;
	}

	/* Take control of the TPM's interrupt hardware and shut it off */
	rc = i2c_ptp_read_buf(client, TPM_INT_ENABLE, 4, (u8 *)&intmask);
	if (rc < 0)
		goto out_err;

	intmask = TPM_INT_TO_ACTIVATE;	// global should be off now
	i2c_ptp_write_buf(client, TPM_INT_ENABLE, 4, (u8 *)&intmask);

	rc = i2c_ptp_enable_csum(chip);
	if (rc < 0) {
		dev_err(&chip->dev, "%s() fail to enable checksum\n", __func__);
		rc = -ENODEV;
		goto out_err;
	}

	rc = tpm_chip_start(chip);
	if (rc)
		goto out_err;

	rc = tpm2_probe(chip);
	if (rc)
		goto out_err;

	rc = i2c_ptp_read_buf(client, TPM_VID_DID, 4, (u8 *)&vendor);
	if (rc < 0)
		goto out_err;

	priv->manufacturer_id = vendor;

	rc = i2c_ptp_read_buf(client, TPM_RID, 1, &rid);
	if (rc < 0)
		goto out_err;

	dev_info(dev, "%s TPM (device-id 0x%X, rev-id %d)\n",
		 (chip->flags & TPM_CHIP_FLAG_TPM2) ? "2.0" : "1.2",
		 vendor >> 16, rid);

	/* INTERRUPT Setup */
	init_waitqueue_head(&priv->int_queue);
	if (irq != -1) {
		/* Before doing irq testing issue a command to the TPM in polling mode
		 * to make sure it works. May as well use that command to set the
		 * proper timeouts for the driver.
		 */
		if (tpm_get_timeouts(chip)) {
			dev_err(dev, "Could not get TPM timeouts and durations\n");
			rc = -ENODEV;
			goto out_err;
		}

		i2c_ptp_probe_irq_single(chip, intmask,	IRQF_TRIGGER_LOW, irq);
		if (!(chip->flags & TPM_CHIP_FLAG_IRQ))
			dev_err(&chip->dev, FW_BUG
				"TPM interrupt not working, polling instead\n");
	}

	rc = tpm_chip_register(chip);
	if (rc)
		goto out_err;

	return 0;

out_err:
	i2c_ptp_remove(client);

	return rc;
}

static int i2c_ptp_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int irq = -1, gpio;
	struct tpm_tis_data *priv;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	priv = devm_kzalloc(&client->dev, sizeof(struct tpm_tis_data),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (i2c_max_size < 1)
		i2c_max_size = CONFIG_TCG_TIS_I2C_PTP_MAX_SIZE;

	// Get interrupt from board device
	if (client->irq) {
		irq = client->irq;
		goto out;
	}

	// If IRQ doesn't exists, get GPIO from device tree
	gpio = of_get_named_gpio(client->dev.of_node, "tpm-pirq", 0);
	if (gpio < 0) {
		dev_err(&client->dev, "Failed to retrieve tpm-pirq\n");
		goto out;
	}

	// GPIO request and configuration
	if (devm_gpio_request_one(&client->dev, gpio, GPIOF_IN, "TPM PIRQ")) {
		dev_err(&client->dev, "Failed to request tpm-pirq pin\n");
		goto out;
	}

	irq = gpio_to_irq(gpio);

out:
	return i2c_ptp_core_init(client, priv, irq, NULL, 0);
}

static const struct of_device_id of_i2c_ptp_match[] = {
	{.compatible = "tcg,tpm_i2c_ptp", .data = OF_IS_TPM2},
	{},
};

static const struct i2c_device_id i2c_ptp_id[] = {
	{"tpm_i2c_ptp"},
	{"tpm2_i2c_ptp", .driver_data = I2C_IS_TPM2},
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_ptp_id);

static SIMPLE_DEV_PM_OPS(i2c_ptp_pm_ops, tpm_pm_suspend, tpm_pm_resume);

static struct i2c_driver i2c_ptp_driver = {
	.id_table = i2c_ptp_id,
	.probe = i2c_ptp_probe,
	.remove = i2c_ptp_remove,
	.driver = {
		.name = "tpm_i2c_ptp",
		.pm = &i2c_ptp_pm_ops,
		.of_match_table = of_match_ptr(of_i2c_ptp_match),
	},
};
module_i2c_driver(i2c_ptp_driver);

MODULE_AUTHOR("Oshri Alkoby (oshri.alkoby@nuvoton.com)");
MODULE_DESCRIPTION("TPM I2C Driver");
MODULE_VERSION("2.1.3");
MODULE_LICENSE("GPL");
