// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2020 Facebook Inc.

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mfd/core.h>
#include <linux/mfd/fboss-usmc.h>
#include <linux/module.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

/*
 * USMC SPI control and data register space sizes.
 */
#define USPI_CTRL_REG_SIZE	0x80
#define USPI_MOSI_BUF_SIZE	0x200	/* 512 bytes */
#define USPI_MISO_BUF_SIZE	0x200	/* 512 bytes */
#define USPI_DATA_BUF_SIZE	(USPI_MOSI_BUF_SIZE + USPI_MISO_BUF_SIZE)

/*
 * Bit fields in SPI Timing Profile Register.
 */
#define USPI_SAMPLE_DELAY_OFFSET	12
#define USPI_CLK_25MHZ			1
#define USPI_CLK_50MHZ			0

/*
 * Bit fields in SPI Descriptor Register.
 */
#define USPI_XFER_START_HW	BIT(31)
#define USPI_CONTINUOUS_CS	BIT(29)
#define USPI_XFER_LEN_OFFSET	8
#define USPI_XFER_LEN_MASK	0x1FF

/*
 * Bit fields in SPI Status Register.
 */
#define USPI_STAT_XFER_DONE	BIT(0)

/*
 * Default timeout settings.
 */
#define USPI_XFER_DELAY_US	10

/*
 * SPI Master Control/Status Registers.
 */
struct usmc_spi_csr {
	u32 timing;
	u32 ctrl;
	u32 desc;
	u32 status;
};

/*
 * Structure for a SPI master.
 */
struct usmc_spi_master {
	u32 id;
	struct spi_master *master;
	struct platform_device *pdev;

	/*
	 * Register base address, initialized at probe time.
	 */
	unsigned long mem_ctrl_reg;
	unsigned long mem_mosi;
	unsigned long mem_miso;

	/*
	 * 1 slave (flash) attached to the spi master.
	 */
	struct spi_device *slave;

	/*
	 * We cannot determine read length when read-related flash ops
	 * are received, so let's cache the read op and combine it with
	 * following "rx" operation.
	 */
	struct {
		u8 op;
		u8 addr[3];
	} read_op_cache;
	unsigned int read_op_size;
};


static void usmc_spi_csr_init(struct usmc_spi_csr *csr)
{
	memset(csr, 0, sizeof(*csr));
	csr->timing = (USPI_CLK_50MHZ |
			(2 << USPI_SAMPLE_DELAY_OFFSET));
	csr->desc &= ~(USPI_XFER_LEN_MASK << USPI_XFER_LEN_OFFSET);
}

static int usmc_spi_xfer_start_hw(struct usmc_spi_master *uspi,
				  unsigned int xfer_len,
				  int continue_cs)
{
	struct usmc_spi_csr csr;

	usmc_spi_csr_init(&csr);

	/*
	 * Set up transfer length.
	 */
	csr.desc |= ((u32)(xfer_len & USPI_XFER_LEN_MASK) <<
			USPI_XFER_LEN_OFFSET);

	/*
	 * Turn on start bit and configure CONTINUOUS_CS.
	 */
	csr.desc |= USPI_XFER_START_HW;
	if (continue_cs)
		csr.desc |= USPI_CONTINUOUS_CS;
	else
		csr.desc &= ~USPI_CONTINUOUS_CS;

	/*
	 * Send the updated CSR register values to the device.
	 */
	return fboss_usmc_write(uspi->pdev, uspi->mem_ctrl_reg, &csr,
				sizeof(csr));
}

static int usmc_spi_mosi_write(struct usmc_spi_master *uspi,
			       const void *buf,
			       unsigned int len)
{
	return fboss_usmc_write(uspi->pdev, uspi->mem_mosi, buf, len);
}

static int usmc_spi_csr_read(struct usmc_spi_master *uspi,
			     void *buf,
			     unsigned int len)
{
	return fboss_usmc_read(uspi->pdev, uspi->mem_ctrl_reg, buf, len);
}

static int usmc_spi_miso_read(struct usmc_spi_master *uspi,
			      void *buf,
			      unsigned int len)
{
	return fboss_usmc_read(uspi->pdev, uspi->mem_miso, buf, len);
}

/*
 * More flash read commands from flashrom spi.h.
 */

/* Some Atmel AT25F* models have bit 3 as don't care bit in commands */
#define AT25F_RDID		0x15	/* 0x15 or 0x1d */

/* Read Electronic Manufacturer Signature */
#define JEDEC_REMS		0x90

/* Read Electronic Signature */
#define JEDEC_RES		0xab

/* Some ST M95X model */
#define ST_M95_RDID		0x83

static bool usmc_spi_is_read_op(u8 flash_op)
{
	bool ret = false;

	switch (flash_op) {
	case SPINOR_OP_RDSR:
	case SPINOR_OP_RDSR2:
	case SPINOR_OP_READ:
	case SPINOR_OP_READ_FAST:
	case SPINOR_OP_RDID:
	case SPINOR_OP_RDSFDP:
	case SPINOR_OP_RDCR:
	case SPINOR_OP_RDFSR:
	case SPINOR_OP_RDEAR:
		ret = true;
		break;

	case AT25F_RDID:
	case ST_M95_RDID:
	case JEDEC_REMS:
	case JEDEC_RES:
		ret = true;
		break;
	}

	return ret;
}

static int usmc_spi_cache_flash_op(struct usmc_spi_master *uspi,
				   struct spi_transfer *xfer)
{
	unsigned int len;
	u8 flash_op = ((u8 *)xfer->tx_buf)[0];

	if (xfer->len <= sizeof(uspi->read_op_cache))
		len = xfer->len;
	else
		len = sizeof(uspi->read_op_cache);

	memcpy(&uspi->read_op_cache, xfer->tx_buf, len);
	if (usmc_spi_is_read_op(flash_op))
		uspi->read_op_size = len;

	return 0;
}

static int usmc_spi_xfer_tx(struct usmc_spi_master *uspi,
			    struct spi_transfer *xfer)
{
	int ret;

	/*
	 * Step 1: upload command/data to mosi buffer.
	 */
	ret = usmc_spi_mosi_write(uspi, xfer->tx_buf, xfer->len);
	if (ret < 0) {
		dev_err(&uspi->pdev->dev,
			"failed to send %d bytes to MOSI, error=%d\n",
			xfer->len, ret);
		return ret;
	}

	/*
	 * Step 2: update control register to start transaction.
	 */
	ret = usmc_spi_xfer_start_hw(uspi, xfer->len, 0);
	if (ret < 0) {
		dev_err(&uspi->pdev->dev,
			"failed to start spi hardware, error=%d\n", ret);
		return ret;
	}

	/* add 40us delay for spi write*/
	udelay(4 * USPI_XFER_DELAY_US);

	return 0;
}

static int usmc_spi_xfer_rx_prepare_op(struct usmc_spi_master *uspi,
					struct spi_transfer *xfer)
{
	int ret;

	/*
	 * The read command should be cached, and let's verify it.
	 */
	if (uspi->read_op_size == 0) {
		dev_err(&uspi->pdev->dev,
			"no read_op command cached (len=%d): last op 0x%02x\n",
			xfer->len, uspi->read_op_cache.op);
		return -EINVAL;
	}

	/*
	 * fill mosi buffer with flash op
	 */
	ret = usmc_spi_mosi_write(uspi, &uspi->read_op_cache,
				  uspi->read_op_size);
	if (ret < 0)
		dev_err(&uspi->pdev->dev,
			"failed to write mosi (op=0x%02x, len=%u), ret=%d\n",
			uspi->read_op_cache.op, uspi->read_op_size, ret);

	return ret;
}

static int usmc_spi_xfer_is_ready(struct usmc_spi_master *uspi)
{
	int ret;
	int retry = 2;
	struct usmc_spi_csr csr;

	while (retry >= 0) {
		ret = usmc_spi_csr_read(uspi, &csr, sizeof(csr));
		if (ret < 0)
			return ret;

		if (csr.status & USPI_STAT_XFER_DONE)
			return 0;  /* transfer completed. */

		retry--;
		udelay(USPI_XFER_DELAY_US);
	}

	return -EBUSY;
}

static int usmc_spi_xfer_single_rx(struct usmc_spi_master *uspi,
				   struct spi_transfer *xfer,
				   unsigned int op_size,
				   unsigned int rx_offset)
{
	int ret, continue_cs;
	u8 *rx_buf, *recv_buf;
	unsigned int nleft, nrequest, payload_max;

	nleft = xfer->len - rx_offset;
	payload_max = FB_USMC_PAYLOAD_MAX - op_size;
	nrequest = (nleft < payload_max ? nleft : payload_max);
	continue_cs = ((nrequest == nleft) ? 0 : 1);

	ret = usmc_spi_xfer_start_hw(uspi, op_size + nrequest,
				     continue_cs);
	if (ret < 0) {
		dev_err(&uspi->pdev->dev,
			"failed to start spi hardware, error=%d\n", ret);
		return ret;
	}

	ret = usmc_spi_xfer_is_ready(uspi);
	if (ret < 0) {
		dev_err(&uspi->pdev->dev,
			"failed to check transfer status, error=%d\n", ret);
		return ret;
	}

	recv_buf = kmalloc(nrequest + op_size, GFP_KERNEL);
	if (recv_buf == NULL)
		return -ENOMEM;

	ret = usmc_spi_miso_read(uspi, recv_buf, nrequest + op_size);
	if (ret < 0) {
		dev_err(&uspi->pdev->dev,
			"failed to read data from MISO, error=%d\n", ret);
	} else {
		rx_buf = xfer->rx_buf;
		memcpy(&rx_buf[rx_offset], &recv_buf[op_size], nrequest);
	}

	kfree(recv_buf);
	return (ret < 0 ? ret : nrequest);
}

static int usmc_spi_xfer_rx(struct usmc_spi_master *uspi,
			    struct spi_transfer *xfer)
{
	int ret;
	unsigned int op_size;
	unsigned int ndata = 0;

	ret = usmc_spi_xfer_rx_prepare_op(uspi, xfer);
	if (ret < 0)
		return ret;

	op_size = uspi->read_op_size;
	uspi->read_op_size = 0;

	while (ndata < xfer->len) {
		fboss_usmc_xfer_lock(uspi->pdev);
		ret = usmc_spi_xfer_single_rx(uspi, xfer, op_size, ndata);
		fboss_usmc_xfer_unlock(uspi->pdev);
		if (ret < 0)
			return ret;

		/*
		 * flash op is only needed for the first spi transaction,
		 * so let's set it to 0.
		 */
		op_size = 0;

		ndata += ret;
	}

	return 0;
}

static int usmc_spi_xfer_one(struct spi_master *master,
			     struct spi_device *slave,
			     struct spi_transfer *xfer)
{
	int ret = 0;
	struct usmc_spi_master *uspi = spi_master_get_devdata(master);


	if (xfer->tx_buf != NULL) {
		u8 flash_op = ((u8 *)(xfer->tx_buf))[0];

		/*
		 * We cache the read command and combine it with following "rx"
		 * request.
		 */
		usmc_spi_cache_flash_op(uspi, xfer);
		if (!usmc_spi_is_read_op(flash_op)) {
			fboss_usmc_xfer_lock(uspi->pdev);
			ret = usmc_spi_xfer_tx(uspi, xfer);
			fboss_usmc_xfer_unlock(uspi->pdev);
			if (ret < 0)
				return ret;
		}
	}

	if (xfer->rx_buf != NULL)
		ret = usmc_spi_xfer_rx(uspi, xfer);

	return ret;
}

static int usmc_spi_mem_init(struct usmc_spi_master *uspi)
{
	struct resource *res;
	struct platform_device *pdev = uspi->pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get ctrl register resource\n");
		return -EINVAL;
	}
	uspi->mem_ctrl_reg = res->start + USPI_CTRL_REG_SIZE * uspi->id;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get data buffer resource\n");
		return -EINVAL;
	}
	uspi->mem_mosi = res->start + USPI_DATA_BUF_SIZE * uspi->id;
	uspi->mem_miso = uspi->mem_mosi + USPI_MOSI_BUF_SIZE;

	return 0;
}

static int usmc_spi_slave_init(struct usmc_spi_master *uspi)
{
	struct spi_master *master = uspi->master;
	struct spi_board_info slave_info = {
		.modalias = "spidev",
		.max_speed_hz = 25000000,
		.chip_select = 0,
		.bus_num = master->bus_num,
	};

	uspi->slave = spi_new_device(master, &slave_info);
	if (uspi->slave == NULL) {
		dev_err(&uspi->pdev->dev,
			"usmc-spi-%d: failed to create slave\n", uspi->id);
		return -ENXIO;
	}

	return 0;
}

static int usmc_spi_probe(struct platform_device *pdev)
{
	int ret;
	struct spi_master *master;
	struct usmc_spi_master *uspi;
	struct device *dev = &pdev->dev;
	const struct mfd_cell *cell = mfd_get_cell(pdev);

	master = spi_alloc_master(&pdev->dev, sizeof(struct usmc_spi_master));
	if (master == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);
	uspi = spi_master_get_devdata(master);
	uspi->id = cell->id;
	uspi->pdev = pdev;
	uspi->master = master;

	ret = usmc_spi_mem_init(uspi);
	if (ret < 0)
		return ret;

	master->num_chipselect = 1;
	master->dev.of_node = dev->of_node;
	master->transfer_one = usmc_spi_xfer_one;
	ret = spi_register_master(master);
	if (ret) {
		dev_err(dev, "failed to register spi_master[%u]\n", uspi->id);
		spi_master_put(master);
		return ret;
	}

	ret = usmc_spi_slave_init(uspi);
	if (ret < 0) {
		spi_master_put(master);
		return ret;
	}

	return 0;
}

static int usmc_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct usmc_spi_master *uspi = spi_master_get_devdata(master);

	spi_unregister_device(uspi->slave);
	spi_unregister_master(uspi->master);
	return 0;
}

static const struct of_device_id usmc_spi_match[] = {
	{ .compatible = "facebook,usmc-spi", },
	{},
};
MODULE_DEVICE_TABLE(of, usmc_spi_match);

static struct platform_driver usmc_spi_driver = {
	.probe = usmc_spi_probe,
	.remove = usmc_spi_remove,
	.driver = {
		.name = "usmc-spi",
		.of_match_table = usmc_spi_match,
	},
};
module_platform_driver(usmc_spi_driver);

MODULE_AUTHOR("Tao Ren <taoren@fb.com>");
MODULE_DESCRIPTION("Facebook USMC SPI Adapter Driver");
MODULE_LICENSE("GPL");
