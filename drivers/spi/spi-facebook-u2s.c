// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2020 Facebook Inc.

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/usb.h>

#define FBUS_NUM_SPI_BUSES	8

#define USB_BULK_MAX_SIZE	512

/*
 * max_payload_size: USB_BULK_MAX_SIZE - 2_byte_type - 2_byte_length -
 * 4_byte_address.
 */
#define TLV_PAYLOAD_MAX_SIZE	(USB_BULK_MAX_SIZE - 8)

/*
 * Supported TLV types.
 */
#define TLV_TYPE_READ_RES	0x8200
#define TLV_TYPE_READ_REQ	0x0200
#define TLV_TYPE_WRITE		0x0100

/*
 * USPI Controller register/memory base address and offset.
 */
#define USPI_CSR_REG_BASE	0x0
#define USPI_CSR_REG_SIZE	0x80

#define USPI_DATA_BUF_BASE	0x2000
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
#define USB_BULK_TIMEOUT_MS	2000
#define USPI_XFER_DELAY_US	10

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

/*
 * usb-bridge chardev ioctl commands
 */
#define UBRG_CMD_MEM_IO	0x101

/*
 * Structure to pass usb memory read/write command and data between user
 * and kernel space.
 */
struct ubrg_ioc_xfer {
	u32 addr;
	void *buf;
	unsigned int len;
	unsigned int flags;
#define UMEM_IOF_WRITE	0x1
};

/*
 * All the USB Bulk transactions should follow "fbus_tlv" format.
 */
struct fbus_tlv {
	u16 length;
	u16 type;
	u32 address;
	u8 data_buf[TLV_PAYLOAD_MAX_SIZE];
};

/*
 * Note: "tlv->length" doesn't cover the 4 byte header (length + type):
 * it defines the size of "data_buf" plus 4-byte "address". As a result:
 *
 * usb_pkt_size = tlv->length + sizeof(tlv->length) + sizeof(tlv->type)
 */
#define TLV_HDR_SIZE		4  /* type + length */
#define TLV_PKT_MIN_SIZE	8  /* type + length + address */
#define TLV_PKT_SIZE(_tlv)	((_tlv)->length + TLV_HDR_SIZE)
#define TLV_PAYLOAD_SIZE(_tlv)	((_tlv)->length - sizeof((_tlv)->address))

/*
 * SPI Master Control/Status Registers.
 */
struct fbus_spi_csr {
	u32 timing;
	u32 ctrl;
	u32 desc;
	u32 status;
};

/*
 * Structure for a SPI master.
 */
struct fbus_spi_master {
	u32 spi_id;
	struct spi_master *master;

	/*
	 * Only 1 slave device (flash) at present.
	 */
	struct spi_device *slave;

	/*
	 * Control/Status registers.
	 */
	struct fbus_spi_csr reg_csr;

	/*
	 * Register base address, initialized at probe time.
	 */
	unsigned long reg_csr_base;
	unsigned long reg_mosi_base;
	unsigned long reg_miso_base;

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

/*
 * List of endpoints (besides Control Endpoint #0) supported by the
 * USB-SPI Adapter.
 */
enum {
	USPI_BULK_IN = 0,
	USPI_BULK_OUT,
	USPI_EP_MAX,
};

/*
 * Structure for the entire FPGA device (usb-spi bridge).
 */
static struct {
	struct usb_device *usb_dev;
	struct usb_interface *usb_intf;

	/*
	 * "xfer_lock" is required to serilize usb device memory read and
	 * write requests.
	 */
	struct mutex xfer_lock;

	u8 ep_list[USPI_EP_MAX];
#define BULK_IN_PIPE	usb_rcvbulkpipe(fbus_bridge.usb_dev, \
					fbus_bridge.ep_list[USPI_BULK_IN])
#define BULK_OUT_PIPE	usb_sndbulkpipe(fbus_bridge.usb_dev, \
					fbus_bridge.ep_list[USPI_BULK_OUT])

	struct fbus_spi_master *spi_buses[FBUS_NUM_SPI_BUSES];

	struct miscdevice miscdev;
} fbus_bridge;

static int fbus_spi_setup(struct spi_device *slave)
{
	/* Nothing needed as of now */
	return 0;
}

static void fbus_spi_set_cs(struct spi_device *slave, bool level)
{
	/* Nothing needed as of now */
}

/*
 * Build a TLV-format USB packet.
 */
static int fbus_tlv_init(struct fbus_tlv *tlv,
		         u16 type,
			 u32 address,
			 const void *data_buf,
			 unsigned int data_len)
{
	if (data_len > sizeof(tlv->data_buf))
		return -E2BIG;

	tlv->type = type;
	tlv->address = address;
	if (type == TLV_TYPE_WRITE) {
		memcpy(tlv->data_buf, data_buf, data_len);
		tlv->length = sizeof(tlv->address) + data_len;
	} else if (type == TLV_TYPE_READ_REQ) {
		/*
		 * For read request, request length is specified by 4-byte
		 * integer (u32).
		 */
		u32 req_len = data_len;

		req_len += 4; /* include 4-byte address */

		memcpy(tlv->data_buf, &req_len, sizeof(req_len));
		tlv->length = sizeof(tlv->address) + sizeof(req_len);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int udev_mem_write(u32 addr, const void *buf, unsigned int size)
{
	struct fbus_tlv *tlv;
	int ret, req_len, actual_len;

	tlv = kmalloc(sizeof(*tlv), GFP_KERNEL);
	if (tlv == NULL)
		return -ENOMEM;

	ret = fbus_tlv_init(tlv, TLV_TYPE_WRITE, addr, buf, size);
	if (ret < 0)
		goto exit;

	mutex_lock(&fbus_bridge.xfer_lock);

	req_len = TLV_PKT_SIZE(tlv);
	ret = usb_bulk_msg(fbus_bridge.usb_dev, BULK_OUT_PIPE,
			   tlv, req_len, &actual_len, USB_BULK_TIMEOUT_MS);
	if ((ret == 0) && (actual_len < req_len)) {
		dev_err(&fbus_bridge.usb_intf->dev,
			"udevmem short write (type=0x%x, addr=0x%x): "
			"expect %u, actual %d\n",
			TLV_TYPE_WRITE, addr, req_len, actual_len);
		ret = -EBADMSG;
	}

	mutex_unlock(&fbus_bridge.xfer_lock);

exit:
	kfree(tlv);
	return ret;
}

static int udev_mem_read(u32 addr, void *buf, unsigned int size)
{
	struct fbus_tlv *tlv;
	int ret, req_len, actual_len;

	tlv = kmalloc(sizeof(*tlv), GFP_KERNEL);
	if (tlv == NULL)
		return -ENOMEM;

	/*
	 * Send read request to the FPGA.
	 */
	ret = fbus_tlv_init(tlv, TLV_TYPE_READ_REQ, addr, NULL, size);
	if (ret < 0)
		goto exit_mem;

	mutex_lock(&fbus_bridge.xfer_lock);

	req_len = TLV_PKT_SIZE(tlv);
	ret = usb_bulk_msg(fbus_bridge.usb_dev, BULK_OUT_PIPE,
			   tlv, req_len, &actual_len, USB_BULK_TIMEOUT_MS);
	if (ret < 0) {
		goto exit_lock;
	} else if (actual_len < req_len) {
		dev_err(&fbus_bridge.usb_intf->dev,
			"udevmem short write (type=0x%x, addr=0x%x): "
			"expect %u, actual %d\n",
			TLV_TYPE_READ_REQ, addr, req_len, actual_len);
		ret = -EBADMSG;
		goto exit_lock;
	}

	/*
	 * Then read from bulk-in pipe.
	 */
	memset(tlv, 0, TLV_PKT_MIN_SIZE); /* reset tlv head only. */
	req_len = TLV_PKT_MIN_SIZE + size;
	ret = usb_bulk_msg(fbus_bridge.usb_dev, BULK_IN_PIPE,
			   tlv, req_len, &actual_len, USB_BULK_TIMEOUT_MS);
	if (ret < 0) {
		goto exit_lock;
	} else if (actual_len < req_len) {
		dev_err(&fbus_bridge.usb_intf->dev,
			"udevmem short read (addr=0x%x): "
			"expect %u, actual %d\n", addr, req_len, actual_len);
		ret = -EBADMSG;
		goto exit_lock;
	}

	memcpy(buf, tlv->data_buf, size);

exit_lock:
	mutex_unlock(&fbus_bridge.xfer_lock);
exit_mem:
	kfree(tlv);
	return ret;
}

static void fbus_spicsr_set_xfer_len(struct fbus_spi_master *uspi,
				     unsigned int xfer_len)
{
	u32 mask = (USPI_XFER_LEN_MASK << USPI_XFER_LEN_OFFSET);

	uspi->reg_csr.desc &= ~mask;
	uspi->reg_csr.desc |= ((u32)(xfer_len & USPI_XFER_LEN_MASK) <<
				USPI_XFER_LEN_OFFSET);
}

static int fbus_spi_xfer_start_hw(struct fbus_spi_master *uspi,
				  unsigned int xfer_len,
				  int continue_cs)
{
	/*
	 * Update Control/Status registers.
	 */
	fbus_spicsr_set_xfer_len(uspi, xfer_len);
	uspi->reg_csr.desc |= USPI_XFER_START_HW;
	if (continue_cs)
		uspi->reg_csr.desc |= USPI_CONTINUOUS_CS;
	else
		uspi->reg_csr.desc &= ~USPI_CONTINUOUS_CS;

	/*
	 * Send the updated CSR register values to the device.
	 */
	return udev_mem_write(uspi->reg_csr_base, &uspi->reg_csr,
			      sizeof(uspi->reg_csr));
}

static int fbus_spi_mosi_write(struct fbus_spi_master *uspi,
			       const void *buf,
			       unsigned int len)
{
	return udev_mem_write(uspi->reg_mosi_base, buf, len);
}

static int fbus_spi_csr_read(struct fbus_spi_master *uspi,
			     void *buf,
			     unsigned int len)
{
	return udev_mem_read(uspi->reg_csr_base, buf, len);
}

static int fbus_spi_miso_read(struct fbus_spi_master *uspi,
			      void *buf,
			      unsigned int len)
{
	return udev_mem_read(uspi->reg_miso_base, buf, len);
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

static bool fbus_spi_is_read_op(u8 flash_op)
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

static int fbus_spi_cache_flash_op(struct fbus_spi_master *uspi,
				   struct spi_transfer *xfer)
{
	unsigned int len;
	u8 flash_op = ((u8*)xfer->tx_buf)[0];

	if (xfer->len <= sizeof(uspi->read_op_cache)) {
		len = xfer->len;
	} else {
		len = sizeof(uspi->read_op_cache);
	}

	memcpy(&uspi->read_op_cache, xfer->tx_buf, len);
	if (fbus_spi_is_read_op(flash_op))
		uspi->read_op_size = len;

	return 0;
}

static int fbus_spi_xfer_tx(struct fbus_spi_master *uspi,
			    struct spi_transfer *xfer)
{
	int ret;

	/*
	 * Step 1: upload command/data to mosi buffer.
	 */
	ret = fbus_spi_mosi_write(uspi, xfer->tx_buf, xfer->len);
	if (ret < 0) {
		dev_err(&uspi->master->dev,
			"failed to send %d bytes to MOSI, error=%d\n",
			xfer->len, ret);
		return ret;
	}

	/*
	 * Step 2: update control register to start transaction.
	 */
	ret = fbus_spi_xfer_start_hw(uspi, xfer->len, 0);
	if (ret < 0) {
		dev_err(&uspi->master->dev,
			"failed to start spi hardware, error=%d\n", ret);
		return ret;
	}

	/* add 40us delay for spi write*/
	udelay(4 * USPI_XFER_DELAY_US);

	return 0;
}

static int fbus_spi_xfer_rx_prepare_op(struct fbus_spi_master *uspi,
					struct spi_transfer *xfer)
{
	int ret;

	/*
	 * The read command should be cached, and let's verify it.
	 */
	if (uspi->read_op_size == 0) {
		dev_err(&uspi->master->dev,
			"no read_op command cached (len=%d): last op 0x%02x\n",
			xfer->len, uspi->read_op_cache.op);
		return -EINVAL;
	}

	/*
	 * fill mosi buffer with flash op
	 */
	ret = fbus_spi_mosi_write(uspi, &uspi->read_op_cache,
				  uspi->read_op_size);
	if (ret < 0)
		dev_err(&uspi->master->dev,
			"failed to send read_op 0x%02x (len=%u) to MOSI: "
			"error=%d\n",
			uspi->read_op_cache.op, uspi->read_op_size, ret);

	return ret;
}

static int fbus_spi_xfer_is_ready(struct fbus_spi_master *uspi)
{
	int ret;
	int retry = 2;
	struct fbus_spi_csr csr;

	while (retry >= 0) {
		ret = fbus_spi_csr_read(uspi, &csr, sizeof(csr));
		if (ret < 0)
			return ret;

		if (csr.status & USPI_STAT_XFER_DONE)
			return 0;  /* transfer completed. */

		retry--;
		udelay(USPI_XFER_DELAY_US);
	}

	return -EBUSY;
}

static int fbus_spi_xfer_single_rx(struct fbus_spi_master *uspi,
				   struct spi_transfer *xfer,
				   unsigned int op_size,
				   unsigned int rx_offset)
{
	int ret, continue_cs;
	u8 *rx_buf, *recv_buf;
	unsigned int nleft, nrequest, payload_max;

	nleft = xfer->len - rx_offset;
	payload_max = TLV_PAYLOAD_MAX_SIZE - op_size;
	nrequest = MIN(nleft, payload_max);
	continue_cs = ((nrequest == nleft) ? 0 : 1);

	ret = fbus_spi_xfer_start_hw(uspi, op_size + nrequest,
				     continue_cs);
	if (ret < 0) {
		dev_err(&uspi->master->dev,
			"failed to start spi hardware, error=%d\n", ret);
		return ret;
	}

	ret = fbus_spi_xfer_is_ready(uspi);
	if (ret < 0) {
		dev_err(&uspi->master->dev,
			"failed to check transfer status, error=%d\n", ret);
		return ret;
	}

	recv_buf = kmalloc(nrequest + op_size, GFP_KERNEL);
	if (recv_buf == NULL)
		return -ENOMEM;

	ret = fbus_spi_miso_read(uspi, recv_buf, nrequest + op_size);
	if (ret < 0) {
		dev_err(&uspi->master->dev,
			"failed to read data from MISO, error=%d\n", ret);
	} else {
		rx_buf = xfer->rx_buf;
		memcpy(&rx_buf[rx_offset], &recv_buf[op_size], nrequest);
	}

	kfree(recv_buf);
	return (ret < 0 ? ret : nrequest);
}

static int fbus_spi_xfer_rx(struct fbus_spi_master *uspi,
			    struct spi_transfer *xfer)
{
	int ret;
	unsigned int op_size;
	unsigned int ndata = 0;

	ret = fbus_spi_xfer_rx_prepare_op(uspi, xfer);
	if (ret < 0)
		return ret;

	op_size = uspi->read_op_size;
	uspi->read_op_size = 0;

	while (ndata < xfer->len) {
		ret = fbus_spi_xfer_single_rx(uspi, xfer, op_size, ndata);
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

static int fbus_spi_xfer_one(struct spi_master *master,
			     struct spi_device *slave,
			     struct spi_transfer *xfer)
{
	int ret = 0;
	struct fbus_spi_master *uspi = spi_master_get_devdata(master);

	if (xfer->tx_buf != NULL) {
		u8 flash_op = ((u8*)(xfer->tx_buf))[0];

		/*
		 * We cache the read command and combine it with following "rx"
		 * request.
		 */
		fbus_spi_cache_flash_op(uspi, xfer);
		if (!fbus_spi_is_read_op(flash_op)) {
			ret = fbus_spi_xfer_tx(uspi, xfer);
			if (ret < 0)
				return ret;
		}
	}

	if (xfer->rx_buf != NULL)
		ret = fbus_spi_xfer_rx(uspi, xfer);

	return ret;
}

static long ubrg_cdev_ioctl(struct file *file,
			    unsigned int cmd,
			    unsigned long arg)
{
	int ret = 0;
	u8 *xfer_buf;
	struct ubrg_ioc_xfer ioc_xfer;

	switch (cmd) {
	case UBRG_CMD_MEM_IO:
		if (copy_from_user(&ioc_xfer, (struct ubrg_ioc_xfer*)arg,
				   sizeof(ioc_xfer)))
			return -EFAULT;

		if ((ioc_xfer.buf == NULL) ||
		    (ioc_xfer.len > TLV_PAYLOAD_MAX_SIZE))
			return -EINVAL;

		xfer_buf = kmalloc(ioc_xfer.len, GFP_KERNEL);
		if (xfer_buf == NULL)
			return -ENOMEM;

		if (ioc_xfer.flags & UMEM_IOF_WRITE) {
			if (copy_from_user(xfer_buf, ioc_xfer.buf,
			    ioc_xfer.len)) {
				ret = -EFAULT;
				goto io_exit;
			}
			ret = udev_mem_write(ioc_xfer.addr, xfer_buf,
					     ioc_xfer.len);
		} else {
			ret = udev_mem_read(ioc_xfer.addr, xfer_buf,
					    ioc_xfer.len);
			if (ret < 0)
				goto io_exit;

			if (copy_to_user(ioc_xfer.buf, xfer_buf, ioc_xfer.len))
				ret = -EFAULT;
		}

io_exit:
		kfree(xfer_buf);
		break;

	default:
		return -ENOTTY;
	} /* switch */

	return ret;
}

static const struct file_operations ubrg_cdev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = ubrg_cdev_ioctl,
};

static void misc_dev_destroy(struct miscdevice *miscdev)
{
	misc_deregister(miscdev);
	kfree(miscdev->name);
}

static int misc_dev_init(struct device *parent, struct miscdevice *miscdev)
{
	int ret;

	miscdev->parent = parent;
	miscdev->fops =  &ubrg_cdev_fops;
	miscdev->minor = MISC_DYNAMIC_MINOR;
	miscdev->name = kasprintf(GFP_KERNEL, "usb-fpga");
	if (miscdev->name == NULL)
		return -ENOMEM;

	ret = misc_register(miscdev);
	if (ret < 0)
		kfree(miscdev->name);

	return ret;
}

static int fbus_spi_master_init(u32 id)
{
	int ret;
	struct spi_master *master;
	struct fbus_spi_master *uspi;
	struct device *parent = &fbus_bridge.usb_intf->dev;
	struct spi_board_info slave_info = {
		.modalias = "spidev",
		.max_speed_hz = 25000000,
		.chip_select = 0,
	};

	master = spi_alloc_master(parent, sizeof(struct fbus_spi_master));
	if (master == NULL) {
		dev_err(parent, "failed to allocate spi_master[%u]\n", id);
		return -ENOMEM;
	}
	master->num_chipselect = 1;
	master->setup = fbus_spi_setup;
	master->set_cs = fbus_spi_set_cs;
	master->transfer_one = fbus_spi_xfer_one;

	uspi = spi_master_get_devdata(master);

	uspi->spi_id = id;
	uspi->master = master;
	uspi->reg_csr_base = USPI_CSR_REG_BASE +
			     (USPI_CSR_REG_SIZE * id);
	uspi->reg_mosi_base = USPI_DATA_BUF_BASE +
			      (USPI_DATA_BUF_SIZE * id);
	uspi->reg_miso_base = uspi->reg_mosi_base + USPI_MOSI_BUF_SIZE;

	/*
	 * Initialize Control/Status registers.
	 */
	uspi->reg_csr.timing = (USPI_CLK_50MHZ |
				(2 << USPI_SAMPLE_DELAY_OFFSET));

	ret = spi_register_master(master);
	if (ret) {
		dev_err(parent, "failed to register spi_master[%u]\n", id);
		spi_master_put(master);
		return ret;
	}

	slave_info.bus_num = master->bus_num,
	uspi->slave = spi_new_device(master, &slave_info);
	if (uspi->slave == NULL) {
		dev_err(parent, "failed to create slave at spibus %u\n", id);
		spi_master_put(master);
		return -ENXIO;
	}

	fbus_bridge.spi_buses[id] = uspi;
	return 0;
}

static void fbus_spi_remove_all(void)
{
	u32 i;
	struct fbus_tlv *tlv;
	struct fbus_spi_master *uspi;

	for (i = 0; i < ARRAY_SIZE(fbus_bridge.spi_buses); i++) {
		uspi = fbus_bridge.spi_buses[i];

		if (uspi != NULL) {
			spi_unregister_device(uspi->slave);
			spi_unregister_master(uspi->master);
		}
	}
}

static int fbus_usb_probe(struct usb_interface *usb_intf,
			  const struct usb_device_id *usb_id)
{
	u32 i;
	int ret = 0;
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;

	fbus_bridge.usb_intf = usb_intf;
	fbus_bridge.usb_dev = usb_get_dev(interface_to_usbdev(usb_intf));

	/*
	 * The device supports 1 bulk-in and 1 bulk-out endpoints now.
	 */
	ret = usb_find_common_endpoints(usb_intf->cur_altsetting,
					&bulk_in, &bulk_out, NULL, NULL);
	if (ret) {
		dev_err(&usb_intf->dev,
			"Could not find bulk-in and/or bulk-out endpoints\n");
		return ret;
	}
	fbus_bridge.ep_list[USPI_BULK_IN] = bulk_in->bEndpointAddress;
	fbus_bridge.ep_list[USPI_BULK_OUT] = bulk_out->bEndpointAddress;

	mutex_init(&fbus_bridge.xfer_lock);

	for (i = 0; i < ARRAY_SIZE(fbus_bridge.spi_buses); i++) {
		ret = fbus_spi_master_init(i);
		if (ret < 0)
			goto error;
	}

	ret = misc_dev_init(&usb_intf->dev, &fbus_bridge.miscdev);
	if (ret < 0) {
		dev_err(&usb_intf->dev,
			"failed to initialize miscdevice, ret=%d\n", ret);
		goto error;
	}

	usb_set_intfdata(usb_intf, &fbus_bridge);

	dev_info(&usb_intf->dev, "ep_bulk_in: 0x%x, ep_bulk_out: 0x%x\n",
		 fbus_bridge.ep_list[USPI_BULK_IN],
		 fbus_bridge.ep_list[USPI_BULK_OUT]);

	return 0;

error:
	fbus_spi_remove_all();
	return ret;
}

static void fbus_usb_disconnect(struct usb_interface *usb_intf)
{
	misc_dev_destroy(&fbus_bridge.miscdev);
	fbus_spi_remove_all();
	usb_set_intfdata(usb_intf, NULL);
	usb_put_dev(fbus_bridge.usb_dev);
}

static const struct usb_device_id fbus_usb_table[] = {
	{USB_DEVICE(0x2ec6, 0x0100)}, /* XILINX */
	{USB_DEVICE(0x0100, 0x2EC6)}, /* XILINX */
	{}
};
MODULE_DEVICE_TABLE(usb, fbus_usb_table);

static struct usb_driver fbus_usb_driver = {
	.name		= "fb-usb-spi",
	.probe		= fbus_usb_probe,
	.disconnect	= fbus_usb_disconnect,
	.id_table	= fbus_usb_table,
};

module_usb_driver(fbus_usb_driver);

MODULE_AUTHOR("Tao Ren<taoren@fb.com>");
MODULE_DESCRIPTION("Facebook USB-SPI Adapter Driver");
MODULE_LICENSE("GPL");
