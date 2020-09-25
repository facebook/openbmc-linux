// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2020 Facebook Inc.

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/mtd/spi-nor.h>

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
	u16 read_op_size;

	/*
	 * All the 8 SPI masters share the same bulk-in pipe, and it is
	 * possible that the current thread may receive a packet which
	 * was requested by a different thread. For example:
	 *    1) spi1-thread issues READ_REQUEST for MISO buffer
	 *    2) spi5-thread issues READ_REQUEST for CSR registers
	 *    3) spi1-thread reads from bulk-in pipe
	 *    4) spi5-thread reads from bulk-in pipe
	 * In above example, spi1-thread may receive the packet requested
	 * by spi5-thread. If it happens, we just "send" the packet to
	 * spi5.
	 * "lock" is required to prevent race when bulk-in packets are
	 * cached/fetched.
	 */
	spinlock_t lock;
	struct fbus_tlv *csr_read_cache;
	struct fbus_tlv *miso_read_cache;
};

/*
 * Helper macros to set/get CSR read cached.
 */
#define USPI_CSR_CACHE_SET(_uspi, _tlv) do {				    \
	spin_lock(&(_uspi)->lock);					    \
	if ((_uspi)->csr_read_cache != NULL) {				    \
		dev_warn(&(_uspi)->master->dev, "overriding csr cache\n");  \
		kfree((_uspi)->csr_read_cache);				    \
	}								    \
	(_uspi)->csr_read_cache = (_tlv);				    \
	spin_unlock(&(_uspi)->lock);					    \
} while (0)

#define USPI_CSR_CACHE_GET(_uspi, _tlv) do {				    \
	spin_lock(&(_uspi)->lock);					    \
	(_tlv) = (_uspi)->csr_read_cache;				    \
	(_uspi)->csr_read_cache = NULL;					    \
	spin_unlock(&(_uspi)->lock);					    \
} while (0)

/*
 * Helper macros to set/get MISO read cached.
 */
#define USPI_MISO_CACHE_SET(_uspi, _tlv) do {				    \
	spin_lock(&(_uspi)->lock);					    \
	if ((_uspi)->miso_read_cache != NULL) {				    \
		dev_warn(&(_uspi)->master->dev, "overriding miso cache\n"); \
		kfree((_uspi)->miso_read_cache);			    \
	}								    \
	(_uspi)->miso_read_cache = (_tlv);				    \
	spin_unlock(&(_uspi)->lock);					    \
} while (0)

#define USPI_MISO_CACHE_GET(_uspi, _tlv) do {				    \
	spin_lock(&(_uspi)->lock);					    \
	(_tlv) = (_uspi)->miso_read_cache;				    \
	(_uspi)->miso_read_cache = NULL;				    \
	spin_unlock(&(_uspi)->lock);					    \
} while (0)

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

	u8 ep_list[USPI_EP_MAX];
#define BULK_IN_PIPE	usb_rcvbulkpipe(fbus_bridge.usb_dev, \
					fbus_bridge.ep_list[USPI_BULK_IN])
#define BULK_OUT_PIPE	usb_sndbulkpipe(fbus_bridge.usb_dev, \
					fbus_bridge.ep_list[USPI_BULK_OUT])

	struct fbus_spi_master *spi_buses[FBUS_NUM_SPI_BUSES];
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
			 u16 data_len)
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

/*
 * Send a TLV-format packet to USB bulk-out pipe.
 * Return value:
 *   - 0 for success, or negative error code on failures.
 *     Actual transaction length is returned in <actual_len> argument.
 */
static int fbus_usb_bulk_out_tlv(struct fbus_tlv *tlv)
{
	int ret;
	int actual_len;

	ret = usb_bulk_msg(fbus_bridge.usb_dev, BULK_OUT_PIPE,
			   tlv, TLV_PKT_SIZE(tlv), &actual_len,
			   USB_BULK_TIMEOUT_MS);
	if (ret < 0)
		return ret;

	if (actual_len < TLV_PKT_SIZE(tlv))
		return -EBADMSG;

	return 0;
}

/*
 * Read a TLV-format packet from USB bulk-in pipe.
 * Return value:
 *   - 0 for success, or negative error code on failures.
 *     Actual transaction length is returned in <actual_len> argument.
 */
static int fbus_usb_bulk_in_tlv(struct fbus_tlv *tlv)
{
	int ret = 0;
	int actual_len;

	/*
	 * Read TLV packet header.
	 */
	ret = usb_bulk_msg(fbus_bridge.usb_dev, BULK_IN_PIPE,
			   tlv, sizeof(*tlv), &actual_len,
			   USB_BULK_TIMEOUT_MS);
	if ((ret == 0) && (actual_len < TLV_PKT_MIN_SIZE))
		ret = -EBADMSG;

	return ret;
}

/*
 * Store the bulk-in packet in spi master's read_cache depending on
 * "tlv->address".
 * Return:
 *   - 0 (zero) if the packet is cached; otherwise -EINVAL.
 */
static int fbus_bulk_in_tlv_dispatch(struct fbus_tlv *tlv)
{
	u8 id;
	u32 addr = tlv->address;
	struct fbus_spi_master *uspi;
	u32 csr_end = (FBUS_NUM_SPI_BUSES * USPI_CSR_REG_SIZE) +
			USPI_CSR_REG_BASE;
	u32 data_buf_end = (FBUS_NUM_SPI_BUSES * USPI_DATA_BUF_SIZE) +
			USPI_DATA_BUF_BASE;

	if ((addr >= USPI_CSR_REG_BASE) && (addr < csr_end)) {
		/*
		 * SPI CSR register space.
		 */
		id = (addr - USPI_CSR_REG_BASE) / USPI_CSR_REG_SIZE;
		uspi = fbus_bridge.spi_buses[id];

		USPI_CSR_CACHE_SET(uspi, tlv);
	} else if ((addr >= USPI_DATA_BUF_BASE) && (addr < data_buf_end)) {
		/*
		 * SPI Data buffer space. We never read MOSI buffer thus
		 * we assume it's always MISO buffer.
		 */
		id = (addr - USPI_DATA_BUF_BASE) / USPI_DATA_BUF_SIZE;
		uspi = fbus_bridge.spi_buses[id];
		USPI_MISO_CACHE_SET(uspi, tlv);
	} else {
		return -EINVAL; /* Invalid address */
	}

	return 0;
}

/*
 * Read TLV-format packets from bulk-in pipe until either of the following
 * condition is met:
 *   - received a TLV whose tlv->address is equal to supplied <addr>
 *   - error happens.
 */
static struct fbus_tlv* fbus_usb_bulk_in_drain(unsigned long addr)
{
	int ret = 0;
	struct fbus_tlv *tlv = NULL;

	while (1) {
		if (tlv == NULL) {
			tlv = kmalloc(sizeof(*tlv), GFP_KERNEL);
			if (tlv == NULL)
				break;
		}

		ret = fbus_usb_bulk_in_tlv(tlv);
		if (ret < 0) {
			dev_err(&fbus_bridge.usb_intf->dev,
				"failed to read bulk-in pipe, error=%d\n",
				ret);
			kfree(tlv);
			tlv = NULL;
			break;
		}

		if (tlv->address == addr)
			break;  /* found matched tlv packet */

		ret = fbus_bulk_in_tlv_dispatch(tlv);
		if (ret == 0)
			tlv = NULL;	/* tlv being cached */
		else
			dev_warn(&fbus_bridge.usb_intf->dev,
				"drop bulk-in packet: unknown address 0x%x)\n",
				tlv->address);
	}

	return tlv;
}

static void fbus_spicsr_set_xfer_len(struct fbus_spi_master *uspi,
				     u16 xfer_len)
{
	u32 mask = (USPI_XFER_LEN_MASK << USPI_XFER_LEN_OFFSET);

	uspi->reg_csr.desc &= ~mask;
	uspi->reg_csr.desc |= ((u32)(xfer_len & USPI_XFER_LEN_MASK) <<
				USPI_XFER_LEN_OFFSET);
}

static int fbus_spi_xfer_start_hw(struct fbus_spi_master *uspi,
				  u16 xfer_len,
				  int continue_cs)
{
	int ret = 0;
	struct fbus_tlv *tlv;

	/*
	 * Update Control/Status registers.
	 */
	fbus_spicsr_set_xfer_len(uspi, xfer_len);
	uspi->reg_csr.desc |= USPI_XFER_START_HW;
	if (continue_cs)
		uspi->reg_csr.desc |= USPI_CONTINUOUS_CS;
	else
		uspi->reg_csr.desc &= ~USPI_CONTINUOUS_CS;

	tlv = kmalloc(sizeof(*tlv), GFP_KERNEL);
	if (tlv == NULL)
		return -ENOMEM;

	ret = fbus_tlv_init(tlv, TLV_TYPE_WRITE, uspi->reg_csr_base,
			    &uspi->reg_csr, sizeof(uspi->reg_csr));
	if (ret < 0)
		goto exit;

	ret = fbus_usb_bulk_out_tlv(tlv);

exit:
	kfree(tlv);
	return ret;
}

static int fbus_spi_mosi_write(struct fbus_spi_master *uspi,
			       const void *buf,
			       unsigned len)
{
	int ret = 0;
	struct fbus_tlv *tlv;

	tlv = kmalloc(sizeof(*tlv), GFP_KERNEL);
	if (tlv == NULL)
		return -ENOMEM;

	ret = fbus_tlv_init(tlv, TLV_TYPE_WRITE, uspi->reg_mosi_base,
			    buf, len);
	if (ret < 0)
		goto exit;

	ret = fbus_usb_bulk_out_tlv(tlv);

exit:
	kfree(tlv);
	return ret;
}

static int fbus_spi_issue_read_request(struct fbus_spi_master *uspi,
				       u16 address,
				       u16 size)
{
	int ret = 0;
	struct fbus_tlv *tlv = NULL;

	/*
	 * First, we need to send request to the USB-SPI adapter.
	 */
	tlv = kmalloc(sizeof(*tlv), GFP_KERNEL);
	if (tlv == NULL)
		return -ENOMEM;

	ret = fbus_tlv_init(tlv, TLV_TYPE_READ_REQ, address, NULL, size);
	if (ret < 0)
		goto exit;

	ret = fbus_usb_bulk_out_tlv(tlv);
	if (ret < 0)
		dev_err(&uspi->master->dev,
			"failed to send read request, error=%d\n", ret);

exit:
	kfree(tlv);
	return ret;
}

static int fbus_spi_csr_read(struct fbus_spi_master *uspi,
			     u16 req_size,
			     struct fbus_tlv **out_tlv)
{
	int ret;
	struct fbus_tlv *tlv = NULL;

	ret = fbus_spi_issue_read_request(uspi, uspi->reg_csr_base, req_size);
	if (ret < 0) {
		dev_err(&uspi->master->dev,
			"failed to send read_csr request: error=%d\n", ret);
		return ret;
	}

	/*
	 * First, let's check if CSR register is already cached.
	 */
	USPI_CSR_CACHE_GET(uspi, tlv);

	/*
	 * Read from bulk-in pipe if the TLV is not cached.
	 */
	if (tlv == NULL)
		tlv = fbus_usb_bulk_in_drain(uspi->reg_csr_base);

	/*
	 * It's possible the bulk-in packet was fetched by another thread:
	 * let's release CPU and try to read from cache again.
	 */
	if (tlv == NULL) {
		yield();
		USPI_CSR_CACHE_GET(uspi, tlv);
		if (tlv == NULL)
			return -ENODATA;
	}

	*out_tlv = tlv;
	return 0;
}

static int fbus_spi_miso_read(struct fbus_spi_master *uspi,
			      u16 req_size,
			      struct fbus_tlv **out_tlv)
{
	int ret;
	struct fbus_tlv *tlv = NULL;

	ret = fbus_spi_issue_read_request(uspi, uspi->reg_miso_base,
					  req_size);
	if (ret < 0)
		return ret;

	/*
	 * First, let's check if MISO buffer is cached.
	 */
	USPI_MISO_CACHE_GET(uspi, tlv);

	/*
	 * Read from bulk-in pipe if the TLV is not cached.
	 */
	if (tlv == NULL)
		tlv = fbus_usb_bulk_in_drain(uspi->reg_miso_base);

	/*
	 * It's possible the bulk-in packet was fetched by another thread:
	 * let's release CPU and try to read from cache again.
	 */
	if (tlv == NULL) {
		yield();
		USPI_MISO_CACHE_GET(uspi, tlv);
		if (tlv == NULL)
			return -ENODATA;
	}

	*out_tlv = tlv;
	return 0;
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
	u16 len;
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
	int retry = 3;
	u16 data_len;
	u32 status;
	struct fbus_tlv *tlv;
	struct fbus_spi_csr *csr;

	while (retry >= 0) {
		ret = fbus_spi_csr_read(uspi, sizeof(*csr), &tlv);
		if (ret < 0)
			return ret;

		data_len = TLV_PAYLOAD_SIZE(tlv);
		if (data_len < sizeof(*csr)) {
			dev_err(&uspi->master->dev,
				"reg_csr short read: expect %d, actual %d\n",
				sizeof(*csr), data_len);
			kfree(tlv);
			return -EBADMSG;
		}

		csr = (struct fbus_spi_csr *)tlv->data_buf;
		status = csr->status;
		kfree(tlv);

		if (status & USPI_STAT_XFER_DONE)
			return 0;  /* transfer completed. */

		retry--;
		udelay(USPI_XFER_DELAY_US);
	}

	return -EBUSY;
}

static int fbus_spi_xfer_single_rx(struct fbus_spi_master *uspi,
				   struct spi_transfer *xfer,
				   u16 op_offset,
				   unsigned int rx_offset)
{
	u16 payload_max;
	int ret, continue_cs;
	struct fbus_tlv *tlv;
	unsigned int nleft, nrequest;
	u8 *rx_buf = xfer->rx_buf;

	nleft = xfer->len - rx_offset;
	payload_max = TLV_PAYLOAD_MAX_SIZE - op_offset;
	nrequest = MIN(nleft, payload_max);
	continue_cs = ((nrequest == nleft) ? 0 : 1);

	ret = fbus_spi_xfer_start_hw(uspi, op_offset + nrequest,
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

	ret = fbus_spi_miso_read(uspi, nrequest + op_offset, &tlv);
	if (ret < 0) {
		dev_err(&uspi->master->dev,
			"failed to read data from MISO, error=%d\n", ret);
		return ret;
	}

	if (TLV_PAYLOAD_SIZE(tlv) < (op_offset + nrequest)) {
		dev_err(&uspi->master->dev,
			"miso buffer short read, expect %d, actual %d\n",
			op_offset + nrequest, TLV_PAYLOAD_SIZE(tlv));
		kfree(tlv);
		return -EBADMSG;
	}

	memcpy(&rx_buf[rx_offset], &tlv->data_buf[op_offset], nrequest);
	kfree(tlv);
	return nrequest;
}

static int fbus_spi_xfer_rx(struct fbus_spi_master *uspi,
			    struct spi_transfer *xfer)
{
	int ret;
	u16 op_offset;
	unsigned int ndata = 0;

	ret = fbus_spi_xfer_rx_prepare_op(uspi, xfer);
	if (ret < 0)
		return ret;

	op_offset = uspi->read_op_size;
	uspi->read_op_size = 0;

	while (ndata < xfer->len) {
		ret = fbus_spi_xfer_single_rx(uspi, xfer, op_offset, ndata);
		if (ret < 0)
			return ret;

		/*
		 * flash op is only needed for the first spi transaction,
		 * so let's set it to 0.
		 */
		op_offset = 0;

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
	spin_lock_init(&uspi->lock);
	uspi->reg_csr_base = USPI_CSR_REG_BASE +
			     (USPI_CSR_REG_SIZE * id);
	uspi->reg_mosi_base = USPI_DATA_BUF_BASE +
			      (USPI_DATA_BUF_SIZE * id);
	uspi->reg_miso_base = uspi->reg_mosi_base + USPI_MOSI_BUF_SIZE;

	/*
	 * Initialize Control/Status registers.
	 */
	uspi->reg_csr.timing = (USPI_CLK_25MHZ |
				(1 << USPI_SAMPLE_DELAY_OFFSET));

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

	for (i = 0; i < ARRAY_SIZE(fbus_bridge.spi_buses); i++) {
		ret = fbus_spi_master_init(i);
		if (ret < 0)
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
