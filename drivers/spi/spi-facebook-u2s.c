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

#include <linux/debugfs.h>

#define FBUS_NUM_SPI_BUSES	8

#define USB_BULK_MAX_SIZE	512
#define USB_CTRL_MAX_SIZE	64
#define READ_MAX_LEN		256

/* TLV define  start */
/* Fuji Base FPGA SPI memory Offset */
#define OFFSET_SPI_BASE_MOSI			0x2000
#define OFFSET_SPI_BASE_MISO			0x2200
#define OFFSET_SPI_MASTER_DATA_SIZE		0x400
#define OFFSET_SPI_BASE_CONTROLLER		0x0
#define OFFSET_SPI_CONTROLLER_DATA_SIZE	0x80

/* Fuji Base SPI TLV head offset */
#define OFFSET_SPI_HEAD				  0x4

/* Fuji Base SPI TLV type */
#define TLV_TYPE_READ_RES	0x8200
#define TLV_TYPE_READ_REQ	0x0200
#define TLV_TYPE_WRITE		0x0100
/* TLV define  end */

/* USB_BULK_MAX_SIZE - type - length - address */
#define TLV_DATA_MAX_SIZE	(USB_BULK_MAX_SIZE - 8)

/* milliseconds , USB data transfer time out */
#define DATA_TIMEOUT				2000
#define SPI_TRANSFER_DELAY			10	/* ? unit of time */
#define SPI_TRANSFER_DELAY_COUNT	10

/* FLASH command*/
#define FLASH_COMMAND_READ  0x3  /* read flash */
#define FLASH_COMMAND_PP	0x2  /* page program flash */
#define FLASH_COMMAND_SE	0x20 /* sector erase flash */
#define FLASH_COMMAND_BE	0x52 /* block erase flash */
#define FLASH_COMMAND_WREN  0x6  /* write enable flash */
#define FLASH_COMMAND_RDSR  0x5  /* read status register flash */
#define FLASH_COMMAND_RDID  0x9f /* read identification */

/* FPGA specific treatment start*/
/* fpga spi controller register force CS on */
#define FBU2S_SPI_CTR_CONTINUE_CS_ON			0xa0
/* fpga spi controller register force CS as normal */
#define FBU2S_SPI_CTR_CONTINUE_CS_OFF			0x80
/* the offset of SPI data access length in the spi controller register */
#define FBU2S_SPI_CTR_DATA_1ST_LEN_FIELD_OFFSET	9
#define FBU2S_SPI_CTR_DATA_2ND_LEN_FIELD_OFFSET	10
/* the offset of continue CS function in the spi controller register */
#define FBU2S_SPI_CTR_DATA_CS_FIELD_OFFSET		11
/* the offset of spi status function in the spi controller register */
#define FBU2S_SPI_CTR_SPI_STATUS_OFFSET			20
/* For 1st read, FPGA will return size_mosi+size_miso byte of data.
 * The first read size_mosi(4byte) byte of data should be discarded.
 */
#define FBU2S_SPI_MISO_DATA_1ST_READ_OFFSET		12
/* For other  read, FPGA will return size_miso byte of data.
 * No data should be discarded.
 */
#define FBU2S_SPI_MISO_DATA_OTHER_READ_OFFSET	8
/* For other operation, the first byte is dummy byte returned from Flash */
#define FBU2S_SPI_MISO_DATA_OP_OFFSET			9
#define FBU2S_SPI_CTR_DATA_LEN_FIELD_NUM		2
#define FBU2S_SPI_CTR_DATA_LEN					16
/* FBU2S_SPI_CTR_BUFF_LEN =  FBU2S_SPI_CTR_DATA_LEN + head + address */
#define FBU2S_SPI_CTR_BUFF_LEN					0x18
/* spi read request len is fixed 4 from IOB FPGA spec */
#define FBU2S_SPI_READ_REQUEST_LEN				0x4
/* request 264 bytes to driver,
 * 256 bytes data + 4bytes TLV head + 4 bytes address
 */
#define FBU2S_SPI_DATA_READ_REQUEST_LEN			264
/* MISO buffer will return data + size_miso byte of
 * data(discard) + 4bytes TLV head + 4 bytes address
 */
#define FBU2S_SPI_DATA_NORMAL_READ_LEN			268

/* spi read data access length is 0x100 */
#define FBU2S_SPI_DATA_READ_ACCESS_LEN			0x100
/* spi write enable data access length is 0x1 */
#define FBU2S_SPI_DATA_WREN_ACCESS_LEN			0x1
#define FBU2S_SPI_DATA_ERASE_ACCESS_LEN			0x4
/* eg. spi write data access length is 0x100 + 1byte command + 3byte address */
#define FBU2S_SPI_DATA_NORMAL_ACCESS_LEN		0x104

/* FPGA specific treatment end */

/*
 * Structure for a single SPI bus.
 */
struct fbu2s_spi_bus {
	u32 spi_id;
	struct spi_master *master;
	struct spi_device *spi;
};

/*
 * Structure for the TLV
 */
struct fbu2s_tlv {
	u16	length;
	u16	type;
	u32	address;
	u8	data_buff[TLV_DATA_MAX_SIZE];
};
static struct fbu2s_tlv tlv_pkg = {0};

/* SPI controller field, SPI Timing Profile(1st 4 bytes),
 * SPI Controller(2nd 4bytes), SPI Descriptor(specific)
 * 0x1 for 25MHz SCK with 100MHz mclk, 0x10 for 10ns delay
 */
#define SPI_FREQ_25MHZ	1
#define DELAY_10NS		0x10
static unsigned char spi_ctr_data[FBU2S_SPI_CTR_DATA_LEN] = {
	SPI_FREQ_25MHZ, DELAY_10NS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* endpoint type define for USB transfer according to diffreent spi operation,
 * TODO: add 8 bulk-in endpoints
 */
enum {
	USPI_BULK_IN = 0,
	USPI_BULK_OUT,
	USPI_EP_MAX,
};

/*
 * Structure for the entire FPGA device (usb-spi bridge).
 */
static struct fbu2s_usb {
	struct usb_device *usb_dev;
	struct usb_interface *usb_intf;

	u8 ep_list[USPI_EP_MAX];
#define BULK_IN_PIPE	usb_sndbulkpipe(fbu2s_bridge.usb_dev, \
					fbu2s_bridge.ep_list[USPI_BULK_IN])
#define BULK_OUT_PIPE	usb_sndbulkpipe(fbu2s_bridge.usb_dev, \
					fbu2s_bridge.ep_list[USPI_BULK_IN])

	struct fbu2s_spi_bus *spi_buses[FBUS_NUM_SPI_BUSES];
} fbu2s_bridge;

/*
 * FIXME this function needs to be implemented.
 */
static int fbu2s_spi_setup(struct spi_device *slave)
{
	return 0;
}

/*
 * FIXME this function needs to be implemented.
 */
static void fbu2s_spi_set_cs(struct spi_device *slave, bool level)
{
	//#CS is controlled by continue CS field of SPI controller register.
}

static int fbu2s_tlv_parse(u16 tlv_type, unsigned long address,
			   const void *data_buf, u16 data_len)
{
	struct device *parent = &(fbu2s_bridge.usb_intf->dev);
	int read_len = FBU2S_SPI_READ_REQUEST_LEN;
	int temp;

	if (data_len > TLV_DATA_MAX_SIZE) {
		dev_info(parent, " tlv parse abnormal para.\n");
		return -1;
	}
	/**** pack TLV ****/

	/* insert tlv type */
	tlv_pkg.type = tlv_type;

	if (tlv_type == TLV_TYPE_WRITE ||
	   tlv_type == TLV_TYPE_READ_RES ||
	   tlv_type == TLV_TYPE_READ_REQ) {
		/* insert tlv address */
		tlv_pkg.address = address;

		/* insert tlv length */
		if (tlv_type == TLV_TYPE_READ_REQ)
			tlv_pkg.length = sizeof(tlv_pkg.address) + read_len;
		else
			tlv_pkg.length = sizeof(tlv_pkg.address) + data_len;

		/* insert tlv datas */
		memset(tlv_pkg.data_buff, 0, TLV_DATA_MAX_SIZE);
		if (tlv_type == TLV_TYPE_WRITE) {
			memcpy(tlv_pkg.data_buff, data_buf, data_len);
			temp = tlv_pkg.data_buff[0];
			if (temp == FLASH_COMMAND_WREN) {
				/* For spi write enable command,
				 * the length is awalys 0x1+ address length
				 */
				tlv_pkg.length = sizeof(tlv_pkg.address) + 1;
				memcpy(tlv_pkg.data_buff, data_buf, 1);
			}
		} else if (tlv_type == TLV_TYPE_READ_REQ) {
			temp = data_len;
			memcpy(tlv_pkg.data_buff, &temp, read_len);
		}
	} else {
		dev_info(parent, "abnormal tlv_type.\n");
		return -1;
	}

	return 0;
}

int fbu2s_usb_transfer(int endp, void *buf, int req_len, int *actual_len)
{
	int ret = 0;

	switch (endp) {
	case USPI_BULK_IN:
		ret = usb_bulk_msg(fbu2s_bridge.usb_dev, BULK_IN_PIPE,
				   buf, req_len, actual_len, DATA_TIMEOUT);
		break;

	case USPI_BULK_OUT:
		ret = usb_bulk_msg(fbu2s_bridge.usb_dev, BULK_OUT_PIPE,
				   buf, req_len, actual_len, DATA_TIMEOUT);
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int fbu2s_spi_get_ctr_buff(int cmd_type,
				  void *spi_ctr_buff,
				  int spi_ctr_buff_len,
				  unsigned long ctr_addr,
				  unsigned char spi_data_continue_cs,
				  int first_read_transfer_flag)
{
	int ret;
	unsigned int spi_data_access_len = 0;

	if ((cmd_type == FLASH_COMMAND_SE) ||
	   (cmd_type == FLASH_COMMAND_BE) ||
	   (cmd_type == FLASH_COMMAND_RDSR))
		spi_data_access_len = FBU2S_SPI_DATA_ERASE_ACCESS_LEN;
	else if (cmd_type == FLASH_COMMAND_WREN)
		spi_data_access_len = FBU2S_SPI_DATA_WREN_ACCESS_LEN;
	else if (cmd_type == FLASH_COMMAND_READ)
		spi_data_access_len = FBU2S_SPI_DATA_READ_ACCESS_LEN;
	else
		spi_data_access_len = FBU2S_SPI_DATA_NORMAL_ACCESS_LEN;

	if (first_read_transfer_flag)
		spi_data_access_len = FBU2S_SPI_DATA_NORMAL_ACCESS_LEN;

	memcpy(spi_ctr_data + FBU2S_SPI_CTR_DATA_1ST_LEN_FIELD_OFFSET,
		   &spi_data_access_len, FBU2S_SPI_CTR_DATA_LEN_FIELD_NUM);
	spi_ctr_data[FBU2S_SPI_CTR_DATA_CS_FIELD_OFFSET] = spi_data_continue_cs;

	ret = fbu2s_tlv_parse(TLV_TYPE_WRITE, ctr_addr,
				(void *)spi_ctr_data, FBU2S_SPI_CTR_DATA_LEN);
	if (ret)
		return -1;

	memcpy(spi_ctr_buff, (void *)&tlv_pkg, spi_ctr_buff_len);

	return ret;
}

static int fbu2s_spi_xfer_tx(struct spi_transfer *xfer, unsigned int id)
{
	int ret = 0;
	int temp = 0;
	int mosi_buff_len;
	int spi_ctr_buff_len;
	unsigned long addr, read_addr, ctr_addr;
	uint8_t *spi_ctr_buff = NULL;
	uint8_t *mosi_buff    = NULL;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	addr = OFFSET_SPI_BASE_MOSI + (OFFSET_SPI_MASTER_DATA_SIZE * id);
	read_addr = OFFSET_SPI_BASE_MISO + (OFFSET_SPI_MASTER_DATA_SIZE * id);
	ctr_addr = OFFSET_SPI_BASE_CONTROLLER +
			   (OFFSET_SPI_CONTROLLER_DATA_SIZE * id);

	ret = fbu2s_tlv_parse(TLV_TYPE_WRITE, addr, xfer->tx_buf, xfer->len);
	temp = tlv_pkg.data_buff[0];
	/* 4K read for continue CS. read command is sent when rx_buff is
	 * received, otherwise sent when tx_buff received.
	 */
	if (temp == FLASH_COMMAND_READ)
		return 0;

	mosi_buff_len = tlv_pkg.length + OFFSET_SPI_HEAD;

	mosi_buff = kmalloc(mosi_buff_len, GFP_KERNEL);
	if (mosi_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(mosi_buff, 0, mosi_buff_len);
	memcpy(mosi_buff, &tlv_pkg, mosi_buff_len);

	ret = fbu2s_usb_transfer(USPI_BULK_OUT, mosi_buff, mosi_buff_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		goto exit;
	}

	//write spi controller:(hard code rigth now)
	spi_ctr_buff_len = FBU2S_SPI_CTR_BUFF_LEN;
	spi_ctr_buff = kmalloc(spi_ctr_buff_len, GFP_KERNEL);
	if (spi_ctr_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(spi_ctr_buff, 0, spi_ctr_buff_len);
	ret = fbu2s_spi_get_ctr_buff(temp, spi_ctr_buff, spi_ctr_buff_len,
			ctr_addr, FBU2S_SPI_CTR_CONTINUE_CS_OFF, 0);
	if (ret) {
		dev_info(parent, "Send spi ctr register fail!\n");
		goto exit;
	}

	ret = fbu2s_usb_transfer(USPI_BULK_OUT, spi_ctr_buff,
				 spi_ctr_buff_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		goto exit;
	}

	/* add 40us delay for spi write*/
	udelay(4 * SPI_TRANSFER_DELAY);

exit:
	if (mosi_buff != NULL) {
		kfree(mosi_buff);
	}
	if (spi_ctr_buff != NULL) {
		kfree(spi_ctr_buff);
	}
	return ret;
}

static int fbu2s_spi_xfer_rx_read(struct spi_transfer *xfer, unsigned int id)
{
	int ret = 0;
	int temp = 0;
	int mosi_buff_len;
	int spi_ctr_buff_len;
	unsigned long addr, read_addr, ctr_addr;
	uint8_t *spi_ctr_buff = NULL;
	uint8_t *mosi_buff    = NULL;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	addr = OFFSET_SPI_BASE_MOSI + (OFFSET_SPI_MASTER_DATA_SIZE * id);
	read_addr = OFFSET_SPI_BASE_MISO + (OFFSET_SPI_MASTER_DATA_SIZE * id);
	ctr_addr = OFFSET_SPI_BASE_CONTROLLER +
			   (OFFSET_SPI_CONTROLLER_DATA_SIZE * id);

	mosi_buff_len = tlv_pkg.length + OFFSET_SPI_HEAD;
	mosi_buff = kmalloc(mosi_buff_len, GFP_KERNEL);
	if (mosi_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(mosi_buff, 0, mosi_buff_len);
	memcpy(mosi_buff, &tlv_pkg, mosi_buff_len);

	ret = fbu2s_usb_transfer(USPI_BULK_OUT, mosi_buff,
				mosi_buff_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		goto exit;
	}

	//write spi controller:(hard code rigth now)
	spi_ctr_buff_len = FBU2S_SPI_CTR_BUFF_LEN;
	spi_ctr_buff = kmalloc(spi_ctr_buff_len, GFP_KERNEL);
	if (spi_ctr_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(spi_ctr_buff, 0, spi_ctr_buff_len);
	ret = fbu2s_spi_get_ctr_buff(temp, spi_ctr_buff,
				spi_ctr_buff_len, ctr_addr,
				FBU2S_SPI_CTR_CONTINUE_CS_OFF, 0);
	if (ret) {
		dev_info(parent, "Send spi ctr register fail!\n");
		goto exit;
	}

	ret = fbu2s_usb_transfer(USPI_BULK_OUT, spi_ctr_buff,
				spi_ctr_buff_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
	}

exit:
	if (mosi_buff != NULL) {
		kfree(mosi_buff);
	}
	if (spi_ctr_buff != NULL) {
		kfree(spi_ctr_buff);
	}
	return ret;
}

static int fbu2s_spi_xfer_rx_status_check(struct spi_transfer *xfer,
					  unsigned int id)
{
	int ret = 0;
	int j = 0;
	int read_len;
	unsigned long ctr_addr;
	unsigned int count;
	uint8_t *ctr_buff            = NULL;
	uint8_t *read_buf            = NULL;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	ctr_addr = OFFSET_SPI_BASE_CONTROLLER +
			   (OFFSET_SPI_CONTROLLER_DATA_SIZE * id);

	ret = fbu2s_tlv_parse(TLV_TYPE_READ_REQ, ctr_addr,
				NULL, FBU2S_SPI_CTR_BUFF_LEN);
	read_len = tlv_pkg.length + OFFSET_SPI_HEAD;
	ctr_buff = kmalloc(USB_BULK_MAX_SIZE, GFP_KERNEL);
	if (ctr_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(ctr_buff, 0, USB_BULK_MAX_SIZE);
	read_buf = kmalloc(read_len, GFP_KERNEL);
	if (read_buf == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(read_buf, 0, read_len);
	memcpy(read_buf, &tlv_pkg, read_len);

	//check the spi controller status to make sure the read is done.
	for (j = 0; j < SPI_TRANSFER_DELAY_COUNT; j++) {
		ret = fbu2s_usb_transfer(USPI_BULK_OUT, read_buf,
					read_len, NULL);
		if (ret) {
			dev_info(parent, "USB transfer fail!\n");
			break;
		}

		memset(ctr_buff, 0, USB_BULK_MAX_SIZE);
		ret = usb_bulk_msg(fbu2s_bridge.usb_dev, BULK_IN_PIPE,
				   ctr_buff, FBU2S_SPI_CTR_BUFF_LEN,
				   &count, DATA_TIMEOUT);
		if (ret) {
			dev_info(parent, "USB transfer fail!\n");
			break;
		}

		//SPI controller done when 1
		if (ctr_buff[FBU2S_SPI_CTR_SPI_STATUS_OFFSET] != 1)
			udelay(SPI_TRANSFER_DELAY);
		else
			break;
	}

exit:
	if (ctr_buff != NULL) {
		kfree(ctr_buff);
	}
	if (read_buf != NULL) {
		kfree(read_buf);
	}
	return ret;
}

static int fbu2s_spi_xfer_rx_get_buf(struct spi_transfer *xfer, unsigned int id)
{
	int ret = 0;
	int read_len;
	unsigned int count;
	unsigned long read_addr;
	uint8_t *recv_data_buff      = NULL;
	uint8_t *read_buf            = NULL;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	read_addr = OFFSET_SPI_BASE_MISO + (OFFSET_SPI_MASTER_DATA_SIZE * id);
	read_buf = kmalloc(USB_BULK_MAX_SIZE, GFP_KERNEL);
	if (read_buf == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(read_buf, 0, USB_BULK_MAX_SIZE);
	recv_data_buff = kmalloc(USB_BULK_MAX_SIZE, GFP_KERNEL);
	if (recv_data_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(recv_data_buff, 0, USB_BULK_MAX_SIZE);

	//write read buffer request
	ret = fbu2s_tlv_parse(TLV_TYPE_READ_REQ, read_addr,
				NULL, FBU2S_SPI_DATA_READ_REQUEST_LEN);
	read_len = tlv_pkg.length + OFFSET_SPI_HEAD;
	memset(read_buf, 0, read_len);
	memcpy(read_buf, &tlv_pkg, read_len);
	ret = fbu2s_usb_transfer(USPI_BULK_OUT, read_buf, read_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		goto exit;
	}

	//read
	memset(recv_data_buff, 0, USB_BULK_MAX_SIZE);
	ret = usb_bulk_msg(fbu2s_bridge.usb_dev, BULK_IN_PIPE,
			   recv_data_buff, FBU2S_SPI_DATA_NORMAL_READ_LEN,
			   &count, DATA_TIMEOUT);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		goto exit;
	}

	/* if the read was successful, copy the data to user space */
	if (!ret) {
		memset(xfer->rx_buf, 0, USB_BULK_MAX_SIZE);
		memcpy(xfer->rx_buf,
			   (recv_data_buff + FBU2S_SPI_MISO_DATA_OP_OFFSET),
			   xfer->len);
	}
	ret = 0;

exit:
	if (recv_data_buff != NULL) {
		kfree(recv_data_buff);
	}
	if (read_buf != NULL) {
		kfree(read_buf);
	}
	return ret;
}

static int fbu2s_spi_xfer_rx_read_1st_256B(uint8_t *mosi_buff,
					struct fbu2s_tlv tlv_pkg_temp,
					int mosi_buff_len)
{
	int ret;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;
	/* only first 256bytes need the read command,
	 * other bytes do not need send command to MOSI buffer
	 */
	memcpy(&tlv_pkg, &tlv_pkg_temp, sizeof(tlv_pkg));
	memcpy(mosi_buff, &tlv_pkg, mosi_buff_len);
	ret = fbu2s_usb_transfer(USPI_BULK_OUT, mosi_buff,
				mosi_buff_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		return -1;
	}
	return 0;
}

static int fbu2s_spi_xfer_rx_read_ctrl(unsigned int id,
					unsigned int i,
					unsigned int read_count,
					uint8_t *spi_ctr_buff)
{
	int ret;
	int first_read_transfer_flag = 0;
	unsigned long ctr_addr;
	int spi_ctr_buff_len;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	ctr_addr = OFFSET_SPI_BASE_CONTROLLER +
			   (OFFSET_SPI_CONTROLLER_DATA_SIZE * id);
	spi_ctr_buff_len = FBU2S_SPI_CTR_BUFF_LEN;
	/* write spi controller:(hard code rigth now)
	 * specific treatment:
	 *   1.At the last 256B cycle of 4K read(0x03), write continue
	 *     #cs to be 0(0x80), otherwise write continue #cs to be 1(0xa0).
	 *   2.For the 1st 256B, the read length is 0x104, otherwise
	 *     the read length is 0x100, because below FPGA design:
	 *     MISO buffer will return data+size_miso byte of data.
	 *     The first size_mosi byte of data should be discarded.
	 */
	if (i != (read_count - 1)) {
		spi_ctr_data[FBU2S_SPI_CTR_DATA_CS_FIELD_OFFSET] =
			FBU2S_SPI_CTR_CONTINUE_CS_ON;
	} else {
		spi_ctr_data[FBU2S_SPI_CTR_DATA_CS_FIELD_OFFSET] =
			FBU2S_SPI_CTR_CONTINUE_CS_OFF;
	}
	if (i == 0)
		first_read_transfer_flag = 1;
	else
		first_read_transfer_flag = 0;

	ret = fbu2s_spi_get_ctr_buff(FLASH_COMMAND_READ, spi_ctr_buff,
			spi_ctr_buff_len, ctr_addr,
			spi_ctr_data[FBU2S_SPI_CTR_DATA_CS_FIELD_OFFSET],
			first_read_transfer_flag);
	if (ret) {
		dev_info(parent, "Send spi ctr register fail!\n");
		return -1;
	}

	ret = fbu2s_usb_transfer(USPI_BULK_OUT, spi_ctr_buff,
				spi_ctr_buff_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		return -1;
	}

	return 0;
}

static int fbu2s_spi_xfer_rx_4k_get_buff(unsigned int id,
					unsigned int i,
					uint8_t *read_buf,
					uint8_t *recv_data_buff,
					uint8_t **recv_full_data_buff)
{
	int ret;
	int read_len;
	unsigned int count;
	unsigned long read_addr;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	read_addr = OFFSET_SPI_BASE_MISO + (OFFSET_SPI_MASTER_DATA_SIZE * id);

	//read begin
	//write read buffer request
	ret = fbu2s_tlv_parse(TLV_TYPE_READ_REQ, read_addr, NULL,
				FBU2S_SPI_DATA_READ_REQUEST_LEN);
	read_len = tlv_pkg.length + OFFSET_SPI_HEAD;
	memset(read_buf, 0, read_len);
	memcpy(read_buf, &tlv_pkg, read_len);
	ret = fbu2s_usb_transfer(USPI_BULK_OUT, read_buf, read_len, NULL);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		return -1;
	}

	//read
	ret = usb_bulk_msg(fbu2s_bridge.usb_dev, BULK_IN_PIPE,
			   recv_data_buff, FBU2S_SPI_DATA_NORMAL_READ_LEN,
			   &count, DATA_TIMEOUT);
	if (ret) {
		dev_info(parent, "USB transfer fail!\n");
		return -1;
	}

	if (!ret) {
	/* TODO: optimization.
	 * The remainder less than 256 is treated as 256 now
	 */
		if (i == 0) {
			memcpy(*recv_full_data_buff,
				   (recv_data_buff +
				   FBU2S_SPI_MISO_DATA_1ST_READ_OFFSET),
				   READ_MAX_LEN);
		} else {
			memcpy(*recv_full_data_buff,
				   (recv_data_buff +
				   FBU2S_SPI_MISO_DATA_OTHER_READ_OFFSET),
				   READ_MAX_LEN);
		}
		*recv_full_data_buff = *recv_full_data_buff + READ_MAX_LEN;
	}
	return 0;
}

static int fbu2s_spi_xfer_rx_4k_read(struct spi_transfer *xfer, unsigned int id)
{
	int ret = 0;
	uint8_t *spi_ctr_buff        = NULL;
	uint8_t *read_buf            = NULL;
	uint8_t *mosi_buff           = NULL;
	uint8_t *recv_data_buff      = NULL;
	uint8_t *recv_full_data_buff = NULL;
	int mosi_buff_len;
	int temp = 0;
	int spi_ctr_buff_len;
	unsigned int read_count = 0;
	unsigned int i;
	struct fbu2s_tlv tlv_pkg_temp = {0};
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	recv_full_data_buff = kmalloc(xfer->len, GFP_KERNEL);
	if (recv_full_data_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(recv_full_data_buff, 0, xfer->len);

	if ((xfer->len) % READ_MAX_LEN == 0)
		read_count = xfer->len / READ_MAX_LEN;
	else
		read_count = (xfer->len / READ_MAX_LEN) + 1;

	temp = tlv_pkg.data_buff[0];
	if (temp == FLASH_COMMAND_READ)
		/* used to store send mosi buffer,
		 * because tlv_pkg will change to miso request buffer
		 */
		memcpy(&tlv_pkg_temp, &tlv_pkg, sizeof(tlv_pkg));

	mosi_buff_len = tlv_pkg.length + OFFSET_SPI_HEAD;
	mosi_buff = kmalloc(mosi_buff_len, GFP_KERNEL);
	if (mosi_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(mosi_buff, 0, mosi_buff_len);

	spi_ctr_buff_len = FBU2S_SPI_CTR_BUFF_LEN;
	spi_ctr_buff = kmalloc(spi_ctr_buff_len, GFP_KERNEL);
	if (spi_ctr_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(spi_ctr_buff, 0, spi_ctr_buff_len);

	read_buf = kmalloc(USB_BULK_MAX_SIZE, GFP_KERNEL);
	if (read_buf == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(read_buf, 0, USB_BULK_MAX_SIZE);
	recv_data_buff = kmalloc(USB_BULK_MAX_SIZE, GFP_KERNEL);
	if (recv_data_buff == NULL) {
		ret = -ENOMEM;
		goto exit;
	}
	memset(recv_data_buff, 0, USB_BULK_MAX_SIZE);

	for (i = 0; i < read_count; i++) {
		if (i == 0) {
			ret = fbu2s_spi_xfer_rx_read_1st_256B(mosi_buff,
						tlv_pkg_temp, mosi_buff_len);
			if (ret) {
				dev_info(parent,
					"spi transfer read 1st 256bytes fail!\n");
				goto exit;
			}
		}

		ret = fbu2s_spi_xfer_rx_read_ctrl(id, i, read_count, spi_ctr_buff);
		if (ret) {
			dev_info(parent, "SPI Rx transfer set read ctrl fail!\n");
			goto exit;
		}

		ret = fbu2s_spi_xfer_rx_status_check(xfer, id);
		if (ret) {
			dev_info(parent, "SPI Rx transfer status check fail!\n");
			goto exit;
		}

		ret = fbu2s_spi_xfer_rx_4k_get_buff(id, i,
				read_buf, recv_data_buff, &recv_full_data_buff);
		if (ret) {
			dev_info(parent, "SPI Rx transfer4k buffer get fail!\n");
			goto exit;
		}

	}
	/* if the read was successful, copy the data to user space */
	memset(xfer->rx_buf, 0, xfer->len);
	recv_full_data_buff -= (READ_MAX_LEN*i);
	memcpy(xfer->rx_buf, recv_full_data_buff, xfer->len);
	ret = 0;

exit:
	if (mosi_buff != NULL) {
		kfree(mosi_buff);
	}
	if (spi_ctr_buff != NULL) {
		kfree(spi_ctr_buff);
	}
	if (read_buf != NULL) {
		kfree(read_buf);
	}
	if (recv_data_buff != NULL) {
		kfree(recv_data_buff);
	}

	return ret;
}
static int fbu2s_spi_xfer_rx(struct spi_transfer *xfer, unsigned int id)
{
	int ret;
	int temp = 0;
	unsigned long addr, read_addr, ctr_addr;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	addr = OFFSET_SPI_BASE_MOSI + (OFFSET_SPI_MASTER_DATA_SIZE * id);
	read_addr = OFFSET_SPI_BASE_MISO + (OFFSET_SPI_MASTER_DATA_SIZE * id);
	ctr_addr = OFFSET_SPI_BASE_CONTROLLER +
			   (OFFSET_SPI_CONTROLLER_DATA_SIZE * id);

	if (xfer->len <= READ_MAX_LEN) {
		temp = tlv_pkg.data_buff[0];
		if (temp == FLASH_COMMAND_READ) {
			ret = fbu2s_spi_xfer_rx_read(xfer, id);
			if (ret) {
				dev_info(parent, "SPI Rx transfer read fail!\n");
				return -1;
			}
		}
		ret = fbu2s_spi_xfer_rx_status_check(xfer, id);
		if (ret) {
			dev_info(parent, "SPI Rx transfer status check fail!\n");
			return -1;
		}
		ret = fbu2s_spi_xfer_rx_get_buf(xfer, id);
		if (ret) {
			dev_info(parent, "SPI Rx transfer get buffer fail!\n");
			return -1;
		}
	} else {
		ret = fbu2s_spi_xfer_rx_4k_read(xfer, id);
		if (ret) {
			dev_info(parent, "SPI Rx transfer 4k read fail!\n");
			return -1;
		}
	}
	return 0;
}


/*
 * spi_xfer_one
 */
static int fbu2s_spi_xfer_one(struct spi_master *master,
				struct spi_device *slave,
				struct spi_transfer *xfer)
{
	int ret;
	unsigned int id;
	struct fbu2s_spi_bus *spi = spi_master_get_devdata(master);
	struct device *parent = &fbu2s_bridge.usb_intf->dev;

	/* search for spi setup of this device */
	for (id = 0; id < FBUS_NUM_SPI_BUSES; id++) {
		if (spi->spi_id == fbu2s_bridge.spi_buses[id]->spi_id)
			break;
	}

	/* err handle */
	if (id == FBUS_NUM_SPI_BUSES) {
		dev_info(parent, "Cannot find the spi device\n");
		return 0;
	}

	if (xfer->tx_buf != NULL) {
		ret = fbu2s_spi_xfer_tx(xfer, id);
		if (ret) {
			dev_info(parent, "SPI Tx transfer fail!\n");
			return -1;
		}
	}

	if (xfer->rx_buf != NULL) {
		ret = fbu2s_spi_xfer_rx(xfer, id);
		if (ret) {
			dev_info(parent, "SPI Rx transfer fail!\n");
			return -1;
		}
	}

	return ret;
}

static int fbu2s_spi_bus_init(u32 id)
{
	int ret;
	struct spi_master *master;
	struct fbu2s_spi_bus *spi_bus;
	struct device *parent = &fbu2s_bridge.usb_intf->dev;
	int	status = 0;
	struct spi_board_info spi_dev = {
		.modalias = "spidev",
		.max_speed_hz = 25000000,
		.bus_num = master->bus_num,
		.chip_select = 0,
	};

	master = spi_alloc_master(parent, sizeof(struct fbu2s_spi_bus));
	if (master == NULL) {
		dev_err(parent, "failed to allocate spi_master[%u]\n", id);
		return -ENOMEM;
	}

	master->num_chipselect = 1;
	master->setup = fbu2s_spi_setup;
	master->set_cs = fbu2s_spi_set_cs;
	master->transfer_one = fbu2s_spi_xfer_one;

	ret = spi_register_master(master);
	if (ret) {
		dev_err(parent, "failed to register spi_master[%u]\n", id);
		spi_master_put(master);
		return ret;
	}

	spi_bus = spi_master_get_devdata(master);
	if (spi_bus == NULL) {
		dev_err(parent, "failed to spi_master_get_devdata spi_master[%u]\n",
				id);
		return -ENXIO;
	}

	spi_bus->spi_id = id;
	fbu2s_bridge.spi_buses[id] = spi_bus;

	spi_new_device(master, &spi_dev);

	return status;
}

static void fbu2s_spi_remove_all(void)
{
	u32 i;
	struct fbu2s_spi_bus *spi_bus;

	for (i = 0; i < ARRAY_SIZE(fbu2s_bridge.spi_buses); i++) {
		spi_bus = fbu2s_bridge.spi_buses[i];

		if (spi_bus != NULL) {
			spi_unregister_device(spi_master_get_devdata(spi_bus->master));
			spi_unregister_master(spi_bus->master);
			spi_master_put(spi_bus->master);
		}
	}
}

static int fbu2s_usb_probe(struct usb_interface *usb_intf,
						   const struct usb_device_id *usb_id)
{
	u32 i;
	int ret;
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;

	fbu2s_bridge.usb_intf = usb_intf;
	fbu2s_bridge.usb_dev = usb_get_dev(interface_to_usbdev(usb_intf));
	usb_set_intfdata(usb_intf, &fbu2s_bridge);

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	ret = usb_find_common_endpoints(usb_intf->cur_altsetting,
			&bulk_in, &bulk_out, NULL, NULL);
	if (ret) {
		dev_err(&usb_intf->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	fbu2s_bridge.ep_list[USPI_BULK_IN] = bulk_in->bEndpointAddress;
	fbu2s_bridge.ep_list[USPI_BULK_OUT] = bulk_out->bEndpointAddress;

	for (i = 0; i < ARRAY_SIZE(fbu2s_bridge.spi_buses); i++) {
		ret = fbu2s_spi_bus_init(i);
		if (ret < 0)
			goto error;
	}

	/* save our data pointer in this interface device */
	usb_set_intfdata(usb_intf, &fbu2s_bridge);

	dev_info(&usb_intf->dev, "ep_bulk_in: 0x%x, ep_bulk_out: 0x%x\n",
		 fbu2s_bridge.ep_list[USPI_BULK_IN],
		 fbu2s_bridge.ep_list[USPI_BULK_OUT]);

	return 0;

 error:
	fbu2s_spi_remove_all();
	return ret;
}

static void fbu2s_usb_disconnect(struct usb_interface *usb_intf)
{
	fbu2s_spi_remove_all();
	usb_set_intfdata(usb_intf, NULL);
	usb_put_dev(fbu2s_bridge.usb_dev);
}

static const struct usb_device_id fbu2s_usb_table[] = {
	{USB_DEVICE(0x2ec6, 0x0100)}, /* XILINX */
	{USB_DEVICE(0x0100, 0x2EC6)}, /* XILINX */
	{}
};
MODULE_DEVICE_TABLE(usb, fbu2s_usb_table);

static struct usb_driver fbu2s_usb_driver = {
	.name		= "fb-usb-spi",
	.probe		= fbu2s_usb_probe,
	.disconnect	= fbu2s_usb_disconnect,
	.id_table	= fbu2s_usb_table,
};

module_usb_driver(fbu2s_usb_driver);

MODULE_AUTHOR("Tao Ren<taoren@fb.com>");
MODULE_DESCRIPTION("Facebook USB-SPI Adapter Driver");
MODULE_LICENSE("GPL");
