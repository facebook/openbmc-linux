/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __LINUX_FBOSS_USMC_H
#define __LINUX_FBOSS_USMC_H

#include <linux/platform_device.h>

/*
 * The maximum bulk packet size is 512 bytes; as each tlv packet contains
 * 8 byte type/length/address, the actually payload is (512 - 8) bytes.
 */
#define FB_USMC_BULK_MAX	512
#define FB_USMC_PAYLOAD_MAX	(512 - 8)

int fboss_usmc_write(struct platform_device *pdev, u32 addr,
		     const void *buf, unsigned int size);
int fboss_usmc_read(struct platform_device *pdev, u32 addr,
		    void *buf, unsigned int size);
void fboss_usmc_xfer_lock(struct platform_device *pdev);
void fboss_usmc_xfer_unlock(struct platform_device *pdev);

#endif /* __LINUX_FBOSS_USMC_H */
