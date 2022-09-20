/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021-2022 Advanced Micro Devices, Inc.
 */

#ifndef _AMD_APML_SBRMI_H_
#define _AMD_APML_SBRMI_H_

#include <linux/miscdevice.h>
#include <linux/amd-apml.h>

/* Each client has this additional data */
struct apml_sbrmi_device {
	struct miscdevice sbrmi_misc_dev;
	struct regmap *regmap;
	struct mutex lock;
	u32 pwr_limit_max;
	u8 rev;
} __packed;

struct apml_spl_ops {
	int (*rmi_cpuid_read)(struct apml_sbrmi_device *rmi_dev,
			      struct apml_message *msg);
	int (*rmi_mca_msr_read)(struct apml_sbrmi_device *rmi_dev,
				struct apml_message *msg);
};

extern struct apml_spl_ops apml_ops;

void rmi_set_apml_ops(int rev);

int sbrmi_enable_alert(struct apml_sbrmi_device *rmi_dev);
int rmi_mca_msr_read_v20(struct apml_sbrmi_device *rmi_dev,
			 struct apml_message *msg);
int rmi_mca_msr_read_v10(struct apml_sbrmi_device *rmi_dev,
			 struct apml_message *msg);
int rmi_cpuid_read_v10(struct apml_sbrmi_device *rmi_dev,
		       struct apml_message *msg);
int rmi_cpuid_read_v20(struct apml_sbrmi_device *rmi_dev,
		       struct apml_message *msg);
int rmi_mailbox_xfer(struct apml_sbrmi_device *rmi_dev,
		     struct apml_message *msg);
#endif /*_AMD_APML_SBRMI_H_*/
