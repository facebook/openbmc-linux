/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2023-2024 Advanced Micro Devices, Inc.
 */

#ifndef _AMD_APML_ALERT_L__
#define _AMD_APML_ALERT_L__

#include "sbrmi-common.h"

/*
 * TODO: Handle TSI alerts
 */
struct apml_alertl_data {
	struct apml_sbrmi_device **rmi_dev;
	u8 num_of_rmi_devs;
} __packed;

#endif /*_AMD_APML_ALERT_L__*/
