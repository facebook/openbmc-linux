// SPDX-License-Identifier: GPL-2.0
// include/linux/jtag.h - JTAG class driver
//
// Copyright (c) 2017 Mellanox Technologies. All rights reserved.
// Copyright (c) 2017 Oleksandr Shamray <oleksandrs@mellanox.com>

#ifndef __JTAG_H
#define __JTAG_H

#include <uapi/linux/jtag.h>

#define jtag_u64_to_ptr(arg) ((void *)(uintptr_t)arg)

#define JTAG_MAX_XFER_DATA_LEN 65535

struct jtag;
/**
 * struct jtag_ops - callbacks for jtag control functions:
 *
 * @freq_get: get frequency function. Filled by device driver
 * @freq_set: set frequency function. Filled by device driver
 * @status_get: set status function. Filled by device driver
 * @idle: set JTAG to idle state function. Filled by device driver
 * @xfer: send JTAG xfer function. Filled by device driver
 * @run_cycle: send JTAG run_cycle function. Filled by device driver
 */
struct jtag_ops {
    int (*freq_get)(struct jtag *jtag, u32 *freq);
    int (*freq_set)(struct jtag *jtag, u32 freq);
    int (*status_get)(struct jtag *jtag, u32 *state);
    int (*idle)(struct jtag *jtag, struct jtag_run_test_idle *idle);
    int (*xfer)(struct jtag *jtag, struct jtag_xfer *xfer, u8 *xfer_data);
    int (*mode_set)(struct jtag *jtag, u32 mode_mask);
    void (*run_cycle)(struct jtag *jtag, struct run_cycle_param *run_cycle);
};

void *jtag_priv(struct jtag *jtag);
int jtag_register(struct jtag *jtag);
void jtag_unregister(struct jtag *jtag);
struct jtag *jtag_alloc(size_t priv_size, const struct jtag_ops *ops);
void jtag_free(struct jtag *jtag);

#endif /* __JTAG_H */

