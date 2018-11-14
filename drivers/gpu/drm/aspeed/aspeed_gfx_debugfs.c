// SPDX-License-Identifier: GPL-2.0
// Copyright © 2017 Broadcom
// Copyright 2018 IBM Corp

#include <linux/seq_file.h>
#include <drm/drm_debugfs.h>

#include "aspeed_gfx.h"

#define REGDEF(reg) { reg, #reg }
static const struct {
	u32 reg;
	const char *name;
} aspeed_gfx_reg_defs[] = {
	REGDEF(CRT_CTRL1),
	REGDEF(CRT_CTRL2),
	REGDEF(CRT_STATUS),
	REGDEF(CRT_MISC),
	REGDEF(CRT_HORIZ0),
	REGDEF(CRT_HORIZ1),
	REGDEF(CRT_VERT0),
	REGDEF(CRT_VERT1),
	REGDEF(CRT_ADDR),
	REGDEF(CRT_OFFSET),
	REGDEF(CRT_THROD),
	REGDEF(CRT_XSCALE),
	REGDEF(CRT_CURSOR0),
	REGDEF(CRT_CURSOR1),
	REGDEF(CRT_CURSOR2),
	REGDEF(CRT_9C),
	REGDEF(CRT_OSD_H),
	REGDEF(CRT_OSD_V),
	REGDEF(CRT_OSD_ADDR),
	REGDEF(CRT_OSD_DISP),
	REGDEF(CRT_OSD_THRESH),
	REGDEF(CRT_B4),
	REGDEF(CRT_STS_V),
	REGDEF(CRT_SCRATCH),
	REGDEF(CRT_BB0_ADDR),
	REGDEF(CRT_BB1_ADDR),
	REGDEF(CRT_BB_COUNT),
	REGDEF(OSD_COLOR1),
	REGDEF(OSD_COLOR2),
	REGDEF(OSD_COLOR3),
	REGDEF(OSD_COLOR4),
	REGDEF(OSD_COLOR5),
	REGDEF(OSD_COLOR6),
	REGDEF(OSD_COLOR7),
	REGDEF(OSD_COLOR8),
};

int aspeed_gfx_debugfs_regs(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct aspeed_gfx *priv = dev->dev_private;
	int i;

	for (i = 0; i < ARRAY_SIZE(aspeed_gfx_reg_defs); i++) {
		seq_printf(m, "%15s (0x%02x): 0x%08x\n",
			   aspeed_gfx_reg_defs[i].name, aspeed_gfx_reg_defs[i].reg,
			   readl(priv->base + aspeed_gfx_reg_defs[i].reg));
	}

	return 0;
}

static const struct drm_info_list aspeed_gfx_debugfs_list[] = {
	{"regs", aspeed_gfx_debugfs_regs, 0},
};

int aspeed_gfx_debugfs_init(struct drm_minor *minor)
{
	return drm_debugfs_create_files(aspeed_gfx_debugfs_list,
					ARRAY_SIZE(aspeed_gfx_debugfs_list),
					minor->debugfs_root, minor);
}
