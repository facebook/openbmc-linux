/*
 * dev-i2c-wedge400.c - i2c device definition for WEDGE400
 *
 * Copyright 2019-present Facebook. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/platform_device.h>

#include <plat/devs.h>
#include <plat/ast_i2c.h>

pca954x_info_st dev_i2c_pca954x_info[] = {
{-1, 2, 0x70},  /* 0 */
{-1, 8, 0x70},  /* 1 */
{-1, 11, 0x76}, /* 2 */
};

const int size_dev_i2c_pca954x_info =
    sizeof(dev_i2c_pca954x_info) / sizeof(dev_i2c_pca954x_info[0]);

extern void dev_i2c_add_pca954x(pca954x_info_st [],
                                  const int n_dev_i2c_pca954x_info);

void __init ast_add_device_i2c(void)
{
  ast_add_device_i2c_common();
  dev_i2c_add_pca954x(dev_i2c_pca954x_info, size_dev_i2c_pca954x_info);
}
