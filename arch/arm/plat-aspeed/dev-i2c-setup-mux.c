/*
 * dev-i2c-setup-mux.c - Setup i2c mux (PCA954x)
 *
 * Copyright 2014-present Facebook. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/pca954x.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <plat/devs.h>
#include <plat/ast_i2c.h>

#define PCA954X_ON_SAME_BUS(i, j) \
  (dev_i2c_pca954x_info[i].cpi_mux_index == dev_i2c_pca954x_info[j].cpi_mux_index \
    && dev_i2c_pca954x_info[i].cpi_chan == dev_i2c_pca954x_info[j].cpi_chan)

#define PCA954X_START_BUS 16
#define PCA954X_N_CHAN 8

void dev_i2c_add_pca954x(pca954x_info_st dev_i2c_pca954x_info[],
                           const int n_dev_i2c_pca954x_info)
{
  struct i2c_board_info *binfo = NULL;
  struct pca954x_platform_data *pdata = NULL;
  struct pca954x_platform_mode *pmode = NULL;
  int i;
  int j;
  int k;
  int m;
  int cur_bus;
  int cur_alloc = PCA954X_START_BUS;
  int binfo_n_entries = 0;

  for (i = 0; i < n_dev_i2c_pca954x_info; i++) {
    dev_i2c_pca954x_info[i].cpi_bus_alloc = -1;
  }

  for (i = 0; i < n_dev_i2c_pca954x_info; i++) {
    if (dev_i2c_pca954x_info[i].cpi_bus_alloc != -1) {
      /* this mux has been registered already */
      continue;
    }

    binfo_n_entries = 1;
    for (j = i + 1; j < n_dev_i2c_pca954x_info; j++) {
      if (PCA954X_ON_SAME_BUS(i, j)) {
        binfo_n_entries++;
      }
    }

    /* find out the bus # that holds those devices */
    if (dev_i2c_pca954x_info[i].cpi_mux_index == -1) {
      cur_bus = 0;
    }
    else {
      /* safety check */
      if (dev_i2c_pca954x_info[i].cpi_mux_index >= n_dev_i2c_pca954x_info) {
        continue;
      }

      j = dev_i2c_pca954x_info[i].cpi_mux_index;
      cur_bus = dev_i2c_pca954x_info[j].cpi_bus_alloc;
      if (cur_bus == -1) {
        continue;
      }
    }

    cur_bus += dev_i2c_pca954x_info[i].cpi_chan;
    binfo = kzalloc(sizeof(*binfo) * binfo_n_entries, GFP_KERNEL);
    if (!binfo) {
      goto error_out;
    }

    k = 0;
    for (j = i; j < n_dev_i2c_pca954x_info; j++) {
      if (!PCA954X_ON_SAME_BUS(i, j)) {
        continue;
      }

      dev_i2c_pca954x_info[j].cpi_bus_alloc = cur_alloc;
      pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
      if (!pdata) {
        goto error_out;
      }

      pmode = kzalloc(sizeof(*pmode) * PCA954X_N_CHAN, GFP_KERNEL);
      if (!pmode) {
        goto error_out;
      }

      for (m = 0; m < PCA954X_N_CHAN; m++) {
        pmode[m].adap_id = cur_alloc++;
        pmode[m].deselect_on_exit = 1;
      }

      /* link platform mode to platform data */
      pdata->modes = pmode;
      pdata->num_modes = PCA954X_N_CHAN;

      /* link platform data to the board info */
      strcpy(binfo[k].type, "pca9548");
      binfo[k].addr = dev_i2c_pca954x_info[j].cpi_addr;
      binfo[k].platform_data = pdata;

      /*
       * reset pmode and pdata to avoid double free
       * in case of error
       */
      pdata = NULL;
      pmode = NULL;
      k++;
    }

    /* register the devices on that i2c bus */
    i2c_register_board_info(cur_bus, binfo, binfo_n_entries);
    binfo = NULL;
  }

  return;

error_out:
  if (pmode) {
    kfree(pmode);
  }

  if (pdata) {
    kfree(pdata);
  }

  if (binfo) {
    for (i = 0; i < binfo_n_entries; i++) {
      if (!binfo[i].platform_data) {
        kfree(binfo[i].platform_data);
      }
    }
    kfree(binfo);
  }
}
