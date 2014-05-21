/********************************************************************************
* File Name     : drivers/video/hdmi_cat6613.c
* Author        : Ryan Chen 
* Description   : HDMI CAT6613 driver
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/08/24 Ryan Chen create this file
*
********************************************************************************/

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>

#include <mach/regs-cat6613.h>
#include "edid.h"

#define DEVICE_NAME "cat6613"
#define CAT6613_DEVICE_ID   0xCA13

struct cat6613_info {
	struct i2c_client *client;
	struct fb_info *fb_info;
	struct aspeed_monitor_info *mon_info;
	struct work_struct cat6613_work;
	int state;//0:unplug 1:plug
	int irq;
};

static struct cat6613_info cat6613_device;
struct aspeed_monitor_info monitor_info;


static void get_detailed_timing(unsigned char *block,
				struct fb_videomode *mode)
{
	mode->xres = H_ACTIVE;
	mode->yres = V_ACTIVE;
	mode->pixclock = PIXEL_CLOCK;
	mode->pixclock /= 1000;
	mode->pixclock = KHZ2PICOS(mode->pixclock);
	mode->right_margin = H_SYNC_OFFSET;
	mode->left_margin = (H_ACTIVE + H_BLANKING) -
		(H_ACTIVE + H_SYNC_OFFSET + H_SYNC_WIDTH);
	mode->upper_margin = V_BLANKING - V_SYNC_OFFSET -
		V_SYNC_WIDTH;
	mode->lower_margin = V_SYNC_OFFSET;
	mode->hsync_len = H_SYNC_WIDTH;
	mode->vsync_len = V_SYNC_WIDTH;
	if (HSYNC_POSITIVE)
		mode->sync |= FB_SYNC_HOR_HIGH_ACT;
	if (VSYNC_POSITIVE)
		mode->sync |= FB_SYNC_VERT_HIGH_ACT;
	mode->refresh = PIXEL_CLOCK/((H_ACTIVE + H_BLANKING) *
				     (V_ACTIVE + V_BLANKING));
	if (INTERLACED) {
		mode->yres *= 2;
		mode->upper_margin *= 2;
		mode->lower_margin *= 2;
		mode->vsync_len *= 2;
		mode->vmode |= FB_VMODE_INTERLACED;
	}
	else
		mode->vmode=0;
	mode->flag = FB_MODE_IS_DETAILED;

}

static void cat6613_parse_cea(void)
{
	int timing_offset,cea_data_offset=0,data_tag,data_len,vic,i;
	char *ext=&cat6613_device.mon_info->edid[128];
	struct fb_monspecs *specs=&cat6613_device.mon_info->specs;
	
	if(cat6613_device.mon_info->edid[126]==0 || ext[0]!=0x2) {
			printk("DVI mode\n");
			cat6613_device.mon_info->type=0; //dvi mode
			return;
	}
  
  printk("CEA Revision=%d\n", ext[1]);
  
  if(ext[3]& (1<<6)) {
  	printk("HDMI mode\n");
  	cat6613_device.mon_info->type=1; //hdmi mode
  }
  else {
		printk("HDMI mode without audio\n");
  	cat6613_device.mon_info->type=0; //dvi mode
	}
  
  if(ext[2]==0) //no timing & cea data for parsing
  	return;
  
  timing_offset=ext[2];
  
  //parsing cea data
  if(timing_offset!=4) {
  		while((cea_data_offset+4)!=timing_offset) {
  			data_tag=(ext[cea_data_offset+4]>>5)&0x7; //bit 5~7
  			data_len=ext[cea_data_offset+4]&0x1f; //bit 0~4
  			switch(data_tag) {
  				case 1:
  					//printk("audio data block\n");
  					break;
  				case 2:
  					//printk("video data block\n");
  					for(i=1;i<=data_len;i++) {
  						vic=ext[cea_data_offset+4+i]&0x7f;
  						//add 720p60 timing
  						if(vic==4) {
  							//printk("add 1280x720p60 timing\n");
  							memcpy(&specs->modedb[specs->modedb_len], &(panels[8].mode),sizeof(struct fb_videomode));
  							specs->modedb_len++;
  						}
  						//add 1080p60 timing
  						if(vic==16) {
  							//printk("add 1920x1080p60 timing\n");
  							memcpy(&specs->modedb[specs->modedb_len], &(panels[9].mode),sizeof(struct fb_videomode));
  							specs->modedb_len++;
  						}
  						if(vic==2 || vic==3) {
  							//printk("add 720x480p60 timing\n");
  							memcpy(&specs->modedb[specs->modedb_len], &(panels[10].mode),sizeof(struct fb_videomode));
  							specs->modedb_len++;
  						}
  						
  					}
  					break;
  				case 3:
  					//printk("vendor data block\n");
  					break;
  				case 4:
  					//printk("speaker data block\n");
  					break;
  				default:
  					//printk("unknown data block tag=%d\n",data_tag);
  					break;
  			}
  			cea_data_offset+=(data_len+1); //go to next block
  		}
	}
  while(ext[timing_offset]!=0) {
  	//printk("%x\n",ext[timing_offset+17]);
		get_detailed_timing(&ext[timing_offset], &specs->modedb[specs->modedb_len]);
		specs->modedb_len++;
		timing_offset+=18;
	}
	
}

static int cat6613_reset(struct i2c_client *client)
{
	int rc;
	rc = i2c_smbus_write_byte_data(client, REG_TX_BANK_CTRL, 0x00);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_SW_RST, 0x3d);
	msleep(2);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_SW_RST, 0x1d);
	msleep(2);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_HDMI_MODE, 0x00);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AV_MUTE, 0x01);
	
	//set int
	rc |=i2c_smbus_write_byte_data(client,REG_TX_INT_CTRL, 0x40);
	rc |=i2c_smbus_write_byte_data(client,REG_TX_INT_MASK1, 0xfd);
	rc |=i2c_smbus_write_byte_data(client,REG_TX_INT_MASK2, 0xff) ;
	rc |=i2c_smbus_write_byte_data(client,REG_TX_INT_MASK3, 0x7f);

	return rc;

}

static int cat6613_afe(struct i2c_client *client)
{
	int tmds,rc=0;
	if(cat6613_device.fb_info) {
		tmds=1000000/(cat6613_device.fb_info->var.pixclock);
		if(tmds > 80) {
			rc = i2c_smbus_write_byte_data(client, REG_TX_AFE_DRV_CTRL,0x10);
			rc |= i2c_smbus_write_byte_data(client, REG_TX_AFE_XP_CTRL,0x88);
			rc |= i2c_smbus_write_byte_data(client, REG_TX_AFE_ISW_CTRL,0x10);
			rc |= i2c_smbus_write_byte_data(client, REG_TX_AFE_IP_CTRL,0x84);
			
		}
		else {

			rc = i2c_smbus_write_byte_data(client, REG_TX_AFE_DRV_CTRL,0x10);
			rc |= i2c_smbus_write_byte_data(client, REG_TX_AFE_XP_CTRL,0x18);
			rc |= i2c_smbus_write_byte_data(client, REG_TX_AFE_ISW_CTRL,0x10);
			rc |= i2c_smbus_write_byte_data(client, REG_TX_AFE_IP_CTRL,0x0c);
		}
		rc |= i2c_smbus_write_byte_data(client, REG_TX_AFE_DRV_CTRL,0x00);	
	}	
	
	return rc;
}

static int cat6613_set_av(struct i2c_client *client)
{
	int rc=0;
	
	rc |= i2c_smbus_write_byte_data(client, REG_TX_SW_RST,0xd); //reset av
	msleep(1);
	if(cat6613_device.mon_info->type==0) {
		rc |= i2c_smbus_write_byte_data(client, REG_TX_HDMI_MODE,0);  //dvi mode
		rc |= i2c_smbus_write_byte_data(client, REG_TX_SW_RST,5);
		msleep(1);
		return rc;
	}
	
	rc |= i2c_smbus_write_byte_data(client, REG_TX_BANK_CTRL,1);  //switch bank 1

	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB1,0x12);  //set underscan
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB2,0x8);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB3,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB4,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB5,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB6,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB7,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB8,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB9,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB10,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB11,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB12,0x0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_DB13,0x0);
	
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVIINFO_SUM,0x55);  //check sum
		
	rc |= i2c_smbus_write_byte_data(client, REG_TX_BANK_CTRL,0);  //switch bank 0	
	rc |= i2c_smbus_write_byte_data(client, REG_TX_PKT_GENERAL_CTRL,1);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_NULL_CTRL,1);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_ACP_CTRL,0);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AVI_INFOFRM_CTRL,3);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AUD_INFOFRM_CTRL,1);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_MPG_INFOFRM_CTRL,0);

	rc |= i2c_smbus_write_byte_data(client, REG_TX_HDMI_MODE,1);  //hdmi mode
	
	rc |= i2c_smbus_write_byte_data(client, 0xf8,0xc3);  
	rc |= i2c_smbus_write_byte_data(client, 0xf8,0xa5);
	rc |= i2c_smbus_write_byte_data(client, REG_TX_PKT_SINGLE_CTRL,0x0); //set auto cts

	rc |= i2c_smbus_write_byte_data(client, REG_TX_AUDIO_CTRL0,0x0);  
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AUDIO_CTRL0,0x1);  //set i2s 16bit
	rc |= i2c_smbus_write_byte_data(client, REG_TX_AUDIO_CTRL1,0x1);  //set not full packet mode & 32bit i2s
	
	rc |= i2c_smbus_write_byte_data(client, REG_TX_SW_RST,1);
	msleep(1);  
	
	return rc;
	
}

static int cat6613_clear_mute(struct i2c_client *client)
{
	int rc;
	rc = i2c_smbus_write_byte_data(client, REG_TX_AV_MUTE,0);
	return rc;
}


static int cat6613_wait_ddc(struct i2c_client *client)
{
	int rc,count;
	
	for(count=0;count<10;count++) {
		rc=i2c_smbus_read_byte_data(client,REG_TX_DDC_STATUS);
		if(rc & B_DDC_DONE)
			return 0;
		msleep(1);
	}
	printk("ddc timeout\n");
	i2c_smbus_write_byte_data(client,REG_TX_DDC_MASTER_CTRL, B_MASTERHOST ) ;
	i2c_smbus_write_byte_data(client,REG_TX_DDC_CMD, CMD_DDC_ABORT) ;			
	return -1;

}

static int cat6613_read_edid(struct i2c_client *client)
{
	int j ;
	int remained_byte, offset, count;
	remained_byte = 256 ;
	offset=0;
	
	while(offset<256) {
		count = (remained_byte<32)?remained_byte:32 ;
		i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_DDC_MASTER_CTRL, B_MASTERDDC|B_MASTERHOST ) ;
		i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_DDC_CMD, CMD_FIFO_CLR);
		if(cat6613_wait_ddc(cat6613_device.client)) {
			return -1;
		}
		
		i2c_smbus_write_byte_data(client,REG_TX_DDC_HEADER, 0xA0) ;
		i2c_smbus_write_byte_data(client,REG_TX_DDC_REQOFF, offset) ;
		i2c_smbus_write_byte_data(client,REG_TX_DDC_REQCOUNT, count) ;
		i2c_smbus_write_byte_data(client,REG_TX_DDC_EDIDSEG, 0) ;
		i2c_smbus_write_byte_data(client,REG_TX_DDC_CMD, 3);
		if(cat6613_wait_ddc(cat6613_device.client)) {
			return -1;
		}
		for( j = 0 ; j < count ; j++)
		{
			cat6613_device.mon_info->edid[offset+j] = i2c_smbus_read_byte_data(client,REG_TX_DDC_READFIFO); ;
		}
		remained_byte -= count ;
		offset += count ;
	}
	return 0;
	
}

static void cat6613_add_modes(void)
{
	int i;
	struct fb_monspecs *specs=&cat6613_device.mon_info->specs;
	struct fb_info *info=cat6613_device.fb_info;
	
	for(i=0;i<specs->modedb_len;i++) {
		fb_add_videomode(&specs->modedb[i],&info->modelist);
	}
	
}

static void cat6613_del_modes(void)
{
	int i;
	struct fb_monspecs *specs=&cat6613_device.mon_info->specs;
	struct fb_info *info=cat6613_device.fb_info;
	if(!info)
		return;
	
	for(i=0;i<specs->modedb_len;i++) {
		fb_delete_videomode(&specs->modedb[i],&info->modelist);
	}
}

static void cat6613_handle(struct work_struct *work)
{
	char int_status,sys_status,rc,int_status3;
	struct fb_var_screeninfo tmp_var;
	int_status=i2c_smbus_read_byte_data(cat6613_device.client,REG_TX_INT_STAT1);
	sys_status=i2c_smbus_read_byte_data(cat6613_device.client, REG_TX_SYS_STATUS);
	int_status3=i2c_smbus_read_byte_data(cat6613_device.client,REG_TX_INT_STAT3);
	if(!(sys_status&B_INT_ACTIVE))
		printk("cat6613_handle: no int\n");
	else {
#if 0
		if(int_status & B_INT_DDCFIFO_ERR) {
			printk("B_INT_DDCFIFO_ERR\n");
			i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_DDC_MASTER_CTRL, B_MASTERHOST ) ;
			i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_DDC_CMD, CMD_FIFO_CLR);
		}
		
		if(int_status & B_INT_DDC_BUS_HANG) {
			printk("B_INT_DDC_BUS_HANG\n");
			i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_DDC_MASTER_CTRL, B_MASTERHOST ) ;
			i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_DDC_CMD, CMD_DDC_ABORT) ;
		}		
		if(int_status & B_INT_HPD_PLUG) {
			if(sys_status & B_HPDETECT)
				printk("HPD PLUG\n");
			else 
				printk("HPD UN PLUG 0\n");
			i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_INT_CLR0,B_CLR_HPD);
		}
		if(int_status3 & B_INT_VIDSTABLE) {
			if(sys_status & B_TXVIDSTABLE) {
				printk("VIDSTABLE\n");
				i2c_smbus_write_byte_data(cat6613_device.client, REG_TX_AFE_DRV_CTRL,0x00);
			}
			else
				printk("UN VIDSTABLE\n");
			i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_INT_CLR1,B_CLR_VIDSTABLE);
		}
#endif		
		if(int_status & B_INT_RX_SENSE) {
			if(sys_status& B_RXSENDETECT) {
				if(cat6613_device.state==0) {
					rc=cat6613_read_edid(cat6613_device.client);
					if(!rc) {
						rc=fb_parse_edid(cat6613_device.mon_info->edid,&tmp_var);
						if(!rc)
							cat6613_device.state=1;
					}
				}
			}
			else {
				printk("HPD UN PLUG 0\n");
				if(cat6613_device.state==1) {
					printk("HPD UN PLUG 1\n");
					cat6613_del_modes();
					cat6613_device.mon_info->status=0;
					cat6613_device.state=0;
				}
			}
			i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_INT_CLR0,B_CLR_RXSENSE);
		}
	}
	
	i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_SYS_STATUS, sys_status | B_INTACTDONE );
	i2c_smbus_write_byte_data(cat6613_device.client,REG_TX_SYS_STATUS, sys_status);
	enable_irq(cat6613_device.irq);
}

static irqreturn_t cat6613_isr(int irq, void *parm)
{
	
	disable_irq_nosync(cat6613_device.irq);
	schedule_work(&cat6613_device.cat6613_work);
	return IRQ_HANDLED;
}

static int hdmi_cat6613_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc=0;
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;
	
	rc=(i2c_smbus_read_byte_data(client, REG_TX_VENDOR_ID1)<<8)&0xff00;
	rc|=i2c_smbus_read_byte_data(client, REG_TX_DEVICE_ID0);
	if(rc != CAT6613_DEVICE_ID) {
		printk(KERN_ERR "%s: read id fail\n", __func__);
		return -ENODEV;
	}
	
	cat6613_device.client=client;
	cat6613_device.irq=client->irq;
	cat6613_device.mon_info=&monitor_info;
	
	//reset
	rc=cat6613_reset(client);
	if(rc)	
		printk(KERN_ERR "%s: reset fail\n", __func__);
	
	INIT_WORK(&cat6613_device.cat6613_work, cat6613_handle);
	rc = request_irq(cat6613_device.irq, cat6613_isr, IRQF_DISABLED, DEVICE_NAME, NULL);
	if(rc) {
		printk(KERN_ERR "%s: request irq fail\n", __func__);
		return rc;
	}	
	
	return rc;
}

static int __devexit hdmi_cat6613_remove(struct i2c_client *client)
{

	return 0;
}


static const struct i2c_device_id hmdi_cat6613_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

static struct i2c_driver hdmi_cat6613_i2c_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = hdmi_cat6613_probe,
	.remove =  __exit_p(hdmi_cat6613_remove),
	.id_table = hmdi_cat6613_id,
};


int aspeed_hdmi_get_info(struct fb_info *fb_info) 
{
	
	if(!fb_info) {
		printk("no fb_info\n");
		return -1;
	}
	cat6613_device.fb_info=fb_info;
	
	if(cat6613_device.state==0)
		return 1;
	
	if(cat6613_device.mon_info->status==0) {
		if(monitor_info.specs.modedb)
			fb_destroy_modedb(monitor_info.specs.modedb);
		fb_edid_to_monspecs(cat6613_device.mon_info->edid, &cat6613_device.mon_info->specs);
		cat6613_parse_cea();
		cat6613_add_modes();
		cat6613_device.mon_info->status=1;	
	}
	return 0;	
}

void aspeed_hdmi_enable(int en)
{
	if(en==0) {
		i2c_smbus_write_byte_data(cat6613_device.client, REG_TX_HDMI_MODE, 0x00);
		i2c_smbus_write_byte_data(cat6613_device.client, REG_TX_AV_MUTE, 0x01);
	}
	else {
		cat6613_set_av(cat6613_device.client);
		cat6613_afe(cat6613_device.client);
		cat6613_clear_mute(cat6613_device.client);
	}
}

static int __init hdmi_cat6613_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&hdmi_cat6613_i2c_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add i2c driver\n", __func__);

	return ret;
}

static void __exit hdmi_cat6613_exit(void)
{
	i2c_del_driver(&hdmi_cat6613_i2c_driver);
}

module_init(hdmi_cat6613_init);
module_exit(hdmi_cat6613_exit);

EXPORT_SYMBOL(aspeed_hdmi_get_info);
EXPORT_SYMBOL(aspeed_hdmi_enable);

MODULE_AUTHOR("Ryan Chen <jsho@aspeed-tech.com>");
MODULE_DESCRIPTION("CAT6023 HDMI Driver");
MODULE_LICENSE("GPL");
