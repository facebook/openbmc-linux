/********************************************************************************
* File Name     : ast_video.c
* Author         : Ryan Chen
* Description   : AST Video Engine Controller
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
*
*   Version      : 1.0
*   History      : 
*      1. 2013/04/30 Ryan Chen create this file 
*    
********************************************************************************/

#include <linux/slab.h>
#include <linux/sched.h>

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <linux/hwmon-sysfs.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <plat/regs-video.h>
#include <mach/ast_video.h>
#include <mach/hardware.h>

//#define CONFIG_AST_VIDEO_LOCK
#define CONFIG_AUTO_MODE

//#define CONFIG_AST_VIDEO_DEBUG

#ifdef CONFIG_AST_VIDEO_DEBUG
	#define VIDEO_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define VIDEO_DBG(fmt, args...)
#endif

/***********************************************************************/
//IOCTL ..
#define VIDEOIOC_BASE       'V'

#define AST_VIDEO_IOCRTIMING		_IOR(VIDEOIOC_BASE, 0, struct timing_negotiation*)
#define AST_VIDEO_IOCWTIMING		_IOW(VIDEOIOC_BASE, 1, struct timing_negotiation*)
#define AST_VIDEO_IOCXFER			_IOWR(VIDEOIOC_BASE, 2, struct xfer_msg*)

///////////////////////////////////
#define  IOCTL_IO_READ    0x1103
#define  IOCTL_IO_WRITE   0x1104
#define  IOCTL_AUTOMODE_TRIGGER         0x1111
#define  IOCTL_PASS3_TRIGGER   0x1112
#define  IOCTL_I2C_READ        0x1113
#define  IOCTL_I2C_WRITE       0x1114

typedef struct _IO_ACCESS_DATA {
    unsigned char Type;
    unsigned long Address;
    unsigned long Data;
    unsigned long Value;
    unsigned long I2CValue;
    int      kernel_socket;
} IO_ACCESS_DATA, *PIO_ACCESS_DATA;

/***********************************************************************/
typedef struct {
	u16	HorizontalActive;
	u16	VerticalActive;
	u16	RefreshRateIndex;
	u32    PixelClock;
} INTERNAL_MODE;

INTERNAL_MODE Internal_Mode[] = {
// 1024x768
  {1024, 768, 0, 65000},
  {1024, 768, 1, 65000},
  {1024, 768, 2, 75000},
  {1024, 768, 3, 78750},
  {1024, 768, 4, 94500},
// 1280x1024
  {1280, 1024, 0, 108000},
  {1280, 1024, 1, 108000},
  {1280, 1024, 2, 135000},
  {1280, 1024, 3, 157500},      
// 1600x1200
  {1600, 1200, 0, 162000},
  {1600, 1200, 1, 162000},
  {1600, 1200, 2, 175500},
  {1600, 1200, 3, 189000},
  {1600, 1200, 4, 202500},
  {1600, 1200, 5, 229500},
// 1920x1200 reduce blank
  {1920, 1200, 0, 157000},
  {1920, 1200, 1, 157000},
};

typedef struct _TRANSFER_HEADER {
    u32	Data_Length;
    u32	Blocks_Changed;
    u16	User_Width;
    u16	User_Height;
    u16	Source_Width;
    u16	Source_Height;
    u8		RC4_Enable;
    u8      RC4_Reset;
    u8      Y_Table;
    u8      UV_Table;
    u8      Mode_420;
    u8      Direct_Mode;
    u8      VQ_Mode;
    u8      Disable_VGA;
    u8      Differential_Enable;
    u8      Auto_Mode;
    u8      VGA_Status;
    u8      RC4State;
    u8      Advance_Table;
} TRANSFER_HEADER, *PTRANSFER_HEADER;

static struct fbinfo
{
	u16		x;
	u16		y;
	u8	color_mode;	//0:NON, 1:EGA, 2:VGA, 3:15bpp, 4:16bpp, 5:32bpp
	u32	PixelClock;
};

//For Socket Transfer head formate ..
static struct compress_header {
	u32 data_len;
	u32 block_changed;
	u16	user_width;
	u16	user_height;
	u8	first_frame;
	u8	compress_type;
	u8	trigger_mode;
	u8	data_format;
	u8	mode;
	u8	VQMode;
	u8	Y_JPEGTableSelector;
	u8	UV_JPEGTableSelector;
	u8	AdvanceTableSelector;
	u8	Visual_Lossless;
};

static struct ast_video_data {
	struct device		*misc_dev;
	void __iomem		*reg_base;			/* virtual */
	int 	irq;				//Video IRQ number 
//	compress_header	
	struct compress_header			compress_mode;
        phys_addr_t             *stream_phy;            /* phy */
        u32                             *stream_virt;           /* virt */
        phys_addr_t             *buff0_phy;             /* phy */
        u32                             *buff0_virt;            /* virt */
        phys_addr_t             *buff1_phy;             /* phy */
        u32                             *buff1_virt;            /* virt */
        phys_addr_t             *bcd_phy;               /* phy */
        u32                             *bcd_virt;              /* virt */
        phys_addr_t             *jpeg_phy;              /* phy */
        u32                             *jpeg_virt;             /* virt */
        phys_addr_t             *jpeg_buf0_phy;              /* phy */
        u32                             *jpeg_buf0_virt;             /* virt */
        phys_addr_t             *jpeg_tbl_phy;          /* phy */
        u32                             *jpeg_tbl_virt;         /* virt */

	//config 
	video_source	input_source;	
	u8	rc4_enable;
	u8 EncodeKeys[256];
	u8	scaling;
		
//JPEG 
	u32		video_mem_size;			/* phy size*/		
	
	wait_queue_head_t 	queue;	

	u32 flag;
	wait_queue_head_t 	video_wq;	

	u32 thread_flag;
	struct task_struct 		*thread_task;



	struct fbinfo					src_fbinfo;
	struct fbinfo					dest_fbinfo;
	struct completion				complete;	
	u32		sts;
	u8		direct_mode;
	u8		stage;
	struct ast_video_plat_data		*plat_data;		
	u32 	bandwidth;
	struct mutex lock;	

        bool is_open;

	
        spinlock_t      video_state_lock;               /* Serializing lock */
	struct fasync_struct *async_queue;		
	
};

//  RC4 structure
struct rc4_state
{
    int x;
    int y;
    int m[256];
};
 
DECLARE_WAIT_QUEUE_HEAD (my_queue);


int  Mode_Changed = 0, Capture_Ready = 0, Compression_Ready = 0, Mode_Changed_Flag = 0;

static inline void
ast_video_write(struct ast_video_data *ast_video, u32 val, u32 reg)
{
//	VIDEO_DBG("write offset: %x, val: %x \n",reg,val);
#ifdef CONFIG_AST_VIDEO_LOCK
	//unlock 
	writel(VIDEO_PROTECT_UNLOCK, ast_video->reg_base);
	writel(val, ast_video->reg_base + reg);
	//lock
	writel(0xaa,ast_video->reg_base);	
#else
	//Video is lock after reset, need always unlock 
	//unlock 
	writel(VIDEO_PROTECT_UNLOCK, ast_video->reg_base);
	writel(val, ast_video->reg_base + reg);
#endif	
}

static inline u32
ast_video_read(struct ast_video_data *ast_video, u32 reg)
{
	u32 val = readl(ast_video->reg_base + reg);
//	VIDEO_DBG("read offset: %x, val: %x \n",reg,val);
	return val;
}

/************************************************ JPEG ***************************************************************************************/
void ast_init_jpeg_table(struct ast_video_data *ast_video)
{
	int i=0;
	int base=0;
	//JPEG header default value:
	for(i = 0; i<12; i++) {
		base = (1024*i);
		ast_video->jpeg_tbl_virt[base + 0] = 0xE0FFD8FF;
		ast_video->jpeg_tbl_virt[base + 1] = 0x464A1000;
		ast_video->jpeg_tbl_virt[base + 2] = 0x01004649;
		ast_video->jpeg_tbl_virt[base + 3] = 0x60000101;
		ast_video->jpeg_tbl_virt[base + 4] = 0x00006000;
		ast_video->jpeg_tbl_virt[base + 5] = 0x0F00FEFF;
		ast_video->jpeg_tbl_virt[base + 6] = 0x00002D05;
		ast_video->jpeg_tbl_virt[base + 7] = 0x00000000;
		ast_video->jpeg_tbl_virt[base + 8] = 0x00000000;
		ast_video->jpeg_tbl_virt[base + 9] = 0x00DBFF00;
		ast_video->jpeg_tbl_virt[base + 44] = 0x081100C0;
		ast_video->jpeg_tbl_virt[base + 45] = 0x00000000;
		ast_video->jpeg_tbl_virt[base + 47] = 0x03011102;
		ast_video->jpeg_tbl_virt[base + 48] = 0xC4FF0111;
		ast_video->jpeg_tbl_virt[base + 49] = 0x00001F00;
		ast_video->jpeg_tbl_virt[base + 50] = 0x01010501;
		ast_video->jpeg_tbl_virt[base + 51] = 0x01010101;
		ast_video->jpeg_tbl_virt[base + 52] = 0x00000000;
		ast_video->jpeg_tbl_virt[base + 53] = 0x00000000;
		ast_video->jpeg_tbl_virt[base + 54] = 0x04030201;
		ast_video->jpeg_tbl_virt[base + 55] = 0x08070605;
		ast_video->jpeg_tbl_virt[base + 56] = 0xFF0B0A09;
		ast_video->jpeg_tbl_virt[base + 57] = 0x10B500C4;
		ast_video->jpeg_tbl_virt[base + 58] = 0x03010200;
		ast_video->jpeg_tbl_virt[base + 59] = 0x03040203;
		ast_video->jpeg_tbl_virt[base + 60] = 0x04040505;
		ast_video->jpeg_tbl_virt[base + 61] = 0x7D010000;
		ast_video->jpeg_tbl_virt[base + 62] = 0x00030201;
		ast_video->jpeg_tbl_virt[base + 63] = 0x12051104;
		ast_video->jpeg_tbl_virt[base + 64] = 0x06413121;
		ast_video->jpeg_tbl_virt[base + 65] = 0x07615113;
		ast_video->jpeg_tbl_virt[base + 66] = 0x32147122;
		ast_video->jpeg_tbl_virt[base + 67] = 0x08A19181;
		ast_video->jpeg_tbl_virt[base + 68] = 0xC1B14223;
		ast_video->jpeg_tbl_virt[base + 69] = 0xF0D15215;
		ast_video->jpeg_tbl_virt[base + 70] = 0x72623324;
		ast_video->jpeg_tbl_virt[base + 71] = 0x160A0982;
		ast_video->jpeg_tbl_virt[base + 72] = 0x1A191817;
		ast_video->jpeg_tbl_virt[base + 73] = 0x28272625;
		ast_video->jpeg_tbl_virt[base + 74] = 0x35342A29;
		ast_video->jpeg_tbl_virt[base + 75] = 0x39383736;
		ast_video->jpeg_tbl_virt[base + 76] = 0x4544433A;
		ast_video->jpeg_tbl_virt[base + 77] = 0x49484746;
		ast_video->jpeg_tbl_virt[base + 78] = 0x5554534A;
		ast_video->jpeg_tbl_virt[base + 79] = 0x59585756;
		ast_video->jpeg_tbl_virt[base + 80] = 0x6564635A;
		ast_video->jpeg_tbl_virt[base + 81] = 0x69686766;
		ast_video->jpeg_tbl_virt[base + 82] = 0x7574736A;
		ast_video->jpeg_tbl_virt[base + 83] = 0x79787776;
		ast_video->jpeg_tbl_virt[base + 84] = 0x8584837A;
		ast_video->jpeg_tbl_virt[base + 85] = 0x89888786;
		ast_video->jpeg_tbl_virt[base + 86] = 0x9493928A;
		ast_video->jpeg_tbl_virt[base + 87] = 0x98979695;
		ast_video->jpeg_tbl_virt[base + 88] = 0xA3A29A99;
		ast_video->jpeg_tbl_virt[base + 89] = 0xA7A6A5A4;
		ast_video->jpeg_tbl_virt[base + 90] = 0xB2AAA9A8;
		ast_video->jpeg_tbl_virt[base + 91] = 0xB6B5B4B3;
		ast_video->jpeg_tbl_virt[base + 92] = 0xBAB9B8B7;
		ast_video->jpeg_tbl_virt[base + 93] = 0xC5C4C3C2;
		ast_video->jpeg_tbl_virt[base + 94] = 0xC9C8C7C6;
		ast_video->jpeg_tbl_virt[base + 95] = 0xD4D3D2CA;
		ast_video->jpeg_tbl_virt[base + 96] = 0xD8D7D6D5;
		ast_video->jpeg_tbl_virt[base + 97] = 0xE2E1DAD9;
		ast_video->jpeg_tbl_virt[base + 98] = 0xE6E5E4E3;
		ast_video->jpeg_tbl_virt[base + 99] = 0xEAE9E8E7;
		ast_video->jpeg_tbl_virt[base + 100] = 0xF4F3F2F1;
		ast_video->jpeg_tbl_virt[base + 101] = 0xF8F7F6F5;
		ast_video->jpeg_tbl_virt[base + 102] = 0xC4FFFAF9;
		ast_video->jpeg_tbl_virt[base + 103] = 0x00011F00;
		ast_video->jpeg_tbl_virt[base + 104] = 0x01010103;
		ast_video->jpeg_tbl_virt[base + 105] = 0x01010101;
		ast_video->jpeg_tbl_virt[base + 106] = 0x00000101;
		ast_video->jpeg_tbl_virt[base + 107] = 0x00000000;
		ast_video->jpeg_tbl_virt[base + 108] = 0x04030201;
		ast_video->jpeg_tbl_virt[base + 109] = 0x08070605;
		ast_video->jpeg_tbl_virt[base + 110] = 0xFF0B0A09;
		ast_video->jpeg_tbl_virt[base + 111] = 0x11B500C4;
		ast_video->jpeg_tbl_virt[base + 112] = 0x02010200;
		ast_video->jpeg_tbl_virt[base + 113] = 0x04030404;
		ast_video->jpeg_tbl_virt[base + 114] = 0x04040507;
		ast_video->jpeg_tbl_virt[base + 115] = 0x77020100;
		ast_video->jpeg_tbl_virt[base + 116] = 0x03020100;
		ast_video->jpeg_tbl_virt[base + 117] = 0x21050411;
		ast_video->jpeg_tbl_virt[base + 118] = 0x41120631;
		ast_video->jpeg_tbl_virt[base + 119] = 0x71610751;
		ast_video->jpeg_tbl_virt[base + 120] = 0x81322213;
		ast_video->jpeg_tbl_virt[base + 121] = 0x91421408;
		ast_video->jpeg_tbl_virt[base + 122] = 0x09C1B1A1;
		ast_video->jpeg_tbl_virt[base + 123] = 0xF0523323;
		ast_video->jpeg_tbl_virt[base + 124] = 0xD1726215;
		ast_video->jpeg_tbl_virt[base + 125] = 0x3424160A;
		ast_video->jpeg_tbl_virt[base + 126] = 0x17F125E1;
		ast_video->jpeg_tbl_virt[base + 127] = 0x261A1918;
		ast_video->jpeg_tbl_virt[base + 128] = 0x2A292827;
		ast_video->jpeg_tbl_virt[base + 129] = 0x38373635;
		ast_video->jpeg_tbl_virt[base + 130] = 0x44433A39;
		ast_video->jpeg_tbl_virt[base + 131] = 0x48474645;
		ast_video->jpeg_tbl_virt[base + 132] = 0x54534A49;
		ast_video->jpeg_tbl_virt[base + 133] = 0x58575655;
		ast_video->jpeg_tbl_virt[base + 134] = 0x64635A59;
		ast_video->jpeg_tbl_virt[base + 135] = 0x68676665;
		ast_video->jpeg_tbl_virt[base + 136] = 0x74736A69;
		ast_video->jpeg_tbl_virt[base + 137] = 0x78777675;
		ast_video->jpeg_tbl_virt[base + 138] = 0x83827A79;
		ast_video->jpeg_tbl_virt[base + 139] = 0x87868584;
		ast_video->jpeg_tbl_virt[base + 140] = 0x928A8988;
		ast_video->jpeg_tbl_virt[base + 141] = 0x96959493;
		ast_video->jpeg_tbl_virt[base + 142] = 0x9A999897;
		ast_video->jpeg_tbl_virt[base + 143] = 0xA5A4A3A2;
		ast_video->jpeg_tbl_virt[base + 144] = 0xA9A8A7A6;
		ast_video->jpeg_tbl_virt[base + 145] = 0xB4B3B2AA;
		ast_video->jpeg_tbl_virt[base + 146] = 0xB8B7B6B5;
		ast_video->jpeg_tbl_virt[base + 147] = 0xC3C2BAB9;
		ast_video->jpeg_tbl_virt[base + 148] = 0xC7C6C5C4;
		ast_video->jpeg_tbl_virt[base + 149] = 0xD2CAC9C8;
		ast_video->jpeg_tbl_virt[base + 150] = 0xD6D5D4D3;
		ast_video->jpeg_tbl_virt[base + 151] = 0xDAD9D8D7;
		ast_video->jpeg_tbl_virt[base + 152] = 0xE5E4E3E2;
		ast_video->jpeg_tbl_virt[base + 153] = 0xE9E8E7E6;
		ast_video->jpeg_tbl_virt[base + 154] = 0xF4F3F2EA;
		ast_video->jpeg_tbl_virt[base + 155] = 0xF8F7F6F5;
		ast_video->jpeg_tbl_virt[base + 156] = 0xDAFFFAF9;
		ast_video->jpeg_tbl_virt[base + 157] = 0x01030C00;
		ast_video->jpeg_tbl_virt[base + 158] = 0x03110200;
		ast_video->jpeg_tbl_virt[base + 159] = 0x003F0011;

		//Table 0
		if (i==0) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x0D140043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x0C0F110F;
			ast_video->jpeg_tbl_virt[base + 12] = 0x11101114;
			ast_video->jpeg_tbl_virt[base + 13] = 0x17141516;
			ast_video->jpeg_tbl_virt[base + 14] = 0x1E20321E;
			ast_video->jpeg_tbl_virt[base + 15] = 0x3D1E1B1B;
			ast_video->jpeg_tbl_virt[base + 16] = 0x32242E2B;
			ast_video->jpeg_tbl_virt[base + 17] = 0x4B4C3F48;
			ast_video->jpeg_tbl_virt[base + 18] = 0x44463F47;
			ast_video->jpeg_tbl_virt[base + 19] = 0x61735A50;
			ast_video->jpeg_tbl_virt[base + 20] = 0x566C5550;
			ast_video->jpeg_tbl_virt[base + 21] = 0x88644644;
			ast_video->jpeg_tbl_virt[base + 22] = 0x7A766C65;
			ast_video->jpeg_tbl_virt[base + 23] = 0x4D808280;
			ast_video->jpeg_tbl_virt[base + 24] = 0x8C978D60;
			ast_video->jpeg_tbl_virt[base + 25] = 0x7E73967D;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF7B80;
			ast_video->jpeg_tbl_virt[base + 27] = 0x1F014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x272D2121;
			ast_video->jpeg_tbl_virt[base + 29] = 0x3030582D;
			ast_video->jpeg_tbl_virt[base + 30] = 0x697BB958;
			ast_video->jpeg_tbl_virt[base + 31] = 0xB8B9B97B;
			ast_video->jpeg_tbl_virt[base + 32] = 0xB9B8A6A6;
			ast_video->jpeg_tbl_virt[base + 33] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 34] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 35] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 36] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 37] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 38] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 39] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 40] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 41] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 42] = 0xB9B9B9B9;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFFB9B9B9;
		}
		//Table 1
		if (i==1) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x0C110043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x0A0D0F0D;
			ast_video->jpeg_tbl_virt[base + 12] = 0x0F0E0F11;
			ast_video->jpeg_tbl_virt[base + 13] = 0x14111213;
			ast_video->jpeg_tbl_virt[base + 14] = 0x1A1C2B1A;
			ast_video->jpeg_tbl_virt[base + 15] = 0x351A1818;
			ast_video->jpeg_tbl_virt[base + 16] = 0x2B1F2826;
			ast_video->jpeg_tbl_virt[base + 17] = 0x4142373F;
			ast_video->jpeg_tbl_virt[base + 18] = 0x3C3D373E;
			ast_video->jpeg_tbl_virt[base + 19] = 0x55644E46;
			ast_video->jpeg_tbl_virt[base + 20] = 0x4B5F4A46;
			ast_video->jpeg_tbl_virt[base + 21] = 0x77573D3C;
			ast_video->jpeg_tbl_virt[base + 22] = 0x6B675F58;
			ast_video->jpeg_tbl_virt[base + 23] = 0x43707170;
			ast_video->jpeg_tbl_virt[base + 24] = 0x7A847B54;
			ast_video->jpeg_tbl_virt[base + 25] = 0x6E64836D;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF6C70;
			ast_video->jpeg_tbl_virt[base + 27] = 0x1B014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x22271D1D;
			ast_video->jpeg_tbl_virt[base + 29] = 0x2A2A4C27;
			ast_video->jpeg_tbl_virt[base + 30] = 0x5B6BA04C;
			ast_video->jpeg_tbl_virt[base + 31] = 0xA0A0A06B;
			ast_video->jpeg_tbl_virt[base + 32] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 33] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 34] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 35] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 36] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 37] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 38] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 39] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 40] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 41] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 42] = 0xA0A0A0A0;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFFA0A0A0;
		}
		//Table 2
		if (i==2) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x090E0043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x090A0C0A;
			ast_video->jpeg_tbl_virt[base + 12] = 0x0C0B0C0E;
			ast_video->jpeg_tbl_virt[base + 13] = 0x110E0F10;
			ast_video->jpeg_tbl_virt[base + 14] = 0x15172415;
			ast_video->jpeg_tbl_virt[base + 15] = 0x2C151313;
			ast_video->jpeg_tbl_virt[base + 16] = 0x241A211F;
			ast_video->jpeg_tbl_virt[base + 17] = 0x36372E34;
			ast_video->jpeg_tbl_virt[base + 18] = 0x31322E33;
			ast_video->jpeg_tbl_virt[base + 19] = 0x4653413A;
			ast_video->jpeg_tbl_virt[base + 20] = 0x3E4E3D3A;
			ast_video->jpeg_tbl_virt[base + 21] = 0x62483231;
			ast_video->jpeg_tbl_virt[base + 22] = 0x58564E49;
			ast_video->jpeg_tbl_virt[base + 23] = 0x385D5E5D;
			ast_video->jpeg_tbl_virt[base + 24] = 0x656D6645;
			ast_video->jpeg_tbl_virt[base + 25] = 0x5B536C5A;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF595D;
			ast_video->jpeg_tbl_virt[base + 27] = 0x16014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x1C201818;
			ast_video->jpeg_tbl_virt[base + 29] = 0x22223F20;
			ast_video->jpeg_tbl_virt[base + 30] = 0x4B58853F;
			ast_video->jpeg_tbl_virt[base + 31] = 0x85858558;
			ast_video->jpeg_tbl_virt[base + 32] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 33] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 34] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 35] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 36] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 37] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 38] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 39] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 40] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 41] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 42] = 0x85858585;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF858585;
		}
		//Table 3
		if (i==3) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x070B0043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x07080A08;
			ast_video->jpeg_tbl_virt[base + 12] = 0x0A090A0B;
			ast_video->jpeg_tbl_virt[base + 13] = 0x0D0B0C0C;
			ast_video->jpeg_tbl_virt[base + 14] = 0x11121C11;
			ast_video->jpeg_tbl_virt[base + 15] = 0x23110F0F;
			ast_video->jpeg_tbl_virt[base + 16] = 0x1C141A19;
			ast_video->jpeg_tbl_virt[base + 17] = 0x2B2B2429;
			ast_video->jpeg_tbl_virt[base + 18] = 0x27282428;
			ast_video->jpeg_tbl_virt[base + 19] = 0x3842332E;
			ast_video->jpeg_tbl_virt[base + 20] = 0x313E302E;
			ast_video->jpeg_tbl_virt[base + 21] = 0x4E392827;
			ast_video->jpeg_tbl_virt[base + 22] = 0x46443E3A;
			ast_video->jpeg_tbl_virt[base + 23] = 0x2C4A4A4A;
			ast_video->jpeg_tbl_virt[base + 24] = 0x50565137;
			ast_video->jpeg_tbl_virt[base + 25] = 0x48425647;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF474A;
			ast_video->jpeg_tbl_virt[base + 27] = 0x12014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x161A1313;
			ast_video->jpeg_tbl_virt[base + 29] = 0x1C1C331A;
			ast_video->jpeg_tbl_virt[base + 30] = 0x3D486C33;
			ast_video->jpeg_tbl_virt[base + 31] = 0x6C6C6C48;
			ast_video->jpeg_tbl_virt[base + 32] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 33] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 34] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 35] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 36] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 37] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 38] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 39] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 40] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 41] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 42] = 0x6C6C6C6C;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF6C6C6C;
		}
		//Table 4
		if (i==4) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x06090043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x05060706;
			ast_video->jpeg_tbl_virt[base + 12] = 0x07070709;
			ast_video->jpeg_tbl_virt[base + 13] = 0x0A09090A;
			ast_video->jpeg_tbl_virt[base + 14] = 0x0D0E160D;
			ast_video->jpeg_tbl_virt[base + 15] = 0x1B0D0C0C;
			ast_video->jpeg_tbl_virt[base + 16] = 0x16101413;
			ast_video->jpeg_tbl_virt[base + 17] = 0x21221C20;
			ast_video->jpeg_tbl_virt[base + 18] = 0x1E1F1C20;
			ast_video->jpeg_tbl_virt[base + 19] = 0x2B332824;
			ast_video->jpeg_tbl_virt[base + 20] = 0x26302624;
			ast_video->jpeg_tbl_virt[base + 21] = 0x3D2D1F1E;
			ast_video->jpeg_tbl_virt[base + 22] = 0x3735302D;
			ast_video->jpeg_tbl_virt[base + 23] = 0x22393A39;
			ast_video->jpeg_tbl_virt[base + 24] = 0x3F443F2B;
			ast_video->jpeg_tbl_virt[base + 25] = 0x38334338;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF3739;
			ast_video->jpeg_tbl_virt[base + 27] = 0x0D014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x11130E0E;
			ast_video->jpeg_tbl_virt[base + 29] = 0x15152613;
			ast_video->jpeg_tbl_virt[base + 30] = 0x2D355026;
			ast_video->jpeg_tbl_virt[base + 31] = 0x50505035;
			ast_video->jpeg_tbl_virt[base + 32] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 33] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 34] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 35] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 36] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 37] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 38] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 39] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 40] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 41] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 42] = 0x50505050;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF505050;
		}
		//Table 5
		if (i==5) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x04060043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x03040504;
			ast_video->jpeg_tbl_virt[base + 12] = 0x05040506;
			ast_video->jpeg_tbl_virt[base + 13] = 0x07060606;
			ast_video->jpeg_tbl_virt[base + 14] = 0x09090F09;
			ast_video->jpeg_tbl_virt[base + 15] = 0x12090808;
			ast_video->jpeg_tbl_virt[base + 16] = 0x0F0A0D0D;
			ast_video->jpeg_tbl_virt[base + 17] = 0x16161315;
			ast_video->jpeg_tbl_virt[base + 18] = 0x14151315;
			ast_video->jpeg_tbl_virt[base + 19] = 0x1D221B18;
			ast_video->jpeg_tbl_virt[base + 20] = 0x19201918;
			ast_video->jpeg_tbl_virt[base + 21] = 0x281E1514;
			ast_video->jpeg_tbl_virt[base + 22] = 0x2423201E;
			ast_video->jpeg_tbl_virt[base + 23] = 0x17262726;
			ast_video->jpeg_tbl_virt[base + 24] = 0x2A2D2A1C;
			ast_video->jpeg_tbl_virt[base + 25] = 0x25222D25;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF2526;
			ast_video->jpeg_tbl_virt[base + 27] = 0x09014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x0B0D0A0A;
			ast_video->jpeg_tbl_virt[base + 29] = 0x0E0E1A0D;
			ast_video->jpeg_tbl_virt[base + 30] = 0x1F25371A;
			ast_video->jpeg_tbl_virt[base + 31] = 0x37373725;
			ast_video->jpeg_tbl_virt[base + 32] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 33] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 34] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 35] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 36] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 37] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 38] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 39] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 40] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 41] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 42] = 0x37373737;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF373737;
		}
		//Table 6
		if (i==6) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x02030043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x01020202;
			ast_video->jpeg_tbl_virt[base + 12] = 0x02020203;
			ast_video->jpeg_tbl_virt[base + 13] = 0x03030303;
			ast_video->jpeg_tbl_virt[base + 14] = 0x04040704;
			ast_video->jpeg_tbl_virt[base + 15] = 0x09040404;
			ast_video->jpeg_tbl_virt[base + 16] = 0x07050606;
			ast_video->jpeg_tbl_virt[base + 17] = 0x0B0B090A;
			ast_video->jpeg_tbl_virt[base + 18] = 0x0A0A090A;
			ast_video->jpeg_tbl_virt[base + 19] = 0x0E110D0C;
			ast_video->jpeg_tbl_virt[base + 20] = 0x0C100C0C;
			ast_video->jpeg_tbl_virt[base + 21] = 0x140F0A0A;
			ast_video->jpeg_tbl_virt[base + 22] = 0x1211100F;
			ast_video->jpeg_tbl_virt[base + 23] = 0x0B131313;
			ast_video->jpeg_tbl_virt[base + 24] = 0x1516150E;
			ast_video->jpeg_tbl_virt[base + 25] = 0x12111612;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF1213;
			ast_video->jpeg_tbl_virt[base + 27] = 0x04014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x05060505;
			ast_video->jpeg_tbl_virt[base + 29] = 0x07070D06;
			ast_video->jpeg_tbl_virt[base + 30] = 0x0F121B0D;
			ast_video->jpeg_tbl_virt[base + 31] = 0x1B1B1B12;
			ast_video->jpeg_tbl_virt[base + 32] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 33] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 34] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 35] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 36] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 37] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 38] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 39] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 40] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 41] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 42] = 0x1B1B1B1B;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF1B1B1B;
		}
		//Table 7
		if (i==7) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x01020043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 12] = 0x01010102;
			ast_video->jpeg_tbl_virt[base + 13] = 0x02020202;
			ast_video->jpeg_tbl_virt[base + 14] = 0x03030503;
			ast_video->jpeg_tbl_virt[base + 15] = 0x06030202;
			ast_video->jpeg_tbl_virt[base + 16] = 0x05030404;
			ast_video->jpeg_tbl_virt[base + 17] = 0x07070607;
			ast_video->jpeg_tbl_virt[base + 18] = 0x06070607;
			ast_video->jpeg_tbl_virt[base + 19] = 0x090B0908;
			ast_video->jpeg_tbl_virt[base + 20] = 0x080A0808;
			ast_video->jpeg_tbl_virt[base + 21] = 0x0D0A0706;
			ast_video->jpeg_tbl_virt[base + 22] = 0x0C0B0A0A;
			ast_video->jpeg_tbl_virt[base + 23] = 0x070C0D0C;
			ast_video->jpeg_tbl_virt[base + 24] = 0x0E0F0E09;
			ast_video->jpeg_tbl_virt[base + 25] = 0x0C0B0F0C;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF0C0C;
			ast_video->jpeg_tbl_virt[base + 27] = 0x03014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x03040303;
			ast_video->jpeg_tbl_virt[base + 29] = 0x04040804;
			ast_video->jpeg_tbl_virt[base + 30] = 0x0A0C1208;
			ast_video->jpeg_tbl_virt[base + 31] = 0x1212120C;
			ast_video->jpeg_tbl_virt[base + 32] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 33] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 34] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 35] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 36] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 37] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 38] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 39] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 40] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 41] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 42] = 0x12121212;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF121212;
		}
		//Table 8
		if (i==8) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x01020043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 12] = 0x01010102;
			ast_video->jpeg_tbl_virt[base + 13] = 0x02020202;
			ast_video->jpeg_tbl_virt[base + 14] = 0x03030503;
			ast_video->jpeg_tbl_virt[base + 15] = 0x06030202;
			ast_video->jpeg_tbl_virt[base + 16] = 0x05030404;
			ast_video->jpeg_tbl_virt[base + 17] = 0x07070607;
			ast_video->jpeg_tbl_virt[base + 18] = 0x06070607;
			ast_video->jpeg_tbl_virt[base + 19] = 0x090B0908;
			ast_video->jpeg_tbl_virt[base + 20] = 0x080A0808;
			ast_video->jpeg_tbl_virt[base + 21] = 0x0D0A0706;
			ast_video->jpeg_tbl_virt[base + 22] = 0x0C0B0A0A;
			ast_video->jpeg_tbl_virt[base + 23] = 0x070C0D0C;
			ast_video->jpeg_tbl_virt[base + 24] = 0x0E0F0E09;
			ast_video->jpeg_tbl_virt[base + 25] = 0x0C0B0F0C;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF0C0C;
			ast_video->jpeg_tbl_virt[base + 27] = 0x02014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x03030202;
			ast_video->jpeg_tbl_virt[base + 29] = 0x04040703;
			ast_video->jpeg_tbl_virt[base + 30] = 0x080A0F07;
			ast_video->jpeg_tbl_virt[base + 31] = 0x0F0F0F0A;
			ast_video->jpeg_tbl_virt[base + 32] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 33] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 34] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 35] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 36] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 37] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 38] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 39] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 40] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 41] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 42] = 0x0F0F0F0F;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF0F0F0F;
		}
		//Table 9
		if (i==9) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x01010043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 12] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 13] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 14] = 0x02020302;
			ast_video->jpeg_tbl_virt[base + 15] = 0x04020202;
			ast_video->jpeg_tbl_virt[base + 16] = 0x03020303;
			ast_video->jpeg_tbl_virt[base + 17] = 0x05050405;
			ast_video->jpeg_tbl_virt[base + 18] = 0x05050405;
			ast_video->jpeg_tbl_virt[base + 19] = 0x07080606;
			ast_video->jpeg_tbl_virt[base + 20] = 0x06080606;
			ast_video->jpeg_tbl_virt[base + 21] = 0x0A070505;
			ast_video->jpeg_tbl_virt[base + 22] = 0x09080807;
			ast_video->jpeg_tbl_virt[base + 23] = 0x05090909;
			ast_video->jpeg_tbl_virt[base + 24] = 0x0A0B0A07;
			ast_video->jpeg_tbl_virt[base + 25] = 0x09080B09;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF0909;
			ast_video->jpeg_tbl_virt[base + 27] = 0x02014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x02030202;
			ast_video->jpeg_tbl_virt[base + 29] = 0x03030503;
			ast_video->jpeg_tbl_virt[base + 30] = 0x07080C05;
			ast_video->jpeg_tbl_virt[base + 31] = 0x0C0C0C08;
			ast_video->jpeg_tbl_virt[base + 32] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 33] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 34] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 35] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 36] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 37] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 38] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 39] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 40] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 41] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 42] = 0x0C0C0C0C;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF0C0C0C;
		}
		//Table 10
		if (i==10) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x01010043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 12] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 13] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 14] = 0x01010201;
			ast_video->jpeg_tbl_virt[base + 15] = 0x03010101;
			ast_video->jpeg_tbl_virt[base + 16] = 0x02010202;
			ast_video->jpeg_tbl_virt[base + 17] = 0x03030303;
			ast_video->jpeg_tbl_virt[base + 18] = 0x03030303;
			ast_video->jpeg_tbl_virt[base + 19] = 0x04050404;
			ast_video->jpeg_tbl_virt[base + 20] = 0x04050404;
			ast_video->jpeg_tbl_virt[base + 21] = 0x06050303;
			ast_video->jpeg_tbl_virt[base + 22] = 0x06050505;
			ast_video->jpeg_tbl_virt[base + 23] = 0x03060606;
			ast_video->jpeg_tbl_virt[base + 24] = 0x07070704;
			ast_video->jpeg_tbl_virt[base + 25] = 0x06050706;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF0606;
			ast_video->jpeg_tbl_virt[base + 27] = 0x01014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x01020101;
			ast_video->jpeg_tbl_virt[base + 29] = 0x02020402;
			ast_video->jpeg_tbl_virt[base + 30] = 0x05060904;
			ast_video->jpeg_tbl_virt[base + 31] = 0x09090906;
			ast_video->jpeg_tbl_virt[base + 32] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 33] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 34] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 35] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 36] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 37] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 38] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 39] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 40] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 41] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 42] = 0x09090909;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF090909;
		}
		//Table 11
		if (i==11) {
			ast_video->jpeg_tbl_virt[base + 10] = 0x01010043;
			ast_video->jpeg_tbl_virt[base + 11] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 12] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 13] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 14] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 15] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 16] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 17] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 18] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 19] = 0x02020202;
			ast_video->jpeg_tbl_virt[base + 20] = 0x02020202;
			ast_video->jpeg_tbl_virt[base + 21] = 0x03020101;
			ast_video->jpeg_tbl_virt[base + 22] = 0x03020202;
			ast_video->jpeg_tbl_virt[base + 23] = 0x01030303;
			ast_video->jpeg_tbl_virt[base + 24] = 0x03030302;
			ast_video->jpeg_tbl_virt[base + 25] = 0x03020303;
			ast_video->jpeg_tbl_virt[base + 26] = 0xDBFF0403;
			ast_video->jpeg_tbl_virt[base + 27] = 0x01014300;
			ast_video->jpeg_tbl_virt[base + 28] = 0x01010101;
			ast_video->jpeg_tbl_virt[base + 29] = 0x01010201;
			ast_video->jpeg_tbl_virt[base + 30] = 0x03040602;
			ast_video->jpeg_tbl_virt[base + 31] = 0x06060604;
			ast_video->jpeg_tbl_virt[base + 32] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 33] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 34] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 35] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 36] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 37] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 38] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 39] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 40] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 41] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 42] = 0x06060606;
			ast_video->jpeg_tbl_virt[base + 43] = 0xFF060606;
		}
	}


}

static void ast_video_encryption_key_setup(struct ast_video_data *ast_video)
{
	int i, j, k, a, StringLength;
	struct rc4_state  s;
	u8 expkey[256];	
	u32     temp;

	//key expansion
	StringLength = strlen(ast_video->EncodeKeys);
//	printk("key %s , len = %d \n",ast_video->EncodeKeys, StringLength);
	for (i = 0; i < 256; i++) {
	    expkey[i] = ast_video->EncodeKeys[i % StringLength];
//		printk(" %x ", expkey[i]);
	}
//	printk("\n");
	//rc4 setup
	s.x = 0;
	s.y = 0;

	for (i = 0; i < 256; i++) {
		s.m[i] = i;
	}

	j = k = 0;
	for (i = 0; i < 256; i++) {
		a = s.m[i];
		j = (unsigned char) (j + a + expkey[k]);
		s.m[i] = s.m[j]; 
		s.m[j] = a;
		k++;
	}
	for (i = 0; i < 64; i++) {
		temp = s.m[i * 4] + ((s.m[i * 4 + 1]) << 8) + ((s.m[i * 4 + 2]) << 16) + ((s.m[i * 4 + 3]) << 24);
		ast_video_write(ast_video, temp, AST_VIDEO_ENCRYPT_SRAM + i*4);
	}

}

void ast_get_vga_scratch_info(struct ast_video_data *ast_video)
{
	u32 i, Mode_Bandwidth, RefreshRateIndex;
	u32 VGA_Scratch_350, VGA_Scratch_34C;	
	u8 RefreshRate;

	VIDEO_DBG("Get VGA SCRATCH INFO ...\n");
	VGA_Scratch_350 = ast_video_read(ast_video, AST_VIDEO_SCRATCH_350);
	//Check VGA Driver supports to write display information in scratch register
	if (SCRATCH_VGA_GET_MODE_HEADER(VGA_Scratch_350) == 0xA8) { 
		//VGA's Color Depth is 0 when real color depth is less than 8
		ast_video->src_fbinfo.color_mode = SCRATCH_VGA_GET_NEW_COLOR_MODE(VGA_Scratch_350); 
		ast_video->src_fbinfo.PixelClock = SCRATCH_VGA_GET_NEW_PIXEL_CLK(VGA_Scratch_350);
		VIDEO_DBG("New Mode Information : Color depth %x, Pixel clk %x \n", ast_video->src_fbinfo.color_mode, ast_video->src_fbinfo.PixelClock);
		//Check Bandwidth
		if (ast_video->src_fbinfo.color_mode == 0) {
			printk("Color Depth is not 16bpp or higher \n");
			printk("disable direct mode \n");
			//ast_video->direct_mode = 0;
		} else {
			//Video uses 32bits
			Mode_Bandwidth = (ast_video->src_fbinfo.PixelClock * (ast_video->src_fbinfo.color_mode + 32)) / 8; 
			printk("Mode_Bandwidth = %d\n", Mode_Bandwidth);
			if (ast_video->bandwidth < Mode_Bandwidth) {
				printk("Band Width is not enough change to direct mode\n");
				//ast_video->direct_mode = 1;
			} else {
				printk("Band Width is enough\n");
				//ast_video->direct_mode = 0;
			}
		}
		
	} else {
		//Judge if bandwidth is not enough then enable direct mode in internal VGA
		VGA_Scratch_34C = ast_video_read(ast_video, AST_VIDEO_SCRATCH_34C);
		ast_video->src_fbinfo.color_mode = SCRATCH_VGA_GET_COLOR_MODE(VGA_Scratch_34C);
		RefreshRate = SCRATCH_VGA_GET_REFLASH_RATE(VGA_Scratch_34C);
		VIDEO_DBG("Old Mode Information : color depth %x, reflash rate %x \n", ast_video->src_fbinfo.color_mode, RefreshRate);
		if (ast_video->src_fbinfo.color_mode >= VGA_15BPP_MODE) { //15bpp or higher
			for (i = 0; i < (sizeof(Internal_Mode) / sizeof(INTERNAL_MODE)); i++) { 
				if ((ast_video->src_fbinfo.x == Internal_Mode[i].HorizontalActive) && 
					(ast_video->src_fbinfo.y == Internal_Mode[i].VerticalActive) && 
					(RefreshRateIndex == Internal_Mode[i].RefreshRateIndex)) {

					ast_video->src_fbinfo.PixelClock = Internal_Mode[i].PixelClock;
					printk("Mode_PixelClock = %d\n", ast_video->src_fbinfo.PixelClock);
				}
			}
			//Calculate bandwidth required for this mode
			//Video requires pixelclock * 3, VGA requires pixelclock * bpp / 8
			Mode_Bandwidth = ast_video->src_fbinfo.PixelClock * (4 + 2 * (ast_video->src_fbinfo.color_mode - 2));
			printk("Mode_Bandwidth = %d\n", Mode_Bandwidth);
			if (ast_video->bandwidth < Mode_Bandwidth) {
				printk("Band Width is not enough\n");
				//ast_video->direct_mode = 1;
			}
			else {
				printk("Band Width is enough\n");
				//ast_video->direct_mode = 0;
			}
		} else {
			printk("!!!! Color Depth is not 16bpp or higher\n");
			//ast_video->direct_mode = 0;
		}
	}

}

static u8 ast_get_vga_signal(struct ast_video_data *ast_video)
{
	u32 VR34C, VR350, VR35C;
	u8	color_mode;
	
	VR35C = ast_video_read(ast_video, AST_VIDEO_SCRATCH_35C);
	VR35C &= 0xff000000;

	if(VR35C & (SCRATCH_VGA_PWR_STS_HSYNC | SCRATCH_VGA_PWR_STS_VSYNC)) {
		VIDEO_DBG("No VGA Signal : PWR STS %x \n", VR35C);
		return VGA_NO_SIGNAL;
	} else if (VR35C == SCRATCH_VGA_MASK_REG) {
		VIDEO_DBG("No VGA Signal : MASK %x \n", VR35C);
		return VGA_NO_SIGNAL;
	} else if (VR35C & SCRATCH_VGA_SCREEN_OFF) {
		VIDEO_DBG("No VGA Signal : Screen off %x \n", VR35C);
		return VGA_NO_SIGNAL;		
	} else if (!(VR35C & (SCRATCH_VGA_ATTRIBTE_INDEX_BIT5 | SCRATCH_VGA_MASK_REG | SCRATCH_VGA_CRT_RST | SCRATCH_VGA_RESET | SCRATCH_VGA_ENABLE ))) {
		VIDEO_DBG("NO VGA Signal : unknow %x \n", VR35C);
		return VGA_NO_SIGNAL;
	} else {
		VIDEO_DBG("VGA Signal VR35C %x \n", VR35C);
		VR350 = ast_video_read(ast_video, AST_VIDEO_SCRATCH_350);
		if(SCRATCH_VGA_GET_MODE_HEADER(VR350) == 0xA8) {
			color_mode = SCRATCH_VGA_GET_NEW_COLOR_MODE(VR350);
		} else {
			VR34C = ast_video_read(ast_video, AST_VIDEO_SCRATCH_34C);	
			if(SCRATCH_VGA_GET_COLOR_MODE(VR34C) >= VGA_15BPP_MODE) 
				color_mode = SCRATCH_VGA_GET_COLOR_MODE(VR34C);
			else
				color_mode = SCRATCH_VGA_GET_COLOR_MODE(VR34C);
		}
		if(color_mode == 0) {
			VIDEO_DBG("EGA Mode \n");
			ast_video->src_fbinfo.color_mode = EGA_MODE;
			return EGA_MODE;
		} else if (color_mode == 1) {
			VIDEO_DBG("VGA Mode \n");
			ast_video->src_fbinfo.color_mode = VGA_MODE;			
			return VGA_MODE;
		} else if (color_mode == 2) {
			VIDEO_DBG("15BPP Mode \n");
			ast_video->src_fbinfo.color_mode = VGA_15BPP_MODE;			
			return VGA_15BPP_MODE;
		} else if (color_mode == 3) {
			VIDEO_DBG("16BPP Mode \n");
			ast_video->src_fbinfo.color_mode = VGA_16BPP_MODE;						
			return VGA_16BPP_MODE;
		} else if (color_mode == 4) {
			VIDEO_DBG("32BPP Mode \n");
			ast_video->src_fbinfo.color_mode = VGA_32BPP_MODE;						
			return VGA_32BPP_MODE;
		} else {
			printk("TODO ... unknow ..\n");
			ast_video->src_fbinfo.color_mode = VGA_MODE;						
			return VGA_MODE;
		}
	
	}		
}

static void ast_video_scaling(struct ast_video_data *ast_video, u8 enable)
{
	u32 scan_line, v_factor, h_factor;
	u32 ctrl = ast_video_read(ast_video, AST_VIDEO_CTRL);
	if(enable) {
		VIDEO_DBG("Scaling Enable\n");
		//Calculate scaling factor D / S = 4096 / Factor	======> Factor = (S / D) * 4096
		v_factor = ((ast_video->src_fbinfo.y - 1) * 4096) / (ast_video->src_fbinfo.y - 1);
		if (v_factor < 4096)
			v_factor = 4096;
		if ((v_factor * (ast_video->src_fbinfo.y - 1)) != (ast_video->src_fbinfo.y - 1) * 4096)
			v_factor += 1;

		//Calculate scaling factor D / S = 4096 / Factor  ======> Factor = (S / D) * 4096
		h_factor = ((ast_video->src_fbinfo.x - 1) * 4096) / (ast_video->src_fbinfo.x - 1);
		if (h_factor < 4096)
		    h_factor = 4096;
		if ((h_factor * (ast_video->src_fbinfo.x - 1)) != (ast_video->src_fbinfo.x - 1) * 4096) 
		    h_factor += 1;

		//Down-Scaling
		ctrl |= VIDEO_CTRL_DWN_SCALING(DWN_V2);
		if(ast_video->src_fbinfo.x <= ast_video->dest_fbinfo.x) {
			ast_video_write(ast_video, 0x00101000, AST_VIDEO_SCALING0);
			ast_video_write(ast_video, 0x00101000, AST_VIDEO_SCALING1);
			ast_video_write(ast_video, 0x00101000, AST_VIDEO_SCALING2);
			ast_video_write(ast_video, 0x00101000, AST_VIDEO_SCALING3);
		} else {
			ast_video_write(ast_video, 0x08080808, AST_VIDEO_SCALING0);
			ast_video_write(ast_video, 0x08080808, AST_VIDEO_SCALING1);
			ast_video_write(ast_video, 0x08080808, AST_VIDEO_SCALING2);
			ast_video_write(ast_video, 0x08080808, AST_VIDEO_SCALING3);
		}
	} else {// 1:1
		VIDEO_DBG("Scaling Disable \n");
		ast_video->dest_fbinfo.x = ast_video->src_fbinfo.x;
		ast_video->dest_fbinfo.y = ast_video->src_fbinfo.y;
		ctrl &= ~VIDEO_CTRL_DWN_SCALING_MASK;
		v_factor = 4096;
		h_factor = 4096;
		ast_video_write(ast_video, 0x00200000, AST_VIDEO_SCALING0);
		ast_video_write(ast_video, 0x00200000, AST_VIDEO_SCALING1);
		ast_video_write(ast_video, 0x00200000, AST_VIDEO_SCALING2);
		ast_video_write(ast_video, 0x00200000, AST_VIDEO_SCALING3);
	}
	ast_video_write(ast_video, ctrl, AST_VIDEO_CTRL);

	VIDEO_DBG("Scaling factor : v : %d , h : %d \n",v_factor, h_factor);
	ast_video_write(ast_video, VIDEO_V_SCAL_FACTOR(v_factor) | VIDEO_H_SCAL_FACTOR(h_factor), AST_VIDEO_SCAL_FACTOR);

	//capture x y 
	ast_video_write(ast_video, VIDEO_CAPTURE_H(ast_video->src_fbinfo.x) |
					VIDEO_CAPTURE_V(ast_video->src_fbinfo.y)
					, AST_VIDEO_CAPTURE_WIN);

	//compression x,y
	ast_video_write(ast_video, VIDEO_COMPRESS_H(ast_video->dest_fbinfo.x) |
					VIDEO_COMPRESS_V(ast_video->dest_fbinfo.y)
					, AST_VIDEO_COMPRESS_WIN);

	// ??? why ??
	if((ast_video->src_fbinfo.x % 8) == 0)
		ast_video_write(ast_video, ast_video->src_fbinfo.x * 4
						, AST_VIDEO_SOURCE_SCAN_LINE);
	else {
		scan_line = ast_video->src_fbinfo.x;
		scan_line = scan_line + 16 - (scan_line % 16);
		scan_line = scan_line * 4;
		ast_video_write(ast_video, scan_line
						, AST_VIDEO_SOURCE_SCAN_LINE);
	}
	
}

static u8 ast_video_mode_detect_analog(struct ast_video_data *ast_video)
{
	u32 ctrl;
	
	//Check polarity (video engine prefers negative signal) bit 0, 1 is 0
	ctrl = VIDEO_FROM_EXT_SOURCE;
	ctrl &= ~VIDEO_DIRT_FATCH;

#if defined(CONFIG_ARCH_1100) || defined(CONFIG_ARCH_2050)
	ctrl |= VIDEO_18BIT_SINGLE_EDGE;
#endif

	//	2nd mode detection
	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & 
					~(VIDEO_DETECT_TRIGGER |
					VIDEO_COMPRESS_TRIGGER |
					VIDEO_AUTO_COMPRESS |
					VIDEO_INSERT_FULL_COMPRESS)
					,AST_VIDEO_SEQ_CTRL);

	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) |
						VIDEO_DETECT_TRIGGER 
						,AST_VIDEO_SEQ_CTRL);	
	//wait for detect ready...
	while (!ast_video_read(ast_video, AST_VIDEO_INT_STS) & VIDEO_MODE_DETECT_RDY);

	ast_video_write(ast_video, VIDEO_MODE_DETECT_RDY, AST_VIDEO_INT_STS);

	return 0;

}

static u8 ast_video_mode_detect_digital(struct ast_video_data *ast_video)
{
	//bit 12, 13 must be 0 , 
	//Check polarity (video engine prefers negative signal) bit 0, 1 is 0
	//detect vga signal 
	
	/* Bruce120907. According to HJ:
	** For analog video source, the HSync and VSync may not always sync together. So, VE may
	** not be able to count line count correctly. And it causes lost of first horizontal line.
	** Digital interface MUST NOT set this value.
	*/
	ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_MODE_DET1) & 
				~VIDEO_DET_HSYNC_DELAY_MASK) |
				VIDEO_DET_LONG_H_STABLE_EN
				, AST_VIDEO_MODE_DET1);	

	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_PASS_CTRL) & 
				~(VIDEO_DUAL_EDGE_MODE | 
				VIDEO_18BIT_SINGLE_EDGE | 
				VIDEO_DVO_INPUT_DELAY_MASK | 
				VIDEO_SO_VSYNC_POLARITY | 
				VIDEO_SO_HSYNC_POLARITY) 
				, AST_VIDEO_PASS_CTRL);	

	ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_MODE_DETECT) & 0xffff) |
					VIDEO_MODE_HOR_TOLER(6) |
					VIDEO_MODE_VER_TOLER(6) |
					VIDEO_MODE_HOR_STABLE(2) |
					VIDEO_MODE_VER_STABLE(2)
					, AST_VIDEO_MODE_DETECT);	

	//	2nd mode detection
	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & 
					~(VIDEO_DETECT_TRIGGER |
					VIDEO_COMPRESS_TRIGGER |
					VIDEO_AUTO_COMPRESS |
					VIDEO_INSERT_FULL_COMPRESS)
					,AST_VIDEO_SEQ_CTRL);

	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) |
						VIDEO_DETECT_TRIGGER 
						,AST_VIDEO_SEQ_CTRL);	
	//wait for detect ready...
	while (!ast_video_read(ast_video, AST_VIDEO_INT_STS) & VIDEO_MODE_DETECT_RDY);

	ast_video_write(ast_video, VIDEO_MODE_DETECT_RDY, AST_VIDEO_INT_STS);

	//Get External digital/analog input signal type
	if(ast_video_read(ast_video, AST_VIDEO_MODE_DET_STS) & VIDEO_DET_FROM_ADC)
		printk("Detect Input signal is ADC \n");
	else
		printk("Detect Input signal is Digital \n");

	return 0;
}

static void ast_video_set_compress(struct ast_video_data *ast_video)
{
	u32 compress_mode = ast_video_read(ast_video, AST_VIDEO_COMPRESS_CTRL);

    if (ast_video->compress_mode.VQMode == 1)
		compress_mode |= VIDEO_4COLOR_VQ_ENCODE;
    else
        compress_mode &= ~VIDEO_4COLOR_VQ_ENCODE;


    switch (ast_video->compress_mode.mode) {
        case 0:
			compress_mode |= VIDEO_DCT_ONLY_ENCODE;
    	    break;
        case 1:
            compress_mode &= ~(VIDEO_4COLOR_VQ_ENCODE |VIDEO_DCT_ONLY_ENCODE);
    	    break;
    	case 2:
			compress_mode |= VIDEO_4COLOR_VQ_ENCODE;
            break;
        case 3: //No one ....
//			compress_mode |= VIDEO_DCT_ONLY_ENCODE | ;
			printk("ERROR .....\n");
            break;
        default:
			printk("ERROR \n");
            compress_mode |= VIDEO_DCT_ONLY_ENCODE;
            break;
    }
	compress_mode |= VIDEO_HQ_DCT_LUM(ast_video->compress_mode.AdvanceTableSelector);	
	compress_mode |= VIDEO_HQ_DCT_CHROM((ast_video->compress_mode.AdvanceTableSelector + 16));
    compress_mode |= VIDEO_DCT_LUM(ast_video->compress_mode.Y_JPEGTableSelector);
    compress_mode |= VIDEO_DCT_CHROM(ast_video->compress_mode.UV_JPEGTableSelector);
    if (ast_video->compress_mode.Visual_Lossless == 1) 
		compress_mode |= VIDEO_HQ_ENABLE;
    else 
		compress_mode &= ~VIDEO_HQ_ENABLE;

	ast_video_write(ast_video, compress_mode, AST_VIDEO_COMPRESS_CTRL);
}

static void ast_video_set_direct_fatch(struct ast_video_data *ast_video, u8 direct_mode)
{
	u32 vga_base;
	u32 H_Start, H_End, V_Start, V_End;
	u32 ctrl = ast_video_read(ast_video, AST_VIDEO_PASS_CTRL);

	H_Start = VIDEO_GET_HSYNC_LEFT(ast_video_read(ast_video, AST_VIDEO_H_DETECT_STS));
	H_End = VIDEO_GET_HSYNC_RIGHT(ast_video_read(ast_video, AST_VIDEO_H_DETECT_STS));
	
	V_Start = VIDEO_GET_VSYNC_TOP(ast_video_read(ast_video, AST_VIDEO_V_DETECT_STS));	
	V_End = VIDEO_GET_VSYNC_BOTTOM(ast_video_read(ast_video, AST_VIDEO_V_DETECT_STS)); 	

	VIDEO_DBG("Get H_Start = %d, H_End = %d, V_Start = %d, V_End = %d\n", H_Start, H_End, V_Start, V_End);

	ast_video->src_fbinfo.x = (H_End - H_Start) + 1;
	ast_video->src_fbinfo.y = (V_End - V_Start) + 1;
	VIDEO_DBG("source : x = %d, y = %d , color mode = %x \n", ast_video->src_fbinfo.x, ast_video->src_fbinfo.y,ast_video->src_fbinfo.color_mode);

	VIDEO_DBG("mode : %d\n",direct_mode);

	if(direct_mode) {
		ctrl &= ~(VIDEO_16BPP_MODE_555 | VIDEO_16BPP_MODE | VIDEO_DVO_INPUT_DELAY_MASK);
		
		if(ast_video->src_fbinfo.color_mode == VGA_16BPP_MODE)
			ctrl |= VIDEO_16BPP_MODE | VIDEO_DIRT_FATCH | VIDEO_AUTO_FATCH | VIDEO_DVO_INPUT_DELAY(0x4);
		else {
			ctrl |= VIDEO_DIRT_FATCH | VIDEO_AUTO_FATCH | VIDEO_DVO_INPUT_DELAY(0x4);
		}
		ast_video_write(ast_video, ctrl , AST_VIDEO_PASS_CTRL);
		
		vga_base = ast_video->plat_data->get_vga_base();
		
		VIDEO_DBG("vga base = %x \n",vga_base );
		
		ast_video_write(ast_video, vga_base, AST_VIDEO_DIRECT_BASE);	
		
		ast_video_write(ast_video, VIDEO_FETCH_TIMING(0) |
						VIDEO_FETCH_LINE_OFFSET(ast_video->src_fbinfo.x * 4)
						, AST_VIDEO_DIRECT_CTRL);		
		
	} else {
		ctrl &=~(VIDEO_DIRT_FATCH | VIDEO_AUTO_FATCH);
		ctrl |= VIDEO_INTERNAL_DE | VIDEO_16BPP_MODE | VIDEO_DVO_INPUT_DELAY(0x4);
		ast_video_write(ast_video, ctrl, AST_VIDEO_PASS_CTRL); 

		ast_video_write(ast_video, VIDEO_HSYNC_PIXEL_FIRST_SET(H_Start - 1) | 
						VIDEO_HSYNC_PIXEL_LAST_SET(H_End), 
						AST_VIDEO_TIMING_H);
		ast_video_write(ast_video, VIDEO_VSYNC_PIXEL_FIRST_SET(V_Start) | 
						VIDEO_VSYNC_PIXEL_LAST_SET(V_End + 1), 
						AST_VIDEO_TIMING_V);
	
	}
}

static void ast_video_mode_detect_trigger(struct ast_video_data *ast_video)
{
	VIDEO_DBG("ast_video_mode_detect_trigger \n");

	if(!ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & VIDEO_CAPTURE_BUSY) {
		printk("Capture Eng busy !! 0x04 : %x \n", ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL));

	}
	
	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & 
						~(VIDEO_DETECT_TRIGGER | VIDEO_INPUT_MODE_CHG_WDT)
						,AST_VIDEO_SEQ_CTRL);
	udelay(1);
	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) |
						VIDEO_DETECT_TRIGGER 
						,AST_VIDEO_SEQ_CTRL);			

	while(!(ast_video_read(ast_video, AST_VIDEO_INT_STS) & VIDEO_MODE_DETECT_RDY));
	printk("int sts = %x \n",ast_video_read(ast_video, AST_VIDEO_INT_STS));

	ast_video_write(ast_video, VIDEO_MODE_DETECT_RDY, AST_VIDEO_INT_STS);			
	
}

static void ast_video_mode_detect_info(struct ast_video_data *ast_video)

{
	u32 H_Start, H_End, V_Start, V_End;

	H_Start = VIDEO_GET_HSYNC_LEFT(ast_video_read(ast_video, AST_VIDEO_H_DETECT_STS));
	H_End = VIDEO_GET_HSYNC_RIGHT(ast_video_read(ast_video, AST_VIDEO_H_DETECT_STS));
	
	V_Start = VIDEO_GET_VSYNC_TOP(ast_video_read(ast_video, AST_VIDEO_V_DETECT_STS));	
	V_End = VIDEO_GET_VSYNC_BOTTOM(ast_video_read(ast_video, AST_VIDEO_V_DETECT_STS)); 	

	VIDEO_DBG("Get H_Start = %d, H_End = %d, V_Start = %d, V_End = %d\n", H_Start, H_End, V_Start, V_End);

	ast_video->src_fbinfo.x = (H_End - H_Start) + 1;
	ast_video->src_fbinfo.y = (V_End - V_Start) + 1;
	VIDEO_DBG("source : x = %d, y = %d , color mode = %x \n", ast_video->src_fbinfo.x, ast_video->src_fbinfo.y,ast_video->src_fbinfo.color_mode);
}

static void ast_video_mode_detect(struct ast_video_data *ast_video)
{
	u32 ctrl;
	int r;			
	u8 color_mode;
	int i;
	u8 HPolarity_Positive = 0, HPolarity_Negative = 0, VPolarity_Positive = 0, VPolarity_Negative = 0;
	u8 Horizontal_Polarity = 0, Vertical_Polarity = 0;
	u32 sts;
    u32 H_Temp = 0, V_Temp = 0;

	VIDEO_DBG("\n");

//	ast_video->plat_data->ctrl_reset();
	//set input signal  and Check polarity (video engine prefers negative signal)		
	if(ast_video->input_source <= VIDEO_SOURCE_INT_CRT) {
		ast_get_vga_scratch_info(ast_video);
		ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_PASS_CTRL) &
						~VIDEO_FROM_EXT_SOURCE) |
						VIDEO_SO_VSYNC_POLARITY | VIDEO_SO_HSYNC_POLARITY,
						AST_VIDEO_PASS_CTRL); 		
	} else {
		ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_PASS_CTRL) &
						~(VIDEO_SO_VSYNC_POLARITY | VIDEO_SO_HSYNC_POLARITY)) |
						VIDEO_FROM_EXT_SOURCE
						,AST_VIDEO_PASS_CTRL); 				
	}
	
	//1st mode detect
	ast_video_mode_detect_trigger(ast_video);

	// 0 for negative, 1 for positive, hardware always use rising edge
	for(i=0; i<10; i++) {
		sts = ast_video_read(ast_video, AST_VIDEO_MODE_DET_STS);
		sts = ast_video_read(ast_video, AST_VIDEO_MODE_DET_STS) & VIDEO_DET_VSYNC_POLAR;
		if (sts == 0)
			VPolarity_Negative++;
		else
			VPolarity_Positive++;
	}
	
	if (VPolarity_Positive > VPolarity_Negative)
		Vertical_Polarity = 1;
	else
		Vertical_Polarity = 0;
	
	
	for(i=0; i<10; i++) {
		sts = ast_video_read(ast_video, AST_VIDEO_MODE_DET_STS);
		sts = ast_video_read(ast_video, AST_VIDEO_MODE_DET_STS) & VIDEO_DET_HSYNC_POLAR;
		if (sts == 0)
			HPolarity_Negative++;
		else
			HPolarity_Positive++;
	}
	
	if (HPolarity_Positive > HPolarity_Negative)
		Horizontal_Polarity = 1;
	else
		Horizontal_Polarity = 0;

	VIDEO_DBG("Get polarity \n H_pos : %d, H_Neg : %d, H_pol : %d, \n V_pos : %d, V_Neg : %d, V_pol : %d \n", 
				HPolarity_Positive, HPolarity_Negative, Horizontal_Polarity, 
				VPolarity_Positive, VPolarity_Negative, Vertical_Polarity);

	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_PASS_CTRL) |
					(Vertical_Polarity * VIDEO_SO_VSYNC_POLARITY) |
					(Horizontal_Polarity * VIDEO_SO_HSYNC_POLARITY)
					, AST_VIDEO_PASS_CTRL); 	

	//2nd mode detect and get detect info
	ast_video_mode_detect_trigger(ast_video);

	if(ast_video->input_source == VIDEO_SOURCE_INT_VGA) {
		color_mode = ast_get_vga_signal(ast_video);
		if(color_mode) {
			if(color_mode >= VGA_15BPP_MODE) {
				VIDEO_DBG("Set Direct Mode \n");
				ast_video->direct_mode = 1;
				ast_video_set_direct_fatch(ast_video, 1);
			} else {
				VIDEO_DBG("Set Sync Mode \n");
				ast_video->direct_mode = 0;
				ast_video_set_direct_fatch(ast_video, 0);
			}
		}else {
			VIDEO_DBG("NO VGA Signal ..... \n");
		}
		
	} else {
		//Get External digital/analog input signal type
		if(ast_video_read(ast_video, AST_VIDEO_MODE_DET_STS) & VIDEO_DET_FROM_ADC) {
			VIDEO_DBG("Detect Input signal : Analog ... TODO ...\n");
			//....JudgeMode 
			ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_PASS_CTRL) | 
						VIDEO_INTERNAL_DE | VIDEO_EXT_ADC_ATTRIBUTE, 
						AST_VIDEO_PASS_CTRL);			
		} else {
			VIDEO_DBG("Detect Input signal : Digital ... TODO ...\n");
			//.... AutoPosition 
			ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_PASS_CTRL) & 
						~(VIDEO_INTERNAL_DE | VIDEO_EXT_ADC_ATTRIBUTE)), 
						AST_VIDEO_PASS_CTRL); 

		}
	}

#if 1
	u8 det_err = 0;
	u32 det_sts = ast_video_read(ast_video, AST_VIDEO_H_DETECT_STS);

	if(det_sts & VIDEO_NO_DISPLAY_CLOCK_DET) {
		det_err = 1;
		printk("VIDEO_NO_DISPLAY_CLOCK_DET \n");
	}
	
	if(det_sts & VIDEO_NO_ACT_DISPLAY_DET) {
		det_err = 1;	
		printk("VIDEO_NO_ACT_DISPLAY_DET \n");
	}
	
	if(det_sts & VIDEO_NO_HSYNC_DET) {
		det_err = 1;
		printk("VIDEO_NO_HSYNC_DET \n");
	}
	
	if(det_sts & VIDEO_NO_VSYNC_DET) {
		det_err = 1;		
		printk("VIDEO_NO_VSYNC_DET \n");
	}

	u32 sync_rdy = ast_video_read(ast_video, AST_VIDEO_MODE_DET_STS);

	if(!sync_rdy & VIDEO_DET_HSYNC_RDY) {
		det_err = 1;		
		printk("VIDEO_DET_HSYNC_RDY NO \n");
	}
	if(!sync_rdy & VIDEO_DET_VSYNC_RDY) {
		det_err = 1;		
		printk("VIDEO_DET_VSYNC_RDY NO \n");
	}

	if(det_err)
		return det_err;
	
#endif
	
	//  Enable Watchdog detection
	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) | VIDEO_INPUT_MODE_CHG_WDT,
						AST_VIDEO_SEQ_CTRL);

}

static irqreturn_t ast_video_handler(int this_irq, void *dev_id)
{
	struct ast_video_data *ast_video = dev_id;
	u32 sts = ast_video_read(ast_video, AST_VIDEO_INT_STS) & ast_video_read(ast_video, AST_VIDEO_INT_EN);
	u32 buff0, buff1;
	
	VIDEO_DBG("ISR = %x : ",sts);
	
	if (sts & VIDEO_MODE_DETECT_WDT) {
		VIDEO_DBG("VIDEO_MODE_DETECT_WDT \n");
		ast_video_write(ast_video, VIDEO_MODE_DETECT_WDT, AST_VIDEO_INT_STS);
//		ast_video->cur_state = VIDEO_MODECHANGE;
		wake_up_process(ast_video->thread_task);
	} 

	if (sts & VIDEO_MODE_DETECT_RDY) {
		VIDEO_DBG("VIDEO_MODE_DETECT_RDY !!\n");
		ast_video_write(ast_video, VIDEO_MODE_DETECT_RDY, AST_VIDEO_INT_STS);
//		ast_video->flag = VIDEO_MODE_DETECT_RDY;
		wake_up_interruptible(&ast_video->video_wq);
	}

	if (sts & VIDEO_CAPTURE_COMPLETE) {
		VIDEO_DBG("VIDEO_CAPTURE_COMPLETE \n");
		ast_video_write(ast_video, VIDEO_CAPTURE_COMPLETE, AST_VIDEO_INT_STS);
//		ast_video->flag = VIDEO_CAPTURE_COMPLETE;		
		wake_up_interruptible(&ast_video->video_wq);
	}

	if (sts & VIDEO_COMPRESS_COMPLETE) {
		VIDEO_DBG("VIDEO_COMPRESS_COMPLETE \n");
		ast_video_write(ast_video, VIDEO_COMPRESS_COMPLETE, AST_VIDEO_INT_STS);
		ast_video->flag = VIDEO_COMPRESS_COMPLETE;
		//swap		
		buff0 = ast_video_read(ast_video, AST_VIDEO_SOURCE_BUFF0);
		buff1 = ast_video_read(ast_video, AST_VIDEO_SOURCE_BUFF1);
		ast_video_write(ast_video, buff1, AST_VIDEO_SOURCE_BUFF0);
		ast_video_write(ast_video, buff0, AST_VIDEO_SOURCE_BUFF1);
		wake_up_interruptible(&ast_video->video_wq);
	}

	if (sts & VIDEO_COMPRESS_PKT_COMPLETE) {
		VIDEO_DBG("VIDEO_COMPRESS_PKT_COMPLETE TODO ..............\n");
		ast_video_write(ast_video, VIDEO_COMPRESS_PKT_COMPLETE, AST_VIDEO_INT_STS);
	}

	if (sts & VIDEO_FRAME_COMPLETE) {
		VIDEO_DBG("VIDEO_FRAME_COMPLETE TODO ...\n");
		ast_video_write(ast_video, VIDEO_FRAME_COMPLETE, AST_VIDEO_INT_STS);		
	}
	
	return IRQ_HANDLED;
}

static irqreturn_t video_interrupt(int this_irq, void *dev_id)
{
	unsigned int status;
	struct ast_video_data *ast_video = dev_id;

	status = ast_video_read(ast_video, AST_VIDEO_INT_STS);

	if(status & VIDEO_MODE_DETECT_WDT) {
		Mode_Changed = 1;
		Mode_Changed_Flag = 1;
		*(u32 *)(IO_ADDRESS(0x1e700308)) = 0xB;
		if(ast_video->async_queue)
			kill_fasync(&ast_video->async_queue, SIGIO, POLL_IN);
	}
	if (status & VIDEO_COMPRESS_COMPLETE) {
		Compression_Ready = 1;
		ast_video_write(ast_video, VIDEO_COMPRESS_COMPLETE, AST_VIDEO_INT_STS);
	}
	if (status & VIDEO_CAPTURE_COMPLETE) {
 		Capture_Ready = 1;
		ast_video_write(ast_video, VIDEO_CAPTURE_COMPLETE, AST_VIDEO_INT_STS);
 	}
	if ((Mode_Changed == 1) || ((Capture_Ready == 1) && (Compression_Ready == 1))) {
		ast_video->flag = 1;
		wake_up_interruptible (&my_queue);
	}

    return IRQ_HANDLED;
}

static void ast_video_ctrl_init(struct ast_video_data *ast_video)
{
	VIDEO_DBG("\n");

	ast_video_write(ast_video, (u32)ast_video->buff0_phy, AST_VIDEO_SOURCE_BUFF0);
	ast_video_write(ast_video, (u32)ast_video->buff1_phy, AST_VIDEO_SOURCE_BUFF1);
	ast_video_write(ast_video, (u32)ast_video->bcd_phy, AST_VIDEO_BCD_BUFF);
	ast_video_write(ast_video, (u32)ast_video->stream_phy, AST_VIDEO_STREAM_BUFF);
	ast_video_write(ast_video, (u32)ast_video->jpeg_tbl_phy, AST_VIDEO_JPEG_HEADER_BUFF);
	ast_video_write(ast_video, (u32)ast_video->jpeg_tbl_phy, AST_VM_JPEG_HEADER_BUFF);		
	ast_video_write(ast_video, (u32)ast_video->jpeg_buf0_phy, AST_VM_SOURCE_BUFF0);
	ast_video_write(ast_video, (u32)ast_video->jpeg_phy, AST_VM_COMPRESS_BUFF);
	ast_video_write(ast_video, 0, AST_VIDEO_COMPRESS_READ);

	//clr int sts
	ast_video_write(ast_video, 0xffffffff, AST_VIDEO_INT_STS);

	// =============================  JPEG init ===========================================
	ast_init_jpeg_table(ast_video);
	ast_video_write(ast_video,  VM_STREAM_PKT_SIZE(STREAM_3MB), AST_VM_STREAM_SIZE);
	ast_video_write(ast_video,  0x00080000 | VIDEO_DCT_LUM(4) | VIDEO_DCT_CHROM(4 + 16) | VIDEO_DCT_ONLY_ENCODE, AST_VM_COMPRESS_CTRL);

	//WriteMMIOLong(0x1e700238, 0x00000000);
	//WriteMMIOLong(0x1e70023c, 0x00000000);

	ast_video_write(ast_video, 0x00001E00, AST_VM_SOURCE_SCAN_LINE); //buffer pitch
	ast_video_write(ast_video, 0x00000000, 0x268);
	ast_video_write(ast_video, 0x00001234, 0x280);

	ast_video_write(ast_video, 0x00000000, AST_VM_PASS_CTRL);
	ast_video_write(ast_video, 0x00000000, AST_VM_BCD_CTRL);

	// ===============================================================================


	//Specification define bit 12:13 must always 0;
	ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_PASS_CTRL) & 
				~(VIDEO_DUAL_EDGE_MODE | VIDEO_18BIT_SINGLE_EDGE)) |
				VIDEO_DVO_INPUT_DELAY(0x4), 
				AST_VIDEO_PASS_CTRL); 

	ast_video_write(ast_video, VIDEO_STREAM_PKT_N(STREAM_32_PKTS) | 
				VIDEO_STREAM_PKT_SIZE(STREAM_128KB), AST_VIDEO_STREAM_SIZE);


	//rc4 init reset ..
	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_CTRL) | VIDEO_CTRL_RC4_RST , AST_VIDEO_CTRL);
	ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_CTRL) & ~VIDEO_CTRL_RC4_RST , AST_VIDEO_CTRL);

	//CRC/REDUCE_BIT register clear
	ast_video_write(ast_video, 0, AST_VIDEO_CRC1);
	ast_video_write(ast_video, 0, AST_VIDEO_CRC2);
	ast_video_write(ast_video, 0, AST_VIDEO_DATA_TRUNCA);
	ast_video_write(ast_video, 0, AST_VIDEO_COMPRESS_READ);

	ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_MODE_DETECT) & 0xff) |
									VIDEO_MODE_HOR_TOLER(6) |
									VIDEO_MODE_VER_TOLER(6) |
									VIDEO_MODE_HOR_STABLE(2) |
									VIDEO_MODE_VER_STABLE(2) |
									VIDEO_MODE_EDG_THROD(0x65)
									, AST_VIDEO_MODE_DETECT);	
}

static long ast_video_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int ret = 1;
	u32 ctrl = 0;
	struct miscdevice *c = fp->private_data;
	struct ast_video_data *ast_video = dev_get_drvdata(c->this_device);
	void __user *argp = (void __user *)arg;

	IO_ACCESS_DATA  Kernal_IO_Data;

	memset (&Kernal_IO_Data, 0, sizeof(IO_ACCESS_DATA));

	Kernal_IO_Data = *(IO_ACCESS_DATA *)arg;

	switch (cmd) {
		case IOCTL_IO_READ:
//			printk("READ OFFSET : %x \n",Kernal_IO_Data.Address);
            Kernal_IO_Data.Data  = *(u32 *)(IO_ADDRESS(Kernal_IO_Data.Address));
            *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
            ret = 0;
			break;

		case IOCTL_IO_WRITE:
//			printk("Write OFFSET : %x, data : %x  \n",Kernal_IO_Data.Address, Kernal_IO_Data.Data);
            *(u32 *)(IO_ADDRESS(Kernal_IO_Data.Address)) = Kernal_IO_Data.Data;			
			ret = 0;
			break;

		case IOCTL_AUTOMODE_TRIGGER:
#if 0
			ctrl = ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL);
			if(!((ctrl & VIDEO_COMPRESS_BUSY) && (ctrl & VIDEO_CAPTURE_BUSY))) {
				printk("Video busy %x \n", ctrl);
				ret = 1;
			} else {
				printk("Mode_Changed %d \n",Mode_Changed);
				if(Mode_Changed) {
					ret = Mode_Changed;
					Mode_Changed = 0;
				} else {
					ast_video_write(ast_video, ctrl & ~(VIDEO_CAPTURE_TRIGGER | VIDEO_COMPRESS_FORCE_IDLE |VIDEO_COMPRESS_TRIGGER), AST_VIDEO_SEQ_CTRL);
					ast_video_write(ast_video, ctrl | VIDEO_CAPTURE_TRIGGER | VIDEO_COMPRESS_TRIGGER, AST_VIDEO_SEQ_CTRL);
					wait_event_interruptible (my_queue, (ast_video->flag == 1));
					ast_video->flag = 0;
					printk("Mode_Changed %d \n",Mode_Changed);
					
					if (Mode_Changed) {
						ret = Mode_Changed;
						Mode_Changed = 0;
					} else 
						ret =0;
					if ((Capture_Ready == 1) && (Compression_Ready == 1)) {
						Capture_Ready = 0;
						Compression_Ready = 0;
					}
				}
			}
#else
			ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & ~(VIDEO_CAPTURE_TRIGGER | VIDEO_COMPRESS_FORCE_IDLE |VIDEO_COMPRESS_TRIGGER), AST_VIDEO_SEQ_CTRL);
			//If CPU is too fast, pleas read back and trigger 
			ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) | VIDEO_CAPTURE_TRIGGER |VIDEO_COMPRESS_TRIGGER, AST_VIDEO_SEQ_CTRL);
			wait_event_interruptible (my_queue, (ast_video->flag == 1));
    			ast_video->flag = 0;
			if (Mode_Changed == 1) {
			Mode_Changed = 0;
			}
			if ((Capture_Ready == 1) && (Compression_Ready == 1)) {
			Capture_Ready = 0;
			Compression_Ready = 0;
			}

			ret = Mode_Changed_Flag;
			if (Mode_Changed_Flag == 1) {
				Mode_Changed_Flag = 0;
				}
#endif			
		break;

		case IOCTL_PASS3_TRIGGER:
    ctrl = *(u32 *)(IO_ADDRESS(0x1e700004));
    ctrl &= 0xFFFFFFC1;
    *(u32 *)(IO_ADDRESS(0x1e700004)) = ctrl;
    barrier ();
    *(u32 *)(IO_ADDRESS(0x1e700004)) = (ctrl | 0x10);
    wait_event_interruptible (my_queue, (ast_video->flag == 1));
    ast_video->flag = 0;
    if (Mode_Changed == 1) {
	Mode_Changed = 0;
    }
    if ((Capture_Ready == 1) && (Compression_Ready == 1)) {
	Capture_Ready = 0;
	Compression_Ready = 0;
    }

    ret =  Mode_Changed_Flag;
			if (Mode_Changed_Flag == 1) {
				Mode_Changed_Flag = 0;
			}
			break;

        case IOCTL_I2C_READ:
			printk("I2C READ \n");
			break;

        case IOCTL_I2C_WRITE:
			printk("I2C Write \n");
			break;

		default:
			ret = 3;
			break;
	}
	return ret;

}

static int ast_video_fasync(int fd, struct file *file, int mode)
{
	struct miscdevice *c = file->private_data;
	struct ast_video_data *ast_video = dev_get_drvdata(c->this_device);
	return fasync_helper(fd, file, mode, &ast_video->async_queue);
}

/** @note munmap handler is done by vma close handler */
static int ast_video_mmap(struct file * file, struct vm_area_struct * vma)
{
        struct miscdevice *c = file->private_data;
        struct ast_video_data *ast_video = dev_get_drvdata(c->this_device);
        size_t size = vma->vm_end - vma->vm_start;
        vma->vm_private_data = ast_video;

        if (PAGE_ALIGN(size) > ast_video->video_mem_size) {
                        printk(KERN_ERR "required length exceed the size "
                                   "of physical sram (%x)\n", ast_video->video_mem_size);
                        return -EAGAIN;
        }

        if ((ast_video->stream_phy + (vma->vm_pgoff << PAGE_SHIFT) + size)
                > (ast_video->stream_phy + ast_video->video_mem_size)) {
                        printk(KERN_ERR "required sram range exceed the size "
                                   "of phisical sram\n");
                        return -EAGAIN;
        }

        vma->vm_flags |= VM_IO;
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        if (io_remap_pfn_range(vma, vma->vm_start,
                        ((u32)ast_video->stream_phy >> PAGE_SHIFT),
                        size,
                        vma->vm_page_prot)) {
                printk(KERN_ERR "remap_pfn_range faile at %s()\n", __func__);
                return -EAGAIN;
        }

        return 0;
}

static int ast_video_open(struct inode *inode, struct file *file)
{
        struct miscdevice *c = file->private_data;
        struct ast_video_data *ast_video = dev_get_drvdata(c->this_device);

        VIDEO_DBG("\n");
        spin_lock(&ast_video->video_state_lock);

        if (ast_video->is_open) {
                spin_unlock(&ast_video->video_state_lock);
                return -EBUSY;
        }

        ast_video->is_open = true;
//        wake_up_process(ast_video->thread_task);
        spin_unlock(&ast_video->video_state_lock);

        return 0;

}

static int ast_video_release(struct inode *inode, struct file *file)
{
        struct miscdevice *c = file->private_data;
        struct ast_video_data *ast_video = dev_get_drvdata(c->this_device);

        VIDEO_DBG("\n");
        spin_lock(&ast_video->video_state_lock);

//        kthread_stop(ast_video->thread_task);

        ast_video->is_open = false;
        spin_unlock(&ast_video->video_state_lock);
        return 0;
}

static const struct file_operations ast_video_fops = {
	.owner 			= THIS_MODULE,
	.llseek 			= no_llseek,
	.unlocked_ioctl 	= ast_video_ioctl,
	.open 			= ast_video_open,
	.release 			= ast_video_release,
	.mmap 			= ast_video_mmap,
	.fasync			= ast_video_fasync,
};

struct miscdevice ast_video_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-video",
	.fops = &ast_video_fops,
};

/************************************************** SYS FS **************************************************************/
static ssize_t show_vga_display(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_video_data *ast_video = dev_get_drvdata(dev);
	return sprintf(buf, "%d: %s\n", ast_video->plat_data->get_vga_display(), ast_video->plat_data->get_vga_display()? "Enable":"Disable");
}

static ssize_t store_vga_display(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_video_data *ast_video = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_video->plat_data->vga_display(1);
	else
		ast_video->plat_data->vga_display(0);
	
	return count;
}

static DEVICE_ATTR(vga_display, S_IRUGO | S_IWUSR, show_vga_display, store_vga_display); 

static ssize_t store_video_reset(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_video_data *ast_video = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val) {
		ast_video->plat_data->ctrl_reset();
		//rc4 init reset ..
		ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_CTRL) | VIDEO_CTRL_RC4_RST , AST_VIDEO_CTRL);
		ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_CTRL) & ~VIDEO_CTRL_RC4_RST , AST_VIDEO_CTRL);
	}
	
	return count;
}

static DEVICE_ATTR(video_reset, S_IRUGO | S_IWUSR, NULL, store_video_reset); 

static ssize_t show_video_mode_detect(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct ast_video_data *ast_video = dev_get_drvdata(dev);

	if (ret < 0)
		return ret;

	ast_video_mode_detect_info(ast_video);

	return sprintf(buf, "%i\n", ret);
}

static ssize_t store_video_mode_detect(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	struct ast_video_data *ast_video = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_video_mode_detect(ast_video);
	
	return count;
}

static DEVICE_ATTR(video_mode_detect, S_IRUGO | S_IWUSR, show_video_mode_detect, store_video_mode_detect); 

static struct attribute *ast_video_attributes[] = {
	&dev_attr_vga_display.attr,
	&dev_attr_video_reset.attr,
	&dev_attr_video_mode_detect.attr,
#if 0	
	&dev_attr_video_jpeg_enc.dev_attr.attr,
	&dev_attr_video_src_x.dev_attr.attr,
	&dev_attr_video_src_y.dev_attr.attr,
	&dev_attr_video_scaling_en.dev_attr.attr,
	&dev_attr_video_dwn_x.dev_attr.attr,
	&dev_attr_video_dwn_y.dev_attr.attr,
	&dev_attr_video_rc4_en.dev_attr.attr,
	&dev_attr_video_rc4_key.dev_attr.attr,
#endif	
	NULL
};

static const struct attribute_group video_attribute_group = {
	.attrs = ast_video_attributes
};

/**************************   Vudeo SYSFS  **********************************************************/
enum ast_video_trigger_mode {
	VIDEO_CAPTURE_MODE = 0,
	VIDEO_COMPRESSION_MODE,
	VIDEO_BUFFER_MODE,
};

static u8 ast_get_trigger_mode(struct ast_video_data *ast_video, u8 eng_idx) 
{
	//VR0004[3:5] 00:capture/compression/buffer
	u32 mode=0;
	switch(eng_idx) {
		case 0:
			mode = ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & (VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS);
			if(mode == 0) {
				return VIDEO_CAPTURE_MODE;
			} else if(mode == VIDEO_AUTO_COMPRESS) {
				return VIDEO_COMPRESSION_MODE;
			} else if(mode == (VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS)) {
				return VIDEO_BUFFER_MODE;
			} else {
				printk("ERROR Mode \n");
			}
		case 1:
			mode = ast_video_read(ast_video, AST_VM_SEQ_CTRL) & (VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS);
			if(mode == 0) {
				return VIDEO_CAPTURE_MODE;
			} else if(mode == VIDEO_AUTO_COMPRESS) {
				return VIDEO_COMPRESSION_MODE;
			} else if(mode == (VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS)) {
				return VIDEO_BUFFER_MODE;
			} else {
				printk("ERROR Mode \n");
			}
			break;
	}
}

static void ast_set_trigger_mode(struct ast_video_data *ast_video, u8 eng_idx, u8 mode) 
{
	int i, base=0;
	//VR0004[3:5] 00/01/11:capture/frame/stream
	switch(eng_idx) {
		case 0:	//video 1 
			if(mode == VIDEO_CAPTURE_MODE) {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & ~(VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS), AST_VIDEO_SEQ_CTRL);
			} else if (mode == VIDEO_COMPRESSION_MODE) {
				ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) | VIDEO_AUTO_COMPRESS) & ~(VIDEO_CAPTURE_MULTI_FRAME) , AST_VIDEO_SEQ_CTRL);
			} else if (mode == VIDEO_BUFFER_MODE) {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) | VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS ,AST_VIDEO_SEQ_CTRL);	
			} else {
				printk("ERROR Mode \n");
			}
			break;
		case 1:	//video M
			if(mode == VIDEO_CAPTURE_MODE) {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VM_SEQ_CTRL) & ~(VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS), AST_VM_SEQ_CTRL);
			} else if (mode == VIDEO_COMPRESSION_MODE) {
				ast_video_write(ast_video, (ast_video_read(ast_video, AST_VM_SEQ_CTRL) | VIDEO_AUTO_COMPRESS) & ~(VIDEO_CAPTURE_MULTI_FRAME) , AST_VM_SEQ_CTRL);
			} else if (mode == VIDEO_BUFFER_MODE) {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VM_SEQ_CTRL) | VIDEO_CAPTURE_MULTI_FRAME | VIDEO_AUTO_COMPRESS ,AST_VM_SEQ_CTRL);	
			} else {
				printk("ERROR Mode \n");
			}
			break;
	}
}

static u8 ast_get_compress_yuv_mode(struct ast_video_data *ast_video, u8 eng_idx) 
{
	switch(eng_idx) {
		case 0:
			return VIDEO_GET_COMPRESS_FORMAT(ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL));
			break;
		case 1:
			return VIDEO_GET_COMPRESS_FORMAT(ast_video_read(ast_video, AST_VM_SEQ_CTRL));
			break;
	}			
}

static void ast_set_compress_yuv_mode(struct ast_video_data *ast_video, u8 eng_idx, u8 yuv_mode) 
{
	int i, base=0;

	switch(eng_idx) {
		case 0:	//video 1 
			if(yuv_mode) 	//YUV420
				ast_video_write(ast_video, (ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & ~VIDEO_COMPRESS_FORMAT_MASK) | VIDEO_COMPRESS_FORMAT(YUV420) , AST_VIDEO_SEQ_CTRL);
			else
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & ~VIDEO_COMPRESS_FORMAT_MASK , AST_VIDEO_SEQ_CTRL);
			break;
		case 1:	//video M
			if(yuv_mode) 	//YUV420
				ast_video_write(ast_video, (ast_video_read(ast_video, AST_VM_SEQ_CTRL) & ~VIDEO_COMPRESS_FORMAT_MASK) | VIDEO_COMPRESS_FORMAT(YUV420) , AST_VM_SEQ_CTRL);
			else
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VM_SEQ_CTRL) & ~VIDEO_COMPRESS_FORMAT_MASK, AST_VM_SEQ_CTRL);

			for(i = 0; i<12; i++) {
				base = (1024*i);
				if(yuv_mode)	//yuv420
					ast_video->jpeg_tbl_virt[base + 46] = 0x00220103; //for YUV420 mode
				else 
					ast_video->jpeg_tbl_virt[base + 46] = 0x00110103; //for YUV444 mode)
			}
			
			break;
	}
}

static u8 ast_get_compress_jpeg_mode(struct ast_video_data *ast_video, u8 eng_idx) 
{
	switch(eng_idx) {
		case 0:
			if(ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & VIDEO_COMPRESS_JPEG_MODE)
				return 1;
			else
				return 0;
			break;
		case 1:
			if(ast_video_read(ast_video, AST_VM_SEQ_CTRL) & VIDEO_COMPRESS_JPEG_MODE)
				return 1;
			else
				return 0;
			break;
	}			
}

static void ast_set_compress_jpeg_mode(struct ast_video_data *ast_video, u8 eng_idx, u8 jpeg_mode) 
{
	int i, base=0;

	switch(eng_idx) {
		case 0:	//video 1 
			if(jpeg_mode) 	
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) | VIDEO_COMPRESS_JPEG_MODE, AST_VIDEO_SEQ_CTRL);
			else
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_SEQ_CTRL) & ~VIDEO_COMPRESS_JPEG_MODE , AST_VIDEO_SEQ_CTRL);
			break;
		case 1:	//video M
			if(jpeg_mode) 	
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VM_SEQ_CTRL) | VIDEO_COMPRESS_JPEG_MODE, AST_VM_SEQ_CTRL);
			else
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VM_SEQ_CTRL) & ~VIDEO_COMPRESS_JPEG_MODE , AST_VM_SEQ_CTRL);			
			break;
	}
}

static u8 ast_get_compress_encrypt_en(struct ast_video_data *ast_video, u8 eng_idx) 
{
	switch(eng_idx) {
		case 0:
			if(ast_video_read(ast_video, AST_VIDEO_COMPRESS_CTRL) & VIDEO_ENCRYP_ENABLE)
				return 1;
			else
				return 0;
			break;
		case 1:
			if(ast_video_read(ast_video, AST_VM_COMPRESS_CTRL) & VIDEO_ENCRYP_ENABLE)
				return 1;
			else
				return 0;
			break;
	}			
}

static void ast_set_compress_encrypt_en(struct ast_video_data *ast_video, u8 eng_idx, u8 enable) 
{
	int i, base=0;

	switch(eng_idx) {
		case 0:	//video 1 
			if(enable) {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_COMPRESS_CTRL) | VIDEO_ENCRYP_ENABLE, AST_VIDEO_COMPRESS_CTRL);		
			} else {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_COMPRESS_CTRL) & ~VIDEO_ENCRYP_ENABLE, AST_VIDEO_COMPRESS_CTRL);
			}
		case 1:	//video M
			if(enable) {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VM_COMPRESS_CTRL) | VIDEO_ENCRYP_ENABLE, AST_VIDEO_COMPRESS_CTRL);		
			} else {
				ast_video_write(ast_video, ast_video_read(ast_video, AST_VM_COMPRESS_CTRL) & ~VIDEO_ENCRYP_ENABLE, AST_VIDEO_COMPRESS_CTRL);
			}
	}
}

static u8 *ast_get_compress_encrypt_key(struct ast_video_data *ast_video, u8 eng_idx) 
{
	switch(eng_idx) {
		case 0:
			return ast_video->EncodeKeys;
			break;
		case 1:
			return ast_video->EncodeKeys;
			break;
	}			
}

static void ast_set_compress_encrypt_key(struct ast_video_data *ast_video, u8 eng_idx, u8 *key) 
{
	int i, base=0;

	switch(eng_idx) {
		case 0:	//video 1 
			memset(ast_video->EncodeKeys, 0, 256);
			//due to system have enter key must be remove
			memcpy(ast_video->EncodeKeys, key, strlen(key) - 1);
			ast_video_encryption_key_setup(ast_video);
			break;
		case 1:	//video M
			break;		
	}
}

static u8 ast_get_compress_encrypt_mode(struct ast_video_data *ast_video) 
{
	if(ast_video_read(ast_video, AST_VIDEO_CTRL) & VIDEO_CTRL_CRYPTO_AES)
		return 1;
	else
		return 0;
}

static void ast_set_compress_encrypt_mode(struct ast_video_data *ast_video, u8 mode) 
{
	if(mode)
		ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_CTRL) | VIDEO_CTRL_CRYPTO_AES, AST_VIDEO_CTRL);
	else
		ast_video_write(ast_video, ast_video_read(ast_video, AST_VIDEO_CTRL) & ~VIDEO_CTRL_CRYPTO_AES, AST_VIDEO_CTRL);
}

static ssize_t 
ast_store_compress(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_video_data *ast_video = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);
//	input_val = StrToHex(sysfsbuf);		
	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0:	//compress mode
			ast_set_trigger_mode(ast_video, sensor_attr->index, input_val);
			break;
		case 1: //yuv mode
			ast_set_compress_yuv_mode(ast_video, sensor_attr->index, input_val);
			break;
		case 2: //jpeg/aspeed mode
			ast_set_compress_jpeg_mode(ast_video, sensor_attr->index, input_val);
			break;
		case 3: //
			ast_set_compress_encrypt_en(ast_video, sensor_attr->index, input_val);
			break;
		case 4: //
			ast_set_compress_encrypt_key(ast_video, sensor_attr->index, sysfsbuf);
			break;
		case 5: //
			ast_set_compress_encrypt_mode(ast_video, sensor_attr->index);
			break;

		default:
			return -EINVAL;
			break;
	}

	return count;
}

static ssize_t 
ast_show_compress(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_video_data *ast_video = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0:
			return sprintf(sysfsbuf, "%d [0:Single, 1:Frame, 2:Stream]\n", ast_get_trigger_mode(ast_video, sensor_attr->index));
			break;
		case 1: 
			return sprintf(sysfsbuf, "%d:%s \n", ast_get_compress_yuv_mode(ast_video, sensor_attr->index), ast_get_compress_yuv_mode(ast_video, sensor_attr->index) ? "YUV420":"YUV444");
			break;			
		case 2: 
			return sprintf(sysfsbuf, "%d:%s \n", ast_get_compress_jpeg_mode(ast_video, sensor_attr->index), ast_get_compress_jpeg_mode(ast_video, sensor_attr->index) ? "JPEG":"ASPEED");
			break;
		case 3:
			return sprintf(sysfsbuf, "%d:%s \n", ast_get_compress_encrypt_en(ast_video, sensor_attr->index), ast_get_compress_encrypt_en(ast_video, sensor_attr->index) ? "Enable":"Disable");
			break;			
		case 4:
			return sprintf(sysfsbuf, "%s \n", ast_get_compress_encrypt_key(ast_video, sensor_attr->index));
			break;			
		case 5:
			return sprintf(sysfsbuf, "%d:%s \n", ast_get_compress_encrypt_mode(ast_video), ast_get_compress_encrypt_mode(ast_video) ? "AES":"RC4");
			break;			
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
}

#define sysfs_compress(index) \
static SENSOR_DEVICE_ATTR_2(compress##index##_trigger_mode, S_IRUGO | S_IWUSR, \
	ast_show_compress, ast_store_compress, 0, index); \
static SENSOR_DEVICE_ATTR_2(compress##index##_yuv, S_IRUGO | S_IWUSR, \
	ast_show_compress, ast_store_compress, 1, index); \
static SENSOR_DEVICE_ATTR_2(compress##index##_jpeg, S_IRUGO | S_IWUSR, \
	ast_show_compress, ast_store_compress, 2, index); \
static SENSOR_DEVICE_ATTR_2(compress##index##_encrypt_en, S_IRUGO | S_IWUSR, \
	ast_show_compress, ast_store_compress, 3, index); \
static SENSOR_DEVICE_ATTR_2(compress##index##_encrypt_key, S_IRUGO | S_IWUSR, \
	ast_show_compress, ast_store_compress, 4, index); \
static SENSOR_DEVICE_ATTR_2(compress##index##_encrypt_mode, S_IRUGO | S_IWUSR, \
	ast_show_compress, ast_store_compress, 5, index); \
\
static struct attribute *compress##index##_attributes[] = { \
	&sensor_dev_attr_compress##index##_trigger_mode.dev_attr.attr, \
	&sensor_dev_attr_compress##index##_yuv.dev_attr.attr, \
	&sensor_dev_attr_compress##index##_jpeg.dev_attr.attr, \
	&sensor_dev_attr_compress##index##_encrypt_en.dev_attr.attr, \
	&sensor_dev_attr_compress##index##_encrypt_key.dev_attr.attr, \
	&sensor_dev_attr_compress##index##_encrypt_mode.dev_attr.attr, \
	NULL \
};

sysfs_compress(0);
sysfs_compress(1);
/************************************************** SYS FS Capture ***********************************************************/
static void ast_set_capture_win(struct ast_video_data *ast_video, u8 eng_idx, u8 win_x, u8 win_y)
{
}

static void ast_get_capture_win(struct ast_video_data *ast_video, u8 eng_idx, u8 win_x, u8 win_y)
{
}

static ssize_t 
ast_store_capture(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_video_data *ast_video = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);
//	input_val = StrToHex(sysfsbuf);		
	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0:	
//			ast_set_capture_win(ast_video, sensor_attr->index, win_x,win_y);
			break;
		case 1: 

			break;
		case 2: 

			break;
		default:
			return -EINVAL;
			break;
	}

	return count;
}

static ssize_t 
ast_show_capture(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_video_data *ast_video = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0:
			return sprintf(sysfsbuf, "%d [0:Single, 1:Frame, 2:Stream]\n", ast_get_trigger_mode(ast_video, sensor_attr->index));
			break;
		case 1: 

			break;			
		case 2: 

			break;			
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
}

static const struct attribute_group compress_attribute_groups[] = {
	{ .attrs = compress0_attributes },
	{ .attrs = compress1_attributes },	
};

#define sysfs_capture(index) \
static SENSOR_DEVICE_ATTR_2(capture##index##_win, S_IRUGO | S_IWUSR, \
	ast_show_capture, ast_store_capture, 0, index); \
\
static struct attribute *capture##index##_attributes[] = { \
	&sensor_dev_attr_capture##index##_win.dev_attr.attr, \
	NULL \
};

sysfs_capture(0);
sysfs_capture(1);

static const struct attribute_group capture_attribute_groups[] = {
	{ .attrs = capture0_attributes },
	{ .attrs = capture1_attributes },	
};

/************************************************** SYS FS End ***********************************************************/
static int ast_video_probe(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	int ret=0;
	int i;
	u8 color_mode;
	struct ast_video_data *ast_video;

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res0) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res0->start, resource_size(res0), res0->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	if(!(ast_video = kzalloc(sizeof(struct ast_video_data), GFP_KERNEL))) {
		return -ENOMEM;
		goto out;
        }
	
	ast_video->reg_base = ioremap(res0->start, resource_size(res0));
	if (!ast_video->reg_base) {
		ret = -EIO;
		goto out_region0;
	}
	
	res1 = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res1)
		return -ENODEV;
	
	if (!request_mem_region(res1->start, resource_size(res1), res1->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out_region0;
	}

	//Phy assign
	ast_video->video_mem_size = resource_size(res1);
	VIDEO_DBG("video_mem_size %d MB\n",ast_video->video_mem_size/1024/1024);
	
	ast_video->stream_phy = res1->start;
	ast_video->buff0_phy = res1->start + 0x400000;  //4M : size 10MB
	ast_video->buff1_phy = res1->start + 0xe00000;  //14M : size 10MB
	ast_video->bcd_phy = res1->start + 0x1800000;   //24M : size 1MB
	ast_video->jpeg_buf0_phy = res1->start + 0x1900000;  //25MB: size 10 MB
	ast_video->jpeg_phy = res1->start + 0x2300000;  //35MB: size 4 MB
	ast_video->jpeg_tbl_phy = res1->start + 0x2700000;      //39MB: size 1 MB

	VIDEO_DBG("\nstream_phy: %x, buff0_phy: %x, buff1_phy:%x, bcd_phy:%x \njpeg_phy:%x, jpeg_tbl_phy:%x \n",
	        (u32)ast_video->stream_phy, (u32)ast_video->buff0_phy, (u32)ast_video->buff1_phy, (u32)ast_video->bcd_phy, (u32)ast_video->jpeg_phy, (u32)ast_video->jpeg_tbl_phy);


	//virt assign
	ast_video->stream_virt = ioremap(res1->start, resource_size(res1));
	if (!ast_video->stream_virt) {
	        ret = -EIO;
	        goto out_region1;
	}

	ast_video->buff0_virt = (u32)ast_video->stream_virt + 0x400000; //4M : size 10MB
	ast_video->buff1_virt = (u32)ast_video->stream_virt + 0xe00000; //14M : size 10MB
	ast_video->bcd_virt = (u32)ast_video->stream_virt + 0x1800000;  //24M : size 4MB
	ast_video->jpeg_buf0_virt = res1->start + 0x1900000;  //25MB: size x MB
	ast_video->jpeg_virt = (u32)ast_video->stream_virt + 0x2300000; //35MB: size 4 MB
	ast_video->jpeg_tbl_virt = (u32)ast_video->stream_virt + 0x2700000;     //39MB: size 1 MB

	VIDEO_DBG("\nstream_virt: %x, buff0_virt: %x, buff1_virt:%x, bcd_virt:%x \njpeg_virt:%x, jpeg_tbl_virt:%x \n",
	        (u32)ast_video->stream_virt, (u32)ast_video->buff0_virt, (u32)ast_video->buff1_virt, (u32)ast_video->bcd_virt, (u32)ast_video->jpeg_virt, (u32)ast_video->jpeg_tbl_virt);

	memset(ast_video->stream_virt, 0, resource_size(res1));	

	ast_video->irq = platform_get_irq(pdev, 0);
	if (ast_video->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region1;
	}

	ast_video->plat_data = pdev->dev.platform_data;	

	// default config 
	ast_video->input_source = VIDEO_SOURCE_INT_VGA;
	ast_video->rc4_enable = 0;
	strcpy(ast_video->EncodeKeys, "fedcba9876543210");
	ast_video->scaling = 0;
	
	//TEST
	ast_video->plat_data->get_vga_base();

	ret = misc_register(&ast_video_misc);
	if (ret){		
		printk(KERN_ERR "VIDEO : failed to request interrupt\n");
		goto out_irq;
	}

	//bandwidth cal utilization 0.4 =  4/10 = 2/5  --> clk * 2 * buswidth(16bits) * 0.4 /8
	VIDEO_DBG("m clk = %d \n",ast_video->plat_data->get_clk());
	ast_video->bandwidth = (ast_video->plat_data->get_clk() /10) * 16;
	VIDEO_DBG("Memory Bandwidth = %d Byte/s\n", ast_video->bandwidth);

//	ast_video->timeout = 5;	
	ast_video->flag = 0;
	init_waitqueue_head(&ast_video->video_wq);

	//workqueue
	init_waitqueue_head (&my_queue);

	ret = sysfs_create_group(&pdev->dev.kobj, &video_attribute_group);
	if (ret)
		goto out_irq;


	for(i=0;i<2;i++) {
		ret = sysfs_create_group(&pdev->dev.kobj, &compress_attribute_groups[i]);
		if (ret)
			goto out_irq;
	}

	platform_set_drvdata(pdev, ast_video);
	dev_set_drvdata(ast_video_misc.this_device, ast_video);

	ast_video_ctrl_init(ast_video);

	ret = request_irq(ast_video->irq, video_interrupt, 0, "ast-video", ast_video);
	if (ret) {
		printk(KERN_INFO "VIDEO: Failed request irq %d\n", ast_video->irq);
		goto out_region1;
	}

#if 0
       ast_video->thread_task = kthread_create(ast_video_thread, (void *) ast_video, "ast-video-kthread");
        if (IS_ERR(ast_video->thread_task)) {
                printk("ast video cannot create kthread\n");
                ret = PTR_ERR(ast_video->thread_task);
                goto out_irq;
        }

	VIDEO_DBG("kthread pid: %d\n", ast_video->thread_task->pid);
#endif			


		
	printk(KERN_INFO "ast_video: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_video->irq, NULL);

out_region1:
	release_mem_region(res1->start, res1->end - res1->start + 1);	

out_region0:
	release_mem_region(res0->start, res0->end - res0->start + 1);
	
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;

}

static int ast_video_remove(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	struct ast_video_data *ast_video = platform_get_drvdata(pdev);
	VIDEO_DBG("ast_video_remove\n");

	misc_deregister(&ast_video_misc);

	free_irq(ast_video->irq, ast_video);

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_video->reg_base);

	release_mem_region(res0->start, res0->end - res0->start + 1);

	res1 = platform_get_resource(pdev, IORESOURCE_DMA, 0);

	iounmap(ast_video->stream_virt);

	release_mem_region(res1->start, res1->end - res1->start + 1);

	return 0;	
}

#ifdef CONFIG_PM
static int 
ast_video_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_video_suspend : TODO \n");
	return 0;
}

static int 
ast_video_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_video_suspend        NULL
#define ast_video_resume         NULL
#endif

static const struct platform_device_id ast_video_idtable[] = {
	{
		.name = "ast-video",
//		.driver_data = ast_video_data,
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, ast_video_idtable);

static struct platform_driver ast_video_driver = {
	.remove 		= ast_video_remove,
	.suspend        = ast_video_suspend,
	.resume         = ast_video_resume,
	.driver  	       = {
	        .name   = "ast-video",
	        .owner  = THIS_MODULE,
	},
	.id_table	= ast_video_idtable,	
};

module_platform_driver_probe(ast_video_driver, ast_video_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Video Engine driver");
MODULE_LICENSE("GPL");
