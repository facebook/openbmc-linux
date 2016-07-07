 /********************************************************************************
* File Name     : drivers/video/astfb.c
* Author         : Ryan Chen
* Description   : ASPEED Framebuffer Driver 
*
* Copyright (C) ASPEED Tech. Inc.
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

*   History      :
*    1. 2012/12/27 Ryan Chen create this file
*        
*
********************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach/map.h>
#include <plat/aspeed.h>
#include <plat/regs-crt.h>
#include <mach/ast_lcd.h>

/***********************************************************************/
struct ast_fb_cursor_set
{
	unsigned int xPos;
	unsigned int yPos;
	unsigned int width;
	unsigned int height;
	unsigned int *data;//cursor pattern tranformed to 32bpp ARGB format
};

struct ast_fb_cursor_pos
{
	unsigned int xPos;
	unsigned int yPos;
};

//IOCTL ..
#define FBIOC_BASE       'F'

#define AST_FB_CURSOR_SET		_IOW(FBIOC_BASE, 0, struct ast_fb_cursor_set*)
#define AST_FB_CURSOR_POS		_IOW(FBIOC_BASE, 1, struct ast_fb_cursor_pos*)
#define AST_FB_CURSOR_EN		_IOW(FBIOC_BASE, 2, int)
/***********************************************************************/


#ifdef CONFIG_DISPLAY_SUPPORT 
#include "display-sys.h"
#endif

#ifdef  CONFIG_DOUBLE_BUFFER
#define NUMBER_OF_BUFFERS 2
#else
#define NUMBER_OF_BUFFERS 1
#endif

////////////////////////////////////////////////////////////////
static wait_queue_head_t wq;
static int gNoPanDisplay;
static int gGUIWaitVsync;

#define ASTFB_GET_DFBINFO       _IOR(0xF3,0x00,struct astfb_dfbinfo)

//#define CONFIG_ASTFB_DEBUG

#ifdef CONFIG_ASTFB_DEBUG
	#define ASTFB_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
	#define ASTFB_DBG(fmt, args...)
#endif

struct pixel_freq_pll_data {
	u32 pixel_freq; //*10000
	u32 pll_set;
};

/**************************************************************************************************************************************
                     24MHz        # SCU28_a1[31:27] SCU28_a1[26:22] SCU28_a0[31:27] SCU28_a0[26:22] SCU28[21:13] SCU28[12:8] SCU28[7:0] SCU28_a1[31:0] SCU28_a0[31:0]
                                  # SCU1C_a1[31:27] SCU1C_a1[26:22] SCU1C_a0[31:27] SCU1C_a0[26:22] SCU1C[21:13] SCU1C[12:8] SCU1C[7:0] SCU1C_a1[31:0] SCU1C_a0[31:0]
#00: VCLK25_175 #    _1 *65  /62  # 0Eh             15h             15h             0Eh             03Dh         00h         40h        7547A040h      AB87A040h
#01: VCLK28_322 #    _1 *59  /50  # 0Eh             15h             15h             0Eh             031h         00h         3Ah        7546203Ah      AB86203Ah
#02: VCLK31_5   #    _1 *63  /48  # 0Eh             15h             15h             0Eh             02Fh         00h         3Eh        7545E03Eh      AB85E03Eh
#03: VCLK36     #    _1 *54  /36  # 0Eh             15h             15h             0Eh             023h         00h         35h        75446035h      AB846035h
#04: VCLK40     #    _1 *50  /30  # 0Eh             15h             15h             0Eh             01Dh         00h         31h        7543A031h      AB83A031h
#05: VCLK49_5   #    _1 *66  /32  # 0Eh             15h             15h             0Eh             01Fh         00h         41h        7543E041h      AB83E041h
#06: VCLK50     #    _1 *50  /24  # 0Eh             15h             15h             0Eh             017h         00h         31h        7542E031h      AB82E031h
#07: VCLK56_25  #    _1 *75  /32  # 0Eh             15h             15h             0Eh             01Fh         00h         4Ah        7543E04Ah      AB83E04Ah
#08: VCLK65     #    _1 *65  /24  # 0Eh             15h             15h             0Eh             017h         00h         40h        7542E040h      AB82E040h
#09: VCLK75     #    _1 *50  /16  # 0Eh             15h             15h             0Eh             00Fh         00h         31h        7541E031h      AB81E031h
#0A: VCLK78_75  #    _1 *59  /18  # 0Eh             15h             15h             0Eh             011h         00h         3Ah        7542203Ah      AB82203Ah
#0B: VCLK94_5   #    _1 *63  /16  # 0Eh             15h             15h             0Eh             00Fh         00h         3Eh        7541E03Eh      AB81E03Eh
#0C: VCLK108    #    _1 *54  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         35h        75416035h      AB816035h
#0D: VCLK135    #    _1 *62  /11  # 0Eh             15h             15h             0Eh             00Ah         00h         3Dh        7541403Dh      AB81403Dh
#0E: VCLK157_5  #    _1 *79  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         4Eh        7541604Eh      AB81604Eh
#0F: VCLK162    #    _1 *54  /8   # 0Eh             15h             15h             0Eh             007h         00h         35h        7540E035h      AB80E035h
#10: VCLK154    #    _1 *77  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         4Ch        7541604Ch      AB81604Ch
#11: VCLK83_5   #    _1 *80  /23  # 0Eh             15h             15h             0Eh             016h         00h         4Fh        7542C04Fh      AB82C04Fh
#12: VCLK106_5  #    _1 *71  /16  # 0Eh             15h             15h             0Eh             00Fh         00h         46h        7541E046h      AB81E046h
#13: VCLK146_25 #    _1 *61  /10  # 0Eh             15h             15h             0Eh             009h         00h         3Ch        7541203Ch      AB81203Ch
#14: VCLK148_5  #    _1 *62  /10  # 0Eh             15h             15h             0Eh             009h         00h         3Dh        7541203Dh      AB81203Dh
#15: VCLK71     #    _1 *71  /24  # 0Eh             15h             15h             0Eh             017h         00h         46h        7542E046h      AB82E046h
#16: VCLK88_75  #    _1 *74  /20  # 0Eh             15h             15h             0Eh             013h         00h         49h        75426049h      AB826049h
#17: VCLK119    #    _1 *79  /16  # 0Eh             15h             15h             0Eh             00Fh         00h         4Eh        7541E04Eh      AB81E04Eh
#18: VCLK85_5   #    _1 *57  /16  # 0Eh             15h             15h             0Eh             00Fh         00h         38h        7541E038h      AB81E038h


                     25MHz        # SCU28_a1[31:27] SCU28_a1[26:22] SCU28_a0[31:27] SCU28_a0[26:22] SCU28[21:13] SCU28[12:8] SCU28[7:0]
                                  # SCU1C_a1[31:27] SCU1C_a1[26:22] SCU1C_a0[31:27] SCU1C_a0[26:22] SCU1C[21:13] SCU1C[12:8] SCU1C[7:0]
#00: VCLK25_175 #    _2 *93  /46  # 0Eh             15h             15h             0Eh             02Dh         01h         5Ch
#01: VCLK28_322 #    _1 *68  /60  # 0Eh             15h             15h             0Eh             03Bh         00h         43h
#02: VCLK31_5   #    _1 *63  /50  # 0Eh             15h             15h             0Eh             031h         00h         3Eh
#03: VCLK36     #    _1 *72  /50  # 0Eh             15h             15h             0Eh             031h         00h         47h
#04: VCLK40     #    _1 *64  /40  # 0Eh             15h             15h             0Eh             027h         00h         3Fh
#05: VCLK49_5   #    _1 *75  /38  # 0Eh             15h             15h             0Eh             025h         00h         4Ah
#06: VCLK50     #    _1 *52  /26  # 0Eh             15h             15h             0Eh             019h         00h         33h
#07: VCLK56_25  #    _1 *63  /28  # 0Eh             15h             15h             0Eh             01Bh         00h         3Eh
#08: VCLK65     #    _1 *52  /20  # 0Eh             15h             15h             0Eh             013h         00h         33h
#09: VCLK75     #    _1 *54  /18  # 0Eh             15h             15h             0Eh             011h         00h         35h
#0A: VCLK78_75  #    _1 *63  /20  # 0Eh             15h             15h             0Eh             013h         00h         3Eh
#0B: VCLK94_5   #    _1 *68  /18  # 0Eh             15h             15h             0Eh             011h         00h         43h
#0C: VCLK108    #    _1 *69  /16  # 0Eh             15h             15h             0Eh             00Fh         00h         44h
#0D: VCLK135    #    _1 *54  /10  # 0Eh             15h             15h             0Eh             009h         00h         35h
#0E: VCLK157_5  #    _1 *63  /10  # 0Eh             15h             15h             0Eh             009h         00h         3Eh
#0F: VCLK162    #    _1 *52  /8   # 0Eh             15h             15h             0Eh             007h         00h         33h
#10: VCLK154    #    _1 *74  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         49h
#11: VCLK83_5   #    _1 *60  /18  # 0Eh             15h             15h             0Eh             011h         00h         3Bh
#12: VCLK106_5  #    _1 *51  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         32h
#13: VCLK146_25 #    _1 *70  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         45h
#14: VCLK148_5  #    _1 *71  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         46h
#15: VCLK71     #    _1 *74  /26  # 0Eh             15h             15h             0Eh             019h         00h         49h
#16: VCLK88_75  #    _1 *71  /20  # 0Eh             15h             15h             0Eh             013h         00h         46h
#17: VCLK119    #    _1 *57  /12  # 0Eh             15h             15h             0Eh             00Bh         00h         38h
#18: VCLK85_5   #    _1 *65  /19  # 0Eh             15h             15h             0Eh             012h         00h         40h




**************************************************************************************************************************************/
#ifdef AST_SOC_G5
static struct pixel_freq_pll_data pll_table[] = {
#if 1
//AST2500A1
	{39721, 		((0x0E << 27) | (0x15 << 22) | (0x3D << 13) | (0x40)) },		/* 00: VCLK25_175		*/		
	{35308, 		((0x0E << 27) | (0x15 << 22) | (0x31 << 13) | (0x3A)) },		/* 01: VCLK28_322		*/
	{31746, 		((0x0E << 27) | (0x15 << 22) | (0x2F << 13) | (0x3E)) },		/* 02: VCLK31_5 		*/
	{27777, 		((0x0E << 27) | (0x15 << 22) | (0x23 << 13) | (0x35))}, 		/* 03: VCLK36			*/	
	{25000, 		((0x0E << 27) | (0x15 << 22) | (0x1D << 13) | (0x31))}, 		/* 04: VCLK40			*/
	{20202, 		((0x0E << 27) | (0x15 << 22) | (0x1F << 13) | (0x41))}, 		/* 05: VCLK49_5 		*/
	{20000, 		((0x0E << 27) | (0x15 << 22) | (0x17 << 13) | (0x31))}, 		/* 06: VCLK50			*/
	{17777, 		((0x0E << 27) | (0x15 << 22) | (0x1F << 13) | (0x4A))}, 		/* 07: VCLK56_25		*/
	{15384, 		((0x0E << 27) | (0x15 << 22) | (0x17 << 13) | (0x40))}, 		/* 08: VCLK65			*/
	{13333, 		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x31))}, 		/* 09: VCLK75			*/
	{12690, 		((0x0E << 27) | (0x15 << 22) | (0x11 << 13) | (0x3A))}, 		/* 0A: VCLK78_75		*/
	{10582, 		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x3E))}, 		/* 0B: VCLK94_5 		*/
	{9259,		((0x0E << 27) | (0x15 << 22) | (0x0B << 13) | (0x35))}, 		/* 0C: VCLK108			*/
	{7407,		((0x0E << 27) | (0x15 << 22) | (0x0A << 13) | (0x3D))}, 		/* 0D: VCLK135			*/
	{6349,		((0x0E << 27) | (0x15 << 22) | (0x0B << 13) | (0x4E))}, 		/* 0E: VCLK157_5		*/
	{6172,		((0x0E << 27) | (0x15 << 22) | (0x07 << 13) | (0x35))}, 		/* 0F: VCLK162			*/
	{6493,		((0x0E << 27) | (0x15 << 22) | (0x0B << 13) | (0x4C))}, 		/* 10: VCLK154			*/
	{11976,		((0x0E << 27) | (0x15 << 22) | (0x16 << 13) | (0x4F))}, 		/* 11: VCLK83_5			*/	
	{9389,		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x46))}, 		/* 12: VCLK106_5			*/	
	{6838,		((0x0E << 27) | (0x15 << 22) | (0x09 << 13) | (0x3C))}, 		/* 13: VCLK146_25		*/	
	{6734,		((0x0E << 27) | (0x15 << 22) | (0x09 << 13) | (0x3D))}, 		/* 14: VCLK148_5		*/	
	{14084,		((0x0E << 27) | (0x15 << 22) | (0x09 << 17) | (0x46))}, 		/* 15: VCLK71		*/	
	{11268,		((0x0E << 27) | (0x15 << 22) | (0x09 << 13) | (0x49))}, 		/* 16: VCLK88_75		*/	
	{8403,		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x4E))}, 		/* 17: VCLK119		*/	
	{11696,		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x38))}, 		/* 18: VCLK85_5		*/	
#else
//AST2500A0
	{39721, 		((0x15 << 27) | (0x0E << 22) | (0x3D << 13) | (0x40)) }, 	/* 00: VCLK25_175		*/		
	{35308, 		((0x15 << 27) | (0x0E << 22) | (0x31 << 13) | (0x3A)) },		/* 01: VCLK28_322		*/
	{31746,  		((0x12 << 27) | (0x0B << 22) | (0x1F << 13) | (0x29)) },		/* 02: VCLK31_5 		*/
	{27777, 		((0x12 << 27) | (0x0B << 22) | (0x1B << 13) | (0x29))},		/* 03: VCLK36			*/	
	{25000,  		((0x15 << 27) | (0x0E << 22) | (0x1D << 13) | (0x31))},		/* 04: VCLK40			*/
	{20202,		((0x15 << 27) | (0x0E << 22) | (0x1F << 13) | (0x41))},		/* 05: VCLK49_5 		*/
	{20000,		((0x15 << 27) | (0x0E << 22) | (0x17 << 13) | (0x31))},		/* 06: VCLK50			*/
	{17777,		((0x15 << 27) | (0x0E << 22) | (0x1F << 13) | (0x4A))},		/* 07: VCLK56_25		*/
	{15384,		((0x15 << 27) | (0x0E << 22) | (0x17 << 13) | (0x40))},		/* 08: VCLK65			*/
	{13333,		((0x15 << 27) | (0x0E << 22) | (0x0F << 13) | (0x31))},		/* 09: VCLK75			*/
	{12690,		((0x15 << 27) | (0x0E << 22) | (0x11 << 13) | (0x3A))},		/* 0A: VCLK78_75		*/
	{10582,		((0x15 << 27) | (0x0E << 22) | (0x0F << 13) | (0x3E))},		/* 0B: VCLK94_5 		*/
	{9259,		((0x12 << 27) | (0x0B << 22) | (0x09 << 13) | (0x2C))},		/* 0C: VCLK108			*/
	{7407,		((0x12 << 27) | (0x0B << 22) | (0x07 << 13) | (0x2C))},		/* 0D: VCLK135			*/
	{6349,		((0x15 << 27) | (0x0E << 22) | (0x0B << 13) | (0x4E))},		/* 0E: VCLK157_5		*/
	{6172,		((0x15 << 27) | (0x0E << 22) | (0x07 << 13) | (0x35))},		/* 0F: VCLK162			*/
#endif	
};

static struct pixel_freq_pll_data pll25M_table[] = {
	{39721, 		((0x0E << 27) | (0x15 << 22) | (0x3D << 13) | (0x40)) },		/* 00: VCLK25_175		*/		
	{35308, 		((0x0E << 27) | (0x15 << 22) | (0x31 << 13) | (0x3A)) },		/* 01: VCLK28_322		*/
	{31746, 		((0x0E << 27) | (0x15 << 22) | (0x2F << 13) | (0x3E)) },		/* 02: VCLK31_5 		*/
	{27777, 		((0x0E << 27) | (0x15 << 22) | (0x23 << 13) | (0x35))}, 		/* 03: VCLK36			*/	
	{25000, 		((0x0E << 27) | (0x15 << 22) | (0x1D << 13) | (0x31))}, 		/* 04: VCLK40			*/
	{20202, 		((0x0E << 27) | (0x15 << 22) | (0x1F << 13) | (0x41))}, 		/* 05: VCLK49_5 		*/
	{20000, 		((0x0E << 27) | (0x15 << 22) | (0x17 << 13) | (0x31))}, 		/* 06: VCLK50			*/
	{17777, 		((0x0E << 27) | (0x15 << 22) | (0x1F << 13) | (0x4A))}, 		/* 07: VCLK56_25		*/
	{15384, 		((0x0E << 27) | (0x15 << 22) | (0x17 << 13) | (0x40))}, 		/* 08: VCLK65			*/
	{13333, 		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x31))}, 		/* 09: VCLK75			*/
	{12690, 		((0x0E << 27) | (0x15 << 22) | (0x11 << 13) | (0x3A))}, 		/* 0A: VCLK78_75		*/
	{10582, 		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x3E))}, 		/* 0B: VCLK94_5 		*/
	{9259,		((0x0E << 27) | (0x15 << 22) | (0x0B << 13) | (0x35))}, 		/* 0C: VCLK108			*/
	{7407,		((0x0E << 27) | (0x15 << 22) | (0x0A << 13) | (0x3D))}, 		/* 0D: VCLK135			*/
	{6349,		((0x0E << 27) | (0x15 << 22) | (0x0B << 13) | (0x4E))}, 		/* 0E: VCLK157_5		*/
	{6172,		((0x0E << 27) | (0x15 << 22) | (0x07 << 13) | (0x35))}, 		/* 0F: VCLK162			*/
	{6493,		((0x0E << 27) | (0x15 << 22) | (0x0B << 13) | (0x4C))}, 		/* 10: VCLK154			*/
	{11976,		((0x0E << 27) | (0x15 << 22) | (0x16 << 13) | (0x4F))}, 		/* 11: VCLK83_5			*/	
	{9389,		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x46))}, 		/* 12: VCLK106_5			*/	
	{6838,		((0x0E << 27) | (0x15 << 22) | (0x09 << 13) | (0x3C))}, 		/* 13: VCLK146_25		*/	
	{6734,		((0x0E << 27) | (0x15 << 22) | (0x09 << 13) | (0x3D))}, 		/* 14: VCLK148_5		*/	
	{14084,		((0x0E << 27) | (0x15 << 22) | (0x09 << 17) | (0x46))}, 		/* 15: VCLK71		*/	
	{11268,		((0x0E << 27) | (0x15 << 22) | (0x09 << 13) | (0x49))}, 		/* 16: VCLK88_75		*/	
	{8403,		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x4E))}, 		/* 17: VCLK119		*/	
	{11696,		((0x0E << 27) | (0x15 << 22) | (0x0F << 13) | (0x38))}, 		/* 18: VCLK85_5		*/	
};

#else
static struct pixel_freq_pll_data pll_table[] = {
	{39721, 		0x00046515}, 		/* 00: VCLK25_175	*/		
	{35308, 		0x00047255},		/* 01: VCLK28_322	*/
	{31746,  		0x0004682a},		/* 02: VCLK31_5 		*/
	{27777, 		0x0004672a},		/* 03: VCLK36		*/	
	{25000,  		0x00046c50},		/* 04: VCLK40		*/
	{20202,		0x00046842},		/* 05: VCLK49_5 	*/
	{20000,		0x00006c32},		/* 06: VCLK50			*/
	{17777,		0x00006a2f},		/* 07: VCLK56_25		*/
	{15384,		0x00006c41},		/* 08: VCLK65		*/
	{13333,		0x00006832},		/* 09: VCLK75			*/
	{12690,		0x0000672e},		/* 0A: VCLK78_75		*/
	{10582,		0x0000683f},		/* 0B: VCLK94_5 		*/
	{9259,		0x00004824},		/* 0C: VCLK108			*/
	{7407,		0x0000482d},		/* 0D: VCLK135			*/
	{6349,		0x0000472e},		/* 0E: VCLK157_5		*/
	{6172,		0x00004836},		/* 0F: VCLK162			*/
};
#endif
// ARGB4444 format
unsigned short cursor_8x8[] = {
	0x0FFF, 0x1FFF, 0x2FFF, 0x3777, 0x4777, 0x5777, 0x6777, 0x7888, 
	0x8FFF, 0xF000, 0xAFFF, 0xB777, 0xC777, 0xD777, 0xE777, 0xF888, 
	0x0FFF, 0x1FFF, 0x2FFF, 0x3FFF, 0x4777, 0x5777, 0x6777, 0x7888, 
	0x8FFF, 0x9FFF, 0xAFFF, 0xBFFF, 0xCFFF, 0xD777, 0xE777, 0xF888, 
	0x0FFF, 0x1FFF, 0x2FFF, 0x3FFF, 0x4FFF, 0x5FFF, 0x6FFF, 0x7888, 
	0x8FFF, 0x9FFF, 0xAFFF, 0xBFFF, 0xCFFF, 0xDFFF, 0xEFFF, 0xFFFF, 
	0x0FFF, 0x1FFF, 0x2777, 0x3FFF, 0x4FFF, 0x5FFF, 0x6FFF, 0x7FFF, 
	0x8FFF, 0x9777, 0xA777, 0xB777, 0xC777, 0xDFFF, 0xEFFF, 0xFFFF, 
};

// XRGB4444 format
unsigned short cursor_16x16[] = {
	0x8777, 0x8777, 0x8777, 0x8777, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x8777, 0xC888, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x8777, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x8777, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x8777, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x4777, 0x4FFF, 0x4FFF, 0x4FFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x4FFF, 0x4FFF, 0x4FFF, 0x4FFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0xCFFF, 0xCFFF, 0xCFFF, 0xCFFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0xCFFF, 0xCFFF, 0xCFFF, 0xCFFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0xCFFF, 0xCFFF, 0xCFFF, 0xCFFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0xCFFF, 0xCFFF, 0xCFFF, 0xCFFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
	0xCFFF, 0xCFFF, 0xCFFF, 0xCFFF, 0x4FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 0x8FFF, 
};

/*
	name, refresh  
	xres, yres, pixclock = 1/pixel clock 
	left_margin = H Back Porch + Border; 	right_margin = H Front Porch + Border
	upper_margin = V Back Porch; lower_margin = V Front Porch + Border 
	hsync_len = Hor Sync Time, vsync_len = Ver Sync Time 
	sync, vmodeflag;
*/
static const struct fb_videomode ast_modedb[] = {
    {
	/* 640x480 @ 60 Hz, 31.5 kHz hsync */
	NULL, 60, 640, 480, 39721, 48, 16, 33, 10, 96, 2,
	0, FB_VMODE_NONINTERLACED
    }, {
	/* 800x600 @ 60 Hz, 37.8 kHz hsync */
	NULL, 60, 800, 600, 25000, 88, 40, 23, 1, 128, 4,
	0, FB_VMODE_NONINTERLACED
    }, {
	/* 1024x768 @ 60 Hz, 48.4 kHz hsync */
	NULL, 60, 1024, 768, 15384, 160, 24, 29, 3, 136, 6,
	0, FB_VMODE_NONINTERLACED
    }, {
	/* #5: 1920x1080i@59.94/60Hz */
	NULL, 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
	0,
	FB_VMODE_NONINTERLACED
	}
};

struct astfb_device {
	int					state;
	struct mutex		rqueue_mutex;
	int					palette_size;
	u32					pseudo_palette[17];
	struct platform_device 		*pdev;
	struct fb_var_screeninfo	new_var;	/* for mode changes */

//	struct omapfb_mem_desc		mem_desc;
//	struct fb_info			*fb_info[OMAPFB_PLANE_NUM];
};

/* data structure */
struct astfb_info {
	struct platform_device 	*pdev;
	struct fb_info			*info;
	struct resource *reg_res;
	struct resource *fb_res;
	void __iomem *base;
	phys_addr_t *cursor_phy;	
	void __iomem *cursor_virt;
	int addr_assign;
	int irq;
	int yuv_mode;
	u32 pseudo_palette[17];
	u8 edid[256];
	
	struct timer_list timer;

	/* driver registered */
	int registered;	
	/* console control */	
	int currcon;

	int need_wakeup;
	void __iomem *next_addr;

	u8 hwcursor;   //0: disable , 1 : enable
	u8 dac;	//0: disable , 1 : enable
	u8 dvo;	//0: disable , 1 : enable
	u8 xmiter;	//0: dvi, 1:hdmi;	
	struct ast_fb_plat_data *fb_plat_data;
	 
};

extern unsigned char *vga_read_ddc_edid(void);

static inline void
astfb_write(struct astfb_info *fbinfo, u32 val, u32 reg)
{
	writel(val, fbinfo->base+ reg);
}

static inline u32
astfb_read(struct astfb_info *fbinfo, u32 reg)
{
	return readl(fbinfo->base + reg);
}

static void astfb_osd_enable(struct astfb_info *sfb, u8 enable)
{
	if(enable)
		astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) | CRT_CTRL_OSD_EN, AST_CRT_CTRL1);
	else
		astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) & ~CRT_CTRL_OSD_EN, AST_CRT_CTRL1);
}

int
astfb_crtc_to_var(struct fb_var_screeninfo *var, struct astfb_info *sfb)
{
    
    /* crtc */
    var->xoffset = var->yoffset = 0;
    
    /* palette */
    switch(var->bits_per_pixel) {
    case 8:
       var->red.offset = var->green.offset = var->blue.offset = 0;
       var->red.length = var->green.length = var->blue.length = 6;
       break;
    case 16:
       var->red.offset = 11;
       var->red.length = 5;
       var->green.offset = 5;
       var->green.length = 6;
       var->blue.offset = 0;
       var->blue.length = 5;
       var->transp.offset = 0;
       var->transp.length = 0;
       break;
	case 24:
    case 32:
       var->red.offset = 16;
       var->red.length = 8;
       var->green.offset = 8;
       var->green.length = 8;
       var->blue.offset = 0;
       var->blue.length = 8;
       var->transp.offset = 24;
       var->transp.length = 8;
       break;
    }
    
    var->red.msb_right = 
    var->green.msb_right =
    var->blue.msb_right =
    var->transp.offset = 
    var->transp.length =  
    var->transp.msb_right = 0;
	    
    return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
static int astfb_hw_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	int x, y;
	struct astfb_info *sfb = info->par;
	if(sfb->cursor_phy == NULL)
		return 1;

//	printk("Enable [%d], set = %x \n ",cursor->enable, cursor->set);

	if (cursor->enable)
		astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) | CRT_CTRL_HW_CURSOR_EN, AST_CRT_CTRL1);
	else 
		astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) & ~CRT_CTRL_HW_CURSOR_EN, AST_CRT_CTRL1);

	y = (cursor->image.dy - info->var.yoffset) & 0x3ff ;
	x = (cursor->image.dx - info->var.xoffset) & 0x7ff;
	astfb_write(sfb, (y << 16)|x, AST_CRT_CURSOR1);

      /*
         * If the cursor is not be changed this means either we want the
         * current cursor state (if enable is set) or we want to query what
         * we can do with the cursor (if enable is not set)
         */
        if (!cursor->set)
                return 0;

	if (cursor->set & FB_CUR_SETHOT) {
		x = cursor->image.dx;
		y = cursor->image.dy;
		astfb_write(sfb, (y << 16)|x, AST_CRT_CURSOR1);
//		printk("FB_CUR_SETHOT x [%d], y[%d] \n", x, y);
	}

       if (cursor->set & FB_CUR_SETPOS) {
		y = (cursor->image.dy) & 0x3ff ;
		x = (cursor->image.dx) & 0x7ff;
		astfb_write(sfb, (y << 16)|x, AST_CRT_CURSOR1);
//		printk("FB_CUR_SETPOS x [%d], y[%d] \n", x, y);
		astfb_write(sfb, ((info->var.yoffset & 0x3f) << 8)|(info->var.xoffset & 0x3f), AST_CRT_CURSOR0);
	}

//      printk("cursor->image.width = %d , cursor->image.h = %d depth = %d \n",cursor->image.width , cursor->image.height, cursor->image.depth);

        /*
         * FB_CUR_SETIMAGE - the cursor image has changed
         * FB_CUR_SETCMAP  - the cursor colors has changed
         * FB_CUR_SETSHAPE - the cursor bitmask has changed
         */
	if (cursor->set & (FB_CUR_SETSHAPE | FB_CUR_SETCMAP | FB_CUR_SETIMAGE)) {
		if(cursor->image.depth == 1) {
			u32 __iomem *cursorbase = sfb->cursor_virt;
			u8 *bitmap = (u8 *)cursor->image.data;
			u8 *mask = (u8 *)cursor->mask;
			int i, j;

			fb_memset(cursorbase, 0x0, 0x2000);

			for (i = 0; i < cursor->image.height; i++) {
				for(j = 0; j < cursor->image.height; j++) {
					if(bitmap[i + j]) {
						cursorbase[i + j] = 0xffffffff;
					} else {
						cursorbase[i + j] = 0;
					}
				}
			}
		}
        }

	return 0;
}
#endif

#if (NUMBER_OF_BUFFERS > 1)
static int astfb_pan_display(struct fb_var_screeninfo *var, struct fb_info* info)
{
	struct astfb_info *sfb = info->par;
	u32 addr;
	s32 timeout;
	
	if(gNoPanDisplay)
			return 0;
	
	addr = var->yoffset * info->fix.line_length + info->fix.smem_start;
	
	astfb_write(sfb, addr, AST_CRT_ADDR);
	
	if(gGUIWaitVsync)
	{
		timeout = interruptible_sleep_on_timeout(&wq,HZ/60);
		if(timeout<0)
				ASTFB_DBG("%s: interruptible_sleep_on_timeout, may lost interrupt! timeout=%d\n",__FUNCTION__,timeout);
	}
	return 0;

} /* astfb_pan_display */
#endif

static int astfb_set_par(struct fb_info *info)
{
	struct astfb_info *sfb = info->par;
	struct fb_var_screeninfo *var = &info->var;
	u32 i,ctrl1, ctrl2, htt, hde, hrs_s, hrs_e, vtt, vde, vrs_s, vrs_e;
	u32 d_offset, t_count, thshld;
	
	//S1  : set H / V 
	//  Horizontal Timing
	htt = var->xres + var->left_margin + var->right_margin + var->hsync_len;
	hde = var->xres;	
	astfb_write(sfb, CRT_H_TOTAL((htt - 1)) | CRT_H_DE((hde - 1)), AST_CRT_HORIZ0);

	hrs_s = var->xres + var->right_margin;
	hrs_e = var->xres + var->right_margin + var->hsync_len;
	astfb_write(sfb, CRT_H_RS_START((hrs_s - 1)) | CRT_H_RS_END((hrs_e - 1)), AST_CRT_HORIZ1);

	ASTFB_DBG("var->upper_margin= %d, var->lower_margin= %d, var->vsync_len = %d \n",var->upper_margin, var->lower_margin, var->vsync_len);

	vtt = var->yres + var->upper_margin + var->lower_margin + var->vsync_len;
	vde = var->yres;
	astfb_write(sfb, CRT_V_TOTAL((vtt - 1)) | CRT_V_DE((vde - 1)), AST_CRT_VERTI0);	
	vrs_s = var->yres + var->lower_margin;
	vrs_e = var->yres + var->lower_margin + var->vsync_len;
	astfb_write(sfb, CRT_V_RS_START((vrs_s - 1)) | CRT_V_RS_END((vrs_e - 1)), AST_CRT_VERTI1);

	if(var->nonstd != 0)
		printk("TODO Check .... nonstd \n");
	
	switch (var->nonstd) {
		case 0:
			break;
		case ASTFB_COLOR_YUV444:
		case ASTFB_COLOR_YUV420:		
			var->bits_per_pixel = 32;
			break;
		case ASTFB_COLOR_RGB888:		
			var->bits_per_pixel = 24;
			break;			
		case ASTFB_COLOR_RGB565:		
			var->bits_per_pixel = 16;
			break;
	}

	//S2 : Offset , TODO ... (x + 0x1f) & ~0x1f
	d_offset = var->xres * var->bits_per_pixel /8;
//	ASTFB_DBG("d_offset %d\n",d_offset);

	switch (var->nonstd) {
		case 0:
			break;
		
		case ASTFB_COLOR_RGB888:		
			var->bits_per_pixel = 32;
			break;
			
		case ASTFB_COLOR_YUV444:
			var->bits_per_pixel = 24;
			break;
		case ASTFB_COLOR_RGB565:		
		case ASTFB_COLOR_YUV420:
			var->bits_per_pixel = 16;
			break;;
	}
#ifdef AST_SOC_G5
	t_count =(var->xres * var->bits_per_pixel + 127) / 128;	
#else
	t_count =(var->xres * var->bits_per_pixel + 63) / 64;
#endif
//	ASTFB_DBG("t_count %d \n",t_count);
	astfb_write(sfb, CRT_DISP_OFFSET(d_offset) | CRT_TERM_COUNT(t_count), AST_CRT_OFFSET);


	//S3 : DCLK
	ASTFB_DBG("var->pixclock = %d \n",var->pixclock);

#ifdef AST_SOC_G5
	if(sfb->fb_plat_data->set_pll) {
		if(sfb->fb_plat_data->clock_src == 24000000) {
			for(i=0; i<sizeof(pll_table)/sizeof(struct pixel_freq_pll_data); i++) {
				if(pll_table[i].pixel_freq == var->pixclock) {
					sfb->fb_plat_data->set_pll(pll_table[i].pll_set);
					ASTFB_DBG("find pixclk in table set 0x%x \n",pll_table[i].pll_set);
					break;
				}
			}
			if(i == sizeof(pll_table)/sizeof(struct pixel_freq_pll_data))
				printk("ERROR pixclk in table ... FIXME \n");
		} else {
			for(i=0; i<sizeof(pll25M_table)/sizeof(struct pixel_freq_pll_data); i++) {
				if(pll25M_table[i].pixel_freq == var->pixclock) {
					sfb->fb_plat_data->set_pll(pll_table[i].pll_set);
					ASTFB_DBG("find pixclk in table set 0x%x \n",pll25M_table[i].pll_set);
					break;
				}
			}
			if(i == sizeof(pll25M_table)/sizeof(struct pixel_freq_pll_data))
				printk("ERROR pixclk in table ... FIXME \n");
		}
	}
#else
	for(i=0; i<sizeof(pll_table)/sizeof(struct pixel_freq_pll_data); i++) {
		if(pll_table[i].pixel_freq == var->pixclock) {
			astfb_write(sfb, pll_table[i].pll_set, AST_CRT_PLL);
			ASTFB_DBG("find pixclk in table set 0x%x \n",pll_table[i].pll_set);
			break;
		}
	}
	if(i == sizeof(pll_table)/sizeof(struct pixel_freq_pll_data))
		printk("ERROR pixclk in table ... FIXME \n");
	
#endif
	

	//S4
	astfb_write(sfb, sfb->info->fix.smem_start, AST_CRT_ADDR);

#ifdef CONFIG_FRAMEBUFFER_CONSOLE 
	//HW Cursor
	astfb_write(sfb, sfb->cursor_phy, AST_CRT_CURSOR2);
#endif

	thshld = CRT_THROD_HIGH(CRT_HIGH_THRESHOLD_VALUE) | CRT_THROD_LOW(CRT_LOW_THRESHOLD_VALUE);
	astfb_write(sfb, thshld, AST_CRT_THROD);

	
	info->fix.line_length = (var->xres*var->bits_per_pixel)/8;
	ASTFB_DBG("x :%d , y : %d , bpp = %d \n",var->xres, var->yres, var->bits_per_pixel);
	//disable crt first .....
	astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL2) & ~(CRT_CTRL_DAC_PWR_EN | CRT_CTRL_DVO_EN), AST_CRT_CTRL2);

	ctrl1 = astfb_read(sfb, AST_CRT_CTRL1);
	//CTRL 1 
	//  SetPolarity
	ASTFB_DBG("var->sync : %x , var->vmode = %d \n",var->sync, var->vmode);
	
	if(var->sync & FB_SYNC_HOR_HIGH_ACT)
		ctrl1 &= ~CRT_CTRL_HSYNC_POLARITY;
	else
		ctrl1 |= CRT_CTRL_HSYNC_POLARITY;
	
	if(var->sync & FB_SYNC_VERT_HIGH_ACT)
		ctrl1 &= ~CRT_CTRL_VSYNC_POLARITY;
	else
		ctrl1 |= CRT_CTRL_VSYNC_POLARITY;
	
    /* Mode Type Setting */
	
	if(var->bits_per_pixel==16)
		ctrl1 &= ~CRT_CTRL_FORMAT_MASK;	//RGB565
	else 
		ctrl1 |= CRT_CTRL_FORMAT(COLOR_XRGB8888);

	if (var->vmode & FB_VMODE_INTERLACED)
		ctrl1 |= CRT_CTRL_INTER_TIMING;
	else
		ctrl1 &= ~CRT_CTRL_INTER_TIMING;
	
	//enable crt ...
	astfb_write(sfb, ctrl1 | CRT_CTRL_GRAPHIC_EN, AST_CRT_CTRL1);

	ASTFB_DBG("var->left_margin= %d, var->right_margin= %d, var->hsync_len = %d \n",var->left_margin, var->right_margin, var->hsync_len);


	//enable dac / dvo
	//CTRL 2 
	ctrl2 = 0;//astfb_read(sfb, AST_CRT_CTRL2);

	// SoC V2 add CRT interrupt support. We should not touch this setting when changing video timing.
	ctrl2 &= ~CRT_CTRL_VLINE_NUM_MASK;

#ifdef CONFIG_AST_DAC
	ctrl2 |= CRT_CTRL_DAC_PWR_EN;
#endif

#ifdef CONFIG_AST_DVO
	ctrl2 |= CRT_CTRL_DVO_EN;
#endif

	astfb_write(sfb, ctrl2 , AST_CRT_CTRL2);
	
	return 0;
}

static int astfb_get_cmap_len(struct fb_var_screeninfo *var)
{
	return (var->bits_per_pixel == 8) ? 256 : 16;
}

static int astfb_setcolreg(unsigned regno, 
							unsigned red, unsigned green, unsigned blue,
							unsigned transp, struct fb_info *info)
{
	if(regno >= astfb_get_cmap_len(&info->var))
		return 1;
      
	switch(info->var.bits_per_pixel) {
	case 8:
		return 1;
		break;
	case 16:
		((u32 *)(info->pseudo_palette))[regno] =
				(red & 0xf800)          |
				((green & 0xfc00) >> 5) |
				((blue & 0xf800) >> 11);
		break;
	case 24:
	case 32:
		red >>= 8;
		green >>= 8;
		blue >>= 8;
		((u32 *)(info->pseudo_palette))[regno] =
				(red << 16) | (green << 8) | (blue);
		break;
	}
	return 0;

}

/*
 * Blank the screen if blank_mode != 0, else unblank. Return 0 if blanking
 * succeeded, != 0 if un-/blanking failed.
 * blank_mode == 2: suspend vsync
 * blank_mode == 3: suspend hsync
 * blank_mode == 4: powerdown
 */
static int astfb_blank(int blank_mode, struct fb_info *info)
{
	u32 ctrl;
	struct astfb_info *sfb = info->par;

	printk(KERN_DEBUG "astfb: astfb_blank mode %d \n",blank_mode);
	ctrl = astfb_read(sfb, AST_CRT_CTRL1);
		
	switch(blank_mode) {
		case FB_BLANK_UNBLANK:	/* on */
			ctrl &= ~CRT_CTRL_SCREEN_OFF;
		break;
		case FB_BLANK_NORMAL:	/* blank */
			ctrl |= CRT_CTRL_SCREEN_OFF;
		break;
		case FB_BLANK_VSYNC_SUSPEND:	/* no vsync */
			ctrl |= CRT_CTRL_VSYNC_OFF;
		break;
		case FB_BLANK_HSYNC_SUSPEND:	/* no hsync */
			ctrl |= CRT_CTRL_HSYNC_OFF;
		break;
		case FB_BLANK_POWERDOWN:	/* off */
			ctrl |= (CRT_CTRL_SCREEN_OFF | CRT_CTRL_VSYNC_OFF | CRT_CTRL_HSYNC_OFF);
		break;
		default:
			return 1;
	}	

    /* set reg */
	astfb_write(sfb, ctrl, AST_CRT_CTRL1);
	
	return 0;

} /* astfb_blank */

static int astfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	var->xres_virtual = var->xres;
	var->yres_virtual =var->yres * NUMBER_OF_BUFFERS;
////////////////////////////////////////////////////////////////////
	/* Sanity check for offsets */
	if(var->xoffset < 0) var->xoffset = 0;
	if(var->yoffset < 0) var->yoffset = 0;

	if(var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;

	/* Truncate offsets to maximum if too high */
	if(var->xoffset > var->xres_virtual - var->xres) {
		var->xoffset = var->xres_virtual - var->xres - 1;
	}

	if(var->yoffset > var->yres_virtual - var->yres) {
		var->yoffset = var->yres_virtual - var->yres - 1;
	}
////////////////////////////////////////////////////////////////////	        
	switch(var->bits_per_pixel) {
		case 8:
			var->red.offset = var->green.offset = var->blue.offset = 0;
			var->red.length = var->green.length = var->blue.length = 6;
			break;
		case 16:
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
			break;
		case 24:
		case 32:
			var->red.offset = 16;
			var->red.length = 8;
			var->green.offset = 8;
			var->green.length = 8;
			var->blue.offset = 0;
			var->blue.length = 8;
			var->transp.length = 8;
			var->transp.offset = 24;
			break;
		default:
			ASTFB_DBG("bpp=%d not support\n",var->bits_per_pixel);
			return -EINVAL;
			break;
	}
	return 0;
}

static int
astfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct astfb_info *sfb = info->par;
//	void __user *argp = (void __user *)arg;
	
	printk(KERN_DEBUG "astfb: astfb_ioctl is called \n");

	switch(cmd) {
		case AST_FB_CURSOR_SET:
			break;
		case AST_FB_CURSOR_POS:
//			astfb_write(sfb, ((yoffset & 0x3f) << 8)|(xoffset & 0x3f), AST_CRT_CURSOR0);
//			astfb_write(sfb, (y << 16)|x, AST_CRT_CURSOR1);
			
			break;
		case AST_FB_CURSOR_EN:

			if (arg)
				astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) | CRT_CTRL_HW_CURSOR_EN, AST_CRT_CTRL1);
			else 
				astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) & ~CRT_CTRL_HW_CURSOR_EN, AST_CRT_CTRL1);
			
			break;
		default:
			return -EINVAL;           
	}

	return 0;
	
} /* astfb_ioctl */

/* fb ops */
static struct fb_ops astfb_ops = {
	.owner          = THIS_MODULE,
	.fb_check_var   = astfb_check_var,
	.fb_set_par     = astfb_set_par,
	.fb_blank       = astfb_blank,
	.fb_setcolreg   = astfb_setcolreg,    
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_ioctl       = astfb_ioctl,
#ifdef CONFIG_FRAMEBUFFER_CONSOLE 
	.fb_cursor		= astfb_hw_cursor,
#endif    
#if (NUMBER_OF_BUFFERS > 1)    
	.fb_pan_display = astfb_pan_display,    
#endif
};

static void ast_fbmem_free(struct astfb_info *sfb)
{
	iounmap(sfb->info->screen_base);
}

static irqreturn_t
astfb_isr(int irq, void *parm)
{
    u32 status;
    struct astfb_info *sfb=parm;
    status = astfb_read(sfb, AST_CRT_CTRL1);
    astfb_write(sfb, status, AST_CRT_CTRL1);
    if (status & CRT_CTRL_VERTICAL_INTR_STS)
            wake_up_interruptible(&wq);

    return IRQ_HANDLED;
}

//TODO ..
static int astfb_setup(struct astfb_info *sfb)
{
	char *this_opt = NULL;
	char *options = NULL;
	char tmp[128];
	char *tmp_opt;
	char name[10];
	int i;
	
	fb_get_options("astfb", &options);
	ASTFB_DBG("%s\n", options);

	if (!options || !*options)
		return -1;

	strcpy(tmp, options);
	tmp_opt=tmp;
	while ((this_opt = strsep(&tmp_opt, ",")) != NULL) {
		printk("x %s \n",this_opt);
		if (!strncmp(this_opt, "mode:", 5)) {
				printk("%s \n",this_opt);		
		} else if(!strncmp(this_opt, "hwcursor:", 9)) {
				printk("%s \n",this_opt);
		} else if(!strncmp(this_opt, "osd:", 4)) {
				printk("%s \n",this_opt);
		} else if (!strncmp(this_opt, "vram:", 8)) {
				printk("%s \n",this_opt);
		} else if(!strncmp(this_opt, "dac:", 4)) {
				printk("%s \n",this_opt);
		} else if(!strncmp(this_opt, "dvo:", 4)) {
				printk("%s \n",this_opt);				
		} else {
			printk("f %s \n",this_opt);
		}
		
	}

    return 0;
    	
} /* astfb_setup */

static void sfb_timer(unsigned long private)
{
    struct astfb_info *sfb = (void *) private;
    if(sfb->need_wakeup)
    {
        sfb->need_wakeup=0;
        wake_up_interruptible(&wq);
    }
    if(sfb->next_addr)
    {
		astfb_write(sfb, (u32)sfb->next_addr, AST_CRT_ADDR);
        sfb->need_wakeup=1;
    }
    mod_timer(&sfb->timer, jiffies + HZ/24);
}

static ssize_t show_lcd_enable(struct device *device,
                             struct device_attribute *attr, char *buf)
{
    struct fb_info *info = dev_get_drvdata(device);
    struct astfb_info *sfb = info->par;
    if(astfb_read(sfb, AST_CRT_CTRL1) & CRT_CTRL_GRAPHIC_EN)
            return sprintf(buf, "%d\n",1);
    else
            return sprintf(buf, "%d\n",0);
}

static ssize_t store_lcd_enable(struct device *device,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    struct fb_info *info = dev_get_drvdata(device);
    struct astfb_info *sfb = info->par;
    if(buf[0]=='1') {
        astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) | CRT_CTRL_GRAPHIC_EN, AST_CRT_CTRL1);
    }
    else {
        astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) & ~CRT_CTRL_GRAPHIC_EN, AST_CRT_CTRL1);
    }

    return count;
}

static ssize_t show_pix_clk(struct device *device,
                             struct device_attribute *attr, char *buf)
{
        struct fb_info *info = dev_get_drvdata(device);
        struct astfb_info *sfb = info->par;

//        return sprintf(buf, "target_clk=%d\ncalc_clk=%d\n",sfb->target_clk,sfb->calc_clk);
}

static ssize_t no_pan_display_show(struct device *device,
                             struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%hu\n", gNoPanDisplay);
}

static ssize_t no_pan_display_store(struct device *device,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
        unsigned short value;

        if (sscanf(buf, "%hu", &value) != 1 ||
            (value != 0 && value != 1 )) {
                printk(KERN_ERR "no_pan_display_store : Invalid value\n");
                return -EINVAL;
        }

        if(value == 0)
                gNoPanDisplay = 0;
        else if(value == 1)
                gNoPanDisplay = 1;

        return count;
}

static ssize_t phys_addr_show(struct device *device,
                             struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(device);
    return sprintf(buf, "%hu\n", info->fix.smem_start);
}

static ssize_t virt_addr_show(struct device *device,
                             struct device_attribute *attr, char *buf)
{
	struct fb_info *info = dev_get_drvdata(device);
	return sprintf(buf, "%hu\n", info->screen_base);
}

#if 0
static struct device_attribute device_attrs[] = {
    __ATTR(virt_addr, S_IRUGO | S_IWUGO, virt_addr_show, NULL),		
    __ATTR(phys_addr, S_IRUGO | S_IWUGO, phys_addr_show, NULL),	
    __ATTR(no_pan_display, S_IRUGO | S_IWUGO, no_pan_display_show, no_pan_display_store),
    __ATTR(lcd_enable, S_IRUGO | S_IWUGO, show_lcd_enable, store_lcd_enable),
    __ATTR(pixel_clock, S_IRUGO, show_pix_clk, NULL),
//    __ATTR(osd_enable, S_IRUGO, show_osd_enable, store_osd_enable), 
//    __ATTR(cursor_enable, S_IRUGO, show_cursor_enable, store_cursor_enable), 

#ifdef CONFIG_VGA_EDID
    __ATTR(vga_status, S_IRUGO, show_vga_status, NULL),
    __ATTR(vga_detect, S_IRUGO | S_IWUGO, show_vga_edid, NULL),
#endif
};
#endif

struct fb_info *info0;
struct fb_info *info1;
struct fb_info *info2;
struct fb_info *info3;

struct fb_info *astfb_get_crt_screen(u8 crt)
{
	switch(crt) {
		case 0:
			return info0;
			break;
		case 1:
			return info1;
			break;
		case 2:
			return info2;
			break;
		case 3:
			return info3;
			break;
	}
}

EXPORT_SYMBOL(astfb_get_crt_screen);

static int astfb_probe(struct platform_device *pdev)
{
	struct astfb_device	*astfbdev = NULL;
	struct astfb_info *sfb;
	struct resource *res;
	struct fb_info *info;
#ifdef CONFIG_DISPLAY_SUPPORT	
	struct ast_display_device *disp_dev;
#endif	
	struct fb_monspecs *specs;	
	struct device *dev = &pdev->dev;
	int ret,i,retval;
	char *mode_option;
	bool found = false;	

	ASTFB_DBG("\n");

	info = framebuffer_alloc(sizeof(struct astfb_info), dev);
	if (!info) {
			dev_err(dev, "cannot allocate memory\n");
			return -ENOMEM;
	}

	sfb = info->par;
	sfb->info = info;

	switch(dev->id) {
		case 0:
			info0 = info;
			break;
		case 1:
			info1 = info;
			break;
		case 2:
			info2 = info;
			break;
		case 3:
			info3 = info;
			break;
	}
	
	sfb->pdev = pdev;	
	sfb->fb_plat_data = (struct ast_fb_plat_data *)dev->platform_data;
	strcpy(info->fix.id, sfb->pdev->name);

	sfb->reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!sfb->reg_res) {
				dev_err(dev, "register resources unusable\n");
		ret = -ENXIO;
		goto free_info;
	}

	sfb->irq = platform_get_irq(pdev, 0);
		if (!sfb->irq) {
				dev_err(dev, "unable to get irq\n");
		ret = -ENXIO;
		goto free_info;
	}

	if(!sfb->fb_plat_data) {
		dev_err(dev, "unable to get ast fb platform data\n");
		ret = -ENXIO;
		goto free_info;
	}

	info->fix.mmio_start	= sfb->reg_res->start;
	info->fix.mmio_len		= sfb->reg_res->end - sfb->reg_res->start + 1;

	if (!request_mem_region(info->fix.mmio_start, info->fix.mmio_len, pdev->name)) {
			dev_err(dev, "cannot request CRT registers\n");
			ret = -EBUSY;
			goto free_info;
	}

	sfb->base = ioremap(info->fix.mmio_start, info->fix.mmio_len);
	if (!sfb->base) {
		dev_err(dev, "cannot map LCDC registers\n");
		ret = -ENOMEM;
		goto free_res;
	}

	info->fbops = &astfb_ops;
	
	specs = &info->monspecs;

	if(astfb_setup(sfb)) {
		dev_warn(dev, "cannot get fb boot options will use default !!!\n");
	}

#ifdef CONFIG_DISPLAY_SUPPORT
	disp_dev = ast_display_device_select(sfb->fb_plat_data->disp_dev_no);
	if(disp_dev) {
		if(disp_dev->ops->getedid(sfb->edid)) {
			fb_edid_to_monspecs(sfb->edid, specs);
		}
		if (specs->modedb == NULL)
			printk("Unable to get Mode Database\n");
		else {
			const struct fb_videomode *m;
			fb_videomode_to_modelist(specs->modedb,
						 specs->modedb_len,
						 &info->modelist);
			m = fb_find_best_display(specs, &info->modelist);
			if (m) {
				disp_dev->ops->config_video(disp_dev, &info->var);
				fb_videomode_to_var(&info->var, m);
				/* fill all other info->var's fields */
				if (astfb_check_var(&info->var, info) < 0) {
					//no found , false
				} else
					found = true;
			}
		}
	}else {
			printk("No Display device !! \n");
	}
#endif

	if(!specs->modedb) {
		specs->modedb = ast_modedb;
		specs->modedb_len = 4;
	}

	if (!mode_option && !found) {
		//use default modes list 
#ifdef AST_SOC_G5
		if(sfb->fb_plat_data->set_pll) {
			mode_option = "1920x1080-32@60";
			if (sfb->fb_plat_data->clock_src == 24000000) {
			} else {
			}
		} else {
			mode_option = "800x600-32@60";
		}
#else
		//AST2500 always use 800x600-32@60
		mode_option = "800x600-32@60";
#endif
		printk("use default mode : %s \n", mode_option);

#if 0			
		const struct fb_videomode *m;
		printk("mode list \n");
		fb_videomode_to_modelist(&modedb,
					 3,
					 &info->modelist);

		printk("find best disp \n");
		m = fb_find_best_display(specs, &info->modelist);
		if (m) {
			fb_videomode_to_var(&info->var, m);
			/* fill all other info->var's fields */
			if (astfb_check_var(&info->var, info) < 0) {
				printk("TODO ........... var ]]] \n");
				//info->var = tdfx_var;
			} else
				found = true;
		}
#endif

		sfb->fb_res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
		info->fix.smem_start = sfb->fb_res->start;
		info->fix.smem_len = sfb->fb_res->end - sfb->fb_res->start + 1;

	}

	if (mode_option) {
		if(fb_find_mode(&info->var, info, mode_option, 
						specs->modedb, specs->modedb_len, 
						NULL, info->var.bits_per_pixel) != 1) {
			dev_err(dev, "cannot find db modes \n");
			ret = -ENOMEM;
			goto free_res;
		}
	}

	if (found) {
		fb_destroy_modedb(specs->modedb);
		specs->modedb = NULL;
	}

	/* resource allocation */
//	info->fix.smem_len = SZ_2M * ((info->var.bits_per_pixel)/8 * NUMBER_OF_BUFFERS); //assign 16M for 1920*1080*32it double-buffering

	printk("info->fix.smem_start = %x, len = %x , bpp = %d\n",info->fix.smem_start, info->fix.smem_len, info->var.bits_per_pixel);

	if (!request_mem_region(info->fix.smem_start, info->fix.smem_len, pdev->name)) {
		dev_err(dev, "cannot request CRT mem %s\n", pdev->name);
		ret = -EBUSY;
		goto free_io;
	}

	info->screen_base = ioremap(info->fix.smem_start, info->fix.smem_len);
	if (!info->screen_base) {
		dev_err(dev, "cannot map CRT mem %s\n", pdev->name);
		ret = -ENOMEM;
		goto free_addr;
	}

	printk(KERN_INFO "FB Phys:%x, Virtual:%x \n", info->fix.smem_start, info->screen_base);

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if(res) {
		//no cursor frame buffer
		sfb->cursor_phy = res->start;
		sfb->cursor_virt = ioremap(res->start, res->end - res->start);
		if (!sfb->cursor_virt) {
			dev_err(dev, "cannot map CURSOR mem\n");
			ret = -ENOMEM;
			goto free_addr;
		}
	}
//

    info->fix.type          = FB_TYPE_PACKED_PIXELS;
    info->fix.type_aux      = 0;

#if (NUMBER_OF_BUFFERS > 1)
    info->fix.ypanstep = 1;
#else
    info->fix.ypanstep = 0;
#endif

	info->fix.xpanstep		= 0;
	info->fix.ywrapstep 	= 0;
	info->fix.visual 		= FB_VISUAL_TRUECOLOR,
	info->fix.accel 			= FB_ACCEL_NONE;
	info->flags 			= FBINFO_FLAG_DEFAULT;
	info->pseudo_palette	= sfb->pseudo_palette;

	/*
	 * Allocate colourmap.
	 */
	ret=fb_alloc_cmap(&(info->cmap), 256, 0);
	if(ret) {
			dev_err(dev, "Alloc color map failed\n");
			goto free_mem;
	}

	init_waitqueue_head(&wq);

	ret = request_irq(sfb->irq, astfb_isr, IRQF_SHARED, pdev->name, sfb);
	if (ret) {
			dev_err(dev, "Can't request LCD irq");
			ret = -EBUSY;
			goto free_cmap;
	}
	
	ret = astfb_check_var(&info->var, info);
	if (ret)
		goto free_irq;

	init_timer(&sfb->timer);
	sfb->timer.data = (long) sfb;
	sfb->timer.function = sfb_timer;
	astfb_set_par(info);
	platform_set_drvdata(pdev, sfb);
	ret = register_framebuffer(info);
	if (!ret) {
//		for(i=0;i<sizeof(device_attrs)/sizeof(struct device_attribute);i++)
//				device_create_file(info->dev, &device_attrs[i]);
		return 0;
	}

    dev_err(dev, "Failed to register framebuffer device: %d\n", ret);

	astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) & ~CRT_CTRL_GRAPHIC_EN, AST_CRT_CTRL1);
    platform_set_drvdata(pdev, NULL);
free_irq:
    free_irq(sfb->irq,sfb);
free_cmap:
    fb_dealloc_cmap(&info->cmap);
free_mem:
    ast_fbmem_free(sfb);
free_addr:
    if(sfb->addr_assign)
            release_mem_region(info->fix.smem_start, info->fix.smem_len);
free_io:
    iounmap(sfb->base);
free_res:
    release_mem_region(info->fix.mmio_start, info->fix.mmio_len);
free_info:
    framebuffer_release(info);
    return ret;
		
}

static int
astfb_remove(struct platform_device *pdev)
{
	struct astfb_info *sfb = platform_get_drvdata(pdev);
	ASTFB_DBG("astfb_remove \n");

	unregister_framebuffer(sfb->info);
	astfb_write(sfb, astfb_read(sfb, AST_CRT_CTRL1) & ~CRT_CTRL_GRAPHIC_EN, AST_CRT_CTRL1);
	free_irq(sfb->irq,sfb);
	fb_dealloc_cmap(&sfb->info->cmap);
	iounmap(sfb->info->screen_base);
	if(sfb->addr_assign)
		release_mem_region(sfb->info->fix.smem_start, sfb->info->fix.smem_len);
	iounmap(sfb->base);
	release_mem_region(sfb->info->fix.mmio_start, sfb->info->fix.mmio_len);
	framebuffer_release(sfb->info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int astfb_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO */
	return 0;
}

static int astfb_resume(struct platform_device *pdev)
{
	/* TODO */
	return 0;
}
#else
#define astfb_suspend NULL
#define astfb_resume  NULL
#endif

/* driver ops */
static struct platform_driver astfb_driver = {
    .probe      = astfb_probe,
    .remove     = astfb_remove,
    .suspend    = astfb_suspend,
    .resume     = astfb_resume,
    .driver     = {
        .name   = "ast-fb",
        .owner  = THIS_MODULE,
    },
	
};
int __init astfb_init(void)
{
    return platform_driver_register(&astfb_driver);
}

static void __exit astfb_cleanup(void)
{
	printk(KERN_DEBUG "astfb: astfb_remove_module is called \n");

	platform_driver_unregister(&astfb_driver);
}

module_init(astfb_init);
module_exit(astfb_cleanup);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("Framebuffer driver for the ASPEED");
MODULE_LICENSE("GPL");
