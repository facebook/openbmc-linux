/* linux/include/asm-arm/arch-aspeed/regs-crt.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ___ASM_ARCH_REGS_CRT_H
#define ___ASM_ARCH_REGS_CRT_H 

/* CRT control registers */


//////////////////////////////////////////////////////////////////
#define AST3000_VGA1_CTLREG	0x00
#define AST3000_VGA1_CTLREG2	0x04
#define AST3000_VGA1_STATUSREG	0x08 
#define AST3000_VGA1_PLL	0x0C
#define AST3000_VGA1_HTREG	0x10
#define AST3000_VGA1_HRREG	0x14
#define AST3000_VGA1_VTREG	0x18
#define AST3000_VGA1_VRREG	0x1C
#define AST3000_VGA1_STARTADDR	0x20
#define AST3000_VGA1_OFFSETREG	0x24
#define AST3000_VGA1_THRESHOLD	0x28

#define AST3000_VGA2_CTLREG	0x60
#define AST3000_VGA2_CTLREG2	0x64
#define AST3000_VGA2_STATUSREG	0x68 
#define AST3000_VGA2_PLL	0x6C
#define AST3000_VGA2_HTREG	0x70
#define AST3000_VGA2_HRREG	0x74
#define AST3000_VGA2_VTREG	0x78
#define AST3000_VGA2_VRREG	0x7C
#define AST3000_VGA2_STARTADDR	0x80
#define AST3000_VGA2_OFFSETREG	0x84
#define AST3000_VGA2_THRESHOLD	0x88
//////////////////////////////////////////////////////////////////

//0x00 ~ 0x5F Reserved - FB0 
//0x60 ~ 90 FB1
#define AST_CRT_CTRL1				0x60
#define AST_CRT_CTRL2				0x64
#define AST_CRT_STS					0x68
#define AST_CRT_PLL					0x6C
#define AST_CRT_HORIZ0				0x70
#define AST_CRT_HORIZ1				0x74
#define AST_CRT_VERTI0				0x78
#define AST_CRT_VERTI1				0x7C
#define AST_CRT_ADDR				0x80
#define AST_CRT_OFFSET				0x84
#define AST_CRT_THROD				0x88
#define AST_CRT_XSCALING			0x8C
//0x8c Reserved
//0x90 ~ Cursor
#define AST_CRT_CURSOR0				0x90
#define AST_CRT_CURSOR1				0x94
#define AST_CRT_CURSOR2				0x98
#define AST_CRT_UADDR				0x9C
//0x9c Reserved
//0xA0 ~ OSD
#define AST_CRT_OSDH				0xA0
#define AST_CRT_OSDV				0xA4
#define AST_CRT_OSDADDR				0xA8
#define AST_CRT_OSDDISP				0xAC
#define AST_CRT_OSDTHROD			0xB0
#define AST_CRT_VADDR				0xB4

//0xb4 Reserved
#define AST_CRT_STS_V				0xB8
#define AST_CRT_SCRATCH				0xBC
#define AST_CRT_X					0xC0
//0xC4
#define AST_CRT_OSD_COLOR			0xE0

/* AST_CRT_CTRL1 - 0x60 : CRT Control Register I */
#define CRT_CTRL_VERTICAL_INTR_STS	(0x1 << 31) 
#define CRT_CTRL_VERTICAL_INTR_EN	(0x1 << 30) 
//24~28 reserved
#define CRT_CTRL_DESK_OFF			(0x1 << 23) 
#define CRT_CTRL_FSYNC_OFF			(0x1 << 22) 
#define CRT_CTRL_FSYNC_POLARITY		(0x1 << 21) 
#define CRT_CTRL_SCREEN_OFF			(0x1 << 20) 
#define CRT_CTRL_VSYNC_OFF			(0x1 << 19) 
#define CRT_CTRL_HSYNC_OFF			(0x1 << 18) 
#define CRT_CTRL_VSYNC_POLARITY		(0x1 << 17) 
#define CRT_CTRL_HSYNC_POLARITY		(0x1 << 16) 
#define CRT_CTRL_TILE_EN			(0x1 << 15) 
#define CRT_CTRL_HDTVYUV_EN			(0x1 << 14) 
#define CRT_CTRL_YUV_FORMAT(x)		(x << 12) 
#define YUV_MODE0					0
#define YUV_MODE1					1
#define YUV_MODE2					2
// bit 11 reserved 
#define CRT_CTRL_HW_CURSOR_FORMAT	(0x1 << 10) // 0: XRGB4444, 1:ARGB4444
#define CRT_CTRL_FORMAT_MASK		(0x7 << 7)
#define CRT_CTRL_FORMAT(x)			(x << 7)
#define COLOR_RGB565				(0)
#define COLOR_YUV444				(1)
#define COLOR_XRGB8888				(2)
#define COLOR_YUV444_2RGB			(5)
#define CRT_CTRL_ENVEFLIP			(0x1 << 6)
//bit 5
#define CRT_CTRL_SCALING_X			(0x1 << 4)
#define CRT_CTRL_INTER_TIMING		(0x1 << 3)
#define CRT_CTRL_OSD_EN				(0x1 << 2)
#define CRT_CTRL_HW_CURSOR_EN		(0x1 << 1)
#define CRT_CTRL_GRAPHIC_EN			(0x1)

/*AST_CRT_CTRL2 - 0x64 : CRT Control Register II */
#define CRT_CTRL_VLINE_NUM_MASK		(0xfff << 20)
#define CRT_CTRL_VLINE_NUM(x)		(x << 20)
#define CRT_CTRL_TESTDVO_MASK		(0xfff << 8)
#define CRT_CTRL_TESTDVO(x)			(x << 8)
#define CRT_CTRL_DVO_EN				(0x1 << 7)
#define CRT_CTRL_DVO_DUAL			(0x1 << 6)
#define CRT_CTRL_FIFO_FULL			(0x1 << 5)
#define CRT_CTRL_TEST_EN			(0x1 << 4)
#define CRT_CTRL_SIGN_DON			(0x1 << 3)
#define CRT_CTRL_SIGN_TRIGGER		(0x1 << 2)
#define CRT_CTRL_DAC_TEST_EN		(0x1 << 1)
#define CRT_CTRL_DAC_PWR_EN			(0x1)

/* AST_CRT_STS	- 0x68 : CRT Status Register */
#define CRT_STS_RED_RB(x)			(x << 24)
#define CRT_STS_GREEN_RB(x)			(x << 16)
#define CRT_STS_BLUE_RB(x)			(x << 8)
#define CRT_STS_DAC_SENSE_EN		(0x1 << 7)
//6 reserved
#define CRT_STS_ODDFIELD_SYNC		(0x1 << 5)
#define CRT_STS_ODDFIELD			(0x1 << 4)
#define CRT_STS_HDISPLAY_RB			(0x1 << 3)
#define CRT_STS_HRETRACE_RB			(0x1 << 2)
#define CRT_STS_VDISPLAY_RB			(0x1 << 1)
#define CRT_STS_VRETRACE_RB			(0x1)

/* AST_CRT_PLL -  0x6C : CRT Video PLL Setting Register */
#define CRT_PLL_DAC_MODE_SENSE(x)	(x << 30)
#define CRT_PLL_DAC_SENSE(x)		(x << 28)
#define CRT_PLL_BYPASS				(0x1 << 17)
#define CRT_PLL_PWR_DWN				(0x1 << 16)
#define CRT_PLL_POST_DIVIDER(x)		(((x & 0x3) << 13) | (((x >> 2) & 0xf) << 18) | (((x >> 6) & 0x1) << 23) | (((x >> 7) & 0x1) << 22))
#define CRT_PLL_DENUM(x)			(x << 8)
#define CRT_PLL_NUM(x)				(x)

/* AST_CRT_HORIZ0 - 0x70 : CRT Horizontal Total & Display Enable End Register */
#define CRT_H_TOTAL(x)				(x)
#define CRT_H_DE(x) 				(x << 16)

/* AST_	0x74 : CRT Horizontal Retrace Start & End Register */
#define CRT_H_RS_START(x)			(x)
#define CRT_H_RS_END(x)				(x << 16)

/* AST_CRT_ - 0x78 : CRT Horizontal Total & Display Enable End Register */
#define CRT_V_TOTAL(x)				(x)
#define CRT_V_DE(x) 				(x << 16)

/* AST_	0x7C : CRT Horizontal Retrace Start & End Register */
#define CRT_V_RS_START(x)			(x)
#define CRT_V_RS_END(x)				(x << 16)

/* AST_CRT_OFFSET - 0x84 : CRT Display Offset & Terminal Count Register */
#define CRT_DISP_OFFSET(x)			(x)
#define CRT_TERM_COUNT(x)			(x << 16)

/* AST_CRT_THROD - 0x88 : CRT Threadhold Register */
#define CRT_THROD_LOW(x)			(x)
#define CRT_THROD_HIGH(x)			(x << 8)
#define CRT_THROD_X_SCALING(x)		(x << 16)
#define CRT_THROD_CRT2Y				(0x1 << 20)

/* AST_CRT_XSCALING - 0x8C : CRT X Scaling-up Factor Register */


/* AST_CRT_CURSOR0	: 0x90 - CRT Hardware Cursor X & Y Offset Register */
#define CRT_HW_CURSOR_X_OFFSET(x)			(x)
#define CRT_HW_CURSOR_Y_OFFSET(x)			(x << 16)

/* AST_CRT_CURSOR1 : 0x94 - CRT Hardware Cursor X & Y Position Register */
#define CRT_HW_CURSOR_X_POSITION(x)			(x)
#define CRT_HW_CURSOR_Y_POSITION(x)			(x << 16)

#endif /* ___ASM_ARCH_REGS_CRT_H */
