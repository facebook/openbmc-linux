/* arch/arm/mach-aspeed/include/mach/regs-video.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2012/08/15 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __AST_VIDEO_H
#define __AST_VIDEO_H                     1

/*
 *  Register for VIDEO
 *  */
#define AST_VIDEO_PROTECT		0x000		/*	protection key register	*/
#define AST_VIDEO_SEQ_CTRL		0x004		/*	Video Sequence Control register	*/
#define AST_VIDEO_PASS_CTRL		0x008		/*	Video Pass 1 Control register	*/

//VR008[5]=1
#define AST_VIDEO_DIRECT_BASE	0x00C		/*	Video Direct Frame buffer mode control Register VR008[5]=1 */
#define AST_VIDEO_DIRECT_CTRL	0x010		/*	Video Direct Frame buffer mode control Register VR008[5]=1 */

//VR008[5]=0
#define AST_VIDEO_TIMING_H		0x00C		/*	Video Timing Generation Setting Register */
#define AST_VIDEO_TIMING_V		0x010		/*	Video Timing Generation Setting Register */
#define AST_VIDEO_SCAL_FACTOR	0x014		/*	Video Scaling Factor Register */

#define AST_VIDEO_SCALING0		0x018		/*	Video Scaling Filter Parameter Register #0 */
#define AST_VIDEO_SCALING1		0x01C		/*	Video Scaling Filter Parameter Register #1 */
#define AST_VIDEO_SCALING2		0x020		/*	Video Scaling Filter Parameter Register #2 */
#define AST_VIDEO_SCALING3		0x024		/*	Video Scaling Filter Parameter Register #3 */

#define AST_VIDEO_BCD_CTRL		0x02C		/*	Video BCD Control Register */
#define AST_VIDEO_CAPTURE_WIN	0x030		/*	 Video Capturing Window Setting Register */
#define AST_VIDEO_COMPRESS_WIN	0x034		/*	 Video Compression Window Setting Register */


#define AST_VIDEO_COMPRESS_PRO	0x038		/* Video Compression Stream Buffer Processing Offset Register */
#define AST_VIDEO_COMPRESS_READ	0x03C		/* Video Compression Stream Buffer Read Offset Register */

#define AST_VIDEO_JPEG_HEADER_BUFF		0x040		/*	Video Based Address of JPEG Header Buffer Register */
#define AST_VIDEO_SOURCE_BUFF0	0x044		/*	Video Based Address of Video Source Buffer #1 Register */
#define AST_VIDEO_SOURCE_SCAN_LINE	0x048		/*	Video Scan Line Offset of Video Source Buffer Register */
#define AST_VIDEO_SOURCE_BUFF1	0x04C		/*	Video Based Address of Video Source Buffer #2 Register */
#define AST_VIDEO_BCD_BUFF		0x050		/*	Video Base Address of BCD Flag Buffer Register */
#define AST_VIDEO_STREAM_BUFF	0x054		/*	Video Base Address of Compressed Video Stream Buffer Register */
#define AST_VIDEO_STREAM_SIZE	0x058		/*	Video Stream Buffer Size Register */

#define AST_VIDEO_COMPRESS_CTRL	0x060		/* Video Compression Control Register */


#define AST_VIDEO_COMPRESS_DATA_COUNT		0x070		/* Video Total Size of Compressed Video Stream Read Back Register */
#define AST_VIDEO_COMPRESS_BLOCK_COUNT		0x074		/* Video Total Number of Compressed Video Block Read Back Register */
#define AST_VIDEO_COMPRESS_FRAME_END		0x078		/* Video Frame-end offset of compressed video stream buffer read back Register */



#define AST_VIDEO_DEF_HEADER	0x080		/* Video User Defined Header Parameter Setting with Compression */
#define AST_VIDEO_JPEG_COUNT	0x084		/*  */

#define AST_VIDEO_H_DETECT_STS  0x090		/* Video Source Left/Right Edge Detection Read Back Register */
#define AST_VIDEO_V_DETECT_STS  0x094		/* Video Source Top/Bottom Edge Detection Read Back Register */


#define AST_VIDEO_MODE_DET_STS	0x098		/* Video Mode Detection Status Read Back Register */
 
#define AST_VIDEO_MODE_DET1		0x0A4		/* Video Mode Detection Control Register 1*/

#define AST_VM_SEQ_CTRL			0x204		/* Video Management Control Sequence Register */
#define AST_VM_PASS_CTRL			0x208		/* Video Management Pass 1 Control register	*/
#define AST_VM_SCAL_FACTOR		0x214		/* Video Management Scaling Factor Register */
#define AST_VM_BCD_CTRL			0x22C		/* Video Management BCD Control Register */
#define AST_VM_CAPTURE_WIN		0x230		/* Video Management Capturing Window Setting Register */
#define AST_VM_COMPRESS_WIN		0x234		/* Video Management Compression Window Setting Register */
#define AST_VM_JPEG_HEADER_BUFF	0x240		/* Video Management Based Address of JPEG Header Buffer Register */
#define AST_VM_SOURCE_BUFF0		0x244		/* Video Management Based Address of Video Source Buffer Register */
#define AST_VM_SOURCE_SCAN_LINE	0x248		/* Video Management Scan Line Offset of Video Source Buffer Register */

#define AST_VM_COMPRESS_BUFF		0x254		/* Video Management Based Address of Compressed Video Buffer Register */
#define AST_VM_STREAM_SIZE			0x258		/* Video Management Buffer Size Register */
#define AST_VM_COMPRESS_CTRL			0x260		/* Video Management Compression or Video Profile 2-5 Decompression Control Register */
#define AST_VM_COMPRESS_VR264			0x264		/* VR264 REserved */
#define AST_VM_COMPRESS_FRAME_END	0x278		/* Video Management Frame-end offset of compressed video stream buffer read back Register */


#define AST_VIDEO_CTRL			0x300		/* Video Control Register */
#define AST_VIDEO_INT_EN		0x304		/* Video interrupt Enable */
#define AST_VIDEO_INT_STS		0x308		/* Video interrupt status */
#define AST_VIDEO_MODE_DETECT	0x30C		/* Video Mode Detection Parameter Register */

#define AST_VIDEO_CRC1 			0x320		/* Primary CRC Parameter Register */
#define AST_VIDEO_CRC2 			0x324		/* Second CRC Parameter Register */
#define AST_VIDEO_DATA_TRUNCA	0x328		/* Video Data Truncation Register */


#define AST_VIDEO_SCRATCH_340	0x340		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_344	0x344		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_348	0x348		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_34C	0x34C		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_350	0x350		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_354	0x354		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_358	0x358		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_35C	0x35C		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_360	0x360		/* Video Scratch Remap Read Back */
#define AST_VIDEO_SCRATCH_364	0x364		/* Video Scratch Remap Read Back */


#define AST_VIDEO_ENCRYPT_SRAM	0x400		/* Video RC4/AES128 Encryption Key Register #0 ~ #63 */

/////////////////////////////////////////////////////////////////////////////

/*	AST_VIDEO_PROTECT: 0x000  - protection key register */
#define VIDEO_PROTECT_UNLOCK			0x1A038AA8

/*	AST_VIDEO_SEQ_CTRL		0x004		Video Sequence Control register	*/
#define VIDEO_HALT_ENG_STS				(1 << 21)
#define VIDEO_COMPRESS_BUSY				(1 << 18)
#define VIDEO_CAPTURE_BUSY				(1 << 16)
#define VIDEO_HALT_ENG_TRIGGER			(1 << 12)
#define VIDEO_COMPRESS_FORMAT_MASK		(3 << 10)
#define VIDEO_GET_COMPRESS_FORMAT(x)		((x >> 10) & 0x3)   // 0 YUV444
#define VIDEO_COMPRESS_FORMAT(x)		(x << 10)	// 0 YUV444
#define YUV420		1
#ifdef AST_SOC_G5
#define VIDEO_COMPRESS_JPEG_MODE			(1 << 13)
#define VIDEO_YUV2RGB_DITHER_EN			(1 << 8)
#else
#define VIDEO_COMPRESS_JPEG_MODE			(1 << 8)
#endif
//if bit 0 : 1
#define VIDEO_INPUT_MODE_CHG_WDT		(1 << 7)
#define VIDEO_INSERT_FULL_COMPRESS		(1 << 6)
#define VIDEO_AUTO_COMPRESS				(1 << 5)
#define VIDEO_COMPRESS_TRIGGER			(1 << 4)
#define VIDEO_CAPTURE_MULTI_FRAME		(1 << 3)
#define VIDEO_COMPRESS_FORCE_IDLE		(1 << 2)
#define VIDEO_CAPTURE_TRIGGER			(1 << 1)
#define VIDEO_DETECT_TRIGGER			(1 << 0)


#define VIDEO_HALT_ENG_RB				(1 << 21)
#define VIDEO_HALT_ENG_RB				(1 << 21)
#define VIDEO_HALT_ENG_RB				(1 << 21)
#define VIDEO_HALT_ENG_RB				(1 << 21)
#define VIDEO_HALT_ENG_RB				(1 << 21)
#define VIDEO_HALT_ENG_RB				(1 << 21)


/*	AST_VIDEO_PASS_CTRL			0x008		Video Pass1 Control register	*/
//x * source frame rate / 60 
#define VIDEO_FRAME_RATE_CTRL(x)		(x << 16)
#define VIDEO_HSYNC_POLARITY_CTRL		(1 << 15)
#define VIDEO_INTERLANCE_MODE			(1 << 14)
#define VIDEO_DUAL_EDGE_MODE			(1 << 13)	//0 : Single edage
#define VIDEO_18BIT_SINGLE_EDGE			(1 << 12)	//0: 24bits
#define VIDEO_DVO_INPUT_DELAY_MASK		(7 << 9)	
#define VIDEO_DVO_INPUT_DELAY(x)		(x << 9) //0 : no delay , 1: 1ns, 2: 2ns, 3:3ns, 4: inversed clock but no delay
// if biit 5 : 0
#define VIDEO_HW_CURSOR_DIS				(1 << 8)
// if biit 5 : 1
#define VIDEO_AUTO_FATCH					(1 << 8)	//
#define VIDEO_CAPTURE_FORMATE_MASK		(3 << 6)

#define VIDEO_SET_CAPTURE_FORMAT(x)			(x << 6)
#define JPEG_MODE		1
#define RGB_MODE		2
#define GRAY_MODE		3
#define VIDEO_DIRT_FATCH				(1 << 5)
// if biit 5 : 0
#define VIDEO_INTERNAL_DE				(1 << 4)
#define VIDEO_EXT_ADC_ATTRIBUTE			(1 << 3)	

// if biit 5 : 1
#define VIDEO_16BPP_MODE				(1 << 4)
#define VIDEO_16BPP_MODE_555			(1 << 3)	//0:565

#define VIDEO_FROM_EXT_SOURCE			(1 << 2)	
#define VIDEO_SO_VSYNC_POLARITY			(1 << 1)
#define VIDEO_SO_HSYNC_POLARITY			(1 << 0)

/*	AST_VIDEO_TIMING_H		0x00C		Video Timing Generation Setting Register */
#define VIDEO_HSYNC_PIXEL_FIRST_SET(x)	((x) << 16)
#define VIDEO_HSYNC_PIXEL_FIRST_MASK	0x0000FFFF
#define VIDEO_HSYNC_PIXEL_LAST_SET(x)	(x)
#define VIDEO_HSYNC_PIXEL_LAST_MASK	0xFFFF0000


/*	AST_VIDEO_DIRECT_CTRL	0x010		Video Direct Frame buffer mode control Register VR008[5]=1 */
#define VIDEO_FETCH_TIMING(x)			((x) << 16)
#define VIDEO_FETCH_LINE_OFFSET(x)		(x)

/*	AST_VIDEO_TIMING_V		0x010		Video Timing Generation Setting Register */
#define VIDEO_VSYNC_PIXEL_FIRST_SET(x)	((x) << 16)
#define VIDEO_VSYNC_PIXEL_LAST_SET(x)	(x)


/*	AST_VIDEO_SCAL_FACTOR	0x014		Video Scaling Factor Register */
#define VIDEO_V_SCAL_FACTOR(x)			(((x) & 0xffff) << 16)
#define VIDEO_V_SCAL_FACTOR_MASK		(0x0000ffff)
#define VIDEO_H_SCAL_FACTOR(x)			(x & 0xffff)
#define VIDEO_H_SCAL_FACTOR_MASK		(0xffff0000)



/*	AST_VIDEO_SCALING0		0x018		Video Scaling Filter Parameter Register #0 */
/*	AST_VIDEO_SCALING1		0x01C		Video Scaling Filter Parameter Register #1 */
/*	AST_VIDEO_SCALING2		0x020		Video Scaling Filter Parameter Register #2 */
/*	AST_VIDEO_SCALING3		0x024		Video Scaling Filter Parameter Register #3 */


/*	AST_VIDEO_BCD_CTRL		0x02C		Video BCD Control Register */
#define VIDEO_ABCD_TOL(x)				(x << 24)
#define VIDEO_BCD_TOL(x)				(x << 16)
#define VIDEO_ABCD_CHG_EN				(1 << 1)
#define VIDEO_BCD_CHG_EN				(1 << 0)



/*	 AST_VIDEO_CAPTURE_WIN	0x030		Video Capturing Window Setting Register */
#define VIDEO_CAPTURE_V(x)				(x & 0x7ff)
#define VIDEO_CAPTURE_H(x)				((x & 0x7ff) << 16)

/*	 AST_VIDEO_COMPRESS_WIN	0x034		Video Compression Window Setting Register */
#define VIDEO_COMPRESS_V(x)				(x & 0x7ff)
#define VIDEO_GET_COMPRESS_V(x)			(x & 0x7ff)
#define VIDEO_COMPRESS_H(x)				((x & 0x7ff) << 16)
#define VIDEO_GET_COMPRESS_H(x)			((x >> 16) & 0x7ff)



/*	AST_VIDEO_RESET :0x03c	 - system reset control register */

/*	AST_VIDEO_STREAM_SIZE	0x058		Video Stream Buffer Size Register */
#define VIDEO_STREAM_PKT_N(x)			(x << 3)
#define STREAM_4_PKTS		0
#define STREAM_8_PKTS		1
#define STREAM_16_PKTS		2
#define STREAM_32_PKTS		3
#define STREAM_64_PKTS		4
#define STREAM_128_PKTS		5

#define VIDEO_STREAM_PKT_SIZE(x)		(x)
#define STREAM_1KB		0
#define STREAM_2KB		1
#define STREAM_4KB		2
#define STREAM_8KB		3
#define STREAM_16KB		4
#define STREAM_32KB		5
#define STREAM_64KB		6
#define STREAM_128KB	7

/* AST_VIDEO_COMPRESS_CTRL	0x060		Video Compression Control Register */
#define VIDEO_HQ_DCT_LUM(x)				((x) << 27)
#define VIDEO_GET_HQ_DCT_LUM(x)			((x >> 27) & 0x1f)
#define VIDEO_HQ_DCT_CHROM(x)				((x) << 22)
#define VIDEO_GET_HQ_DCT_CHROM(x)			((x >> 22) & 0x1f)
#define VIDEO_HQ_DCT_MASK					(0x3ff << 22)
#define VIDEO_DCT_HUFFMAN_ENCODE(x)		((x) << 20)
#define VIDEO_DCT_RESET					(1 << 17)
#define VIDEO_HQ_ENABLE					(1 << 16)
#define VIDEO_GET_HQ_ENABLE(x)				((x >> 16) & 0x1)
#define VIDEO_DCT_LUM(x)					((x) << 11)
#define VIDEO_GET_DCT_LUM(x)				((x >> 11) & 0x1f)
#define VIDEO_DCT_CHROM(x)					((x) << 6)
#define VIDEO_GET_DCT_CHROM(x)			((x >> 6) & 0x1f)
#define VIDEO_DCT_MASK						(0x3ff << 6)
#define VIDEO_ENCRYP_ENABLE				(1 << 5)
#define VIDEO_COMPRESS_QUANTIZ_MODE		(1 << 2)
#define VIDEO_4COLOR_VQ_ENCODE			(1 << 1)
#define VIDEO_DCT_ONLY_ENCODE				(1 << 0)
#define VIDEO_DCT_VQ_MASK					(0x3)

/* AST_VIDEO_COMPRESS_BLOCK_COUNT - 0x074		Video Total Number of Compressed Video Block Read Back Register */
#define GET_BLOCK_CHG(x)				((x >> 16) & 0xffff)

/* AST_VIDEO_H_DETECT_STS  0x090		Video Source Left/Right Edge Detection Read Back Register */
#define VIDEO_DET_INTERLANCE_MODE		(1 << 31)
#define VIDEO_GET_HSYNC_RIGHT(x)		((x & 0x0FFF0000) >> 16)
#define VIDEO_GET_HSYNC_LEFT(x)			(x & 0xFFF)
#define VIDEO_NO_DISPLAY_CLOCK_DET		(1 << 15)
#define VIDEO_NO_ACT_DISPLAY_DET		(1 << 14)
#define VIDEO_NO_HSYNC_DET				(1 << 13)
#define VIDEO_NO_VSYNC_DET				(1 << 12)

/* AST_VIDEO_V_DETECT_STS  0x094		Video Source Top/Bottom Edge Detection Read Back Register */
#define VIDEO_GET_VSYNC_BOTTOM(x)		((x & 0x0FFF0000) >> 16)
#define VIDEO_GET_VSYNC_TOP(x)			(x & 0xFFF)


/* AST_VIDEO_MODE_DET_STS	0x098		Video Mode Detection Status Read Back Register */
#define VIDEO_DET_HSYNC_RDY				(1 << 31)
#define VIDEO_DET_VSYNC_RDY				(1 << 30)
#define VIDEO_DET_HSYNC_POLAR			(1 << 29)
#define VIDEO_DET_VSYNC_POLAR			(1 << 28)
#define VIDEO_GET_VER_SCAN_LINE(x)		((x >> 16) & 0xfff)
#define VIDEO_OUT_SYNC					(1 << 15)
#define VIDEO_DET_VER_STABLE			(1 << 14)
#define VIDEO_DET_HOR_STABLE			(1 << 13)
#define VIDEO_DET_FROM_ADC				(1 << 12)
#define VIDEO_DET_HOR_PERIOD(x)			(x & 0xfff)


/* AST_VIDEO_MODE_DET1		0x0A4		Video Mode Detection Control Register 1*/
#define VIDEO_DET_HSYNC_DELAY_MASK		(0xff << 16)
#define VIDEO_DET_LONG_H_STABLE_EN		(1 << 29)

/* AST_VM_SEQ_CTRL	0x204		Video Management Control Sequence Register */
#define VIDEO_VM_SET_YUV420				(1 << 10)
#define VIDEO_VM_JPEG_COMPRESS_MODE		(1 << 8)
#define VIDEO_VM_AUTO_COMPRESS			(1 << 5)
#define VIDEO_VM_COMPRESS_TRIGGER		(1 << 4)
#define VIDEO_VM_CAPTURE_TRIGGER			(1 << 1)

/* AST_VM_BUFF_SIZE			0x258		Video Management Buffer Size Register */
#define VM_STREAM_PKT_SIZE(x)		(x)
#define STREAM_1MB		0
#define STREAM_2MB		1
#define STREAM_3MB		2
#define STREAM_4MB		3

/* AST_VIDEO_CTRL			0x300		Video Control Register */
#define VIDEO_CTRL_CRYPTO(x)			(x << 17)
#define VIDEO_CTRL_CRYPTO_AES				(1 << 17)
#define VIDEO_CTRL_CRYPTO_FAST			(1 << 16)
//15 reserved
#define VIDEO_CTRL_RC4_VC				(1 << 14)
#define VIDEO_CTRL_CAPTURE_MASK			(3 << 12)
#define VIDEO_CTRL_CAPTURE_MODE(x)		(x << 12)
#define VIDEO_CTRL_COMPRESS_MASK		(3 << 10)
#define VIDEO_CTRL_COMPRESS_MODE(x)		(x << 10)
#define MODE_32BPP_YUV444		0
#define MODE_24BPP_YUV444		1
#define MODE_16BPP_YUV422		3

#define VIDEO_CTRL_RC4_TEST_MODE		(1 << 9)
#define VIDEO_CTRL_RC4_RST				(1 << 8)
#define VIDEO_CTRL_RC4_VIDEO_M_SEL		(1 << 7)		//video management 
#define VIDEO_CTRL_RC4_VIDEO_2_SEL		(1 << 6)		// Video 2 

#define VIDEO_CTRL_DWN_SCALING_MASK		(0x3 << 4)
#define VIDEO_CTRL_DWN_SCALING(x)		(x << 4)
#define DWN_V1		0x1
#define DWN_V2		0x2
#define DWN_VM		0x3

#define VIDEO_CTRL_VSYNC_DELAY_MASK		(3 << 2)
#define VIDEO_CTRL_VSYNC_DELAY(x)		(x << 2)
#define NO_DELAY			0
#define DELAY_DIV12_HSYNC	1
#define AUTO_DELAY			2

/* AST_VIDEO_INT_EN			0x304		Video interrupt Enable */
/* AST_VIDEO_INT_STS		0x308		Video interrupt status */
#define VM_COMPRESS_COMPLETE			(1 << 17)
#define VM_CAPTURE_COMPLETE			(1 << 16)

#define VIDEO_FRAME_COMPLETE			(1 << 5)
#define VIDEO_MODE_DETECT_RDY			(1 << 4)
#define VIDEO_COMPRESS_COMPLETE			(1 << 3)
#define VIDEO_COMPRESS_PKT_COMPLETE		(1 << 2)
#define VIDEO_CAPTURE_COMPLETE			(1 << 1)
#define VIDEO_MODE_DETECT_WDT			(1 << 0)

/* AST_VIDEO_MODE_DETECT	0x30C		Video Mode Detection Parameter Register */
#define VIDEO_MODE_HOR_TOLER(x)			(x << 28)
#define VIDEO_MODE_VER_TOLER(x)			(x << 24)
#define VIDEO_MODE_HOR_STABLE(x)		(x << 20)
#define VIDEO_MODE_VER_STABLE(x)		(x << 16)
#define VIDEO_MODE_EDG_THROD(x)			(x << 8)

#define MODEDETECTION_VERTICAL_STABLE_MAXIMUM       0x6
#define MODEDETECTION_HORIZONTAL_STABLE_MAXIMUM     0x6
#define MODEDETECTION_VERTICAL_STABLE_THRESHOLD     0x2
#define MODEDETECTION_HORIZONTAL_STABLE_THRESHOLD   0x2

/* AST_VIDEO_SCRATCH_34C	0x34C		Video Scratch Remap Read Back */
#define SCRATCH_VGA_GET_REFLASH_RATE(x)			((x >> 8) & 0xf)
#define SCRATCH_VGA_GET_COLOR_MODE(x)			((x >> 4) & 0xf)

/* AST_VIDEO_SCRATCH_350	0x350		Video Scratch Remap Read Back */
#define SCRATCH_VGA_GET_MODE_HEADER(x)			((x >> 8) & 0xff)
#define SCRATCH_VGA_GET_NEW_COLOR_MODE(x)		((x >> 16) & 0xff)
#define SCRATCH_VGA_GET_NEW_PIXEL_CLK(x)		((x >> 24) & 0xff)


/* AST_VIDEO_SCRATCH_35C	0x35C		Video Scratch Remap Read Back */
#define SCRATCH_VGA_PWR_STS_HSYNC				(1 << 31)
#define SCRATCH_VGA_PWR_STS_VSYNC				(1 << 30)
#define SCRATCH_VGA_ATTRIBTE_INDEX_BIT5			(1 << 29)
#define SCRATCH_VGA_MASK_REG					(1 << 28)
#define SCRATCH_VGA_CRT_RST						(1 << 27)
#define SCRATCH_VGA_SCREEN_OFF					(1 << 26)
#define SCRATCH_VGA_RESET						(1 << 25)
#define SCRATCH_VGA_ENABLE						(1 << 24)


#endif

