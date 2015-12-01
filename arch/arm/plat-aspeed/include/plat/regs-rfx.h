/* arch/arm/mach-aspeed/include/mach/regs-rfx.h
 *
 * Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *   History      : 
 *    1. 2014/12/29 Ryan Chen Create
 * 
********************************************************************************/
#ifndef __AST_RFX_H
#define __AST_RFX_H                     1

/*RFX decode engine*/
#define AST_RFX_EPDEC_FIFO_STS		0x000		/* FIFO Status And RFXDEC Version Register */
#define AST_RFX_EPDEC_RING_START		0x004		/* EPDEC BitStream Ring-Buffer Start Address Register */
#define AST_RFX_EPDEC_RING_END		0x008		/* EPDEC BitStream Ring-Buffer End Address Register */
#define AST_RFX_EPDEC_BS_START		0x00C		/* EPDEC BitStream Start Address Register */
#define AST_RFX_EPDEC_BITPOST_START	0x010		/* EPDEC Content BitPost Start Address Register */
#define AST_RFX_EPDEC_SIGNBIT_START	0x014		/* EPDEC Content SignBit Start Address Register */
#define AST_RFX_EPDEC_TILEC_START		0x018		/* EPDEC Content Tile Coefficient Start Address Register */
#define AST_RFX_EPDEC_XY_SIZE			0x01C		/* EPDEC Vertical Size and Horizontal Size Register */
#define AST_RFX_EPDEC_ENG_RDY			0x020		/* EPDEC Engine Ready Register */
#define AST_RFX_EPDEC_PARAMETER0		0x024		/* EPDEC Paramter Register */
#define AST_RFX_EPDEC_PARAMETER1		0x028		/* EPDEC Paramter Register */
#define AST_RFX_EPDEC_IDWTFIFO_CTRL	0x02C		/* EPDEC IDWT FIFO Control Register */
#define AST_RFX_EPDEC_BSFIFO_CTRL		0x030		/* EPDEC BitStream FIFO Control Register */
#define AST_RFX_EPDEC_CTRL				0x034		/* EPDEC Control Register */
#define AST_RFX_EPDEC_ISR_CLR			0x038		/* EPDEC Interrupt Clear Register */
#define AST_RFX_EPDEC_IER				0x03C		/* EPDEC Interrupt Masker Register */
#define AST_RFX_EPDEC_ISR				0x040		/* EPDEC Interrupt Status Register */
#define AST_RFX_EPDEC_LAST_START		0x044		/* EPDEC BitStream Start Address Of The Last Time Register */
#define AST_RFX_EPDEC_RING_READP		0x048		/* EPDEC BitStream Ring-Buffer Read Point Register */
#define AST_RFX_EPDEC_BITPOST_END		0x04C		/* EPDEC Content BitPost End Address Register */
#define AST_RFX_EPDEC_SIGNBIT_END		0x050		/* EPDEC Content SignBit End Address Register */
#define AST_RFX_EPDEC_TILEC_END		0x054		/* EPDEC Content Tile Coefficient End Address Register */
#define AST_RFX_EPDEC_ENG_STS			0x058		/* EPDEC Engine Status Register */
#define AST_RFX_REGION_MESSAGE		0x05C		/* Region Message Register */
#define AST_RFX_TILESET_MESSAGE0		0x060		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE1		0x064		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE2		0x068		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE3		0x06C		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE4		0x070		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE5		0x074		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE6		0x078		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE7		0x07C		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE8		0x080		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE9		0x084		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE10		0x088		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE11		0x08C		/* Tile Set Message Register */
#define AST_RFX_TILESET_MESSAGE12		0x090		/* Tile Set Message Register */
#define AST_RFX_EPDEC_QT_TABLE		0x094		/* EPDEC Quantization Table Register */

/*bulk decompression engine*/
#define AST_RFX_BULK_FIFO_STS			0x100		/* FIFO Status and BULKDEC Version Register */
#define AST_RFX_BULK_RING_START		0x104		/* BULKDEC BitStream Ring-Buffer Start Address Register */
#define AST_RFX_BULK_RING_END			0x108		/* BULKDEC BitStream Ring-Buffer End Address Register */
#define AST_RFX_BULK_BS_START			0x10C		/* BULKDEC BitStream Start Address Register */
#define AST_RFX_BULK_BS_SIZE			0x110		/* BULKDEC0C BitStream Total Bytes Register */
#define AST_RFX_BULK_HIS_RING_START	0x114		/* BULKDEC Histiry Ring-Buffer Start Address Register */
#define AST_RFX_BULK_HIS_RING_END		0x118		/* BULKDEC Histiry Ring-Buffer End Address Register */
#define AST_RFX_BULK_ENG_RDY			0x11C		/* BULKDEC Engine Ready Register */
#define AST_RFX_BULK_HIS_BUFFRP_REST	0x120		/* BULKDEC Histiry Buffer Read Point Reset Register */
#define AST_RFX_BULK_ISR_CLR			0x124		/* BULKDEC Interrupt Clear Register */
#define AST_RFX_BULK_IER_ISR			0x128		/* BULKDEC Interrupt Status/Mask Register */
#define AST_RFX_BULK_ENG_STS			0x12C		/* BULKDEC Engine Status Register */
#define AST_RFX_BULK_DEST_END			0x130		/* BULKDEC Destination End Point Register */
#define AST_RFX_BULK_RING_READP		0x134		/* BULKDEC BitStream Ring-Buffer Read Point Register */
#define AST_RFX_BULK_STS				0x138		/* BULKDEC Status Register */
#define AST_RFX_BULK_UNCOMP_SIZE		0x13C		/* BULKDEC Uncompress Size Register */

/*CMD queue engine*/
#define AST_RFX_CMDQ_FIFO_STS			0x180		/* FIFO Status and CmdQ Version Register */
#define AST_RFX_CMDQ_ENABLE			0x184		/* Enable CmdQ Register */
#define AST_RFX_CMDQ_BUFF_START		0x188		/* CmdQ Command Buffer Start Address Register */
#define AST_RFX_CMDQ_BUFF_END		0x18C		/* CmdQ Command Buffer End Address Register */
#define AST_RFX_CMDQ_BUFF_WRITEP		0x190		/* CmdQ Command Buffer Write Point Register */
#define AST_RFX_CMDQ_BUFF_READP		0x194		/* CmdQ Command Buffer Read Point Register */


/* BitBlt engine */
#define AST_BITBLT_Y_TEARFREE			0x204		/* Y Coordinate of TearFree Register */
#define AST_BITBLT_PIXEL_BYTE_CMD		0x208 		/* Pixel Byte Mode Command */
#define AST_BITBLT_PIXEL_CONST_VALUE	0x20C 		/* Pixel Constant Value */
#define AST_BITBLT_HOR_INIT_SCALING	0x210 		/* Horizontal Initial Scaling Factor */
#define AST_BITBLT_VERT_INIT_SCALING	0x214 		/* Vertical Initial Scaling Factor */
#define AST_BITBLT_HOR_SCALING		0x218 		/* Horizontal Scaling Factor */
#define AST_BITBLT_VERT_SCALING		0x21C 		/* Vertical Scaling Factor */
#define AST_BITBLT_TILEMASKC_BUFF		0x220 		/* Base Address of Tile Mask-C Buffer Register */
#define AST_BITBLT_PIXELMASKC_BUFF	0x224 		/* Base Address of Pixel Mask-C Buffer Register */
#define AST_BITBLT_PIXELMASKC_RPITCH	0x228 		/* Row Pitch of Pixel Mask-C Buffer Register */
#define AST_BITBLT_CRTFLIP_RPITCH		0x22C 		/* Row Pitch of CRT Flip Display Buffer Register */
#define AST_BITBLT_TILEMASKB_BUFF		0x230 		/* Base Address of Tile Mask-B Buffer Register */
#define AST_BITBLT_PIXELMASKB_BUFF	0x234 		/* Base Address of Pixel Mask-B Buffer Register */
#define AST_BITBLT_PIXELMASKB_RPITCH	0x238 		/* Row Pitch of Pixel Mask-B Buffer Register */
#define AST_BITBLT_CRT_FLIP_INDEX		0x23C 		/* CRT Flipping Index Register */
#define AST_BITBLT_TILEMASKA_BUFF		0x240 		/* Base Address of Tile Mask-A Buffer Register */
#define AST_BITBLT_PIXELMASKA_BUFF	0x244 		/* Base Address of Pixel Mask-A Buffer Register */
#define AST_BITBLT_PIXELMASKA_RPITCH	0x248 		/* Row Pitch of Pixel Mask-A Buffer Register */
#define AST_BITBLT_SW_TAG				0x24C 		/* Software Tag Register */
#define AST_BITBLT_SOURCE_BUFF		0x250 		/* Base Address of Source Buffer Register */
#define AST_BITBLT_RPITCH_BUFF			0x254 		/* Row Pitch of Source Buffer Register */
#define AST_BITBLT_DEST_BUFF			0x258 		/* Base Address of Destination Buffer Register */
#define AST_BITBLT_RPITCH_DEST_BUFF	0x25C 		/* Row Pitch of Destination Buffer Register */
#define AST_BITBLT_SOURCE_BLOCK		0x260 		/* Coordinate of Source Block Register */
#define AST_BITBLT_DESTINATE_BLOCK	0x264 		/* Coordinate of Destination Block Register */
#define AST_BITBLT_BITMAP_XY			0x268 		/* Bibmap Block Width and Height Register */
#define AST_BITBLT_CMD					0x26C 		/* BitBlt Engine Command Register */
#define AST_BITBLT_CS_VALUE			0x270 		/* Checksum Compare Value Register */

/* Microsoft RemoteFX decoder engine, it support color depth 24/16/15/8 bpp in Interleaved RLE mode */
#define AST_RFX_RLEDEC_FIFO_STS		0x300		/* FIFO Status and RLEDEC Version Register */

/*EGFX engine*/
#define AST_RFX_EGFX_ENG_BASE			0x400		

#define AST_RFX_IER						0x700
#define AST_RFX_ISR						0x704


/* AST_RFX_FIFO_STS :	0x000	-	FIFO Status And RFXDEC Version Register */
#define RFX_FIFO_STS_CLR						(0x1 << 31)
#define RFX_FIFO_IDWT_ACK_EN					(0x1 << 30)
/* 29 ~ 24 */
#define RFX_BITPOST2DRAM_FIFO_OVFLOW			(0x1 << 23)
#define RFX_BITPOST2DRAM_FIFO_UNFLOW		(0x1 << 22)
#define RFX_DRAM2BITPOST_FIFO_OVFLOW			(0x1 << 21)
#define RFX_DRAM2BITPOST_FIFO_UNFLOW		(0x1 << 20)
#define RFX_SIGNBIT2DRAM_FIFO_OVFLOW			(0x1 << 19)
#define RFX_SIGNBIT2DRAM_FIFO_UNFLOW			(0x1 << 18)
#define RFX_DRAM2SIGNBIT_FIFO_OVFLOW			(0x1 << 17)
#define RFX_DRAM2SIGNBIT_FIFO_UNFLOW			(0x1 << 16)
#define RFX_TILEC2DRAM_FIFO_OVFLOW			(0x1 << 15)
#define RFX_TILEC2DRAM_FIFO_UNFLOW			(0x1 << 14)
#define RFX_DRAM2TILEC_FIFO_OVFLOW			(0x1 << 13)
#define RFX_DRAM2TILEC_FIFO_UNFLOW			(0x1 << 12)
#define RFX_EPDEC2IDWT_FIFO_OVFLOW			(0x1 << 11)
#define RFX_EPDEC2IDWT_FIFO_UNFLOW			(0x1 << 10)
#define RFX_IDWT2EPDEC_FIFO_OVFLOW			(0x1 << 9)
#define RFX_IDWT2EPDEC_FIFO_UNFLOW			(0x1 << 8)
/* 7 ~ 5 */
#define RFX_GET_EPDEC_PARSER_STATE(x)			(x & 0x1f)
#define RFX_EPDEC_PARSER_IDEL					0



/* AST_RFX_BULK_ISR		0x124	*/
#define RFX_BULK_ENGINE_INT_STAT_MASK	0xFFFF0000
#define RFX_BULK_ENGINE_INT_STAT_SHIFT	16

#define RFX_BULK_ENGINE_INT_DONE			(0x1)

/* AST_RFX_CMDQ_CTRL		0x184 */
#define RFX_CMDQ_RESUME		(0x1 << 1)
#define RFX_CMDQ_EN			(0x1)




/* AST_RFX_IER				0x700 */
#define RFX_INT_GMASK_DONE 			(0x1 << 26)
#define RFX_INT_VMASK_DONE			(0x1 << 23)
#define RFX_INT_EGFX_DEC_HANG			(0x1 << 22)
#define RFX_INT_EGFX_DEC_EXCPT		(0x1 << 21)
#define RFX_INT_EGFX_DEC_DONE			(0x1 << 20)
#define RFX_INT_EGFX_BULK_ERR			(0x1 << 16)
#define RFX_INT_EGFX_BULK_DONE		(0x1 << 15)
#define RFX_INT_CMDQ_SW_CMD			(0x1 << 8)
#define RFX_INT_CMDQ_ENG_DONE		(0x1 << 7)
#define RFX_INT_CMDQ_CMD_DONE 		(0x1 << 4)
#define RFX_INT_BITBLIT_ERR				(0x1 << 3)
#define RFX_INT_BITBLIT_DONE			(0x1 << 1)

#endif
