/********************************************************************************
* File Name     : ast_bitblt.c
* Author         : Ryan Chen
* Description   : AST BITBLT Controller
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
*      1. 2015/04/30 Ryan Chen create this file 
*    
********************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/sched.h>  
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>

#include <mach/ast_lcd.h>

#include <plat/ast_bitblt.h>

/***********************************************************************/
#define AST_BITBLIT_HOR_SCAL_FACT				0x18
#define AST_BITBLIT_VERT_SCAL_FACT				0x1C
#define AST_BITBLIT_TILE_MASKC_BASE				0x20
#define AST_BITBLIT_PIXEL_MASKC_BASE			0x24
#define AST_BITBLIT_PIXEL_MASKC_PITCH			0x28
#define AST_BITBLIT_TILE_MASKB_BASE				0x30
#define AST_BITBLIT_PIXEL_MASKB_BASE			0x34
#define AST_BITBLIT_MASK_FEATURES				0x38
	#define BITBLIT_MASK_FEATURES_PIXEL_MASKB_PITCH_SHIFT		0
	#define BITBLIT_MASK_FEATURES_POSITION_SEL_SHIFT 			16
	#define BITBLIT_MASK_FEATURES_L2_OP_SEL_SHIFT				18
	#define BITBLIT_MASK_FEATURES_L1_OP_SEL_SHIFT				20
		#define BITBLIT_MASK_OP_AND				0
		#define BITBLIT_MASK_OP_OR				1
	#define BITBLIT_MASK_FEATURES_OUTPUT_CTRL_SHIFT			22
		#define BITBLIT_MASK_OUTPUT_BYPASS		0
		#define BITBLIT_MASK_OUTPUT_L2			1
		#define BITBLIT_MASK_OUTPUT_L1			2
		#define BITBLIT_MASK_OUTPUT_L1L2			3
	#define BITBLIT_COMMAND_TILE_MASKB_ENABLE_SHIFT 	24
	#define BITBLIT_COMMAND_MASKB_POL_SEL_SHIFT 		25
	#define BITBLIT_MASK_FEATURES_MASKB_COORD_SEL_SHIFT 		26
	#define BITBLIT_COMMAND_TILE_MASKC_ENABLE_SHIFT 	28
	#define BITBLIT_COMMAND_MASKC_POL_SEL_SHIFT 		29
	#define BITBLIT_MASK_FEATURES_MASKC_COORD_SEL_SHIFT 		30
#define AST_BITBLIT_CRT_FLIP_IDX				0x3C

#define BITBLIT_SET_CRT1_FLIP_IDX(x)			(x)
#define BITBLIT_CRT1_FLIP_ENABLE				(0x1 << 2)
#define BITBLIT_SET_CRT2_FLIP_IDX(x)			(x << 3)
#define BITBLIT_CRT2_FLIP_ENABLE				(0x1 << 5)
#define BITBLIT_SET_CRT3_FLIP_IDX(x)			(x << 6)
#define BITBLIT_CRT3_FLIP_ENABLE				(0x1 << 8)
#define BITBLIT_SET_CRT4_FLIP_IDX(x)			(x << 3)
#define BITBLIT_CRT4_FLIP_ENABLE				(0x1 << 11)

#define AST_BITBLIT_TILE_MASKA_BASE				0x0040
#define AST_BITBLIT_PIXEL_MASKA_BASE			0x0044
#define AST_BITBLIT_PIXEL_MASKA_PITCH			0x0048
#define AST_BITBLIT_SW_TAG						0x004C
#define AST_BITBLIT_SRC_ADDR					0x50
#define AST_BITBLIT_SRC_PITCH					0x54
#define AST_BITBLIT_DEST_ADDR					0x58
#define AST_BITBLIT_DEST_PITCH					0x5C
#define AST_BITBLIT_SRC_OFFSET					0x60

#define SET_BITBLT_SRC_X_OFFSET(x)				(x << 16)
#define SET_BITBLT_SRC_Y_OFFSET(x)				(x)

#define AST_BITBLIT_DEST_OFFSET					0x64

#define SET_BITBLT_DEST_X_OFFSET(x)				(x << 16)
#define SET_BITBLT_DEST_Y_OFFSET(x)				(x)

#define AST_BITBLIT_BITMAP_DIMENSION			0x68

#define SET_BITBLIT_BITMAP_DIMENSION_X(x) 		(x << 16)
#define SET_BITBLIT_BITMAP_DIMENSION_Y(x) 		(x)

#define AST_BITBLIT_CMD							0x6C

#define BITBLIT_PIXEL_BYTE_MODE				(1 << 15)

#define BITBLIT_CMD_PIXEL_OP_MASK				0xf
#define BITBLIT_CMD_DEST_BPP(x)					(x)
#define BITBLIT_CMD_SRC_BPP(x)					(x << 2)
#define BITBLIT_8BIT_BPP									(0)
#define BITBLIT_16BIT_BPP									(1)
#define BITBLIT_24BIT_BPP									(2)
#define BITBLIT_32BIT_BPP									(3)

#define BITBLIT_CMD_SRC_COLOR_SPACE_RGB		(0 << 4)
#define BITBLIT_CMD_SRC_COLOR_SPACE_YUV		(1 << 4)


//TODO .....
	#define BITBLIT_COMMAND_MASKA_CTRL_SHIFT			8
	#define BITBLIT_COMMAND_MASKB_CTRL_SHIFT			9
	#define BITBLIT_COMMAND_MASKC_CTRL_SHIFT			10
	#define BITBLIT_COMMAND_MASK_OUTPUT_POLARITY_MASK	0x00000800
		#define BITBLIT_COMMAND_MASK_OUTPUT_POLARITY_INVERSE	0x00000800
	#define BITBLIT_COMMAND_TILE_MASKA_ENABLE_SHIFT		12
	#define BITBLIT_COMMAND_MASKA_POL_SEL_SHIFT 		13
	#define BITBLIT_COMMAND_INT_IDLE_SHIFT 				17

#define BITBLIT_CMD_INT_IDLE 						(0x1 << 17)
#define BITBLIT_CMD_SCALING_ENABLE				(0x1 << 20)
#define BITBLIT_CMD_SCALING_NO_Y					(0x1 << 21)
#define BITBLIT_CMD_Y_DIRECTION_DEC 				(0x1 << 26)		
#define BITBLIT_CMD_X_DIRECTION_DEC 				(0x1 << 27)
#define BITBLIT_CMD_CRT_FLIP						(0x1 << 28)
#define BITBLIT_CMD_WAIT_CRT_FLIP					(0x1 << 29)
#define BITBLIT_CMD_H264_LATCH						(0x1 << 30)
#define BITBLIT_CMD_BYPASS							(0x1 << 31)

#define AST_BITBLIT_STS						0x78

#define BITBLIT_ENG_BUSY						(0x1 << 31)

/***********************************************************************/
#define SCALE_NUMBER_FRAC              	0x8000
#define SCALE_MASK_ALL                 	0xfffff

/***********************************************************************/
//#define CONFIG_BITBLT_WITH_CMDQ

//#define CONFIG_AST_BITBLT_DEBUG

#ifdef CONFIG_AST_BITBLT_DEBUG
	#define BITBLT_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define BITBLT_DBG(fmt, args...)
#endif

/***********************************************************************/
void __iomem	*reg_base;			/* virtual */

static inline void
ast_bitblt_write(u32 val, u32 reg)
{
//	BITBLT_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, reg_base + reg);
}

static inline u32
ast_bitblt_read(u32 reg)
{
	u32 val = readl(reg_base + reg);
//	BITBLT_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}
/***************************************************************************/
#ifdef CONFIG_BITBLT_WITH_CMDQ
void ast_bitblt_crt_flip(u32 flip_idx, u8 crt)
{
	struct cmdq_ctrl_queue bitblt_queue;
	u32 qdata[5];
	bitblt_queue->data = qdata;
	//header 
	qdata[0] = CMDQ_HEADER_DEVICE_BB;
	//addr
	qdata[1] = base + AST_BITBLIT_CRT_FLIP_IDX;
	
	//data
	switch(crt) {
		case 0:
			qdata[2] = BITBLIT_SET_CRT1_FLIP_IDX(flip_idx) | BITBLIT_CRT1_FLIP_ENABLE;
			break;
		case 1:
			qdata[2] = BITBLIT_SET_CRT2_FLIP_IDX(flip_idx) | BITBLIT_CRT2_FLIP_ENABLE;
			break;
		case 2:
			qdata[2] = BITBLIT_SET_CRT3_FLIP_IDX(flip_idx) | BITBLIT_CRT3_FLIP_ENABLE;
			break;
		case 3:
			qdata[2] = BITBLIT_SET_CRT4_FLIP_IDX(flip_idx) | BITBLIT_CRT4_FLIP_ENABLE;
			break;

	}
	
	//fire
	//addr
	qdata[3] = base + AST_BITBLIT_CMD;
	//data
	qdata[4] = BITBLIT_CMD_CRT_FLIP | BITBLIT_CMD_NULL;

	ast_cmdq_enqueue(bitblt_queue);

}
#else
void ast_bitblt_crt_flip(u32 flip_idx, u8 crt)
{
	if(ast_bitblt_read(AST_BITBLIT_STS) & BITBLIT_ENG_BUSY) {
		printk("eng busy \n");
		return;
	}

	switch(crt) {
		case 0:
			ast_bitblt_write(BITBLIT_SET_CRT1_FLIP_IDX(flip_idx) | BITBLIT_CRT1_FLIP_ENABLE, AST_BITBLIT_CRT_FLIP_IDX);
			break;
		case 1:
			ast_bitblt_write(BITBLIT_SET_CRT2_FLIP_IDX(flip_idx) | BITBLIT_CRT2_FLIP_ENABLE, AST_BITBLIT_CRT_FLIP_IDX);		
			break;
		case 2:
			ast_bitblt_write(BITBLIT_SET_CRT3_FLIP_IDX(flip_idx) | BITBLIT_CRT3_FLIP_ENABLE, AST_BITBLIT_CRT_FLIP_IDX);		
			break;
		case 3:
			ast_bitblt_write(BITBLIT_SET_CRT4_FLIP_IDX(flip_idx) | BITBLIT_CRT4_FLIP_ENABLE, AST_BITBLIT_CRT_FLIP_IDX);		
			break;

	}
	//fire
	ast_bitblt_write(BITBLIT_CMD_CRT_FLIP | BITBLIT_CMD_BYPASS, AST_BITBLIT_CMD);
	
}

void ast_bitblt_rgb16to16(struct ast_bitblt_basic *bitblt)
{
	u32 cmd = 0;
	BITBLT_DBG("\n");
	
again:	
	if(ast_bitblt_read(AST_BITBLIT_STS) & BITBLIT_ENG_BUSY) {
		printk("bitblt eng busy ~~~~~~~~~~~~~~~~`!!\n");
		goto again;
	}
	
	//source	
	ast_bitblt_write(bitblt->src_base, AST_BITBLIT_SRC_ADDR);
	ast_bitblt_write(bitblt->src_pitch, AST_BITBLIT_SRC_PITCH);
	ast_bitblt_write(SET_BITBLT_SRC_X_OFFSET(bitblt->src_x) | bitblt->src_y , AST_BITBLIT_SRC_OFFSET);

	//destination
	ast_bitblt_write(bitblt->dest_base, AST_BITBLIT_DEST_ADDR);
	ast_bitblt_write(bitblt->dest_pitch, AST_BITBLIT_DEST_PITCH);
	ast_bitblt_write(SET_BITBLIT_BITMAP_DIMENSION_X(bitblt->dest_x) | bitblt->dest_y, AST_BITBLIT_DEST_OFFSET);		
	
	//dimension
	ast_bitblt_write(SET_BITBLIT_BITMAP_DIMENSION_X(bitblt->dest_w) | bitblt->dest_h, AST_BITBLIT_BITMAP_DIMENSION);	

	cmd |= (0x5/*0b0101*/) | BITBLIT_CMD_SRC_COLOR_SPACE_RGB;
	ast_bitblt_write(cmd, AST_BITBLIT_CMD);	//8bpp 
}

void ast_bitblt_rgbsrc16todst32(struct ast_bitblt_basic *bitblt)
{
	u32 cmd = 0;
	BITBLT_DBG("\n");
	
again:	
	if(ast_bitblt_read(AST_BITBLIT_STS) & BITBLIT_ENG_BUSY) {
		printk("bitblt eng busy ~~~~~~~~~~~~~~~~`!!\n");
		goto again;
	}
	
	//source	
	ast_bitblt_write(bitblt->src_base, AST_BITBLIT_SRC_ADDR);
	ast_bitblt_write(bitblt->src_pitch, AST_BITBLIT_SRC_PITCH);
	ast_bitblt_write(SET_BITBLT_SRC_X_OFFSET(bitblt->src_x) | bitblt->src_y , AST_BITBLIT_SRC_OFFSET);

	//destination
	ast_bitblt_write(bitblt->dest_base, AST_BITBLIT_DEST_ADDR);
	ast_bitblt_write(bitblt->dest_pitch, AST_BITBLIT_DEST_PITCH);
	ast_bitblt_write(SET_BITBLIT_BITMAP_DIMENSION_X(bitblt->dest_x) | bitblt->dest_y, AST_BITBLIT_DEST_OFFSET);		
	
	//dimension
	ast_bitblt_write(SET_BITBLIT_BITMAP_DIMENSION_X(bitblt->dest_w) | bitblt->dest_h, AST_BITBLIT_BITMAP_DIMENSION);	

	cmd |= (0x7/*0b0110*/) | BITBLIT_CMD_SRC_COLOR_SPACE_RGB;
	ast_bitblt_write(cmd, AST_BITBLIT_CMD);	//8bpp 
}

void ast_bitblt_rgbsrctodst16(struct ast_bitblt_basic *bitblt)
{
	u32 cmd = 0;
	BITBLT_DBG("\n");
	
again:	
	if(ast_bitblt_read(AST_BITBLIT_STS) & BITBLIT_ENG_BUSY) {
		printk("bitblt eng busy ~~~~~~~~~~~~~~~~`!!\n");
		goto again;
	}
	
	//source	
	ast_bitblt_write(bitblt->src_base, AST_BITBLIT_SRC_ADDR);
	ast_bitblt_write(bitblt->src_pitch, AST_BITBLIT_SRC_PITCH);
	ast_bitblt_write(SET_BITBLT_SRC_X_OFFSET(bitblt->src_x) | bitblt->src_y , AST_BITBLIT_SRC_OFFSET);

	//destination
	ast_bitblt_write(bitblt->dest_base, AST_BITBLIT_DEST_ADDR);
	ast_bitblt_write(bitblt->dest_pitch, AST_BITBLIT_DEST_PITCH);
	ast_bitblt_write(SET_BITBLT_DEST_X_OFFSET(bitblt->dest_x) | bitblt->dest_y , AST_BITBLIT_DEST_OFFSET);	
	
	//dimension
	ast_bitblt_write(SET_BITBLIT_BITMAP_DIMENSION_X(bitblt->dest_w) | bitblt->dest_h, AST_BITBLIT_BITMAP_DIMENSION);	

	cmd |= (0x5/*0b0101*/) | BITBLIT_CMD_SRC_COLOR_SPACE_RGB;
	ast_bitblt_write(cmd, AST_BITBLIT_CMD);	//8bpp 
}

void ast_bitblt_memcpy(struct ast_bitblt_basic *bitblt)
{
	BITBLT_DBG("\n");
again:	
	if(ast_bitblt_read(AST_BITBLIT_STS) & BITBLIT_ENG_BUSY) {
		printk("bitblt eng busy ~~~~~~~~~~~~~~~~`!!\n");
		goto again;
	}
	
	//source	
	ast_bitblt_write(bitblt->src_base, AST_BITBLIT_SRC_ADDR);
	ast_bitblt_write(bitblt->src_pitch, AST_BITBLIT_SRC_PITCH);

	ast_bitblt_write(0, AST_BITBLIT_SRC_OFFSET);

	//destination
	ast_bitblt_write(bitblt->dest_base, AST_BITBLIT_DEST_ADDR);
	ast_bitblt_write(bitblt->dest_pitch, AST_BITBLIT_DEST_PITCH);
	
	ast_bitblt_write(0 , AST_BITBLIT_DEST_OFFSET);	
	
	ast_bitblt_write(BITBLIT_PIXEL_BYTE_MODE, AST_BITBLIT_CMD);	//8bpp 
}

void ast_h264_bitblt(struct ast_bitblt_basic *bitblt)
{
	u8 bitblt_latch_formatter = 0;
	u32 cmd = 0;
	u8 scaling = 0;
	u32 ScalePara_X, ScalePara_Y;
	
	BITBLT_DBG("\n");
again:	
	if(ast_bitblt_read(AST_BITBLIT_STS) & BITBLIT_ENG_BUSY) {
		printk("bitblt eng busy ~~~~~~~~~~~~~~~~`!!\n");
		goto again;
	}
	
	//source	??? TODO ...	
	ast_bitblt_write(bitblt->src_base, AST_BITBLIT_SRC_ADDR);
	ast_bitblt_write(bitblt->src_pitch, AST_BITBLIT_SRC_PITCH);

	ast_bitblt_write(SET_BITBLT_SRC_X_OFFSET(bitblt->src_x) | bitblt->src_y , AST_BITBLIT_SRC_OFFSET);

	//destination
	ast_bitblt_write(bitblt->dest_base, AST_BITBLIT_DEST_ADDR);
	ast_bitblt_write(bitblt->dest_pitch, AST_BITBLIT_DEST_PITCH);
	ast_bitblt_write(SET_BITBLT_DEST_X_OFFSET(bitblt->dest_x) | bitblt->dest_y , AST_BITBLIT_DEST_OFFSET);	
	
	//dimension
	ast_bitblt_write(SET_BITBLIT_BITMAP_DIMENSION_X(bitblt->dest_w) | bitblt->dest_h, AST_BITBLIT_BITMAP_DIMENSION);	

	//scaling	
#if 0	
	if(bitblt->dest_w != bitblt->src_w) {
		ScalePara_X = ( ((bitblt->src_w-1) * SCALE_NUMBER_FRAC) / (bitblt->dest_w-1) ) & SCALE_MASK_ALL;
		scaling = 1;
	} else {
		ScalePara_X = SCALE_NUMBER_FRAC;
	}
	ast_bitblt_write(ScalePara_X, AST_BITBLIT_HOR_SCAL_FACT);

	
	if(bitblt->dest_h != bitblt->src_h) {
		ScalePara_Y = ( ((bitblt->src_h-1) * SCALE_NUMBER_FRAC) / (bitblt->dest_h-1) ) & SCALE_MASK_ALL;
		scaling = 1;
	} else {
		ScalePara_Y = SCALE_NUMBER_FRAC;
	}
	ast_bitblt_write(ScalePara_Y, AST_BITBLIT_VERT_SCAL_FACT);	

	////TODO .....
	if(scaling) {
		cmd |= BITBLIT_CMD_SCALING_ENABLE;
		if (ScalePara_Y == SCALE_NUMBER_FRAC)
			cmd |= BITBLIT_CMD_SCALING_NO_Y;
	}
	
#else
	ast_bitblt_write(SCALE_NUMBER_FRAC, AST_BITBLIT_HOR_SCAL_FACT);

	ast_bitblt_write(SCALE_NUMBER_FRAC, AST_BITBLIT_VERT_SCAL_FACT);	
#endif

	/* 
	Reg6ch[1:0] = BPP_dst;
	Reg6ch[3:2] = (Is_YUYV) ? 1 : BPP_src;
	Reg6ch[4]	= (Is_YUYV) ? 1 : 0;
	Reg6ch[5]	= (Enable_Dither) ? 1 : 0;
	Reg6ch[20]	= 1;
	Reg6ch[21]	= (H_src == H_dst) ? 1 : 0;
	*/

	BITBLT_DBG("bitblt->dest_format %d, bitblt->src_format %d \n", bitblt->dest_format, bitblt->src_format);

	//command	
	if (bitblt->dest_format == ASTFB_COLOR_RGB888) {	
		if (bitblt->src_format == ASTFB_COLOR_RGB888)
			cmd |= (0xA/*0b1010*/) | BITBLIT_CMD_SRC_COLOR_SPACE_RGB;
		else if (bitblt->src_format == ASTFB_COLOR_YUV422)
			cmd |= (0x7/*0b0110*/) | BITBLIT_CMD_SRC_COLOR_SPACE_YUV;
		
	} else if (bitblt->dest_format == ASTFB_COLOR_RGB565) {
		if (bitblt->src_format == ASTFB_COLOR_RGB565)
			cmd |= (0x5/*0b0101*/) | BITBLIT_CMD_SRC_COLOR_SPACE_RGB;
		else if (bitblt->src_format == ASTFB_COLOR_YUV422)
			cmd |= (0x5/*0b0110*/) | BITBLIT_CMD_SRC_COLOR_SPACE_YUV;
	}	

//	cmd |= BITBLIT_CMD_H264_LATCH;

	BITBLT_DBG("cmd %x , %d\n", cmd,  jiffies/HZ);
printk("bitblt %d \n", jiffies * 1000 /HZ);

	ast_bitblt_write(cmd, AST_BITBLIT_CMD);	
}

EXPORT_SYMBOL(ast_h264_bitblt);

#endif
/***************************************************************************/
#ifdef CONFIG_BITBLT_WITH_CMDQ

#else
static irqreturn_t ast_bitblt_err_irq(int this_irq, void *dev_id)
{
	printk("ast_bitblt_err_irq \n");
	return IRQ_HANDLED;
}


static irqreturn_t ast_bitblt_done_irq(int this_irq, void *dev_id)
{
	printk("ast_bitblt_done_irq \n");
	return IRQ_HANDLED;
}
#endif

static void ast_bitblt_init(void)
{
	int i = 0;
	for(i = 0; i < 30; i++)
		ast_bitblt_write(0, i*4);
}

static int ast_bitblt_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;
	int irq = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	reg_base = ioremap(res->start, resource_size(res));
	if (!reg_base) {
		ret = -EIO;
		goto out_region;
	}

#ifdef CONFIG_BITBLT_WITH_CMDQ
#else
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(irq, ast_bitblt_done_irq, IRQF_DISABLED,
			  "ast-bitblt-done", NULL);

	if (ret) {
		printk(KERN_INFO "Bitblt: Failed request irq %d\n", irq);
		goto out_region;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(irq, ast_bitblt_err_irq, IRQF_DISABLED,
			  "ast-bitblt-error", NULL);

	if (ret) {
		printk(KERN_INFO "Bitblt: Failed request irq %d\n", irq);
		goto out_region;
	}
	
#endif

//	platform_set_drvdata(pdev, ast_bitblt);

	ast_bitblt_init();
	
	printk(KERN_INFO "ast_bitblt: driver successfully loaded.\n");

	return 0;

out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "driver init failed (ret=%d)!\n", ret);
	return ret;
}

static struct platform_driver ast_bitblt_driver = {
	.driver         = {
		.name   = "ast-bitblt",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_bitblt_driver, ast_bitblt_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Bitblt driver");
MODULE_LICENSE("GPL");
