/********************************************************************************
* File Name     : ast_egfx.c
* Author         : Ryan Chen
* Description   : AST egfx Controller
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
*      1. 2013/01/30 Ryan Chen create this file 
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

/***********************************************************************/
#define AST_EGFX_CTRL				0x00
#define AST_EGFX_SADDR				0x04
#define AST_EGFX_SA_QUANTITY		0x08
#define AST_EGFX_SB_QUANTITY		0x0C
#define AST_EGFX_SC_QUANTITY		0x10
#define AST_EGFX_SD_QUANTITY		0x14
#define AST_EGFX_BITMAP_DEC_DEST	0x18
#define AST_EGFX_BITMAP_DEC_DEST_PITCH	0x1C
#define AST_EGFX_BITMAP_STARTP		0x20
#define AST_EGFX_BITMAP_ENDP		0x24
#define AST_EGFX_BITMAP_DIMENSION		0x28
#define AST_EGFX_BITMAP_DEST		0x2C
#define AST_EGFX_VBAR				0x30
#define AST_EGFX_SHORT_VBAR		0x34

#define EGFX_DEC_CFG0			0x0038
#define EGFX_DEC_CFG1			0x003C
#define EGFX_DEC_CFG2			0x0040
#define EGFX_DEC_CFG3			0x0044
#define EGFX_DEC_CFG4			0x0048
#define EGFX_DEC_CFG5			0x004C
#define EGFX_DEC_CFG6			0x0050
#define EGFX_DEC_CFG7			0x0054
#define EGFX_DEC_CFG8			0x0058
#define EGFX_DEC_CFG9			0x005C

#define AST_EGFX_WDT				0x70
#define EGFX_WDT_KEEP_BUSY		(0x1 << 8)
#define EGFX_WDT_TIMELIMIT(x)		(x)

#define EGFX_DEC_PLANAR_CFG0	0x0074
	#define EGFX_DEC_PLANAR_CH_NO_SWAP	0x00000001
#define EGFX_DEC_RAW_CFG0		0x0078
	#define EGFX_DEC_RAW_CH_CNT_MASK	0x00000002
	#define EGFX_DEC_RAW_CH_CNT_4		0x00000002
	#define EGFX_DEC_RAW_CH_CNT_3		0x00000000

#define AST_EGFX_BITMAP_COLOR_MODE		0x7C

#define EGFX_TRUE_COLOR					0x0
#define EGFX_HIGH_COLOR					0x1


#define AST_EGFX_STRAM_START		0x80
#define AST_EGFX_STRAM_END		0x84	

#define EGFX_DEC_CFG1A			0x0088
#define EGFX_DEC_CFG1B			0x008C
#define EGFX_DEC_CFG1C			0x0090
#define EGFX_DEC_CFG1D			0x0094
	#define EGFX_DIRECT_TILE_OUTPUT_SHIFT	0
	#define EGFX_COMB_MASK_GEN_SHIFT	1
	#define EGFX_TILE_MASK_GEN_SHIFT	2
	#define EGFX_REG_MASK_GEN_SHIFT		3
	#define EGFX_COMB_MASK_CLR_SHIFT	4
	#define EGFX_TILE_MASK_CLR_SHIFT	5
	#define EGFX_REG_MASK_CLR_SHIFT 	6
	#define CAP_OUTPUT_READ_BYPASS_SHIFT 	7
	#define CA_FAST_OUTPUT_SHIFT	8
	
#define AST_EGFX_TRIG				0x0100

	#define EGFX_TRIG_ENABLE			0x00000001
	
#define EGFX_DEC_STATUS0		0x0104
	#define EGFX_SOLID_FILL_BUSY	0x00000001
	#define EGFX_UNCOMPRESSED_BUSY	0x00000002
	#define EGFX_PLANAR_BUSY		0x00000004
	#define EGFX_NSCODEC_BUSY		0x00000008
	#define EGFX_RESIDUAL_BUSY		0x00000010
	#define EGFX_BAND_BUSY			0x00000020
	#define EGFX_RLEX_BUSY			0x00000040
	#define EGFX_CAVIDEO_BUSY		0x00000080
	#define EGFX_CAPROGRESSIVE_BUSY 0x00000100
	#define EGFX_VMASK_BUSY 		0x00000200
#define EGFX_DEC_STATUS1		0x0108

/***********************************************************************/
/*		AST_EGFX_DEC_CTRL			0x00		*/
#define EGFX_DEC_VBAR_RESET 						(0x1 << 31)
#define EGFX_CODEC_ID_SOLID_FILL					0x00
#define EGFX_CODEC_ID_UNCOMPRESSED				0x01
#define EGFX_CODEC_ID_PLANAR						0x02
#define EGFX_CODEC_ID_NSCODEC						0x03
#define EGFX_CODEC_ID_RESIDUAL					0x04
#define EGFX_CODEC_ID_BAND						0x05
#define EGFX_CODEC_ID_RLEX							0x06
#define EGFX_CODEC_ID_CAVIDEO						0x07
#define EGFX_CODEC_ID_CAPROGRESSIVE				0x08


/***********************************************************************/
#define AST3200_EGFX_GLYPH_LEN	3072//3 //3 * 1024
#define AST3200_EGFX_GLYPH_NUM	4000
#define AST3200_EGFX_GLYPH_SUM	(AST3200_EGFX_GLYPH_LEN * AST3200_EGFX_GLYPH_NUM)

#define MAX_NUM_BITMAP_CACHE_SLOTS	4096
#define MAX_NUM_DECOMP_GLYPH_SLOTS	4000

struct _EGFX_CACHE
{
	unsigned int valid;
	u64	cacheKey;
	u16	width;
	u16	height;
	u8	pixelFormat;
	u32 data_bus;
};
typedef struct _EGFX_CACHE EGFX_CACHE;

struct _Decompressor_Glyph
{
	u32 pixels;
	u32 data_bus;
};
typedef struct _Decompressor_Glyph Decompressor_Glyph;

/***********************************************************************/
static EGFX_CACHE Bitmap_Cache[MAX_NUM_BITMAP_CACHE_SLOTS];
static Decompressor_Glyph Decompressor_Glyph_Storage[AST3200_EGFX_GLYPH_NUM];
/***********************************************************************/


//#define CONFIG_AST_EGFX_DEBUG

#ifdef CONFIG_AST_EGFX_DEBUG
	#define EGFX_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define EGFX_DBG(fmt, args...)
#endif

/***********************************************************************/
static struct ast_egfx_data {
	void __iomem		*reg_base;			/* virtual */
	int 				irq;	
	phys_addr_t		*graphic_stream_phy;
	char				*graphic_stream_virt;		
	u32				graphic_stream_size;	
	phys_addr_t		*vbar_phy;
	char				*vbar_virt;		
	u32				vbar_size;		
	phys_addr_t		*short_vbar_phy;	
	char				*short_vbar_virt;	
	u32				short_vbar_size;			
	int 				high_color;
	bool 			is_open;	
	spinlock_t 		lock;		
};

static inline void
ast_egfx_write(struct ast_egfx_data *ast_egfx, u32 val, u32 reg)
{
//	EGFX_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_egfx->reg_base + reg);
}

static inline u32
ast_egfx_read(struct ast_egfx_data *ast_egfx, u32 reg)
{
	u32 val = readl(ast_egfx->reg_base + reg);
//	EGFX_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

static void ast_egfx_ctrl_init(struct ast_egfx_data *ast_egfx)
{
	int i;
	for (i = 0; i < MAX_NUM_BITMAP_CACHE_SLOTS; i++)
	{
		Bitmap_Cache[i].valid = 0;
	}
	for (i = 0; i < AST3200_EGFX_GLYPH_NUM; i++)
	{
		Decompressor_Glyph_Storage[i].pixels = 0;
	}
#if 0
	for (i = 0; i < MAX_NUM_BITMAP_CACHE_SLOTS; i++)
	{
		Bitmap_Cache[i].data_bus = AST3200_EGFX_BITMAP_CACHE_OFFSET + (AST3200_EGFX_BITMAP_CACHE_LEN * i);
	}
	for (i = 0; i < AST3200_EGFX_GLYPH_NUM; i++)
	{
		Decompressor_Glyph_Storage[i].data_bus = AST3200_EGFX_GLYPH_OFFSET + AST3200_EGFX_GLYPH_LEN * i;
	}
#endif

	ast_egfx_write(ast_egfx, EGFX_DEC_VBAR_RESET | EGFX_CODEC_ID_BAND, AST_EGFX_CTRL);
	ast_egfx_write(ast_egfx, EGFX_TRIG_ENABLE, AST_EGFX_TRIG);

	ast_egfx_write(ast_egfx, ast_egfx->vbar_phy, AST_EGFX_VBAR);
	ast_egfx_write(ast_egfx, ast_egfx->short_vbar_phy, AST_EGFX_SHORT_VBAR);	

	ast_egfx_write(ast_egfx, ast_egfx->graphic_stream_phy, AST_EGFX_STRAM_START);
	ast_egfx_write(ast_egfx, ast_egfx->graphic_stream_phy + ast_egfx->graphic_stream_size, AST_EGFX_STRAM_END);	

#if (AST3200_REV == 4)//meter
	//TODO ///.....
	SetGFXReg(AST3200_EGFX_ENGINE_BASE + 0xA4, 0x30000000);
#endif
	ast_egfx_write(ast_egfx, EGFX_WDT_KEEP_BUSY | EGFX_WDT_TIMELIMIT(128), AST_EGFX_WDT); 	//around 5 seconds on FPGA board
//	SetGFXReg(AST3200_EGFX_ENGINE_BASE + EGFX_DEC_CFG14, (1 << 8) | (1 & EGFX_DEC_WATCHDOG_LIMIT_TIME_MASK));//around 0.3 seconds on FPGA board

	if(ast_egfx->high_color)
		ast_egfx_write(ast_egfx, EGFX_HIGH_COLOR, AST_EGFX_BITMAP_COLOR_MODE);
	else
		ast_egfx_write(ast_egfx, EGFX_TRUE_COLOR, AST_EGFX_BITMAP_COLOR_MODE);

	

}


/***************************************************************************/
static irqreturn_t ast_egfx_handler(int this_irq, void *dev_id)
{
	struct ast_egfx_data *ast_egfx = dev_id;

//	u32 sts = ast_egfx_read(ast_egfx, AST_EGFX_ISR);
	u32 idx = 0;
	
	
	return IRQ_HANDLED;
}

//better for performance 
struct ast_egfx_data *ast_egfx;

struct miscdevice ast_egfx_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-egfx",
//	.fops = &ast_egfx_fops,
};

static int ast_egfx_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret=0;

	ast_egfx = kzalloc(sizeof(struct ast_egfx_data), GFP_KERNEL);
	if(ast_egfx == NULL) {
		printk("memalloc error");
		goto out;
	}
	
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

	ast_egfx->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_egfx->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_DMA\n");
		ret = -ENOENT;
		goto out;
	}

	if (!request_mem_region(res->start, resource_size(res), res->name)) {
		dev_err(&pdev->dev, "cannot reserved region\n");
		ret = -ENXIO;
		goto out;
	}

	ast_egfx->graphic_stream_phy = res->start;
	ast_egfx->graphic_stream_size = resource_size(res);

	printk("ast_egfx->stream:  %x \n", ast_egfx->graphic_stream_phy);

	ast_egfx->graphic_stream_virt = ioremap(res->start, resource_size(res));
	if (!ast_egfx->graphic_stream_virt) {
		ret = -EIO;
		goto out_region;
	}

	ast_egfx->irq = platform_get_irq(pdev, 0);
	if (ast_egfx->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(ast_egfx->irq, ast_egfx_handler, IRQF_SHARED,
			  "ast-egfx", ast_egfx);
	
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_egfx->irq);
		goto out_region;
	}

	ret = misc_register(&ast_egfx_misc);
	if (ret){		
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}
	spin_lock_init(&ast_egfx->lock);

	platform_set_drvdata(pdev, ast_egfx);

	ast_egfx_ctrl_init(ast_egfx);
	
	printk(KERN_INFO "ast_egfx: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_egfx->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_egfx_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_egfx_data *ast_egfx = platform_get_drvdata(pdev);

	free_irq(ast_egfx->irq, &ast_egfx);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_egfx->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;	
}

static struct platform_driver ast_egfx_driver = {
	.driver         = {
		.name   = "ast-egfx",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_egfx_driver, ast_egfx_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST egfx driver");
MODULE_LICENSE("GPL");
