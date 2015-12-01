/********************************************************************************
* File Name     : ast_cmdq.c
* Author         : Ryan Chen
* Description   : AST CMDQ Controller
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

#include <plat/ast_cmdq.h>

/***********************************************************************/
//#define AST_CMDQ_STS			0x00
#define AST_CMDQ_CTRL			0x04

#define CMDQ_CTRL_ENABLE		(0x1)
#define CMDQ_CTRL_STOP			(0x1 << 1)

#define AST_CMDQ_START			0x08
#define AST_CMDQ_END			0x0C
#define AST_CMDQ_WRITEP		0x10
#define AST_CMDQ_READP			0x14

/***********************************************************************/
#define CONFIG_AST_CMDQ_DEBUG

#ifdef CONFIG_AST_CMDQ_DEBUG
	#define CMDQ_DBG(fmt, args...) printk(KERN_DEBUG "%s(): " fmt, __FUNCTION__, ## args)
#else
	#define CMDQ_DBG(fmt, args...)
#endif

/***********************************************************************/
static struct ast_cmdq_data {
	void __iomem		*reg_base;			/* virtual */
	int 				irq;	
	phys_addr_t		*dma_addr;
	u32				*virt_addr;
	u32				cmdq_size;	
};

static inline void
ast_cmdq_write(struct ast_cmdq_data *ast_cmdq, u32 val, u32 reg)
{
//	CMDQ_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_cmdq->reg_base + reg);
}

static inline u32
ast_cmdq_read(struct ast_cmdq_data *ast_cmdq, u32 reg)
{
	u32 val = readl(ast_cmdq->reg_base + reg);
//	CMDQ_DBG("read offset: %x, val: %x \n",reg,val);
	return val;
}

/***************************************************************************/
#if 0
int ast_cmdq_enqueue(struct cmdq_ctrl_queue *en_queue)
{
	unsigned long flags;
	int i ;
	u32 rd_pt;
	u32 cmdq_cur_size;

	//check cmdq buffer size
	rd_pt = ast_cmdq_read(ast_cmdq, AST_CMDQ_READP);
#if 0	
	if(ast_cmdq->cmd_wp >= rd_pt) {
		cmdq_cur_size = (ast_cmdq->cmdq_phy + ast_cmdq->cmdq_size - ast_cmdq->cmd_wp) + (rd_pt - ast_cmdq->cmdq_phy);
	} else {
		cmdq_cur_size = rd_pt - ast_cmdq->cmd_wp;
	}
	if(cmdq_cur_size < en_queue->qsize) {
		printk("cmdq size not enough \n");
		return 1;
	}
#endif		
	for(i = 0; i < en_queue->qsize; i++) {
		ast_cmdq->cmdq_virt[ast_cmdq->cmd_idx + i] = en_queue->data[i];
		ast_cmdq->cmd_idx++;
		ast_cmdq->cmd_idx %= ast_cmdq->cmdq_size;
	}

	ast_cmdq->cmd_idx %= ast_cmdq->cmdq_size;
	
	ast_cmdq_write(ast_cmdq, ast_cmdq->cmdq_phy[ast_cmdq->cmd_idx], AST_CMDQ_WRITEP);
	


}
#endif

unsigned int write_cmd_queue(unsigned int pcmd, void* pdata, int size)
{
#if 0
	int left = 0; 
	int over = 0;
	
	if((pcmd + size) >= g_cmd_queue_end)	//end 
	{
		left = g_cmd_queue_end - pcmd;
		over = pcmd + size - g_cmd_queue_end;
		memcpy((void*)pcmd, pdata, left);
		memcpy((void*)g_cmd_queue_start, (char*)pdata+left, over);
		pcmd = g_cmd_queue_start + over;					//cycle
	}
	else
	{
		memcpy((void*)pcmd, pdata, size);
		pcmd += size;
	}
	
	return pcmd;
#endif	
}

void check_cmd_queue(unsigned int pcmd, unsigned int size)
{
	unsigned int gard = 0;
	unsigned int cnt = 0;
#if 0
	do
	{
		CMD_READ_P = ReadMemoryLong(0x1e6eb180, 0x14);
		g_cmd_read_p = CMD_ToLinear(CMD_READ_P);
		gard = GetDataOutputSize(pcmd, g_cmd_read_p, g_cmd_queue_size);
		
		cnt ++;
		if(cnt > 100000)
		{
			printk("k:cmdqueue gard=%x, pcmd=%08x, read_p=%08x\n", gard, CMD_ToPhys(pcmd), CMD_ToPhys(g_cmd_read_p));
			break;
		}
	}
	while(gard <= (size + CMD_GARD_BAND));	
#endif	
}

extern int GetCmdQueueLeft(struct ast_cmdq_data *ast_cmdq)
{
//    unsigned int curr_rp;
//	int          datas;
	
	curr_rp = ast_cmdq_read(ast_cmdq, AST_CMDQ_READP);
//	datas = GetDataOutputSize(CMD_ToLinear(curr_rp), g_cmd_write_p, g_cmd_queue_size);
///	if(datas == g_cmd_queue_size)
//	    datas = 0;
//	return datas;
}

inline void CMD_UpdateWritePort(struct ast_cmdq_data *ast_cmdq, u32 wp)
{
	//volatile unsigned int dummy;
	//unsigned int update;
	
	//update = MAX(g_cmd_write_p - 4, g_cmd_queue_start);
	//dummy = *(unsigned int*)update;

	ast_cmdq_write(ast_cmdq, wp, AST_CMDQ_WRITEP);

}

inline void CMD_SingleWrite(unsigned int base, unsigned int reg, unsigned int data)
{		

/*
	*(unsigned int*)g_cmd_write_p = base + reg;
	g_cmd_write_p += 4;
	if(g_cmd_write_p == g_cmd_queue_end)
	{
		g_cmd_write_p = g_cmd_queue_start;
	}
	*(unsigned int*)g_cmd_write_p = data;
	
	g_cmd_write_p += 4;
	if(g_cmd_write_p == g_cmd_queue_end)
	{
		g_cmd_write_p = g_cmd_queue_start;
	}
*/	
}

static irqreturn_t ast_cmdq_done_irq(int this_irq, void *dev_id)
{

	return IRQ_HANDLED;
}

static void ast_cmdq_init(struct ast_cmdq_data *ast_cmdq)
{
	ast_cmdq_write(ast_cmdq, 0, AST_CMDQ_CTRL);			//disable cmd_queue
	ast_cmdq_write(ast_cmdq, ast_cmdq->dma_addr, AST_CMDQ_START);
	ast_cmdq_write(ast_cmdq, ast_cmdq->dma_addr + ast_cmdq->cmdq_size, AST_CMDQ_END);	

	ast_cmdq_write(ast_cmdq, ast_cmdq->dma_addr, AST_CMDQ_WRITEP);
	ast_cmdq_write(ast_cmdq, ast_cmdq->dma_addr, AST_CMDQ_READP);	
	
	ast_cmdq_write(ast_cmdq, CMDQ_CTRL_ENABLE, AST_CMDQ_CTRL);
}


struct ast_cmdq_data *ast_cmdq;

static int ast_cmdq_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;
	int irq = 0;

	ast_cmdq = kzalloc(sizeof(struct ast_cmdq_data), GFP_KERNEL);
	if(ast_cmdq == NULL) {
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

	ast_cmdq->reg_base = ioremap(res->start, resource_size(res));
	if (!ast_cmdq->reg_base) {
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

	ast_cmdq->dma_addr = res->start;
	ast_cmdq->cmdq_size = resource_size(res);

	ast_cmdq->virt_addr = ioremap(res->start, resource_size(res));
	if (!ast_cmdq->virt_addr) {
		ret = -EIO;
		goto out_region;
	}
	
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = request_irq(irq, ast_cmdq_done_irq, IRQF_DISABLED,
			  "ast-cmdq-done", NULL);

	if (ret) {
		printk(KERN_INFO "Bitblt: Failed request irq %d\n", irq);
		goto out_region;
	}

//	platform_set_drvdata(pdev, ast_cmdq);

	ast_cmdq_init(ast_cmdq);
	
	printk(KERN_INFO "ast_cmdq: driver successfully loaded.\n");

	return 0;

out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "driver init failed (ret=%d)!\n", ret);
	return ret;
}

static struct platform_driver ast_cmdq_driver = {
	.driver         = {
		.name   = "ast-cmdq",
		.owner  = THIS_MODULE,
	},
};

module_platform_driver_probe(ast_cmdq_driver, ast_cmdq_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST CMDQ driver");
MODULE_LICENSE("GPL");
