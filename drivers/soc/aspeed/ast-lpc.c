/********************************************************************************
* File Name     : arch/arm/mach-aspeed/ast-lpc.c
* Author         : Ryan Chen
* Description   : AST LPC
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

*   History      :
*    1. 2013/05/15 Ryan Chen Create
*
*Check Host is in Continuous mode
*Check under Host : SIRQ_CNTL
   (Serial IRQ Mode Select (SIRQMD) �X R/W.
   0 =  The serial IRQ machine will be in quiet mode.
   1 =  The serial IRQ machine will be in continuous mode.)
*Checked under BMC: SIRQCR0 0x70[7] = 0/1 (Quiet/Continuous mode)

*In ICH6, GEN1_DEC = C81h / GEN2_DEC = 81h
In ICH7 or newer, choose any 2 of GENx_DEC, set 00040081h & 007C0C81h

********************************************************************************/
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <linux/dma-mapping.h>

#include <plat/regs-lpc.h>
#include <plat/ast-lpc.h>
#ifdef CONFIG_ARCH_AST1070
#include <plat/ast-uart-dma.h>
#include <plat/ast1070-scu.h>
#include <plat/ast1070-devs.h>
#include <plat/regs-ast1070-intc.h>
#endif
#include <linux/miscdevice.h>
#include <linux/hwmon-sysfs.h>

//#define AST_LPC_DEBUG

#ifdef AST_LPC_DEBUG
#define LPC_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define LPC_DBUG(fmt, args...)
#endif

static inline u32
ast_lpc_read(struct ast_lpc_data *ast_lpc, u32 reg)
{
#if 0
	u32 val;
	val = readl(ast_lpc->reg_base + reg);
	LPC_DBUG("ast_lpc_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_lpc->reg_base + reg);
#endif
}

static inline void
ast_lpc_write(struct ast_lpc_data *ast_lpc, u32 val, u32 reg)
{
//	LPC_DBUG("ast_lpc_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_lpc->reg_base + reg);
}

extern u32 StrToHex(char *p)
{
	int i, sum;
	int temp, length;
	char c;
	sum = 0;
	length = strlen(p);
	for( i = 0; i < (length - 1) ; i++ )
	{
		c = *p;
		if( c >= 'a' && c <= 'z') {
			temp = c - 87;
			sum += ((temp) << (4*(length - i - 2)));
		} else if( c >= 'A' && c <= 'Z') {
			temp = c - 55;
			sum += ((temp) << (4*(length - i - 2)));
		} else {
			temp = c - 48;
			sum = sum + ((temp) << (4*(length - i - 2)));
		}
		p = p + 1;
	}
	return sum;
}
EXPORT_SYMBOL(StrToHex);

static struct ast_lpc_data *ast_lpc;

/**************************   IPMI KCS Function  **********************************************************/
//Set default KCS address KCS0 : CA0/CA4, KCS1 : CA8/CAC, KCS3 : CA2/CA3, KCS4 : CB2/CB3
struct ast_kcs_data *register_ipmi_kcs_drv(u8 kcs_no)
{
	switch(kcs_no) {
		case 0:
			ast_lpc->ast_kcs[0].kcs_reg = 1;
			ast_lpc->ast_kcs[0].ast_lpc = ast_lpc;
			ast_lpc->ast_kcs[0].regspacings = AST_IPMI_KCS0_REGSPACINGS;	//fix
			ast_lpc->ast_kcs[0].kcs_irq_hander = NULL;
			ast_lpc->ast_kcs[0].str = ast_lpc->reg_base + AST_LPC_STR1;
			ast_lpc->ast_kcs[0].idr = ast_lpc->reg_base + AST_LPC_IDR1;
			ast_lpc->ast_kcs[0].odr = ast_lpc->reg_base + AST_LPC_ODR1;
			ast_set_ipmi_kcs_addr(ast_lpc, 0, 0xCA0);
			break;
		case 1:
			ast_lpc->ast_kcs[1].kcs_reg = 1;
			ast_lpc->ast_kcs[1].ast_lpc = ast_lpc;
			ast_lpc->ast_kcs[1].regspacings = AST_IPMI_KCS1_REGSPACINGS;	//fix
			ast_lpc->ast_kcs[1].kcs_irq_hander = NULL;
			ast_lpc->ast_kcs[1].str = ast_lpc->reg_base + AST_LPC_STR2;
			ast_lpc->ast_kcs[1].idr = ast_lpc->reg_base + AST_LPC_IDR2;
			ast_lpc->ast_kcs[1].odr = ast_lpc->reg_base + AST_LPC_ODR2;
			ast_set_ipmi_kcs_addr(ast_lpc, 1, 0xCA8);
			break;
		case 2:
			ast_lpc->ast_kcs[2].kcs_reg = 1;
			ast_lpc->ast_kcs[2].ast_lpc = ast_lpc;
			ast_lpc->ast_kcs[2].regspacings = AST_IPMI_KCS2_REGSPACINGS;	//fix
			ast_lpc->ast_kcs[2].kcs_irq_hander = NULL;
			ast_lpc->ast_kcs[2].str = ast_lpc->reg_base + AST_LPC_STR3;
			ast_lpc->ast_kcs[2].idr = ast_lpc->reg_base + AST_LPC_IDR3;
			ast_lpc->ast_kcs[2].odr = ast_lpc->reg_base + AST_LPC_ODR3;
			ast_set_ipmi_kcs_addr(ast_lpc, 2, 0xCA2);
			break;
		case 3:
			ast_lpc->ast_kcs[3].kcs_reg = 1;
			ast_lpc->ast_kcs[3].ast_lpc = ast_lpc;
			ast_lpc->ast_kcs[3].regspacings = AST_IPMI_KCS3_REGSPACINGS;	//can change any
			ast_lpc->ast_kcs[3].kcs_irq_hander = NULL;
			ast_lpc->ast_kcs[3].str = ast_lpc->reg_base + AST_LPC_STR4;
			ast_lpc->ast_kcs[3].idr = ast_lpc->reg_base + AST_LPC_IDR4;
			ast_lpc->ast_kcs[3].odr = ast_lpc->reg_base + AST_LPC_ODR4;
			ast_set_ipmi_kcs_addr(ast_lpc, 3, 0xCB2);
			break;
	}

	return &(ast_lpc->ast_kcs[kcs_no]);
}
EXPORT_SYMBOL(register_ipmi_kcs_drv);

void request_ipmi_kcs_irq(u8 kcs_no, ast_ipmi_irq handler)
{
	ast_lpc->ast_kcs[kcs_no].kcs_irq_hander = handler;

	switch(kcs_no) {
		case 0:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) | LPC_IBFIF1, AST_LPC_HICR2);
			break;
		case 1:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) | LPC_IBFIF2, AST_LPC_HICR2);
			break;
		case 2:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR3) | LPC_IBFIF3, AST_LPC_HICR2);
			break;
		case 3:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) | LPC_KCS4_RCV_INTR, AST_LPC_HICRB);
			break;
	}
}
EXPORT_SYMBOL(request_ipmi_kcs_irq);

u16 ast_get_ipmi_kcs_addr(struct ast_lpc_data *ast_lpc, u8 kcs_ch)
{
	u16 tmp = 0;
	switch(kcs_ch) {
		case 0:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_LADR12AS, AST_LPC_HICR4);
			tmp = (ast_lpc_read(ast_lpc, AST_LPC_LADR12H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR12L);
			break;
		case 1:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_LADR12AS, AST_LPC_HICR4);
			tmp = (ast_lpc_read(ast_lpc, AST_LPC_LADR12H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR12L);
			break;
		case 2:
			tmp = (ast_lpc_read(ast_lpc, AST_LPC_LADR3H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR3L);
			break;
		case 3:
			tmp = ast_lpc_read(ast_lpc, AST_LPC_LADR4) & 0xffff;
			break;
		default:
			break;
	}
	return tmp;
}
EXPORT_SYMBOL(ast_get_ipmi_kcs_addr);

void ast_set_ipmi_kcs_addr(struct ast_lpc_data *ast_lpc, u8 kcs_ch, u16 kcs_addr)
{
	LPC_DBUG("set ch %d, addr %x \n", kcs_ch, kcs_addr);
	switch(kcs_ch) {
		case 0:	//0xca0, 0xca4
			/* channel address select for channel 0  */
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_LADR12AS, AST_LPC_HICR4);
			ast_lpc_write(ast_lpc, kcs_addr >> 8, AST_LPC_LADR12H);
			ast_lpc_write(ast_lpc, kcs_addr & 0xff, AST_LPC_LADR12L);
			break;
		case 1:	//0xca2, 0xca6
			/* channel address select for channel 1  */
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_LADR12AS, AST_LPC_HICR4);
			ast_lpc_write(ast_lpc, kcs_addr >> 8, AST_LPC_LADR12H);
			ast_lpc_write(ast_lpc, kcs_addr & 0xff, AST_LPC_LADR12L);
			break;
		case 2:	//0xcb2, 0xcb3
			ast_lpc_write(ast_lpc, kcs_addr >> 8,AST_LPC_LADR3H);
			ast_lpc_write(ast_lpc, kcs_addr & 0xff, AST_LPC_LADR3L);
			break;
		case 3:	//0xcax~+4 +1
			ast_lpc_write(ast_lpc, ((kcs_addr + 1) << 16) | kcs_addr, AST_LPC_LADR4);
			break;
		case 4:	//bt +1

			break;
		default:
			break;
	}
}
EXPORT_SYMBOL(ast_set_ipmi_kcs_addr);

u8 ast_get_ipmi_kcs_en(struct ast_lpc_data *ast_lpc, u8 kcs_ch)
{
	u8 tmp = 0;
	switch(kcs_ch) {
		case 0:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC1_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 1:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC2_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 2:	//kcs 3
			if((ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC3_EN) && (ast_lpc_read(ast_lpc, AST_LPC_HICR4) & LPC_HICS_KCSENBL))
				tmp = 1;
			else
				tmp = 0;
			break;
		case 3:	//kcs4
			if(ast_lpc_read(ast_lpc, AST_LPC_HICRB) & LPC_KCS4_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}

	return tmp;

}
EXPORT_SYMBOL(ast_get_ipmi_kcs_en);

void ast_set_ipmi_kcs_en(struct ast_lpc_data *ast_lpc, u8 kcs_ch, u8 enable)
{
	if(enable) {
		switch(kcs_ch) {
			case 0: //kcs1
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC1_EN, AST_LPC_HICR0);
				break;
			case 1:	//kcs2
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC2_EN, AST_LPC_HICR0);
				break;
			case 2: //kcs3	--> fix 0xca2 / 0xca3
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC3_EN, AST_LPC_HICR0);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_KCSENBL, AST_LPC_HICR4);
				break;
			case 3: //kcs4
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) | LPC_KCS4_EN, AST_LPC_HICRB);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) | LPC_KCS4_RCV_INTR, AST_LPC_HICRB);
				break;
		}
	} else {
		switch(kcs_ch) {
			case 0:	//kcs 1
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) & ~LPC_LPC1_EN, AST_LPC_HICR0);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) & ~LPC_IBFIF1, AST_LPC_HICR2);
				break;
			case 1:	//kcs 2
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) & ~LPC_LPC2_EN, AST_LPC_HICR0);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) & ~LPC_IBFIF2, AST_LPC_HICR2);
				break;
			case 2:	//kcs 3
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_KCSENBL, AST_LPC_HICR4);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) & ~LPC_IBFIF2, AST_LPC_HICR2);
				break;
			case 3:	//kcs 4
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) & ~LPC_KCS4_RCV_INTR, AST_LPC_HICRB);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICRB) & ~LPC_KCS4_EN, AST_LPC_HICRB);
				break;
		}

	}
}
EXPORT_SYMBOL(ast_set_ipmi_kcs_en);

/**************************   IPMI BT Function  **********************************************************/
//Set default BT address BT0 :
struct ast_bt_data *register_ipmi_bt_drv(u8 bt_no)
{
	switch(bt_no) {
		case 0:
			ast_lpc->ast_bt[0].bt_reg = 1;
			ast_lpc->ast_bt[0].ast_lpc = ast_lpc;
			ast_lpc->ast_bt[0].regspacings = AST_IPMI_KCS0_REGSPACINGS;	//fix
			ast_lpc->ast_bt[0].bt_irq_hander = NULL;

			ast_lpc->ast_bt[0].str = ast_lpc->reg_base + AST_LPC_BTDTR;
			ast_lpc->ast_bt[0].fifo = ast_lpc->reg_base + AST_LPC_BTR1;
			ast_lpc->ast_bt[0].seq = 0;
			ast_lpc->ast_bt[0].state = BT_STATE_IDLE;	/* start here */
			ast_lpc->ast_bt[0].complete = BT_STATE_IDLE;	/* end here */
			ast_lpc->ast_bt[0].BT_CAP_req2rsp = BT_NORMAL_TIMEOUT * 1000000;
			ast_lpc->ast_bt[0].BT_CAP_retries = BT_NORMAL_RETRY_LIMIT;
			break;
		case 1:
			ast_lpc->ast_bt[1].bt_reg = 1;
			ast_lpc->ast_bt[1].ast_lpc = ast_lpc;
			ast_lpc->ast_bt[1].regspacings = AST_IPMI_KCS0_REGSPACINGS;	//fix
			ast_lpc->ast_bt[1].bt_irq_hander = NULL;

			ast_lpc->ast_bt[1].isr = ast_lpc->reg_base + AST_LPC_IBTCR2;
			ast_lpc->ast_bt[1].str = ast_lpc->reg_base + AST_LPC_IBTCR4;
			ast_lpc->ast_bt[1].fifo = ast_lpc->reg_base + AST_LPC_IBTCR5;
			ast_lpc->ast_bt[1].fifo_sts = ast_lpc->reg_base + AST_LPC_IBTCR3;
			ast_lpc->ast_bt[1].seq = 0;
			ast_lpc->ast_bt[1].state = BT_STATE_IDLE;	/* start here */
			ast_lpc->ast_bt[1].complete = BT_STATE_IDLE;	/* end here */
			ast_lpc->ast_bt[1].BT_CAP_req2rsp = BT_NORMAL_TIMEOUT * 1000000;
			ast_lpc->ast_bt[1].BT_CAP_retries = BT_NORMAL_RETRY_LIMIT;
			ast_set_ipmi_bt_addr(ast_lpc, 1, 0xca0);
//			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) | LPC_iBT_ENABLE | LPC_iBT_ClrSvRdP_EN | LPC_iBT_ClrSvWrP_EN, AST_LPC_IBTCR0); //Enable BT Interface
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) | LPC_iBT_ClrSvRdP_EN | LPC_iBT_ClrSvWrP_EN, AST_LPC_IBTCR0); //Enable BT Interface
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR1) | LPC_iBT_H2B_RISING_ISR, AST_LPC_IBTCR1); //Enable BT H2B interrupt Interrupt
#if 1
			if(ast_lpc_read(ast_lpc, AST_LPC_IBTCR4) & BT_B_BUSY) {
				ast_lpc_write(ast_lpc, BT_B_BUSY, AST_LPC_IBTCR4);
				printk("change to IDLE \n");
			}
#endif
			break;
	}

	return &(ast_lpc->ast_bt[bt_no]);
}
EXPORT_SYMBOL(register_ipmi_bt_drv);

void request_ipmi_bt_irq(u8 bt_no, ast_ipmi_irq handler)
{
	ast_lpc->ast_bt[bt_no].bt_irq_hander = handler;

	switch(bt_no) {
		case 0:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR2) | LPC_IBFIF1, AST_LPC_HICR2);
			break;
		case 1:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR1) | LPC_iBT_H2B_RISING_ISR, AST_LPC_IBTCR1);	//Enable BT H2B interrupt Interrupt
			break;

	}
}
EXPORT_SYMBOL(request_ipmi_bt_irq);

u16 ast_get_ipmi_bt_irq(struct ast_lpc_data *ast_lpc, u8 bt_ch)
{
	u16 tmp = 0;
	switch(bt_ch) {
		case 0:
			//TODO
			tmp = 0;
			break;
		case 1:	//ibt
			tmp = LPC_iBT_GET_IRQ(ast_lpc_read(ast_lpc, AST_LPC_IBTCR0));
			break;
		default:
			break;
	}
	return tmp;
}
EXPORT_SYMBOL(ast_get_ipmi_bt_irq);

void ast_set_ipmi_bt_irq(struct ast_lpc_data *ast_lpc, u8 bt_ch, u16 bt_addr)
{
	LPC_DBUG("set ch %d, addr %x \n", bt_ch, bt_addr);
	switch(bt_ch) {
		case 0:
			//TODO ~~
			break;
		case 1:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) & ~LPC_iBT_IRQ_MASK) | LPC_iBT_SET_IRQ(bt_addr), AST_LPC_IBTCR0);
			break;
		default:
			break;
	}
}
EXPORT_SYMBOL(ast_set_ipmi_bt_irq);

u16 ast_get_ipmi_bt_addr(struct ast_lpc_data *ast_lpc, u8 bt_ch)
{
	u16 tmp = 0;
	switch(bt_ch) {
		case 0:	//0xca2, 0xca3 is kcs , ca4,ca5,ca6 is bt
			tmp = ((ast_lpc_read(ast_lpc, AST_LPC_LADR3H) << 8) | ast_lpc_read(ast_lpc, AST_LPC_LADR3L)) + 2;
			break;
		case 1:	//ibt
			tmp = LPC_iBT_GET_ADDR(ast_lpc_read(ast_lpc, AST_LPC_IBTCR0));
			break;
		default:
			break;
	}
	return tmp;
}
EXPORT_SYMBOL(ast_get_ipmi_bt_addr);

void ast_set_ipmi_bt_addr(struct ast_lpc_data *ast_lpc, u8 bt_ch, u16 bt_addr)
{
	LPC_DBUG("set ch %d, addr %x \n", bt_ch, bt_addr);
	u32 iBT_Ctrl = 0;
	switch(bt_ch) {
		case 0:	//0xca2, 0xca3 is kcs , ca4,ca5,ca6 is bt
			ast_lpc_write(ast_lpc, (bt_addr - 2) >> 8,AST_LPC_LADR3H);
			ast_lpc_write(ast_lpc, (bt_addr - 2) & 0xff, AST_LPC_LADR3L);
			break;
		case 1:	//ibt [31:16] ex  == 0x00e4 --> BTCR : 0xE4 , BTDTR : 0xE4 + 1, BTIMSR : 0xE4 + 2
			iBT_Ctrl = ast_lpc_read(ast_lpc, AST_LPC_IBTCR0);
			iBT_Ctrl &= ~(LPC_iBT_ADDR_MASK | LPC_iBT_IRQ_TYPE_MASK);
			//set Host SerIRQ interrupt type for iBT is 11: rising edge trig
			iBT_Ctrl |= (0x3 << 10) | LPC_iBT_SET_ADDR(bt_addr);
			ast_lpc_write(ast_lpc, iBT_Ctrl, AST_LPC_IBTCR0);
			break;
		default:
			break;
	}
}
EXPORT_SYMBOL(ast_set_ipmi_bt_addr);

u8 ast_get_ipmi_bt_en(struct ast_lpc_data *ast_lpc, u8 bt_ch)
{
	u8 tmp = 0;
	switch(bt_ch) {
		case 0:
			if((ast_lpc_read(ast_lpc, AST_LPC_HICR0) & LPC_LPC3_EN) && (ast_lpc_read(ast_lpc, AST_LPC_HICR4) & LPC_HICS_BTENBL))
				tmp = 1;
			else
				tmp = 0;
			break;
		case 1:	//ibt
			if(ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) & LPC_iBT_ENABLE)
				tmp = 1;
			else
				tmp = 0;
			break;

		default:
			printk("Error Ch no !!\n");
			break;
	}

	return tmp;

}
EXPORT_SYMBOL(ast_get_ipmi_bt_en);

void ast_set_ipmi_bt_en(struct ast_lpc_data *ast_lpc, u8 bt_ch, u8 enable)
{
	if(enable) {
		switch(bt_ch) {
			case 0: //bt		--> fix 0xca4 / 0xca5 / 0xca6
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR0) | LPC_LPC3_EN, AST_LPC_HICR0);
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) | LPC_HICS_BTENBL, AST_LPC_HICR4);
				//TODO .....
				break;
			case 1:	//ibt
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) | LPC_iBT_ENABLE | LPC_iBT_ClrSvRdP_EN | LPC_iBT_ClrSvWrP_EN, AST_LPC_IBTCR0);	//Enable BT Interface
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR1) | LPC_iBT_H2B_RISING_ISR, AST_LPC_IBTCR1);	//Enable BT H2B interrupt Interrupt
				break;
		}
	} else {
		switch(bt_ch) {
			case 0:	//bt
				//TODO
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR4) & ~LPC_HICS_BTENBL, AST_LPC_HICR4);
				break;
			case 1:	//ibt
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR1) & ~LPC_iBT_H2B_RISING_ISR, AST_LPC_IBTCR1);	//Enable BT H2B interrupt Interrupt
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_IBTCR0) & ~(LPC_iBT_ENABLE | LPC_iBT_ClrSvRdP_EN | LPC_iBT_ClrSvWrP_EN), AST_LPC_IBTCR0);	//Enable BT Interface
				break;
		}

	}
}

EXPORT_SYMBOL(ast_set_ipmi_bt_en);
/**************************   SNOOP Function  **********************************************************/
struct ast_snoop_data *register_snoop_drv(u8 snoop_no)
{
	switch(snoop_no) {
		case 0:
			ast_lpc->ast_snoop[0].snoop_reg = 1;
			ast_lpc->ast_snoop[0].ast_lpc = ast_lpc;
			ast_lpc->ast_snoop[0].snoop_irq_hander = NULL;
			break;
		case 1:
			ast_lpc->ast_snoop[1].snoop_reg = 1;
			ast_lpc->ast_snoop[1].ast_lpc = ast_lpc;
			ast_lpc->ast_snoop[1].snoop_irq_hander = NULL;
			break;
	}
	return &(ast_lpc->ast_snoop[snoop_no]);
}
EXPORT_SYMBOL(register_snoop_drv);

void request_snoop_irq(u8 snoop_no, ast_ipmi_irq handler)
{
	ast_lpc->ast_snoop[snoop_no].snoop_irq_hander = handler;

	switch(snoop_no) {
		case 0:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_SNP0INT_EN, AST_LPC_HICR5);
			break;
		case 1:
			ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_SNP1INT_EN, AST_LPC_HICR5);
			break;

	}
}
EXPORT_SYMBOL(request_snoop_irq);

u16 ast_get_lpc2gpio(struct ast_lpc_data *ast_lpc, u8 snoop_ch)
{
	u16 tmp = 0;
	switch(snoop_ch) {
		case 0:
			tmp = LPC_HICR5_GET_SEL80HGIO(ast_lpc_read(ast_lpc, AST_LPC_HICR5));
			break;
	}
	return tmp;
}
EXPORT_SYMBOL(ast_get_lpc2gpio);

void ast_set_lpc2gpio(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u16 sel80hgio)
{
	switch(snoop_ch) {
		case 0:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_HICR5) & (~(0x1f << 24))) | (sel80hgio << 24),
					AST_LPC_HICR5);
			break;
	}
}
EXPORT_SYMBOL(ast_set_lpc2gpio);

u16 ast_get_lpc2gpio_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch)
{
	u16 tmp = 0;
	switch(snoop_ch) {
		case 0:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & (LPC_HICR5_ENSIOGIO | LPC_HICR5_EN80HGIO | LPC_HICR5_ENINVGIO))
				tmp = 1;
			else
				tmp = 0;
			break;
	}
	return tmp;
}
EXPORT_SYMBOL(ast_get_lpc2gpio_en);

void ast_set_lpc2gpio_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u8 enable)
{
	switch(snoop_ch) {
		case 0:
			if(enable)
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_ENSIOGIO | LPC_HICR5_EN80HGIO | LPC_HICR5_ENINVGIO,
						AST_LPC_HICR5);
			else
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~(LPC_HICR5_ENSIOGIO | LPC_HICR5_EN80HGIO | LPC_HICR5_ENINVGIO),
						AST_LPC_HICR5);
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
}
EXPORT_SYMBOL(ast_set_lpc2gpio_en);

u8 ast_get_snoop_data(struct ast_lpc_data *ast_lpc, u8 snoop_ch)
{
	u8 tmp;

	LPC_DBUG("Get CH%d data %d:[%x]",snoop_ch,ast_lpc->ast_snoop[snoop_ch].read_idx,ast_lpc->ast_snoop[snoop_ch].fifo[ast_lpc->ast_snoop[snoop_ch].read_idx]);
//	if(!((ast_lpc->ast_snoop[snoop_ch].read_idx + 1) >= ast_lpc->ast_snoop[snoop_ch].write_idx)) {
	if(ast_lpc->ast_snoop[snoop_ch].write_idx != (ast_lpc->ast_snoop[snoop_ch].read_idx)) {
		tmp = ast_lpc->ast_snoop[snoop_ch].fifo[ast_lpc->ast_snoop[snoop_ch].read_idx];
		ast_lpc->ast_snoop[snoop_ch].read_idx ++;
		ast_lpc->ast_snoop[snoop_ch].read_idx %= SNOOP_FIFO_SIZE;
	} else {
		if(ast_lpc->ast_snoop[snoop_ch].read_idx == 0)
			tmp = ast_lpc->ast_snoop[snoop_ch].fifo[SNOOP_FIFO_SIZE - 1];
		else
			tmp = ast_lpc->ast_snoop[snoop_ch].fifo[ast_lpc->ast_snoop[snoop_ch].read_idx -1];
	}
	//tmp = GET_LPC_SNPD0(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
	//tmp = GET_LPC_SNPD1(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
	return tmp;
}
EXPORT_SYMBOL(ast_get_snoop_data);

u16 ast_get_snoop_port(struct ast_lpc_data *ast_lpc, u8 snoop_ch)
{
	u16 tmp = 0;
	switch(snoop_ch) {
		case 0:
			tmp = LPC_GET_SNOOP_ADDR0(ast_lpc_read(ast_lpc, AST_LPC_SNPWADR));
			break;
		case 1:
			tmp = LPC_GET_SNOOP_ADDR1(ast_lpc_read(ast_lpc, AST_LPC_SNPWADR));
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
	return tmp;
}
EXPORT_SYMBOL(ast_get_snoop_port);

void ast_set_snoop_port(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u16 snoop_port)
{
	switch(snoop_ch) {
		case 0:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_SNPWADR) & ~LPC_SNOOP_ADDR0_MASK) | snoop_port,
					AST_LPC_SNPWADR);
			break;

		case 1:
			ast_lpc_write(ast_lpc, (ast_lpc_read(ast_lpc, AST_LPC_SNPWADR) & ~LPC_SNOOP_ADDR1_MASK) | (snoop_port << 16),
					AST_LPC_SNPWADR);
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
}
EXPORT_SYMBOL(ast_set_snoop_port);

void ast_set_snoop_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch, u8 enable)
{
	switch(snoop_ch) {
		case 0:
			if(enable)
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_SNP0W_EN, AST_LPC_HICR5);
			else
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_SNP0W_EN, AST_LPC_HICR5);
			break;
		case 1:
			if(enable)
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_SNP1W_EN, AST_LPC_HICR5);
			else
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_SNP1W_EN, AST_LPC_HICR5);
			break;
		case 2:
			if(enable)
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_PCCR0) | LPC_POST_CODE_EN, AST_LPC_PCCR0);
			else
				ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & ~LPC_POST_CODE_EN, AST_LPC_PCCR0);
		default:
			printk("Error Ch no !!\n");
			break;
	}

}
EXPORT_SYMBOL(ast_set_snoop_en);

u8 ast_get_snoop_en(struct ast_lpc_data *ast_lpc, u8 snoop_ch)
{
	u8 tmp = 0;
	switch(snoop_ch) {
		case 0:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_SNP0W_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 1:
			if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_SNP1W_EN)
				tmp = 1;
			else
				tmp = 0;
			break;
		case 2:
			if(ast_lpc_read(ast_lpc, AST_LPC_PCCR0) & LPC_POST_CODE_EN)
				tmp = 1;
			else
				tmp = 0;
		default:
			printk("Error Ch no !!\n");
			break;
	}

	return tmp;

}
EXPORT_SYMBOL(ast_get_snoop_en);
/**************************   SNOOP DMA Function  **********************************************************/
extern struct ast_snoop_dma_data *register_snoop_dma_drv(void)
{
	ast_lpc->ast_snoop_dma->snoop_dma_reg = 1;
	ast_lpc->ast_snoop_dma->ast_lpc = ast_lpc;
	ast_lpc->ast_snoop_dma->snoop_irq_hander = NULL;

	return ast_lpc->ast_snoop_dma;
}
EXPORT_SYMBOL(register_snoop_dma_drv);

void request_snoop_dma_irq(ast_ipmi_irq handler)
{
	ast_lpc->ast_snoop_dma->snoop_irq_hander = handler;

}
EXPORT_SYMBOL(request_snoop_dma_irq);

/****************************************************************************************************/
static irqreturn_t ast_lpc_isr (int this_irq, void *dev_id)
{
	struct ast_lpc_data *ast_lpc = dev_id;
	LPC_DBUG("\n");

#if 0
	if(ast_lpc_read(ast_lpc, AST_LPC_HICR2) & LPC_LRST) {
		printk("LPC_LRST TODO ~~\n");
	}

	if(ast_lpc_read(ast_lpc, AST_LPC_HICR2) & LPC_SDWN) {
		printk("LPC_SDWN TODO ~~\n");
	}

	if(ast_lpc_read(ast_lpc, AST_LPC_HICR2) & LPC_ABRT) {
		printk("LPC_ABRT TODO ~~\n");
	}
#endif

	//SNOOP
#ifdef CONFIG_AST_SNOOP
	if(ast_lpc_read(ast_lpc, AST_LPC_HICR6) & LPC_HICR6_STR_SNP0W) {
		ast_lpc->ast_snoop[0].fifo[ast_lpc->ast_snoop[0].write_idx] =
			GET_LPC_SNPD0(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
#if 0
		printk("Ch0 data %x \n",
			GET_LPC_SNPD1(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR)));
		printk("Ch0 data in fifo widx %d : %x \n",
		ast_lpc->ast_snoop[0].write_idx,
		ast_lpc->ast_snoop[0].fifo[ast_lpc->ast_snoop[0].write_idx]);
#endif
		ast_lpc->ast_snoop[0].snoop_irq_hander(&ast_lpc->ast_snoop[0]);
		ast_lpc_write(ast_lpc, LPC_HICR6_STR_SNP0W,  AST_LPC_HICR6);
	}

	if(ast_lpc_read(ast_lpc, AST_LPC_HICR6) & LPC_HICR6_STR_SNP1W) {
		ast_lpc->ast_snoop[1].fifo[ast_lpc->ast_snoop[1].write_idx] =
			GET_LPC_SNPD1(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR));
#if 0
		printk("Ch1 data %x \n",
			GET_LPC_SNPD1(ast_lpc_read(ast_lpc, AST_LPC_SNPWDR)));
		printk("Ch1 data in fifo widx %d : %x \n",
		ast_lpc->ast_snoop[0].write_idx,
		ast_lpc->ast_snoop[1].fifo[ast_lpc->ast_snoop[1].write_idx]);
#endif
		ast_lpc->ast_snoop[1].snoop_irq_hander(&ast_lpc->ast_snoop[1]);
		ast_lpc_write(ast_lpc, LPC_HICR6_STR_SNP1W,  AST_LPC_HICR6);
	}
#endif

#ifdef CONFIG_AST_SNOOP_DMA
	if(ast_lpc_read(ast_lpc, AST_LPC_PCCR2) & LPC_POST_CODE_STS) {
    if (ast_lpc->ast_snoop_dma->snoop_irq_hander)
      ast_lpc->ast_snoop_dma->snoop_irq_hander(ast_lpc->ast_snoop_dma);
	}
#endif

#ifdef CONFIG_AST_IPMI_KCS
	//kcs 1:
	if(ast_lpc_read(ast_lpc, AST_LPC_STR1) & LPC_STR_IBF) {
		ast_lpc->ast_kcs[0].kcs_irq_hander(&ast_lpc->ast_kcs[0]);
	}
	//kcs 2
	if(ast_lpc_read(ast_lpc, AST_LPC_STR2) & LPC_STR_IBF) {
		ast_lpc->ast_kcs[2].kcs_irq_hander(&ast_lpc->ast_kcs[1]);
	}
	//kcs 3
	if(ast_lpc_read(ast_lpc, AST_LPC_STR3) & LPC_STR_IBF) {
		ast_lpc->ast_kcs[2].kcs_irq_hander(&ast_lpc->ast_kcs[2]);
	}
	//kcs 4
	if(ast_lpc_read(ast_lpc, AST_LPC_STR4) & LPC_STR_IBF) {
		ast_lpc->ast_kcs[3].kcs_irq_hander(&ast_lpc->ast_kcs[3]);
	}
#endif

#ifdef CONFIG_AST_IPMI_BT
	//bt 0
	if(ast_lpc_read(ast_lpc, AST_LPC_HICR4) & LPC_HICS_BTENBL) {
		ast_lpc->ast_bt[0].bt_irq_hander(&ast_lpc->ast_bt[0]);
	}
	//ibt 1
	if(ast_lpc_read(ast_lpc, AST_LPC_IBTCR2) & LPC_iBT_H2B_RISING_ISR) {
		ast_lpc->ast_bt[1].bt_irq_hander(&ast_lpc->ast_bt[1]);
	}
#endif
	return IRQ_HANDLED;
}
/************************************************** SYS FS **************************************************************/
/* SERIRQ generate
SBIOS Programming:
SIORD_70[3:0]=Ah (for SERIRQ#10)
SIORD_30[4]=1b (for enabling)
SIORx_20[4] (for status reading; write 1 clear)

BMC Programming:
1e789108[0] (HISR0) =0b then =1b (triggering SERIRQ by setting 0->1)
1e789178[4] (for status reading)

*/
static ssize_t store_sio_irq(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_lpc_write(ast_lpc, val, AST_LPC_HISR0);
	else
		ast_lpc_write(ast_lpc, 0, AST_LPC_HISR0);

	return count;
}

static DEVICE_ATTR(sio_irq, S_IWUSR, NULL, store_sio_irq);

static ssize_t store_lpc2ahb_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_ENL2H, AST_LPC_HICR5);
	else
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_ENL2H, AST_LPC_HICR5);

	ast_lpc_write(ast_lpc, 0xffff0000, AST_LPC_HICR8);

	return count;
}

static ssize_t show_lpc2ahb_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);

	if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_ENL2H)
		return sprintf(buf, "1: Enable\n");
	else
		return sprintf(buf, "0: Disable\n");
}

static DEVICE_ATTR(lpc2ahb_en, S_IRUGO | S_IWUSR, show_lpc2ahb_en, store_lpc2ahb_en);

static ssize_t store_fwcycle_en(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) | LPC_HICR5_ENFWH, AST_LPC_HICR5);
	else
		ast_lpc_write(ast_lpc, ast_lpc_read(ast_lpc, AST_LPC_HICR5) & ~LPC_HICR5_ENFWH, AST_LPC_HICR5);

	return count;
}

static ssize_t show_fwcycle_en(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);

	if(ast_lpc_read(ast_lpc, AST_LPC_HICR5) & LPC_HICR5_ENFWH)
		return sprintf(buf, "1: Enable\n");
	else
		return sprintf(buf, "0: Disable\n");
}

static DEVICE_ATTR(fwcycle_en, S_IRUGO | S_IWUSR, show_fwcycle_en, store_fwcycle_en);

static ssize_t show_fwcycle_addr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);

	return sprintf(buf, "%x\n", ast_lpc_read(ast_lpc, AST_LPC_HICR7) );
}

static ssize_t store_fwcycle_addr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);

	val = StrToHex(buf);

	ast_lpc_write(ast_lpc, val, AST_LPC_HICR7);
	return count;
}

static DEVICE_ATTR(fwcycle_addr, S_IRUGO | S_IWUSR, show_fwcycle_addr, store_fwcycle_addr);

static struct attribute *ast_lpc_attributes[] = {
	&dev_attr_fwcycle_en.attr,
	&dev_attr_fwcycle_addr.attr,
	&dev_attr_lpc2ahb_en.attr,
	&dev_attr_sio_irq.attr,
	NULL
};

static const struct attribute_group lpc_attribute_group = {
	.attrs = ast_lpc_attributes
};

/**************************   LPC  Snoop Sys fs   End **********************************************************/
static int
ast_get_route_selio(struct ast_lpc_data *ast_lpc, u8 io_ch, char *sysfsbuf)
{
	u8 tmp = 0;
	switch(io_ch) {
		case 0:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL5IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART0->IO5, 1:UART1->IO5, 2:UART2->IO5, 3:UART3->IO5, 4:UART4->IO5");
			break;
		case 1:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL1IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART1->IO1, 1:UART2->IO1, 2:UART3->IO1, 3:UART4->IO1, 4:UART0->IO1");
			break;
		case 2:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL2IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART2->IO2, 1:UART3->IO2, 2:UART4->IO2, 3:UART0->IO2, 4:UART1->IO2");
			break;
		case 3:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL3IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART3->IO3, 1:UART4->IO3, 2:UART5->IO3, 3:UART1->IO3, 4:UART2->IO3");
			break;
		case 4:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL4IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: UART4->IO4, 1:UART0->IO4, 2:UART1->IO4, 3:UART2->IO4, 4:UART3->IO4");
			break;
		case 6:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL6IO(ast_lpc_read(ast_lpc, AST_LPC_HICR9)), "0: UART1->IO6, 1:UART2->IO6, 2:UART3->IO6, 3:UART4->IO6, 4:UART0->IO6");
			break;

		default:
			printk("Error Ch no !!\n");
			break;
	}

	return tmp;

}

static int
ast_get_route_seldw(struct ast_lpc_data *ast_lpc, u8 io_ch, char *sysfsbuf)
{
	u8 tmp = 0;
	switch(io_ch) {
		case 0:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL5DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO0->UART0, 1:IO1->UART0, 2:IO2->UART0, 3:IO3->UART0, 4:IO4->UART0, 5:UART1->UART0, 6:UART2->UART0, 7:UART3->UART0, 8:UART4->UART0, 9:IO6->UART0");
			break;
		case 1:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL1DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO1->UART1, 1:IO2->UART1, 2:IO3->UART1, 3:IO4->UART1, 4:UART2->UART1, 5:UART3->UART1, 6:UART4->UART1, 7:IO6->UART1");
			break;
		case 2:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL2DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO2->UART2, 1:IO3->UART2, 2:IO4->UART2, 3:IO1->UART2, 4:UART3->UART2, 5:UART4->UART2, 6:UART1->UART2, 7:IO6->UART2");
			break;
		case 3:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL3DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO3->UART3, 1:IO4->UART3, 2:IO1->UART3, 3:IO2->UART3, 4:UART4->UART3, 5:UART1->UART3, 6:UART2->UART3, 7:IO6->UART3");
			break;
		case 4:
			return sprintf(sysfsbuf, "%d - [%s]\n", GET_LPC_SEL4DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA)), "0: IO4->UART4, 1:IO1->UART4, 2:IO2->UART4, 3:IO3->UART4, 4:UART1->UART4, 5:UART2->UART4, 6:UART3->UART4, 7:IO6->UART4");
			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}

	return tmp;

}

static void
ast_set_route_selio(struct ast_lpc_data *ast_lpc, u8 io_ch, u8 value)
{
	switch(io_ch) {
		case 0:
			ast_lpc_write(ast_lpc, SET_LPC_SEL5IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 1:
			ast_lpc_write(ast_lpc, SET_LPC_SEL1IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 2:
			ast_lpc_write(ast_lpc, SET_LPC_SEL2IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 3:
			ast_lpc_write(ast_lpc, SET_LPC_SEL3IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 4:
			ast_lpc_write(ast_lpc, SET_LPC_SEL4IO(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 6:
			ast_lpc_write(ast_lpc, SET_LPC_SEL6IO(ast_lpc_read(ast_lpc, AST_LPC_HICR9), value),
						AST_LPC_HICR9);
			break;

		default:
			printk("Error Ch no !!\n");
			break;
	}

}

static void
ast_set_route_seldw(struct ast_lpc_data *ast_lpc, u8 io_ch, u8 value)
{
	switch(io_ch) {
		case 0:
			ast_lpc_write(ast_lpc, SET_LPC_SEL5DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 1:
			ast_lpc_write(ast_lpc, SET_LPC_SEL1DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 2:
			ast_lpc_write(ast_lpc, SET_LPC_SEL2DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 3:
			ast_lpc_write(ast_lpc, SET_LPC_SEL3DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);
			break;
		case 4:
			ast_lpc_write(ast_lpc, SET_LPC_SEL4DW(ast_lpc_read(ast_lpc, AST_LPC_HICRA), value),
						AST_LPC_HICRA);

			break;
		default:
			printk("Error Ch no !!\n");
			break;
	}
}

static ssize_t
ast_store_route(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr)
	{
		case 0: //selio
			ast_set_route_selio(ast_lpc, sensor_attr->index, input_val);
			break;
		case 1: //seldw
			ast_set_route_seldw(ast_lpc, sensor_attr->index, input_val);
			break;
		default:
			return -EINVAL;
			break;
	}

	return count;
}

static ssize_t
ast_show_route(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{

	struct ast_lpc_data *ast_lpc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	//sensor_attr->index : ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr)
	{
		case 0: //selio
			return ast_get_route_selio(ast_lpc, sensor_attr->index, sysfsbuf);
			break;
		case 1: //seldw
			return ast_get_route_seldw(ast_lpc, sensor_attr->index, sysfsbuf);
			break;
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
}

#define sysfs_route_ch(index) \
static SENSOR_DEVICE_ATTR_2(route##index##_selio, S_IRUGO | S_IWUSR, \
	ast_show_route, ast_store_route, 0, index); \
static SENSOR_DEVICE_ATTR_2(route##index##_seldw, S_IRUGO | S_IWUSR, \
	ast_show_route, ast_store_route, 1, index); \
\
static struct attribute *route##index##_attributes[] = { \
	&sensor_dev_attr_route##index##_selio.dev_attr.attr, \
	&sensor_dev_attr_route##index##_seldw.dev_attr.attr, \
	NULL \
};

sysfs_route_ch(0);
sysfs_route_ch(1);
sysfs_route_ch(2);
sysfs_route_ch(3);
sysfs_route_ch(4);
sysfs_route_ch(5);
sysfs_route_ch(6);


static const struct attribute_group route_attribute_groups[] = {
	{ .attrs = route0_attributes },
	{ .attrs = route1_attributes },
	{ .attrs = route2_attributes },
	{ .attrs = route3_attributes },
	{ .attrs = route4_attributes },
	{ .attrs = route5_attributes },
	{ .attrs = route6_attributes },
};

/**************************   LPC  route Sys fs   End **********************************************************/
static int ast_lpc_probe(struct platform_device *pdev)
{
//	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct resource *res;
	int ret = 0;
	int i = 0;
	LPC_DBUG("\n");

	ast_lpc = kzalloc(sizeof(struct ast_lpc_data), GFP_KERNEL);
	if (ast_lpc == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ast_lpc->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto err_free;
	}

	ast_lpc->reg_base = ioremap(res->start, resource_size(res));
	if (ast_lpc->reg_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	ast_lpc->irq = platform_get_irq(pdev, 0);
	if (ast_lpc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto err_free_mem;
	}

	ret = request_irq(ast_lpc->irq, ast_lpc_isr, IRQF_SHARED, "ast-lpc", ast_lpc);
	if (ret) {
		printk("AST LPC Unable to get IRQ");
		goto err_free_mem;
	}

#ifdef CONFIG_AST_LPC_MASTER
	printk("LPC Scan Device... \n");
#ifdef CONFIG_ARCH_AST1070
	for(i=0;i<ast_lpc->bus_info->scan_node;i++) {
		ast1070_scu_init(i ,AST_LPC_BRIDGE + i*0x10000);
		printk("C%d-[%x] ", i, ast1070_revision_id_info(i));
		ast1070_vic_init(i, (AST_LPC_BRIDGE + i*0x10000), IRQ_C0_VIC_CHAIN + i, IRQ_C0_VIC_CHAIN_START + (i*AST_CVIC_NUM));
		ast1070_i2c_irq_init(i, (AST_LPC_BRIDGE + i*0x10000), IRQ_C0_VIC_CHAIN + i, IRQ_C0_VIC_CHAIN_START + (i*AST_CVIC_NUM));
		ast1070_scu_dma_init(i);
		ast1070_uart_dma_init(i, AST_LPC_BRIDGE);
		ast_add_device_cuart(i,AST_LPC_BRIDGE + i*0x10000);
		ast_add_device_ci2c(i,AST_LPC_BRIDGE + i*0x10000);
	}
#endif
	printk("\n");
	platform_set_drvdata(pdev, ast_lpc);
#else
	platform_set_drvdata(pdev, ast_lpc);

	dev_set_drvdata(&pdev->dev, ast_lpc);

	ret = sysfs_create_group(&pdev->dev.kobj, &lpc_attribute_group);
	if (ret)
		goto err_free_mem;

	for(i=0; i< 7; i++) {
		ret = sysfs_create_group(&pdev->dev.kobj, &route_attribute_groups[i]);
		if (ret)
			goto err_free_mem;
	}

#ifdef CONFIG_AST_SNOOP
	ast_lpc->ast_snoop = kzalloc(sizeof(struct ast_snoop_data) * AST_SNOOP_NUM, GFP_KERNEL);
#endif

#ifdef CONFIG_AST_SNOOP_DMA
	ast_lpc->ast_snoop_dma = kzalloc(sizeof(struct ast_snoop_dma_data), GFP_KERNEL);
#endif

#ifdef CONFIG_AST_IPMI_KCS
	ast_lpc->ast_kcs = kzalloc(sizeof(struct ast_kcs_data) * AST_KCS_NUM, GFP_KERNEL);
#endif

#ifdef CONFIG_AST_IPMI_BT
	ast_lpc->ast_bt = kzalloc(sizeof(struct ast_bt_data) * AST_BT_NUM, GFP_KERNEL);
#endif

#endif

	return 0;

err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free:
	kfree(ast_lpc);

	return ret;
}

static struct platform_driver ast_lpc_driver = {
	.driver		= {
		.name			= "ast-lpc",
		.owner			= THIS_MODULE,
	},
	.probe 		= ast_lpc_probe,
};

static int __init ast_lpc_init(void)
{
	return platform_driver_register(&ast_lpc_driver);
}
arch_initcall(ast_lpc_init);
