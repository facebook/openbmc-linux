/********************************************************************************
* File Name     : arch/arm/plat-aspeed/ast-scu.c 
* Author         : Ryan Chen
* Description   : AST SCU 
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

CLK24M 
 |
 |--> H-PLL -->HCLK
 |
 |--> M-PLL -xx->MCLK
 |
 |--> V-PLL1 -xx->DCLK
 |
 |--> V-PLL2 -xx->D2CLK
 |
 |--> USB2PHY -->UTMICLK


*   History      : 
*    1. 2012/08/15 Ryan Chen Create
* 
********************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/platform.h>
#include <asm/io.h>

#include <mach/hardware.h>

#include <plat/ast2400-scu.h>
#include <plat/regs-scu.h>

//#define ASPEED_SCU_LOCK
//#define ASPEED_SCU_DEBUG

#ifdef ASPEED_SCU_DEBUG
#define SCUDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SCUDBUG(fmt, args...)
#endif

#define SCUMSG(fmt, args...) printk(fmt, ## args)

static u32 ast2400_scu_base;

static inline u32 
ast2400_scu_read(u32 reg)
{
	u32 val;
		
	val = readl(ast2400_scu_base + reg);
	
	SCUDBUG("ast2400_scu_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast2400_scu_write(u32 val, u32 reg) 
{
	SCUDBUG("ast2400_scu_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
#ifdef CONFIG_AST_SCU_LOCK
	//unlock 
	writel(SCU_PROTECT_UNLOCK, ast2400_scu_base);
	writel(val, ast2400_scu_base + reg);
	//lock
	writel(0xaa,ast2400_scu_base);	
#else
	writel(val, ast2400_scu_base + reg);
#endif
}

extern void
ast2400_scu_init(u32 scu_base, u32 size)
{
	ast2400_scu_base = ioremap(scu_base, size);
}

//////////////////////////////////////////////////////////////////////////////
//SoC mapping Table 
struct soc_id {
	const char	* name;
	u32	rev_id;
};

static struct soc_id soc_map_table[] = {
	[0] = {
		.name = "AST1100/AST2050-A0",
		.rev_id = 0x00000200,
	},
	[1] = {
		.name = "AST1100/AST2050-A1",
		.rev_id = 0x00000201,
	},
	[2] = {
		.name = "AST1100/AST2050-A2,3/AST2150-A0,1",
		.rev_id = 0x00000202,
	},
	[3] = {
		.name = "AST1510/AST2100-A0",
		.rev_id = 0x00000300,
	},
	[4] = {
		.name = "AST1510/AST2100-A1",
		.rev_id = 0x00000301,
	},
	[5] = {
		.name = "AST1510/AST2100-A2,3",
		.rev_id = 0x00000302,
	},
	[6] = {
		.name = "AST2200-A0,1",
		.rev_id = 0x00000102,
	},
	[7] = {
		.name = "AST2300-A0",
		.rev_id = 0x01000003,
	},
	[8] = {
		.name = "AST2300-A1",
		.rev_id = 0x01010303,
	},
	[9] = {
		.name = "AST1300-A1",
		.rev_id = 0x01010003,
	},
	[10] = {
		.name = "AST1050-A1",
		.rev_id = 0x01010203,
	},
	[11] = {
		.name = "AST2400-A0",
		.rev_id = 0x02000303,
	},
	[12] = {
		.name = "AST2400-A1",
		.rev_id = 0x02010303,
	},

};

//***********************************Initial control***********************************
extern void
ast2400_scu_reset_video(void)
{
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_VIDEO, AST_SCU_RESET);
	udelay(100);
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_VIDEO, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_reset_video);

extern void
ast2400_scu_init_video(void)
{
	//Video Engine Clock Enable and Reset
	//  Enable Clock & ECLK = inverse of (M-PLL / 2)
	ast2400_scu_write((ast2400_scu_read(AST_SCU_CLK_SEL) & ~SCU_ECLK_SOURCE_MASK) | SCU_ECLK_SOURCE(2), AST_SCU_CLK_SEL);
	// Enable CLK
	ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) & ~(SCU_ECLK_STOP_EN | SCU_VCLK_STOP_EN), AST_SCU_CLK_STOP);	
	mdelay(10);
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_VIDEO, AST_SCU_RESET);
	udelay(100);
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_VIDEO, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_init_video);

extern void
ast2400_scu_init_eth(u8 num)
{
	//eth 0 
	switch(num) {
		case 0:
			ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_MAC0, 
							AST_SCU_RESET);		
			udelay(100);
			ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) & ~SCU_MAC0CLK_STOP_EN, 
							AST_SCU_CLK_STOP);		
			udelay(1000);
			ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_MAC0, 
							AST_SCU_RESET);		
			
			break;
		case 1:
			ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_MAC1, 
							AST_SCU_RESET);			
			udelay(100);
			ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) & ~SCU_MAC1CLK_STOP_EN, 
							AST_SCU_CLK_STOP);		
			udelay(1000);
			ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_MAC1, 
							AST_SCU_RESET);			
			break;
			
	}		
}


extern void
ast2400_scu_init_ehci(void)
{
	/* EHCI controller engine init. Process similar to VHub. */
	/* Following reset sequence can resolve "vhub dead on first power on" issue on V4 board. */
	//reset USB20
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_USB20, AST_SCU_RESET);

	//enable USB20 clock
	ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) | SCU_USB20_CLK_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_USB20, AST_SCU_RESET);

	udelay(500);

//	printk("%x \n",ast2400_scu_read(AST_SCU_RESET));


}

EXPORT_SYMBOL(ast2400_scu_init_ehci);

extern void
ast2400_scu_init_uhci(void)
{
	//USB1.1 Host's Clock Enable and Reset
	ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) & ~SCU_USB11CLK_STOP_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_USB11, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_init_uhci);

///
extern void
ast2400_scu_init_sdhci(void)
{
	//SDHCI Host's Clock Enable and Reset
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_SD, AST_SCU_RESET);
	
	ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) & ~SCU_SDCLK_STOP_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_SEL) | SCU_CLK_SD_EN, AST_SCU_CLK_SEL);
	mdelay(10);

	// SDCLK = H-PLL / 4
	ast2400_scu_write((ast2400_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_SD_MASK) | SCU_CLK_SD_DIV(1), 
		AST_SCU_CLK_SEL);
	mdelay(10);
	
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_SD, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_init_sdhci);

extern void
ast2400_scu_init_i2c(void)
{
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_I2C, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_init_i2c);

extern void
ast2400_scu_init_pwm_tacho(void)
{
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_PWM, AST_SCU_RESET);
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_PWM, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_init_pwm_tacho);

extern void
ast2400_scu_init_adc(void)
{
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_ADC, AST_SCU_RESET);
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_ADC, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_init_adc);

extern void
ast2400_scu_init_peci(void)
{
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_PECI, AST_SCU_RESET);
	udelay(3);
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_PECI, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast2400_scu_init_peci);

extern void
ast2400_scu_init_lpc(void)
{
	//Note .. It have been enable in U-boot..... 
//	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_LPC, AST_SCU_RESET);

	//enable LPC clock LHCLK = H-PLL/8
	ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) | 
					SCU_SET_LHCLK_DIV(3) | 
					SCU_LHCLK_SOURCE_EN, 
					AST_SCU_CLK_STOP);

}

EXPORT_SYMBOL(ast2400_scu_init_lpc);

extern void
ast2400_scu_init_crt(void)
{
	//enable D2 pll , //enable DVO (bit18) is VGA , enable DAC (bit16) is CRT
#if defined(CONFIG_AST_DAC) || defined(CONFIG_AST_DVO)
	ast2400_scu_write((ast2400_scu_read(AST_SCU_MISC1_CTRL) & ~(SCU_MISC_D2_PLL_DIS | SCU_MISC_DAC_MASK))
				| SCU_MISC_DAC_SOURCE_CRT | SCU_MISC_DVO_SOURCE_CRT | SCU_MISC_2D_CRT_EN , AST_SCU_MISC1_CTRL);
#elif defined(CONFIG_AST_DVO) 
	ast2400_scu_write((ast2400_scu_read(AST_SCU_MISC1_CTRL) & ~(SCU_MISC_D2_PLL_DIS)) |
				SCU_MISC_DVO_SOURCE_CRT| SCU_MISC_2D_CRT_EN, AST_SCU_MISC1_CTRL);
#else //default(CONFIG_AST_DAC) 
	ast2400_scu_write((ast2400_scu_read(AST_SCU_MISC1_CTRL) & ~(SCU_MISC_D2_PLL_DIS | SCU_MISC_DAC_MASK))
				| SCU_MISC_DAC_SOURCE_CRT | SCU_MISC_2D_CRT_EN, AST_SCU_MISC1_CTRL);
#endif
	/* Set Delay 5 Compensation TODO ...*/
	ast2400_scu_write((ast2400_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_VIDEO_DELAY_MASK) |
						SCU_CLK_VIDEO_DELAY(5), AST_SCU_CLK_SEL);

	/* Reset CRT */ 
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) | SCU_RESET_CRT, AST_SCU_RESET);

	//enable D2 CLK
	ast2400_scu_write(ast2400_scu_read(AST_SCU_CLK_STOP) & ~SCU_D2CLK_STOP_EN , AST_SCU_CLK_STOP);

	udelay(10);
	ast2400_scu_write(ast2400_scu_read(AST_SCU_RESET) & ~SCU_RESET_CRT, AST_SCU_RESET);
	
}

EXPORT_SYMBOL(ast2400_scu_init_crt);
//***********************************CLK control***********************************
extern void
ast2400_scu_clk_stop(u32 clk_name,u8 stop_enable)
{
	switch(clk_name){
		default:
			SCUMSG("ERRO clk_name :%d \n",clk_name);
			break;
	}
}

EXPORT_SYMBOL(ast2400_scu_clk_stop);


//***********************************CLK Information***********************************
extern u32
ast2400_get_clk_source(void)
{
	if(ast2400_scu_read(AST_SCU_HW_STRAP1) & CLK_25M_IN)
		return AST_PLL_25MHZ;
	else
		return AST_PLL_24MHZ;
}

EXPORT_SYMBOL(ast2400_get_clk_source);


extern u32
ast2400_get_h_pll_clk(void)
{
	u32 speed,clk=0;
	u32 h_pll_set = ast2400_scu_read(AST_SCU_H_PLL);

	if(h_pll_set & SCU_H_PLL_OFF)
		return 0;
	
	if(h_pll_set & SCU_H_PLL_PARAMETER) {
		// Programming
		clk = ast2400_get_clk_source();
		if(h_pll_set & SCU_H_PLL_BYPASS_EN) {
			return clk;
		} else {
			//OD == SCU24[4]
			//OD = SCU_H_PLL_GET_DIV(h_pll_set);
			//Numerator == SCU24[10:5]
			//num = SCU_H_PLL_GET_NUM(h_pll_set);
			//Denumerator == SCU24[3:0]
			//denum = SCU_H_PLL_GET_DENUM(h_pll_set);

			//hpll = 24MHz * (2-OD) * ((Numerator+2)/(Denumerator+1))
			clk = ((clk * (2-SCU_H_PLL_GET_DIV(h_pll_set)) * (SCU_H_PLL_GET_NUM(h_pll_set)+2))/(SCU_H_PLL_GET_DENUM(h_pll_set)+1));
		}
	} else {
		// HW Trap
		speed = SCU_HW_STRAP_GET_H_PLL_CLK(ast2400_scu_read(AST_SCU_HW_STRAP1));
		switch (speed) {
			case 0:
				clk = 384000000; 
				break;
			case 1:
				clk = 360000000; 
				break;
			case 2:
				clk = 336000000; 
				break;
			case 3:
				clk = 408000000; 
				break;
			default:
				BUG(); 
				break;
		}		
	}
	SCUDBUG("h_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast2400_get_h_pll_clk);

extern u32
ast2400_get_m_pll_clk(void)
{
	u32 clk=0;
	u32 m_pll_set = ast2400_scu_read(AST_SCU_M_PLL);

	if(m_pll_set & SCU_M_PLL_OFF)
		return 0;
	
	// Programming
	clk = ast2400_get_clk_source();
	if(m_pll_set & SCU_M_PLL_BYPASS_EN) {
		return clk;
	} else {
		//OD == SCU24[4]
		//OD = SCU_M_PLL_GET_DIV(h_pll_set);
		//Numerator == SCU24[10:5]
		//num = SCU_M_PLL_GET_NUM(h_pll_set);
		//Denumerator == SCU24[3:0]
		//denum = SCU_M_PLL_GET_DENUM(h_pll_set);

		//hpll = 24MHz * (2-OD) * ((Numerator+2)/(Denumerator+1))
		clk = (clk * (2-SCU_M_PLL_GET_DIV(m_pll_set)) * ((SCU_M_PLL_GET_NUM(m_pll_set)+2)/(SCU_M_PLL_GET_DENUM(m_pll_set)+1)));
	}
	SCUDBUG("m_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast2400_get_m_pll_clk);

extern u32
ast2400_get_ahbclk(void)
{
	unsigned int div, hpll;

	hpll = ast2400_get_h_pll_clk();
	div = SCU_HW_STRAP_GET_CPU_AHB_RATIO(ast2400_scu_read(AST_SCU_HW_STRAP1));
	switch(div) {
		case 0:
			div = 1;
		break;
		case 1:
			div = 2;
		break;
		case 2:
			div = 3;
		break;
		case 3:
			div = 4;
		break;
		
	}
	
	SCUDBUG("HPLL=%d, Div=%d, AHB CLK=%d\n", hpll, div, hpll/div);	
	return (hpll/div);

}
EXPORT_SYMBOL(ast2400_get_ahbclk);

extern u32
ast2400_get_pclk(void)
{
	unsigned int div, hpll;

	hpll = ast2400_get_h_pll_clk();
	div = SCU_GET_PCLK_DIV(ast2400_scu_read(AST_SCU_CLK_SEL));
	div = (div+1) << 1;
	
	SCUDBUG("HPLL=%d, Div=%d, PCLK=%d\n", hpll, div, hpll/div);	
	return (hpll/div);

}
EXPORT_SYMBOL(ast2400_get_pclk);

extern u32
ast2400_get_lhclk(void)
{
	unsigned int div, hpll;
	u32 clk_sel = ast2400_scu_read(AST_SCU_CLK_SEL);
//FPGA AST1070 is default 100/2 Mhz input
//	return 50000000;	
	hpll = ast2400_get_h_pll_clk();
	if(SCU_LHCLK_SOURCE_EN & clk_sel) {
		div = SCU_GET_LHCLK_DIV(clk_sel);
		switch(div) {
			case 0:
				div = 2;
			break;
			case 1:
				div = 4;
			break;
			case 2:
				div = 6;
			break;
			case 3:
				div = 8;
			break;
			case 4:
				div = 10;
			break;
			case 5:
				div = 12;
			break;
			case 6:
				div = 14;
			break;
			case 7:
				div = 16;
			break;			
		}
		
		SCUDBUG("HPLL=%d, Div=%d, LHCLK = %d\n", hpll, div, hpll/div);	
		return (hpll/div);
	} else {
		SCUMSG("LPC CLK not enable \n");
		return 0;
	}

}

EXPORT_SYMBOL(ast2400_get_lhclk);

extern u32
ast2400_get_d2_pll_clk(void)
{
	u32 clk=0;
	u32 d2_pll_set = ast2400_scu_read(AST_SCU_D2_PLL);
	u32 OD,NUM,DENUM,PD,PD2;

	if(d2_pll_set & SCU_D2_PLL_OFF)
		return 0;

	// Programming
	clk = ast2400_get_clk_source();
	if(d2_pll_set & SCU_D2_PLL_BYPASS_EN) {
		return clk;
	} else {
		NUM = SCU_D2_PLL_GET_NUM(d2_pll_set);
		DENUM = SCU_D2_PLL_GET_DENUM(d2_pll_set);
		OD = SCU_D2_PLL_GET_OD(d2_pll_set);
		OD = (1 << (OD - 1));
		PD = SCU_D2_PLL_GET_PD(d2_pll_set);
		switch(PD) {
			case 0:
				PD = 1;
				break;
			case 1:
				PD = 2;
				break;
			case 2:
				PD = 2;
				break;
			case 3:
				PD = 4;
				break;
		}
		PD2 = SCU_D2_PLL_GET_PD2(d2_pll_set);
		PD2 = PD2+1;
//		printk("clk %d ,num %d ,denum %d ,od %d ,pd %d ,pd2 %d \n",clk, NUM , DENUM, OD, PD, PD2);
		//hpll = 24MHz * (Numerator * 2) / (Denumerator * OD * PD * PD2)
		clk = (clk * NUM * 2) / (DENUM* OD * PD * PD2);
	}

	SCUDBUG("d2_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast2400_get_d2_pll_clk);

//Because value 0 is not allowed in SDIO12C D[15:8]: Host Control Settings #1 Register, we have to increase the maximum
//host's clock in case that system will not ask host to set 1 in the sdhci_set_clock() function
/*
SCU7C: Silicon Revision ID Register
D[31:24]: Chip ID
0: AST2050/AST2100/AST2150/AST2200/AST3000
1: AST2300

D[23:16] Silicon revision ID for AST2300 generation and later
0: A0
1: A1
2: A2
.
.
.
FPGA revision starts from 0x80


D[11:8] Bounding option

D[7:0] Silicon revision ID for AST2050/AST2100 generation (for software compatible)
0: A0
1: A1
2: A2
3: A3
.
.
FPGA revision starts from 0x08, 8~10 means A0, 11+ means A1, AST2300 should be assigned to 3
*/

extern u32
ast2400_get_sd_clock_src(void)
{
	u32 clk=0, sd_div;

#if defined(FPGA)
		clk = 100000000;
#else
		clk = ast2400_get_h_pll_clk();
		//get div
		sd_div = SCU_CLK_SD_GET_DIV(ast2400_scu_read(AST_SCU_CLK_SEL));
		SCUDBUG("div %d, sdclk =%d \n",((sd_div + 1) * 2),clk/((sd_div + 1) * 2));
		clk /= ((sd_div + 1) * 2);
		
#endif
	return clk;
}

EXPORT_SYMBOL(ast2400_get_sd_clock_src);

extern void
ast2400_scu_show_system_info (void)
{
	u32 h_pll, div;

	h_pll = ast2400_get_h_pll_clk();
	
	div = SCU_HW_STRAP_GET_CPU_AHB_RATIO(ast2400_scu_read(AST_SCU_HW_STRAP1));
	switch(div) {
		case 0:
			div = 1;
		break;
		case 1:
			div = 2;
		break;
		case 2:
			div = 3;
		break;
		case 3:
			div = 4;
		break;
		
	}

	SCUMSG("CPU = %d MHz ,AHB = %d MHz (%d:1) \n", h_pll/1000000, h_pll/div/1000000,div); 

	return ;
}

EXPORT_SYMBOL(ast2400_scu_show_system_info);

//*********************************** Multi-function pin control ***********************************
extern void
ast2400_scu_multi_func_uart(u8 uart)
{
	switch(uart) {
		case 1:
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_UART1_RXD |
						SCU_FUN_PIN_UART1_TXD |
						SCU_FUN_PIN_UART1_NRTS |
						SCU_FUN_PIN_UART1_NDTR |
						SCU_FUN_PIN_UART1_NRI |
						SCU_FUN_PIN_UART1_NDSR |
						SCU_FUN_PIN_UART1_NDCD |
						SCU_FUN_PIN_UART1_NCTS, 
				AST_SCU_FUN_PIN_CTRL1); 
			break;		
		case 2:
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_UART2_RXD |
						SCU_FUN_PIN_UART2_TXD |
						SCU_FUN_PIN_UART2_NRTS |
						SCU_FUN_PIN_UART2_NDTR |
						SCU_FUN_PIN_UART2_NRI |
						SCU_FUN_PIN_UART2_NDSR |
						SCU_FUN_PIN_UART2_NDCD |
						SCU_FUN_PIN_UART2_NCTS, 
				AST_SCU_FUN_PIN_CTRL1); 
			break;		
		case 3:
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_UART3_RXD |
						SCU_FUN_PIN_UART3_TXD |
						SCU_FUN_PIN_UART3_NRTS |
						SCU_FUN_PIN_UART3_NDTR |
						SCU_FUN_PIN_UART3_NRI |
						SCU_FUN_PIN_UART3_NDSR |
						SCU_FUN_PIN_UART3_NDCD |
						SCU_FUN_PIN_UART3_NCTS, 
				AST_SCU_FUN_PIN_CTRL1); 
			break;
		case 4:
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_UART4_RXD |
						SCU_FUN_PIN_UART4_TXD |
						SCU_FUN_PIN_UART4_NRTS |
						SCU_FUN_PIN_UART4_NDTR |
						SCU_FUN_PIN_UART4_NRI |
						SCU_FUN_PIN_UART4_NDSR |
						SCU_FUN_PIN_UART4_NDCD |
						SCU_FUN_PIN_UART4_NCTS, 
				AST_SCU_FUN_PIN_CTRL1); 			
			break;
	}


}

extern void
ast2400_scu_multi_func_video()
{
#if defined(CONFIG_ARCH_2100) || defined(CONFIG_ARCH_2200)
	ast2400_scu_write(ast2400_scu_read(AST_SCU_MULTI_FUNC_2) |
				MULTI_FUNC_VIDEO_RGB18 |
				MULTI_FUNC_VIDEO_SINGLE_EDGE, 
		AST_SCU_MULTI_FUNC_2); 
#elif defined(CONFIG_ARCH_1100) || defined(CONFIG_ARCH_2050)
	ast2400_scu_write(ast2400_scu_read(AST_SCU_MULTI_FUNC_2) |
				MULTI_FUNC_VIDEO_RGB18 |
				MULTI_FUNC_VIDEO_SINGLE_EDGE, 
		AST_SCU_MULTI_FUNC_2); 
#else

#endif
}

extern void
ast2400_scu_multi_func_eth(u8 num)
{
	switch(num) {
		case 0:
#ifdef CONFIG_MAC0_PHY_LINK			
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) | 
						SCU_FUN_PIN_MAC0_PHY_LINK, 
				AST_SCU_FUN_PIN_CTRL1); 
#else
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) &
						~SCU_FUN_PIN_MAC0_PHY_LINK, 
				AST_SCU_FUN_PIN_CTRL1); 

#endif			
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL3) | 
						SCU_FUN_PIN_MAC0_MDIO |
						SCU_FUN_PIN_MAC0_MDC, 
				AST_SCU_FUN_PIN_CTRL3); 
			
			break;
		case 1:
#ifdef CONFIG_MAC0_PHY_LINK						
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) | 
						SCU_FUN_PIN_MAC1_PHY_LINK, 
				AST_SCU_FUN_PIN_CTRL1); 
#else
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL1) & 
					~SCU_FUN_PIN_MAC1_PHY_LINK, 
				AST_SCU_FUN_PIN_CTRL1); 
#endif
			ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | 
						SCU_FUC_PIN_MAC1_MDIO,
				AST_SCU_FUN_PIN_CTRL5); 

			break;
	}
}

extern void
ast2400_scu_multi_func_nand(void)
{
	//enable NAND flash multipin FLBUSY and FLWP
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL2) | 
				SCU_FUN_PIN_NAND_FLBUSY | SCU_FUN_PIN_NAND_FLWP, 
		AST_SCU_FUN_PIN_CTRL2); 

}

extern void
ast2400_scu_multi_func_nor(void)
{
	//Address 
	//ROMA2~17
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL8) | 
				SCU_FUN_PIN_ROMA2 | SCU_FUN_PIN_ROMA3 |
				SCU_FUN_PIN_ROMA4 | SCU_FUN_PIN_ROMA5 |
				SCU_FUN_PIN_ROMA6 | SCU_FUN_PIN_ROMA7 |
				SCU_FUN_PIN_ROMA8 | SCU_FUN_PIN_ROMA9 |
				SCU_FUN_PIN_ROMA10 | SCU_FUN_PIN_ROMA11 |
				SCU_FUN_PIN_ROMA12 | SCU_FUN_PIN_ROMA13 |
				SCU_FUN_PIN_ROMA14 | SCU_FUN_PIN_ROMA15 |
				SCU_FUN_PIN_ROMA16 | SCU_FUN_PIN_ROMA17, 
		AST_SCU_FUN_PIN_CTRL8);

	//ROMA18~21
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL9) | 
				SCU_FUN_PIN_ROMA18 | SCU_FUN_PIN_ROMA19 |
				SCU_FUN_PIN_ROMA20 | SCU_FUN_PIN_ROMA21, 
		AST_SCU_FUN_PIN_CTRL9);	
	
	//ROMA22,23
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL4) | SCU_FUN_PIN_ROMA22 | SCU_FUN_PIN_ROMA23, 
		AST_SCU_FUN_PIN_CTRL4);
	
	//ROMA24,25
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL3) | SCU_FUN_PIN_ROMA24 | SCU_FUN_PIN_ROMA25, 
		AST_SCU_FUN_PIN_CTRL3);

	//SCU94 [1] = 0
 	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL6) & SCU_VIDEO_OUT_MASK, 
		AST_SCU_FUN_PIN_CTRL6);

	
	//data
	//ROMD 4~7 //ROMWE#, OE#
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL4) | 
			SCU_FUN_PIN_ROMOE | SCU_FUN_PIN_ROMWE |
			SCU_FUN_PIN_ROMD4 | SCU_FUN_PIN_ROMD5 |
			SCU_FUN_PIN_ROMD6 | SCU_FUN_PIN_ROMD7,
			AST_SCU_FUN_PIN_CTRL4);
	
	//ROMD 8~15
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | 
			SCU_FUC_PIN_ROM_16BIT, 
		AST_SCU_FUN_PIN_CTRL5);

}

extern void
ast2400_scu_multi_func_romcs(u8 num)
{
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL3) | 
			SCU_FUN_PIN_ROMCS(num), 
		AST_SCU_FUN_PIN_CTRL3);
}

extern void
ast2400_scu_multi_func_i2c(void)
{
	//TODO check ... //In AST2400 Due to share pin with SD , please not enable I2C 10 ~14 
	// AST 2400 have 14 , AST 2300 9 ...
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | 
			SCU_FUC_PIN_I2C3 | 
			SCU_FUC_PIN_I2C4 | 
			SCU_FUC_PIN_I2C5 | 
			SCU_FUC_PIN_I2C6 | 
			SCU_FUC_PIN_I2C7 | 
			SCU_FUC_PIN_I2C8 | 
			SCU_FUC_PIN_I2C9 | 
			SCU_FUC_PIN_I2C10 | 
			SCU_FUC_PIN_I2C11 | 
			SCU_FUC_PIN_I2C12 | 
			SCU_FUC_PIN_I2C13 | 
			SCU_FUC_PIN_I2C14, 
		AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast2400_scu_multi_func_i2c);

extern void
ast2400_scu_multi_func_pwm_tacho(void)
{
	//TODO check
	u32 sts = ast2400_scu_read(AST_SCU_FUN_PIN_CTRL3) &~0xcfffff;
	ast2400_scu_write(sts | 0xc000ff, AST_SCU_FUN_PIN_CTRL3);
}	

EXPORT_SYMBOL(ast2400_scu_multi_func_pwm_tacho);

//0 : hub mode , 1: usb host mode
extern void
ast2400_scu_multi_func_usb20_host_hub(u8 mode)
{
	if(mode)
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB20_HOST, 
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB20_HOST, 
					AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast2400_scu_multi_func_usb20_host_hub);

//0 : gpioQ6,7 mode , 1: usb1.1 host port 4 mode
extern void
ast2400_scu_multi_func_usb11_host_port4(u8 mode)
{
	if(mode)
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT4, 
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT4, 
					AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast2400_scu_multi_func_usb11_host_port4);

//0 : USB 1.1 HID mode , 1: usb1.1 host port 2 mode
extern void
ast2400_scu_multi_func_usb11_host_port2(u8 mode)
{
	if(mode)
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT2, 
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT2, 
					AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast2400_scu_multi_func_usb11_host_port2);

//0 : 1: SD1 function 
extern void
ast2400_scu_multi_func_sdhc_slot1(u8 mode)
{
	if(mode)
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD1, 
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_SD1, 
					AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast2400_scu_multi_func_sdhc_slot1);

extern void
ast2400_scu_multi_func_sdhc_slot2(u8 mode)
{
	if(mode)
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD2, 
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_SD2, 
					AST_SCU_FUN_PIN_CTRL5);

}	

EXPORT_SYMBOL(ast2400_scu_multi_func_sdhc_slot2);

extern void
ast2400_scu_multi_func_crt(void)
{
	/* multi-pin for DVO */

	//Digital vodeo input function pins : 00 disable, 10 24bits mode 888,
	ast2400_scu_write((ast2400_scu_read(AST_SCU_FUN_PIN_CTRL5) &
			~SCU_FUC_PIN_DIGI_V_OUT_MASK) | 
			SCU_FUC_PIN_DIGI_V_OUT(VIDEO_24BITS),AST_SCU_FUN_PIN_CTRL5);

	//VPI input
#if 0	
	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL2) |
			SCU_FUN_PIN_VPIB9 | SCU_FUN_PIN_VPIB8 |
			SCU_FUN_PIN_VPIB7 | SCU_FUN_PIN_VPIB6 |
			SCU_FUN_PIN_VPIB5 | SCU_FUN_PIN_VPIB4 |
			SCU_FUN_PIN_VPIB3 | SCU_FUN_PIN_VPIB2 |
			SCU_FUN_PIN_VPIB1 | SCU_FUN_PIN_VPIB0 |
			SCU_FUN_PIN_VPICLK | SCU_FUN_PIN_VPIVS |
			SCU_FUN_PIN_VPIHS | SCU_FUN_PIN_VPIODD |
			SCU_FUN_PIN_VPIDE ,AST_SCU_FUN_PIN_CTRL2);

	ast2400_scu_write(ast2400_scu_read(AST_SCU_FUN_PIN_CTRL3) |
			SCU_FUN_PIN_VPIR9 | SCU_FUN_PIN_VPIR8 |
			SCU_FUN_PIN_VPIR7 | SCU_FUN_PIN_VPIR6 |	
			SCU_FUN_PIN_VPIR5 | SCU_FUN_PIN_VPIR4 |
			SCU_FUN_PIN_VPIR3 | SCU_FUN_PIN_VPIR2 |
			SCU_FUN_PIN_VPIR1 | SCU_FUN_PIN_VPIR0 |
			SCU_FUN_PIN_VPIG9 | SCU_FUN_PIN_VPIG8 |	
			SCU_FUN_PIN_VPIG7 | SCU_FUN_PIN_VPIG6 |
			SCU_FUN_PIN_VPIG5 | SCU_FUN_PIN_VPIG4 |
			SCU_FUN_PIN_VPIG3 | SCU_FUN_PIN_VPIG2 |
			SCU_FUN_PIN_VPIG1 | SCU_FUN_PIN_VPIG0 ,AST_SCU_FUN_PIN_CTRL3);
#endif
}

EXPORT_SYMBOL(ast2400_scu_multi_func_crt);
//***********************************Information ***********************************
extern u32
ast2400_scu_revision_id(void)
{
	int i;
	u32 rev_id = ast2400_scu_read(AST_SCU_REVISION_ID);
	for(i=0;i<ARRAY_SIZE(soc_map_table);i++) {
		if(rev_id == soc_map_table[i].rev_id)
			break;
	}
	if(i == ARRAY_SIZE(soc_map_table))
		SCUMSG("SOC : %x \n",rev_id);
	else
		SCUMSG("SOC : %4s \n",soc_map_table[i].name);
	
	return rev_id;
}	

EXPORT_SYMBOL(ast2400_scu_revision_id);

extern u32
ast2400_scu_get_phy_config(u8 mac_num)
{
	u32 scatch = ast2400_scu_read(AST_SCU_SOC_SCRATCH0);

	switch(mac_num) {
		case 0:
			return (SCU_MAC0_GET_PHY_MODE(scatch));
			break;
		case 1:
			return (SCU_MAC1_GET_PHY_MODE(scatch));
			break;
		default:
			SCUMSG("error mac number \n");
			break;
	}
	return -1;
}
EXPORT_SYMBOL(ast2400_scu_get_phy_config);

extern u32
ast2400_scu_get_phy_interface(u8 mac_num)
{
	u32 trap1 = ast2400_scu_read(AST_SCU_HW_STRAP1);

	switch(mac_num) {
		case 0:
			if(SCU_HW_STRAP_MAC0_INF & trap1)
				return 1;
			else
				return 0;
			break;
		case 1:
			if(SCU_HW_STRAP_MAC1_INF & trap1)
				return 1;
			else
				return 0;
			break;
		default:
			SCUMSG("error mac number \n");
			break;
	}
	return -1;
}
EXPORT_SYMBOL(ast2400_scu_get_phy_interface);

extern void
ast2400_scu_set_vga_display(u8 enable)
{
	if(enable)
		printk("111111");
}

EXPORT_SYMBOL(ast2400_scu_set_vga_display);

extern u32
ast2400_scu_get_vga_memsize(void)
{
	u32 size=0;

	switch(SCU_HW_STRAP_VGA_SIZE_GET(ast2400_scu_read(AST_SCU_HW_STRAP1))) {
		case 0:
			size = 8*1024*1024;
			break;
		case 1:
			size = 16*1024*1024;
			break;
		case 2:
			size = 32*1024*1024;
			break;
		case 3:
			size = 64*1024*1024;
			break;
		default:
			SCUMSG("error vga size \n");
			break;
	}
	return size;
}

EXPORT_SYMBOL(ast2400_scu_get_vga_memsize);

extern void
ast2400_scu_get_who_init_dram(void)
{
	switch(SCU_VGA_DRAM_INIT_MASK(ast2400_scu_read(AST_SCU_VGA0))) {
		case 0:
			SCUMSG("VBIOS init \n");
			break;
		case 1:
			SCUMSG("SOC init \n");
			break;
		default:
			SCUMSG("error vga size \n");
			break;
	}
}
