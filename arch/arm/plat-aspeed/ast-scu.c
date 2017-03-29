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
#include <linux/interrupt.h>

#include <mach/platform.h>
#include <asm/io.h>

#include <mach/hardware.h>

#include <plat/ast-scu.h>
#include <plat/regs-scu.h>

//#define ASPEED_SCU_LOCK
//#define ASPEED_SCU_DEBUG

#ifdef ASPEED_SCU_DEBUG
#define SCUDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SCUDBUG(fmt, args...)
#endif

#define SCUMSG(fmt, args...) printk(fmt, ## args)

static u32 ast_scu_base = IO_ADDRESS(AST_SCU_BASE);

spinlock_t ast_scu_lock;

static inline u32
ast_scu_read(u32 reg)
{
	u32 val;

	val = readl((void *)(ast_scu_base + reg));

	SCUDBUG("ast_scu_read : reg = 0x%08x, val = 0x%08x\n", reg, val);

	return val;
}

static inline void
ast_scu_write(u32 val, u32 reg)
{
	SCUDBUG("ast_scu_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
#ifdef CONFIG_AST_SCU_LOCK
	//unlock
	writel(SCU_PROTECT_UNLOCK, (void *)ast_scu_base);
	writel(val, (void *) (ast_scu_base + reg));
	//lock
	writel(0xaa,ast_scu_base);
#else
	writel(SCU_PROTECT_UNLOCK, (void *) ast_scu_base);
	writel(val, (void *) (ast_scu_base + reg));
#endif
}

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
	[13] = {
		.name = "AST1010-A0",
		.rev_id = 0x03000003,
	},
	[14] = {
		.name = "AST1010-A1",
		.rev_id = 0x03010003,
	},
	[15] = {
		.name = "AST1520-A0",
		.rev_id = 0x03000203,
	},
	[16] = {
		.name = "AST3200-A0",
		.rev_id = 0x03000303,
	},
	[17] = {
		.name = "AST2500-A0",
		.rev_id = 0x04000303,
	},
	[18] = {
		.name = "AST2510-A0",
		.rev_id = 0x04000103,
	},
	[19] = {
		.name = "AST2520-A0",
		.rev_id = 0x04000203,
	},
	[20] = {
		.name = "AST2530-A0",
		.rev_id = 0x04000403,
	},
	[21] = {
		.name = "AST1520-A1",
		.rev_id = 0x03010203,
	},
	[22] = {
		.name = "AST3200-A1",
		.rev_id = 0x03010303,
	},
	[23] = {
		.name = "AST2500-A1",
		.rev_id = 0x04010303,
	},
	[24] = {
		.name = "AST2510-A1",
		.rev_id = 0x04010103,
	},
	[25] = {
		.name = "AST2520-A1",
		.rev_id = 0x04010203,
	},
	[26] = {
		.name = "AST2530-A1",
		.rev_id = 0x04010403,
	},
};
//***********************************Initial control***********************************
#ifdef SCU_RESET_VIDEO
extern void
ast_scu_reset_video(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_VIDEO, AST_SCU_RESET);
	udelay(100);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_VIDEO, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_reset_video);

extern void
ast_scu_init_video(u8 dynamic_en)
{
	//Video Engine Clock Enable and Reset
	//  Enable Clock & ECLK = inverse of (M-PLL / 2)
	if(dynamic_en)
		ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_VIDEO_SLOW_MASK) | SCU_CLK_VIDEO_SLOW_EN | SCU_CLK_VIDEO_SLOW_SET(0), AST_SCU_CLK_SEL);
	else {
		if(GET_CHIP_REVISION(ast_scu_read(AST_SCU_REVISION_ID)) == 4)
			ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_MASK | SCU_CLK_VIDEO_SLOW_EN)), AST_SCU_CLK_SEL);
		else
			ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_EN)) | SCU_ECLK_SOURCE(2), AST_SCU_CLK_SEL);
	}

	// Enable CLK
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~(SCU_ECLK_STOP_EN | SCU_VCLK_STOP_EN), AST_SCU_CLK_STOP);
	mdelay(10);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_VIDEO, AST_SCU_RESET);
	udelay(100);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_VIDEO, AST_SCU_RESET);
}
EXPORT_SYMBOL(ast_scu_init_video);
#endif

#ifdef SCU_UART1CLK_STOP_EN
extern void
ast_scu_init_uart(u8 uart)
{
	u32 clk_stop_en = 0;

	//uart 1
	if(uart & 0x2) {
		clk_stop_en |= SCU_UART1CLK_STOP_EN;
	}

	if(uart & 0x4) {
		clk_stop_en |= SCU_UART2CLK_STOP_EN;
	}

	if(uart & 0x8) {
		clk_stop_en |= SCU_UART3CLK_STOP_EN;
	}

	if(uart & 0x10) {
		clk_stop_en |= SCU_UART4CLK_STOP_EN;
	}

	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~(clk_stop_en), AST_SCU_CLK_STOP);

}
EXPORT_SYMBOL(ast_scu_init_uart);
#endif

extern void
ast_scu_init_eth(u8 num)
{

//Set MAC delay Timing
#if defined(AST_SOC_G5)
	//a1
//	ast_scu_write(0x00145249, AST_SCU_MAC_CLK);
//	ast_scu_write(0x00145249, AST_SCU_MAC_CLK_DELAY_100M);
//	ast_scu_write(0x00145249, AST_SCU_MAC_CLK_DELAY_10M);
//	ast_scu_write((0x6a << 16) | (0x6a << 8), AST_SCU_MAC_CLK_DUTY);
#elif defined(CONFIG_ARCH_AST1010)
// do nothing
#else
	//AST2300 max clk to 125Mhz, AST2400 max clk to 198Mhz
	if(ast_scu_read(AST_SCU_HW_STRAP1) & (SCU_HW_STRAP_MAC1_RGMII | SCU_HW_STRAP_MAC0_RGMII))	//RGMII --> H-PLL/6
		ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_MAC_MASK) | SCU_CLK_MAC_DIV(2), AST_SCU_CLK_SEL);
	else	//RMII --> H-PLL/10
		ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_MAC_MASK) | SCU_CLK_MAC_DIV(4), AST_SCU_CLK_SEL);

	ast_scu_write(0x2255, AST_SCU_MAC_CLK);
#endif

	switch(num) {
		case 0:
			ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_MAC0,
							AST_SCU_RESET);
			udelay(100);
			ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_MAC0CLK_STOP_EN,
							AST_SCU_CLK_STOP);
			udelay(1000);
			ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_MAC0,
							AST_SCU_RESET);

			break;
#if defined(AST_MAC1_BASE)
		case 1:
			ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_MAC1,
							AST_SCU_RESET);
			udelay(100);
			ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_MAC1CLK_STOP_EN,
							AST_SCU_CLK_STOP);
			udelay(1000);
			ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_MAC1,
							AST_SCU_RESET);
			break;
#endif
	}
}

#ifdef SCU_RESET_USB11
extern void
ast_scu_init_uhci(void)
{
	//USB1.1 Host's Clock Enable and Reset
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_USB11CLK_STOP_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_USB11, AST_SCU_RESET);
}
EXPORT_SYMBOL(ast_scu_init_uhci);
#endif

#ifdef SCU_RESET_USB20
extern void
ast_scu_init_usb_port1(void)
{
	/* EHCI controller engine init. Process similar to VHub. */
	/* Following reset sequence can resolve "vhub dead on first power on" issue on V4 board. */
	//reset USB20
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_USB20, AST_SCU_RESET);

	//enable USB20 clock
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) | SCU_USB20_PHY_CLK_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_USB20, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_init_usb_port1);

extern void
ast_scu_init_usb_port2()
{
#ifdef AST_SOC_G5
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_USB_P1, AST_SCU_RESET);

	//enable USB20 clock
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_USB_P1_STOP_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_USB_P1, AST_SCU_RESET);
#endif
}
EXPORT_SYMBOL(ast_scu_init_usb_port2);
#endif

#ifdef SCU_RESET_SD
extern void
ast_scu_init_sdhci(void)
{
	//SDHCI Host's Clock Enable and Reset
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_SD, AST_SCU_RESET);

	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_SDCLK_STOP_EN, AST_SCU_CLK_STOP);
	mdelay(10);

	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL) | SCU_CLK_SD_EN, AST_SCU_CLK_SEL);
	mdelay(10);

#ifdef CONFIG_ARCH_AST3200
	// SDCLK = H-PLL / 12
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_SD_MASK) | SCU_CLK_SD_DIV(7),
		AST_SCU_CLK_SEL);
#else
	// SDCLK = G4  H-PLL / 4, G5 = H-PLL /8
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_SD_MASK) | SCU_CLK_SD_DIV(1),
		AST_SCU_CLK_SEL);
#endif

	mdelay(10);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_SD, AST_SCU_RESET);
}
EXPORT_SYMBOL(ast_scu_init_sdhci);
#endif

extern void
ast_scu_init_i2c(void)
{
	spin_lock(&ast_scu_lock);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_I2C, AST_SCU_RESET);
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_I2C, AST_SCU_RESET);

	spin_unlock(&ast_scu_lock);
}

EXPORT_SYMBOL(ast_scu_init_i2c);

extern void
ast_scu_init_pwm_tacho(void)
{
	spin_lock(&ast_scu_lock);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PWM, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PWM, AST_SCU_RESET);

	spin_unlock(&ast_scu_lock);
}

EXPORT_SYMBOL(ast_scu_init_pwm_tacho);

extern void
ast_scu_init_adc(void)
{
	spin_lock(&ast_scu_lock);
#ifdef CONFIG_ARCH_AST1010
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL4) &
					~(SCU_FUN_PIN_GPI0 |
					SCU_FUN_PIN_GPI1 |
					SCU_FUN_PIN_GPI2 |
					SCU_FUN_PIN_GPI3 |
					SCU_FUN_PIN_GPI4 |
					SCU_FUN_PIN_GPI5 |
					SCU_FUN_PIN_GPI6 |
					SCU_FUN_PIN_GPI7),
			AST_SCU_FUN_PIN_CTRL4);
#endif
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_ADC, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_ADC, AST_SCU_RESET);
	spin_unlock(&ast_scu_lock);
}
EXPORT_SYMBOL(ast_scu_init_adc);

#ifdef SCU_RESET_PCIE
extern void
ast_scu_init_pcie(void)
{
	if((ast_scu_read(AST_SCU_RESET) & SCU_RESET_PCIE_DIR) && (!(ast_scu_read(AST_SCU_RESET) & SCU_RESET_PCIE))) {
		//do nothing
		//printk("No need init PCIe \n");
	} else {
		//pcie host reset
		ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PCIE, AST_SCU_RESET);
		ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PCIE_DIR, AST_SCU_RESET);
		ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PCIE, AST_SCU_RESET);
		mdelay(10);
		ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PCIE, AST_SCU_RESET);

		//p2x reset
		ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_P2X, AST_SCU_RESET);

		//use 0x7c for clr
		ast_scu_write(SCU_HW_STRAP_VGA_MASK, AST_SCU_REVISION_ID);
		ast_scu_write(SCU_HW_STRAP_VGA_SIZE_SET(VGA_64M_DRAM), AST_SCU_HW_STRAP1);

		ast_scu_write(ast_scu_read(AST_SCU_MISC2_CTRL) | SCU_PCIE_MAPPING_HIGH | SCU_MALI_RC_MODE | SCU_MALI_DTY_MODE, AST_SCU_MISC2_CTRL);
	}

}
EXPORT_SYMBOL(ast_scu_init_pcie);
#endif

#ifdef SCU_RESET_MCTP
extern void
ast_scu_init_mctp(void)
{
	//Notice : Must after Host reset
	spin_lock(&ast_scu_lock);
//	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_MCTP, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_MCTP, AST_SCU_RESET);
	spin_unlock(&ast_scu_lock);
}
EXPORT_SYMBOL(ast_scu_init_mctp);
#endif

#ifdef SCU_RESET_XDMA
extern void
ast_scu_init_xdma(void)
{
	//Notice : 1. Must after Host reset, 2. DRAM Controller 0x08 memory protection must disable [Protect REQ#] 3. VGA PCI Bus master enable offset 0x04[bit2]
	spin_lock(&ast_scu_lock);
//	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_XDMA, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_XDMA, AST_SCU_RESET);
	spin_unlock(&ast_scu_lock);
}
EXPORT_SYMBOL(ast_scu_init_xdma);
#endif

extern void
ast_scu_init_peci(void)
{
	spin_lock(&ast_scu_lock);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PECI, AST_SCU_RESET);
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PECI, AST_SCU_RESET);
	spin_unlock(&ast_scu_lock);
}

EXPORT_SYMBOL(ast_scu_init_peci);

extern void
ast_scu_init_jtag(void)
{
	spin_lock(&ast_scu_lock);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_JTAG, AST_SCU_RESET);
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_JTAG, AST_SCU_RESET);
	spin_unlock(&ast_scu_lock);
}

EXPORT_SYMBOL(ast_scu_init_jtag);

#ifdef SCU_RESET_HACE
extern void
ast_scu_init_hace(void)
{
	//enable YCLK for HAC
	spin_lock(&ast_scu_lock);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) &
					~(SCU_YCLK_STOP_EN | SCU_RSACLK_STOP_EN),
					AST_SCU_CLK_STOP);
	mdelay(1);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) &
					~SCU_RESET_HACE,
					AST_SCU_RESET);
	spin_unlock(&ast_scu_lock);
}
EXPORT_SYMBOL(ast_scu_init_hace);
#endif

extern void
ast_scu_reset_espi(void)
{
	//Note .. It have been enable in U-boot.....
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_ESPI, AST_SCU_RESET);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_ESPI, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_reset_espi);

extern void
ast_scu_reset_lpc(void)
{
	//Note .. It have been enable in U-boot.....
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_LPC, AST_SCU_RESET);

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_LPC, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_reset_lpc);

extern void
ast_scu_init_lpc(void)
{
	//Note .. It have been enable in U-boot.....
//	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_LPC, AST_SCU_RESET);

	//enable LPC clock LHCLK = H-PLL/8, SOC_G5 H-PLL/16
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) |
					SCU_SET_LHCLK_DIV(3) |
					SCU_LHCLK_SOURCE_EN,
					AST_SCU_CLK_STOP);
}

EXPORT_SYMBOL(ast_scu_init_lpc);

//////1 : lpc plus modes
extern u8
ast_scu_get_lpc_plus_enable(void)
{
	if(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & SCU_FUN_PIN_LPC_PLUS)
		return 1;
	else
		return 0;
}

EXPORT_SYMBOL(ast_scu_get_lpc_plus_enable);

#ifdef AST_SOC_G5
extern void
ast_scu_init_rfx(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_RFX, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET2) | (SCU_RESET_RFXDEC | SCU_RESET_RFXCMQ | SCU_RESET_BITBLT), AST_SCU_RESET2);

	//Use D1-PLL
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_MASK | SCU_CLK_VIDEO_SLOW_EN)), AST_SCU_CLK_SEL);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL) | SCU_ECLK_SOURCE(2), AST_SCU_CLK_SEL);

	ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_D_PLL_DIS) | SCU_MISC_D_PLL_SOURCE, AST_SCU_MISC1_CTRL);

	ast_scu_write(0x75402031, AST_SCU_D_PLL);
	ast_scu_write(0x00000580, AST_SCU_DPLL_PAR0);
	ast_scu_write(0x00000000, AST_SCU_DPLL_PAR1);
	ast_scu_write(0x0004AB1C, AST_SCU_DPLL_PAR2);

	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & 	~(SCU_RFX_CLK_STOP_EN), AST_SCU_CLK_STOP);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP2) & ~(SCU_CMQCLK_STOP | SCU_RFXCLK_STOP | SCU_BITBLTCLK_STOP), AST_SCU_CLK_STOP2);
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_RFX, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET2) & ~(SCU_RESET_RFXDEC | SCU_RESET_RFXCMQ | SCU_RESET_BITBLT), AST_SCU_RESET2);

	//Multi fun pin
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) | SCU_FUN_PIN_DVO_24BIT, AST_SCU_FUN_PIN_CTRL6);

}
#else
extern void
ast_scu_init_rfx(void)
{
}
#endif
EXPORT_SYMBOL(ast_scu_init_rfx);

#ifdef SCU_RESET_H264
extern void
ast_scu_init_h264(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_H264, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL2) & 	~SCU_H264CLK_MASK, AST_SCU_CLK_SEL2);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL2) | SCU_SET_H264CLK_DIV(3), AST_SCU_CLK_SEL2);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & 	~SCU_H264_STOP_EN, AST_SCU_CLK_STOP);
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_H264, AST_SCU_RESET);
}
EXPORT_SYMBOL(ast_scu_init_h264);
#endif

/* 0:disable spi 1: enable spi master 2:enable spi master and spi slave to ahb 3: enable spi pass-through*/
extern void
ast_scu_spi_master(u8 mode)
{
#ifdef AST_SOC_G5
	switch(mode) {
		case 0:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			break;
		case 1:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			ast_scu_write(SCU_HW_STRAP_SPI_MASTER, AST_SCU_HW_STRAP1);
			break;
		case 2:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			ast_scu_write(SCU_HW_STRAP_SPI_M_S_EN, AST_SCU_HW_STRAP1);
			break;
		case 3:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			ast_scu_write(SCU_HW_STRAP_SPI_PASS_THROUGH, AST_SCU_HW_STRAP1);
			break;
	}
#else
	switch(mode) {
		case 0:
			ast_scu_write(ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_HW_STRAP1);
			break;
		case 1:
			ast_scu_write((ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK) |SCU_HW_STRAP_SPI_MASTER, AST_SCU_HW_STRAP1);
			break;
		case 2:
			ast_scu_write((ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK) |SCU_HW_STRAP_SPI_MASTER, AST_SCU_HW_STRAP1);
			break;
		case 3:
			ast_scu_write((ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK) |SCU_HW_STRAP_SPI_PASS_THROUGH, AST_SCU_HW_STRAP1);
			break;
	}

#endif
}

EXPORT_SYMBOL(ast_scu_spi_master);

#ifdef SCU_RESET_CRT
extern void
ast_scu_init_crt(void)
{
	//ast2400 : VGA use D1 clk, CRT use D2 clk
	//ast2500 : VGA use D1 clk, CRT use 40Mhz
	//ast3200/ast1520 : VGA use D1 clk, CRT use D1/D2 clk select L: SCU08[bit 8] - H SCU2C[bit 21]

#ifdef AST_SOC_G5

#ifdef CONFIG_ARCH_AST3200
	//Select D2 CLK source 00:D-PLL, 01: D2-PLL, 1x : 40Mhz
	//H: 2c[bit : 21], L: 08[bit : 8]
	//Select D2-PLL parameter source [01]
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL) | SCU_CRT_CLK_L_SOURCE , AST_SCU_CLK_SEL);
	ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_CRT_CLK_H_SOURCE , AST_SCU_MISC1_CTRL);

	//Off D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) |  SCU_D2_PLL_OFF | SCU_D2_PLL_RESET , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x585, AST_SCU_D2_PLL_EXTEND);

	//set D2-PLL parameter
	ast_scu_write((0x15 << 27) | (0xE << 22) | (0x03D << 13) | (0x40), AST_SCU_D2_PLL);

	//enable D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) &  ~(SCU_D2_PLL_OFF | SCU_D2_PLL_RESET) , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x580, AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_CRT, AST_SCU_RESET);

	ast_scu_write(ast_scu_read(AST_SCU_RESET2) & ~(SCU_RESET_CRT0 | SCU_RESET_CRT1 | SCU_RESET_CRT2 | SCU_RESET_CRT3), AST_SCU_RESET2);

	//For DVO output timing
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL2) & SCU_VIDEO1_OUTPUT_CLK_DELAY_MASK) | SCU_VIDEO1_OUTPUT_CLK_DELAY(5), AST_SCU_CLK_SEL2);
#else
	//ast2500 use 40Mhz (init @ platform.S)
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_CRT, AST_SCU_RESET);

	ast_scu_write(ast_scu_read(AST_SCU_RESET2) & ~SCU_RESET_CRT0, AST_SCU_RESET2);
#endif

	//enable CRT CLK
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_D2CLK_STOP_EN , AST_SCU_CLK_STOP);

	ast_scu_write(0x1df, 0xd4);

#else
	//SOC VER < G5
	/* Enable D2 - PLL */
	ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_D2_PLL_DIS, AST_SCU_MISC1_CTRL);

	/* Reset CRT */
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_CRT, AST_SCU_RESET);

	/* Set Delay 5 Compensation TODO ...*/
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_VIDEO_DELAY_MASK) |
						SCU_CLK_VIDEO_DELAY(5), AST_SCU_CLK_SEL);

	//enable D2 CLK
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_D2CLK_STOP_EN , AST_SCU_CLK_STOP);

	udelay(10);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_CRT, AST_SCU_RESET);

#endif


}
EXPORT_SYMBOL(ast_scu_init_crt);
#endif

extern void
ast_scu_uartx_init(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP2) &
				~(SCU_UART_DIV13 | SCU_UARTXCLK_STOP),
		AST_SCU_CLK_STOP2);

}

EXPORT_SYMBOL(ast_scu_uartx_init);

//***********************************CLK control***********************************
extern void
ast_scu_uart_div(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP2) &
				~SCU_UART_DIV13,
		AST_SCU_CLK_STOP2);
}

EXPORT_SYMBOL(ast_scu_uart_div);

extern void
ast_scu_clk_stop(u32 clk_name,u8 stop_enable)
{
	switch(clk_name){
		default:
			SCUMSG("ERRO clk_name :%d \n",clk_name);
			break;
	}
}

EXPORT_SYMBOL(ast_scu_clk_stop);

//***********************************CLK Information***********************************
#ifdef CONFIG_ARCH_AST1010
extern u32
ast_get_clk_source(void)
{
	return AST_PLL_CLOCK;
}
#else
extern u32
ast_get_clk_source(void)
{
	if(ast_scu_read(AST_SCU_HW_STRAP1) & CLK_25M_IN)
		return AST_PLL_25MHZ;
	else
		return AST_PLL_24MHZ;
}
#endif
EXPORT_SYMBOL(ast_get_clk_source);

#if defined(AST_SOC_G5)
extern u32
ast_get_h_pll_clk(void)
{
	u32 clk=0;
	u32 h_pll_set = ast_scu_read(AST_SCU_H_PLL);

	if(h_pll_set & SCU_H_PLL_OFF)
		return 0;

	// Programming
	clk = ast_get_clk_source();
	if(h_pll_set & SCU_H_PLL_BYPASS_EN) {
		return clk;
	} else {
		//P = SCU24[18:13]
		//M = SCU24[12:5]
		//N = SCU24[4:0]
		//hpll = 24MHz * [(M+1) /(N+1)] / (P+1)
		clk = ((clk * (SCU_H_PLL_GET_MNUM(h_pll_set) + 1)) / (SCU_H_PLL_GET_NNUM(h_pll_set) + 1)) /(SCU_H_PLL_GET_PNUM(h_pll_set) + 1);
	}
	SCUDBUG("h_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast_get_h_pll_clk);

extern u32
ast_get_m_pll_clk(void)
{
	u32 clk=0;
	u32 m_pll_set = ast_scu_read(AST_SCU_M_PLL);

	if(m_pll_set & SCU_M_PLL_OFF)
		return 0;

	// Programming
	clk = ast_get_clk_source();
	if(m_pll_set & SCU_M_PLL_BYPASS) {
		return clk;
	} else {
		//PD  == SCU20[13:18]
		//M  == SCU20[5:12]
		//N  == SCU20[0:4]
		//mpll =  24MHz * [(M+1) /(N+1)] / (P+1)
		clk = ((clk * (SCU_M_PLL_GET_MNUM(m_pll_set) + 1)) / (SCU_M_PLL_GET_NNUM(m_pll_set) + 1))/(SCU_M_PLL_GET_PDNUM(m_pll_set) + 1);
	}
	SCUDBUG("m_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast_get_m_pll_clk);

extern u32
ast_get_ahbclk(void)
{
	unsigned int axi_div, ahb_div, hpll;

	hpll = ast_get_h_pll_clk();
	//AST2500 A1 fix
	axi_div = 2;
	ahb_div = (SCU_HW_STRAP_GET_AXI_AHB_RATIO(ast_scu_read(AST_SCU_HW_STRAP1)) + 1);

	SCUDBUG("HPLL=%d, AXI_Div=%d, AHB_DIV = %d, AHB CLK=%d\n", hpll, axi_div, ahb_div, (hpll/axi_div)/ahb_div);
	return ((hpll/axi_div)/ahb_div);

}

EXPORT_SYMBOL(ast_get_ahbclk);

extern u32
ast_get_d2_pll_clk(void)
{
	u32 clk=0;
	u32 d2_pll_set = ast_scu_read(AST_SCU_D2_PLL);
	u32 d2_pll_conf = ast_scu_read(AST_SCU_D2_PLL_EXTEND);
	u32 MNUM,NNUM,PNUM,ODNUM;

	if(d2_pll_conf & SCU_D2_PLL_OFF)
		return 0;

	// Programming
	clk = ast_get_clk_source();
	if(d2_pll_conf & SCU_D2_PLL_BYPASS) {
		return clk;
	} else {
		MNUM = SCU_D2_PLL_GET_MNUM(d2_pll_set);
		MNUM += 1;
		NNUM = SCU_D2_PLL_GET_NNUM(d2_pll_set);
		NNUM += 1;
		PNUM = SCU_D2_PLL_GET_PNUM(d2_pll_set);
		PNUM += 1;
		ODNUM = SCU_D2_PLL_GET_ODNUM(d2_pll_set);
		ODNUM += 1;
//		printf("clk %d ,num %d ,denum %d ,od %d ,pd %d ,pd2 %d \n",clk, NUM , DENUM, OD, PD, PD2);
		//hpll = 24MHz * [(M + 1) /(N + 1)] / (P + 1) / (OD + 1)
		clk = (clk * MNUM) / (NNUM) / PNUM / ODNUM;
	}

	SCUDBUG("d2_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast_get_d2_pll_clk);

extern void
ast_set_d2_pll_clk(u32 pll_setting)
{
	//Off D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) |  SCU_D2_PLL_OFF | SCU_D2_PLL_RESET , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x585, AST_SCU_D2_PLL_EXTEND);

	//set D2-PLL parameter
	ast_scu_write(pll_setting, AST_SCU_D2_PLL);

	//enable D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) &  ~(SCU_D2_PLL_OFF | SCU_D2_PLL_RESET) , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x580, AST_SCU_D2_PLL_EXTEND);

}

EXPORT_SYMBOL(ast_set_d2_pll_clk);

extern u32
ast_get_d_pll_clk(void)
{
	u32 clk=0;
	u32 d_pll_set = ast_scu_read(AST_SCU_D_PLL);
	u32 d_pll_conf = ast_scu_read(AST_SCU_D_PLL_EXTEND0);
	u32 MNUM,NNUM,PNUM,ODNUM;

	if(d_pll_conf & SCU_D_PLL_OFF)
		return 0;

	// Programming
	clk = ast_get_clk_source();
	if(d_pll_conf & SCU_D_PLL_BYPASS) {
		return clk;
	} else {
		MNUM = SCU_D_PLL_GET_MNUM(d_pll_set);
		MNUM += 1;
		NNUM = SCU_D_PLL_GET_NNUM(d_pll_set);
		NNUM += 1;
		PNUM = SCU_D_PLL_GET_PNUM(d_pll_set);
		PNUM += 1;
		ODNUM = SCU_D_PLL_GET_ODNUM(d_pll_set);
		ODNUM += 1;
//		printf("clk %d ,num %d ,denum %d ,od %d ,pd %d ,pd2 %d \n",clk, NUM , DENUM, OD, PD, PD2);
		//hpll = 24MHz * [(M + 1) /(N + 1)] / (P + 1) / (OD + 1)
		clk = (clk * MNUM) / (NNUM) / PNUM / ODNUM;
	}

	SCUDBUG("d_pll = %d\n",clk);
	return clk;
}
EXPORT_SYMBOL(ast_get_d_pll_clk);

#elif defined(CONFIG_ARCH_AST1010)
extern u32
ast_get_h_pll_clk(void)
{
	u32 speed,clk=0;
	u32 OD, NUM, DENUM;
	u32 h_pll_set = ast_scu_read(AST_SCU_H_PLL);

	clk = ast_get_clk_source();
	OD = (1 << (SCU_H_PLL_GET_DIV(h_pll_set)));
	NUM = SCU_H_PLL_GET_NUM(h_pll_set);
	DENUM = SCU_H_PLL_GET_DENUM(h_pll_set);
	//hpll = 24MHz * (Numerator+1) / ((OD) * (Denumerator+1))
	clk = clk * (NUM + 1) / OD / (DENUM + 1);

//	printf("h_pll = %d\n",clk);
	return clk;
}
EXPORT_SYMBOL(ast_get_h_pll_clk);

extern u32
ast_get_ahbclk(void)
{
	return ast_get_h_pll_clk();
}

EXPORT_SYMBOL(ast_get_ahbclk);

extern u32
ast_get_ahb_div(void)
{
	u32 div = ast_scu_read(AST_SCU_CLK_SEL);
	div = SCU_GET_AHB_DIV(div);
	div = (div + 1) * 2;
	return div;
}
EXPORT_SYMBOL(ast_get_ahb_div);

extern u32
ast_get_pclk(void)
{
        unsigned int div, hpll;

        hpll = ast_get_h_pll_clk();
        div = SCU_GET_PCLK_DIV(ast_scu_read(AST_SCU_CLK_SEL));
        if((div >> 2) == 1) {
                SCUDBUG("div=%d , return 24000000\n", div);
                return 24000000;
        } else {
                SCUDBUG("hpll=%d, Div=%d, PCLK=%d\n", hpll, div, hpll/div);
                div = (div+1) << 1;
                return (hpll/div);
        }

//      SCUDBUG("HPLL=%d, Div=%d, PCLK=%d\n", hpll, div, hpll/div);
//      return (hpll/div);

}
EXPORT_SYMBOL(ast_get_pclk);

#else
extern u32
ast_get_h_pll_clk(void)
{
	u32 speed,clk=0;
	u32 h_pll_set = ast_scu_read(AST_SCU_H_PLL);

	if(h_pll_set & SCU_H_PLL_OFF)
		return 0;

	if(h_pll_set & SCU_H_PLL_PARAMETER) {
		// Programming
		clk = ast_get_clk_source();
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
		speed = SCU_HW_STRAP_GET_H_PLL_CLK(ast_scu_read(AST_SCU_HW_STRAP1));
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

EXPORT_SYMBOL(ast_get_h_pll_clk);

extern u32
ast_get_m_pll_clk(void)
{
	u32 clk=0;
	u32 m_pll_set = ast_scu_read(AST_SCU_M_PLL);

	if(m_pll_set & SCU_M_PLL_OFF)
		return 0;

	// Programming
	clk = ast_get_clk_source();
	if(m_pll_set & SCU_M_PLL_BYPASS) {
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

EXPORT_SYMBOL(ast_get_m_pll_clk);

extern u32
ast_get_ahbclk(void)
{
	unsigned int div, hpll;

	hpll = ast_get_h_pll_clk();
	div = SCU_HW_STRAP_GET_CPU_AHB_RATIO(ast_scu_read(AST_SCU_HW_STRAP1));
	div += 1;

	SCUDBUG("HPLL=%d, Div=%d, AHB CLK=%d\n", hpll, div, hpll/div);
	return (hpll/div);
}

EXPORT_SYMBOL(ast_get_ahbclk);

extern u32
ast_get_d2_pll_clk(void)
{
	u32 clk=0;
	u32 d2_pll_set = ast_scu_read(AST_SCU_D2_PLL);
	u32 OD,NUM,DENUM,PD,PD2;

	if(d2_pll_set & SCU_D2_PLL_OFF)
		return 0;

	// Programming
	clk = ast_get_clk_source();
	if(d2_pll_set & SCU_D2_PLL_BYPASS) {
		return clk;
	} else {
		NUM = SCU_D2_PLL_GET_NUM(d2_pll_set);
		DENUM = SCU_D2_PLL_GET_DENUM(d2_pll_set);
		OD = SCU_D2_PLL_GET_OD(d2_pll_set);
		OD = (1 << (OD - 1));
		PD = SCU_D2_PLL_GET_PD(d2_pll_set);
		PD += 1;
		PD2 = SCU_D2_PLL_GET_PD2(d2_pll_set);
		PD2 += 1;
//		printf("clk %d ,num %d ,denum %d ,od %d ,pd %d ,pd2 %d \n",clk, NUM , DENUM, OD, PD, PD2);
		//hpll = 24MHz * (Numerator * 2) / (Denumerator * OD * PD * PD2)
		clk = (clk * NUM * 2) / (DENUM* OD * PD * PD2);
	}

	SCUDBUG("d2_pll = %d\n",clk);
	return clk;
}

EXPORT_SYMBOL(ast_get_d2_pll_clk);

#endif

extern u32
ast_get_pclk(void)
{
	unsigned int div, hpll;

	hpll = ast_get_h_pll_clk();
	div = SCU_GET_PCLK_DIV(ast_scu_read(AST_SCU_CLK_SEL));
#ifdef AST_SOC_G5
	div = (div+1) << 2;
#else
	div = (div+1) << 1;
#endif

	SCUDBUG("HPLL=%d, Div=%d, PCLK=%d\n", hpll, div, hpll/div);
	return (hpll/div);

}

EXPORT_SYMBOL(ast_get_pclk);

extern u32
ast_get_lhclk(void)
{
	unsigned int div, hpll;
	u32 clk_sel = ast_scu_read(AST_SCU_CLK_SEL);
//FPGA AST1070 is default 100/2 Mhz input
//	return 50000000;
	hpll = ast_get_h_pll_clk();
	if(SCU_LHCLK_SOURCE_EN & clk_sel) {
		div = SCU_GET_LHCLK_DIV(clk_sel);
#ifdef AST_SOC_G5
		div = (div+1) << 2;
#else
		div = (div+1) << 1;
#endif
		SCUDBUG("HPLL=%d, Div=%d, LHCLK = %d\n", hpll, div, hpll/div);
		return (hpll/div);
	} else {
		SCUMSG("LPC CLK not enable \n");
		return 0;
	}

}

EXPORT_SYMBOL(ast_get_lhclk);

extern void
ast_scu_osc_clk_output(void)
{
	//in ast3200 for usb audio code clock
//	if (!(ast_scu_read(AST_SCU_HW_STRAP1) & CLK_25M_IN))
//	{
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) | SCU_MISC_OSC_CLK_OUT_PIN, AST_SCU_MISC1_CTRL);
		ast_scu_write((ast_scu_read(AST_SCU_COUNT_CTRL) & ~SCU_FREQ_SOURCE_FOR_MEASU_MASK) | SCU_FREQ_SOURCE_FOR_MEASU(SCU_FREQ_SOURCE_FOR_MEASU_12MHZ), AST_SCU_COUNT_CTRL);
//	}
}

EXPORT_SYMBOL(ast_scu_osc_clk_output);

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
ast_get_sd_clock_src(void)
{
	u32 clk=0, sd_div;

	clk = ast_get_h_pll_clk();
	//get div
	sd_div = SCU_CLK_SD_GET_DIV(ast_scu_read(AST_SCU_CLK_SEL));
#ifdef AST_SOC_G5
		sd_div = (sd_div+1) << 2;
#else
		sd_div = (sd_div+1) << 1;
#endif
		SCUDBUG("div %d, sdclk =%d \n",sd_div,clk/sd_div);
		clk /= sd_div;

	return clk;
}

EXPORT_SYMBOL(ast_get_sd_clock_src);

extern void
ast_scu_set_lpc_mode(void)
{
#ifdef AST_SOC_G5
	ast_scu_write(SCU_HW_STRAP_ESPI_MODE , AST_SCU_REVISION_ID);
#endif
}
EXPORT_SYMBOL(ast_scu_set_lpc_mode);

extern void
ast_scu_show_system_info (void)
{

#ifdef AST_SOC_G5
	u32 axi_div, ahb_div, h_pll, pclk_div;

	h_pll = ast_get_h_pll_clk();

	//AST2500 A1 fix
	axi_div = 2;
	ahb_div = (SCU_HW_STRAP_GET_AXI_AHB_RATIO(ast_scu_read(AST_SCU_HW_STRAP1)) + 1);
	pclk_div = (SCU_GET_PCLK_DIV(ast_scu_read(AST_SCU_CLK_SEL)) + 1) * 4;

	SCUMSG("CPU = %d MHz , AXI = %d MHz, AHB = %d MHz, PCLK = %d Mhz (div: %d:%d:%d) \n",
			h_pll/1000000,
			h_pll/axi_div/1000000,
			h_pll/axi_div/ahb_div/1000000,
			h_pll/pclk_div/1000000, axi_div, ahb_div, pclk_div);

#else
	u32 h_pll, ahb_div, pclk_div;

	h_pll = ast_get_h_pll_clk();

	ahb_div = SCU_HW_STRAP_GET_CPU_AHB_RATIO(ast_scu_read(AST_SCU_HW_STRAP1));
	ahb_div += 1;
	pclk_div = (SCU_GET_PCLK_DIV(ast_scu_read(AST_SCU_CLK_SEL)) + 1) * 4;

	SCUMSG("CPU = %d MHz ,AHB = %d MHz, PCLK = %d Mhz (div : %d:%d) \n",
			h_pll/1000000, h_pll/ahb_div/1000000, h_pll/ahb_div/1000000,
			ahb_div, pclk_div);
#endif
	return ;
}

EXPORT_SYMBOL(ast_scu_show_system_info);

//*********************************** Multi-function pin control ***********************************
extern void
ast_scu_multi_func_uart(u8 uart)
{
	switch(uart) {
		case 1:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
						SCU_FUN_PIN_UART1_RXD |
						SCU_FUN_PIN_UART1_TXD,
				AST_SCU_FUN_PIN_CTRL2);
			break;
		case 2:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
						SCU_FUN_PIN_UART2_RXD |
						SCU_FUN_PIN_UART2_TXD,
				AST_SCU_FUN_PIN_CTRL2);
			break;
		case 3:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_UART3_RXD |
						SCU_FUN_PIN_UART3_TXD,
				AST_SCU_FUN_PIN_CTRL1);
			break;
		case 4:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
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
		case 6:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |
						SCU_FUC_PIN_UART6,
				AST_SCU_FUN_PIN_CTRL5);
			break;
#ifdef AST_SOC_G5
		case 7:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART7,
				AST_SCU_FUN_PIN_CTRL6);
			break;
		case 8:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART8,
				AST_SCU_FUN_PIN_CTRL6);
			break;
		case 9:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART9,
				AST_SCU_FUN_PIN_CTRL6);
			break;
		case 10:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) &
						~(SCU_FUN_PIN_VGAVS | SCU_FUN_PIN_VGAHS),
				AST_SCU_FUN_PIN_CTRL2);

			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART10,
				AST_SCU_FUN_PIN_CTRL6);
			break;
		case 11:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) &
						~(SCU_FUN_PIN_DDCDAT | SCU_FUN_PIN_DDCCLK),
				AST_SCU_FUN_PIN_CTRL2);

			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART11,
				AST_SCU_FUN_PIN_CTRL6);
			break;
		case 12:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART12,
				AST_SCU_FUN_PIN_CTRL6);
			break;
		case 13:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART13,
				AST_SCU_FUN_PIN_CTRL6);
			break;
#endif
	}


}

extern void
ast_scu_multi_func_video()
{
#if defined(CONFIG_ARCH_2100) || defined(CONFIG_ARCH_2200)
	ast_scu_write(ast_scu_read(AST_SCU_MULTI_FUNC_2) |
				MULTI_FUNC_VIDEO_RGB18 |
				MULTI_FUNC_VIDEO_SINGLE_EDGE,
		AST_SCU_MULTI_FUNC_2);
#elif defined(CONFIG_ARCH_1100) || defined(CONFIG_ARCH_2050)
	ast_scu_write(ast_scu_read(AST_SCU_MULTI_FUNC_2) |
				MULTI_FUNC_VIDEO_RGB18 |
				MULTI_FUNC_VIDEO_SINGLE_EDGE,
		AST_SCU_MULTI_FUNC_2);
#else

#endif
}

#ifdef CONFIG_ARCH_AST1010
extern void
ast_scu_multi_func_eth(u8 num)
{
	switch(num) {
		case 0:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) |
						SCU_FUN_PIN_MAC0_MDIO |
						SCU_FUN_PIN_MAC0_MDC |
						0xff000000,
				AST_SCU_FUN_PIN_CTRL3);

			/* Currently we use fix value in MAC timing on EVB */
			ast_scu_write(0x2255, AST_SCU_MAC_CLK);

			break;
		case 1:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_MAC1_PHY_LINK,
				AST_SCU_FUN_PIN_CTRL1);

			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |
						SCU_FUC_PIN_MAC1_MDIO,
				AST_SCU_FUN_PIN_CTRL5);

			break;
	}
}
#else
extern void
ast_scu_multi_func_eth(u8 num)
{
#if defined(CONFIG_FBTP) // On FBTP, need these pins as default GPIOs
  return;
#endif
	switch(num) {
		case 0:
			if(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_MAC0_RGMII) {
				SCUMSG("MAC0 : RGMII \n");
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
							SCU_FUN_PIN_MAC0_PHY_LINK,
					AST_SCU_FUN_PIN_CTRL1);
			} else {
				SCUMSG("MAC0 : RMII/NCSI \n");
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) &
							~SCU_FUN_PIN_MAC0_PHY_LINK,
					AST_SCU_FUN_PIN_CTRL1);
			}

#ifdef AST_SOC_G5
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_MAC0_PHY_LINK,
				AST_SCU_FUN_PIN_CTRL1);

#endif

			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) |
						SCU_FUN_PIN_MAC0_MDIO |
						SCU_FUN_PIN_MAC0_MDC,
				AST_SCU_FUN_PIN_CTRL3);

			break;
		case 1:
			if(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_MAC1_RGMII) {
				SCUMSG("MAC1 : RGMII \n");
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
							SCU_FUN_PIN_MAC1_PHY_LINK,
					AST_SCU_FUN_PIN_CTRL1);
			} else {
				SCUMSG("MAC1 : RMII/NCSI \n");
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) &
						~SCU_FUN_PIN_MAC1_PHY_LINK,
					AST_SCU_FUN_PIN_CTRL1);
			}

			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_MAC1_PHY_LINK,
				AST_SCU_FUN_PIN_CTRL1);

			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |
						SCU_FUC_PIN_MAC1_MDIO,
				AST_SCU_FUN_PIN_CTRL5);

			break;
	}
}
#endif

extern void
ast_scu_multi_func_nand(void)
{
#ifdef AST_SOC_G5
#else
	//enable NAND flash multipin FLBUSY and FLWP
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
				SCU_FUN_PIN_NAND_FLBUSY | SCU_FUN_PIN_NAND_FLWP,
		AST_SCU_FUN_PIN_CTRL2);
#endif

}

#if 0
extern void
ast_scu_multi_func_nor(void)
{
	//Address
	//ROMA2~17
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL8) |
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
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL9) |
				SCU_FUN_PIN_ROMA18 | SCU_FUN_PIN_ROMA19 |
				SCU_FUN_PIN_ROMA20 | SCU_FUN_PIN_ROMA21,
		AST_SCU_FUN_PIN_CTRL9);

	//ROMA22,23
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL4) | SCU_FUN_PIN_ROMA22 | SCU_FUN_PIN_ROMA23,
		AST_SCU_FUN_PIN_CTRL4);

	//ROMA24,25
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) | SCU_FUN_PIN_ROMA24 | SCU_FUN_PIN_ROMA25,
		AST_SCU_FUN_PIN_CTRL3);

	//SCU94 [1] = 0
 	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) & SCU_VIDEO_OUT_MASK,
		AST_SCU_FUN_PIN_CTRL6);


	//data
	//ROMD 4~7 //ROMWE#, OE#
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL4) |
			SCU_FUN_PIN_ROMOE | SCU_FUN_PIN_ROMWE |
			SCU_FUN_PIN_ROMD4 | SCU_FUN_PIN_ROMD5 |
			SCU_FUN_PIN_ROMD6 | SCU_FUN_PIN_ROMD7,
			AST_SCU_FUN_PIN_CTRL4);

	//ROMD 8~15
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |
			SCU_FUC_PIN_ROM_16BIT,
		AST_SCU_FUN_PIN_CTRL5);

}
#endif

extern void
ast_scu_multi_func_romcs(u8 num)
{
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) |
			SCU_FUN_PIN_ROMCS(num),
		AST_SCU_FUN_PIN_CTRL3);
}

extern void
ast_scu_multi_func_i2c(void)
{
#ifdef CONFIG_ARCH_AST1010
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL4) |
					SCU_FUN_PIN_SCL13 |
					SCU_FUN_PIN_SDA13 |
					SCU_FUN_PIN_SCL14 |
					SCU_FUN_PIN_SDA14,
			AST_SCU_FUN_PIN_CTRL4);

	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
					SCU_FUN_PIN_SCL1 |
					SCU_FUN_PIN_SDA1 |
					SCU_FUN_PIN_SCL2 |
					SCU_FUN_PIN_SDA2 |
					SCU_FUN_PIN_SCL3 |
					SCU_FUN_PIN_SDA3 |
					SCU_FUN_PIN_SCL4 |
					SCU_FUN_PIN_SDA4 |
					SCU_FUN_PIN_SCL5 |
					SCU_FUN_PIN_SDA5 |
					SCU_FUN_PIN_SCL6 |
					SCU_FUN_PIN_SDA6 |
					SCU_FUN_PIN_SCL7 |
					SCU_FUN_PIN_SDA7 |
					SCU_FUN_PIN_SCL8 |
					SCU_FUN_PIN_SDA8 |
					SCU_FUN_PIN_SALT1 |
					SCU_FUN_PIN_SALT2 |
					SCU_FUN_PIN_SALT3 |
					SCU_FUN_PIN_SALT4,
			AST_SCU_FUN_PIN_CTRL2);

	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
					SCU_FUN_PIN_SCL9 |
					SCU_FUN_PIN_SDA9 |
					SCU_FUN_PIN_SCL10 |
					SCU_FUN_PIN_SDA10 |
					SCU_FUN_PIN_SCL11 |
					SCU_FUN_PIN_SDA11 |
					SCU_FUN_PIN_SCL12 |
					SCU_FUN_PIN_SDA12,
			AST_SCU_FUN_PIN_CTRL1);
#else
       // In AST2400 and AST2500, i2c 10 - 13 pins are shared w/ SD/MMC.
#ifdef CONFIG_MMC_AST
       ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |
                       SCU_FUC_PIN_I2C3 |
                       SCU_FUC_PIN_I2C4 |
                       SCU_FUC_PIN_I2C5 |
                       SCU_FUC_PIN_I2C6 |
                       SCU_FUC_PIN_I2C7 |
                       SCU_FUC_PIN_I2C8 |
                       SCU_FUC_PIN_I2C9 |
                       SCU_FUC_PIN_I2C14,
               AST_SCU_FUN_PIN_CTRL5);
#else
       ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |
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
			SCU_FUC_PIN_I2C14) &
			~(SCU_FUC_PIN_SD1 | SCU_FUC_PIN_SD2),
		AST_SCU_FUN_PIN_CTRL5);

#endif
#endif
}

EXPORT_SYMBOL(ast_scu_multi_func_i2c);

extern void
ast_scu_multi_func_pwm_tacho(void)
{
	u32 sts = 0;
	spin_lock(&ast_scu_lock);
#ifdef CONFIG_ARCH_AST1010
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
					SCU_FUN_PIN_PWM1 |
					SCU_FUN_PIN_PWM2 |
					SCU_FUN_PIN_PWM3 |
					SCU_FUN_PIN_PWM4 |
					SCU_FUN_PIN_TACH1 |
					SCU_FUN_PIN_TACH2 |
					SCU_FUN_PIN_TACH3 |
					SCU_FUN_PIN_TACH4 |
					SCU_FUN_PIN_TACH5 |
					SCU_FUN_PIN_TACH6 |
					SCU_FUN_PIN_TACH7 |
					SCU_FUN_PIN_TACH8,
			AST_SCU_FUN_PIN_CTRL2);

	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
					SCU_FUN_PIN_PWM5 |
					SCU_FUN_PIN_PWM6 |
					SCU_FUN_PIN_PWM7 |
					SCU_FUN_PIN_PWM8 |
					SCU_FUN_PIN_TACH9 |
					SCU_FUN_PIN_TACH10 |
					SCU_FUN_PIN_TACH11 |
					SCU_FUN_PIN_TACH12,
			AST_SCU_FUN_PIN_CTRL1);
#else
	sts = ast_scu_read(AST_SCU_FUN_PIN_CTRL3) &~0xcfffff;
	ast_scu_write(sts | 0xc00003, AST_SCU_FUN_PIN_CTRL3);
	spin_unlock(&ast_scu_lock);
#endif
}

EXPORT_SYMBOL(ast_scu_multi_func_pwm_tacho);

//0 : usb 2.0 hub mode, 1:usb 2.0 host2 controller
extern void
ast_scu_multi_func_usb_port1_mode(u8 mode)
{
	if(mode)
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB20_HOST,
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB20_HOST,
					AST_SCU_FUN_PIN_CTRL5);
}

EXPORT_SYMBOL(ast_scu_multi_func_usb_port1_mode);

//0 : 1.1 hid 1, 1.1 host , 2, 2.0 host 3, 2.0 device
extern void
ast_scu_multi_func_usb_port2_mode(u8 mode)
{
#if defined(AST_SOC_G5)
	if(mode == 0)
		ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) & ~SCU_FUN_PIN_USBP1_MASK),
					AST_SCU_FUN_PIN_CTRL6);
	else if ((mode == 1) || (mode == 2))
		ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) & ~SCU_FUN_PIN_USBP1_MASK) |
					SCU_FUN_PIN_USBP1_MODE(0x2),
					AST_SCU_FUN_PIN_CTRL6);
	else if (mode == 3)
		ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) & ~SCU_FUN_PIN_USBP1_MASK) |
					SCU_FUN_PIN_USBP1_MODE(0x1),
					AST_SCU_FUN_PIN_CTRL6);
	else {
		printk("nothing\n");
	}
#else
	if(mode == 0)
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT2,
					AST_SCU_FUN_PIN_CTRL5);
	else if (mode == 1)
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT2,
			AST_SCU_FUN_PIN_CTRL5);
	else if (mode == 2)
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB20_HOST,
					AST_SCU_FUN_PIN_CTRL5);
	else {
		printk("nothing\n");
	}
#endif
}

EXPORT_SYMBOL(ast_scu_multi_func_usb_port2_mode);

//0 : gpioQ6,7 mode , 1: usb1.1 host port 4 mode
extern void
ast_scu_multi_func_usb_port34_mode(u8 mode)
{
#if defined(AST_SOC_G5)
	if(mode) {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT4,
					AST_SCU_FUN_PIN_CTRL5);
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) |
					(SCU_FUN_PIN_USBP3_DP |SCU_FUN_PIN_USBP3_DN | SCU_FUN_PIN_USBP4_DP | SCU_FUN_PIN_USBP4_DN),
					AST_SCU_FUN_PIN_CTRL3);
	} else {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT4,
					AST_SCU_FUN_PIN_CTRL5);
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) &
					~(SCU_FUN_PIN_USBP3_DP |SCU_FUN_PIN_USBP3_DN | SCU_FUN_PIN_USBP4_DP | SCU_FUN_PIN_USBP4_DN),
					AST_SCU_FUN_PIN_CTRL3);
	}
#else
	if(mode) {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT4,
					AST_SCU_FUN_PIN_CTRL5);
	} else {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT4,
					AST_SCU_FUN_PIN_CTRL5);
	}
#endif
}

EXPORT_SYMBOL(ast_scu_multi_func_usb_port34_mode);

//0 : 1: SD1 function
extern void
ast_scu_multi_func_sdhc_8bit_mode(void)
{
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD1 | SCU_FUC_PIN_SD1_8BIT,
					AST_SCU_FUN_PIN_CTRL5);
}

EXPORT_SYMBOL(ast_scu_multi_func_sdhc_8bit_mode);

extern void
ast_scu_multi_func_sdhc_slot(u8 slot)
{
	switch(slot) {
		case 1:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD1,
						AST_SCU_FUN_PIN_CTRL5);
			break;
		case 2:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD2,
						AST_SCU_FUN_PIN_CTRL5);
			break;
		case 3:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD1 | SCU_FUC_PIN_SD2,
						AST_SCU_FUN_PIN_CTRL5);
			break;
	}
}

EXPORT_SYMBOL(ast_scu_multi_func_sdhc_slot);

//0: VGA , 1 : CRT, 2 : PASS through Port -A, 3 : PASS through Port -B
extern void
ast_scu_set_crt_source(u8 dac_soource)
{
	ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_DAC_MASK) |
				SCU_MISC_SET_DAC_SOURCE(dac_soource)  , AST_SCU_MISC1_CTRL);

}

EXPORT_SYMBOL(ast_scu_set_crt_source);

extern void
ast_scu_multi_func_crt(void)
{
	/* multi-pin for DVO enable DVO (bit18) is VGA , enable DAC (bit16) is CRT  */
#if defined(CONFIG_AST_DAC) || defined(CONFIG_AST_DVO)
		ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_DAC_MASK)
					| SCU_MISC_DAC_SOURCE_CRT | SCU_MISC_DVO_SOURCE_CRT | SCU_MISC_2D_CRT_EN , AST_SCU_MISC1_CTRL);
#elif defined(CONFIG_AST_DVO)
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) | SCU_MISC_DVO_SOURCE_CRT| SCU_MISC_2D_CRT_EN, AST_SCU_MISC1_CTRL);
#else //default(CONFIG_AST_DAC)
		ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_DAC_MASK)
					| SCU_MISC_DAC_SOURCE_CRT | SCU_MISC_2D_CRT_EN, AST_SCU_MISC1_CTRL);
#endif

	//Digital vodeo input function pins : 00 disable, 10 24bits mode 888,
	ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) &
			~SCU_FUC_PIN_DIGI_V_OUT_MASK) |
			SCU_FUC_PIN_DIGI_V_OUT(VIDEO_24BITS), AST_SCU_FUN_PIN_CTRL6);

	//VPI input
#if 0
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
			SCU_FUN_PIN_VPIB9 | SCU_FUN_PIN_VPIB8 |
			SCU_FUN_PIN_VPIB7 | SCU_FUN_PIN_VPIB6 |
			SCU_FUN_PIN_VPIB5 | SCU_FUN_PIN_VPIB4 |
			SCU_FUN_PIN_VPIB3 | SCU_FUN_PIN_VPIB2 |
			SCU_FUN_PIN_VPIB1 | SCU_FUN_PIN_VPIB0 |
			SCU_FUN_PIN_VPICLK | SCU_FUN_PIN_VPIVS |
			SCU_FUN_PIN_VPIHS | SCU_FUN_PIN_VPIODD |
			SCU_FUN_PIN_VPIDE ,AST_SCU_FUN_PIN_CTRL2);

	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) |
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

EXPORT_SYMBOL(ast_scu_multi_func_crt);

extern void
ast_scu_multi_nic_switch(u8 enable)
{
	if(enable) {
		ast_scu_write((0x1 << 28) | ast_scu_read(AST_SCU_MAC_CLK), AST_SCU_MAC_CLK);
		ast_scu_write(~(0x1 << 11) & ast_scu_read(AST_SCU_RESET), AST_SCU_RESET);	//A1 ECO
	} else {
		ast_scu_write(~(0x1 << 28) & ast_scu_read(AST_SCU_MAC_CLK), AST_SCU_MAC_CLK);
		ast_scu_write((0x1 << 11) | ast_scu_read(AST_SCU_RESET), AST_SCU_RESET);
	}

}

extern void
ast_scu_multi_func_sgpio(void)
{
#ifdef CONFIG_ARCH_AST1010
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
			SCU_FUN_PIN_SGPMI |
			SCU_FUN_PIN_SGPMO |
			SCU_FUN_PIN_SGPMLD |
			SCU_FUN_PIN_SGPMCK, AST_SCU_FUN_PIN_CTRL1);
#else
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
			SCU_FUN_PIN_SGPMI |
			SCU_FUN_PIN_SGPMO |
			SCU_FUN_PIN_SGPMLD |
			SCU_FUN_PIN_SGPMCK, AST_SCU_FUN_PIN_CTRL2);
#endif
}

EXPORT_SYMBOL(ast_scu_multi_func_sgpio);

//***********************************Information ***********************************
extern u32
ast_scu_revision_id(void)
{
	int i;
	u32 rev_id = ast_scu_read(AST_SCU_REVISION_ID);
	for(i=0;i<ARRAY_SIZE(soc_map_table);i++) {
		if(rev_id == soc_map_table[i].rev_id)
			break;
	}
	if(i == ARRAY_SIZE(soc_map_table))
		SCUMSG("UnKnow-SOC : %x \n",rev_id);
	else
		SCUMSG("SOC : %4s \n",soc_map_table[i].name);

	return rev_id;
}

EXPORT_SYMBOL(ast_scu_revision_id);

extern void
ast_scu_security_info(void)
{
	switch((ast_scu_read(AST_SCU_HW_STRAP2) >> 18) & 0x3) {
		case 1:
			printk("SEC : DSS Mode \n");
			break;
		case 2:
			printk("SEC : UnKnow \n");
			break;
		case 3:
			printk("SEC : SPI2 Mode \n");
			break;
	}

}

extern void
ast_scu_sys_rest_info(void)
{
	u32 rest = ast_scu_read(AST_SCU_SYS_CTRL);

#ifdef CONFIG_ARCH_AST1010
	if(rest & SCU_SYS_WDT_FULL_FLAG) {
		SCUMSG("RST : External \n");
		ast_scu_write(SCU_SYS_WDT_FULL_FLAG, AST_SCU_SYS_CTRL);
	} else if (rest & SCU_SYS_WDT_SOC_RESET) {
		SCUMSG("RST : Watchdog - SOC\n");
		ast_scu_write(SCU_SYS_WDT_SOC_RESET, AST_SCU_SYS_CTRL);
	} else if (rest & SCU_SYS_PWR_RESET_FLAG) {
		SCUMSG("RST : Power On \n");
		ast_scu_write(SCU_SYS_PWR_RESET_FLAG, AST_SCU_SYS_CTRL);
	} else {
	}
#else
	if(rest & SCU_SYS_EXT_RESET_FLAG) {
		SCUMSG("RST : External \n");
		ast_scu_write(SCU_SYS_EXT_RESET_FLAG, AST_SCU_SYS_CTRL);
	} else if (rest & SCU_SYS_WDT_RESET_FLAG) {
		SCUMSG("RST : Watchdog \n");
		ast_scu_write(SCU_SYS_WDT_RESET_FLAG, AST_SCU_SYS_CTRL);
	} else if (rest & SCU_SYS_PWR_RESET_FLAG) {
		SCUMSG("RST : Power On \n");
		ast_scu_write(SCU_SYS_PWR_RESET_FLAG, AST_SCU_SYS_CTRL);
	} else {
		SCUMSG("RST : CLK en \n");
	}
#endif
}


/*
* D[15:11] in 0x1E6E2040 is NCSI scratch from U-Boot. D[15:14] = MAC1, D[13:12] = MAC2
* The meanings of the 2 bits are:
* 00(0): Dedicated PHY
* 01(1): ASPEED's EVA + INTEL's NC-SI PHY chip EVA
* 10(2): ASPEED's MAC is connected to NC-SI PHY chip directly
* 11: Reserved
*/

extern u32
ast_scu_get_phy_config(u8 mac_num)
{
	u32 scatch = ast_scu_read(AST_SCU_SOC_SCRATCH0);

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
EXPORT_SYMBOL(ast_scu_get_phy_config);

extern u32
ast_scu_get_phy_interface(u8 mac_num)
{
	u32 trap1 = ast_scu_read(AST_SCU_HW_STRAP1);

	switch(mac_num) {
		case 0:
			if(SCU_HW_STRAP_MAC0_RGMII & trap1)
				return 1;
			else
				return 0;
			break;
		case 1:
			if(SCU_HW_STRAP_MAC1_RGMII & trap1)
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
EXPORT_SYMBOL(ast_scu_get_phy_interface);

extern void
ast_scu_set_vga_display(u8 enable)
{
	if(enable)
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_VGA_CRT_DIS, AST_SCU_MISC1_CTRL);
	else
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) | SCU_MISC_VGA_CRT_DIS, AST_SCU_MISC1_CTRL);
}

EXPORT_SYMBOL(ast_scu_set_vga_display);

extern u8
ast_scu_get_vga_display(void)
{
	if(ast_scu_read(AST_SCU_MISC1_CTRL) & SCU_MISC_VGA_CRT_DIS)
		return 0;
	else
		return 1;
}

EXPORT_SYMBOL(ast_scu_get_vga_display);

extern u32
ast_scu_get_vga_memsize(void)
{
	u32 size=0;

	switch(SCU_HW_STRAP_VGA_SIZE_GET(ast_scu_read(AST_SCU_HW_STRAP1))) {
		case VGA_8M_DRAM:
			size = 8*1024*1024;
			break;
		case VGA_16M_DRAM:
			size = 16*1024*1024;
			break;
		case VGA_32M_DRAM:
			size = 32*1024*1024;
			break;
		case VGA_64M_DRAM:
			size = 64*1024*1024;
			break;
		default:
			SCUMSG("error vga size \n");
			break;
	}
	return size;
}

EXPORT_SYMBOL(ast_scu_get_vga_memsize);

extern u32
ast_scu_get_soc_dram_base(void)
{
	u32 rev_id = ast_scu_read(AST_SCU_REVISION_ID);
	if((rev_id >> AST_SOC_GEN) > 3)
		return AST_DRAM_BASE_8;
	else
		return AST_DRAM_BASE_4;
}

extern void
ast_scu_get_who_init_dram(void)
{
	switch(SCU_VGA_DRAM_INIT_MASK(ast_scu_read(AST_SCU_VGA0))) {
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
EXPORT_SYMBOL(ast_scu_get_who_init_dram);

extern u8
ast_scu_adc_trim_read(void)
{
	return (ast_scu_read(AST_SCU_OTP1) >> 28);
}
EXPORT_SYMBOL(ast_scu_adc_trim_read);

extern void
ast_scu_hw_random_enable(u8 enable)
{
	if(enable)
		ast_scu_write(ast_scu_read(AST_SCU_RAMDOM_GEN) | RNG_ENABLE, AST_SCU_RAMDOM_GEN);
	else
		ast_scu_write(ast_scu_read(AST_SCU_RAMDOM_GEN) & ~RNG_ENABLE, AST_SCU_RAMDOM_GEN);
}
EXPORT_SYMBOL(ast_scu_hw_random_enable);

extern u32
ast_scu_hw_random_read(void)
{
	return (ast_scu_read(AST_SCU_RAMDOM_DATA));
}
EXPORT_SYMBOL(ast_scu_hw_random_read);

extern u8
ast_scu_get_hw_random_type(void)
{
	return (RNG_GET_TYPE(ast_scu_read(AST_SCU_RAMDOM_GEN)));
}
EXPORT_SYMBOL(ast_scu_get_hw_random_type);

extern void
ast_scu_set_hw_random_type(u8 type)
{
	ast_scu_write(((ast_scu_read(AST_SCU_RAMDOM_GEN) & ~RNG_TYPE_MASK) | RNG_SET_TYPE(type)), AST_SCU_RAMDOM_GEN);
}
EXPORT_SYMBOL(ast_scu_set_hw_random_type);

#ifdef AST_SCU_OTP_READ_CTRL
extern u8
ast_scu_otp_read(u8 reg)
{
        ast_scu_write(SCU_OTP_TRIGGER | SCU_OTP_READ_ADDR(reg), AST_SCU_OTP_READ_CTRL);
        while(SCU_OTP_TRIGGER_STS & ast_scu_read(AST_SCU_OTP_READ_CTRL));
        return (SCU_OTP_READ_DATA(ast_scu_read(AST_SCU_OTP_READ_CTRL)));
}

EXPORT_SYMBOL(ast_scu_otp_read);
#endif

static irqreturn_t ast_scu_isr (int this_irq, void *dev_id)
{
	u32 sts = ast_scu_read(AST_SCU_INTR_CTRL);

	SCUDBUG(" %x\n",sts);

	if(sts & INTR_LPC_H_L_RESET) {
		printk("SCU : INTR_LPC_H_L_RESET \n");
	}

	if(sts & INTR_LPC_L_H_RESET) {
		printk("SCU : INTR_LPC_L_H_RESET \n");
	}

	if(sts & INTR_PCIE_H_L_RESET) {
		printk("SCU : INTR_PCIE_H_L_RESET\n");
	}

	if(sts & INTR_PCIE_L_H_RESET) {
		printk("SCU : INTR_PCIE_L_H_RESET \n");
	}

	if(sts & INTR_VGA_SCRATCH_CHANGE)
{
		printk("SCU : INTR_VGA_SCRATCH_CHANGE \n");
		printk("%x, %x, %x, %x, %x, %x, %x, %x \n", ast_scu_read(AST_SCU_VGA_SCRATCH0), ast_scu_read(AST_SCU_VGA_SCRATCH1), ast_scu_read(AST_SCU_VGA_SCRATCH2), ast_scu_read(AST_SCU_VGA_SCRATCH3), ast_scu_read(AST_SCU_VGA_SCRATCH4), ast_scu_read(AST_SCU_VGA_SCRATCH5), ast_scu_read(AST_SCU_VGA_SCRATCH6), ast_scu_read(AST_SCU_VGA_SCRATCH7));

	}

	if(sts & INTR_VGA_CURSOR_CHANGE) {
		printk("SCU : INTR_VGA_CURSOR_CHANGE \n");
	}

	ast_scu_write(sts, AST_SCU_INTR_CTRL);
	return IRQ_HANDLED;
}

#if 0
static int __init ast_scu_init(void)
{
	int ret = 0;
	SCUDBUG("\n");

	ret = request_irq(IRQ_SCU, ast_scu_isr, IRQF_SHARED, "ast-scu", &ret);
	if (ret) {
		printk("AST SCU Unable request IRQ \n");
		goto out;
	}

	//SCU intr enable
	ast_scu_write(0x003f0000,
				AST_SCU_INTR_CTRL);

	ast_scu_write(ast_scu_read(AST_SCU_INTR_CTRL) |
				INTR_LPC_H_L_RESET_EN	| INTR_LPC_L_H_RESET_EN | INTR_PCIE_H_L_RESET_EN |
				INTR_PCIE_L_H_RESET_EN |INTR_VGA_SCRATCH_CHANGE_EN | INTR_VGA_CURSOR_CHANGE_EN	,
				AST_SCU_INTR_CTRL);

out:
	return ret;
}

arch_initcall(ast_scu_init);
#endif
