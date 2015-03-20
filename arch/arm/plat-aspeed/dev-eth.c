/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-eth.c
* Author        : Ryan Chen
* Description   : Aspeed Ethernet Device
*
* Copyright (C) ASPEED Technology Inc.
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
*    1. 2012/08/24 Ryan Chen initial
*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/ftgmac100_drv.h>

#include <plat/devs.h>
#include <plat/ast-scu.h>

/* --------------------------------------------------------------------
 *  Ethernet
 * -------------------------------------------------------------------- */
#if defined(CONFIG_ASPEEDMAC) || defined(CONFIG_ASPEEDMAC_MODULE)
#ifdef AST_MAC0_BASE
static struct ftgmac100_eth_data ast_eth0_data = {
	.dev_addr = { 0x00, 0x84, 0x14, 0xA0, 0xB0, 0x22},		
	.phy_id = 1,
};

static u64 ast_eth_dmamask = 0xffffffffUL;
static struct resource ast_mac0_resources[] = {
	[0] = {
		.start = AST_MAC0_BASE,
		.end = AST_MAC0_BASE + SZ_128K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MAC0,
		.end = IRQ_MAC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_eth0_device = {
	.name		= "ast_gmac",
	.id		= 0,
	.dev		= {
				.dma_mask		= &ast_eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
				.platform_data = &ast_eth0_data,
	},
	.resource	= ast_mac0_resources,
	.num_resources = ARRAY_SIZE(ast_mac0_resources),
};
#endif
#ifdef AST_MAC1_BASE
static struct ftgmac100_eth_data ast_eth1_data = {
	.dev_addr = { 0x00, 0x84, 0x14, 0xA0, 0xB0, 0x23},		
	.phy_id = 1,
};

static struct resource ast_mac1_resources[] = {
	[0] = {
		.start = AST_MAC1_BASE,
		.end = AST_MAC1_BASE + SZ_128K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_MAC1,
		.end = IRQ_MAC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ast_eth1_device = {
	.name		= "ast_gmac",
	.id		= 1,
	.dev		= {
				.dma_mask		= &ast_eth_dmamask,
				.coherent_dma_mask	= 0xffffffff,
				.platform_data = &ast_eth1_data,
	},
	.resource	= ast_mac1_resources,
	.num_resources = ARRAY_SIZE(ast_mac1_resources),
};
#endif

/*
 * MAC1 always has MII MDC+MDIO pins to access PHY registers.  We assume MAC1
 * always has a PHY chip, if MAC1 is enabled.
 * U-Boot can enable MAC2 MDC+MDIO pins for a 2nd PHY, or MAC2 might be
 * disabled (only one port), or it's sideband-RMII which has no PHY chip.
 *
 * Return miiPhyId==0 if the MAC cannot be accessed.
 * Return miiPhyId==1 if the MAC registers are OK but it cannot carry traffic.
 * Return miiPhyId==2 if the MAC can send/receive but it has no PHY chip.
 * Else return the PHY 22-bit vendor ID, 6-bit model and 4-bit revision.
 */

void __init ast_add_device_gmac(void)
{

	u8 phy_mode,phy_inter;
	u32 isRevA0;
	u32 rev_id;

	rev_id = ast_scu_revision_id() & 0xff;


	if (rev_id >= 0x08 && rev_id <= 0x0f) {
		// AST2100 FPGA board: up to 10 means rev.A0, 11 means rev.A1
		isRevA0 = (rev_id < 11);
	} else {
		// Real silicon: rev.A0 has 0x00 in bits[7:0]. rev A2 = 0x02 in bits[7:0]
		isRevA0 = 0; //((regVal & 0x00ff) == 0x00);
//		out->isRevA2 = 1; //((regVal & 0x00ff) == 0x02);
	}

	ast_eth0_data.DF_support = !isRevA0;
	
	// Wedge/6-Pack hardware attaches to MAC1;  there's nothing on
	// MAC0.  Older drivers would drop interfaces without PHYs, but
	// the latest open source drivers do not, so we drop the first
	// MAC specs.
#ifndef CONFIG_WEDGE
	ast_scu_init_eth(0);
	ast_scu_multi_func_eth(0);
	
	
	/*
	* D[15:11] in 0x1E6E2040 is NCSI scratch from U-Boot. D[15:14] = MAC1, D[13:12] = MAC2
	* The meanings of the 2 bits are:
	* 00(0): Dedicated PHY
	* 01(1): ASPEED's EVA + INTEL's NC-SI PHY chip EVA
	* 10(2): ASPEED's MAC is connected to NC-SI PHY chip directly
	* 11: Reserved
	*/
	
	phy_mode = ast_scu_get_phy_config(0);
	switch(phy_mode) {
		case 0:
			ast_eth0_data.INTEL_NCSI_EVA_support = 0;
			ast_eth0_data.NCSI_support = 0;
			break;
		case 1:
			ast_eth0_data.NCSI_support = 1;
			break;
		case 2:
			ast_eth0_data.INTEL_NCSI_EVA_support = 1;
			break;
			
	}

	phy_inter = ast_scu_get_phy_interface(0);

	// We assume the Clock Stop register does not disable the MAC1 or MAC2 clock
	// unless Reset Control also holds the MAC in reset.

	
	platform_device_register(&ast_eth0_device);
#endif

#ifdef AST_MAC1_BASE
	ast_scu_init_eth(1);
	ast_scu_multi_func_eth(1);	

	ast_eth1_data.DF_support = !isRevA0;

	phy_mode = ast_scu_get_phy_config(1);
	switch(phy_mode) {
		case 0:
			ast_eth1_data.INTEL_NCSI_EVA_support = 0;
			ast_eth1_data.NCSI_support = 0;
			break;
		case 1:
			ast_eth1_data.NCSI_support = 1;
			break;
		case 2:
			ast_eth1_data.INTEL_NCSI_EVA_support = 1;
			break;
			
	}
	phy_inter = ast_scu_get_phy_interface(1);

	platform_device_register(&ast_eth1_device);

#endif

}
#else
void __init ast_add_device_gmac(void) {}
#endif

