/*
 *  linux/arch/arm/mach-aspeed/ast.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <asm/mach-types.h>
#include <asm/system_misc.h>

#include <asm/mach/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <plat/devs.h>

#include <plat/ast-scu.h>
#include <plat/core.h>

#ifdef CONFIG_ARCH_AST2500
#include <mach/ast-uart-dma.h>
#endif
#include <mach/ast_wdt.h>

static struct map_desc ast_io_desc[] __initdata = {
	{ 	
		.virtual		= IO_ADDRESS(AST_VIC_BASE), 
		.pfn			= __phys_to_pfn(AST_VIC_BASE),
		.length 		= SZ_64K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_SCU_BASE), 
		.pfn			= __phys_to_pfn(AST_SCU_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_WDT_BASE), 
		.pfn			= __phys_to_pfn(AST_WDT_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, { 	
		.virtual		= IO_ADDRESS(AST_SDMC_BASE), 
		.pfn			= __phys_to_pfn(AST_SDMC_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,		
	}, { 
		.virtual		= IO_ADDRESS(AST_CRYPTO_BASE), 
		.pfn			= __phys_to_pfn(AST_CRYPTO_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS(AST_GPIO_BASE), 
		.pfn			= __phys_to_pfn(AST_GPIO_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS(AST_VIDEO_BASE), 
		.pfn			= __phys_to_pfn(AST_VIDEO_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, { 
	
		.virtual		= IO_ADDRESS(AST_UART0_BASE), 
		.pfn			= __phys_to_pfn(AST_UART0_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,		
	},
};

void __init ast_map_io(void)
{
	iotable_init(ast_io_desc, ARRAY_SIZE(ast_io_desc));
}

static void __init ast_init(void)
{
#ifdef CONFIG_ARCH_AST2500
	ast_uart_sdma_init();
#endif

	ast_add_all_devices();		

	ast_scu_show_system_info();
#ifdef CONFIG_AST2400_BMC
	ast2400_scu_init(AST_PCI_EXT_SCU, SZ_4K);
	ast2400_add_device_uart();
//	ast2400_add_device_i2c();
#endif	
}

static const char * const ast_dt_match[] = {
	"aspeed,ast2500",
	NULL
};

//Non-DT
MACHINE_START(ASPEED, AST_MACH_NAME)
	.map_io			= ast_map_io,
	.init_irq			= ast_init_irq,	
	.init_machine		= ast_init,	
	.init_time			= ast_init_timer,
#if defined(CONFIG_AST_WATCHDOG) || defined(CONFIG_AST_WATCHDOG_MODULE)	 	
	.restart			= ast_soc_wdt_reset,
#endif
MACHINE_END
