/*
 *  linux/arch/arm/mach-ast2300/ast2300.c
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
#include <linux/sysdev.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/time.h>
#include <mach/hardware.h>
#include <plat/devs.h>

#include "core.h"

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
		.virtual		= IO_ADDRESS(AST_SDMC_BASE), 
		.pfn			= __phys_to_pfn(AST_SDMC_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,		
	}, {
		.virtual		= IO_ADDRESS(AST_VIDEO_BASE), 
		.pfn			= __phys_to_pfn(AST_VIDEO_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,		
	}, {
		.virtual		= IO_ADDRESS(AST_MAC0_BASE),
		.pfn			= __phys_to_pfn(AST_MAC0_BASE),
		.length 		= SZ_64K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_MAC1_BASE),
		.pfn			= __phys_to_pfn(AST_MAC1_BASE),
		.length 		= SZ_64K,
		.type			= MT_DEVICE,		
	}, { 
		.virtual		= IO_ADDRESS(AST_CRYPTO_BASE), 
		.pfn			= __phys_to_pfn(AST_CRYPTO_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_JTAG_BASE), 
		.pfn			= __phys_to_pfn(AST_JTAG_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_XDMA_BASE), 
		.pfn			= __phys_to_pfn(AST_XDMA_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS(AST_MCTP_BASE), 
		.pfn			= __phys_to_pfn(AST_MCTP_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_SRAM_BASE), 
		.pfn			= __phys_to_pfn(AST_SRAM_BASE),
		.length 		= SZ_16K*2,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_2D_BASE), 
		.pfn			= __phys_to_pfn(AST_2D_BASE),
		.length 		= SZ_64K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_GPIO_BASE), 
		.pfn			= __phys_to_pfn(AST_GPIO_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_TIMER_BASE), 
		.pfn			= __phys_to_pfn(AST_TIMER_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_UART0_BASE), 
		.pfn			= __phys_to_pfn(AST_UART0_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_UART4_BASE), 
		.pfn			= __phys_to_pfn(AST_UART4_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_WDT_BASE), 
		.pfn			= __phys_to_pfn(AST_WDT_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_UDC11_BASE	), 
		.pfn			= __phys_to_pfn(AST_UDC11_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_VUART0_BASE), 
		.pfn			= __phys_to_pfn(AST_VUART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_PUART_BASE), 
		.pfn			= __phys_to_pfn(AST_PUART_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS(AST_LPC_BASE), 
		.pfn			= __phys_to_pfn(AST_LPC_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS(AST_PECI_BASE), 
		.pfn			= __phys_to_pfn(AST_PECI_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
#if defined(CONFIG_ARCH_AST1070)
		.virtual		= IO_ADDRESS2(AST_C0_VIC_BASE), 
		.pfn			= __phys_to_pfn(AST_C0_VIC_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS2(AST_C0_SCU_BASE), 
		.pfn			= __phys_to_pfn(AST_C0_SCU_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS2(AST_C0_I2C_BASE), 
		.pfn			= __phys_to_pfn(AST_C0_I2C_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS2(AST_C1_VIC_BASE), 
		.pfn			= __phys_to_pfn(AST_C1_VIC_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS2(AST_C1_SCU_BASE), 
		.pfn			= __phys_to_pfn(AST_C1_SCU_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, { 
		.virtual		= IO_ADDRESS2(AST_C1_I2C_BASE), 
		.pfn			= __phys_to_pfn(AST_C1_I2C_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,
	}, {	
		.virtual		= IO_ADDRESS2(AST_C0_UART0_BASE), 
		.pfn			= __phys_to_pfn(AST_C0_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {	
		.virtual		= IO_ADDRESS2(AST_C0_UART1_BASE), 
		.pfn			= __phys_to_pfn(AST_C0_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {	
		.virtual		= IO_ADDRESS2(AST_C0_UART2_BASE), 
		.pfn			= __phys_to_pfn(AST_C0_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {	
		.virtual		= IO_ADDRESS2(AST_C0_UART3_BASE), 
		.pfn			= __phys_to_pfn(AST_C0_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {	
		.virtual		= IO_ADDRESS2(AST_C1_UART0_BASE), 
		.pfn			= __phys_to_pfn(AST_C1_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {	
		.virtual		= IO_ADDRESS2(AST_C1_UART1_BASE), 
		.pfn			= __phys_to_pfn(AST_C1_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {	
		.virtual		= IO_ADDRESS2(AST_C1_UART2_BASE), 
		.pfn			= __phys_to_pfn(AST_C1_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {	
		.virtual		= IO_ADDRESS2(AST_C1_UART3_BASE), 
		.pfn			= __phys_to_pfn(AST_C1_UART0_BASE),
		.length 		= SZ_1K,
		.type			= MT_DEVICE,		
	}, {		
#endif
		.virtual		= IO_ADDRESS(AST_UART1_BASE), 
		.pfn			= __phys_to_pfn(AST_UART1_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_UART2_BASE), 
		.pfn			= __phys_to_pfn(AST_UART2_BASE),
		.length 		= SZ_4K,
		.type			= MT_DEVICE,
	}, {
		.virtual		= IO_ADDRESS(AST_UART3_BASE), 
		.pfn			= __phys_to_pfn(AST_UART3_BASE),
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
	ast_add_all_devices();
}

MACHINE_START(ASPEED, "AST2400")
	.phys_io		= AST_IO_START,
//	.phys_ram		= AST_SDRAM_BASE,
	.io_pg_offst		= (IO_ADDRESS(AST_IO_START)>>18) & 0xfffc,
	.boot_params	= 0x40000100,	
	.map_io			= ast_map_io,
	.timer			= &ast_timer,
	.init_irq		= ast_init_irq,
	.init_machine		= ast_init,
MACHINE_END
