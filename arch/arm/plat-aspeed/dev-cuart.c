/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-cuart.c
* Author        : Ryan chen
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
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

* History      :
*    1. 2012/09/15 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/serial_8250.h>

#include <mach/irqs.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <mach/ast-uart-dma.h>

#include <plat/ast1070-devs.h>
#include <plat/ast1070-scu.h>

/* --------------------------------------------------------------------
 * UART
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SERIAL_8250) && defined(CONFIG_ARCH_AST1070)
static struct ast_uart_dma_data c0_uart0_dma_data = {
	.chip_no = 0,	
	.dma_ch = 0,
};

static struct ast_uart_dma_data c0_uart1_dma_data = {
	.chip_no = 0,
	.dma_ch = 1,
};

static struct ast_uart_dma_data c0_uart2_dma_data = {
	.chip_no = 0,
	.dma_ch = 2,
};

static struct ast_uart_dma_data c0_uart3_dma_data = {
	.chip_no = 0,
	.dma_ch = 3,
};

#if (CONFIG_AST1070_NR >=2)
static struct ast_uart_dma_data c1_uart0_dma_data = {
	.chip_no = 1,	
	.dma_ch = 0,
};

static struct ast_uart_dma_data c1_uart1_dma_data = {
	.chip_no = 1,
	.dma_ch = 1,
};

static struct ast_uart_dma_data c1_uart2_dma_data = {
	.chip_no = 1,
	.dma_ch = 2,
};

static struct ast_uart_dma_data c1_uart3_dma_data = {
	.chip_no = 1,
	.dma_ch = 3,
};
#endif

static struct plat_serial8250_port ast1070_c_uart_data[] = {
	{
		.mapbase	= AST_C0_UART0_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C0_UART0_BASE)),
		.irq		= IRQ_C0_N1_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c0_uart0_dma_data,
	},
	{
		.mapbase	= AST_C0_UART1_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C0_UART1_BASE)),
		.irq		= IRQ_C0_N2_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c0_uart1_dma_data,
	},	
	{
		.mapbase	= AST_C0_UART2_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C0_UART2_BASE)),
		.irq		= IRQ_C0_N3_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c0_uart2_dma_data,
	},	
	{
		.mapbase	= AST_C0_UART3_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C0_UART3_BASE)),
		.irq		= IRQ_C0_N4_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c0_uart3_dma_data,
	},		
#if (CONFIG_AST1070_NR >=2)
	{
		.mapbase	= AST_C1_UART0_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C1_UART0_BASE)),
		.irq		= IRQ_C1_N1_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c1_uart0_dma_data,
	},
	{
		.mapbase	= AST_C1_UART1_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C1_UART1_BASE)),
		.irq		= IRQ_C1_N2_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c1_uart1_dma_data,
	},	
	{
		.mapbase	= AST_C1_UART2_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C1_UART2_BASE)),
		.irq		= IRQ_C1_N3_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c1_uart2_dma_data,
	},	
	{
		.mapbase	= AST_C1_UART3_BASE,
		.membase	= (char*)(IO_ADDRESS2(AST_C1_UART3_BASE)),
		.irq		= IRQ_C1_N4_UART,
		.uartclk	= (24*1000000L),
		.regshift	= 2,
		.iotype 	= UPIO_MEM,
		.flags		= UPF_IOREMAP | UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.private_data = &c1_uart3_dma_data,
	},		
#endif
	{ },
};

struct platform_device ast1070_c_uart_device = {
#ifdef CONFIG_SERIAL_AST_DMA_UART	
	.name	= "ast-uart-dma",
#else
	.name	= "serial8250",
#endif	
	.id		= PLAT8250_DEV_PLATFORM1,
	.dev			= {
		.platform_data	= ast1070_c_uart_data,
	},	
};

void __init ast_add_device_cuart(void)
{
	int i;//j;
	for(i=0;i<CONFIG_AST1070_NR;i++) {
		//reset 4 UART 
		ast1070_scu_init_uart(i);
		//Please don't enable : Feature remove
//		for(j=0;j<4;j++) 
//			ast1070_multi_func_uart(i, j);
	}
	
	platform_device_register(&ast1070_c_uart_device);	
}
#else
void __init ast_add_device_cuart(void) {}
#endif
