/*
 *  linux/arch/arm/mach-ast1070/ast1070.c
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
//#include <linux/config.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/devs.h>
#include <plat/ast1070-scu.h>
#include <plat/ast1070-uart-dma.h>
#include <mach/time.h>
#include <mach/gpio.h>

static int __init ast1070_init(void)
{
	int i=0;
	u8 num = 0;
	if(gpio_get_value(PIN_GPIOI2))
		num = 2;	//dual 1070
	else
		num = 1;	//single 1070

	if(CONFIG_AST1070_NR != num)
		printk("Please check Configuration !!! \n");
	
#if 0	
	if(gpio_get_value(PIN_GPIOI1))
		printk("Use LPC+ Bus Access \n");
	else
		printk("Use LPC Bus Access \n");		
#endif

	for(i=0; i< CONFIG_AST1070_NR;i++) {	
		ast1070_scu_revision_id(i);
		ast1070_dma_init(i);
		ast1070_uart_dma_init(i);
	}

	return 0;
}

subsys_initcall(ast1070_init);

