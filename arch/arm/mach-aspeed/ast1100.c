/*
 *  linux/arch/arm/mach-ast2000/ast2000.c
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

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <mach/time.h>

extern void aspeed_map_io(void);
extern void aspeed_init_irq(void);
extern struct sys_timer aspeed_timer;

static void __init aspeed_init(void)
{
	ast_add_all_devices();
}

MACHINE_START(ASPEED, "AST1100")
	.phys_io		= ASPEED_IO_START,
//	.phys_ram		= ASPEED_SDRAM_BASE,
	.io_pg_offst		= (IO_ADDRESS(IO_START)>>18) & 0xfffc,
	.map_io			= aspeed_map_io,
	.timer			= &aspeed_timer,
	.init_irq		= aspeed_init_irq,
	.init_machine		= aspeed_init,
MACHINE_END
