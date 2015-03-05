/*
 *  hardware.h
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <mach/platform.h>

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 */

#define IO_BASE					0xF8000000                 // VA of IO
/*#define IO_BASE2				0xE0000000                 // VA of IO2 (AST1070) */

#ifdef CONFIG_AST_PCIE_EXT
#define ASPEED_IO_START2		AST_PCIE_WIN_BASE
#else
#define ASPEED_IO_START2		AST_LPC_BRIDGE
#endif

/* macro to get at IO space when running virtually */
//#define IO_ADDRESS(x) (((x) >> 4) + IO_BASE) 
/*#define IO_ADDRESS(x)  (x - 0x10000000 + IO_BASE)  */
#define IO_ADDRESS(x)  (x - 0x1e600000 + IO_BASE) 
/*#define IO_ADDRESS2(x) (x - ASPEED_IO_START2 + IO_BASE2) */

//PCIE
#ifdef CONFIG_AST_PCIE
#define PCIBIOS_MIN_IO		0x0
#define PCIBIOS_MIN_MEM		0x0
#define pcibios_assign_all_busses()	1
#endif

#endif	/* __ASM_ARCH_HARDWARE_H END */

