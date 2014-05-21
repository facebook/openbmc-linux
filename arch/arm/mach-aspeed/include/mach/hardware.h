/*
 *  linux/include/asm-arm/arch-mvp2000/hardware.h
 *
 *  This file contains the hardware definitions of the MVP-2000.
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

#define IO_BASE					0xF0000000                 // VA of IO
#define IO_BASE2				0xE0000000                 // VA of IO2 (AST1070)
#define IO_SIZE                 ASPEED_IO_SIZE             // How much?
#define IO_START                ASPEED_IO_START            // PA of IO
#define IO_SIZE2                ASPEED_IO_SIZE             // How much?
#define IO_START2               ASPEED_IO_START2           // PA of IO


#define ASPEED_IO_START2		AST_LPC_BRIDGE

/* macro to get at IO space when running virtually */
//#define IO_ADDRESS(x) (((x) >> 4) + IO_BASE) 
#define IO_ADDRESS(x)  (x - 0x10000000 + IO_BASE) 
#define IO_ADDRESS2(x) (x - ASPEED_IO_START2 + IO_BASE2) 

//PCIE
#ifdef CONFIG_PCIE
#define PCIBIOS_MIN_IO		0
#define PCIBIOS_MIN_MEM		0
#define pcibios_assign_all_busses()	0
#define ARCH_HAS_DMA_SET_COHERENT_MASK
#endif

#endif	/* __ASM_ARCH_HARDWARE_H END */

