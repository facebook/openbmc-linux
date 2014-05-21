/********************************************************************************
* arch/arm/plat-aspeed/include/plat/devs.h
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
********************************************************************************/
#ifndef __ASM_PLAT_AST1070_H
#define __ASM_PLAT_AST1070_H

//ast1070
extern void __init ast_add_device_cuart(u8 chip, u32 lpc_base);
extern void __init ast_add_device_clpc(u8 chip, u32 lpc_base);
extern void __init ast_add_device_ci2c(u8 chip, u32 lpc_base);

#endif
