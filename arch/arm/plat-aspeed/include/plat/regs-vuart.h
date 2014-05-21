/* arch/arm/plat-aspeed/include/mach/regs-iic.h
 *
 * Copyright (c) 2012 ASPEED Technology Inc. <ryan_chen@aspeedtech.com>
 *		http://www.aspeedtech.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ASPEED VUART Controller
*/

#ifndef __AST_VUART_H_
#define __AST_VUART_H_ 

#define AST_VUART_CTRLA			0x20
#define AST_VUART_CTRLB			0x24
#define AST_VUART_ADDRL			0x28
#define AST_VUART_ADDRH			0x2C
#define AST_VUART_CTRLE			0x30
#define AST_VUART_CTRLF			0x34
#define AST_VUART_CTRLG			0x38
#define AST_VUART_CTRLH			0x3C



/* AST_VUART_CTRLA			0x20 */
#define VUART_ENABLE				(1 << 0)
#define VUART_SIRQ_POLARITY			(1 << 1)
#define VUART_DISABLE_H_TX_DISCARD	(1 << 5)


/* AST_VUART_CTRLB			0x24 */
#define SET_SIRQ_NUM(x)			(x << 4)




#endif 
