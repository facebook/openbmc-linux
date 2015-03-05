/*
 * entry-macro.S
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/regs-intr.h>

		.macro	disable_fiq
		.endm

		.macro	get_irqnr_preamble, base, tmp
		ldr	\base, =IO_ADDRESS(AST_VIC_BASE)
		.endm

		.macro	arch_ret_to_user, tmp1, tmp2
		.endm
#if 1

#if defined(NEW_VIC)
		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		
		ldr	\tmp, =IO_ADDRESS(AST_SCU_BASE)
		ldr	\irqnr, [\tmp, #0x44]

		cmp	\irqnr, #0
		beq	2000f
		
1000:           /* pass1 */
                cmp	\irqnr, #32
                ble	1001f
		ldr	\tmp, =IO_ADDRESS(AST_VIC_BASE)
		ldr	\irqstat, [\tmp, #0x84]
                sub	\irqnr, \irqnr, #32		
		mov	\tmp, #32
		sub	\tmp, \tmp, \irqnr
                mov     \irqstat, \irqstat, lsl \tmp	/* mask uncompare parts */
                mov     \irqstat, \irqstat, lsr \tmp
                mov	\irqnr, #63
                clz     \tmp, \irqstat
                cmp     \tmp, #32
                bne     3000f  
                mov	\irqnr, #32                
1001:
		ldr	\tmp, =IO_ADDRESS(AST_VIC_BASE)
		ldr	\irqstat, [\tmp, #0x00]
		mov	\tmp, #32
		sub	\tmp, \tmp, \irqnr		
                mov     \irqstat, \irqstat, lsl \tmp	/* mask uncompare parts */
                mov     \irqstat, \irqstat, lsr \tmp
                mov	\irqnr, #31                
                clz     \tmp, \irqstat
                cmp     \tmp, #32
                bne	3000f
                
2000:           /* pass 2 */
		ldr	\tmp, =IO_ADDRESS(AST_VIC_BASE)
		ldr	\irqstat, [\tmp, #0x84]
                mov	\irqnr, #63
                clz     \tmp, \irqstat
                cmp     \tmp, #32
                bne     3000f  
2001:                
		ldr	\tmp, =IO_ADDRESS(AST_VIC_BASE)
		ldr	\irqstat, [\tmp, #0x00]
                mov	\irqnr, #31		
                clz     \tmp, \irqstat
                cmp     \tmp, #32
                beq     4000f			/* not find */  
                                
3000:		/* find */
		sub	\irqnr, \irqnr, \tmp
		ldr	\tmp, =IO_ADDRESS(AST_SCU_BASE)
		str	\irqnr, [\tmp, #0x44]
                cmp     \irqnr, #64
4000:		/* done */                
		.endm
#else
		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
/* FIXME: should not be using soo many LDRs here */
		ldr	\irqnr, =IO_ADDRESS(AST_VIC_BASE)
		ldr	\irqstat, [\irqnr, #ASPEED_VIC_STATUS_OFFSET]		@ get masked status

		mov	\irqnr, #0
1001:		tst	\irqstat, #1
		bne	1002f
		add	\irqnr, \irqnr, #1
		mov	\irqstat, \irqstat, lsr #1
		cmp	\irqnr, #31
		bcc	1001b
1002:		/* EQ will be set if we reach 31 */
		.endm
		
#endif
#else

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

		/*********************************************/
		/* load VIC (VIC1) status value into irqstat */
		/*********************************************/
		ldr		\irqnr, 	=IO_ADDRESS(MVP2_VIC_BASE)
		ldr		\irqstat,	[\irqnr, #MVP2_VIC_STATUS_OFFSET]


		/**********************************************/
		/* check each status bit and start from bit 0 */
		/**********************************************/
		mov		\irqnr,		#0
1000:	tst		\irqstat,	#1		/* Check irqstat[irqnr] is 1 or not.  */
		bne		1100f				/* If irqstat[irqnr] is 1, service    */
									/* this interrupt.                    */
1001:
		/* check next bit */
		add		\irqnr,		\irqnr,		#1
		mov		\irqstat,	\irqstat,	lsr #1

		cmp		\irqnr,		#32	/* If irqnr is number 32, all bits on VIC1 */
		beq		1300f			/* have been checked and leave this macro. */
								/* Note that the Zero bit should be 1.     */

		bne		1000b			/* continue to check next bit */



1100:	ldr		\irqstat,	=INT_VIC2IRQ		/* interrupt from VIC2? */
		cmp		\irqnr,		\irqstat

		bne		1300f		/* Interupt isn't from VIC2 (i.e. irqnr != INT_VIC2IRQ). */
							/* Leave this macro with irqnr isn't changed and keep Zero */
							/* flag not set */


		/***************************************/
		/* load VIC2 status value into irqstat */
		/***************************************/
#if 0
		ldr		\irqnr, 	=IO_ADDRESS(MVP2_VIC2_BASE)
		ldr		\irqstat,	[\irqnr, #MVP2_VIC_STATUS_OFFSET]
#else
		ldr		\irqnr, 	=IO_ADDRESS(MVP2_VIC_BASE)
		ldr		\irqstat,	=0x1000
		add		\irqnr,		\irqnr,		\irqstat
		ldr		\irqstat,	[\irqnr, #MVP2_VIC_STATUS_OFFSET]
#endif

		/***********************************************/
		/* Check each status bit and start from bit 0. */
		/* Note that bit 0 in VIC2 is IRQ number 32.   */
		/***********************************************/
		mov		\irqnr,		#32
1200:	tst		\irqstat,	#1
		bne		1300f


		/* check next bit */
		add		\irqnr,		\irqnr,		#1
		mov		\irqstat,	\irqstat,	lsr #1

		cmp		\irqnr,		#64		/* If irqnr isn't reach 64 */
		bne		1200b				/* continue check irqstat. */



		/*************************************************************************/
		/* Examine all the other interrupt bits larger than INT_VIC2IRQ on VIC1. */
		/*************************************************************************/
		ldr		\irqnr, 	=IO_ADDRESS(MVP2_VIC_BASE)
		ldr		\irqstat,	[\irqnr, #MVP2_VIC_STATUS_OFFSET]
		mov		\irqnr,		#INT_VIC2IRQ
		mov		\irqstat,	\irqstat,	lsr #INT_VIC2IRQ
		b		1001b

		/* TODO : if needed */
		/* All interrupt bits on VIC2 have been checked and no bit with value */
		/* 1 is found. Write 1 to EdgeClearReg[INT_VIC2IRQ] to clear interrupt. */

1300:
		.endm

#endif



		.macro	irq_prio_table
		.endm

