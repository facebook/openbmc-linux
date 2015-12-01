#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/entry-macro.S>
#include <mach/platform.h>

/*
 * Register usage
 * r8  - temporary
 * r9  - the driver buffer
 * r10 - temporary
 * r11 - interrupts mask
 * r12 - base pointers
 * r13 - interrupts status
 */

	.text

	.global	ast_fiq_end
ENTRY(ast_fiq_start)
	subs    pc, lr, #4
ast_fiq_end:

/*
 * Check the size of the FIQ,
 * it cannot go beyond 0xffff0200, and is copied to 0xffff001c
 */
.if (ast_fiq_end - ast_fiq_start) > (0x200 - 0x1c)
        .err
.endif

