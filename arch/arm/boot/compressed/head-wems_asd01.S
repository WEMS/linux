/* 
 * linux/arch/arm/boot/compressed/head-wems_asd01.S
 * 
 * Copyright (C) 2013 Wireless Energy Management Systems International Ltd.
 * Author: Guy Thouret <guythouret@wems.co.uk>
 * 
 * Some code borrowed from Nicolas Pitre's 'head-sa1100.S' file. This
 * is merged with head.S by the linker.
 */

#include <linux/config.h>
#include <asm/mach-types.h>

        .section        ".start", "ax"

__WEMS_ASD01_start:
        b       4f

1:      .word   MACH_TYPE_WEMS_LINUX	/* Machine type */
2:      .word   0x10027000		/* PLL regs phys addr */
3:      .word   0x2e000000		/* Value for PCCR1 - enable GPT clocks */

4:
        ldr     r1, 2b
        ldr     r2, 3b
        str     r2, [r1, #36]		/* Write to PCCR1 */

        ldr     r7, 1b			/* Set machine type */

