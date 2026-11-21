    .syntax unified
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .thumb

/* =========================================================================
 * 1. Initial 160-Word Application Flash Vector Table
 * ========================================================================= */
    .section .isr_vector, "a", %progbits
    .align 2
    .global g_flash_vectors
g_flash_vectors:
    .word __stack_top                 /* 00: Initial MSP */
    .word Reset_Handler               /* 01: Reset Handler */
    .word NMI_Handler                 /* 02: NMI */
    .word HardFault_Handler           /* 03: HardFault */
    .word Default_Handler             /* 04: MemManage */
    .word Default_Handler             /* 05: BusFault */
    .word Default_Handler             /* 06: UsageFault */
    .word 0                           /* 07: Reserved */
    .word 0                           /* 08: Reserved */
    .word 0                           /* 09: Reserved */
    .word 0                           /* 10: Reserved */
    .word Default_Handler             /* 11: SVCall */
    .word Default_Handler             /* 12: DebugMon */
    .word 0                           /* 13: Reserved */
    .word Default_Handler             /* 14: PendSV */
    .word Default_Handler             /* 15: SysTick */

    /* External Interrupts IRQ0 .. IRQ39 (40 words) */
    .rept 40
    .word Default_Handler
    .endr

    /* IRQ40: SPI1 TX DMA2_TC0 (INT_SEL40 = 36) */
    .word SPI1_DMA2_TC0_IRQHandler
    /* IRQ41: USART1 TX DMA2_TC3 (INT_SEL41 = 39) */
    .word USART1_DMA2_TC3_IRQHandler

    /* External Interrupts IRQ42 .. IRQ79 (38 words) */
    .rept 38
    .word Default_Handler
    .endr

    /* IRQ80: E4 VSync TMRA4_OVF (INT_SEL80 = 265) */
    .word TMRA4_OVF_IRQHandler
    /* IRQ81: Unused */
    .word Default_Handler
    /* IRQ82: 64-bit Timebase TMRA1_OVF (INT_SEL82 = 256) */
    .word TMRA1_OVF_IRQHandler

    /* External Interrupts IRQ83 .. IRQ143 (61 words) */
    .rept 61
    .word Default_Handler
    .endr

/* =========================================================================
 * 2. Bootloader Version Stamp (V2.24 2026-11)
 * ========================================================================= */
    .section .readme_ver, "a", %progbits
    .global g_readme_version_stamp
g_readme_version_stamp:
    .byte 0x18, 0x02, 0x0B, 0x1A

/* =========================================================================
 * 3. Runtime 1024B-Aligned RAM Vector Table in SRAMH
 * ========================================================================= */
    .section .ram_vectors, "aw", %nobits
    .align 10
    .global g_ram_vectors
g_ram_vectors:
    .space 640

/* =========================================================================
 * 4. Cold-Boot Reset_Handler
 * ========================================================================= */
    .section .startup_text, "ax", %progbits
    .align 2
    .global Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    cpsid   i

    /* Stop unused XTAL32 oscillator at first Reset_Handler entry */
    ldr     r0, =0x400543FE           /* PWC_FPRC */
    movw    r1, #0xA501
    strh    r1, [r0]
    ldr     r2, =0x40054420           /* CMU_XTAL32CR */
    movs    r1, #1
    strb    r1, [r2]
    movw    r1, #0xA500
    strh    r1, [r0]

    /* Set MSP leaving 256-byte guard band below SRAMH upper boundary */
    ldr     r0, =__stack_top
    msr     msp, r0

    /* Stop SysTick and clear any stale NVIC enables/pendings inherited from Bootloader */
    ldr     r0, =0xE000E010           /* SysTick->CTRL */
    movs    r1, #0
    str     r1, [r0]
    ldr     r0, =0xE000E180           /* NVIC->ICER[0..4] */
    ldr     r2, =0xE000E280           /* NVIC->ICPR[0..4] */
    mvn     r1, #0
    movs    r3, #5
0:
    str     r1, [r0], #4
    str     r1, [r2], #4
    subs    r3, r3, #1
    bne     0b

    /* Enable SRAMH/SRAM12/SRAM3/RetSRAM clocks in PWC_FCG0 before parity/ECC init */
    ldr     r0, =0x40048010           /* PWC_FCG0PC */
    ldr     r1, =0xA5A50001
    str     r1, [r0]
    ldr     r2, =0x40048000           /* PWC_FCG0 */
    ldr     r3, [r2]
    ldr     r1, =0xFFFFFAEE           /* Clear bit0(SRAMH), bit4(SRAM12), bit8(SRAM3), bit10(Ret) */
    ands    r3, r3, r1
    str     r3, [r2]
    ldr     r1, =0xA5A50000
    str     r1, [r0]

    /* Pre-zero upper stack window before calling early 192 MHz switch */
    ldr     r0, =0x1FFFE000
    ldr     r1, =0x1FFFFF00
    movs    r2, #0
    movs    r3, #0
    movs    r4, #0
    movs    r5, #0
1:
    cmp     r0, r1
    bhs     2f
    stmia   r0!, {r2, r3, r4, r5}
    b       1b
2:
    sub     sp, sp, #32
    mov     r0, sp
    bl      sys_clock_early_192mhz

    /* Zero lower high-speed SRAM plus banks 1..3 at 192 MHz */
    movs    r2, #0
    movs    r3, #0
    movs    r4, #0
    movs    r5, #0
    ldr     r0, =0x1FFF8000
    ldr     r1, =0x1FFFE000
21:
    cmp     r0, r1
    bhs     22f
    stmia   r0!, {r2, r3, r4, r5}
    b       21b
22:
    ldr     r0, =0x20000000
    ldr     r1, =0x20027000
23:
    cmp     r0, r1
    bhs     24f
    stmia   r0!, {r2, r3, r4, r5}
    b       23b
24:

    /* Copy 160-word vector table from Flash to fast SRAM */
    ldr     r0, =__isr_vector_start
    ldr     r1, =__ram_vectors_start
    movs    r2, #160
3:
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    subs    r2, r2, #1
    bne     3b

    /* Copy .ramfunc (ISRs + flash + fast handlers + 3-word prefetch pad) */
    ldr     r0, =__ramfunc_load
    ldr     r1, =__ramfunc_start
    ldr     r2, =__ramfunc_end
4:
    cmp     r1, r2
    bhs     5f
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    b       4b
5:

    /* Copy .data to bank 1 */
    ldr     r0, =__data_load
    ldr     r1, =__data_start
    ldr     r2, =__data_end
6:
    cmp     r1, r2
    bhs     7f
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    b       6b
7:

    mov     r0, sp
    bl      sys_clock_finish_boot_state
    add     sp, sp, #32

    /* Point vector offset to application Flash table */
    ldr     r0, =0xE000ED08
    ldr     r1, =__isr_vector_start
    str     r1, [r0]
    dsb     sy
    isb     sy

    /* Enable hardware FPU coprocessor full access + ASPEN/LSPEN lazy stacking */
    ldr     r0, =0xE000ED88           /* SCB->CPACR */
    ldr     r1, [r0]
    orr     r1, r1, #(0xF << 20)
    str     r1, [r0]
    ldr     r0, =0xE000EF34           /* FPU->FPCCR */
    ldr     r1, [r0]
    orr     r1, r1, #0xC0000000
    str     r1, [r0]
    dsb     sy
    isb     sy

    /* Enter C firmware superloop */
    bl      boryoza_main

10:
    b       10b
    .size Reset_Handler, . - Reset_Handler

/* =========================================================================
 * 5. SRAMH Zero-Wait Exception & Default Handlers (Zero FPU Instructions)
 * ========================================================================= */
    .section .ramfunc, "ax", %progbits
    .align 2
    .global NMI_Handler
    .type NMI_Handler, %function
NMI_Handler:
    ldr     r3, =g_nmi_snapshot
    ldr     r0, =0x40050810
    ldr     r1, [r0]
    str     r1, [r3]
    ldr     r0, =0x40051008
    ldr     r1, [r0]
    str     r1, [r3, #4]
    ldr     r0, =0x40050808
    ldr     r1, [r0]
    str     r1, [r3, #12]
    ldr     r0, =0x40051004
    ldr     r1, [r0]
    str     r1, [r3, #16]
    ldr     r1, [r3, #8]
    adds    r1, r1, #1
    str     r1, [r3, #8]
    /* Clear any latched SRAMC parity/ECC status & INTC_NMICFR so NMI never re-enters */
    ldr     r0, =0x4005080C           /* SRAMC_CKPR */
    movs    r1, #0x77
    str     r1, [r0]
    ldr     r2, =0x40050810           /* SRAMC_CKSR */
    movs    r1, #0x1F
    str     r1, [r2]
    movs    r1, #0x76
    str     r1, [r0]
    ldr     r0, =0x4005100C           /* INTC_NMICFR */
    movw    r1, #0x0F2F
    str     r1, [r0]
    dsb     sy
    bx      lr
    .size NMI_Handler, . - NMI_Handler

    .align 2
    .global HardFault_Handler
    .type HardFault_Handler, %function
HardFault_Handler:
    tst     lr, #4
    ite     eq
    mrseq   r0, msp
    mrsne   r0, psp
    b       boryoza_hardfault_c_handler
    .size HardFault_Handler, . - HardFault_Handler

    .align 2
    .global Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b       Default_Handler
    .size Default_Handler, . - Default_Handler
