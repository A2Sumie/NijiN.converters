/* Application Flash and SRAM Linker Script */
ENTRY(Reset_Handler)

MEMORY
{
    APP    (rx)  : ORIGIN = 0x00010000, LENGTH = 0x00050000
    SRAMH  (rwx) : ORIGIN = 0x1FFF8000, LENGTH = 0x00008000
    SRAM1  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00010000
    SRAM2  (rw)  : ORIGIN = 0x20010000, LENGTH = 0x00010000
    SRAM3  (rw)  : ORIGIN = 0x20020000, LENGTH = 0x00007000
    RETRAM (rw)  : ORIGIN = 0x200F0000, LENGTH = 0x00001000
}

__sramh_origin   = ORIGIN(SRAMH);
__sramh_end_init = 0x1FFFFF00;
__sram1_origin   = ORIGIN(SRAM1);
__sram1_end      = ORIGIN(SRAM1) + LENGTH(SRAM1);
__sram2_origin   = ORIGIN(SRAM2);
__sram2_end      = ORIGIN(SRAM2) + LENGTH(SRAM2);
__sram3_origin   = ORIGIN(SRAM3);
__sram3_end      = ORIGIN(SRAM3) + LENGTH(SRAM3);

__stack_bottom   = 0x1FFFE000;
__stack_top      = 0x1FFFFF00;

SECTIONS
{
    /* 1. Initial 160-word Vector Table + early startup + pad to version anchor */
    .isr_vector 0x00010000 :
    {
        FILL(0xFF);
        . = ALIGN(4);
        __isr_vector_start = .;
        KEEP(*(.isr_vector))
        __isr_vector_end = .;
        KEEP(*(.startup_text*))
        ASSERT(. <= 0x1000, "Startup code overflowed into 0x00011000 .readme_ver slot");
        . = 0x1000;
    } > APP

    /* 2. Fixed README.TXT version anchor */
    .readme_ver 0x00011000 :
    {
        __readme_ver_start = .;
        KEEP(*(.readme_ver))
        __readme_ver_end = .;
    } > APP

    /* 3. Main Flash Code & Read-Only Lens/Optics Constants */
    .text :
    {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame*)
        . = ALIGN(4);
        __text_end = .;
    } > APP

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > APP

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > APP

    /* 4. SRAMH Partition 1: 1024B-aligned Runtime Vector Table */
    .ram_vectors 0x1FFF8000 (NOLOAD) :
    {
        . = ALIGN(1024);
        __ram_vectors_start = .;
        KEEP(*(.ram_vectors))
        . = 0x400;
        __ram_vectors_end = .;
    } > SRAMH

    /* 5. SRAMH Partition 2: Zero-Wait .ramfunc (ISRs + EFM + SPI3 window switch) */
    __ramfunc_load = LOADADDR(.ramfunc);
    .ramfunc 0x1FFF8400 :
    {
        . = ALIGN(4);
        __ramfunc_start = .;
        KEEP(*(.ramfunc*))
        . = ALIGN(4);
        /* 3-word prefetch safety guard initialized in SRAMH */
        LONG(0x00000000);
        LONG(0x00000000);
        LONG(0x00000000);
        __ramfunc_end = .;
    } > SRAMH AT > APP

    ASSERT(__ramfunc_end <= 0x1FFFA800, ".ramfunc exceeded 9216B budget in SRAMH")

    /* 6. SRAMH Partition 3: 4B-aligned AHB DMA Dedicated Buffers */
    .dma_sramh 0x1FFFA800 (NOLOAD) :
    {
        . = ALIGN(4);
        __dma_sramh_start = .;
        *(.dma_sramh*)
        . = ALIGN(4);
        __dma_sramh_end = .;
    } > SRAMH

    ASSERT(__dma_sramh_end <= __stack_bottom, "DMA buffers collided with MSP stack bottom 0x1FFFE000")
    ASSERT(__stack_top == 0x1FFFFF00, "MSP top must leave 256B guard band below 0x20000000")

    /* 7. SRAM1: Initialized .data and Zeroed .bss */
    __data_load = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > SRAM1 AT > APP

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > SRAM1

    /* 8. SRAM2: Optics Double-Buffered Descriptors + Breathing Dataset + 36KiB LF Workspace */
    .optics_sram2 (NOLOAD) :
    {
        . = ALIGN(4);
        __optics_sram2_start = .;
        *(.optics_sram2*)
        . = ALIGN(4);
        __optics_sram2_end = .;
    } > SRAM2

    /* 9. SRAM3: 28KiB ECC-Protected Diagnostic Ring */
    .diag_sram3 (NOLOAD) :
    {
        . = ALIGN(4);
        __diag_sram3_start = .;
        KEEP(*(.diag_sram3*))
        . = ALIGN(4);
        __diag_sram3_end = .;
    } > SRAM3

    /* 10. Ret_SRAM: 4KiB Warm-Reset State Block */
    .retram_noload (NOLOAD) :
    {
        . = ALIGN(4);
        __retram_start = .;
        *(.retram_noload*)
        . = ALIGN(4);
        __retram_end = .;
    } > RETRAM

    __flash_image_end = LOADADDR(.data) + SIZEOF(.data);

    /* Hard Static Contract Assertions */
    ASSERT(__isr_vector_end - __isr_vector_start == 640, "Initial vector table must be 160 words (640B)")
    ASSERT(ADDR(.readme_ver) == 0x00011000, ".readme_ver must be anchored at 0x00011000")
    ASSERT(LOADADDR(.readme_ver) == 0x00011000, ".readme_ver LMA must equal 0x00011000")
    ASSERT(__flash_image_end <= 0x0005FFE0, "Flash payload exceeds 0x0005FFE0 (violates 0x60000 FS guard)")
    ASSERT(__bss_end <= 0x20010000, "SRAM1 overflow")
    ASSERT(__optics_sram2_end <= 0x20020000, "SRAM2 overflow")
    ASSERT(__diag_sram3_end <= 0x20027000, "SRAM3 overflow into unmapped RAM hole [0x20027000, 0x200F0000)")
    ASSERT(SIZEOF(.diag_sram3) >= 23552, "Diagnostic + V2.24 trace SRAM3 storage was eliminated")
    ASSERT(__retram_start >= 0x200F0000 && __retram_end <= 0x200F1000, "Ret_SRAM out of bounds")
}
