MEMORY {
    /* Top 1MB on a 2MB chip */
    FLASH : ORIGIN = 0x10100000, LENGTH = 1024K
    /* Top two banks, separate from the banks Core 0 is using */
    RAM   : ORIGIN = 0x20040000, LENGTH = 8K
}

EXTERN(ENTRY_POINT);
EXTERN(reset_func);
ENTRY(reset_func);

SECTIONS {
    PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

    .entry_point ORIGIN(FLASH) :
    {
        KEEP(*(.entry_point));
    } > FLASH

    .text :
    {
        *(.text .text.*);
    } > FLASH

    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*);
    } > FLASH

    .data : ALIGN(4)
    {
        . = ALIGN(4);
        __sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
        __edata = .;
    } > RAM AT>FLASH
    __sidata = LOADADDR(.data);

    .bss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __sbss = .;
        *(.bss .bss.*);
        . = ALIGN(4);
        __ebss = .;
    } > RAM

    .uninit (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.uninit .uninit.*);
        . = ALIGN(4);
    } > RAM

    .got (NOLOAD) :
    {
        KEEP(*(.got .got.*));
    }

    /DISCARD/ :
    {
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
    }
}