MEMORY {
    /* Starts 256K in. Must match xtask/src/main.rs and core0/memory.x */
    FLASH : ORIGIN = 0x10000000 + 256K, LENGTH = 2048K - 256K
    /* Separate from the banks Core 0 is using. This is Bank 4. We put the stack in Bank 5 */
    RAM   : ORIGIN = 0x20040000, LENGTH = 4K
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

/* End of file */
