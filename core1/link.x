MEMORY {
    /* Starts 256K in. Must match xtask/src/main.rs and core0/memory.x */
    FLASH : ORIGIN = 0x10000000 + 256K, LENGTH = 2048K - 256K
    /* Separate from the banks Core 0 is using. This is Bank 4. We put the stack in Bank 5 */
    RAM   : ORIGIN = 0x20041000 - 20K, LENGTH = 20K
}

EXTERN(ENTRY_POINT);
EXTERN(reset_func);
ENTRY(reset_func);

SECTIONS {
    PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

    /* EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE */
    /* EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE */
    /* Use the cortex-m-rt crate to give us a proper vector table and init code.                     */
    /* Then our interrupts might work...                                                             */
    /* Core 0 can read our reset vector and stack pointer out of the vector table                    */
    /* EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE */
    /* EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE */

    .entry_point ORIGIN(FLASH) :
    {
        KEEP(*(.entry_point));
    } > FLASH

    /* ### Vector table */
    .vector_table ORIGIN(FLASH) :
    {
        /* Initial Stack Pointer (SP) value */
        LONG(_stack_start);

        /* Reset vector */
        KEEP(*(.vector_table.reset_vector)); /* this is the `__RESET_VECTOR` symbol */
        __reset_vector = .;

        /* Exceptions */
        KEEP(*(.vector_table.exceptions)); /* this is the `__EXCEPTIONS` symbol */
        __eexceptions = .;

        /* Device specific interrupts */
        KEEP(*(.vector_table.interrupts)); /* this is the `__INTERRUPTS` symbol */
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
