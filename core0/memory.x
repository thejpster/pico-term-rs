MEMORY {
    /* Start of Flash */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /* After Boot2. */
    /* Core 1 at the 256K mark (which must match xtask/src/main.rs and core1/link.x) */
    FLASH : ORIGIN = 0x10000100, LENGTH = 256K - 0x100
    /* Only banks 0 to 3 - banks 4 and 5 belong the other core */
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
    /* Bank 5 */
    CORE1_STACK: ORIGIN = 0x20041000, LENGTH = 4K
}

EXTERN(BOOT2_FIRMWARE);

PROVIDE(CORE1_ENTRY = ORIGIN(FLASH) + LENGTH(FLASH));
PROVIDE(CORE1_STACK_BOTTOM = ORIGIN(CORE1_STACK));
PROVIDE(CORE1_STACK_TOP = ORIGIN(CORE1_STACK) + LENGTH(CORE1_STACK));

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

/* End of file */
