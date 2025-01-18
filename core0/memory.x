MEMORY {
    /* Start of Flash */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /* After Boot2. */
    /* Core 1 at the 256K mark (which must match xtask/src/main.rs and core1/link.x) */
    FLASH : ORIGIN = 0x10000100, LENGTH = 0x3FF00
    /* This is Banks 0-3, minus 8K at the end */
    RAM   : ORIGIN = 0x20000000, LENGTH = 0x3E000
}

EXTERN(BOOT2_FIRMWARE);

PROVIDE(CORE1_VECTOR_TABLE = ORIGIN(FLASH) + LENGTH(FLASH));

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

SECTIONS {
    /* ### Boot ROM info
     *
     * Goes after .vector_table, to keep it in the first 512 bytes of flash,
     * where picotool can find it
     */
    .boot_info : ALIGN(4)
    {
        KEEP(*(.boot_info));
    } > FLASH

} INSERT AFTER .vector_table;

/* move .text to start /after/ the boot info */
_stext = ADDR(.boot_info) + SIZEOF(.boot_info);

SECTIONS {
    /* ### Picotool 'Binary Info' Entries
     *
     * Picotool looks through this block (as we have pointers to it in our
     * header) to find interesting information.
     */
    .bi_entries : ALIGN(4)
    {
        /* We put this in the header */
        __bi_entries_start = .;
        /* Here are the entries */
        KEEP(*(.bi_entries));
        /* Keep this block a nice round size */
        . = ALIGN(4);
        /* We put this in the header */
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

/* End of file */
