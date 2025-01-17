MEMORY {
    /* Starts 256K in. Must match xtask/src/main.rs and core0/memory.x */
    FLASH : ORIGIN = 0x10040000, LENGTH = 0x40000
    /* Separate from the memory Core 0 is using. This is Bank 4 and 5. Stack is provided by Core 0. */
    RAM   : ORIGIN = 0x2003E000, LENGTH = 0x4000
}

PROVIDE(FLASH_ORIGIN = ORIGIN(FLASH));
