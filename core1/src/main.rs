//! The firmware that runs on Core 1
//!
//! This core assumes that all the GPIO set-up has been done on Core 0 already.
//!
//! All this firmware does is render pixels into scan-line buffers, and push
//! them out using PIO0.

#![no_std]
#![no_main]

mod vga;

use rp2040_hal::{self as hal, pac};

// This is defined in global_asm
extern "C" {
    fn reset_func() -> !;
}

/// This variable is what Core 0 uses to boot Core 1
#[no_mangle]
#[link_section = ".entry_point"]
pub static ENTRY_POINT: unsafe extern "C" fn() -> ! = reset_func;

// Code to init .bss and .data, and jump to rust_main
core::arch::global_asm!(
    ".cfi_sections .debug_frame",
    ".section .text, \"ax\"",
    ".global .reset_func",
    ".type reset_func,%function",
    ".thumb_func",
    ".cfi_startproc",
    "reset_func:",
    "   ldr r0, =__sbss",
    "   ldr r1, =__ebss",
    "   movs r2, #0",
    "   0:",
    "      cmp r1, r0",
    "      beq 1f",
    "      stm r0!, {{r2}}",
    "      b 0b",
    "   1:",
    "   ldr r0, =__sdata",
    "   ldr r1, =__edata",
    "   ldr r2, =__sidata",
    "   0:",
    "      cmp r1, r0",
    "      beq 1f",
    "      ldm r2!, {{r3}}",
    "      stm r0!, {{r3}}",
    "      b 0b",
    "   1:",
    "   bl rust_main",
    "   udf #0",
    ".cfi_endproc",
    ".size reset_func, . - reset_func",
);

/// The entry point to this firmare
///
/// # Safety
///
/// Do not call this function manually - only let the Boot ROM call this
/// function.
#[no_mangle]
pub unsafe fn rust_main() -> ! {
    // Note: Core 0 is going to be using most of these so we must be careful.
    let periph = pac::Peripherals::take().unwrap();
    // only keep the FIFO - we don't want to touch anything else
    let sio = hal::sio::Sio::new(periph.SIO);
    let fifo = sio.fifo;
    _ = sio;
    // some other things we need
    let pio = periph.PIO0;
    let dma = periph.DMA;
    let mut resets = periph.RESETS;
    // set up our vector table (the boot ROM doesn't do this for us)
    periph.PPB.vtor().write(|w| w.tbloff().bits(0x2000));
    // don't touch anything else
    _ = periph;

    vga::init(pio, dma, &mut resets);

    // We are on Core 1, so these interrupts will run on Core 1
    unsafe {
        cortex_m::peripheral::NVIC::unpend(crate::pac::Interrupt::PIO0_IRQ_1);
        cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::PIO0_IRQ_1);
        core::arch::asm!("cpsie i");
    }

    vga::render_loop(fifo);
}

#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// End of file
