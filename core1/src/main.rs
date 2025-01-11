//! The firmware that runs on Core 1

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embedded_hal::digital::OutputPin as _;
use rp2040_hal::{self as hal, pac};

// This is defined in global_asm
extern "C" {
    fn reset_func() -> !;
}

/// This variable is what Core 0 uses to boot Core 1
#[no_mangle]
#[link_section = ".entry_point"]
pub static ENTRY_POINT: unsafe extern "C" fn() -> ! = reset_func;

#[used]
static COUNTER: AtomicU32 = AtomicU32::new(500);

static SYSTEM_CLOCK_SPEED: u32 = 125_000_000;

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
    let mut periph = pac::Peripherals::take().unwrap();
    let cm = cortex_m::Peripherals::take().unwrap();
    let mut sio = hal::sio::Sio::new(periph.SIO);
    let pins = hal::gpio::Pins::new(
        periph.IO_BANK0,
        periph.PADS_BANK0,
        sio.gpio_bank0,
        &mut periph.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut delay = cortex_m::delay::Delay::new(cm.SYST, SYSTEM_CLOCK_SPEED);

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(COUNTER.load(Ordering::Relaxed));
        if let Some(x) = sio.fifo.read() {
            COUNTER.store(x, Ordering::Relaxed);
        }
        led_pin.set_low().unwrap();
        delay.delay_ms(COUNTER.load(Ordering::Relaxed));
        if let Some(x) = sio.fifo.read() {
            COUNTER.store(x, Ordering::Relaxed);
        }
    }
}

#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// End of file
