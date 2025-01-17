//! The firmware that runs on Core 1
//!
//! This core assumes that all the GPIO set-up has been done on Core 0 already.
//!
//! All this firmware does is render pixels into scan-line buffers, and push
//! them out using PIO0.

#![no_std]
#![no_main]

mod vga;

use cortex_m_rt::exception;
use rp2040_hal::{self as hal, pac};

extern "C" {
    static FLASH_ORIGIN: u32;
}

/// The entry point to this firmare
///
/// # Safety
///
/// Do not call this function manually - only let the Boot ROM call this
/// function.
#[cortex_m_rt::entry]
fn main() -> ! {
    // Note: Core 0 is going to be using most of these so we must be careful.
    let periph = pac::Peripherals::take().unwrap();
    // only keep the FIFO - we don't want to touch anything else
    let sio = hal::sio::Sio::new(periph.SIO);
    let fifo = sio.fifo;
    _ = sio;
    // some other things we need
    let pio = periph.PIO0;
    let dma = periph.DMA;
    let ppb = periph.PPB;
    let mut resets = periph.RESETS;
    // don't touch anything else
    _ = periph;

    // set up our vector table (we have our own, different to Core 0)
    ppb.vtor().write(|w| {
        unsafe {
            w.bits(core::ptr::addr_of!(FLASH_ORIGIN) as u32);
        }
        w
    });

    vga::init(pio, dma, &mut resets);
    // alt_init(pio, dma, &mut resets);

    // We are on Core 1, so these interrupts will run on Core 1
    unsafe {
        cortex_m::peripheral::NVIC::unpend(crate::pac::Interrupt::PIO0_IRQ_1);
        cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::PIO0_IRQ_1);
        cortex_m::interrupt::enable();
    }

    vga::render_loop(fifo);
}

#[exception(trampoline = true)]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    fifo_write(0xDDDD_0003);
    fifo_write(frame.r0());
    fifo_write(frame.r1());
    fifo_write(frame.r2());
    fifo_write(frame.r3());
    fifo_write(frame.r12());
    fifo_write(frame.lr());
    fifo_write(frame.pc());
    loop {}
}

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    if let Some(location) = info.location() {
        fifo_write(0xDDDD_0001);
        fifo_write(location.line());
    } else {
        fifo_write(0xDDDD_0002);
    }
    loop {}
}

fn fifo_ready() -> bool {
    let sio = unsafe { &(*pac::SIO::ptr()) };
    sio.fifo_st().read().rdy().bit_is_set()
}

pub fn fifo_write(value: u32) {
    while !fifo_ready() {
        core::hint::spin_loop();
    }

    let sio = unsafe { &(*pac::SIO::ptr()) };
    sio.fifo_wr().write(|w| unsafe { w.bits(value) });
    // Fire off an event to the other core.
    // This is required as the other core may be `wfe` (waiting for event)
    unsafe {
        core::arch::asm!("sev");
    }
}

// End of file
