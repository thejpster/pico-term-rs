//! # pico-term-rs
//!
//! Rust Firmware for the RC2014 RP2040 VGA Terminal.
//!
//! This is the firmware for Core 0.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

unsafe extern "C" {
    /// This variable contains the address of the entry function
    safe static CORE1_ENTRY: usize;
    /// The top of Core 1's stack
    safe static mut CORE1_STACK_TOP: usize;
    /// The bottom of Core 1's stack
    safe static mut CORE1_STACK_BOTTOM: usize;
}

#[hal::entry]
fn main() -> ! {
    defmt::info!(
        "Firmware {} {} starting up",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );

    let mut periph = pac::Peripherals::take().unwrap();
    let cm = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(periph.WATCHDOG);
    let mut sio = Sio::new(periph.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        periph.XOSC,
        periph.CLOCKS,
        periph.PLL_SYS,
        periph.PLL_USB,
        &mut periph.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    defmt::info!("Setting up Core 1");

    let mut multicore =
        hal::multicore::Multicore::new(&mut periph.PSM, &mut periph.PPB, &mut sio.fifo);
    let core1 = &mut multicore.cores()[1];
    let stack_allocation = unsafe {
        hal::multicore::StackAllocation::from_raw_parts(
            core::ptr::addr_of_mut!(CORE1_STACK_BOTTOM),
            core::ptr::addr_of_mut!(CORE1_STACK_TOP),
        )
    };
    defmt::info!("Core 1 entry point is {=usize:08x}", CORE1_ENTRY);
    // off the top of the chip? (or blank...)
    if CORE1_ENTRY > 0x1020_0000 {
        defmt::panic!("Core1 entry address 0x{:08x} is bad?!", CORE1_ENTRY);
    }
    let entry_func: fn() -> ! = unsafe { core::mem::transmute(CORE1_ENTRY) };
    defmt::info!("Spawning Core 1...");
    core1
        .spawn(stack_allocation, move || entry_func())
        .expect("Spawning Core 1");

    let mut delay = cortex_m::delay::Delay::new(cm.SYST, clocks.system_clock.freq().to_Hz());

    info!("Looping...");

    loop {
        for i in [10, 100, 250] {
            delay.delay_ms(2000);
            info!("Core 0 writing {=u32}", i);
            sio.fifo.write(i);
        }
    }
}

// End of file
