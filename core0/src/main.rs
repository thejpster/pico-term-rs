//! # pico-term-rs
//!
//! Rust Firmware for the RC2014 RP2040 VGA Terminal.
//!
//! This is the firmware for Core 0.

#![no_std]
#![no_main]

use core::fmt::Write;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use rp2040_hal::{
    self as hal, binary_info,
    clocks::Clock as _,
    fugit::RateExtU32 as _,
    gpio::{self, bank0, FunctionPio0, FunctionUart, Pin, PullNone},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [binary_info::EntryAddr; 5] = [
    binary_info::rp_program_name!(c"pico-term-rs"),
    binary_info::rp_program_description!(c"Rust firmware for the RC2014 RP2040 VGA Terminal"),
    binary_info::rp_cargo_version!(),
    binary_info::rp_program_url!(c"https://github.com/thejpster/pico-term-rs"),
    binary_info::rp_program_build_attribute!(),
];

/// On-board crystal frequency, in Hz.
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

unsafe extern "C" {
    /// This variable contains the address of the entry function
    safe static CORE1_ENTRY: usize;
    /// The top of Core 1's stack
    safe static mut CORE1_STACK_TOP: usize;
    /// The bottom of Core 1's stack
    safe static mut CORE1_STACK_BOTTOM: usize;
}

struct RedPins {
    /// Red colour data bit 0 (the LSB)
    ///
    /// Also connected to a pull-up button. We should sample this as an input
    /// during the vertical blanking interval.
    _bit0: Pin<bank0::Gpio0, FunctionPio0, PullNone>,
    /// Red colour data bit 1
    _bit1: Pin<bank0::Gpio1, FunctionPio0, PullNone>,
    /// Red colour data bit 2
    _bit2: Pin<bank0::Gpio2, FunctionPio0, PullNone>,
    /// Red colour data bit 3
    _bit3: Pin<bank0::Gpio3, FunctionPio0, PullNone>,
    /// Red colour data bit 4 (the MSB)
    _bit4: Pin<bank0::Gpio4, FunctionPio0, PullNone>,
}

struct GreenPins {
    /// Green colour data bit 0 (the LSB)
    ///
    /// Also connected to a pull-up button. We should sample this as an input
    /// during the vertical blanking interval.
    _bit0: Pin<bank0::Gpio6, FunctionPio0, PullNone>,
    /// Green colour data bit 1
    _bit1: Pin<bank0::Gpio7, FunctionPio0, PullNone>,
    /// Green colour data bit 2
    _bit2: Pin<bank0::Gpio8, FunctionPio0, PullNone>,
    /// Green colour data bit 3
    _bit3: Pin<bank0::Gpio9, FunctionPio0, PullNone>,
    /// Green colour data bit 4 (the MSB)
    _bit4: Pin<bank0::Gpio10, FunctionPio0, PullNone>,
}

struct BluePins {
    /// Blue colour data bit 0 (the LSB)
    ///
    /// Also connected to a pull-up button. We should sample this as an input
    /// during the vertical blanking interval.
    _bit0: Pin<bank0::Gpio11, FunctionPio0, PullNone>,
    /// Blue colour data bit 1
    _bit1: Pin<bank0::Gpio12, FunctionPio0, PullNone>,
    /// Blue colour data bit 2
    _bit2: Pin<bank0::Gpio13, FunctionPio0, PullNone>,
    /// Blue colour data bit 3
    _bit3: Pin<bank0::Gpio14, FunctionPio0, PullNone>,
    /// Blue colour data bit 4 (the MSB)
    _bit4: Pin<bank0::Gpio15, FunctionPio0, PullNone>,
}

struct VgaPins {
    /// Analog value for the red channel, over five bits
    _red_pins: RedPins,
    /// Analog value for the green channel, over five bits
    _green_pins: GreenPins,
    /// Analog value for the blue channel, over five bits
    _blue_pins: BluePins,
    /// Horizontal Sync pin
    _h_sync: Pin<bank0::Gpio16, FunctionPio0, PullNone>,
    /// Vertical Sync pin
    _v_sync: Pin<bank0::Gpio17, FunctionPio0, PullNone>,
}

type UartTx = Pin<bank0::Gpio20, FunctionUart, PullNone>;
type UartRx = Pin<bank0::Gpio21, FunctionUart, PullNone>;
type UartPins = (UartTx, UartRx);

struct Hardware {
    /// Our pins for VGA video output
    ///
    /// These pins are all controlled by PIO0, which is driven from the Core 1
    /// firmware. We just set them up on this side whilst we set up all the
    /// other pins.
    _vga_pins: VgaPins,
    /// Our UART
    uart: hal::uart::UartPeripheral<hal::uart::Enabled, hal::pac::UART1, UartPins>,
}

#[hal::entry]
fn main() -> ! {
    info!(
        "Firmware {} {} starting up",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );

    let mut periph = pac::Peripherals::take().unwrap();
    let cm = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(periph.WATCHDOG);
    let mut sio = Sio::new(periph.SIO);

    info!("Configuring clocks...");

    // Run at 151.2 MHz SYS_PLL, 48 MHz, USB_PLL. This is important, we as clock
    // the PIO at ÷ 6, to give 25.2 MHz (which is close enough to the 25.175
    // MHz standard VGA pixel clock).

    // Step 1. Turn on the crystal.
    let xosc = hal::xosc::setup_xosc_blocking(periph.XOSC, XOSC_CRYSTAL_FREQ.Hz())
        .map_err(|_x| false)
        .unwrap();
    // Step 2. Configure watchdog tick generation to tick over every microsecond.
    watchdog.enable_tick_generation((XOSC_CRYSTAL_FREQ / 1_000_000) as u8);
    // Step 3. Create a clocks manager.
    let mut clocks = hal::clocks::ClocksManager::new(periph.CLOCKS);
    // Step 4. Set up the system PLL.
    //
    // We take the Crystal Oscillator (=12 MHz) with no divider, and ×126 to
    // give a FOUTVCO of 1512 MHz. This must be in the range 750 MHz - 1600 MHz.
    // The factor of 126 is calculated automatically given the desired FOUTVCO.
    //
    // Next we ÷5 on the first post divider to give 302.4 MHz.
    //
    // Finally we ÷2 on the second post divider to give 151.2 MHz.
    //
    // We note from the [RP2040
    // Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf),
    // Section 2.18.2.1:
    //
    // > Jitter is minimised by running the VCO at the highest possible
    // > frequency, so that higher post-divide values can be used.
    let pll_sys = hal::pll::setup_pll_blocking(
        periph.PLL_SYS,
        xosc.operating_frequency(),
        hal::pll::PLLConfig {
            vco_freq: 1512.MHz(),
            refdiv: 1,
            post_div1: 5,
            post_div2: 2,
        },
        &mut clocks,
        &mut periph.RESETS,
    )
    .map_err(|_x| false)
    .unwrap();
    // Step 5. Set up a 48 MHz PLL for the USB system.
    let pll_usb = hal::pll::setup_pll_blocking(
        periph.PLL_USB,
        xosc.operating_frequency(),
        hal::pll::common_configs::PLL_USB_48MHZ,
        &mut clocks,
        &mut periph.RESETS,
    )
    .map_err(|_x| false)
    .unwrap();
    // Step 6. Set the system to run from the PLLs we just configured.
    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .map_err(|_x| false)
        .unwrap();

    info!("Clocks OK!");

    info!("Configuring pins...");
    let hal_pins = gpio::Pins::new(
        periph.IO_BANK0,
        periph.PADS_BANK0,
        sio.gpio_bank0,
        &mut periph.RESETS,
    );
    let mut hw = Hardware {
        _vga_pins: VgaPins {
            _red_pins: RedPins {
                _bit0: {
                    let mut pin = hal_pins.gpio0.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit1: {
                    let mut pin = hal_pins.gpio1.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit2: {
                    let mut pin = hal_pins.gpio2.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit3: {
                    let mut pin = hal_pins.gpio3.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit4: {
                    let mut pin = hal_pins.gpio4.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
            },
            _green_pins: GreenPins {
                _bit0: {
                    let mut pin = hal_pins.gpio6.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit1: {
                    let mut pin = hal_pins.gpio7.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit2: {
                    let mut pin = hal_pins.gpio8.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit3: {
                    let mut pin = hal_pins.gpio9.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit4: {
                    let mut pin = hal_pins.gpio10.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
            },
            _blue_pins: BluePins {
                _bit0: {
                    let mut pin = hal_pins.gpio11.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit1: {
                    let mut pin = hal_pins.gpio12.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit2: {
                    let mut pin = hal_pins.gpio13.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit3: {
                    let mut pin = hal_pins.gpio14.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
                _bit4: {
                    let mut pin = hal_pins.gpio15.reconfigure();
                    pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
                    pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                    pin
                },
            },
            _h_sync: {
                let mut pin = hal_pins.gpio16.reconfigure();
                // todo: is this required?
                pin.set_drive_strength(gpio::OutputDriveStrength::EightMilliAmps);
                pin.set_slew_rate(gpio::OutputSlewRate::Fast);
                pin
            },
            _v_sync: {
                let mut pin = hal_pins.gpio17.reconfigure();
                // todo: is this required?
                pin.set_drive_strength(gpio::OutputDriveStrength::EightMilliAmps);
                pin.set_slew_rate(gpio::OutputSlewRate::Fast);
                pin
            },
        },
        uart: {
            let uart_tx = hal_pins.gpio20.reconfigure();
            let uart_rx = hal_pins.gpio21.reconfigure();
            let uart = hal::uart::UartPeripheral::new(
                periph.UART1,
                (uart_tx, uart_rx),
                &mut periph.RESETS,
            );
            let config = hal::uart::UartConfig::new(
                115200.Hz(),
                hal::uart::DataBits::Eight,
                None,
                hal::uart::StopBits::One,
            );
            let mut uart = uart
                .enable(config, clocks.system_clock.freq())
                .expect("UART failed to configure");
            uart.set_fifos(false);
            // uart.enable_rx_interrupt();
            // uart.enable_tx_interrupt();
            uart
        },
    };

    info!("Setting up Core 1...");

    let _ = writeln!(hw.uart, "Hello UART!");

    start_core1(
        periph.DMA,
        periph.PIO0,
        periph.RESETS,
        &mut periph.PSM,
        &mut periph.PPB,
        &mut sio.fifo,
    );

    let mut delay = cortex_m::delay::Delay::new(cm.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        info!("Tick...");
        delay.delay_ms(1000);
        sio.fifo.write_blocking(0xA200_0000);
        let result = sio.fifo.read_blocking();
        info!("Sent A2, got {}", result);
    }
}

/// Start the video renderer on Core 1.
///
/// Core 1 will use the RESETS, DMA and PIO0 peripherals, so we take ownership
/// of those objects away from the Core 0 firmware.
fn start_core1(
    _dma: hal::pac::DMA,
    _pio: hal::pac::PIO0,
    _resets: hal::pac::RESETS,
    psm: &mut hal::pac::PSM,
    ppb: &mut hal::pac::PPB,
    fifo: &mut hal::sio::SioFifo,
) {
    let mut multicore = hal::multicore::Multicore::new(psm, ppb, fifo);
    let core1 = &mut multicore.cores()[1];
    let stack_allocation = unsafe {
        hal::multicore::StackAllocation::from_raw_parts(
            core::ptr::addr_of_mut!(CORE1_STACK_BOTTOM),
            core::ptr::addr_of_mut!(CORE1_STACK_TOP),
        )
    };
    info!("Core 1 entry point is {=usize:08x}", CORE1_ENTRY);
    // off the top of the chip? (or blank...)
    if CORE1_ENTRY > 0x1020_0000 {
        defmt::panic!("Core1 entry address 0x{:08x} is bad?!", CORE1_ENTRY);
    }
    let entry_func: fn() -> ! = unsafe { core::mem::transmute(CORE1_ENTRY) };
    info!("Spawning Core 1...");
    // start core 1
    core1
        .spawn(stack_allocation, move || entry_func())
        .expect("Spawning Core 1");
}

// End of file
