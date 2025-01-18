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
    gpio::{self, bank0, FunctionPio0, FunctionSioOutput, FunctionUart, Pin, PullNone},
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

#[repr(C)]
struct MiniVectorTable {
    pub stack_pointer: usize,
    pub reset_function: extern "C" fn() -> !,
}

extern "C" {
    /// This is the start of Core1's vector table
    static CORE1_VECTOR_TABLE: MiniVectorTable;
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
    /// Our blinky LED
    _led: hal::gpio::Pin<bank0::Gpio25, FunctionSioOutput, PullNone>,
}

#[hal::entry]
fn main() -> ! {
    info!(
        "Firmware {} {} starting up",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );

    let mut periph = pac::Peripherals::take().unwrap();

    // Check if stuff is running that shouldn't be. If so, do a full watchdog reboot.
    if stuff_running(&mut periph) {
        watchdog_reboot();
    }

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
        _led: { hal_pins.gpio25.reconfigure() },
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

    loop {
        let msg = sio.fifo.read_blocking();
        info!("Got {=u32:08x} from Core 1", msg);
    }
}

/// Check if the rest of the system appears to be running already.
///
/// If so, returns `true`, else `false`.
///
/// This probably means Core 0 reset but the rest of the system did not, and you
/// should do a hard reset to get everything back into sync.
fn stuff_running(p: &mut pac::Peripherals) -> bool {
    // Look at scratch register 7 and see what we left ourselves. If it's zero,
    // this was a full clean boot-up. If it's 0xDEADC0DE, this means we were
    // running and Core 0 restarted without restarting everything else.
    let scratch = p.WATCHDOG.scratch7().read().bits();
    defmt::info!("WD Scratch is 0x{:08x}", scratch);
    if scratch == 0xDEADC0DE {
        // we need a hard reset
        true
    } else {
        // set the marker so we know Core 0 has booted up
        p.WATCHDOG
            .scratch7()
            .write(|w| unsafe { w.bits(0xDEADC0DE) });
        false
    }
}

/// Clear the scratch register so we don't force a full watchdog reboot on the
/// next boot.
fn clear_scratch() {
    let p = unsafe { pac::Peripherals::steal() };
    p.WATCHDOG.scratch7().write(|w| unsafe { w.bits(0) });
}

/// Do a full watchdog reboot
fn watchdog_reboot() -> ! {
    clear_scratch();
    let p = unsafe { pac::Peripherals::steal() };
    let mut watchdog = hal::Watchdog::new(p.WATCHDOG);
    watchdog.start(fugit::Duration::<u32, 1, 1000000>::millis(10));
    loop {
        cortex_m::asm::wfi();
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
    static CORE1_STACK: rp2040_hal::multicore::Stack<4096> = rp2040_hal::multicore::Stack::new();

    let mut multicore = hal::multicore::Multicore::new(psm, ppb, fifo);
    let core1 = &mut multicore.cores()[1];
    let core1_vector_table = unsafe { &CORE1_VECTOR_TABLE };
    info!(
        "Core 1 sp=0x{=usize:08x}, reset=0x{=usize:08x}",
        core1_vector_table.stack_pointer, core1_vector_table.reset_function as usize
    );
    // off the top of the chip? (or blank...)
    if core1_vector_table.stack_pointer < 0x2000_0000
        || core1_vector_table.stack_pointer > 0x2004_2000
    {
        defmt::panic!(
            "Core1 entry address 0x{:08x} is bad?!",
            core1_vector_table.stack_pointer
        );
    }
    info!("Spawning Core 1...");
    // start core 1 (although give them some stack - don't use their stack pointer)
    let reset_func = core1_vector_table.reset_function;
    core1
        .spawn(CORE1_STACK.take().unwrap(), move || reset_func())
        .expect("Spawning Core 1");
}

// End of file
