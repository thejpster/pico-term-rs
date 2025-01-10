#![no_std]
#![no_main]

#[no_mangle]
#[link_section = ".entry_point"]
pub static ENTRY_POINT: unsafe fn() -> ! = reset_func;

/// The entry point to this firmare
///
/// # Safety
///
/// Do not call this function manually - only let the Boot ROM call this
/// function.
#[no_mangle]
pub unsafe fn reset_func() -> ! {
    panic!("Oh no");
}

#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
