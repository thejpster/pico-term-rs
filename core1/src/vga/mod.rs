//! # VGA Driver for the RC2014 RP2040 VGA Terminal.
//!
//! VGA output on the VGA Terminal uses 17 GPIO pins and two PIO state machines.
//!
//! It can generate 640x480@60Hz and 640x400@70Hz standard VGA video, with a
//! 25.2 MHz pixel clock. The spec is 25.175 MHz, so we are 0.1% off). The
//! assumption is that the CPU is clocked at 151.2 MHz, i.e. 6x the pixel clock.
//! All of the PIO code relies on this assumption!
//!
//! Currently 80x25, 80x30, 80x50 and 80x60 modes are supported in colour.
//!
//! This code is derived from the Neotron PICO BIOS.

// -----------------------------------------------------------------------------
// Licence Statement
// -----------------------------------------------------------------------------
// Copyright (c) Jonathan 'theJPster' Pallant and the Neotron Developers, 2023
// Copyright (c) The pico-term-rs developers, 2025
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
// details.
//
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <https://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Sub-modules
// -----------------------------------------------------------------------------

mod rgb;

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use crate::hal::{self, pac::interrupt, pio::PIOExt};
use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering},
};
use neotron_common_bios::video::{Attr, GlyphAttr, TextBackgroundColour, TextForegroundColour};
use rp2040_hal::sio::SioFifo;

use core::sync::atomic::AtomicUsize;
pub use rgb::{RGBColour, RGBPair};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// Describes the polarity of a sync pulse.
///
/// Some pulses are positive (active-high), some are negative (active-low).
enum SyncPolarity {
    /// An active-high pulse
    Positive,
    /// An active-low pulse
    Negative,
}

impl SyncPolarity {
    const fn enabled(&self) -> bool {
        match self {
            SyncPolarity::Positive => true,
            SyncPolarity::Negative => false,
        }
    }

    const fn disabled(&self) -> bool {
        match self {
            SyncPolarity::Positive => false,
            SyncPolarity::Negative => true,
        }
    }
}

/// Holds some data necessary to run the Video.
///
/// This structure is owned entirely by the main thread (or the drawing
/// thread). Data handled under interrupt is stored in various other places.
struct RenderEngine {
    /// How many frames have been drawn
    frame_count: u32,
    /// The current video mode
    current_video_mode: neotron_common_bios::video::Mode,
    /// The current framebuffer pointer
    current_video_ptr: *const u32,
    /// The current font pointer
    current_font_ptr: *const u32,
    /// How many rows of text are we showing right now
    num_text_rows: usize,
    /// How many columns of text are we showing right now
    num_text_cols: usize,
}

impl RenderEngine {
    // Initialise the main-thread resources
    pub fn new() -> RenderEngine {
        RenderEngine {
            frame_count: 0,
            // Should match the default value of TIMING_BUFFER and VIDEO_MODE
            current_video_mode: unsafe { neotron_common_bios::video::Mode::from_u8(1) },
            current_video_ptr: core::ptr::null(),
            current_font_ptr: core::ptr::null(),
            num_text_cols: 0,
            num_text_rows: 0,
        }
    }

    /// Call this once per frame
    pub fn new_frame(&mut self, fifo: &mut SioFifo) {
        self.frame_count += 1;
        while let Some(read_value) = fifo.read() {
            let command = read_value >> 24;
            let arg = read_value & 0xFF_FFFF;
            match command {
                // Change video mode
                0xA0 => {
                    let mode = unsafe { neotron_common_bios::video::Mode::from_u8(arg as u8) };
                    let result = if test_video_mode(mode) {
                        // let's go!
                        self.current_video_mode = mode;
                        // Tell the ISR to now generate our newly chosen timing
                        CURRENT_TIMING_MODE
                            .store(self.current_video_mode.timing() as usize, Ordering::Relaxed);
                        DOUBLE_SCAN_MODE
                            .store(self.current_video_mode.is_vert_2x(), Ordering::Relaxed);
                        // set up our text console to be the right size
                        self.num_text_cols =
                            self.current_video_mode.text_width().unwrap_or(0) as usize;
                        self.num_text_rows =
                            self.current_video_mode.text_height().unwrap_or(0) as usize;
                        // success
                        1
                    } else {
                        // failure
                        0
                    };
                    fifo.write_blocking(result);
                }
                // Change framebuffer pointer
                0xA1 => {
                    // pointers are 32-bit aligned so we shave two bits off to store a 26-bit pointer
                    let ptr = (arg << 2) as usize;
                    // add on the standard Cortex-M SRAM base address
                    let ptr = ptr + 0x2000_0000_usize;
                    // how big is this buffer?
                    let ptr_end = ptr + self.current_video_mode.frame_size_bytes();
                    let result = if SRAM_RANGE.contains(&ptr) && SRAM_RANGE.contains(&ptr_end) {
                        // well I guess it looks OK
                        self.current_video_ptr = ptr as *const u32;
                        1
                    } else {
                        0
                    };
                    fifo.write_blocking(result);
                }
                // Change font pointer
                0xA2 => {
                    // pointers are 32-bit aligned so we shave two bits off to store a 26-bit pointer
                    let ptr = (arg << 2) as usize;
                    // add on the standard Cortex-M SRAM base address
                    let ptr = ptr + 0x2000_0000_usize;
                    // how big is this buffer?
                    let ptr_end = ptr + self.current_video_mode.frame_size_bytes();
                    let result = if SRAM_RANGE.contains(&ptr) && SRAM_RANGE.contains(&ptr_end) {
                        // well I guess it looks OK
                        self.current_font_ptr = ptr as *const u32;
                        1
                    } else {
                        0
                    };
                    fifo.write_blocking(result);
                }
                // Test a video mode
                0xA3 => {
                    let mode = unsafe { neotron_common_bios::video::Mode::from_u8(arg as u8) };
                    let result = if test_video_mode(mode) { 1 } else { 0 };
                    fifo.write_blocking(result as u32);
                }
                // Get current scan-line
                0xA4 => {
                    let scan_line = get_scan_line();
                    fifo.write_blocking(scan_line as u32);
                }
                // Set a palette entry (packed as [ command | arg | 16-bit colour ])
                0xA5 => {
                    let index = arg as u8;
                    let rgb = (arg >> 8) as u16;
                    set_palette(index, RGBColour(rgb));
                }
                // Get a palette entry
                0xA6 => {
                    let index = arg as u8;
                    let rgb = get_palette(index);
                    fifo.write_blocking(rgb.0 as u32);
                }
                // Get the frame count
                0xA7 => {
                    fifo.write_blocking(self.frame_count);
                }
                _ => {
                    // ignored
                }
            }
        }
    }

    /// Draw a line of pixels into the relevant pixel buffer (either
    /// [`PIXEL_DATA_BUFFER_ODD`] or [`PIXEL_DATA_BUFFER_EVEN`]).
    ///
    /// The `current_line_num` goes from `0..NUM_LINES`.
    #[link_section = ".data"]
    pub fn draw_next_line(&mut self, mut current_line_num: u16) {
        if self.current_video_mode.is_vert_2x() {
            // in double scan mode we only draw ever other line
            current_line_num >>= 1;
        }

        // Pick a buffer to render into based on the line number we are drawing.
        // It's safe to write to this buffer because it's the the other one that
        // is currently being DMA'd out to the Pixel SM.
        let scan_line_buffer = if (current_line_num & 1) == 0 {
            &PIXEL_DATA_BUFFER_EVEN
        } else {
            &PIXEL_DATA_BUFFER_ODD
        };

        if self.current_video_ptr.is_null() {
            // do nothing
            return;
        }

        match self.current_video_mode.format() {
            neotron_common_bios::video::Format::Text8x16 => {
                // Text with 8x16 glyphs
                self.draw_next_line_text::<16>(scan_line_buffer, current_line_num)
            }
            neotron_common_bios::video::Format::Text8x8 => {
                // Text with 8x8 glyphs
                self.draw_next_line_text::<8>(scan_line_buffer, current_line_num)
            }
            neotron_common_bios::video::Format::Chunky1 => {
                // Bitmap with 1 bit per pixel
                self.draw_next_line_chunky1(scan_line_buffer, current_line_num);
            }
            neotron_common_bios::video::Format::Chunky2 => {
                // Bitmap with 2 bit per pixel
                self.draw_next_line_chunky2(scan_line_buffer, current_line_num);
            }
            neotron_common_bios::video::Format::Chunky4 => {
                // Bitmap with 4 bits per pixel
                self.draw_next_line_chunky4(scan_line_buffer, current_line_num);
            }
            neotron_common_bios::video::Format::Chunky8 => {
                // Bitmap with 8 bits per pixel
                self.draw_next_line_chunky8(scan_line_buffer, current_line_num);
            }
            _ => {
                // Draw nothing
            }
        };
    }

    /// Draw a line of 1-bpp bitmap as pixels.
    ///
    /// Writes into the relevant pixel buffer (either [`PIXEL_DATA_BUFFER_ODD`]
    /// or [`PIXEL_DATA_BUFFER_EVEN`]) assuming the framebuffer is a bitmap.
    ///
    /// The `current_line_num` goes from `0..NUM_LINES`.
    #[link_section = ".data"]
    pub fn draw_next_line_chunky1(&mut self, scan_line_buffer: &LineBuffer, current_line_num: u16) {
        let base_ptr = self.current_video_ptr as *const u8;
        let line_len_bytes = self.current_video_mode.line_size_bytes();
        let is_double = self.current_video_mode.is_horiz_2x();
        let offset = usize::from(current_line_num) * line_len_bytes;
        let line_start = unsafe { base_ptr.add(offset) };
        // Get a pointer into our scan-line buffer
        let mut scan_line_buffer_ptr = scan_line_buffer.pixel_ptr();
        if is_double {
            let white_pixel = RGBColour(
                VIDEO_PALETTE[TextForegroundColour::White as usize].load(Ordering::Relaxed),
            );
            let black_pixel = RGBColour(
                VIDEO_PALETTE[TextForegroundColour::Black as usize].load(Ordering::Relaxed),
            );
            // double-width mode.
            // sixteen RGB pixels (eight pairs) per byte
            let white_pair = RGBPair::from_pixels(white_pixel, white_pixel);
            let black_pair = RGBPair::from_pixels(black_pixel, black_pixel);
            for col in 0..line_len_bytes {
                let mono_pixels = unsafe { line_start.add(col).read() };
                unsafe {
                    // 0bX-------
                    let pixel = (mono_pixels & 1 << 7) != 0;
                    scan_line_buffer_ptr.offset(0).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // 0b-X------
                    let pixel = (mono_pixels & 1 << 6) != 0;
                    scan_line_buffer_ptr.offset(1).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // 0b--X-----
                    let pixel = (mono_pixels & 1 << 5) != 0;
                    scan_line_buffer_ptr.offset(2).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // 0b---X----
                    let pixel = (mono_pixels & 1 << 4) != 0;
                    scan_line_buffer_ptr.offset(3).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // 0b----X---
                    let pixel = (mono_pixels & 1 << 3) != 0;
                    scan_line_buffer_ptr.offset(4).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // 0b-----X--
                    let pixel = (mono_pixels & 1 << 2) != 0;
                    scan_line_buffer_ptr.offset(5).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // 0b------X-
                    let pixel = (mono_pixels & 1 << 1) != 0;
                    scan_line_buffer_ptr.offset(6).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // 0b-------X
                    let pixel = (mono_pixels & 1) != 0;
                    scan_line_buffer_ptr.offset(7).write(if pixel {
                        white_pair
                    } else {
                        black_pair
                    });
                    // move pointer along 16 pixels / 8 pairs
                    scan_line_buffer_ptr = scan_line_buffer_ptr.add(8);
                }
            }
        } else {
            // Non-double-width mode.
            // eight RGB pixels (four pairs) per byte
            let attr = Attr::new(
                TextForegroundColour::White,
                TextBackgroundColour::Black,
                false,
            );
            for col in 0..line_len_bytes {
                let mono_pixels = unsafe { line_start.add(col).read() };
                unsafe {
                    // 0bXX------
                    let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 6);
                    scan_line_buffer_ptr.write(pair);
                    // 0b--XX----
                    let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 4);
                    scan_line_buffer_ptr.add(1).write(pair);
                    // 0b----XX--
                    let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 2);
                    scan_line_buffer_ptr.add(2).write(pair);
                    // 0b------XX
                    let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels);
                    scan_line_buffer_ptr.add(3).write(pair);
                    scan_line_buffer_ptr = scan_line_buffer_ptr.add(4);
                }
            }
        }
    }

    /// Draw a line of 2-bpp bitmap as pixels.
    ///
    /// Writes into the relevant pixel buffer (either [`PIXEL_DATA_BUFFER_ODD`]
    /// or [`PIXEL_DATA_BUFFER_EVEN`]) assuming the framebuffer is a bitmap.
    ///
    /// The `current_line_num` goes from `0..NUM_LINES`.
    #[link_section = ".data"]
    pub fn draw_next_line_chunky2(&mut self, scan_line_buffer: &LineBuffer, current_line_num: u16) {
        let is_double = self.current_video_mode.is_horiz_2x();
        let base_ptr = self.current_video_ptr as *const u8;
        let line_len_bytes = self.current_video_mode.line_size_bytes();
        let offset = usize::from(current_line_num) * line_len_bytes;
        let line_start = unsafe { base_ptr.add(offset) };
        // Get a pointer into our scan-line buffer
        let mut scan_line_buffer_ptr = scan_line_buffer.pixel_ptr();
        let pixel_colours = [
            RGBColour(VIDEO_PALETTE[0].load(Ordering::Relaxed)),
            RGBColour(VIDEO_PALETTE[1].load(Ordering::Relaxed)),
            RGBColour(VIDEO_PALETTE[2].load(Ordering::Relaxed)),
            RGBColour(VIDEO_PALETTE[3].load(Ordering::Relaxed)),
        ];
        if is_double {
            let pixel_pairs = [
                RGBPair::from_pixels(pixel_colours[0], pixel_colours[0]),
                RGBPair::from_pixels(pixel_colours[1], pixel_colours[1]),
                RGBPair::from_pixels(pixel_colours[2], pixel_colours[2]),
                RGBPair::from_pixels(pixel_colours[3], pixel_colours[3]),
            ];
            let pixel_pairs_ptr = pixel_pairs.as_ptr();
            // Double-width mode.
            // eight RGB pixels (four pairs) per byte
            for col in 0..line_len_bytes {
                let chunky_pixels = unsafe { line_start.add(col).read() } as usize;
                unsafe {
                    let pair = pixel_pairs_ptr.add((chunky_pixels >> 6) & 0x03);
                    scan_line_buffer_ptr.write(*pair);
                    let pair = pixel_pairs_ptr.add((chunky_pixels >> 4) & 0x03);
                    scan_line_buffer_ptr.add(1).write(*pair);
                    let pair = pixel_pairs_ptr.add((chunky_pixels >> 2) & 0x03);
                    scan_line_buffer_ptr.add(2).write(*pair);
                    let pair = pixel_pairs_ptr.add(chunky_pixels & 0x03);
                    scan_line_buffer_ptr.add(3).write(*pair);
                    scan_line_buffer_ptr = scan_line_buffer_ptr.add(4);
                }
            }
        } else {
            let pixel_pairs = [
                RGBPair::from_pixels(pixel_colours[0], pixel_colours[0]),
                RGBPair::from_pixels(pixel_colours[0], pixel_colours[1]),
                RGBPair::from_pixels(pixel_colours[0], pixel_colours[2]),
                RGBPair::from_pixels(pixel_colours[0], pixel_colours[3]),
                RGBPair::from_pixels(pixel_colours[1], pixel_colours[0]),
                RGBPair::from_pixels(pixel_colours[1], pixel_colours[1]),
                RGBPair::from_pixels(pixel_colours[1], pixel_colours[2]),
                RGBPair::from_pixels(pixel_colours[1], pixel_colours[3]),
                RGBPair::from_pixels(pixel_colours[2], pixel_colours[0]),
                RGBPair::from_pixels(pixel_colours[2], pixel_colours[1]),
                RGBPair::from_pixels(pixel_colours[2], pixel_colours[2]),
                RGBPair::from_pixels(pixel_colours[2], pixel_colours[3]),
                RGBPair::from_pixels(pixel_colours[3], pixel_colours[0]),
                RGBPair::from_pixels(pixel_colours[3], pixel_colours[1]),
                RGBPair::from_pixels(pixel_colours[3], pixel_colours[2]),
                RGBPair::from_pixels(pixel_colours[3], pixel_colours[3]),
            ];
            let pixel_pairs_ptr = pixel_pairs.as_ptr();
            // Non-double-width mode.
            // four RGB pixels (two pairs) per byte
            for col in 0..line_len_bytes {
                let chunky_pixels = unsafe { line_start.add(col).read() } as usize;
                unsafe {
                    let pair = pixel_pairs_ptr.add(chunky_pixels >> 4);
                    scan_line_buffer_ptr.write(*pair);
                    scan_line_buffer_ptr = scan_line_buffer_ptr.add(1);
                    let pair = pixel_pairs_ptr.add(chunky_pixels & 0x0F);
                    scan_line_buffer_ptr.write(*pair);
                    scan_line_buffer_ptr = scan_line_buffer_ptr.add(1);
                }
            }
        }
    }

    /// Draw a line of 4-bpp bitmap as pixels.
    ///
    /// Writes into the relevant pixel buffer (either [`PIXEL_DATA_BUFFER_ODD`]
    /// or [`PIXEL_DATA_BUFFER_EVEN`]) assuming the framebuffer is a bitmap.
    ///
    /// The `current_line_num` goes from `0..NUM_LINES`.
    #[link_section = ".data"]
    pub fn draw_next_line_chunky4(&mut self, scan_line_buffer: &LineBuffer, current_line_num: u16) {
        let is_double = self.current_video_mode.is_horiz_2x();
        let base_ptr = self.current_video_ptr as *const u8;
        let line_len_bytes = self.current_video_mode.line_size_bytes();
        let line_start_offset_bytes = usize::from(current_line_num) * line_len_bytes;
        let line_start_bytes = unsafe { base_ptr.add(line_start_offset_bytes) };
        // Get a pointer into our scan-line buffer
        let mut scan_line_buffer_ptr = scan_line_buffer.pixel_ptr();
        if is_double {
            let palette_ptr = VIDEO_PALETTE.as_ptr() as *const RGBColour;
            // Double-width mode.
            // four RGB pixels (two pairs) per byte
            for col in 0..line_len_bytes {
                unsafe {
                    let chunky_pixels = line_start_bytes.add(col).read() as usize;
                    let left = palette_ptr.add((chunky_pixels >> 4) & 0x0F).read();
                    let right = palette_ptr.add(chunky_pixels & 0x0F).read();
                    scan_line_buffer_ptr.write(RGBPair::from_pixels(left, left));
                    scan_line_buffer_ptr
                        .add(1)
                        .write(RGBPair::from_pixels(right, right));
                    scan_line_buffer_ptr = scan_line_buffer_ptr.add(2);
                }
            }
        } else {
            // // This code optimises poorly, leaving a load from the literal pool in the middle of the for loop.
            //
            // for col in 0..line_len_bytes {
            // 	unsafe {
            // 		let pixel_pair = line_start_bytes.add(col).read();
            // 		let pair = CHUNKY4_COLOUR_LOOKUP.lookup(pixel_pair);
            // 		scan_line_buffer_ptr.write(pair);
            // 		scan_line_buffer_ptr = scan_line_buffer_ptr.add(1);
            // 	}
            // }

            // So I wrote it by hand in assembly instead, saving two clock cycles per loop
            // We have 640x4 (320x8) input and must produce 320x32 output
            unsafe {
                core::arch::asm!(
                    "0:",
                    // load a byte from line_start_bytes
                    "ldrb	{tmp}, [{lsb}]",
                    // multiply it by sizeof(u32)
                    "lsls	{tmp}, {tmp}, #0x2",
                    // load a 32-bit RGB pair from CHUNKY4_COLOUR_LOOKUP
                    "ldr	{tmp}, [{chunky}, {tmp}]",
                    // store the 32-bit RGB pair to the scanline buffer, and increment
                    "stm	{slbp}!, {{ {tmp} }}",
                    // increment the pointer to the start of the line
                    "adds	{lsb}, {lsb}, #0x1",
                    // loop until we're done
                    "cmp	{lsb}, {lsb_max}",
                    "bne	0b",
                    lsb = in(reg) line_start_bytes,
                    lsb_max = in(reg) line_start_bytes.add(line_len_bytes),
                    chunky = in(reg) core::ptr::addr_of!(CHUNKY4_COLOUR_LOOKUP),
                    tmp = in(reg) 0,
                    slbp = in(reg) scan_line_buffer_ptr,
                );
            }
        }
    }

    /// Draw a line of 8-bpp bitmap as pixels.
    ///
    /// Writes into the relevant pixel buffer (either [`PIXEL_DATA_BUFFER_ODD`]
    /// or [`PIXEL_DATA_BUFFER_EVEN`]) assuming the framebuffer is a bitmap.
    ///
    /// The `current_line_num` goes from `0..NUM_LINES`.
    #[link_section = ".data"]
    pub fn draw_next_line_chunky8(&mut self, scan_line_buffer: &LineBuffer, current_line_num: u16) {
        let is_double = self.current_video_mode.is_horiz_2x();
        let base_ptr = self.current_video_ptr as *const u8;
        let line_len_bytes = self.current_video_mode.line_size_bytes();
        let line_start_offset_bytes = usize::from(current_line_num) * line_len_bytes;
        let line_start_bytes = unsafe { base_ptr.add(line_start_offset_bytes) };
        // Get a pointer into our scan-line buffer
        let mut scan_line_buffer_ptr = scan_line_buffer.pixel_ptr();
        let palette_ptr = VIDEO_PALETTE.as_ptr() as *const RGBColour;
        if is_double {
            // Double-width mode.
            // two RGB pixels (one pair) per byte

            // This code optimises poorly
            // for col in 0..line_len_bytes {
            // 	unsafe {
            // 		let chunky_pixel = line_start_bytes.add(col).read() as usize;
            // 		let rgb = palette_ptr.add(chunky_pixel).read();
            // 		scan_line_buffer_ptr.write(RGBPair::from_pixels(rgb, rgb));
            // 		scan_line_buffer_ptr = scan_line_buffer_ptr.add(1);
            // 	}
            // }

            // So I wrote it by hand in assembly instead, saving two clock cycles per loop
            // We have 320x8 input and must produce 320x32 output
            unsafe {
                core::arch::asm!(
                    "0:",
                    // load a byte from line_start_bytes
                    "ldrb	{tmp}, [{lsb}]",
                    // multiply it by sizeof(u16)
                    "lsls	{tmp}, {tmp}, #0x1",
                    // load a single 16-bit RGB value from the palette
                    "ldrh	{tmp}, [{palette}, {tmp}]",
                    // double it up to make a 32-bit RGB pair containing two identical pixels
                    "lsls   {tmp2}, {tmp}, #16",
                    "adds   {tmp}, {tmp}, {tmp2}",
                    // store the 32-bit RGB pair to the scanline buffer, and increment
                    "stm	{slbp}!, {{ {tmp} }}",
                    // increment the pointer to the start of the line
                    "adds	{lsb}, {lsb}, #0x1",
                    // loop until we're done
                    "cmp	{lsb}, {lsb_max}",
                    "bne	0b",
                    lsb = in(reg) line_start_bytes,
                    lsb_max = in(reg) line_start_bytes.add(line_len_bytes),
                    palette = in(reg) core::ptr::addr_of!(VIDEO_PALETTE),
                    tmp = in(reg) 0,
                    tmp2 = in(reg) 1,
                    slbp = in(reg) scan_line_buffer_ptr,
                );
            }
        } else {
            // Single-width mode. This won't run fast enough on an RP2040, but no supported mode uses it.
            // one RGB pixel per byte
            for col in 0..line_len_bytes / 2 {
                unsafe {
                    let chunky_pixel_left = line_start_bytes.add(col * 2).read() as usize;
                    let rgb_left = palette_ptr.add(chunky_pixel_left).read();
                    let chunky_pixel_right = line_start_bytes.add((col * 2) + 1).read() as usize;
                    let rgb_right = palette_ptr.add(chunky_pixel_right).read();
                    scan_line_buffer_ptr.write(RGBPair::from_pixels(rgb_left, rgb_right));
                    scan_line_buffer_ptr = scan_line_buffer_ptr.add(1);
                }
            }
        }
    }

    /// Draw a line text as pixels.
    ///
    /// Writes into the relevant pixel buffer (either [`PIXEL_DATA_BUFFER_ODD`]
    /// or [`PIXEL_DATA_BUFFER_EVEN`]) using the 8x16 font.
    ///
    /// The `current_line_num` goes from `0..NUM_LINES`.
    #[link_section = ".data"]
    pub fn draw_next_line_text<const GLYPH_HEIGHT: usize>(
        &mut self,
        scan_line_buffer: &LineBuffer,
        current_line_num: u16,
    ) {
        if self.current_font_ptr.is_null() {
            // draw nothing
            return;
        }
        // Convert our position in scan-lines to a text row, and a line within each glyph on that row
        let text_row = current_line_num as usize / GLYPH_HEIGHT;
        let font_row = current_line_num as usize % GLYPH_HEIGHT;

        if text_row >= self.num_text_rows {
            return;
        }

        // Note (unsafe): We are using whatever memory we were given and we
        // assume there is good data there and there is enough data there. To
        // try and avoid Undefined Behaviour, we only access through a pointer
        // and never make a reference to the data.
        let fb_ptr = self.current_video_ptr as *const GlyphAttr;
        let row_ptr = unsafe { fb_ptr.add(text_row * self.num_text_cols) };

        // Every font look-up we are about to do for this row will
        // involve offsetting by the row within each glyph. As this
        // is the same for every glyph on this row, we calculate a
        // new pointer once, in advance, and save ourselves an
        // addition each time around the loop.
        let font_ptr = self.current_font_ptr as *const u8;
        let font_ptr = unsafe { font_ptr.add(font_row) };

        // Get a pointer into our scan-line buffer
        let mut scan_line_buffer_ptr = scan_line_buffer.pixel_ptr();

        // Convert from characters to coloured pixels, using the font as a look-up table.
        for col in 0..self.num_text_cols {
            unsafe {
                // Note (unsafe): We use pointer arithmetic here because we
                // can't afford a bounds-check on an array. This is safe
                // because the font is `256 * width` bytes long and we can't
                // index more than `255 * width` bytes into it.
                let glyphattr = row_ptr.add(col).read();
                let index = (glyphattr.glyph().0 as usize) * GLYPH_HEIGHT;
                let attr = glyphattr.attr();
                let mono_pixels = *font_ptr.add(index);

                // 0bXX------
                let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 6);
                scan_line_buffer_ptr.write(pair);
                // 0b--XX----
                let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 4);
                scan_line_buffer_ptr.add(1).write(pair);
                // 0b----XX--
                let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 2);
                scan_line_buffer_ptr.add(2).write(pair);
                // 0b------XX
                let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels);
                scan_line_buffer_ptr.add(3).write(pair);

                scan_line_buffer_ptr = scan_line_buffer_ptr.add(4);
            }
        }
    }
}

impl Default for RenderEngine {
    fn default() -> Self {
        RenderEngine::new()
    }
}

/// Describes one scan-line's worth of pixels, including the length word required by the Pixel FIFO.
#[repr(C, align(16))]
struct LineBuffer {
    /// Must be one less than the number of pixel-pairs in `pixels`
    length: u32,
    /// Pixels to be displayed, grouped into pairs (to save FIFO space and reduce DMA bandwidth)
    pixels: UnsafeCell<[RGBPair; MAX_NUM_PIXEL_PAIRS_PER_LINE]>,
}

impl LineBuffer {
    /// Make a new LineBuffer
    ///
    /// Use this for the even lines, so you get a checkerboard
    const fn new_even() -> LineBuffer {
        LineBuffer {
            length: (MAX_NUM_PIXEL_PAIRS_PER_LINE as u32) - 1,
            pixels: UnsafeCell::new(
                [RGBPair::from_pixels(RGBColour::WHITE, RGBColour::BLACK);
                    MAX_NUM_PIXEL_PAIRS_PER_LINE],
            ),
        }
    }

    /// Make a new LineBuffer
    ///
    /// Use this for the odd lines, so you get a checkerboard
    const fn new_odd() -> LineBuffer {
        LineBuffer {
            length: (MAX_NUM_PIXEL_PAIRS_PER_LINE as u32) - 1,
            pixels: UnsafeCell::new(
                [RGBPair::from_pixels(RGBColour::BLACK, RGBColour::WHITE);
                    MAX_NUM_PIXEL_PAIRS_PER_LINE],
            ),
        }
    }

    /// Get a pointer to the entire linebuffer.
    ///
    /// This produces a 32-bit address that the DMA engine understands.
    fn as_ptr(&self) -> u32 {
        self as *const _ as usize as u32
    }

    /// Get a pointer to the pixel data
    fn pixel_ptr(&self) -> *mut RGBPair {
        self.pixels.get() as *mut RGBPair
    }
}

unsafe impl Sync for LineBuffer {}

/// The kind of IRQ we want to raise
#[derive(Debug, Copy, Clone)]
enum RaiseIrq {
    None,
    Irq0,
    Irq1,
}

impl RaiseIrq {
    const IRQ0_INSTR: u16 = pio::InstructionOperands::IRQ {
        clear: false,
        wait: false,
        index: 0,
        relative: false,
    }
    .encode();

    const IRQ1_INSTR: u16 = pio::InstructionOperands::IRQ {
        clear: false,
        wait: false,
        index: 1,
        relative: false,
    }
    .encode();

    const IRQ_NONE_INSTR: u16 = pio::InstructionOperands::MOV {
        destination: pio::MovDestination::Y,
        op: pio::MovOperation::None,
        source: pio::MovSource::Y,
    }
    .encode();

    /// Produces a PIO command that raises the appropriate IRQ
    #[inline]
    pub const fn into_command(self) -> u16 {
        match self {
            RaiseIrq::None => Self::IRQ_NONE_INSTR,
            RaiseIrq::Irq0 => Self::IRQ0_INSTR,
            RaiseIrq::Irq1 => Self::IRQ1_INSTR,
        }
    }
}

/// Holds the four scan-line timing FIFO words we need for one scan-line.
///
/// See `make_timing` for a function which can generate these words. We DMA
/// them into the timing FIFO, so they must sit on a 16-byte boundary.
#[repr(C, align(16))]
struct ScanlineTimingBuffer {
    data: [u32; 4],
}

impl ScanlineTimingBuffer {
    const CLOCKS_PER_PIXEL: u32 = 6;

    /// Create a timing buffer for each scan-line in the V-Sync visible portion.
    ///
    /// The timings are in the order (front-porch, sync, back-porch, visible) and are in pixel clocks.
    #[inline]
    const fn new_v_visible(
        hsync: SyncPolarity,
        vsync: SyncPolarity,
        timings: (u32, u32, u32, u32),
    ) -> ScanlineTimingBuffer {
        ScanlineTimingBuffer {
            data: [
                // Front porch (as per the spec)
                Self::make_timing(
                    timings.0 * Self::CLOCKS_PER_PIXEL,
                    hsync.disabled(),
                    vsync.disabled(),
                    RaiseIrq::Irq1,
                ),
                // Sync pulse (as per the spec)
                Self::make_timing(
                    timings.1 * Self::CLOCKS_PER_PIXEL,
                    hsync.enabled(),
                    vsync.disabled(),
                    RaiseIrq::None,
                ),
                // Back porch. Adjusted by a few clocks to account for interrupt +
                // PIO SM start latency.
                Self::make_timing(
                    (timings.2 * Self::CLOCKS_PER_PIXEL) - 10,
                    hsync.disabled(),
                    vsync.disabled(),
                    RaiseIrq::None,
                ),
                // Visible portion. It also triggers the IRQ to start pixels
                // moving. Adjusted to compensate for changes made to previous
                // period to ensure scan-line remains at correct length.
                Self::make_timing(
                    (timings.3 * Self::CLOCKS_PER_PIXEL) + 10,
                    hsync.disabled(),
                    vsync.disabled(),
                    RaiseIrq::Irq0,
                ),
            ],
        }
    }

    /// Create a timing buffer for each scan-line in the V-Sync front-porch and back-porch
    #[inline]
    const fn new_v_porch(
        hsync: SyncPolarity,
        vsync: SyncPolarity,
        timings: (u32, u32, u32, u32),
    ) -> ScanlineTimingBuffer {
        ScanlineTimingBuffer {
            data: [
                // Front porch (as per the spec)
                Self::make_timing(
                    timings.0 * Self::CLOCKS_PER_PIXEL,
                    hsync.disabled(),
                    vsync.disabled(),
                    RaiseIrq::Irq1,
                ),
                // Sync pulse (as per the spec)
                Self::make_timing(
                    timings.1 * Self::CLOCKS_PER_PIXEL,
                    hsync.enabled(),
                    vsync.disabled(),
                    RaiseIrq::None,
                ),
                // Back porch.
                Self::make_timing(
                    timings.2 * Self::CLOCKS_PER_PIXEL,
                    hsync.disabled(),
                    vsync.disabled(),
                    RaiseIrq::None,
                ),
                // Visible portion.
                Self::make_timing(
                    timings.3 * Self::CLOCKS_PER_PIXEL,
                    hsync.disabled(),
                    vsync.disabled(),
                    RaiseIrq::None,
                ),
            ],
        }
    }

    /// Create a timing buffer for each scan-line in the V-Sync pulse
    #[inline]
    const fn new_v_pulse(
        hsync: SyncPolarity,
        vsync: SyncPolarity,
        timings: (u32, u32, u32, u32),
    ) -> ScanlineTimingBuffer {
        ScanlineTimingBuffer {
            data: [
                // Front porch (as per the spec)
                Self::make_timing(
                    timings.0 * Self::CLOCKS_PER_PIXEL,
                    hsync.disabled(),
                    vsync.enabled(),
                    RaiseIrq::Irq1,
                ),
                // Sync pulse (as per the spec)
                Self::make_timing(
                    timings.1 * Self::CLOCKS_PER_PIXEL,
                    hsync.enabled(),
                    vsync.enabled(),
                    RaiseIrq::None,
                ),
                // Back porch.
                Self::make_timing(
                    timings.2 * Self::CLOCKS_PER_PIXEL,
                    hsync.disabled(),
                    vsync.enabled(),
                    RaiseIrq::None,
                ),
                // Visible portion.
                Self::make_timing(
                    timings.3 * Self::CLOCKS_PER_PIXEL,
                    hsync.disabled(),
                    vsync.enabled(),
                    RaiseIrq::None,
                ),
            ],
        }
    }

    /// Generate a 32-bit value we can send to the Timing FIFO.
    ///
    /// * `period` - The length of this portion of the scan-line, in system clock ticks
    /// * `hsync` - true if the H-Sync pin should be high during this period, else false
    /// * `vsync` - true if the H-Sync pin should be high during this period, else false
    /// * `raise_irq` - true the timing statemachine should raise an IRQ at the start of this period
    ///
    /// Returns a 32-bit value you can post to the Timing FIFO.
    #[inline]
    const fn make_timing(period: u32, hsync: bool, vsync: bool, raise_irq: RaiseIrq) -> u32 {
        let command = raise_irq.into_command() as u32;
        let mut value: u32 = 0;
        if hsync {
            value |= 1 << 0;
        }
        if vsync {
            value |= 1 << 1;
        }
        value |= (period - FIXED_CLOCKS_PER_TIMING_PULSE) << 2;
        value | command << 16
    }
}

/// Holds the different kinds of scan-line timing buffers we need for various
/// portions of the screen.
#[repr(C)]
struct TimingBuffer {
    /// We use this when there are visible pixels on screen
    visible_line: ScanlineTimingBuffer,
    /// We use this during the v-sync front-porch and v-sync back-porch
    vblank_porch_buffer: ScanlineTimingBuffer,
    /// We use this during the v-sync sync pulse
    vblank_sync_buffer: ScanlineTimingBuffer,
    /// The last visible scan-line,
    visible_lines_ends_at: u16,
    /// The last scan-line of the front porch
    front_porch_end_at: u16,
    /// The last scan-line of the sync pulse
    sync_pulse_ends_at: u16,
    /// The last scan-line of the back-porch (and the frame)
    back_porch_ends_at: u16,
}

impl TimingBuffer {
    /// Make a timing buffer suitable for 640 x 400 @ 70 Hz
    const fn make_640x400() -> TimingBuffer {
        TimingBuffer {
            visible_line: ScanlineTimingBuffer::new_v_visible(
                SyncPolarity::Negative,
                SyncPolarity::Positive,
                (16, 96, 48, 640),
            ),
            vblank_porch_buffer: ScanlineTimingBuffer::new_v_porch(
                SyncPolarity::Negative,
                SyncPolarity::Positive,
                (16, 96, 48, 640),
            ),
            vblank_sync_buffer: ScanlineTimingBuffer::new_v_pulse(
                SyncPolarity::Negative,
                SyncPolarity::Positive,
                (16, 96, 48, 640),
            ),
            visible_lines_ends_at: 399,
            front_porch_end_at: 399 + 12,
            sync_pulse_ends_at: 399 + 12 + 2,
            back_porch_ends_at: 399 + 12 + 2 + 35,
        }
    }

    /// Make a timing buffer suitable for 640 x 480 @ 60 Hz
    pub const fn make_640x480() -> TimingBuffer {
        TimingBuffer {
            visible_line: ScanlineTimingBuffer::new_v_visible(
                SyncPolarity::Negative,
                SyncPolarity::Negative,
                (16, 96, 48, 640),
            ),
            vblank_porch_buffer: ScanlineTimingBuffer::new_v_porch(
                SyncPolarity::Negative,
                SyncPolarity::Negative,
                (16, 96, 48, 640),
            ),
            vblank_sync_buffer: ScanlineTimingBuffer::new_v_pulse(
                SyncPolarity::Negative,
                SyncPolarity::Negative,
                (16, 96, 48, 640),
            ),
            visible_lines_ends_at: 479,
            front_porch_end_at: 479 + 10,
            sync_pulse_ends_at: 479 + 10 + 2,
            back_porch_ends_at: 479 + 10 + 2 + 33,
        }
    }
}

/// See [`TEXT_COLOUR_LOOKUP`]
struct TextColourLookup {
    entries: [AtomicU32; 512],
}

impl TextColourLookup {
    const FOREGROUND: [TextForegroundColour; 16] = [
        TextForegroundColour::Black,
        TextForegroundColour::Blue,
        TextForegroundColour::Green,
        TextForegroundColour::Cyan,
        TextForegroundColour::Red,
        TextForegroundColour::Magenta,
        TextForegroundColour::Brown,
        TextForegroundColour::LightGray,
        TextForegroundColour::DarkGray,
        TextForegroundColour::LightBlue,
        TextForegroundColour::LightGreen,
        TextForegroundColour::LightCyan,
        TextForegroundColour::LightRed,
        TextForegroundColour::Pink,
        TextForegroundColour::Yellow,
        TextForegroundColour::White,
    ];

    const BACKGROUND: [TextBackgroundColour; 8] = [
        TextBackgroundColour::Black,
        TextBackgroundColour::Blue,
        TextBackgroundColour::Green,
        TextBackgroundColour::Cyan,
        TextBackgroundColour::Red,
        TextBackgroundColour::Magenta,
        TextBackgroundColour::Brown,
        TextBackgroundColour::LightGray,
    ];

    const fn blank() -> TextColourLookup {
        #[allow(clippy::declare_interior_mutable_const)]
        const ZERO: AtomicU32 = AtomicU32::new(0);
        TextColourLookup {
            entries: [ZERO; 512],
        }
    }

    fn init(&self, palette: &[AtomicU16]) {
        for (fg, fg_colour) in Self::FOREGROUND.iter().zip(palette.iter()) {
            for (bg, bg_colour) in Self::BACKGROUND.iter().zip(palette.iter()) {
                let attr = Attr::new(*fg, *bg, false);
                for pixels in 0..=3 {
                    let index: usize = (((attr.0 & 0x7F) as usize) << 2) | (pixels & 0x03) as usize;
                    let pair = RGBPair::from_pixels(
                        if pixels & 0x02 == 0x02 {
                            RGBColour(fg_colour.load(Ordering::Relaxed))
                        } else {
                            RGBColour(bg_colour.load(Ordering::Relaxed))
                        },
                        if pixels & 0x01 == 0x01 {
                            RGBColour(fg_colour.load(Ordering::Relaxed))
                        } else {
                            RGBColour(bg_colour.load(Ordering::Relaxed))
                        },
                    );
                    self.entries[index].store(pair.0, Ordering::Relaxed);
                }
            }
        }
    }

    fn update_index(&self, updated_palette_entry: u8, palette: &[AtomicU16]) {
        for (fg, (fg_colour_idx, fg_colour)) in
            Self::FOREGROUND.iter().zip(palette.iter().enumerate())
        {
            for (bg, (bg_colour_idx, bg_colour)) in
                Self::BACKGROUND.iter().zip(palette.iter().enumerate())
            {
                if fg_colour_idx != usize::from(updated_palette_entry)
                    && bg_colour_idx != usize::from(updated_palette_entry)
                {
                    // skip this one - it didn't change
                    continue;
                }
                let attr = Attr::new(*fg, *bg, false);
                for pixels in 0..=3 {
                    let index: usize = (((attr.0 & 0x7F) as usize) << 2) | (pixels & 0x03) as usize;
                    let pair = RGBPair::from_pixels(
                        if pixels & 0x02 == 0x02 {
                            RGBColour(fg_colour.load(Ordering::Relaxed))
                        } else {
                            RGBColour(bg_colour.load(Ordering::Relaxed))
                        },
                        if pixels & 0x01 == 0x01 {
                            RGBColour(fg_colour.load(Ordering::Relaxed))
                        } else {
                            RGBColour(bg_colour.load(Ordering::Relaxed))
                        },
                    );
                    self.entries[index].store(pair.0, Ordering::Relaxed);
                }
            }
        }
    }

    #[inline]
    fn lookup(&self, attr: Attr, pixels: u8) -> RGBPair {
        let index: usize = (((attr.0 & 0x7F) as usize) << 2) | (pixels & 0x03) as usize;
        RGBPair(self.entries[index].load(Ordering::Relaxed))
    }
}

/// See [`CHUNKY4_COLOUR_LOOKUP`]
struct Chunky4ColourLookup {
    entries: [AtomicU32; 256],
}

impl Chunky4ColourLookup {
    /// Create a blank look-up table.
    const fn blank() -> Chunky4ColourLookup {
        Chunky4ColourLookup {
            entries: [const { AtomicU32::new(0) }; 256],
        }
    }

    /// Initialise this look-up table from the palette.
    fn init(&self, palette: &[AtomicU16]) {
        let palette = &palette[0..16];
        for (left_idx, left_colour) in palette.iter().enumerate() {
            for (right_idx, right_colour) in palette.iter().enumerate() {
                let left_colour = left_colour.load(Ordering::Relaxed);
                let right_colour = right_colour.load(Ordering::Relaxed);
                let index = (left_idx << 4) + right_idx;
                let pair = RGBPair::from_pixels(RGBColour(left_colour), RGBColour(right_colour));
                self.entries[index].store(pair.0, Ordering::Relaxed);
            }
        }
    }

    /// Update a look-up table entry.
    ///
    /// The `updated_palette_entry` is an index from 0..16 into the main palette
    /// (given as `palette`).
    fn update_index(&self, updated_palette_entry: u8, palette: &[AtomicU16]) {
        let palette = &palette[0..16];
        let updated_palette_entry = usize::from(updated_palette_entry);
        for (left_idx, left_colour) in palette.iter().enumerate() {
            for (right_idx, right_colour) in palette.iter().enumerate() {
                if left_idx == updated_palette_entry || right_idx == updated_palette_entry {
                    let left_colour = left_colour.load(Ordering::Relaxed);
                    let right_colour = right_colour.load(Ordering::Relaxed);
                    let index = (left_idx << 4) + right_idx;
                    let pair =
                        RGBPair::from_pixels(RGBColour(left_colour), RGBColour(right_colour));
                    self.entries[index].store(pair.0, Ordering::Relaxed);
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// Any valid framebuffer or font pointer will be in this range
const SRAM_RANGE: core::ops::Range<usize> = 0x2000_0000..0x2004_2000;

/// How many pixels per scan-line.
///
/// Adjust the pixel PIO program to run at the right speed to the screen is
/// filled. For example, if this is only 320 but you are aiming at 640x480,
/// make the pixel PIO take twice as long per pixel.
pub const MAX_NUM_PIXELS_PER_LINE: usize = 640;

/// Holds 16 palette entries, paired with every other of 16 palette entries.
///
/// Allows a fast lookup of an RGB pixel pair given two 4-bpp pixels packed into a byte.
static CHUNKY4_COLOUR_LOOKUP: Chunky4ColourLookup = Chunky4ColourLookup::blank();

/// Holds the 256-entry palette for indexed colour modes.
///
/// Note, the first eight entries should match
/// [`neotron_common_bios::video::TextBackgroundColour`] and the first 16 entries
/// should match [`neotron_common_bios::video::TextForegroundColour`].
static VIDEO_PALETTE: [AtomicU16; 256] = [
    // Index 000: 0x000 (Black)
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 001: 0x00a (Blue)
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0xaa).0),
    // Index 002: 0x0a0 (Green)
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xaa, 0x00).0),
    // Index 003: 0x0aa (Cyan)
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xaa, 0xaa).0),
    // Index 004: 0xa00 (Red)
    AtomicU16::new(RGBColour::from_24bit(0xaa, 0x00, 0x00).0),
    // Index 005: 0xa0a (Magenta)
    AtomicU16::new(RGBColour::from_24bit(0xaa, 0x00, 0xaa).0),
    // Index 006: 0xaa0 (Brown)
    AtomicU16::new(RGBColour::from_24bit(0xaa, 0x55, 0x00).0),
    // Index 007: 0xaaa (Light Gray)
    AtomicU16::new(RGBColour::from_24bit(0xaa, 0xaa, 0xaa).0),
    // Index 008: 0x666 (Dark Gray)
    AtomicU16::new(RGBColour::from_24bit(0x55, 0x55, 0x55).0),
    // Index 009: 0x00f (Light Blue)
    AtomicU16::new(RGBColour::from_24bit(0x55, 0x55, 0xff).0),
    // Index 010: 0x0f0 (Light Green)
    AtomicU16::new(RGBColour::from_24bit(0x55, 0xff, 0x55).0),
    // Index 011: 0x0ff (Light Cyan)
    AtomicU16::new(RGBColour::from_24bit(0x55, 0xff, 0xff).0),
    // Index 012: 0xf00 (Light Red)
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x55, 0x55).0),
    // Index 013: 0xf0f (Pink)
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x55, 0xff).0),
    // Index 014: 0xff0 (Yellow)
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xff, 0x55).0),
    // Index 015: 0xfff (White)
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xff, 0xff).0),
    // Index 016: 0x003
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 017: 0x006
    AtomicU16::new(RGBColour::from_24bit(0x14, 0x14, 0x14).0),
    // Index 018: 0x00c
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x20, 0x20).0),
    // Index 019: 0x020
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x2c, 0x2c).0),
    // Index 020: 0x023
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x38, 0x38).0),
    // Index 021: 0x026
    AtomicU16::new(RGBColour::from_24bit(0x45, 0x45, 0x45).0),
    // Index 022: 0x028
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x51, 0x51).0),
    // Index 023: 0x02c
    AtomicU16::new(RGBColour::from_24bit(0x61, 0x61, 0x61).0),
    // Index 024: 0x02f
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x71, 0x71).0),
    // Index 025: 0x040
    AtomicU16::new(RGBColour::from_24bit(0x82, 0x82, 0x82).0),
    // Index 026: 0x043
    AtomicU16::new(RGBColour::from_24bit(0x92, 0x92, 0x92).0),
    // Index 027: 0x046
    AtomicU16::new(RGBColour::from_24bit(0xa2, 0xa2, 0xa2).0),
    // Index 028: 0x048
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xb6, 0xb6).0),
    // Index 029: 0x04c
    AtomicU16::new(RGBColour::from_24bit(0xcb, 0xcb, 0xcb).0),
    // Index 030: 0x04f
    AtomicU16::new(RGBColour::from_24bit(0xe3, 0xe3, 0xe3).0),
    // Index 031: 0x083
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xff, 0xff).0),
    // Index 032: 0x086
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0xff).0),
    // Index 033: 0x08c
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x00, 0xff).0),
    // Index 034: 0x08f
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0x00, 0xff).0),
    // Index 035: 0x0a0
    AtomicU16::new(RGBColour::from_24bit(0xbe, 0x00, 0xff).0),
    // Index 036: 0x0a3
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x00, 0xff).0),
    // Index 037: 0x0a6
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x00, 0xbe).0),
    // Index 038: 0x0a8
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x00, 0x7d).0),
    // Index 039: 0x0ac
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x00, 0x41).0),
    // Index 040: 0x0af
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x00, 0x00).0),
    // Index 041: 0x0e0
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x41, 0x00).0),
    // Index 042: 0x0e3
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x7d, 0x00).0),
    // Index 043: 0x0e6
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xbe, 0x00).0),
    // Index 044: 0x0e8
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xff, 0x00).0),
    // Index 045: 0x0ec
    AtomicU16::new(RGBColour::from_24bit(0xbe, 0xff, 0x00).0),
    // Index 046: 0x0ef
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xff, 0x00).0),
    // Index 047: 0x0f3
    AtomicU16::new(RGBColour::from_24bit(0x41, 0xff, 0x00).0),
    // Index 048: 0x0f6
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xff, 0x00).0),
    // Index 049: 0x0f8
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xff, 0x41).0),
    // Index 050: 0x0fc
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xff, 0x7d).0),
    // Index 051: 0x300
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xff, 0xbe).0),
    // Index 052: 0x303
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xff, 0xff).0),
    // Index 053: 0x306
    AtomicU16::new(RGBColour::from_24bit(0x00, 0xbe, 0xff).0),
    // Index 054: 0x308
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x7d, 0xff).0),
    // Index 055: 0x30c
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x41, 0xff).0),
    // Index 056: 0x30f
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0x7d, 0xff).0),
    // Index 057: 0x320
    AtomicU16::new(RGBColour::from_24bit(0x9e, 0x7d, 0xff).0),
    // Index 058: 0x323
    AtomicU16::new(RGBColour::from_24bit(0xbe, 0x7d, 0xff).0),
    // Index 059: 0x326
    AtomicU16::new(RGBColour::from_24bit(0xdf, 0x7d, 0xff).0),
    // Index 060: 0x328
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x7d, 0xff).0),
    // Index 061: 0x32c
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x7d, 0xdf).0),
    // Index 062: 0x32f
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x7d, 0xbe).0),
    // Index 063: 0x340
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x7d, 0x9e).0),
    // Index 064: 0x343
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x7d, 0x7d).0),
    // Index 065: 0x346
    AtomicU16::new(RGBColour::from_24bit(0xff, 0x9e, 0x7d).0),
    // Index 066: 0x348
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xbe, 0x7d).0),
    // Index 067: 0x34c
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xdf, 0x7d).0),
    // Index 068: 0x34f
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xff, 0x7d).0),
    // Index 069: 0x380
    AtomicU16::new(RGBColour::from_24bit(0xdf, 0xff, 0x7d).0),
    // Index 070: 0x383
    AtomicU16::new(RGBColour::from_24bit(0xbe, 0xff, 0x7d).0),
    // Index 071: 0x386
    AtomicU16::new(RGBColour::from_24bit(0x9e, 0xff, 0x7d).0),
    // Index 072: 0x388
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xff, 0x7d).0),
    // Index 073: 0x38c
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xff, 0x9e).0),
    // Index 074: 0x38f
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xff, 0xbe).0),
    // Index 075: 0x3a0
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xff, 0xdf).0),
    // Index 076: 0x3a3
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xff, 0xff).0),
    // Index 077: 0x3a6
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xdf, 0xff).0),
    // Index 078: 0x3a8
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0xbe, 0xff).0),
    // Index 079: 0x3ac
    AtomicU16::new(RGBColour::from_24bit(0x7d, 0x9e, 0xff).0),
    // Index 080: 0x3af
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xb6, 0xff).0),
    // Index 081: 0x3e0
    AtomicU16::new(RGBColour::from_24bit(0xc7, 0xb6, 0xff).0),
    // Index 082: 0x3e3
    AtomicU16::new(RGBColour::from_24bit(0xdb, 0xb6, 0xff).0),
    // Index 083: 0x3e6
    AtomicU16::new(RGBColour::from_24bit(0xeb, 0xb6, 0xff).0),
    // Index 084: 0x3e8
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xb6, 0xff).0),
    // Index 085: 0x3ec
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xb6, 0xeb).0),
    // Index 086: 0x3ef
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xb6, 0xdb).0),
    // Index 087: 0x3f0
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xb6, 0xc7).0),
    // Index 088: 0x3f3
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xb6, 0xb6).0),
    // Index 089: 0x3f6
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xc7, 0xb6).0),
    // Index 090: 0x3f8
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xdb, 0xb6).0),
    // Index 091: 0x3fc
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xeb, 0xb6).0),
    // Index 092: 0x3ff
    AtomicU16::new(RGBColour::from_24bit(0xff, 0xff, 0xb6).0),
    // Index 093: 0x600
    AtomicU16::new(RGBColour::from_24bit(0xeb, 0xff, 0xb6).0),
    // Index 094: 0x603
    AtomicU16::new(RGBColour::from_24bit(0xdb, 0xff, 0xb6).0),
    // Index 095: 0x606
    AtomicU16::new(RGBColour::from_24bit(0xc7, 0xff, 0xb6).0),
    // Index 096: 0x608
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xff, 0xb6).0),
    // Index 097: 0x60c
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xff, 0xc7).0),
    // Index 098: 0x60f
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xff, 0xdb).0),
    // Index 099: 0x620
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xff, 0xeb).0),
    // Index 100: 0x623
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xff, 0xff).0),
    // Index 101: 0x626
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xeb, 0xff).0),
    // Index 102: 0x628
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xdb, 0xff).0),
    // Index 103: 0x62c
    AtomicU16::new(RGBColour::from_24bit(0xb6, 0xc7, 0xff).0),
    // Index 104: 0x62f
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x71).0),
    // Index 105: 0x640
    AtomicU16::new(RGBColour::from_24bit(0x1c, 0x00, 0x71).0),
    // Index 106: 0x643
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x00, 0x71).0),
    // Index 107: 0x646
    AtomicU16::new(RGBColour::from_24bit(0x55, 0x00, 0x71).0),
    // Index 108: 0x648
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x00, 0x71).0),
    // Index 109: 0x64c
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x00, 0x55).0),
    // Index 110: 0x64f
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x00, 0x38).0),
    // Index 111: 0x680
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x00, 0x1c).0),
    // Index 112: 0x683
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x00, 0x00).0),
    // Index 113: 0x686
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x1c, 0x00).0),
    // Index 114: 0x688
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x38, 0x00).0),
    // Index 115: 0x68c
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x55, 0x00).0),
    // Index 116: 0x68f
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x71, 0x00).0),
    // Index 117: 0x6a0
    AtomicU16::new(RGBColour::from_24bit(0x55, 0x71, 0x00).0),
    // Index 118: 0x6a3
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x71, 0x00).0),
    // Index 119: 0x6a6
    AtomicU16::new(RGBColour::from_24bit(0x1c, 0x71, 0x00).0),
    // Index 120: 0x6a8
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x71, 0x00).0),
    // Index 121: 0x6ac
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x71, 0x1c).0),
    // Index 122: 0x6af
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x71, 0x38).0),
    // Index 123: 0x6e0
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x71, 0x55).0),
    // Index 124: 0x6e3
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x71, 0x71).0),
    // Index 125: 0x6e6
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x55, 0x71).0),
    // Index 126: 0x6e8
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x38, 0x71).0),
    // Index 127: 0x6ec
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x1c, 0x71).0),
    // Index 128: 0x6ef
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x38, 0x71).0),
    // Index 129: 0x6f0
    AtomicU16::new(RGBColour::from_24bit(0x45, 0x38, 0x71).0),
    // Index 130: 0x6f3
    AtomicU16::new(RGBColour::from_24bit(0x55, 0x38, 0x71).0),
    // Index 131: 0x6f6
    AtomicU16::new(RGBColour::from_24bit(0x61, 0x38, 0x71).0),
    // Index 132: 0x6f8
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x38, 0x71).0),
    // Index 133: 0x6fc
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x38, 0x61).0),
    // Index 134: 0x6ff
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x38, 0x55).0),
    // Index 135: 0x803
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x38, 0x45).0),
    // Index 136: 0x806
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x38, 0x38).0),
    // Index 137: 0x80c
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x45, 0x38).0),
    // Index 138: 0x80f
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x55, 0x38).0),
    // Index 139: 0x820
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x61, 0x38).0),
    // Index 140: 0x823
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x71, 0x38).0),
    // Index 141: 0x826
    AtomicU16::new(RGBColour::from_24bit(0x61, 0x71, 0x38).0),
    // Index 142: 0x828
    AtomicU16::new(RGBColour::from_24bit(0x55, 0x71, 0x38).0),
    // Index 143: 0x82c
    AtomicU16::new(RGBColour::from_24bit(0x45, 0x71, 0x38).0),
    // Index 144: 0x82f
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x71, 0x38).0),
    // Index 145: 0x840
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x71, 0x45).0),
    // Index 146: 0x843
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x71, 0x55).0),
    // Index 147: 0x846
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x71, 0x61).0),
    // Index 148: 0x848
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x71, 0x71).0),
    // Index 149: 0x84c
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x61, 0x71).0),
    // Index 150: 0x84f
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x55, 0x71).0),
    // Index 151: 0x883
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x45, 0x71).0),
    // Index 152: 0x886
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x51, 0x71).0),
    // Index 153: 0x88c
    AtomicU16::new(RGBColour::from_24bit(0x59, 0x51, 0x71).0),
    // Index 154: 0x88f
    AtomicU16::new(RGBColour::from_24bit(0x61, 0x51, 0x71).0),
    // Index 155: 0x8a0
    AtomicU16::new(RGBColour::from_24bit(0x69, 0x51, 0x71).0),
    // Index 156: 0x8a3
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x51, 0x71).0),
    // Index 157: 0x8a6
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x51, 0x69).0),
    // Index 158: 0x8a8
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x51, 0x61).0),
    // Index 159: 0x8ac
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x51, 0x59).0),
    // Index 160: 0x8af
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x51, 0x51).0),
    // Index 161: 0x8e0
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x59, 0x51).0),
    // Index 162: 0x8e3
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x61, 0x51).0),
    // Index 163: 0x8e6
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x69, 0x51).0),
    // Index 164: 0x8e8
    AtomicU16::new(RGBColour::from_24bit(0x71, 0x71, 0x51).0),
    // Index 165: 0x8ec
    AtomicU16::new(RGBColour::from_24bit(0x69, 0x71, 0x51).0),
    // Index 166: 0x8ef
    AtomicU16::new(RGBColour::from_24bit(0x61, 0x71, 0x51).0),
    // Index 167: 0x8f0
    AtomicU16::new(RGBColour::from_24bit(0x59, 0x71, 0x51).0),
    // Index 168: 0x8f3
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x71, 0x51).0),
    // Index 169: 0x8f6
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x71, 0x59).0),
    // Index 170: 0x8f8
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x71, 0x61).0),
    // Index 171: 0x8fc
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x71, 0x69).0),
    // Index 172: 0x8ff
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x71, 0x71).0),
    // Index 173: 0xc00
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x69, 0x71).0),
    // Index 174: 0xc03
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x61, 0x71).0),
    // Index 175: 0xc06
    AtomicU16::new(RGBColour::from_24bit(0x51, 0x59, 0x71).0),
    // Index 176: 0xc08
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x41).0),
    // Index 177: 0xc0c
    AtomicU16::new(RGBColour::from_24bit(0x10, 0x00, 0x41).0),
    // Index 178: 0xc0f
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x00, 0x41).0),
    // Index 179: 0xc20
    AtomicU16::new(RGBColour::from_24bit(0x30, 0x00, 0x41).0),
    // Index 180: 0xc23
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x00, 0x41).0),
    // Index 181: 0xc26
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x00, 0x30).0),
    // Index 182: 0xc28
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x00, 0x20).0),
    // Index 183: 0xc2c
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x00, 0x10).0),
    // Index 184: 0xc2f
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x00, 0x00).0),
    // Index 185: 0xc40
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x10, 0x00).0),
    // Index 186: 0xc43
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x20, 0x00).0),
    // Index 187: 0xc46
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x30, 0x00).0),
    // Index 188: 0xc48
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x41, 0x00).0),
    // Index 189: 0xc4c
    AtomicU16::new(RGBColour::from_24bit(0x30, 0x41, 0x00).0),
    // Index 190: 0xc4f
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x41, 0x00).0),
    // Index 191: 0xc80
    AtomicU16::new(RGBColour::from_24bit(0x10, 0x41, 0x00).0),
    // Index 192: 0xc83
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x41, 0x00).0),
    // Index 193: 0xc86
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x41, 0x10).0),
    // Index 194: 0xc88
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x41, 0x20).0),
    // Index 195: 0xc8c
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x41, 0x30).0),
    // Index 196: 0xc8f
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x41, 0x41).0),
    // Index 197: 0xca0
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x30, 0x41).0),
    // Index 198: 0xca3
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x20, 0x41).0),
    // Index 199: 0xca6
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x10, 0x41).0),
    // Index 200: 0xca8
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x20, 0x41).0),
    // Index 201: 0xcac
    AtomicU16::new(RGBColour::from_24bit(0x28, 0x20, 0x41).0),
    // Index 202: 0xcaf
    AtomicU16::new(RGBColour::from_24bit(0x30, 0x20, 0x41).0),
    // Index 203: 0xce0
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x20, 0x41).0),
    // Index 204: 0xce3
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x20, 0x41).0),
    // Index 205: 0xce6
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x20, 0x38).0),
    // Index 206: 0xce8
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x20, 0x30).0),
    // Index 207: 0xcec
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x20, 0x28).0),
    // Index 208: 0xcef
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x20, 0x20).0),
    // Index 209: 0xcf0
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x28, 0x20).0),
    // Index 210: 0xcf3
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x30, 0x20).0),
    // Index 211: 0xcf6
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x38, 0x20).0),
    // Index 212: 0xcf8
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x41, 0x20).0),
    // Index 213: 0xcfc
    AtomicU16::new(RGBColour::from_24bit(0x38, 0x41, 0x20).0),
    // Index 214: 0xcff
    AtomicU16::new(RGBColour::from_24bit(0x30, 0x41, 0x20).0),
    // Index 215: 0xf03
    AtomicU16::new(RGBColour::from_24bit(0x28, 0x41, 0x20).0),
    // Index 216: 0xf06
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x41, 0x20).0),
    // Index 217: 0xf08
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x41, 0x28).0),
    // Index 218: 0xf0c
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x41, 0x30).0),
    // Index 219: 0xf20
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x41, 0x38).0),
    // Index 220: 0xf23
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x41, 0x41).0),
    // Index 221: 0xf26
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x38, 0x41).0),
    // Index 222: 0xf28
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x30, 0x41).0),
    // Index 223: 0xf2c
    AtomicU16::new(RGBColour::from_24bit(0x20, 0x28, 0x41).0),
    // Index 224: 0xf2f
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x2c, 0x41).0),
    // Index 225: 0xf40
    AtomicU16::new(RGBColour::from_24bit(0x30, 0x2c, 0x41).0),
    // Index 226: 0xf43
    AtomicU16::new(RGBColour::from_24bit(0x34, 0x2c, 0x41).0),
    // Index 227: 0xf46
    AtomicU16::new(RGBColour::from_24bit(0x3c, 0x2c, 0x41).0),
    // Index 228: 0xf48
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x2c, 0x41).0),
    // Index 229: 0xf4c
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x2c, 0x3c).0),
    // Index 230: 0xf4f
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x2c, 0x34).0),
    // Index 231: 0xf80
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x2c, 0x30).0),
    // Index 232: 0xf83
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x2c, 0x2c).0),
    // Index 233: 0xf86
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x30, 0x2c).0),
    // Index 234: 0xf88
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x34, 0x2c).0),
    // Index 235: 0xf8c
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x3c, 0x2c).0),
    // Index 236: 0xf8f
    AtomicU16::new(RGBColour::from_24bit(0x41, 0x41, 0x2c).0),
    // Index 237: 0xfa0
    AtomicU16::new(RGBColour::from_24bit(0x3c, 0x41, 0x2c).0),
    // Index 238: 0xfa3
    AtomicU16::new(RGBColour::from_24bit(0x34, 0x41, 0x2c).0),
    // Index 239: 0xfa6
    AtomicU16::new(RGBColour::from_24bit(0x30, 0x41, 0x2c).0),
    // Index 240: 0xfa8
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x41, 0x2c).0),
    // Index 241: 0xfac
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x41, 0x30).0),
    // Index 242: 0xfaf
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x41, 0x34).0),
    // Index 243: 0xfe0
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x41, 0x3c).0),
    // Index 244: 0xfe3
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x41, 0x41).0),
    // Index 245: 0xfe6
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x3c, 0x41).0),
    // Index 246: 0xfe8
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x34, 0x41).0),
    // Index 247: 0xfec
    AtomicU16::new(RGBColour::from_24bit(0x2c, 0x30, 0x41).0),
    // Index 248: 0xfef
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 249: 0xff3
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 250: 0xff6
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 251: 0xff8
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 252: 0xffc
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 253: 0xbbb
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 254: 0x333
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
    // Index 255: 0x777
    AtomicU16::new(RGBColour::from_24bit(0x00, 0x00, 0x00).0),
];

/// How many pixel pairs we send out.
///
/// Each pixel is two 12-bit values packed into one 32-bit word(an `RGBPair`).
/// This is to make more efficient use of DMA and FIFO resources.
const MAX_NUM_PIXEL_PAIRS_PER_LINE: usize = MAX_NUM_PIXELS_PER_LINE / 2;

/// Stores timing data which we DMA into the timing PIO State Machine.
///
/// Must match the `Timing` enum, except we don't support 800x600.
#[link_section = ".data"]
static TIMING_BUFFER: [TimingBuffer; 2] =
    [TimingBuffer::make_640x480(), TimingBuffer::make_640x400()];

/// Tracks which timing mode we use
///
/// Ensure this matches the default chosen in [`RenderEngine::new()`]
static CURRENT_TIMING_MODE: AtomicUsize = AtomicUsize::new(0);

/// Tracks which scan-line will be shown next, therefore which one you should be drawing right now.
///
/// This is for timing purposes, therefore it goes from
/// `0..TIMING_BUFFER.back_porch_ends_at`.
///
/// Set by the PIO IRQ.
static NEXT_SCAN_LINE: AtomicU16 = AtomicU16::new(0);

/// Are we in double-scan mode?
///
/// If we are, each scan-line buffer is played out twice, and you should divide
/// `NEXT_SCAN_LINE` by 2 before rendering a line.
static DOUBLE_SCAN_MODE: AtomicBool = AtomicBool::new(false);

/// Indicates that we should draw the current scan-line given by [`NEXT_SCAN_LINE`].
///
/// Set by the PIO IRQ.
static DRAW_THIS_LINE: AtomicBool = AtomicBool::new(false);

/// DMA channel for the timing FIFO
const TIMING_DMA_CHAN: usize = 0;

/// DMA channel for the pixel FIFO
const PIXEL_DMA_CHAN: usize = 1;

/// One scan-line's worth of 12-bit pixels, used for the even scan-lines (0, 2, 4 ... NUM_LINES-2).
///
/// Gets read by DMA, which pushes them into the pixel state machine's FIFO.
///
/// Gets written to by `RenderEngine` running on Core 1.
static PIXEL_DATA_BUFFER_EVEN: LineBuffer = LineBuffer::new_even();

/// One scan-line's worth of 12-bit pixels, used for the odd scan-lines (1, 3, 5 ... NUM_LINES-1).
///
/// Gets read by DMA, which pushes them into the pixel state machine's FIFO.
///
/// Gets written to by `RenderEngine` running on Core 1.
static PIXEL_DATA_BUFFER_ODD: LineBuffer = LineBuffer::new_odd();

/// Holds the colour look-up table for text mode.
///
/// The input is a 9-bit value comprised of the 4-bit foreground colour index,
/// the 3-bit background colour index, and a two mono pixels. The output is a
/// 32-bit RGB Colour Pair, containing two RGB pixels.
///
/// ```
/// +-----+-----+-----+-----+-----+-----+-----+-----+-----+
/// | FG3 | FG2 | FG1 | FG0 | BG2 | BG1 | BG0 | PX1 | PX0 |
/// +-----+-----+-----+-----+-----+-----+-----+-----+-----+
/// ```
static TEXT_COLOUR_LOOKUP: TextColourLookup = TextColourLookup::blank();

/// How many fixed clock cycles there are per timing pulse.
const FIXED_CLOCKS_PER_TIMING_PULSE: u32 = 5;

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// Initialise all the static data and peripherals we need for our video display.
///
/// We need to keep `pio` and `dma` to run the video. We need `resets` to set
/// things up, so we only borrow that.
pub fn init(pio: super::pac::PIO0, dma: super::pac::DMA, resets: &mut super::pac::RESETS) {
    // Grab PIO0 and the state machines it contains
    let (mut pio, sm0, sm1, _sm2, _sm3) = pio.split(resets);

    // Reset the DMA Peripheral.
    resets.reset().modify(|_r, w| w.dma().set_bit());
    unsafe {
        core::arch::asm!("nop");
    }
    resets.reset().modify(|_r, w| w.dma().clear_bit());
    while resets.reset_done().read().dma().bit_is_clear() {}

    // This program runs the timing loop. We post timing data (i.e. the length
    // of each period, along with what the H-Sync and V-Sync pins should do) and
    // it sets the GPIO pins and busy-waits the appropriate amount of time. It
    // also takes an extra 'instruction' which we can use to trigger the
    // appropriate interrupts.
    //
    // Note that the timing period value should be:
    //
    // timing_period = actual_timing_period - FIXED_CLOCKS_PER_TIMING_PULSE
    //
    // This is because there are unavoidable clock cycles within the algorithm.
    // Currently FIXED_CLOCKS_PER_TIMING_PULSE should be set to 5.
    //
    // Post <value:32> where value: <clock_cycles:14> <hsync:1> <vsync:1>
    // <instruction:16>
    //
    // The SM will execute the instruction (typically either a NOP or an IRQ),
    // set the H-Sync and V-Sync pins as desired, then wait the given number of
    // clock cycles.
    //
    // Note: autopull should be set to 32-bits, OSR is set to shift right.
    let timing_program = pio_proc::pio_asm!(
        ".wrap_target"
        // Step 1. Push next 2 bits of OSR into `pins`, to set H-Sync and V-Sync.
        //         Takes 1 clock cycle.
        "out pins, 2"
        // Step 2. Push last 14 bits of OSR into X for the timing loop.
        //         Takes 1 clock cycle.
        "out x, 14"
        // Step 3. Execute bottom 16-bits of OSR as an instruction.
        //         This take two cycles, always.
        "out exec, 16"
        // Spin until X is zero
        // Takes X + 1 clock cycles because the branch is conditioned on the initial value of the register.
        // i.e. X = 0 => 1 clock cycle (the jmp when x = 0)
        // i.e. X = 1 => 2 clock cycles (the jmp when x = 1 and again when x = 0)
        "loop0:"
            "jmp x-- loop0"
        ".wrap"
    );

    // This is the video pixels program. It waits for an IRQ (posted by the
    // timing loop) then pulls pixel data from the FIFO. We post the number of
    // pixels for that line, then the pixel data.
    //
    // Post <num_pixels> <pixel1> <pixel2> ... <pixelN>; each <pixelX> maps to
    // the RGB output pins. On a Neotron Pico, there are 12 (4 Red, 4 Green and
    // 4 Blue) - each value should be 12-bits long in the bottom of a 16-bit
    // word.
    //
    // Currently the FIFO supplies only the pixels, not the length value. When
    // we read the length from the FIFO as well, all hell breaks loose.
    //
    // Note autopull should be set to 32-bits, OSR is set to shift right.
    let pixel_program = pio_proc::pio_asm!(
        ".wrap_target"
        // Wait for timing state machine to start visible line
        "wait 1 irq 0"
        // Read the line length (in pixel-pairs)
        "out x, 32"
        "loop1:"
            // Write out first pixel - takes 5 clocks per pixel
            "out pins, 16 [5]"
            // Write out second pixel - takes 5 clocks per pixel (allowing one clock for the jump)
            "out pins, 16 [4]"
            // Repeat until all pixel pairs sent
            "jmp x-- loop1"
        // Clear all pins after visible section
        "mov pins null"
        ".wrap"
    );

    // These two state machines run thus:
    //
    // | Clock | Timing PIOSM | Pixel PIOSM      |
    // |:------|:-------------|:-----------------|
    // | 1     | out pins, 2  | wait 1 irq 0     |
    // | 2     | out x, 14    | wait 1 irq 0     |
    // | 3     | out exec, 16 | wait 1 irq 0     |
    // | 4     | <exec irq>   | wait 1 irq 0     |
    // | 5     | jmp x--      | wait 1 irq 0     |
    // | 6     |              | out x, 32        |
    // | 7     |              | out pins, 16 [4] |
    // | 8     |              | ..               |
    // | 9     |              | ..               |
    // | 10    |              | ..               |
    // | 11    |              | ..               |
    // | 12    |              | out pins, 16 [3] |
    // | 13    |              | ..               |
    // | 14    |              | ..               |
    // | 15    |              | ..               |
    // | 16    |              | jump x--         |
    //
    // Note: Credit to
    // https://gregchadwick.co.uk/blog/playing-with-the-pico-pt5/ who had a
    // very similar idea to me, but wrote it up far better than I ever could.

    let timing_installed = pio.install(&timing_program.program).unwrap();
    let (mut timing_sm, _, timing_fifo) =
        hal::pio::PIOBuilder::from_installed_program(timing_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(16, 2) // H-Sync is GPIO16, V-Sync is GPIO17
            .autopull(true)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .pull_threshold(32)
            .build(sm0);
    timing_sm.set_pindirs([
        (16, hal::pio::PinDir::Output),
        (17, hal::pio::PinDir::Output),
    ]);

    // Important notes!
    //
    // You must not set a clock_divider (other than 1.0) on the pixel state
    // machine. You might want the pixels to be twice as wide (or mode), but
    // enabling a clock divider adds a lot of jitter (i.e. the start each
    // each line differs by some number of 151.2 MHz clock cycles).

    let pixels_installed = pio.install(&pixel_program.program).unwrap();
    let (mut pixel_sm, _, pixel_fifo) =
        hal::pio::PIOBuilder::from_installed_program(pixels_installed)
            .buffers(hal::pio::Buffers::OnlyTx)
            .out_pins(0, 16) // Red0 is GPIO0, Blue4 is GPIO15.
            .autopull(true)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .pull_threshold(32) // We read all 32-bits in each FIFO word
            .build(sm1);
    pixel_sm.set_pindirs((0..=15).map(|x| (x, hal::pio::PinDir::Output)));

    pio.irq1().enable_sm_interrupt(1);

    // Read from the timing buffer and write to the timing FIFO.
    dma.ch(TIMING_DMA_CHAN).ch_ctrl_trig().write(|w| {
        w.data_size().size_word();
        w.incr_read().set_bit();
        w.incr_write().clear_bit();
        unsafe { w.treq_sel().bits(timing_fifo.dreq_value()) };
        unsafe { w.chain_to().bits(TIMING_DMA_CHAN as u8) };
        unsafe { w.ring_size().bits(0) };
        w.ring_sel().clear_bit();
        w.bswap().clear_bit();
        w.irq_quiet().clear_bit();
        w.en().set_bit();
        w.sniff_en().clear_bit();
        w
    });
    let timing_buffer = &TIMING_BUFFER[0];
    dma.ch(TIMING_DMA_CHAN)
        .ch_read_addr()
        .write(|w| unsafe { w.bits(&raw const timing_buffer.visible_line.data as u32) });
    dma.ch(TIMING_DMA_CHAN)
        .ch_write_addr()
        .write(|w| unsafe { w.bits(timing_fifo.fifo_address() as usize as u32) });
    dma.ch(TIMING_DMA_CHAN)
        .ch_trans_count()
        .write(|w| unsafe { w.bits(timing_buffer.visible_line.data.len() as u32) });

    // Read from the pixel buffer (even first) and write to the pixel FIFO
    dma.ch(PIXEL_DMA_CHAN).ch_ctrl_trig().write(|w| {
        w.data_size().size_word();
        w.incr_read().set_bit();
        w.incr_write().clear_bit();
        unsafe { w.treq_sel().bits(pixel_fifo.dreq_value()) };
        unsafe { w.chain_to().bits(PIXEL_DMA_CHAN as u8) };
        unsafe { w.ring_size().bits(0) };
        w.ring_sel().clear_bit();
        w.bswap().clear_bit();
        w.irq_quiet().clear_bit();
        w.en().set_bit();
        w.sniff_en().clear_bit();
        w
    });
    dma.ch(PIXEL_DMA_CHAN)
        .ch_read_addr()
        .write(|w| unsafe { w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()) });
    dma.ch(PIXEL_DMA_CHAN)
        .ch_write_addr()
        .write(|w| unsafe { w.bits(pixel_fifo.fifo_address() as usize as u32) });
    dma.ch(PIXEL_DMA_CHAN)
        .ch_trans_count()
        .write(|w| unsafe { w.bits(MAX_NUM_PIXEL_PAIRS_PER_LINE as u32 + 1) });

    // Enable the DMA
    dma.multi_chan_trigger()
        .write(|w| unsafe { w.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN)) });

    timing_sm.start();
    pixel_sm.start();

    // We drop our state-machine and PIO objects here - this means the video
    // cannot be reconfigured at a later time, but they do keep on running
    // as-is.

    // No-one else is looking at this right now.
    TEXT_COLOUR_LOOKUP.init(&VIDEO_PALETTE);
    CHUNKY4_COLOUR_LOOKUP.init(&VIDEO_PALETTE);
}

/// Check the given video mode is allowable
pub fn test_video_mode(mode: neotron_common_bios::video::Mode) -> bool {
    if !mode.is_horiz_2x() {
        // in the 640-px modes we can only do up to 4-bpp
        matches!(
            (mode.timing(), mode.format()),
            (
                neotron_common_bios::video::Timing::T640x480
                    | neotron_common_bios::video::Timing::T640x400,
                neotron_common_bios::video::Format::Text8x16
                    | neotron_common_bios::video::Format::Text8x8
                    | neotron_common_bios::video::Format::Chunky1
                    | neotron_common_bios::video::Format::Chunky2
                    | neotron_common_bios::video::Format::Chunky4,
            )
        )
    } else {
        // in the 320-px modes we can also do 8-bpp
        matches!(
            (mode.timing(), mode.format()),
            (
                neotron_common_bios::video::Timing::T640x480
                    | neotron_common_bios::video::Timing::T640x400,
                neotron_common_bios::video::Format::Chunky1
                    | neotron_common_bios::video::Format::Chunky2
                    | neotron_common_bios::video::Format::Chunky4
                    | neotron_common_bios::video::Format::Chunky8,
            )
        )
    }
}

/// Get the current scan line.
///
/// Note that these are timing scan lines, not visible scan lines (so we count
/// to 480 even in a 240 line mode).
pub fn get_scan_line() -> u16 {
    NEXT_SCAN_LINE.load(Ordering::Relaxed)
}

/// Set a palette entry
pub fn set_palette(index: u8, colour: RGBColour) {
    // Store it
    VIDEO_PALETTE[index as usize].store(colour.0, Ordering::Relaxed);
    // Update the text cache
    if index <= 15 {
        TEXT_COLOUR_LOOKUP.update_index(index, &VIDEO_PALETTE);
        CHUNKY4_COLOUR_LOOKUP.update_index(index, &VIDEO_PALETTE);
    }
}

/// Get a palette entry
pub fn get_palette(index: u8) -> RGBColour {
    RGBColour(VIDEO_PALETTE[index as usize].load(Ordering::Relaxed))
}

/// This function runs the video processing loop on Core 1.
///
/// It keeps the odd/even scan-line buffers updated, as per the contents of
/// the text buffer.
#[link_section = ".data"]
pub fn render_loop(mut fifo: rp2040_hal::sio::SioFifo) -> ! {
    let mut render_engine = RenderEngine::new();

    // The LED pin was configured in the code that ran on Core 0. Rather than
    // try and move the pin over to this core, we just unsafely poke the GPIO
    // registers to set/clear the relevant pin.
    let gpio_out_set = 0xd000_0014 as *mut u32;
    let gpio_out_clr = 0xd000_0018 as *mut u32;

    loop {
        // Wait for a free DMA buffer. Can't do a compare-and-swap on ARMv6-M :/
        while !DRAW_THIS_LINE.load(Ordering::Acquire) {
            unsafe {
                core::arch::asm!("wfe");
            }
        }
        DRAW_THIS_LINE.store(false, Ordering::Relaxed);

        // The one we draw *now* is the one that is *shown* next
        let this_line = NEXT_SCAN_LINE.load(Ordering::Relaxed);

        unsafe {
            // Turn on LED
            gpio_out_set.write(1 << 25);
        }

        // This function currently consumes about 70% CPU (or rather, 90% CPU
        // on each of visible lines, and 0% CPU on the other lines)
        render_engine.draw_next_line(this_line);

        unsafe {
            // Turn off LED
            gpio_out_clr.write(1 << 25);
        }

        // we've just drawn the last visible line, so let's take a moment to inspect the FIFO
        if this_line == (render_engine.current_video_mode.vertical_lines() - 1) {
            render_engine.new_frame(&mut fifo)
        }
    }
}

/// This function is called whenever the Timing State Machine starts a
/// scan-line.
///
/// Timing wise, we should be at the start of the front-porch (i.e. just after
/// the visible portion finishes). This is because it is the 'back porch' part
/// of the timing data sent to the Timing State Machine that contains a "Raise
/// IRQ 1" instruction, and that IRQ triggers this function.
///
/// The visible section contains a "Raise IRQ 0" instruction, but that only
/// triggers the Pixel State Machine and not a CPU interrupt.
///
/// # Safety
///
/// Only call this from the PIO IRQ handler.
#[link_section = ".data"]
#[interrupt]
unsafe fn PIO0_IRQ_1() {
    let pio = unsafe { &*crate::pac::PIO0::ptr() };
    let dma = unsafe { &*crate::pac::DMA::ptr() };

    // Clear the interrupt
    pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));

    // Current timing mode
    let current_mode_nr = CURRENT_TIMING_MODE.load(Ordering::Relaxed);
    let timing_data = &TIMING_BUFFER[current_mode_nr];
    // Are we double scanning?
    let double_scan = DOUBLE_SCAN_MODE.load(Ordering::Relaxed);

    // This is now the line we are currently in the middle of playing;
    // timing-wise anyway - the pixels will be along in moment once we've told
    // the DMA which pixels to play.
    let current_timing_line = NEXT_SCAN_LINE.load(Ordering::Relaxed);
    // This is the line we should cue up to play next
    let next_timing_line = if current_timing_line == timing_data.back_porch_ends_at {
        // Wrap around
        0
    } else {
        // Keep going
        current_timing_line + 1
    };

    let (mask, draw_now) = if double_scan {
        // Only tell the main loop to re-draw on odd lines (i.e. 1, 3, 5, etc)
        // because on even lines (0, 2, 4, ...) we still need to draw the line
        // again.

        // The mask is 2, so we have:
        //
        // 0 = (play even, draw = true)
        // 1 = (play even, draw = false)
        // 2 = (play odd, draw = true)
        // 3 = (play odd, draw = false)
        // 4 = (play even, draw = true)
        // etc
        (2, (next_timing_line & 1) == 0)
    } else {
        // tell the main loop to draw, always
        //
        // The mask is 1, so we have:
        //
        // 0 = (play even, draw = true)
        // 1 = (play odd, draw = true)
        // 2 = (play even, draw = true)
        // 3 = (play odd, draw = true)
        // 4 = (play even, draw = true)
        // etc
        (1, true)
    };

    // Are we in the visible portion *right* now? If so, copy some pixels into
    // the Pixel SM FIFO using DMA. Hopefully the main thread has them ready for
    // us (though we're playing them, ready or not).
    if current_timing_line <= timing_data.visible_lines_ends_at {
        if (current_timing_line & mask) != 0 {
            // Load the odd line into the Pixel SM FIFO for immediate playback
            dma.ch(PIXEL_DMA_CHAN)
                .ch_al3_read_addr_trig()
                .write(|w| w.bits(PIXEL_DATA_BUFFER_ODD.as_ptr()));
        } else {
            // Load the even line into the Pixel SM FIFO for immediate playback
            dma.ch(PIXEL_DMA_CHAN)
                .ch_al3_read_addr_trig()
                .write(|w| w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()));
        }
        // The data will start pouring into the FIFO, but the output is corked until
        // the timing SM generates the second interrupt, just before the visible
        // portion.
    }

    // Set this before we set the `DRAW_THIS_LINE` flag.
    NEXT_SCAN_LINE.store(next_timing_line, Ordering::Relaxed);

    // Work out what sort of sync pulses we need on the *next* scan-line, and
    // also tell the main thread what to draw ready for the *next* scan-line.
    let buffer = if next_timing_line <= timing_data.visible_lines_ends_at {
        // A visible line is *up next* so maybe start drawing it *right now*.
        DRAW_THIS_LINE.store(draw_now, Ordering::Release);
        &raw const timing_data.visible_line
    } else if next_timing_line <= timing_data.front_porch_end_at {
        // VGA front porch before VGA sync pulse
        &raw const timing_data.vblank_porch_buffer
    } else if next_timing_line <= timing_data.sync_pulse_ends_at {
        // Sync pulse
        &raw const timing_data.vblank_sync_buffer
    } else {
        // VGA back porch following VGA sync pulse.
        &raw const timing_data.vblank_porch_buffer
    };
    // Start transferring the next block of timing info into the FIFO, ready for
    // the next line. We will be back in this interrupt once it starts actually
    // playing.
    dma.ch(TIMING_DMA_CHAN)
        .ch_al3_read_addr_trig()
        .write(|w| w.bits(buffer as u32));
}

// End of file
