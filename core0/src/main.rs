//! # pico-term-rs
//!
//! Rust Firmware for the RC2014 RP2040 VGA Terminal.
//!
//! This is the firmware for Core 0.

// -----------------------------------------------------------------------------
// Licence Statement
// -----------------------------------------------------------------------------
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

#![no_std]
#![no_main]

use core::{fmt::Write, sync::atomic::AtomicBool};

use defmt_rtt as _;
use neotron_common_bios::video::{Attr, TextBackgroundColour, TextForegroundColour};
use panic_probe as _;

use rp2040_hal::{self as hal, binary_info};

mod hw;
mod vga;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Are we rendering a panic right now?
pub static IS_PANIC: AtomicBool = AtomicBool::new(false);

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [binary_info::EntryAddr; 5] = [
    binary_info::rp_program_name!(c"pico-term-rs"),
    binary_info::rp_program_description!(c"Rust firmware for the RC2014 RP2040 VGA Terminal"),
    binary_info::rp_cargo_version!(),
    binary_info::rp_program_url!(c"https://github.com/thejpster/pico-term-rs"),
    binary_info::rp_program_build_attribute!(),
];

#[hal::entry]
fn main() -> ! {
    defmt::info!(
        "Firmware {} {} starting up",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );

    let mut hw = hw::Hardware::init();

    let _ = writeln!(hw.uart, "Hello UART!");

    // Load the 8x16 font
    vga::FONT_BUFFER.load_font(&vga::font16::FONT);

    // Set video mode to 0x00 (640x480 @ 60Hz, 80x30, 8x16 font)
    hw.fifo.write_blocking(0xA000_0000);
    let msg = hw.fifo.read_blocking();
    defmt::info!("Set video mode, got {=u32:08x} from Core 1", msg);

    // Set framebuffer pointer
    let command = 0xA100_0000;
    let ptr = vga::VIDEO_BUFFER.get_ptr() as u32;
    let send = command | ((ptr - 0x2000_0000) >> 2);
    hw.fifo.write_blocking(send);
    let msg = hw.fifo.read_blocking();
    defmt::info!(
        "Set buffer pointer {=u32:08x}, got {=u32:08x} from Core 1",
        send,
        msg
    );

    // Set font pointer
    let command = 0xA200_0000;
    let ptr = vga::FONT_BUFFER.get_ptr() as u32;
    let send = command | ((ptr - 0x2000_0000) >> 2);
    hw.fifo.write_blocking(send);
    let msg = hw.fifo.read_blocking();
    defmt::info!(
        "Set font pointer {=u32:08x}, got {=u32:08x} from Core 1",
        send,
        msg
    );

    let mut vte: vte::Parser<16> = vte::Parser::new_with_size();

    let mut console = unsafe { Console::new(80, 30) };

    console.clear();
    console.cursor_enable();

    loop {
        let mut buffer = [0u8; 1];
        if let Ok(1) = hw.uart.read_raw(&mut buffer) {
            // defmt::info!("Got {:02x} ({})", buffer[0], buffer[0] as char);
            vte.advance(&mut console, buffer[0]);
        }
    }
}

struct Console {
    /// The width of the screen in characters
    width_chars: u16,
    /// The height of the screen in characters
    height_chars: u16,
    /// The current row position in characters
    row: u16,
    /// The current column position in characters
    col: u16,
    /// The attribute to apply to the next character we draw
    attr: Attr,
    /// Have we seen the ANSI 'bold' command?
    bright: bool,
    /// Have we seen the ANSI 'reverse' command?
    reverse: bool,
    /// Should we draw a cursor?
    cursor_wanted: bool,
    /// How many times has the cursor been turned off?
    ///
    /// The cursor is only enabled when at `0`.
    cursor_depth: u8,
    /// What character should be where the cursor currently is?
    cursor_holder: Option<u8>,
}

impl Console {
    const DEFAULT_ATTR: Attr = Attr::new(
        TextForegroundColour::LightGray,
        TextBackgroundColour::Black,
        false,
    );

    /// Create a new console
    ///
    /// # Safety
    ///  
    /// `addr` must point to a buffer of at least `width * height * 2`` bytes.
    pub unsafe fn new(width: u16, height: u16) -> Console {
        Console {
            width_chars: width,
            height_chars: height,
            col: 0,
            row: 0,
            attr: Self::DEFAULT_ATTR,
            bright: false,
            reverse: false,
            cursor_wanted: false,
            cursor_depth: 0,
            cursor_holder: None,
        }
    }

    /// Replace the glyph at the current location with a cursor.
    fn cursor_enable(&mut self) {
        self.cursor_depth = self.cursor_depth.saturating_sub(1);
        if self.cursor_depth == 0 && self.cursor_wanted && self.cursor_holder.is_none() {
            // Remember what was where our cursor is (unless the cursor is off-screen, when we make something up)
            if self.row < self.height_chars && self.col < self.width_chars {
                let value = self.read();
                self.write_at(self.row, self.col, b'_', true);
                self.cursor_holder = Some(value);
            } else {
                self.cursor_holder = Some(b' ');
            }
        }
    }

    /// Replace the cursor at the current location with its previous contents.
    fn cursor_disable(&mut self) {
        if let Some(glyph) = self.cursor_holder.take() {
            if self.row < self.height_chars && self.col < self.width_chars {
                // cursor was on-screen, so restore it
                self.write(glyph);
            }
        }
        self.cursor_depth += 1;
    }

    /// Move the cursor relative to the current location.
    ///
    /// Clamps to the visible screen.
    fn move_cursor_relative(&mut self, rows: i16, cols: i16) {
        let new_row = self.row as i16 + rows;
        if new_row < 0 {
            self.row = 0;
        } else if new_row >= self.height_chars as i16 {
            self.row = self.height_chars - 1;
        } else {
            self.row = new_row as u16;
        }
        let new_col = self.col as i16 + cols;
        if new_col < 0 {
            self.col = 0;
        } else if new_col >= self.width_chars as i16 {
            self.col = self.width_chars - 1;
        } else {
            self.col = new_col as u16;
        }
    }

    /// Move the cursor to the given location.
    ///
    /// Clamps to the visible screen.
    fn move_cursor_absolute(&mut self, rows: u16, cols: u16) {
        // move it
        self.row = rows;
        self.col = cols;
        // clamp it
        self.move_cursor_relative(0, 0);
    }

    /// Move the cursor to 0,0
    fn home(&mut self) {
        self.move_cursor_absolute(0, 0);
    }

    /// If we are currently positioned off-screen, scroll and fix that.
    ///
    /// We defer this so you can write the last char on the last line without
    /// causing it to scroll pre-emptively.
    fn scroll_as_required(&mut self) {
        while self.col >= self.width_chars {
            self.col -= self.width_chars;
            self.row += 1;
        }
        while self.row >= self.height_chars {
            self.row -= 1;
            self.scroll_page();
        }
    }

    /// Blank the screen
    fn clear(&mut self) {
        self.cursor_disable();
        for row in 0..self.height_chars {
            for col in 0..self.width_chars {
                self.write_at(row, col, b' ', false);
            }
        }
        self.home();
        self.cursor_enable();
    }

    /// Put a glyph at the current position on the screen.
    ///
    /// Don't do this if the cursor is enabled.
    fn write(&mut self, glyph: u8) {
        self.write_at(self.row, self.col, glyph, false);
    }

    /// Put a glyph at a given position on the screen.
    ///
    /// Don't do this if the cursor is enabled.
    fn write_at(&mut self, row: u16, col: u16, glyph: u8, is_cursor: bool) {
        assert!(row < self.height_chars, "{} >= {}?", row, self.height_chars);
        assert!(col < self.width_chars, "{} => {}?", col, self.width_chars);
        if !crate::IS_PANIC.load(core::sync::atomic::Ordering::Relaxed) && !is_cursor {
            assert!(self.cursor_holder.is_none());
        }

        let attr = if self.reverse {
            let new_fg = self.attr.bg().make_foreground();
            let new_bg = self.attr.fg().make_background();
            Attr::new(new_fg, new_bg, false)
        } else {
            self.attr
        };

        vga::VIDEO_BUFFER.store_at(glyph, attr.as_u8(), col, row);
    }

    /// Read a glyph at the current position
    ///
    /// Don't do this if the cursor is enabled.
    fn read(&mut self) -> u8 {
        self.read_at(self.row, self.col)
    }

    /// Read a glyph at the given position
    ///
    /// Don't do this if the cursor is enabled.
    fn read_at(&mut self, row: u16, col: u16) -> u8 {
        assert!(row < self.height_chars, "{} >= {}?", row, self.height_chars);
        assert!(col < self.width_chars, "{} => {}?", col, self.width_chars);
        if !crate::IS_PANIC.load(core::sync::atomic::Ordering::Relaxed) {
            assert!(self.cursor_holder.is_none());
        }
        let (glyph, _attr) = vga::VIDEO_BUFFER.read_at(col, row);
        glyph
    }

    /// Move everyone on screen up one line, losing the top line.
    ///
    /// The bottom line will be all space characters.
    fn scroll_page(&mut self) {
        let row_len_words = self.width_chars / 2;
        unsafe {
            let addr = vga::VIDEO_BUFFER.get_ptr();
            // Scroll rows[1..=height-1] to become rows[0..=height-2].
            core::ptr::copy(
                addr.add(row_len_words as usize),
                addr,
                (row_len_words * (self.height_chars - 1)) as usize,
            );
        }
        // Blank the bottom line of the screen (rows[height-1]).
        for col in 0..self.width_chars {
            self.write_at(self.height_chars - 1, col, b' ', false);
        }
    }

    /// Convert a Unicode Scalar Value to a font glyph.
    ///
    /// Zero-width and modifier Unicode Scalar Values (e.g. `U+0301 COMBINING,
    /// ACCENT`) are not supported. Normalise your Unicode before calling
    /// this function.
    fn map_char_to_glyph(input: char) -> u8 {
        // This fixed table only works for the default font. When we support
        // changing font, we will need to plug-in a different table for each font.
        match input {
            '\u{0020}'..='\u{007E}' => input as u8,
            // 0x80 to 0x9F are the C1 control codes with no visual
            // representation
            '\u{00A0}' => 255, // NBSP
            '\u{00A1}' => 173, // ¡
            '\u{00A2}' => 189, // ¢
            '\u{00A3}' => 156, // £
            '\u{00A4}' => 207, // ¤
            '\u{00A5}' => 190, // ¥
            '\u{00A6}' => 221, // ¦
            '\u{00A7}' => 245, // §
            '\u{00A8}' => 249, // ¨
            '\u{00A9}' => 184, // ©
            '\u{00AA}' => 166, // ª
            '\u{00AB}' => 174, // «
            '\u{00AC}' => 170, // ¬
            '\u{00AD}' => 240, // - (Soft Hyphen)
            '\u{00AE}' => 169, // ®
            '\u{00AF}' => 238, // ¯
            '\u{00B0}' => 248, // °
            '\u{00B1}' => 241, // ±
            '\u{00B2}' => 253, // ²
            '\u{00B3}' => 252, // ³
            '\u{00B4}' => 239, // ´
            '\u{00B5}' => 230, // µ
            '\u{00B6}' => 244, // ¶
            '\u{00B7}' => 250, // ·
            '\u{00B8}' => 247, // ¸
            '\u{00B9}' => 251, // ¹
            '\u{00BA}' => 167, // º
            '\u{00BB}' => 175, // »
            '\u{00BC}' => 172, // ¼
            '\u{00BD}' => 171, // ½
            '\u{00BE}' => 243, // ¾
            '\u{00BF}' => 168, // ¿
            '\u{00C0}' => 183, // À
            '\u{00C1}' => 181, // Á
            '\u{00C2}' => 182, // Â
            '\u{00C3}' => 199, // Ã
            '\u{00C4}' => 142, // Ä
            '\u{00C5}' => 143, // Å
            '\u{00C6}' => 146, // Æ
            '\u{00C7}' => 128, // Ç
            '\u{00C8}' => 212, // È
            '\u{00C9}' => 144, // É
            '\u{00CA}' => 210, // Ê
            '\u{00CB}' => 211, // Ë
            '\u{00CC}' => 222, // Ì
            '\u{00CD}' => 214, // Í
            '\u{00CE}' => 215, // Î
            '\u{00CF}' => 216, // Ï
            '\u{00D0}' => 209, // Ð
            '\u{00D1}' => 165, // Ñ
            '\u{00D2}' => 227, // Ò
            '\u{00D3}' => 224, // Ó
            '\u{00D4}' => 226, // Ô
            '\u{00D5}' => 229, // Õ
            '\u{00D6}' => 153, // Ö
            '\u{00D7}' => 158, // ×
            '\u{00D8}' => 157, // Ø
            '\u{00D9}' => 235, // Ù
            '\u{00DA}' => 233, // Ú
            '\u{00DB}' => 234, // Û
            '\u{00DC}' => 154, // Ü
            '\u{00DD}' => 237, // Ý
            '\u{00DE}' => 232, // Þ
            '\u{00DF}' => 225, // ß
            '\u{00E0}' => 133, // à
            '\u{00E1}' => 160, // á
            '\u{00E2}' => 131, // â
            '\u{00E3}' => 198, // ã
            '\u{00E4}' => 132, // ä
            '\u{00E5}' => 134, // å
            '\u{00E6}' => 145, // æ
            '\u{00E7}' => 135, // ç
            '\u{00E8}' => 138, // è
            '\u{00E9}' => 130, // é
            '\u{00EA}' => 136, // ê
            '\u{00EB}' => 137, // ë
            '\u{00EC}' => 141, // ì
            '\u{00ED}' => 161, // í
            '\u{00EE}' => 140, // î
            '\u{00EF}' => 139, // ï
            '\u{00F0}' => 208, // ð
            '\u{00F1}' => 164, // ñ
            '\u{00F2}' => 149, // ò
            '\u{00F3}' => 162, // ó
            '\u{00F4}' => 147, // ô
            '\u{00F5}' => 228, // õ
            '\u{00F6}' => 148, // ö
            '\u{00F7}' => 246, // ÷
            '\u{00F8}' => 155, // ø
            '\u{00F9}' => 151, // ù
            '\u{00FA}' => 163, // ú
            '\u{00FB}' => 150, // û
            '\u{00FC}' => 129, // ü
            '\u{00FD}' => 236, // ý
            '\u{00FE}' => 231, // þ
            '\u{00FF}' => 152, // ÿ
            '\u{0131}' => 213, // ı
            '\u{0192}' => 159, // ƒ
            '\u{2017}' => 242, // ‗
            '\u{2022}' => 7,   // •
            '\u{203C}' => 19,  // ‼
            '\u{2190}' => 27,  // ←
            '\u{2191}' => 24,  // ↑
            '\u{2192}' => 26,  // →
            '\u{2193}' => 25,  // ↓
            '\u{2194}' => 29,  // ↔
            '\u{2195}' => 18,  // ↕
            '\u{21A8}' => 23,  // ↨
            '\u{221F}' => 28,  // ∟
            '\u{2302}' => 127, // ⌂
            '\u{2500}' => 196, // ─
            '\u{2502}' => 179, // │
            '\u{250C}' => 218, // ┌
            '\u{2510}' => 191, // ┐
            '\u{2514}' => 192, // └
            '\u{2518}' => 217, // ┘
            '\u{251C}' => 195, // ├
            '\u{2524}' => 180, // ┤
            '\u{252C}' => 194, // ┬
            '\u{2534}' => 193, // ┴
            '\u{253C}' => 197, // ┼
            '\u{2550}' => 205, // ═
            '\u{2551}' => 186, // ║
            '\u{2554}' => 201, // ╔
            '\u{2557}' => 187, // ╗
            '\u{255A}' => 200, // ╚
            '\u{255D}' => 188, // ╝
            '\u{2560}' => 204, // ╠
            '\u{2563}' => 185, // ╣
            '\u{2566}' => 203, // ╦
            '\u{2569}' => 202, // ╩
            '\u{256C}' => 206, // ╬
            '\u{2580}' => 223, // ▀
            '\u{2584}' => 220, // ▄
            '\u{2588}' => 219, // █
            '\u{2591}' => 176, // ░
            '\u{2592}' => 177, // ▒
            '\u{2593}' => 178, // ▓
            '\u{25A0}' => 254, // ■
            '\u{25AC}' => 22,  // ▬
            '\u{25B2}' => 30,  // ▲
            '\u{25BA}' => 16,  // ►
            '\u{25BC}' => 31,  // ▼
            '\u{25C4}' => 17,  // ◄
            '\u{25CB}' => 9,   // ○
            '\u{25D8}' => 8,   // ◘
            '\u{25D9}' => 10,  // ◙
            '\u{263A}' => 1,   // ☺
            '\u{263B}' => 2,   // ☻
            '\u{263C}' => 15,  // ☼
            '\u{2640}' => 12,  // ♀
            '\u{2642}' => 11,  // ♂
            '\u{2660}' => 6,   // ♠
            '\u{2663}' => 5,   // ♣
            '\u{2665}' => 3,   // ♥
            '\u{2666}' => 4,   // ♦
            '\u{266A}' => 13,  // ♪
            '\u{266B}' => 14,  // ♫
            _ => b'?',
        }
    }
}

impl vte::Perform for Console {
    /// Draw a character to the screen and update states.
    fn print(&mut self, ch: char) {
        self.scroll_as_required();
        self.write(Self::map_char_to_glyph(ch));
        self.col += 1;
    }

    /// Execute a C0 or C1 control function.
    fn execute(&mut self, byte: u8) {
        self.scroll_as_required();
        match byte {
            0x08 => {
                // This is a backspace, so we go back one character (if we
                // can). We expect the caller to provide "\u{0008} \u{0008}"
                // to actually erase the char then move the cursor over it.
                if self.col > 0 {
                    self.col -= 1;
                }
            }
            b'\r' => {
                self.col = 0;
            }
            b'\t' => {
                self.col = (self.col + 8) & !7;
            }
            b'\n' => {
                self.col = 0;
                self.row += 1;
            }
            _ => {
                // ignore unknown C0 or C1 control code
            }
        }
        // We may now be off-screen, but that's OK because we will scroll before
        // we print the next thing.
    }

    /// A final character has arrived for a CSI sequence
    ///
    /// The `ignore` flag indicates that either more than two intermediates arrived
    /// or the number of parameters exceeded the maximum supported length,
    /// and subsequent characters were ignored.
    fn csi_dispatch(
        &mut self,
        params: &vte::Params,
        intermediates: &[u8],
        _ignore: bool,
        action: char,
    ) {
        // Just in case you want a single parameter, here it is
        let mut first = *params.iter().next().and_then(|s| s.first()).unwrap_or(&1) as i32;
        let mut second = *params.iter().nth(1).and_then(|s| s.first()).unwrap_or(&1) as i32;

        match action {
            'm' => {
                // Select Graphic Rendition
                for p in params.iter() {
                    let Some(p) = p.first() else {
                        // Can't handle sub-params, i.e. params with more than one value
                        return;
                    };
                    defmt::info!("SGR {=u16}", *p);
                    match *p {
                        0 => {
                            // Reset, or normal
                            self.attr = Self::DEFAULT_ATTR;
                            self.bright = false;
                            self.reverse = false;
                        }
                        1 => {
                            // Bold intensity
                            self.bright = true;
                        }
                        7 => {
                            // Reverse video
                            self.reverse = true;
                        }
                        22 => {
                            // Normal intensity
                            self.bright = false;
                        }
                        // Foreground
                        30 => {
                            self.attr.set_fg(TextForegroundColour::Black);
                        }
                        31 => {
                            self.attr.set_fg(TextForegroundColour::Red);
                        }
                        32 => {
                            self.attr.set_fg(TextForegroundColour::Green);
                        }
                        33 => {
                            self.attr.set_fg(TextForegroundColour::Brown);
                        }
                        34 => {
                            self.attr.set_fg(TextForegroundColour::Blue);
                        }
                        35 => {
                            self.attr.set_fg(TextForegroundColour::Magenta);
                        }
                        36 => {
                            self.attr.set_fg(TextForegroundColour::Cyan);
                        }
                        37 | 39 => {
                            self.attr.set_fg(TextForegroundColour::LightGray);
                        }
                        // Background
                        40 => {
                            self.attr.set_bg(TextBackgroundColour::Black);
                        }
                        41 => {
                            self.attr.set_bg(TextBackgroundColour::Red);
                        }
                        42 => {
                            self.attr.set_bg(TextBackgroundColour::Green);
                        }
                        43 => {
                            self.attr.set_bg(TextBackgroundColour::Brown);
                        }
                        44 => {
                            self.attr.set_bg(TextBackgroundColour::Blue);
                        }
                        45 => {
                            self.attr.set_bg(TextBackgroundColour::Magenta);
                        }
                        46 => {
                            self.attr.set_bg(TextBackgroundColour::Cyan);
                        }
                        47 | 49 => {
                            self.attr.set_bg(TextBackgroundColour::LightGray);
                        }
                        _ => {
                            // Ignore unknown code
                        }
                    }
                }
                // Now check if we're bright, and make it brighter. We do this
                // last, because they might set the colour first and set the
                // bright bit afterwards.
                if self.bright {
                    self.attr.set_fg(self.attr.fg().brighten())
                }
            }
            'A' => {
                // Cursor Up
                if first == 0 {
                    first = 1;
                }
                self.move_cursor_relative(-first as i16, 0);
            }
            'B' => {
                // Cursor Down
                if first == 0 {
                    first = 1;
                }
                self.move_cursor_relative(first as i16, 0);
            }
            'C' => {
                // Cursor Forward
                if first == 0 {
                    first = 1;
                }
                self.move_cursor_relative(0, first as i16);
            }
            'D' => {
                // Cursor Back
                if first == 0 {
                    first = 1;
                }
                self.move_cursor_relative(0, -first as i16);
            }
            'E' => {
                // Cursor next line
                if first == 0 {
                    first = 1;
                }
                self.move_cursor_relative(first as i16, 0);
                self.move_cursor_absolute(self.row, 0);
            }
            'F' => {
                // Cursor previous line
                if first == 0 {
                    first = 1;
                }
                self.move_cursor_relative(-first as i16, 0);
                self.move_cursor_absolute(self.row, 0);
            }
            'G' => {
                // Cursor horizontal absolute
                if first == 0 {
                    first = 1;
                }
                // We are zero-indexed, ANSI is 1-indexed
                self.move_cursor_absolute(self.row, (first - 1) as u16);
            }
            'H' | 'f' => {
                // Cursor Position (or Horizontal Vertical Position)
                if first == 0 {
                    first = 1;
                }
                if second == 0 {
                    second = 1;
                }
                // We are zero-indexed, ANSI is 1-indexed
                self.move_cursor_absolute((first - 1) as u16, (second - 1) as u16);
            }
            'J' => {
                // Erase in Display
                match first {
                    0 => {
                        // Erase the cursor through the end of the display
                        for row in 0..self.height_chars {
                            for col in 0..self.width_chars {
                                if row > self.row || (row == self.row && col >= self.col) {
                                    self.write_at(row, col, b' ', false);
                                }
                            }
                        }
                    }
                    1 => {
                        // Erase from the beginning of the display through the cursor
                        for row in 0..self.height_chars {
                            for col in 0..self.width_chars {
                                if row < self.row || (row == self.row && col <= self.col) {
                                    self.write_at(row, col, b' ', false);
                                }
                            }
                        }
                    }
                    2 => {
                        // Erase the complete display
                        for row in 0..self.height_chars {
                            for col in 0..self.width_chars {
                                self.write_at(row, col, b' ', false);
                            }
                        }
                    }
                    _ => {
                        // Ignore it
                    }
                }
            }
            'K' => {
                // Erase in Line
                match first {
                    0 => {
                        // Erase the cursor through the end of the line
                        for col in self.col..self.width_chars {
                            self.write_at(self.row, col, b' ', false);
                        }
                    }
                    1 => {
                        // Erase from the beginning of the line through the cursor
                        for col in 0..=self.col {
                            self.write_at(self.row, col, b' ', false);
                        }
                    }
                    2 => {
                        // Erase the complete line
                        for col in 0..self.width_chars {
                            self.write_at(self.row, col, b' ', false);
                        }
                    }
                    _ => {
                        // Ignore it
                    }
                }
            }
            'n' if first == 6 => {
                // Device Status Report - todo.
                //
                // We should send "\u{001b}[<rows>;<cols>R" where <rows> and
                // <cols> are integers for 1-indexed rows and columns
                // respectively. But for that we need an input buffer to put bytes into.
            }
            'h' if intermediates.first().cloned() == Some(b'?') => {
                // DEC special code for Cursor On. It'll be activated whenever
                // we finish what we're printing.
                self.cursor_wanted = true;
            }
            'l' if intermediates.first().cloned() == Some(b'?') => {
                // DEC special code for Cursor Off.
                self.cursor_wanted = false;
            }
            _ => {
                // Unknown code - ignore it
            }
        }
    }
}

// End of file
