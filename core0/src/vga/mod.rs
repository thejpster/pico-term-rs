//! Video related code for Core0

use core::cell::UnsafeCell;

pub mod font16;
pub mod font8;

/// A font
pub struct Font<'a> {
    pub data: &'a [u8],
}

pub static VIDEO_BUFFER: VideoBuffer = VideoBuffer::new();
pub static FONT_BUFFER: FontBuffer = FontBuffer::new();

/// Holds an 80x60 screen full of text, with two bytes per glyph
#[repr(align(4))]
pub struct VideoBuffer {
    contents: UnsafeCell<[u8; Self::LENGTH]>,
}

impl VideoBuffer {
    const LENGTH: usize = 80 * 60 * 2;

    pub const fn new() -> Self {
        VideoBuffer {
            contents: UnsafeCell::new([0u8; Self::LENGTH]),
        }
    }

    pub fn get_ptr(&self) -> *mut u32 {
        // this type is align(4) so this is OK
        self.contents.get() as *mut u32
    }

    pub fn store_at(&self, ch: u8, attr: u8, x: u16, y: u16) {
        if x >= 80 || y >= 60 {
            return;
        }
        let ptr = self.contents.get() as *mut u8;
        let offset = ((y * 80) + x) as usize * 2;
        unsafe {
            ptr.add(offset).write(ch);
            ptr.add(offset).add(1).write(attr);
        }
    }

    pub fn read_at(&self, x: u16, y: u16) -> (u8, u8) {
        if x >= 80 || y >= 60 {
            return (0, 0);
        }
        let ptr = self.contents.get() as *mut u8;
        let offset = ((y * 80) + x) as usize * 2;
        unsafe { (ptr.add(offset).read(), ptr.add(offset).add(1).read()) }
    }
}

unsafe impl Sync for VideoBuffer {}

/// Holds 256 glyphs of an 8x8 or 8x16 font
#[repr(align(4))]
pub struct FontBuffer {
    contents: UnsafeCell<[u8; Self::LENGTH]>,
}

impl FontBuffer {
    const LENGTH: usize = 256 * 16;

    pub const fn new() -> Self {
        FontBuffer {
            contents: UnsafeCell::new([0u8; Self::LENGTH]),
        }
    }

    pub fn get_ptr(&self) -> *const u32 {
        // this type is align(4) so this is OK
        self.contents.get() as *const u32
    }

    pub fn load_font(&self, font: &Font) {
        if font.data.len() > Self::LENGTH {
            defmt::panic!("Font too long!?");
        }
        let ptr = self.contents.get() as *mut u8;
        for (idx, b) in font.data.iter().enumerate() {
            unsafe {
                ptr.add(idx).write(*b);
            }
        }
    }
}

unsafe impl Sync for FontBuffer {}
