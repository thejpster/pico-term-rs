//! Code for handling RGB colours.

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

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// Represents two `RGBColour` pixels packed together.
///
/// The `first` pixel is packed in the lower 16-bits. This is because the PIO
/// shifts-right.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, Default)]
pub struct RGBPair(pub u32);

impl RGBPair {
    pub const fn from_pixels(first: RGBColour, second: RGBColour) -> RGBPair {
        let first: u32 = first.0 as u32;
        let second: u32 = second.0 as u32;
        RGBPair((second << 16) | first)
    }
}

/// Represents a 15-bit colour value.
///
/// Each channel has five bits, and they are packed in `BGR` format. This is
/// so the PIO can shift them out right-first, and we have RED0 assigned to
/// the lowest GPIO pin.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RGBColour(pub u16);

impl RGBColour {
    /// Black (all bits off)
    pub(crate) const BLACK: RGBColour = RGBColour::from_24bit(0x00, 0x00, 0x00);

    /// White
    pub(crate) const WHITE: RGBColour = RGBColour::from_24bit(0xF0, 0xF0, 0xF0);

    /// Make an [`RGBColour`] from a 24-bit RGB triplet.
    ///
    /// Only the top 5 bits of each colour channel are retained, as RGB colour
    /// is a 15-bit value.
    pub const fn from_24bit(red: u8, green: u8, blue: u8) -> RGBColour {
        let red5: u16 = ((red >> 3) & 0b11111) as u16;
        let green5: u16 = ((green >> 3) & 0b11111) as u16;
        let blue5: u16 = ((blue >> 3) & 0b11111) as u16;
        RGBColour((blue5 << 10) | (green5 << 5) | red5)
    }

    /// Make an [`RGBColour`] from a 12-bit RGB triplet.
    ///
    /// Only the bottom 4 bits of each colour channel are retained.
    pub const fn from_12bit(red: u8, green: u8, blue: u8) -> RGBColour {
        let red5: u16 = ((red << 1) & 0b11111) as u16;
        let green5: u16 = ((green << 1) & 0b11111) as u16;
        let blue5: u16 = ((blue << 1) & 0b11111) as u16;
        RGBColour((blue5 << 10) | (green5 << 5) | red5)
    }

    /// Make an [`RGBColour`] from a 15-bit RGB triplet.
    ///
    /// Only the bottom 5 bits of each colour channel are retained, as RGB colour
    /// is a 15-bit value.
    pub const fn from_15bit(red: u8, green: u8, blue: u8) -> RGBColour {
        let red5: u16 = (red & 0b11111) as u16;
        let green5: u16 = (green & 0b11111) as u16;
        let blue5: u16 = (blue & 0b11111) as u16;
        RGBColour((blue5 << 10) | (green5 << 5) | red5)
    }

    /// Get the red component as an 8-bit value
    pub const fn red8(self) -> u8 {
        let red5 = self.0 & 0b11111;
        (red5 << 5) as u8
    }

    /// Get the green component as an 8-bit value
    pub const fn green8(self) -> u8 {
        let green5 = (self.0 >> 5) & 0b11111;
        (green5 << 5) as u8
    }

    /// Get the blue component as an 8-bit value
    pub const fn blue8(self) -> u8 {
        let blue5 = (self.0 >> 10) & 0x0F;
        (blue5 << 5) as u8
    }
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
