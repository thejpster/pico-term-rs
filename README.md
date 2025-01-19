# pico-term-rs

## Introduction

This is theJPster's alternative firmware for the [RC2014 RP2040 VGA Terminal].

## Features

This firmware is a work in progress, but is my intention to support:

* Basic ANSI terminal emulator over UART
* 640x480 @ 60 Hz and 640x400 @ 70 Hz VGA video output
* Text mode with both 8x8 and 8x16 fonts supported
* 16 foreground colours, 8 background colours, from a palette of 32,768
* USB Host support for USB Keyboard input with multiple keyboard layouts
* Built-in settings menu
* Beeps when ASCII BEL received

This firmware is based on the code for the [Neotron Pico], specifically parts of
the [Neotron Pico BIOS] and the [Neotron OS].

## Memory Layout

There are three Rust applications here:

* [xtask](./xtask) - a build tool that builds this firmware
* [core0](./core0) - an application that runs on Core 0
  * Uses the first 256 KiB of Flash, and the first 256 KiB of SRAM.
  * Global variables are at the top of SRAM and stack is underneath.
  * Boots first, and is responsible for starting Core 1
* [core1](./core1) - an application that runs on Core 1
  * Controls the second 256 KiB of Flash, and the top 8 KiB of SRAM
* Core 0 and Core 1 communicate by message-passing using the SIO FIFO

## Compiling

Install [`rustup`] and run:

```bash
cargo xtask build
```

You will get `./target/pico-term-rs.uf2` for drag-dropping onto an [RC2014
RP2040 VGA Terminal] in "Mass-Storage Bootloader Mode".

## Changelog

See [CHANGELOG.md](./CHANGELOG.md)

## Licence

```text
Copyright (c) Jonathan 'theJPster' Pallant and the pico-term-rs Developers, 2025

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

See the full text in [LICENSE.txt](./LICENSE.txt). Broadly, we (the developers)
interpret this to mean (and note that we are not lawyers and this is not legal
advice) that if you give someone a board programmed with this firmware, you must
also give them one of:

* Complete and corresponding source code (e.g. as a link to your **own** on-line
  Git repo) for any GPL components (e.g. the BIOS and the OS), as supplied on
  the board.
* A written offer to provide complete and corresponding source code on
  request.

If you are not offering pre-programmed boards commercially (i.e. you are not
selling the board for commercial gain), and you are using an unmodified upstream
version of the source code, then the third option is to give them:

* A link to the tag/commit-hash on the relevant official Github
  repositories - <https://github.com/thejpster/picoterm-rs>.

This is to ensure everyone always has the freedom to access the pico-term-rs
source code used in their [RC2014 RP2040 VGA Terminal].

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you shall be licensed as above,
without any additional terms or conditions.

[RC2014 RP2040 VGA Terminal]: https://rc2014.co.uk/modules/rp2040-vga-terminal
[Neotron Pico]: https://github.com/neotron-compute/neotron-pico
[Neotron Pico BIOS]: https://github.com/neotron-compute/neotron-pico-bios
[Neotron OS]: https://github.com/neotron-compute/neotron-os
[`rustup`]: https://rustup.rs
