# pico-term-rs Core 0 Firmware

## Introduction

This is the Core 0 component of theJPster's alternative firmware for the [RC2014
RP2040 VGA Terminal].

## Features

Core 0 is responsible for:

* Starting Core 1 (which runs the video)
* Processing bytes from the UART
* Running an ANSI Terminal Emulator, and writing characters to shared RAM for
  Core 1 to render
* Handling an attached USB Keyboard
* Talking to an I2C GPIO Expander
* Displaying a built-in settings menu
* Beeping when ASCII BEL received

This firmware is based on the code for the [Neotron Pico], specifically parts of
the [Neotron Pico BIOS] and the [Neotron OS].

This firmware controls the first 256 KiB of Flash, and the first 256 KiB of SRAM
(see [`memory.x`](./memory.x) for details). Note that the Flash and RAM regions
for Core 0 and Core 1 must not overlap.

## Compiling

Install [`rustup`], go to the root of this project and run:

```bash
cargo xtask build
```

You will get `./target/pico-term-rs.uf2` for drag-dropping onto an [RC2014
RP2040 VGA Terminal] in "Mass-Storage Bootloader Mode".

If you want to see the defmt logs, then you can connect a Raspberry Pi Debug
Probe to your RP2040, install [`probe-rs`], and then in this folder run:

```bash
cargo run --release
```

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

See the [top-level README](../README.md) for details.

[RC2014 RP2040 VGA Terminal]: https://rc2014.co.uk/modules/rp2040-vga-terminal
[Neotron Pico]: https://github.com/neotron-compute/neotron-pico
[Neotron Pico BIOS]: https://github.com/neotron-compute/neotron-pico-bios
[Neotron OS]: https://github.com/neotron-compute/neotron-os
[`rustup`]: https://rustup.rs
[`probe-rs`]: https://probe.rs
