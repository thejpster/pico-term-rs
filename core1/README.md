# pico-term-rs Core 0 Firmware

## Introduction

This is the Core 1 component of theJPster's alternative firmware for the [RC2014
RP2040 VGA Terminal].

## Features

Core 1 is responsible for:

* Generating 640x480 @ 60 Hz or 640x400 @ 70 Hz video
* Listening to a FIFO for commands from Core 0
  * Change video mode
  * Change buffer pointer
  * Change font pointer

This firmware is based on the code for the [Neotron Pico], specifically parts of
the [Neotron Pico BIOS] and the [Neotron OS].

This firmware controls the second 256 KiB of Flash, and the top 8 KiB of SRAM
(see [`memory.x`](./memory.x) for details). Note that the Flash and RAM regions
for Core 0 and Core 1 must not overlap.

## Compiling

Install [`rustup`], go to the root of this project and run:

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

See the [top-level README](../README.md) for details.

[RC2014 RP2040 VGA Terminal]: https://rc2014.co.uk/modules/rp2040-vga-terminal
[Neotron Pico]: https://github.com/neotron-compute/neotron-pico
[Neotron Pico BIOS]: https://github.com/neotron-compute/neotron-pico-bios
[Neotron OS]: https://github.com/neotron-compute/neotron-os
[`rustup`]: https://rustup.rs
