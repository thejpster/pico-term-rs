# xtask - the builder for the pico-term-rs project

This program knows how to compile firmware for both cores and convert it into a UF2 image.

* `cargo xtask build` - builds a UF2 file
* `cargo xtask fmt` - formats all three projects
* `cargo xtask clippy` - runs clippy on all three projects

It will use `rustup` to install the required target, if missing.
