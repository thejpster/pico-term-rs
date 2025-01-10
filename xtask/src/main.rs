//! The pico-term-rs firmware builder

use std::{
    io::Write as _,
    path::{Path, PathBuf},
};

static HELP_TEXT: &str = r#"
The pico-term-rs firmware builder version $CARGO_PKG_VERSION.

$CARGO_PKG_LICENSE

Options:

  * "help" - print this help text
  * "build" - compile the firmware into a UF2 file
  * "fmt" - format the code
  * "fmt-check" - check the code is formatted
  * "clippy" - run clippy on the firmware
"#;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Entry point to the program
///
/// We don't use anyhow or eyre here to keep the dependencies to a minimum
fn main() -> Result<()> {
    let command = std::env::args().nth(1).unwrap_or("help".to_string());
    match command.as_str() {
        "build" => {
            println!("Building...");
            build()?;
        }
        "fmt" => {
            println!("Running fmt...");
            run_all(&["fmt"])?;
        }
        "fmt-check" => {
            println!("Running fmt --check...");
            run_all(&["fmt", "--check"])?;
        }
        "clippy" => {
            println!("Running clippy...");
            run_all(&["clippy"])?;
        }
        _ => {
            let help = HELP_TEXT.to_string();
            let help = help.replace("$CARGO_PKG_VERSION", env!("CARGO_PKG_VERSION"));
            let help = help.replace("$CARGO_PKG_LICENSE", env!("CARGO_PKG_LICENSE"));
            println!("{}", help);
        }
    }
    Ok(())
}

/// Build the firmware
fn build() -> Result<()> {
    get_target("thumbv6m-none-eabi")?;
    let core1_elf = build_core("core1")?;
    let core0_elf = build_core("core0")?;
    let target_hex = Path::new("./target/pico-term-rs.hex");
    let target_uf2 = Path::new("./target/pico-term-rs.uf2");
    merge_firmware(&core0_elf, &core1_elf, target_hex)?;
    make_uf2(target_hex, target_uf2)?;
    Ok(())
}

/// Add our target to rustup
fn get_target(target: &str) -> Result<()> {
    let mut command = std::process::Command::new("rustup");
    command.arg("target");
    command.arg("add");
    command.arg(target);
    run_command(command)?;
    let mut command = std::process::Command::new("rustup");
    command.arg("component");
    command.arg("add");
    command.arg("llvm-tools");
    run_command(command)?;
    Ok(())
}

/// Build the firmware for one core
fn build_core(core: &str) -> Result<PathBuf> {
    println!("Building {}", core);
    let mut command = std::process::Command::new("cargo");
    command.arg("build");
    command.arg("--release");
    command.current_dir(core);
    run_command(command)?;
    let mut output_elf = Path::new(".").to_path_buf();
    output_elf.push(core);
    output_elf.push("target");
    output_elf.push("thumbv6m-none-eabi");
    output_elf.push("release");
    output_elf.push(core);
    Ok(output_elf)
}

/// Combine the two firmwares
///
/// We convert them into .hex files, and then merge those hex files.
fn merge_firmware(core0_elf: &Path, core1_elf: &Path, output_hex: &Path) -> Result<()> {
    println!(
        "Merging {} and {} into {}",
        core0_elf.display(),
        core1_elf.display(),
        output_hex.display()
    );
    let core0_hex = core0_elf.with_extension("hex");
    let core1_hex = core1_elf.with_extension("hex");
    let llvm_objcopy = llvm_tool("llvm-objcopy")?;
    println!("Running {}", llvm_objcopy.display());
    for pairs in [(&core0_elf, &core0_hex), (&core1_elf, &core1_hex)] {
        println!("Converting {} to {}", pairs.0.display(), pairs.1.display());
        let mut command = std::process::Command::new(&llvm_objcopy);
        command.arg("-O");
        command.arg("ihex");
        command.arg(pairs.0);
        command.arg(pairs.1);
        run_command(command)?;
    }
    println!(
        "Reading and merging {} and {} to {}",
        core0_hex.display(),
        core1_hex.display(),
        output_hex.display()
    );
    let hex0 = std::fs::read(core0_hex)?;
    let hex1 = std::fs::read(core1_hex)?;
    println!("Creating {}", output_hex.display());
    let parent = {
        let mut p = output_hex.to_path_buf();
        p.pop();
        p
    };
    std::fs::create_dir_all(parent)?;
    let mut output = std::fs::File::create(output_hex)?;
    output.write_all(&hex0)?;
    output.write_all(&hex1)?;
    Ok(())
}

/// Turn a hex file into a UF2
fn make_uf2(input_hex: &Path, output_uf2: &Path) -> Result<()> {
    println!(
        "Converting {} to {}",
        input_hex.display(),
        output_uf2.display()
    );
    Ok(())
}

/// Run a cargo command against all sub-projects
fn run_all(args: &[&str]) -> Result<()> {
    for x in ["xtask", "core0", "core1"] {
        let mut command = std::process::Command::new("cargo");
        command.args(args);
        command.current_dir(x);
        run_command(command)?;
    }
    Ok(())
}

/// Run a command to completion and check it worked OK
fn run_command(mut command: std::process::Command) -> Result<()> {
    let mut child = command.spawn()?;
    let status = child.wait()?;
    if !status.success() {
        return Err(Box::from("Command failed"));
    }
    Ok(())
}

/// Get the rust sysroot where the LLVM tools live
fn sysroot() -> Result<PathBuf> {
    let output = std::process::Command::new("rustc")
        .arg("--print")
        .arg("target-libdir")
        .output()?;
    let sysroot = String::from_utf8(output.stdout)?;
    let sysroot: std::path::PathBuf = sysroot.trim().into();
    Ok(sysroot)
}

/// Get the path to an llvm-tools tool
fn llvm_tool(name: &str) -> Result<PathBuf> {
    let mut output = sysroot()?;
    output.pop();
    output.push("bin");
    output.push(name);
    Ok(output)
}

// END OF FILE
