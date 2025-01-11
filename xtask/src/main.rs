//! The pico-term-rs firmware builder

use std::{
    io::Write,
    path::{Path, PathBuf},
};

/// Where Core1 starts.
///
/// Core 1 starts 256K in. Must match core0/memory.x and core1/link.x
const CORE1_START_OFFSET: u32 = 256 * 1024;

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
    // we need this to exist
    let target_dir = Path::new("target");
    std::fs::create_dir_all(target_dir)?;

    let command = std::env::args().nth(1).unwrap_or("help".to_string());
    match command.as_str() {
        "build" => {
            println!("Building...");
            build(target_dir)?;
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
            get_target("thumbv6m-none-eabi")?;
            run_all(&["clippy"])?;
        }
        _ => {
            let help = HELP_TEXT.to_string();
            let help = help.replace("$CARGO_PKG_VERSION", env!("CARGO_PKG_VERSION"));
            let help = help.replace("$CARGO_PKG_LICENSE", env!("CARGO_PKG_LICENSE"));
            println!("{help}");
        }
    }
    Ok(())
}

/// Build the firmware
fn build(target_dir: &Path) -> Result<()> {
    get_package("flip-link")?;
    get_target("thumbv6m-none-eabi")?;
    let picotool = get_picotool(target_dir)?;
    let target_core0 = target_dir.join("pico-term-rs-core0.elf");
    let target_core1 = target_dir.join("pico-term-rs-core1.elf");
    build_core("core0", &target_core0)?;
    build_core("core1", &target_core1)?;
    let target_bin = target_dir.join("pico-term-rs.bin");
    let target_uf2 = target_dir.join("pico-term-rs.uf2");
    merge_firmware(&target_core0, &target_core1, &target_bin)?;
    make_uf2(&target_bin, &target_uf2, &picotool)?;
    println!(
        "** Complete! You now have {uf2} **",
        uf2 = target_uf2.display()
    );
    Ok(())
}

/// Install a package
fn get_package(package: &str) -> Result<()> {
    let mut command = std::process::Command::new("cargo");
    command.arg("install");
    command.arg("--locked");
    command.arg(package);
    run_command(command)?;
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
fn build_core(core: &str, output_path: &Path) -> Result<()> {
    println!("Building {core}");
    let mut command = std::process::Command::new("cargo");
    command.arg("build");
    command.arg("--release");
    command.current_dir(core);
    run_command(command)?;
    let mut generated_elf = Path::new(".").to_path_buf();
    generated_elf.push(core);
    generated_elf.push("target");
    generated_elf.push("thumbv6m-none-eabi");
    generated_elf.push("release");
    generated_elf.push(core);
    std::fs::copy(&generated_elf, output_path)?;
    Ok(())
}

/// Combine the two firmwares
///
/// We convert them into .bin files, and then merge those bin files.
fn merge_firmware(core0_elf: &Path, core1_elf: &Path, output_bin: &Path) -> Result<()> {
    println!(
        "Merging {elf0} and {elf1} into {bin}",
        elf0 = core0_elf.display(),
        elf1 = core1_elf.display(),
        bin = output_bin.display()
    );
    let core0_bin = core0_elf.with_extension("bin");
    let core1_bin = core1_elf.with_extension("bin");
    let llvm_objcopy = llvm_tool("llvm-objcopy")?;
    println!("Running {objcopy}", objcopy = llvm_objcopy.display());
    for pairs in [(&core0_elf, &core0_bin), (&core1_elf, &core1_bin)] {
        println!(
            "Converting {elf} to {bin}",
            elf = pairs.0.display(),
            bin = pairs.1.display()
        );
        let mut command = std::process::Command::new(&llvm_objcopy);
        command.arg("-O");
        command.arg("binary");
        command.arg(pairs.0);
        command.arg(pairs.1);
        run_command(command)?;
    }
    println!(
        "Reading and merging {bin0} and {bin1} to {bin_out}",
        bin0 = core0_bin.display(),
        bin1 = core1_bin.display(),
        bin_out = output_bin.display()
    );
    let bin0 = std::fs::read(core0_bin)?;
    let bin1 = std::fs::read(core1_bin)?;
    println!("Creating {bin_out}", bin_out = output_bin.display());
    let mut output = std::io::BufWriter::new(std::fs::File::create(output_bin)?);
    output.write_all(&bin0)?;
    // Core 1's image starts after Core 0
    let padding = CORE1_START_OFFSET - bin0.len() as u32;
    for _ in 0..padding {
        output.write_all(&[00])?;
    }
    output.write_all(&bin1)?;
    Ok(())
}

/// Turn a bin file into a UF2
fn make_uf2(input_bin: &Path, output_uf2: &Path, picotool: &Path) -> Result<()> {
    println!(
        "Converting {inp} to {out}",
        inp = input_bin.display(),
        out = output_uf2.display()
    );
    let mut command = std::process::Command::new(picotool);
    command.arg("uf2");
    command.arg("convert");
    command.arg(input_bin);
    command.arg(output_uf2);
    run_command(command)?;
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
    if let Some(cwd) = command.get_current_dir() {
        println!("Running {command:?} in {cwd}", cwd = cwd.display());
    } else {
        println!("Running {command:?}");
    }
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

/// Download picotool for Windows using powershell
#[cfg(target_os = "windows")]
fn get_picotool(target_dir: &Path) -> Result<PathBuf> {
    let picotool_dir = target_dir.join("picotool");
    let picotool_zip = target_dir.join("picotool.zip");
    let picotool_path = picotool_dir.join("picotool.exe");
    if std::fs::exists(&picotool_path)? {
        return Ok(picotool_path);
    }
    // let's use powershell to download and unpack this URL. Every modern
    // Windows install has powershell and it keeps our dependencies small so
    // this xtask builds fast.
    println!("Downloading picotool...");
    let script = format!("& {{
            Invoke-RestMethod -Method Get -Uri https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.1.0-0/picotool-2.1.0-x64-win.zip -OutFile {zipfile} ;
            Expand-Archive {zipfile} -DestinationPath {unpack_dir} ;
        }}",
        zipfile = picotool_zip.to_str().expect("Non-UTF8 path?"),
        unpack_dir = target_dir.to_str().expect("Non-UTF8 path?")
    );
    let mut command = std::process::Command::new("powershell.exe");
    command.arg("-Command");
    command.arg(script);
    run_command(command)?;

    Ok(picotool_path)
}

/// Download picotool for Linux using curl
#[cfg(target_os = "linux")]
fn get_picotool(target_dir: &Path) -> Result<PathBuf> {
    let picotool_dir = target_dir.join("picotool");
    let picotool_path = picotool_dir.join("picotool");
    if std::fs::exists(&picotool_path)? {
        return Ok(picotool_path);
    }

    // let's assume you have curl. Everyone has curl.
    println!("Downloading picotool with curl...");
    let tarball = target_dir.join("picotool.tar.gz");
    let mut command = std::process::Command::new("curl");
    if cfg!(target_arch = "x86_64") {
        command.arg("https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.1.0-0/picotool-2.1.0-x86_64-lin.tar.gz");
    } else if cfg!(target_arch = "aarch64") {
        command.arg("https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.1.0-0/picotool-2.1.0-aarch64-lin.tar.gz");
    } else {
        panic!("picotool isn't available from https://github.com/raspberrypi/pico-sdk-tools for your architecture");
    }
    command.arg("-L");
    command.arg("-o");
    command.arg(&tarball);
    run_command(command)?;

    // unpack it
    let mut command = std::process::Command::new("tar");
    command.arg("xvzf");
    command.arg(&tarball);
    command.arg("-C");
    command.arg(&target_dir);
    run_command(command)?;

    Ok(picotool_path)
}

/// Download picotool for macOS using curl
///
/// This works on Aarch64 and x86-64 because it's a universal binary
#[cfg(target_os = "macos")]
fn get_picotool(target_dir: &Path) -> Result<PathBuf> {
    let picotool_dir = target_dir.join("picotool");
    let picotool_path = picotool_dir.join("picotool");
    if std::fs::exists(&picotool_path)? {
        return Ok(picotool_path);
    }

    // let's assume you have curl. Everyone has curl.
    println!("Downloading picotool with curl...");
    let zipfile = target_dir.join("picotool.zip");
    let mut command = std::process::Command::new("curl");
    command.arg("https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.1.0-0/picotool-2.1.0-mac.zip");
    command.arg("-L");
    command.arg("-o");
    command.arg(&zipfile);
    run_command(command)?;

    // unpack it
    let mut command = std::process::Command::new("unzip");
    command.arg(&zipfile);
    command.arg("-d");
    command.arg(&target_dir);
    run_command(command)?;

    Ok(picotool_path)
}

// End of file
