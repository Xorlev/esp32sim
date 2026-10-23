//! esp32sim-p4 refuses an image built for ESP32-P4 rev >= 3 before it runs, and names the file.
#[path = "../../tests/common.rs"]
mod common;
use common::*;

/// `name` from web/wasm/fw/public with its header's revision range set as a rev >= 3 build sets it.
fn rev3_copy(name: &str) -> std::path::PathBuf {
    let mut d = std::fs::read(root().join("web/wasm/fw/public").join(name)).unwrap();
    d[15..17].copy_from_slice(&300u16.to_le_bytes());
    d[17..19].copy_from_slice(&399u16.to_le_bytes());
    let p = tmp(name);
    std::fs::write(&p, d).unwrap();
    p
}

#[test]
fn a_rev3_bootloader_or_app_stops_the_run_with_the_file_named() {
    let fw = "web/wasm/fw/public";
    for (flag, name) in [("--app", "p4-hello_world.bin"), ("--bootloader", "p4-hello-bootloader.bin")] {
        let patched = rev3_copy(name);
        let (bl, app) = (format!("{fw}/p4-hello-bootloader.bin"), format!("{fw}/p4-hello_world.bin"));
        let mut args = vec!["--boot", "app", "--flash-mb", "2", "--bootloader", &bl, "--ptable", "web/wasm/fw/public/p4-hello-ptable.bin", "--app", &app, "--max-seconds", "1"];
        let at = args.iter().position(|a| *a == flag).unwrap() + 1;
        args[at] = patched.to_str().unwrap();
        let out = std::process::Command::new(env!("CARGO_BIN_EXE_esp32sim-p4")).args(&args).current_dir(root()).output().unwrap();
        let err = String::from_utf8_lossy(&out.stderr);
        assert_eq!(out.status.code(), Some(2), "{err}");
        assert!(err.contains(&format!("[emu] {} was built for ESP32-P4 rev ≥ v3.0 (min_chip_rev_full=300); this model is rev < 3 (v1.0). \
            Rebuild with CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y, or wait for rev 3 support.", patched.display())), "{err}");
        assert!(!err.contains("[emu] stop:"), "the run started:\n{err}");
    }
}
