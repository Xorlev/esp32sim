//! Shared by the tests in `cli/tests`, `esp32c6/tests` and `wasm/tests` (included with `#[path]`):
//! the goldens and the emulator binaries. The helpers a test in another crate can use too are
//! `esp32sim-testkit`'s, re-exported here.
#![allow(dead_code, unused_imports)]
pub use esp32sim_testkit::{rom, sha256, sha256_hex, tmp};
use std::path::{Path, PathBuf};
use std::process::Command;

/// Workspace root (this file lives in `<root>/tests/`).
pub fn root() -> PathBuf { Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf() }
pub fn golden(name: &str) -> PathBuf { root().join("tests/golden").join(name) }

pub struct Run { pub stdout: String, pub stderr: String, pub insns: u64 }

/// Run an emulator binary; stdout is the guest console, stderr the emulator's own report.
pub fn run(bin: &str, args: &[&str]) -> Run {
    let out = Command::new(bin).args(args).current_dir(root()).output().unwrap_or_else(|e| panic!("{}: {}", bin, e));
    let stderr = String::from_utf8_lossy(&out.stderr).to_string();
    assert!(out.status.success(), "{} {:?} failed: {}\n{}", bin, args, out.status, tail(&stderr));
    let insns = insn_count(&stderr).unwrap_or_else(|| panic!("no `[emu] stop:` line in stderr:\n{}", tail(&stderr)));
    Run { stdout: String::from_utf8_lossy(&out.stdout).to_string(), stderr, insns }
}

/// Instructions executed, from the `[emu] stop:` line (both cores on the S3, one on the C3).
pub fn insn_count(stderr: &str) -> Option<u64> {
    let line = stderr.lines().find(|l| l.starts_with("[emu] stop:"))?;
    if let Some(i) = line.find("core0 ") {
        let mut it = line[i..].split_whitespace();
        let c0: u64 = it.nth(1)?.parse().ok()?; let c1: u64 = it.nth(2)?.parse().ok()?;
        return Some(c0 + c1);
    }
    let i = line.find(" — ")?; line[i + " — ".len()..].split_whitespace().next()?.parse().ok()
}

fn tail(s: &str) -> String { let n = s.len(); s[n.saturating_sub(1500)..].to_string() }

fn update_goldens() -> bool { matches!(std::env::var("UPDATE_GOLDENS").as_deref(), Ok("1" | "true")) }

/// Compare `actual` with the golden file; `UPDATE_GOLDENS=1` rewrites it instead.
pub fn expect_text(name: &str, actual: &str) {
    let p = golden(name);
    if update_goldens() { std::fs::write(&p, actual).unwrap(); return; }
    let want = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {} (run with UPDATE_GOLDENS=1 to create it)", p.display(), e));
    if want != actual {
        let got = p.with_extension("actual");
        std::fs::write(&got, actual).unwrap();
        let (mut ln, mut wl, mut al) = (0, want.lines(), actual.lines());
        loop {
            ln += 1;
            match (wl.next(), al.next()) {
                (Some(w), Some(a)) if w == a => continue,
                (w, a) => panic!("{} differs at line {}:\n  want: {:?}\n  got:  {:?}\n(full output in {})", name, ln, w, a, got.display()),
            }
        }
    }
}

pub fn expect_sha(name: &str, data: &[u8]) {
    let p = golden(name);
    let hex = sha256_hex(data);
    if update_goldens() { std::fs::write(&p, format!("{}\n", hex)).unwrap(); return; }
    let want = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {} (run with UPDATE_GOLDENS=1 to create it)", p.display(), e));
    assert_eq!(want.trim(), hex, "{}: {} bytes hash differently", name, data.len());
}

pub fn expect_u64(name: &str, v: u64) {
    let p = golden(name);
    if update_goldens() { std::fs::write(&p, format!("{}\n", v)).unwrap(); return; }
    let want: u64 = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {}", p.display(), e)).trim().parse().unwrap();
    assert_eq!(want, v, "{}", name);
}
