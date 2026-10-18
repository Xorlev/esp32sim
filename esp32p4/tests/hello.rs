//! The committed ESP32-P4 hello_world (`web/wasm/fw/public/p4-hello-*`, built from
//! `examples/hello_world-p4`) through the bootloader into the application, as far as the core
//! model allows. The plain run stops in the bootloader (`hello-p4` in cli/tests/goldens.rs); this
//! one goes on with a stand-in for the core's missing `cycle`/`cycleh` CSRs.
#[path = "../../tests/common.rs"]
mod common;
use common::*;
use emu_core::Core;

const FW: &str = "web/wasm/fw/public";

/// Stands in for the core's `cycle`/`cycleh` (0xC00/0xC80) aliases of `mcycle` until riscv-rv32
/// provides them: before every 64-instruction slice the CSRs the core stores but does not
/// implement are set to the cycle count. Delete with the shim when the core has them.
fn run_with_cycle_csr_shim(m: &mut esp32p4::Machine) -> esp_soc::Stop {
    loop {
        let c = &mut m.cores[0];
        let cycles = c.read_csr(0xb00) as u64 | (c.read_csr(0xb80) as u64) << 32;
        c.csr_other.insert(0xc00, cycles as u32);
        c.csr_other.insert(0xc80, (cycles >> 32) as u32);
        match m.run(64) { esp_soc::Stop::MaxInsns => continue, stop => return stop }
    }
}

/// With the cycle CSR in place the app reaches FreeRTOS: the heap regions and the clock are
/// the board's, and the run ends where the first interrupts would be taken. The systimer tick
/// (CLIC ID 18) and the scheduler's first yield (ID 17) are pending, and the core has no CLIC
/// mode to take them, so `vTaskStartScheduler` returns and `start_cpu0` spins on its final `j .`.
#[test]
#[ignore = "needs the ESP32-P4 mask ROM ELF"]
fn hello_world_p4_with_cycle_csr_shim() {
    let mut m = esp32p4::machine([0x30, 0xed, 0xa0, 0x00, 0x11, 0x22], 2 << 20);
    esp_soc::SocBus::set_flash_size(&mut m.bus, 2 << 20);
    m.load_rom(&std::fs::read(rom("esp32p4_rev0")).unwrap()).unwrap();
    for (off, file) in [(0x2000, "p4-hello-bootloader.bin"), (0x8000, "p4-hello-ptable.bin"), (0x10000, "p4-hello_world.bin")] {
        m.write_flash(off, &std::fs::read(root().join(FW).join(file)).unwrap()).unwrap();
    }
    m.boot_rom();
    m.console.capture = true;
    m.max_cycles = esp32p4::periph::CPU_HZ / 10;
    let stop = run_with_cycle_csr_shim(&mut m);
    assert!(matches!(stop, esp_soc::Stop::Halted), "{stop:?}");
    let console = String::from_utf8_lossy(&m.console.uart0).to_string();
    assert!(console.contains("cpu_start: cpu freq: 360000000 Hz"), "{console}");
    assert!(console.contains("heap_init: At 4FF40000 len 00060000 (384 KiB): RAM"), "{console}");
    // start_cpu0's `j .` after esp_startup_start_app returns (hello_world.elf of this build)
    assert_eq!(m.cores[0].pc(), 0x4000_0682);
    assert_eq!(m.bus.periph.clic.core[0].candidate().map(|(id, _)| id), Some(18));
    assert_eq!((m.exceptions, m.interrupts), (0, 0));
    expect_text("hello-p4-cycle-shim.console.txt", &console);
    expect_u64("hello-p4-cycle-shim.insns", m.insns());
}

/// `I (1234) tag: text` -> `I () tag: text`: log timestamps come from the cycle counter, which
/// the model does not keep the board's way.
fn strip_timestamp(line: &str) -> String {
    let t = line.trim_end_matches('\r');
    let b = t.as_bytes();
    if b.len() > 3 && b"EWIDV".contains(&b[0]) && b[1] == b' ' && b[2] == b'(' {
        if let Some(end) = t[3..].find(')') { if t[3..3 + end].bytes().all(|c| c.is_ascii_digit()) { return format!("{} (){}", &t[..1], &t[3 + end + 1..]); } }
    }
    t.to_string()
}

/// The owner's oxidrive firmware (not in this repository), booted from the same bootloader,
/// partition table and app as the board whose console is `board-boot.log`, and compared line
/// for line with timestamps stripped. The first two lines differ by construction (the ECO0 ROM
/// stands in for the board's ECO2 one). Not expected to pass yet: it reports how far the console
/// matches, the first line that does not, and why the run stopped there.
#[test]
#[ignore = "needs the oxidrive images under ~/Code/dlp/sim-fixtures"]
fn external_oxidrive_console_matches_the_board() {
    let dir = std::env::var("OXIDRIVE_DIR").unwrap_or_else(|_| format!("{}/Code/dlp/sim-fixtures/esp32p4/oxidrive-46a5472", std::env::var("HOME").unwrap_or_default()));
    let file = |name: &str| std::fs::read(format!("{dir}/{name}")).unwrap_or_else(|e| panic!("{dir}/{name}: {e} (set OXIDRIVE_DIR to the directory holding bootloader.bin, partition-table.bin, app.bin and board-boot.log)"));
    // espflash wrote the flash mode it was given (QIO, as the board's `SPI mode:QIO` line shows)
    // into the bootloader's header and re-digested the image.
    let mut bootloader = file("bootloader.bin");
    bootloader[2] = 0;
    if bootloader[23] == 1 { let n = bootloader.len() - 32; let digest = sha256(&bootloader[..n]); bootloader[n..].copy_from_slice(&digest); }
    // the board's efuse MAC, which its console prints as the MQTT topic suffix
    let mut m = esp32p4::machine([0x30, 0xed, 0xa0, 0x00, 0x11, 0x22], 32 << 20);
    esp_soc::SocBus::set_flash_size(&mut m.bus, 32 << 20);
    esp_soc::SocBus::set_psram_size(&mut m.bus, 32 << 20).unwrap();
    m.load_rom(&std::fs::read(rom("esp32p4_rev0")).unwrap()).unwrap();
    for (off, data) in [(0x2000, bootloader), (0x8000, file("partition-table.bin")), (0x30000, file("app.bin"))] { m.write_flash(off, &data).unwrap(); }
    m.boot_rom();
    m.console.capture = true;
    m.max_cycles = esp32p4::periph::CPU_HZ;
    let stop = run_with_cycle_csr_shim(&mut m);
    let board = String::from_utf8_lossy(&file("board-boot.log")).to_string();
    let board: Vec<String> = board.lines().skip_while(|l| !l.starts_with("ESP-ROM:")).map(strip_timestamp).collect();
    let emu: Vec<String> = String::from_utf8_lossy(&m.console.uart0).lines().map(strip_timestamp).collect();
    assert!(board.len() > 2 && emu.len() > 2, "no console to compare: board {} lines, emulator {}", board.len(), emu.len());
    eprintln!("[oxidrive] banner (ECO0 ROM, not the board's ECO2): {:?} / {:?} vs the board's {:?} / {:?}", emu[0], emu[1], board[0], board[1]);
    let same = emu.iter().zip(&board).skip(2).take_while(|(e, b)| e == b).count() + 2;
    let why = if m.bus.periph.hp_clkrst.core1_released() { "core 1 was released from reset and is held: the core model's mhartid reads 0 on every hart" } else { "see the run's stderr" };
    eprintln!("[oxidrive] {same} of {} board lines match; the run stopped ({stop:?}) at pc {:#010x} after {} instructions; {why}", board.len(), m.cores[0].pc(), m.insns());
    if let Some(e) = emu.get(same) { panic!("line {}: the emulator printed {e:?} where the board printed {:?}", same + 1, board.get(same)); }
    assert_eq!(same, board.len(), "the console matches the board's for {same} lines, then stops; the board's next line is {:?} ({why})", board.get(same));
}
