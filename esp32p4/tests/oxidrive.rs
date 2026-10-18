//! ESP32-P4 runs that need inputs from outside the repository.
#[path = "../../tests/common.rs"]
mod common;
use common::*;
use emu_core::Core;

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

/// The board's efuse MAC, which is not committed: `OXIDRIVE_MAC` (aa:bb:cc:dd:ee:ff), else
/// Espressif's OUI 30:ed:a0 and the three bytes the firmware's console prints as its MQTT topic
/// (`oxidrive/<hex>/cmd`), else the synthetic default.
fn board_mac(log: &str) -> [u8; 6] {
    let hex = |s: &str| u8::from_str_radix(s, 16).ok();
    if let Ok(v) = std::env::var("OXIDRIVE_MAC") {
        let b: Vec<u8> = v.split(':').filter_map(hex).collect();
        if let Ok(mac) = b.try_into() { return mac; }
        panic!("OXIDRIVE_MAC={v}: expected aa:bb:cc:dd:ee:ff");
    }
    let suffix = log.split("oxidrive/").skip(1).filter_map(|t| t.get(..6).filter(|_| t[6..].starts_with("/cmd")))
        .find_map(|h| Some([hex(&h[..2])?, hex(&h[2..4])?, hex(&h[4..])?]));
    match suffix { Some([d, e, f]) => [0x30, 0xed, 0xa0, d, e, f], None => [0x30, 0xed, 0xa0, 0x00, 0x11, 0x22] }
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
    let board_log = String::from_utf8_lossy(&file("board-boot.log")).to_string();
    let mut m = esp32p4::machine(board_mac(&board_log), 32 << 20);
    esp_soc::SocBus::set_flash_size(&mut m.bus, 32 << 20);
    esp_soc::SocBus::set_psram_size(&mut m.bus, 32 << 20).unwrap();
    m.load_rom(&std::fs::read(rom("esp32p4_rev0")).unwrap()).unwrap();
    for (off, data) in [(0x2000, bootloader), (0x8000, file("partition-table.bin")), (0x30000, file("app.bin"))] { m.write_flash(off, &data).unwrap(); }
    m.boot_rom();
    m.console.capture = true;
    m.max_cycles = esp32p4::periph::CPU_HZ;
    let stop = m.run(u64::MAX);
    let board: Vec<String> = board_log.lines().skip_while(|l| !l.starts_with("ESP-ROM:")).map(strip_timestamp).collect();
    let emu: Vec<String> = String::from_utf8_lossy(&m.console.uart0).lines().map(strip_timestamp).collect();
    assert!(board.len() > 2 && emu.len() > 2, "no console to compare: board {} lines, emulator {}", board.len(), emu.len());
    eprintln!("[oxidrive] banner (ECO0 ROM, not the board's ECO2): {:?} / {:?} vs the board's {:?} / {:?}", emu[0], emu[1], board[0], board[1]);
    let same = emu.iter().zip(&board).skip(2).take_while(|(e, b)| e == b).count() + 2;
    let why = if m.bus.periph.hp_clkrst.core1_released() { "core 1 was released from reset and is held: the core model's mhartid reads 0 on every hart" } else { "see the run's stderr" };
    eprintln!("[oxidrive] {same} of {} board lines match; the run stopped ({stop:?}) at pc {:#010x} after {} instructions; {why}", board.len(), m.cores[0].pc(), m.insns());
    if let Some(e) = emu.get(same) { panic!("line {}: the emulator printed {e:?} where the board printed {:?}", same + 1, board.get(same)); }
    assert_eq!(same, board.len(), "the console matches the board's for {same} lines, then stops; the board's next line is {:?} ({why})", board.get(same));
}
