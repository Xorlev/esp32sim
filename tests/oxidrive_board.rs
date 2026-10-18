//! The owner's oxidrive firmware (not in this repository) booted as on its board: shared by
//! esp32p4/tests/oxidrive.rs and cli/tests/cosim.rs, included with `#[path]` next to `common`
//! and `sd_image`.
#![allow(dead_code)]
use crate::common::*;
use crate::sd_image;

/// `I (1234) tag: text` -> `I () tag: text`: log timestamps come from the cycle counter, which
/// the model does not keep the board's way.
pub fn strip_timestamp(line: &str) -> String {
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
pub fn board_mac(log: &str) -> [u8; 6] {
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

/// The board card's size and serial number from the firmware's probe line in the capture
/// (`sd: <size> MiB, serial <hex>, ...`), so the image presents as that card; neither value is
/// in this repository. Without the line: 128 GB and the model's own serial.
pub fn board_card(log: &str) -> (u64, Option<u32>) {
    let probe = log.lines().find_map(|l| l.split_once(" sd: ").map(|(_, r)| r.to_string()));
    let mib = probe.as_deref().and_then(|r| r.split_once(" MiB")).and_then(|(n, _)| n.trim().parse::<u64>().ok());
    let serial = probe.as_deref().and_then(|r| r.split_once("serial ")).and_then(|(_, t)| u32::from_str_radix(t.get(..8)?, 16).ok());
    (mib.map_or(128_000_000_000, |n| n << 20), serial)
}

/// Lines that differ because the ECO0 ROM stands in for the board's ECO2 one: the banner, and
/// the heap region below the ROM's own data, which IDF sizes from the ROM's `ets_rom_layout`
/// (`_dram0_rtos_reserved_start` is 0x4FF3FBA4 in the ECO0 ELF; the board's ROM ends it 12 bytes
/// later).
pub fn known_rom_difference(line: usize, emu: &str, board: &str) -> bool {
    line < 2 || (emu.starts_with("I () heap_init: At 4FF3AFC0 len ") && board.starts_with("I () heap_init: At 4FF3AFC0 len "))
}

/// The images, the board's capture and a machine booted from them with a card in the socket: a
/// sparse image of the board card's size and serial number (`board_card`), an SDXC card like the
/// board's, laid out as a card formatter lays one out (MBR, FAT32 from LBA 2048, 32 KiB
/// clusters); its FAT geometry and contents are the image's, not the board card's.
/// `write_through` writes what the firmware writes to the image file.
pub fn boot(card_name: &str, write_through: bool) -> (esp32p4::Machine, String, std::path::PathBuf) { boot_card(card_name, write_through, None) }

/// `boot` with a card of `size` bytes instead of the board card's size.
pub fn boot_card(card_name: &str, write_through: bool, size: Option<u64>) -> (esp32p4::Machine, String, std::path::PathBuf) {
    let dir = dir();
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
    let card = tmp(card_name);
    let (board_size, serial) = board_card(&board_log);
    sd_image::fat32(&card, size.unwrap_or(board_size), 32 << 10, "ESP32SIM").unwrap();
    let timed_bus = std::env::var("OXIDRIVE_SD_TIMING").is_ok_and(|v| v == "1");
    let config = esp_periph::sdmmc::SdCardConfig { serial, timed_bus, ..Default::default() };
    let path = card.to_str().unwrap();
    let store = if write_through { esp_periph::sdmmc::FileStore::write_through(path) } else { esp_periph::sdmmc::FileStore::copy_on_write(path) };
    let what = esp_soc::SocBus::attach_sd_card(&mut m.bus, Box::new(store.unwrap()), config).unwrap();
    eprintln!("[oxidrive] SD card: {what}");
    m.boot_rom();
    m.console.capture = true;
    (m, board_log, card)
}

/// The board's console from the ROM's banner on, timestamps stripped.
pub fn board_lines(board_log: &str) -> Vec<String> { board_log.lines().skip_while(|l| !l.starts_with("ESP-ROM:")).map(strip_timestamp).collect() }

/// How far the emulator's console matches the board's: the lines that match, every block that
/// does not (emulator start, its length, board start, its length), and the board line the
/// comparison reached.
pub struct Comparison { pub same: usize, pub blocks: Vec<(usize, usize, usize, usize)>, pub reached: usize }

/// Line by line; after lines that differ, the comparison picks up at the nearest pair of equal
/// lines (at most 48 lines on), so every divergent block is reported, not only the first. The
/// blocks are printed as they are found.
pub fn compare(emu: &[String], board: &[String]) -> Comparison {
    assert!(board.len() > 2 && emu.len() > 2, "no console to compare: board {} lines, emulator {}", board.len(), emu.len());
    let (mut e, mut b, mut same, mut blocks) = (0, 0, 0, Vec::new());
    while e < emu.len() && b < board.len() {
        if emu[e] == board[b] || known_rom_difference(b, &emu[e], &board[b]) {
            if emu[e] != board[b] { eprintln!("[oxidrive] line {}, the ROM's: the emulator printed {:?}, the board {:?}", b + 1, emu[e], board[b]); }
            (same, e, b) = (same + 1, e + 1, b + 1);
            continue;
        }
        let resync = (1..48).flat_map(|d| (0..=d).map(move |i| (i, d - i))).find(|&(i, j)| emu.get(e + i).is_some_and(|x| board.get(b + j) == Some(x)));
        let (i, j) = resync.unwrap_or((emu.len() - e, board.len() - b));
        blocks.push((e, i, b, j));
        (e, b) = (e + i, b + j);
    }
    for &(e, i, b, j) in &blocks {
        eprintln!("[oxidrive] board lines {}-{} differ:", b + 1, b + j);
        for l in &emu[e..e + i] { eprintln!("[oxidrive]   emulator: {l}"); }
        for l in &board[b..b + j] { eprintln!("[oxidrive]   board:    {l}"); }
    }
    Comparison { same, blocks, reached: b }
}

/// The firmware's directory: `OXIDRIVE_DIR`, else the fixtures' default.
pub fn dir() -> String {
    std::env::var("OXIDRIVE_DIR").unwrap_or_else(|_| format!("{}/Code/dlp/sim-fixtures/esp32p4/oxidrive-46a5472", std::env::var("HOME").unwrap_or_default()))
}

/// Offset and size of the first NVS data partition (type 0x01, subtype 0x02) in an ESP-IDF
/// partition table: 32-byte entries starting with the magic AA 50 (components/partition_table/
/// gen_esp32part.py, `PartitionDefinition.to_binary`).
pub fn nvs_partition(table: &[u8]) -> Option<(usize, usize)> {
    table.as_chunks::<32>().0.iter().take_while(|e| e[..2] == [0xaa, 0x50]).find(|e| e[2] == 0x01 && e[3] == 0x02).map(|e| {
        let word = |i: usize| u32::from_le_bytes(e[i..i + 4].try_into().unwrap()) as usize;
        (word(4), word(8))
    })
}

/// An NVS partition image of `size` bytes from the generator CSV `csv` (`key,type,encoding,value`
/// rows, the namespace row first), made at test time with ESP-IDF's
/// components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py (`IDF_PATH`, else
/// ~/.espressif/esp-idf/v5.5.3; its Python: `IDF_PYTHON`, else ESP-IDF's virtualenv).
pub fn nvs_image(csv: &std::path::Path, size: usize) -> Vec<u8> {
    let home = std::env::var("HOME").unwrap_or_default();
    let idf = std::env::var("IDF_PATH").unwrap_or_else(|_| format!("{home}/.espressif/esp-idf/v5.5.3"));
    let gen = format!("{idf}/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py");
    let python = std::env::var("IDF_PYTHON").ok().or_else(|| {
        let mut envs: Vec<_> = std::fs::read_dir(format!("{home}/.espressif/python_env")).ok()?.flatten().map(|e| e.path().join("bin/python")).filter(|p| p.exists()).collect();
        envs.sort();
        envs.pop().map(|p| p.display().to_string())
    }).unwrap_or_else(|| panic!("no ESP-IDF python env: set IDF_PYTHON"));
    let bin = tmp("nvs.bin");
    let out = std::process::Command::new(&python).args([gen.as_str(), "generate", csv.to_str().unwrap(), bin.to_str().unwrap(), &format!("{size:#x}")]).output()
        .unwrap_or_else(|e| panic!("{python} {gen}: {e}"));
    assert!(out.status.success(), "nvs_partition_gen.py failed: {}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    std::fs::read(&bin).unwrap()
}
