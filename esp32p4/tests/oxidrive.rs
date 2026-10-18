//! ESP32-P4 runs that need inputs from outside the repository.
#[path = "../../tests/common.rs"]
mod common;
#[path = "../../tests/sd_image.rs"]
mod sd_image;
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

/// The board card's size and serial number from the firmware's probe line in the capture
/// (`sd: <size> MiB, serial <hex>, ...`), so the image presents as that card; neither value is
/// in this repository. Without the line: 128 GB and the model's own serial.
fn board_card(log: &str) -> (u64, Option<u32>) {
    let probe = log.lines().find_map(|l| l.split_once(" sd: ").map(|(_, r)| r.to_string()));
    let mib = probe.as_deref().and_then(|r| r.split_once(" MiB")).and_then(|(n, _)| n.trim().parse::<u64>().ok());
    let serial = probe.as_deref().and_then(|r| r.split_once("serial ")).and_then(|(_, t)| u32::from_str_radix(t.get(..8)?, 16).ok());
    (mib.map_or(128_000_000_000, |n| n << 20), serial)
}

/// Lines that differ because the ECO0 ROM stands in for the board's ECO2 one: the banner, and
/// the heap region below the ROM's own data, which IDF sizes from the ROM's `ets_rom_layout`
/// (`_dram0_rtos_reserved_start` is 0x4FF3FBA4 in the ECO0 ELF; the board's ROM ends it 12 bytes
/// later).
fn known_rom_difference(line: usize, emu: &str, board: &str) -> bool {
    line < 2 || (emu.starts_with("I () heap_init: At 4FF3AFC0 len ") && board.starts_with("I () heap_init: At 4FF3AFC0 len "))
}

/// The owner's oxidrive firmware (not in this repository), booted from the same bootloader,
/// partition table and app as the board whose console is `board-boot.log`, with a FAT32 card in
/// the SD socket, and compared line for line with timestamps stripped and the ROM's known
/// differences set aside. The card is a sparse image of the board card's size and serial
/// number (`board_card`), an SDXC card like the board's, laid out as a card formatter lays one
/// out (MBR, FAT32 from LBA 2048, 32 KiB clusters); its FAT geometry and contents are the
/// image's, not the board card's. Not expected to pass yet: it reports how far the console
/// matches, every block that does not, and where the run stopped.
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
    let card = tmp("oxidrive-card.img");
    let (size, serial) = board_card(&board_log);
    sd_image::fat32(&card, size, 32 << 10, "ESP32SIM").unwrap();
    let config = esp_periph::sdmmc::SdCardConfig { serial, ..Default::default() };
    let what = esp_soc::SocBus::attach_sd_card(&mut m.bus, Box::new(esp_periph::sdmmc::FileStore::copy_on_write(card.to_str().unwrap()).unwrap()), config).unwrap();
    eprintln!("[oxidrive] SD card: {what}");
    m.boot_rom();
    m.console.capture = true;
    m.max_cycles = 3 * esp32p4::periph::CPU_HZ;
    let stop = m.run(u64::MAX);
    let board: Vec<String> = board_log.lines().skip_while(|l| !l.starts_with("ESP-ROM:")).map(strip_timestamp).collect();
    let emu: Vec<String> = String::from_utf8_lossy(&m.console.uart0).lines().map(strip_timestamp).collect();
    assert!(board.len() > 2 && emu.len() > 2, "no console to compare: board {} lines, emulator {}", board.len(), emu.len());
    // Line by line; after lines that differ, the comparison picks up at the nearest pair of equal
    // lines (at most 48 lines on), so every divergent block is reported, not only the first.
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
    // USB-OTG HS (0x50000000-0x5003FFFF) is not modelled: its registers are register RAM.
    let usb = m.bus.periph.misc.generic.keys().filter(|&&b| (0x50000..=0x5003f).contains(&b)).min().copied();
    let why = match usb { Some(b) => format!("the firmware reached USB-OTG HS ({:#010x}), which is not modelled", b << 12), None => "see the run's stderr".to_string() };
    eprintln!("[oxidrive] {}", esp_soc::SocBus::report(&m.bus));
    eprintln!("[oxidrive] {same} of {} board lines match, {} blocks differ; the board's lines from {} were not reached; the run stopped ({stop:?}) at pc {:#010x} after {} instructions; {why}",
              board.len(), blocks.len(), b + 1, m.cores[0].pc(), m.insns());
    if let Some(&(e, _, b, _)) = blocks.first() { panic!("line {}: the emulator printed {:?} where the board printed {:?}", b + 1, emu.get(e), board.get(b)); }
    assert_eq!(same, board.len(), "the console matches the board's for {same} lines, then stops; the board's next line is {:?} ({why})", board.get(same));
}
