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

/// The images, the board's capture and a machine booted from them with a card in the socket: a
/// sparse image of the board card's size and serial number (`board_card`), an SDXC card like the
/// board's, laid out as a card formatter lays one out (MBR, FAT32 from LBA 2048, 32 KiB
/// clusters); its FAT geometry and contents are the image's, not the board card's.
/// `write_through` writes what the firmware writes to the image file.
fn boot(card_name: &str, write_through: bool) -> (esp32p4::Machine, String, std::path::PathBuf) {
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
    let card = tmp(card_name);
    let (size, serial) = board_card(&board_log);
    sd_image::fat32(&card, size, 32 << 10, "ESP32SIM").unwrap();
    let config = esp_periph::sdmmc::SdCardConfig { serial, ..Default::default() };
    let path = card.to_str().unwrap();
    let store = if write_through { esp_periph::sdmmc::FileStore::write_through(path) } else { esp_periph::sdmmc::FileStore::copy_on_write(path) };
    let what = esp_soc::SocBus::attach_sd_card(&mut m.bus, Box::new(store.unwrap()), config).unwrap();
    eprintln!("[oxidrive] SD card: {what}");
    m.boot_rom();
    m.console.capture = true;
    (m, board_log, card)
}

/// The owner's oxidrive firmware (not in this repository), booted from the same bootloader,
/// partition table and app as the board whose console is `board-boot.log`, with a FAT32 card in
/// the SD socket (`boot`), and compared line for line with timestamps stripped and the ROM's
/// known differences set aside. Not expected to pass yet: it reports how far the console
/// matches, every block that does not, and where the run stopped.
#[test]
#[ignore = "needs the oxidrive images under ~/Code/dlp/sim-fixtures"]
fn external_oxidrive_console_matches_the_board() {
    let (mut m, board_log, _card) = boot("oxidrive-card.img", false);
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
    // the blocks the firmware touched that are register RAM here
    let mut unmodelled: Vec<u32> = m.bus.periph.misc.generic.keys().copied().collect();
    unmodelled.sort_unstable();
    let names: Vec<String> = unmodelled.iter().map(|&b| format!("{} ({:#010x})", esp32p4::periph::Peripherals::block_name(b), b << 12)).collect();
    let why = format!("unmodelled blocks it touched: {}", names.join(", "));
    eprintln!("[oxidrive] {}", esp_soc::SocBus::report(&m.bus));
    eprintln!("[oxidrive] {same} of {} board lines match, {} blocks differ; the board's lines from {} were not reached; the run stopped ({stop:?}) at pc {:#010x} after {} instructions; {why}",
              board.len(), blocks.len(), b + 1, m.cores[0].pc(), m.insns());
    if let Some(&(e, _, b, _)) = blocks.first() { panic!("line {}: the emulator printed {:?} where the board printed {:?}", b + 1, emu.get(e), board.get(b)); }
    assert_eq!(same, board.len(), "the console matches the board's for {same} lines, then stops; the board's next line is {:?} ({why})", board.get(same));
}

/// oxidrive with a car on its USB port, as a car is when the board powers up with it: VBUS from
/// the start. The car enumerates the drive, polls it until it is ready, mounts its FAT32 volume,
/// records a 4 MiB clip in 64 KiB chunks 16 ms apart (a camera at 4 MB/s), reads it back, and
/// idles polling the drive while oxidrive runs on. Afterwards the clip must be on the card image
/// (written through) byte for byte. The firmware's console from the moment the car connected is
/// printed: how it accounts for the car's writes. Archive windows, which take the card from the
/// car, need WiFi and the NAS (milestone 4).
#[test]
#[ignore = "needs the oxidrive images under ~/Code/dlp/sim-fixtures"]
fn external_oxidrive_presents_the_card_to_a_car() {
    use esp_soc::usb_host::{fat32::{clip_bytes, BlockDevice, Fat32}, msc::Msc, UsbHost};
    let (mut m, _board_log, card) = boot("oxidrive-car.img", true);
    m.max_cycles = 20 * esp32p4::periph::CPU_HZ;
    esp_soc::SocBus::attach_usb_host(&mut m.bus, Default::default()).unwrap();
    let mut host = UsbHost::new(&mut m);
    host.log = true;
    let step = |what: &str, r: Result<(), String>| { if let Err(e) = r { panic!("[oxidrive] the car could not {what}: {e}"); } eprintln!("[oxidrive] car: {what}: ok"); };
    let r = host.enumerate(10_000_000_000).map_err(|e| e.to_string());
    let seen_at = host.now_ns();
    step("enumerate the drive", r);
    let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("[oxidrive] no mass-storage interface: {e}"));
    let i = msc.inquiry().map_err(|e| e.to_string());
    step("INQUIRY", i.map(|i| eprintln!("[oxidrive] car: INQUIRY {:?} {:?} {:?}", i.vendor, i.product, i.revision)));
    let r = msc.wait_ready(100_000_000, 10_000_000_000).map_err(|e| e.to_string());
    step("see the medium ready", r.map(|n| eprintln!("[oxidrive] car: ready after {n} failed TEST UNIT READY polls")));
    let r = msc.read_capacity().map_err(|e| e.to_string());
    step("read the capacity", r.map(|(n, bs)| eprintln!("[oxidrive] car: {n} blocks of {bs} bytes")));
    let mut fs = Fat32::mount(&mut msc).unwrap_or_else(|e| panic!("[oxidrive] the car could not mount the volume: {e}"));
    eprintln!("[oxidrive] car: FAT32 {:?}, {} clusters of {} bytes", fs.label, fs.clusters, fs.cluster_bytes());
    let clip = clip_bytes("CLIP0001.MP4", 4 << 20);
    let t0 = msc.host.now_ns();
    let r = (|| {
        let mut f = fs.create(&mut msc, "CLIP0001.MP4")?;
        for (n, part) in clip.chunks(64 << 10).enumerate() {
            if n > 0 { msc.pause_ns(16_000_000)?; }
            f.write(&mut fs, &mut msc, part)?;
        }
        f.close(&mut fs, &mut msc).map(|_| ())
    })();
    let dt = msc.host.now_ns() - t0;
    step("record a 4 MiB clip", r);
    eprintln!("[oxidrive] car: the clip took {:.3} s, {:.2} MB/s", dt as f64 / 1e9, clip.len() as f64 * 1e3 / dt as f64);
    let back = fs.read_file(&mut msc, "CLIP0001.MP4");
    assert!(back.as_ref().is_ok_and(|b| *b == clip), "[oxidrive] the clip read back over USB differs: {:?}", back.map(|b| b.len()));
    let r = (|| { let end = msc.host.now_ns() + 2_000_000_000; while msc.host.now_ns() < end { msc.test_unit_ready().map_err(|e| e.to_string())?; msc.host.idle_ns(100_000_000).map_err(|e| e.to_string())?; } Ok(()) })();
    step("poll the idle drive for 2 s", r);
    let at = msc.host.now_ns();
    drop(msc);
    let console = String::from_utf8_lossy(&m.console.uart0).to_string();
    eprintln!("[oxidrive] {}", esp_soc::SocBus::report(&m.bus));
    eprintln!("[oxidrive] the firmware's console from the car's connect ({:.3} s) to {:.3} s:", seen_at as f64 / 1e9, at as f64 / 1e9);
    let tail: Vec<&str> = console.lines().skip_while(|l| !l.contains("usb")).collect();
    for l in tail.iter().take(60) { eprintln!("[oxidrive]   {}", strip_timestamp(l)); }
    let files = sd_image::Volume::open(&card).unwrap().root_files();
    let on_card = files.iter().find(|f| f.0 == "CLIP0001.MP4");
    assert!(on_card.is_some_and(|f| f.1 == clip), "the clip is not on the card image intact: {:?}", files.iter().map(|f| (&f.0, f.1.len())).collect::<Vec<_>>());
    eprintln!("[oxidrive] the clip is on the card image intact ({} bytes)", clip.len());
}
