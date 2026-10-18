//! ESP32-P4 runs that need inputs from outside the repository.
#[path = "../../tests/common.rs"]
mod common;
#[path = "../../tests/sd_image.rs"]
mod sd_image;
#[path = "../../tests/oxidrive_board.rs"]
mod oxidrive_board;
use oxidrive_board::*;
use emu_core::Core;

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
    let board = board_lines(&board_log);
    let emu: Vec<String> = String::from_utf8_lossy(&m.console.uart0).lines().map(strip_timestamp).collect();
    let Comparison { same, blocks, reached: b } = compare(&emu, &board);
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
/// records a clip in 64 KiB chunks, one due every 16 ms (a 4.1 MB/s camera, `fat32::write_clip`),
/// reads it back, and idles polling the drive while oxidrive runs on. Afterwards the clip must be
/// on the card image (written through) byte for byte. The firmware's console from the moment the
/// car connected is printed: how it accounts for the car's writes. Archive windows, which take the
/// card from the car, need WiFi and the NAS (milestone 4). `OXIDRIVE_CLIP_MIB` sets the clip's
/// size (4), `OXIDRIVE_CAR_GAP_MS` the camera's interval (16; 0 writes as fast as the drive
/// takes it), `OXIDRIVE_SD_TIMING=1` times the SD bus (`--sd-timing`).
#[test]
#[ignore = "needs the oxidrive images under ~/Code/dlp/sim-fixtures"]
fn external_oxidrive_presents_the_card_to_a_car() {
    use esp_soc::usb_host::{fat32::{clip_bytes, write_clip, Fat32}, msc::Msc, UsbHost};
    let (mut m, _board_log, card) = boot("oxidrive-car.img", true);
    // 64 KiB every 16 ms is a 4.1 MB/s camera; OXIDRIVE_CLIP_MIB=64 outlasts the drive's buffering
    let env = |k: &str, d: u64| std::env::var(k).ok().and_then(|v| v.parse::<u64>().ok()).unwrap_or(d);
    let mib = env("OXIDRIVE_CLIP_MIB", 4);
    let gap_ns = env("OXIDRIVE_CAR_GAP_MS", 16) * 1_000_000;
    m.max_cycles = (20 + mib / 2) * esp32p4::periph::CPU_HZ;
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
    let clip = clip_bytes("CLIP0001.MP4", (mib << 20) as usize);
    let r = write_clip(&mut fs, &mut msc, "CLIP0001.MP4", &clip, 64 << 10, gap_ns, 1);
    let st = r.map(|s| s.summary(gap_ns));
    step(&format!("record a {mib} MiB clip"), st.map(|s| eprintln!("[oxidrive] car: {s}")));
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
