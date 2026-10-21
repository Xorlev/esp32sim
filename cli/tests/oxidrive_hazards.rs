//! Milestone 4c: what a car meets in oxidrive's archive windows and shared passes. The owner's
//! oxidrive firmware (not in this repository; `OXIDRIVE_CURRENT_DIR`, the 728b71f build by
//! default) runs on the rig of `oxidrive_services` (`oxidrive_rig`), and a car of
//! `esp_soc::usb_host::car` records on its USB port: writes that land in a window, a car that keeps
//! its FAT and directories between polls, windows shorter than its poll, Linux's writeback, and
//! the fsck burst of the first epoch after a mount. Each test asserts what the firmware does today,
//! named so that it fails once the firmware changes it, and prints what it measured.
//!
//! The SD bus is timed and the card programs for `OXIDRIVE_SD_PRG_US` (1500 us) after each write:
//! an assumption, not a value measured on a card. The emulator runs one instruction a cycle with no
//! cache or PSRAM stalls, and leaves the card's read access time and the IDMAC's pace untimed, so
//! every time here is a lower bound. smbd and mosquitto answer in host time: two runs agree on the
//! mechanisms, not on every microsecond.
#[path = "../../tests/common.rs"]
mod common;
#[path = "../../tests/sd_image.rs"]
mod sd_image;
#[path = "../../tests/oxidrive_board.rs"]
mod oxidrive_board;
#[path = "../../tests/services.rs"]
mod services;
#[path = "../../tests/oxidrive_rig.rs"]
mod oxidrive_rig;
use esp_soc::usb_host::car::{Car, CarCfg, CarFile, Recorded};
use esp_soc::usb_host::fat32::{mp4_bytes, Fat32};
use esp_soc::usb_host::msc::Msc;
use esp_soc::usb_host::pagecache::{Landing, PageCache};
use esp_soc::usb_host::UsbHost;
use oxidrive_rig::*;
use std::collections::{BTreeMap, HashMap};

const S: u64 = 1_000_000_000;
const MS: u64 = 1_000_000;

fn env_u64(name: &str, default: u64) -> u64 { std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default) }
fn secs(ns: u64) -> String { format!("{:.3} s", ns as f64 / 1e9) }

/// The rig on the current firmware, a car on its port, the card's writes logged.
fn hazard_rig(card: &str, image: Option<std::path::PathBuf>) -> Rig {
    let prg = env_u64("OXIDRIVE_SD_PRG_US", 1500);
    let timed = std::env::var("OXIDRIVE_SD_TIMING").map_or(true, |v| v != "0");
    eprintln!("[hazard] SD bus {}, card programming time {prg} us (assumed)", if timed { "timed" } else { "untimed" });
    let o = RigOpts { card_name: card.into(), card_bytes: env_u64("OXIDRIVE_CARD_MIB", 4096) << 20, car: true, image, prg_us: Some(prg), timed: Some(timed), log_writes: true, ..RigOpts::new(oxidrive_board::current_dir()) };
    rig_with(o)
}

/// Enumerate, open the mass-storage interface, wait for the drive, and mount the volume as `cfg`
/// says, through `disk`.
fn car_on<'p>(r: &'p mut Rig, disk: impl FnOnce(Msc<'p>) -> PageCache<Msc<'p>>, cfg: CarCfg) -> Car<Msc<'p>> { car_on_with(r, disk, cfg, None) }

/// `car_on`, with the car's fsck at a mount of a dirty volume.
fn car_on_with<'p>(r: &'p mut Rig, disk: impl FnOnce(Msc<'p>) -> PageCache<Msc<'p>>, cfg: CarCfg, fsck: Option<esp_soc::usb_host::fat32::FsckWrite>) -> Car<Msc<'p>> {
    let mut host = UsbHost::new(r);
    host.log = std::env::var("OXIDRIVE_USB_LOG").is_ok_and(|v| v == "1");
    host.enumerate(10 * S).unwrap_or_else(|e| panic!("enumerate: {e}"));
    let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
    msc.wait_ready(100 * MS, 10 * S).unwrap_or_else(|e| panic!("never ready: {e}"));
    let mut car = Car::new(disk(msc), cfg);
    car.echo = true;
    car.disk.record = true;
    car.fsck_at_mount = fsck;
    car.mount().unwrap_or_else(|e| panic!("mount: {e}"));
    car
}

/// Tesla's names: an event directory and a clip per camera, stamped to the minute.
fn stamp(minute: u32) -> String { format!("2026-01-01_{:02}-{:02}-00", minute / 60, minute % 60) }
fn clip_path(folder: &str, event: u32, minute: u32, camera: &str) -> String { format!("TeslaCam/{folder}/{}/{}-{camera}.mp4", stamp(event), stamp(minute)) }
/// An event's single combined clip, which oxidrive takes as a whole event by itself (its
/// camera.rs `EventContents`; a per-camera event waits for every camera of the car).
fn event_path(folder: &str, event: u32) -> String { format!("TeslaCam/{folder}/{}/event.mp4", stamp(event)) }

/// Idle until the console has a line holding `what`, polling as the car does; when it came.
fn idle_until_line(car: &mut Car<Msc<'_>>, shared: &std::rc::Rc<std::cell::RefCell<Shared>>, what: &str, step_ns: u64, deadline_ns: u64) -> Option<u64> {
    while car.now() < deadline_ns {
        if let Some(t) = shared.borrow().when(what) { return Some(t); }
        car.idle(step_ns).unwrap_or_else(|e| panic!("idle: {e}"));
    }
    None
}

/// The card's writes the car did not make: blocks the car's own landings do not account for
/// within a second.
fn device_writes(card: &[CardWrite], car: &[Landing]) -> Vec<CardWrite> {
    let mut by_lba: HashMap<u64, Vec<u64>> = HashMap::new();
    for l in car { for lba in l.lba..l.lba + l.count as u64 { by_lba.entry(lba).or_default().push(l.at_ns); } }
    let mut out: Vec<CardWrite> = Vec::new();
    for w in card {
        for lba in w.lba..w.lba + w.count {
            let ours = by_lba.get(&lba).is_some_and(|v| v.iter().any(|&t| t + S >= w.at_ns && t <= w.at_ns + S));
            if ours { continue; }
            match out.last_mut() {
                Some(o) if o.at_ns == w.at_ns && o.lba + o.count == lba => o.count += 1,
                _ => out.push(CardWrite { at_ns: w.at_ns, lba, count: 1 }),
            }
        }
    }
    out
}

/// The region of the volume a block is in.
fn region(fs: &Fat32, lba: u64) -> &'static str {
    let fat = fs.volume_lba + fs.reserved as u64;
    if lba < fs.volume_lba { "MBR" } else if lba == fs.volume_lba + fs.fsinfo as u64 { "FSInfo" } else if lba < fat { "reserved" } else if lba < fat + fs.fat_sectors as u64 { "FAT1" } else if lba < fs.data_lba { "FAT2" } else { "data" }
}

/// An upload against what the car believes the file holds.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Archived { Same, Prefix(usize), Differs { at: usize, len: usize }, Absent }

fn archived(share: &std::path::Path, files: &BTreeMap<String, CarFile>) -> Vec<(String, Archived)> {
    files.iter().map(|(p, f)| {
        let remote = share.join(p.strip_prefix("TeslaCam/").unwrap_or(p));
        let v = match std::fs::read(&remote) {
            Err(_) => Archived::Absent,
            Ok(b) if b == f.data => Archived::Same,
            Ok(b) if b.len() < f.data.len() && f.data.starts_with(&b) => Archived::Prefix(b.len()),
            Ok(b) => Archived::Differs { at: b.iter().zip(&f.data).position(|(x, y)| x != y).unwrap_or(b.len().min(f.data.len())), len: b.len() },
        };
        (p.clone(), v)
    }).collect()
}

/// Window intervals from the console: card taken, card returned.
fn windows(shared: &Shared) -> Vec<(u64, u64)> {
    let taken = shared.all("supervisor: card taken for archiving");
    let back = shared.all("supervisor: card returned to the car");
    taken.iter().map(|&t| (t, back.iter().copied().find(|&b| b >= t).unwrap_or(u64::MAX))).collect()
}

/// Which way each archived clip reached the share: a window or a shared pass, by when its
/// `smb-> finish` line came.
fn path_of_upload(shared: &Shared, remote: &str) -> &'static str {
    let Some(t) = shared.lines.iter().find(|l| l.1.contains("smb-> finish") && l.1.ends_with(remote)).map(|l| l.0) else { return "?" };
    if windows(shared).iter().any(|&(a, b)| (a..=b).contains(&t)) { "window" } else { "shared pass" }
}

/// The firmware's lines about windows, passes, the card and the archive.
fn console(sh: &Shared) {
    for (t, l) in &sh.lines {
        if ["supervisor", "window", "shared", "scan:", "archiver", "preSMB", "msc:", "usb:", "smb-> finish", "smb-> begin_part", "verify", "senses"].iter().any(|k| l.contains(k)) { eprintln!("[console {:>9}] {l}", secs(*t)); }
    }
}

/// fsck on the image, its verdict and output.
fn fsck_report(card: &std::path::Path) -> (bool, String) {
    match sd_image::fsck(card) { Ok(o) => (true, o), Err(e) => (false, e) }
}

/// Item 1. A car recording when a window opens keeps writing until its next poll: esp_tinyusb
/// answers TEST UNIT READY with MEDIUM NOT PRESENT once oxidrive holds the card, but its
/// WRITE(10) callback writes whatever the mount point (oxidrive's esp_tinyusb 2.2.1
/// components/esp_tinyusb/tinyusb_msc.c, `tud_msc_write10_cb` and
/// `msc_storage_write_sector_deferred`), so the car's writes between the take and its poll land
/// on the card under oxidrive's FatFs and are acknowledged GOOD. The car (2 s polls, a vfat-like
/// writer) records two saved events, idles a minute so a window opens to take the newer one, and
/// starts a Sentry event the moment it does. Asserted, as the firmware behaves today: the car's
/// writes after the take reach the card until its poll, the clip it was writing is left as
/// clusters no entry reaches, and oxidrive's quiet timer does not count the writes it took while
/// it held the card (its write tap records only while the car has the card,
/// app/src/storage/card.rs `__wrap_sdmmc_write_sectors`).
#[test]
#[ignore = "needs the oxidrive 728b71f images and nvs.csv under OXIDRIVE_CURRENT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and a FAT checker"]
fn external_oxidrive_takes_car_writes_into_a_window() {
    let mut r = hazard_rig("hazard-window-writes.img", None);
    let shared = r.shared.clone();
    let wall = std::time::Instant::now();
    let (files, landings, gone_at, sentry, stats, rec) = {
        let mut car = car_on(&mut r, PageCache::new, CarCfg::default());
        for e in 0..2 {
            let p = event_path("SavedClips", e);
            let st = car.record(&p, &mp4_bytes(&p, 2 << 20)).unwrap();
            assert!(matches!(st, Recorded::Closed { .. }), "{st:?}");
        }
        let taken = idle_until_line(&mut car, &shared, "supervisor: card taken for archiving", 10 * MS, 200 * S).expect("no window within 200 s");
        let sentry = event_path("SentryClips", 2);
        let start = car.now();
        eprintln!("[hazard] window taken at {}; the car starts a Sentry clip at {}, its next poll due at {}", secs(taken), secs(start), secs(car.next_poll_ns));
        let rec = car.record(&sentry, &mp4_bytes(&sentry, 16 << 20)).unwrap();
        let gone_at = car.log.iter().rev().find(|l| l.1.contains("medium gone")).map_or(u64::MAX, |l| l.0);
        eprintln!("[hazard] the Sentry clip: {rec:?}; the medium seen gone at {}", secs(gone_at));
        while !car.present && car.now() < 400 * S { car.idle(100 * MS).unwrap(); }
        assert!(car.present, "the medium never came back");
        car.idle(5 * S).unwrap();
        let after = event_path("SavedClips", 5);
        car.record(&after, &mp4_bytes(&after, 2 << 20)).unwrap();
        car.idle(4 * S).unwrap();
        (car.files.clone(), car.disk.landings.clone(), gone_at, sentry, car.stats, rec)
    };
    r.report(10);
    eprintln!("[hazard] {:.0} s wall for {} emulated; car: {stats:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    console(&shared.borrow());
    let sh = shared.borrow();
    let ws = windows(&sh);
    let (taken, returned) = *ws.first().expect("a window");
    eprintln!("[hazard] the window: {} to {} ({})", secs(taken), secs(returned), secs(returned.saturating_sub(taken)));
    let into: Vec<&Landing> = landings.iter().filter(|l| (taken..gone_at.min(returned)).contains(&l.at_ns)).collect();
    let blocks: u64 = into.iter().map(|l| l.count as u64).sum();
    eprintln!("[hazard] car WRITE(10)s answered GOOD while oxidrive held the card: {} commands, {} KiB, from {} to {}", into.len(), blocks / 2,
              secs(into.first().map_or(0, |l| l.at_ns)), secs(into.last().map_or(0, |l| l.at_ns)));
    let mut disk = FileDisk(std::fs::File::open(&r.card).unwrap());
    let fs = Fat32::mount(&mut disk).unwrap();
    let on_card: u64 = sh.writes.iter().filter(|w| (taken..gone_at.min(returned).saturating_add(S)).contains(&w.at_ns)).map(|w| w.count).sum();
    let ours = device_writes(&sh.writes, &landings);
    let in_window: Vec<&CardWrite> = ours.iter().filter(|w| (taken..=returned).contains(&w.at_ns)).collect();
    eprintln!("[hazard] the card took {on_card} blocks then; blocks oxidrive wrote itself in the window: {}", in_window.iter().map(|w| w.count).sum::<u64>());
    for w in &in_window { eprintln!("[hazard]   {} {} blocks at {} ({})", secs(w.at_ns), w.count, w.lba, region(&fs, w.lba)); }
    let e = fs.stat(&mut disk, &sentry).unwrap();
    eprintln!("[hazard] the cut Sentry clip on the card: {e:?}");
    let (clean, out) = fsck_report(&r.card);
    eprintln!("[hazard] fsck: {}", out.trim().replace('\n', "; "));
    for (p, v) in archived(&r.smb.path(), &files) { eprintln!("[hazard] {p}: {v:?} ({})", path_of_upload(&sh, p.strip_prefix("TeslaCam/").unwrap())); }
    let failed = sh.when("window: finished with failures").is_some_and(|t| (taken..=returned).contains(&t));
    let io_error = sh.lines.iter().any(|(t, l)| (taken..=returned).contains(t) && l.contains("Input/output error") || (taken..=returned).contains(t) && l.contains("I/O error"));
    eprintln!("[hazard] outcome: the car {} at its poll; the window {}{}", if gone_at < u64::MAX { "saw the medium go" } else { "never saw the medium go" },
              if failed { "finished with failures" } else { "finished clean" }, if io_error { ", after an I/O error reading the card" } else { "" });
    for (p, f) in files.iter().filter(|f| f.1.complete) {
        assert!(fs.read_path(&mut disk, p).unwrap() == f.data, "{p}: a closed clip differs on the card");
    }
    assert!(!into.is_empty() && on_card >= blocks, "the car's writes after the take reached the card: {blocks} blocks written, {on_card} taken");
    if gone_at < u64::MAX {
        // the car learned at its poll: its clip is cut, its clusters orphaned
        assert!(gone_at - taken <= 2 * S + 50 * MS, "the car learns at its next poll, within 2 s of the take");
        assert!(matches!(rec, Recorded::MediumGone { .. }));
        assert_eq!(e.map(|e| (e.size, e.cluster)), Some((0, 0)), "the cut clip keeps its empty entry");
        assert!(!clean, "fsck finds the cut clip's clusters, which no entry reaches");
    } else {
        // the window ended before the car's poll: the car never knew
        assert!(returned - taken < 2 * S && failed, "a window the car missed ended early, with failures");
        assert!(matches!(rec, Recorded::Closed { .. }) && clean);
    }
}

/// Item 2. A car that keeps its FAT and directories between polls, as Linux's buffer cache does,
/// and reads the volume again only after a medium change (NOT READY then READY; esp_tinyusb never
/// answers UNIT ATTENTION, tinyusb_msc.c `tud_msc_test_unit_ready_cb`). It records, oxidrive's own
/// policy opens a window after a quiet minute, `force_open` a second one; the car's 2 s poll sees
/// both, so it reads the volume again after each. Asserted, as oxidrive 728b71f behaves: it writes
/// nothing in a window on a card that has TeslaCam (reclaim off), so the volume passes fsck, every
/// closed clip is on the card as the car wrote it, and every upload equals the car's file.
#[test]
#[ignore = "needs the oxidrive 728b71f images and nvs.csv under OXIDRIVE_CURRENT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and a FAT checker"]
fn external_oxidrive_caching_car_through_windows_it_sees() {
    let mut r = hazard_rig("hazard-caching.img", None);
    let shared = r.shared.clone();
    let (cmd, mqtt) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    let wall = std::time::Instant::now();
    let (files, landings, stats, cache) = {
        let mut car = car_on(&mut r, PageCache::keeping_meta, CarCfg::default());
        let rec = |car: &mut Car<Msc<'_>>, folder: &str, event: u32, mib: usize| {
            let p = event_path(folder, event);
            let st = car.record(&p, &mp4_bytes(&p, mib << 20)).unwrap();
            assert!(matches!(st, Recorded::Closed { .. }), "{p}: {st:?}");
        };
        rec(&mut car, "SavedClips", 0, 2);
        rec(&mut car, "SentryClips", 1, 1);
        idle_until_line(&mut car, &shared, "supervisor: card taken for archiving", 100 * MS, 200 * S).expect("no window within 200 s");
        while car.stats.media_changes < 2 && car.now() < 400 * S { car.idle(100 * MS).unwrap(); }
        assert_eq!(car.stats.media_changes, 2, "the car saw the medium go and come back");
        rec(&mut car, "SavedClips", 3, 2);
        rec(&mut car, "SentryClips", 4, 1);
        car.idle(15 * S).unwrap();
        let before = shared.borrow().all("supervisor: card taken for archiving").len();
        while shared.borrow().all("supervisor: card taken for archiving").len() == before && car.now() < 600 * S {
            mqtt.publish(&cmd, "force_open").unwrap();
            car.idle(2 * S).unwrap();
        }
        while car.stats.media_changes < 4 && car.now() < 700 * S { car.idle(100 * MS).unwrap(); }
        rec(&mut car, "SavedClips", 9, 1);
        car.idle(5 * S).unwrap();
        (car.files.clone(), car.disk.landings.clone(), car.stats, car.disk.stats)
    };
    r.report(30);
    eprintln!("[hazard] {:.0} s wall for {} emulated; car: {stats:?}; its cache: {cache:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    let sh = shared.borrow();
    let ws = windows(&sh);
    for (a, b) in &ws { eprintln!("[hazard] window {} to {} ({})", secs(*a), secs(*b), secs(b - a)); }
    let mut disk = FileDisk(std::fs::File::open(&r.card).unwrap());
    let fs = Fat32::mount(&mut disk).unwrap();
    let ours: Vec<CardWrite> = device_writes(&sh.writes, &landings).into_iter().filter(|w| ws.iter().any(|&(a, b)| (a..=b).contains(&w.at_ns))).collect();
    for w in &ours { eprintln!("[hazard] oxidrive wrote {} blocks at {} ({}) at {}", w.count, w.lba, region(&fs, w.lba), secs(w.at_ns)); }
    let (clean, out) = fsck_report(&r.card);
    eprintln!("[hazard] fsck: {}", out.trim().replace('\n', "; "));
    let ups = archived(&r.smb.path(), &files);
    for (p, v) in &ups { eprintln!("[hazard] {p}: {v:?} ({})", path_of_upload(&sh, p.strip_prefix("TeslaCam/").unwrap())); }
    assert_eq!(stats.media_changes, 4, "both windows seen");
    assert!(ws.len() >= 2, "two windows");
    assert!(ours.is_empty(), "oxidrive wrote the card in a window");
    for (p, f) in &files { assert!(fs.read_path(&mut disk, p).unwrap() == f.data, "{p} differs on the card"); }
    assert!(ups.iter().all(|(_, v)| matches!(v, Archived::Same | Archived::Absent)), "an upload differs from the car's file");
    assert!(ups.iter().any(|(_, v)| *v == Archived::Same), "nothing archived");
    assert!(clean, "fsck");
}

/// What one run of `short_windows` saw.
struct ShortWindow { poll_ns: u64, window: (u64, u64), missed: bool, oxidrive_wrote: Vec<CardWrite>, teslacam: Vec<String>, fsck: (bool, String) }

/// A fresh card (FAT32, nothing on it), a car that keeps its FAT and directories and has looked
/// for TeslaCam before the window, polling every `poll_ns`. oxidrive's first window finds no
/// TeslaCam on a card this empty and makes it (TeslaCam self-heal: pipeline `ensure_layout`, the
/// four directories through FatFs), then has nothing to archive and gives the card back. Afterwards
/// the car makes TeslaCam/RecentClips and records a clip.
fn short_window_run(name: &str, poll_ns: u64) -> ShortWindow {
    let mut r = hazard_rig(name, None);
    let shared = r.shared.clone();
    let (landings, stats) = {
        let mut car = car_on(&mut r, PageCache::keeping_meta, CarCfg { poll_ns, ..CarCfg::default() });
        let fs = car.fs.clone().unwrap();
        assert!(fs.stat(&mut car.disk, "TeslaCam").unwrap().is_none(), "a fresh card");
        idle_until_line(&mut car, &shared, "supervisor: card returned to the car", 100 * MS, 200 * S).expect("no window within 200 s");
        car.idle(poll_ns + 100 * MS).unwrap();
        let p = clip_path("RecentClips", 0, 0, "front").replace("/2026-01-01_00-00-00/", "/");
        let st = car.record(&p, &mp4_bytes(&p, 1 << 20)).unwrap();
        assert!(matches!(st, Recorded::Closed { .. }), "{st:?}");
        car.idle(3 * S).unwrap();
        (car.disk.landings.clone(), car.stats)
    };
    let sh = shared.borrow();
    let window = *windows(&sh).first().expect("a window");
    let mut disk = FileDisk(std::fs::File::open(&r.card).unwrap());
    let fs = Fat32::mount(&mut disk).unwrap();
    let oxidrive_wrote: Vec<CardWrite> = device_writes(&sh.writes, &landings).into_iter().filter(|w| (window.0..=window.1).contains(&w.at_ns)).collect();
    let teslacam = fs.list(&mut disk, "TeslaCam").map(|v| v.into_iter().map(|e| e.name).collect()).unwrap_or_default();
    let fsck = fsck_report(&r.card);
    eprintln!("[hazard] poll every {}: the window {} to {} ({}); the car saw {} medium changes; {:?}", secs(poll_ns), secs(window.0), secs(window.1), secs(window.1 - window.0), stats.media_changes, stats);
    for w in &oxidrive_wrote { eprintln!("[hazard]   oxidrive wrote {} blocks at {} ({}) at {}", w.count, w.lba, region(&fs, w.lba), secs(w.at_ns)); }
    eprintln!("[hazard]   TeslaCam afterwards: {teslacam:?}; root: {:?}", fs.list_root(&mut disk).unwrap().iter().map(|e| (e.name.clone(), e.cluster)).collect::<Vec<_>>());
    eprintln!("[hazard]   fsck: {}", fsck.1.trim().replace('\n', "; "));
    ShortWindow { poll_ns, window, missed: stats.media_changes == 0, oxidrive_wrote, teslacam, fsck }
}

/// Item 3. Windows shorter than the poll: a window on a fresh card takes the card and gives it
/// back in about 1.4 s, so a car polling every 5 s, or every 2 s as udev polls a removable disk
/// (systemd v256 rules.d/60-block.rules), sees it only if a poll falls inside it. A car that
/// misses it still holds the root and the FAT it read before the window, with no TeslaCam and
/// the clusters oxidrive took marked free. Asserted, as oxidrive 728b71f behaves: the window
/// writes the card (TeslaCam and its three folders, their FAT entries in both FATs, FSInfo); a car
/// that missed it makes its own TeslaCam in the same root slot and cluster, so oxidrive's
/// SavedClips and SentryClips are gone, and fsck finds nothing wrong; a car that saw it reads the
/// volume again and keeps them. At least one of the two cars misses the window.
#[test]
#[ignore = "needs the oxidrive 728b71f images and nvs.csv under OXIDRIVE_CURRENT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and a FAT checker"]
fn external_oxidrive_window_between_polls_is_overwritten_by_a_caching_car() {
    let slow = short_window_run("hazard-short-5s.img", env_u64("OXIDRIVE_CAR_POLL_MS", 5000) * MS);
    let fast = short_window_run("hazard-short-2s.img", 2 * S);
    for w in [&slow, &fast] { eprintln!("[hazard] poll {}: a window of {} missed {}, TeslaCam {:?}, fsck clean {}", secs(w.poll_ns), secs(w.window.1 - w.window.0), w.missed, w.teslacam, w.fsck.0); }
    for w in [&slow, &fast] {
        assert!(!w.oxidrive_wrote.is_empty(), "the window wrote the card");
        let kept = w.teslacam.iter().any(|n| n == "SavedClips") && w.teslacam.iter().any(|n| n == "SentryClips");
        assert_eq!(kept, !w.missed, "poll {}: oxidrive's folders survive exactly when the car saw the window: {:?}", secs(w.poll_ns), w.teslacam);
        assert!(w.teslacam.iter().any(|n| n == "RecentClips") && w.fsck.0, "the car's folder is there and fsck passes either way");
    }
    assert!(slow.missed || fast.missed, "neither car missed the window");
}

const CAMS: [&str; 4] = ["front", "back", "left_repeater", "right_repeater"];

/// A Tesla-like recorder, scaled down: a set of four camera clips every `set_ns` in RecentClips,
/// a ring of `ring` sets with the oldest deleted (on a full card the next set takes its
/// clusters), and every `event_every` sets an event that copies the last `event_sets` sets into
/// SentryClips or SavedClips at the bus's speed; past `events_kept` events the oldest event's
/// clips are deleted, as the car makes room.
struct Tesla { set_ns: u64, clip: usize, ring: usize, event_every: u32, event_sets: usize, events_kept: usize, minute: u32, sets: u32, events: u32, recent: std::collections::VecDeque<Vec<String>>, kept: std::collections::VecDeque<Vec<String>> }

impl Tesla {
    fn new(minute: u32) -> Self { Tesla { set_ns: 6 * S, clip: 384 << 10, ring: 12, event_every: 5, event_sets: 2, events_kept: 8, minute, sets: 0, events: 0, recent: Default::default(), kept: Default::default() } }

    /// Take up the ring a previous boot left in RecentClips, oldest first, as the car's rotation
    /// does when it lists the folder.
    fn resume<D: esp_soc::usb_host::car::Drive>(&mut self, car: &mut Car<D>) {
        let fs = car.fs.clone().unwrap();
        let mut sets: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for e in fs.list(&mut car.disk, "TeslaCam/RecentClips").unwrap_or_default() {
            sets.entry(e.name.chars().take(19).collect()).or_default().push(format!("TeslaCam/RecentClips/{}", e.name));
        }
        self.recent = sets.into_values().collect();
        let mut events = Vec::new();
        for folder in ["SentryClips", "SavedClips"] {
            for ev in fs.list(&mut car.disk, &format!("TeslaCam/{folder}")).unwrap_or_default() {
                let dir = format!("TeslaCam/{folder}/{}", ev.name);
                let files: Vec<String> = fs.list(&mut car.disk, &dir).unwrap_or_default().into_iter().map(|e| format!("{dir}/{}", e.name)).collect();
                if !files.is_empty() { events.push((ev.name.clone(), files)); }
            }
        }
        events.sort();
        self.kept = events.into_iter().map(|e| e.1).collect();
    }

    /// One set; an event after every `event_every`. Stops at the first set the medium cut.
    fn step<D: esp_soc::usb_host::car::Drive>(&mut self, car: &mut Car<D>) -> Result<Recorded, String> {
        let m = self.minute;
        self.minute += 1;
        self.sets += 1;
        let paths: Vec<String> = CAMS.iter().map(|c| format!("TeslaCam/RecentClips/{}-{c}.mp4", stamp(m))).collect();
        let datas: Vec<Vec<u8>> = paths.iter().map(|p| mp4_bytes(p, self.clip)).collect();
        let clips: Vec<(&str, &[u8])> = paths.iter().map(|p| p.as_str()).zip(datas.iter().map(|d| d.as_slice())).collect();
        let chunks = self.clip.div_ceil(car.cfg.chunk) as u64;
        let r = car.record_set(&clips, self.set_ns / chunks.max(1))?;
        self.recent.push_back(paths);
        if !matches!(r, Recorded::Closed { .. }) { return Ok(r); }
        while self.recent.len() > self.ring { for p in self.recent.pop_front().unwrap() { car.delete(&p)?; } }
        if self.sets.is_multiple_of(self.event_every) {
            self.events += 1;
            let folder = if self.events % 2 == 1 { "SentryClips" } else { "SavedClips" };
            let dir = format!("TeslaCam/{folder}/{}", stamp(m));
            let src: Vec<String> = self.recent.iter().rev().take(self.event_sets).flatten().cloned().collect();
            let copies: Vec<(String, Vec<u8>)> = src.iter().map(|p| (format!("{dir}/{}", p.rsplit('/').next().unwrap()), car.files[p].data.clone())).collect();
            let clips: Vec<(&str, &[u8])> = copies.iter().map(|(p, d)| (p.as_str(), d.as_slice())).collect();
            let r = car.record_set(&clips, 0)?;
            self.kept.push_back(copies.into_iter().map(|c| c.0).collect());
            while self.kept.len() > self.events_kept { for p in self.kept.pop_front().unwrap() { car.delete(&p)?; } }
            if !matches!(r, Recorded::Closed { .. }) { return Ok(r); }
        }
        Ok(r)
    }
}

/// Fill all of a volume but `leave` bytes with one file, FILLER.BIN, its chain written straight
/// into both FATs (its data left as the image's zeros): a card as full as a car keeps it.
fn fill_volume(image: &std::path::Path, leave: u64) {
    let mut f = FileDisk(std::fs::OpenOptions::new().read(true).write(true).open(image).unwrap());
    let mut fs = Fat32::mount(&mut f).unwrap();
    let cb = fs.cluster_bytes() as u64;
    let n = fs.clusters as u64 - 1 - leave / cb;
    // clusters 3.. n+2 (2 is the root); the FAT built a sector at a time
    let fat = fs.volume_lba + fs.reserved as u64;
    let last = 3 + n - 1;
    for sec in (3 / 128)..=(last / 128) {
        let mut s = [0u8; 512];
        f.read_blocks(fat + sec, &mut s).unwrap();
        for c in (sec * 128).max(3)..((sec + 1) * 128).min(last + 1) {
            let v: u32 = if c == last { 0x0fff_ffff } else { c as u32 + 1 };
            s[(c % 128) as usize * 4..(c % 128) as usize * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        for copy in 0..fs.fats as u64 { f.write_blocks(fat + copy * fs.fat_sectors as u64 + sec, &s).unwrap(); }
    }
    // the entry, made empty, then given cluster 3 and the size
    drop(fs.create(&mut f, "FILLER.BIN").unwrap());
    let root = fs.data_lba;
    let mut s = [0u8; 512];
    f.read_blocks(root, &mut s).unwrap();
    let slot = s.chunks(32).position(|e| &e[..11] == b"FILLER  BIN").unwrap();
    let e = &mut s[slot * 32..slot * 32 + 32];
    e[20..22].copy_from_slice(&0u16.to_le_bytes());
    e[26..28].copy_from_slice(&3u16.to_le_bytes());
    e[28..32].copy_from_slice(&((n * cb).min(u32::MAX as u64) as u32).to_le_bytes());
    f.write_blocks(root, &s).unwrap();
    let mut info = [0u8; 512];
    f.read_blocks(fs.volume_lba + fs.fsinfo as u64, &mut info).unwrap();
    let free = fs.clusters - 1 - n as u32;
    info[488..492].copy_from_slice(&free.to_le_bytes());
    info[492..496].copy_from_slice(&(last as u32).to_le_bytes());
    f.write_blocks(fs.volume_lba + fs.fsinfo as u64, &info).unwrap();
}

/// What the car did on earlier boots.
struct History { files: BTreeMap<String, CarFile>, minute: u32, notes: Vec<String> }

/// `boots` boots of a car on `image`, offline: each mounts the volume (fsck.fat first, dosfstools'
/// way, when it is dirty), records with Linux's writeback (`WritebackCfg`'s defaults, a seed of
/// its own) for `secs_each` seconds, and loses power then, its last pages with it: the card keeps
/// what an interrupted writeback leaves, dirty, as a car that sleeps leaves it.
fn previous_boots(image: &std::path::Path, boots: u32, secs_each: u64) -> History {
    use esp_soc::usb_host::car::Offline;
    use esp_soc::usb_host::fat32::FsckWrite;
    use esp_soc::usb_host::pagecache::WritebackCfg;
    let mut h = History { files: BTreeMap::new(), minute: 0, notes: Vec::new() };
    let mut t = 0;
    for b in 0..boots {
        let dev = Offline { dev: FileDisk(std::fs::OpenOptions::new().read(true).write(true).open(image).unwrap()), now: t };
        let cfg = WritebackCfg { seed: 0x2545_f491_4f6c_dd1d ^ (b as u64 + 1).wrapping_mul(0x9e37_79b9_7f4a_7c15), ..WritebackCfg::default() };
        let mut car = Car::new(PageCache::writeback(dev, cfg), CarCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, ..CarCfg::default() });
        car.fsck_at_mount = Some(FsckWrite::Changed);
        car.mount().unwrap();
        if let Some((_, _, r)) = &car.last_fsck { h.notes.push(format!("boot {b}: fsck: {}", r.summary())); }
        let mut tesla = Tesla::new(h.minute);
        tesla.resume(&mut car);
        let end = car.now() + secs_each * S;
        while car.now() < end { tesla.step(&mut car).unwrap(); }
        // power goes at a time the seed sets within the next set
        car.idle((cfg.seed % 6_000) * MS).unwrap();
        let lost = car.power_cut();
        h.notes.push(format!("boot {b}: {} sets, {} events, power cut at {}: {lost} dirty pages lost", tesla.sets, tesla.events, secs(car.now())));
        for (p, f) in car.files { h.files.insert(p, f); }
        h.minute = tesla.minute + 5;
        t = car.disk.dev.now;
    }
    h
}

/// MP4's top-level boxes as oxidrive checks them (its oxidrive-core/src/clip/complete.rs
/// `verify_mp4`): `ftyp` first, every box type printable, sizes tiling the file, a `moov`.
fn mp4_complete(b: &[u8]) -> bool {
    let (mut off, mut first, mut moov) = (0usize, true, false);
    let mut boxes = 0;
    while off < b.len() {
        boxes += 1;
        if boxes > 64 || b.len() - off < 8 { return false; }
        let size32 = u32::from_be_bytes(b[off..off + 4].try_into().unwrap()) as u64;
        let kind = &b[off + 4..off + 8];
        if !kind.iter().all(|c| (0x20..=0x7e).contains(c)) || (first && kind != b"ftyp") { return false; }
        moov |= kind == b"moov";
        let size = match size32 { 0 => (b.len() - off) as u64, 1 => { if b.len() - off < 16 { return false; } u64::from_be_bytes(b[off + 8..off + 16].try_into().unwrap()) }, s => s };
        if size < if size32 == 1 { 16 } else { 8 } || off as u64 + size > b.len() as u64 { return false; }
        off += size as usize;
        first = false;
    }
    moov && b.len() >= 32 << 10
}

fn f_len(truth: &BTreeMap<String, CarFile>, p: &str) -> usize { truth.get(p).map_or(0, |f| f.data.len()) }

/// What the car knew of a file when a snapshot was taken, its clip kept once elsewhere.
#[derive(Clone, Copy, Debug)]
struct FileState { written: u64, complete: bool, deleted: bool }
fn file_states(files: &BTreeMap<String, CarFile>) -> BTreeMap<String, FileState> {
    files.iter().map(|(p, f)| (p.clone(), FileState { written: f.written, complete: f.complete, deleted: f.deleted })).collect()
}

/// One clip on a medium snapshot, judged as each completeness rule would judge it.
#[derive(Default, Clone, Copy, Debug)]
struct RuleCount { clips: u64, read_on_sight: (u64, u64), mp4: (u64, u64), structure_mp4: (u64, u64), structure_mp4_settled: (u64, u64), torn_closed: u64, torn_unknown: u64 }

/// Every Saved and Sentry clip on a snapshot of the card: what each rule accepts, and how many
/// of those differ from what the car believed the file held at `at_ns`. The rules: anything with
/// a size; MP4 boxes complete (oxidrive's complete.rs); and the entry agreeing with the chain
/// (oxidrive's shared.rs `structural`: a whole chain, size set, as many clusters as the size
/// needs); and nothing written to the clip's data, FAT sectors or entry sector for `settle_ns`
/// (`assess` with a settle window, from the card's write log).
fn judge_snapshot(img: &std::path::Path, truth: &BTreeMap<String, CarFile>, state: &BTreeMap<String, FileState>, writes: &[CardWrite], at_ns: u64, settle_ns: u64) -> RuleCount {
    let mut d = FileDisk(std::fs::File::open(img).unwrap());
    let mut c = RuleCount::default();
    let Ok(fs) = Fat32::mount(&mut d) else { return c };
    let cb = fs.cluster_bytes() as u64;
    let quiet = |lo: u64, n: u64| !writes.iter().any(|w| w.at_ns + settle_ns > at_ns && w.at_ns <= at_ns && w.lba < lo + n && lo < w.lba + w.count);
    for folder in ["SavedClips", "SentryClips"] {
        let Ok(events) = fs.list(&mut d, &format!("TeslaCam/{folder}")) else { continue };
        for ev in events.iter().filter(|e| e.attr & 0x10 != 0) {
            let Ok(files) = fs.list(&mut d, &format!("TeslaCam/{folder}/{}", ev.name)) else { continue };
            for e in files.iter().filter(|e| e.attr & 0x10 == 0) {
                let path = format!("TeslaCam/{folder}/{}/{}", ev.name, e.name);
                c.clips += 1;
                let chain = if e.cluster >= 2 { fs.chain(&mut d, e.cluster).ok() } else { Some(Vec::new()) };
                let whole = chain.is_some();
                let chain = chain.unwrap_or_default();
                let mut bytes = Vec::new();
                for &k in chain.iter().take((e.size as u64).div_ceil(cb) as usize) {
                    let mut buf = vec![0u8; cb as usize];
                    if d.read_blocks(fs.data_lba + (k as u64 - 2) * fs.sectors_per_cluster as u64, &mut buf).is_err() { break; }
                    bytes.extend_from_slice(&buf);
                }
                bytes.truncate(e.size as usize);
                let torn = truth.get(&path).is_none_or(|t| t.data != bytes);
                let structural = whole && !(e.size == 0 && e.cluster != 0) && chain.len() as u64 == (e.size as u64).div_ceil(cb);
                let mp4 = bytes.len() == e.size as usize && mp4_complete(&bytes);
                let settled = chain.iter().all(|&k| quiet(fs.data_lba + (k as u64 - 2) * fs.sectors_per_cluster as u64, fs.sectors_per_cluster as u64))
                    && chain.iter().all(|&k| { let s = fs.volume_lba + fs.reserved as u64 + k as u64 / 128; quiet(s, 1) && quiet(s + fs.fat_sectors as u64, 1) });
                let add = |r: &mut (u64, u64), ok: bool| if ok { r.0 += 1; if torn { r.1 += 1; } };
                add(&mut c.read_on_sight, e.size > 0);
                add(&mut c.mp4, mp4);
                add(&mut c.structure_mp4, mp4 && structural);
                add(&mut c.structure_mp4_settled, mp4 && structural && settled);
                // of the torn ones structure and MP4 accept: closed by the car by then, or not its
                // (a file it no longer knows, from a boot before)
                if mp4 && structural && torn {
                    match state.get(&path) { Some(f) if f.complete && !f.deleted && f.written as usize == f_len(truth, &path) => c.torn_closed += 1, Some(_) => {} None => c.torn_unknown += 1 }
                }
            }
        }
    }
    c
}
use esp_soc::usb_host::fat32::BlockDevice;

/// A copy of the card, as of now, kept for judging offline: an APFS clone where the file system
/// makes one, else a full copy.
fn snapshot(card: &std::path::Path, dir: &std::path::Path, n: usize, at_ns: u64) -> std::path::PathBuf {
    let p = dir.join(format!("snap-{n:03}-{:.0}s.img", at_ns as f64 / 1e9));
    std::fs::copy(card, &p).unwrap();
    p
}

/// Beside the snapshots, for judging them offline: `truth.tsv` (every file the car made: path,
/// bytes, CRC-32 of its clip), `snap-NNN-<t>s.tsv` (what the car knew of each file then: bytes
/// written, closed, deleted) and `writes.tsv` (every run of blocks the card was written, with its
/// time: a dirty log no window hides).
fn dump_truth(dir: &std::path::Path, files: &BTreeMap<String, CarFile>, snaps: &[(std::path::PathBuf, u64, BTreeMap<String, FileState>)], writes: &[CardWrite]) {
    let crc = |d: &[u8]| !d.iter().fold(!0u32, |c, &b| (0..8).fold(c ^ b as u32, |c, _| if c & 1 != 0 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 }));
    let truth: String = files.iter().map(|(p, f)| format!("{p}\t{}\t{:08x}\n", f.data.len(), crc(&f.data))).collect();
    std::fs::write(dir.join("truth.tsv"), format!("path\tbytes\tcrc32\n{truth}")).unwrap();
    for (img, _, state) in snaps {
        let rows: String = state.iter().map(|(p, f)| format!("{p}\t{}\t{}\t{}\n", f.written, f.complete, f.deleted)).collect();
        std::fs::write(img.with_extension("tsv"), format!("path\twritten\tclosed\tdeleted\n{rows}")).unwrap();
    }
    let rows: String = writes.iter().map(|w| format!("{}\t{}\t{}\n", w.at_ns, w.lba, w.count)).collect();
    std::fs::write(dir.join("writes.tsv"), format!("at_ns\tlba\tcount\n{rows}")).unwrap();
    eprintln!("[hazard] the car's truth, its state at each snapshot and the card's writes are in {}", dir.display());
}

/// What `judge_snapshot` counts, summed over snapshots.
fn sum_counts(cs: &[RuleCount]) -> RuleCount {
    cs.iter().fold(RuleCount::default(), |mut a, c| {
        a.clips += c.clips;
        for (x, y) in [(&mut a.read_on_sight, c.read_on_sight), (&mut a.mp4, c.mp4), (&mut a.structure_mp4, c.structure_mp4), (&mut a.structure_mp4_settled, c.structure_mp4_settled)] { x.0 += y.0; x.1 += y.1; }
        a.torn_closed += c.torn_closed;
        a.torn_unknown += c.torn_unknown;
        a
    })
}

/// Item 4. Linux's writeback against oxidrive's shared passes and windows: a full card that
/// earlier boots left torn (each lost its last pages to a power cut), then a car recording on it
/// with the firmware's writeback model (pagecache `WritebackCfg`, testkit/src/sim.rs's
/// parameters): sets of four camera clips into a RecentClips ring whose deletions hand their
/// clusters to the next set, and Sentry and Saved events copied from it, the entries' sizes
/// rewritten as the files grow, the metadata held back. oxidrive passes every minute while the car
/// records, and opens windows in the quiet minutes between. Every upload is compared with what the
/// car believes the file holds, and counted by the way it came: a shared pass or a window. The
/// medium is copied at every set (`OXIDRIVE_SNAPSHOT_DIR` keeps the copies) and each copy judged
/// by the completeness rules, as a reader with no dirty log would judge it.
#[test]
#[ignore = "needs the oxidrive 728b71f images and nvs.csv under OXIDRIVE_CURRENT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and a FAT checker"]
fn external_oxidrive_archives_no_torn_clip_from_a_writeback_car() {
    use esp_soc::usb_host::fat32::FsckWrite;
    use esp_soc::usb_host::pagecache::WritebackCfg;
    let card_mib = env_u64("OXIDRIVE_CARD_MIB", 4096);
    let proto = common::tmp("hazard-writeback-proto.img");
    sd_image::fat32(&proto, card_mib << 20, 32 << 10, "ESP32SIM").unwrap();
    fill_volume(&proto, env_u64("OXIDRIVE_FREE_MIB", 160) << 20);
    let hist = previous_boots(&proto, env_u64("OXIDRIVE_BOOTS", 4) as u32, env_u64("OXIDRIVE_BOOT_SECS", 90));
    for n in &hist.notes { eprintln!("[hazard] {n}"); }
    let snaps = std::env::var("OXIDRIVE_SNAPSHOT_DIR").map(std::path::PathBuf::from).unwrap_or_else(|_| common::tmp("hazard-writeback-snaps"));
    std::fs::create_dir_all(&snaps).unwrap();
    let mut r = hazard_rig("hazard-writeback.img", Some(proto.clone()));
    let shared = r.shared.clone();
    let card = r.card.clone();
    let (cmd, mqtt) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    let wall = std::time::Instant::now();
    let record_secs = env_u64("OXIDRIVE_RECORD_SECS", 120);
    let taken_snaps: Vec<(std::path::PathBuf, u64, BTreeMap<String, FileState>)>;
    let (files, landings, stats, cache, fsck) = {
        let cfg = WritebackCfg::default();
        let mut car = car_on_with(&mut r, |m| PageCache::writeback(m, cfg), CarCfg { linux_entries: true, dirty_flag: true, ..CarCfg::default() }, Some(FsckWrite::Changed));
        let fsck = car.last_fsck.clone();
        for (p, f) in &hist.files { car.files.entry(p.clone()).or_insert_with(|| f.clone()); }
        // the medium copied every `OXIDRIVE_SNAPSHOT_MS` while the car records, mid-write included
        let every = env_u64("OXIDRIVE_SNAPSHOT_MS", 1000) * MS;
        let hooked = std::rc::Rc::new(std::cell::RefCell::new(Vec::new()));
        let (h, c, d) = (hooked.clone(), card.clone(), snaps.clone());
        let mut next = 0u64;
        let mut tesla = Tesla::new(hist.minute);
        tesla.resume(&mut car);
        for phase in 0..2 {
            let (h, c, d) = (h.clone(), c.clone(), d.clone());
            car.on_tick = Some(Box::new(move |t, files| {
                if t < next { return; }
                next = t + every;
                let n = h.borrow().len();
                h.borrow_mut().push((snapshot(&c, &d, n, t), t, file_states(files)));
            }));
            let end = car.now() + record_secs * S;
            while car.now() < end {
                let st = tesla.step(&mut car).unwrap();
                if !matches!(st, Recorded::Closed { .. }) { eprintln!("[hazard] a set was cut at {}: {st:?}", secs(car.now())); }
            }
            car.on_tick = None;
            // quiet: a window when oxidrive's policy allows (the card nearly full overrides the
            // cooldown), forced after the first
            let windows_before = shared.borrow().all("supervisor: card taken for archiving").len();
            let deadline = car.now() + 150 * S;
            while shared.borrow().all("supervisor: card returned to the car").len() <= windows_before && car.now() < deadline {
                if phase > 0 && car.now() + 80 * S > deadline { mqtt.publish(&cmd, "force_open").unwrap(); }
                car.idle(2 * S).unwrap();
            }
            car.idle(10 * S).unwrap();
            let t = car.now();
            let n = hooked.borrow().len();
            hooked.borrow_mut().push((snapshot(&card, &snaps, n, t), t, file_states(&car.files)));
        }
        taken_snaps = hooked.take();
        car.unmount().unwrap();
        (car.files.clone(), car.disk.landings.clone(), car.stats, car.disk.stats, fsck)
    };
    r.report(10);
    eprintln!("[hazard] {:.0} s wall for {} emulated; car: {stats:?}; its cache: {cache:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    if let Some((a, b, f)) = &fsck { eprintln!("[hazard] the car's fsck at mount, {} to {}: {}", secs(*a), secs(*b), f.summary()); }
    let sh = shared.borrow();
    console(&sh);
    for (a, b) in windows(&sh) { eprintln!("[hazard] window {} to {} ({})", secs(a), secs(b), secs(b.saturating_sub(a))); }
    // every upload against the car's file, by path
    let ups = archived(&r.smb.path(), &files);
    let mut per: BTreeMap<(&str, String), u64> = BTreeMap::new();
    for (p, v) in &ups {
        if *v == Archived::Absent { continue; }
        let way = path_of_upload(&sh, p.strip_prefix("TeslaCam/").unwrap());
        let kind = match v { Archived::Same => "same".to_string(), Archived::Prefix(_) => "a prefix".to_string(), _ => "differs".to_string() };
        *per.entry((way, kind)).or_default() += 1;
        if *v != Archived::Same { eprintln!("[hazard] TORN: {p}: {v:?} by {way}; the car's file: {} of {} bytes written, closed {}", files[p].written, files[p].data.len(), files[p].complete); }
    }
    eprintln!("[hazard] uploads by the way they came and against the car's file: {per:?}");
    let in_flight = |t: u64| files.values().filter(|f| f.started_ns <= t && f.closed_ns.is_none_or(|c| c > t)).count();
    eprintln!("[hazard] the car wrote {} landings; {} snapshots in {}", landings.len(), taken_snaps.len(), snaps.display());
    for meta in [false, true] {
        let mut lag: Vec<u64> = landings.iter().filter(|l| l.meta == meta).map(|l| l.at_ns - l.dirtied_ns).collect();
        let mut late: Vec<u64> = landings.iter().filter(|l| l.meta == meta).map(|l| l.at_ns.saturating_sub(l.due_ns)).collect();
        lag.sort_unstable();
        late.sort_unstable();
        let q = |v: &[u64], f: f64| v.get(((v.len() as f64 * f) as usize).min(v.len().saturating_sub(1))).map_or(0.0, |&x| x as f64 / 1e9);
        eprintln!("[hazard] {} landings: {}; written to landed median {:.2} s, 99% {:.2} s, max {:.2} s; landed after falling due median {:.3} s, 99% {:.3} s, max {:.3} s",
                  if meta { "metadata" } else { "data" }, lag.len(), q(&lag, 0.5), q(&lag, 0.99), q(&lag, 1.0), q(&late, 0.5), q(&late, 0.99), q(&late, 1.0));
    }
    let mut counts = Vec::new();
    for (img, t, state) in &taken_snaps {
        let c = judge_snapshot(img, &files, state, &sh.writes, *t, 120 * S);
        eprintln!("[hazard] snapshot at {}: {} clips ({} files in flight): accepted, torn: on sight {:?}, MP4 {:?}, structure+MP4 {:?} ({} torn closed, {} from earlier boots), +settle 120 s {:?}", secs(*t), c.clips, in_flight(*t), c.read_on_sight, c.mp4, c.structure_mp4, c.torn_closed, c.torn_unknown, c.structure_mp4_settled);
        counts.push(c);
        if std::env::var_os("OXIDRIVE_SNAPSHOT_DIR").is_none() { let _ = std::fs::remove_file(img); }
    }
    let total = sum_counts(&counts);
    eprintln!("[hazard] over all snapshots: {total:?}");
    if std::env::var_os("OXIDRIVE_SNAPSHOT_DIR").is_some() { dump_truth(&snaps, &files, &taken_snaps, &sh.writes); }
    assert!(total.read_on_sight.1 > 0, "no torn clip on the medium in any snapshot: nothing for a rule to catch");
    let torn: u64 = per.iter().filter(|(k, _)| k.1 != "same").map(|(_, n)| n).sum();
    let same: u64 = per.iter().filter(|(k, _)| k.1 == "same").map(|(_, n)| n).sum();
    assert!(same > 0, "nothing archived");
    assert_eq!(torn, 0, "a torn clip was archived as finished");
}

/// Item 6. The first epoch after a mount. A car that wakes runs fsck when the volume is dirty,
/// which it nearly always is (a car that sleeps cuts the board's power); here its fsck rewrites
/// FAT1 and FAT2 whole (`FsckWrite::Bulk`, the shared-reading plan's assumption; dosfstools 4.2
/// writes the changed sectors and the second FAT whole only when the copies differ). The card is
/// left dirty and torn by earlier boots (`previous_boots`), larger than the other tests' so the
/// FATs take time to rewrite. Two bursts: the fsck at the first mount, and, once oxidrive's first
/// shared pass (forced as early as its broker allows) is reading a clip, a USB reset without power
/// loss (`UsbHost::reset_device`) and the car's fsck again. Asserted, as oxidrive 728b71f
/// behaves: its first pass starts well after the mount's burst (it needs WiFi and the NAS), the
/// pass reads through the second burst, and nothing it archives differs from the car's file.
#[test]
#[ignore = "needs the oxidrive 728b71f images and nvs.csv under OXIDRIVE_CURRENT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and a FAT checker"]
fn external_oxidrive_reads_through_the_cars_fsck_burst_after_a_reset() {
    use esp_soc::usb_host::fat32::FsckWrite;
    use esp_soc::usb_host::pagecache::WritebackCfg;
    let card_gib = env_u64("OXIDRIVE_FSCK_CARD_GIB", 64);
    let proto = common::tmp("hazard-fsck-proto.img");
    sd_image::fat32(&proto, card_gib << 30, 32 << 10, "ESP32SIM").unwrap();
    let hist = previous_boots(&proto, 2, 60);
    for n in &hist.notes { eprintln!("[hazard] {n}"); }
    let mut o = RigOpts { card_name: "hazard-fsck.img".into(), card_bytes: card_gib << 30, car: true, image: Some(proto.clone()), log_writes: true, ..RigOpts::new(oxidrive_board::current_dir()) };
    o.prg_us = Some(env_u64("OXIDRIVE_SD_PRG_US", 1500));
    o.timed = Some(std::env::var("OXIDRIVE_SD_TIMING").map_or(true, |v| v != "0"));
    let mut r = rig_with(o);
    let shared = r.shared.clone();
    let (cmd, mqtt) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    let wall = std::time::Instant::now();
    let (files, landings, mount_burst, reset_burst, read_at) = {
        let cfg = WritebackCfg::default();
        let mut car = car_on_with(&mut r, |m| PageCache::writeback(m, cfg), CarCfg { linux_entries: true, dirty_flag: true, ..CarCfg::default() }, Some(FsckWrite::Bulk));
        let mount_burst = car.last_fsck.clone().expect("the volume was dirty: fsck at the mount");
        for (p, f) in &hist.files { car.files.entry(p.clone()).or_insert_with(|| f.clone()); }
        let mut tesla = Tesla::new(hist.minute);
        tesla.resume(&mut car);
        // record until the broker is up, then force a pass until one reads a clip
        while shared.borrow().when("session ready").is_none() && car.now() < 60 * S { tesla.step(&mut car).unwrap(); }
        // a clip upload begun outside any window: a shared pass reading it
        let reading = |sh: &Shared| {
            let ws = windows(sh);
            sh.lines.iter().find(|l| l.1.contains("smb-> begin_part") && !l.1.contains("begin_part .") && !ws.iter().any(|&(a, b)| (a..=b).contains(&l.0))).map(|l| l.0)
        };
        mqtt.publish(&cmd, "force_open").unwrap();
        while reading(&shared.borrow()).is_none() && car.now() < 200 * S { car.idle(20 * MS).unwrap(); }
        let read_at = reading(&shared.borrow()).expect("no shared pass read a clip");
        eprintln!("[hazard] the pass began reading a clip at {}; the car resets the port at {}", secs(read_at), secs(car.now()));
        car.disk.dev.host.reset_device().unwrap_or_else(|e| panic!("reset: {e}"));
        let r2 = car.fsck_now(FsckWrite::Bulk).unwrap();
        let reset_burst = car.last_fsck.clone().unwrap();
        eprintln!("[hazard] after the reset: {}", r2.summary());
        let end = car.now() + 40 * S;
        while car.now() < end { tesla.step(&mut car).unwrap(); }
        car.unmount().unwrap();
        (car.files.clone(), car.disk.landings.clone(), mount_burst, reset_burst, read_at)
    };
    r.report(10);
    eprintln!("[hazard] {:.0} s wall for {} emulated", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    let sh = shared.borrow();
    console(&sh);
    let passes = sh.all("pipeline::archiver: buffer:");
    let first_pass = passes.first().copied().expect("a pass");
    eprintln!("[hazard] the mount's fsck burst {} to {} ({}); oxidrive's first pass or window began at {}, {} after it", secs(mount_burst.0), secs(mount_burst.1), mount_burst.2.summary(), secs(first_pass), secs(first_pass.saturating_sub(mount_burst.1)));
    eprintln!("[hazard] the reset's fsck burst {} to {}: {}", secs(reset_burst.0), secs(reset_burst.1), reset_burst.2.summary());
    let in_burst: u64 = sh.writes.iter().filter(|w| (reset_burst.0..=reset_burst.1).contains(&w.at_ns)).map(|w| w.count).sum();
    eprintln!("[hazard] the card took {in_burst} blocks during the reset's burst; the car {} landings in all", landings.len());
    let ups = archived(&r.smb.path(), &files);
    for (p, v) in ups.iter().filter(|(_, v)| *v != Archived::Absent) { eprintln!("[hazard] {p}: {v:?} ({})", path_of_upload(&sh, p.strip_prefix("TeslaCam/").unwrap())); }
    let (clean, out) = fsck_report(&r.card);
    eprintln!("[hazard] fsck after the run (clean {clean}): {}", out.trim().replace('\n', "; "));
    assert!(mount_burst.2.dirty_cleared && mount_burst.2.fat_bytes_written > 0, "a burst at the mount");
    assert!(first_pass > mount_burst.1 + 5 * S, "the first pass waits out the mount's burst");
    assert!(read_at < reset_burst.1, "the pass was reading when the burst began");
    assert!(ups.iter().all(|(_, v)| matches!(v, Archived::Same | Archived::Absent)), "a torn clip archived");
}

/// Item 9. The die temperature sensor is a model now (`esp32p4::periph::Tsens`), not a register
/// the test seeds, so a P4 that restarts reads it again: oxidrive vetoes every window and pass
/// without a reading (its window.rs `why_not`, `may_share`). The test restarts the P4 the way
/// esp_restart does once oxidrive has read its senses, and waits for them again.
#[test]
#[ignore = "needs the oxidrive 728b71f images and nvs.csv under OXIDRIVE_CURRENT_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_reads_its_temperature_after_a_restart() {
    let mut r = rig_with(RigOpts::new(oxidrive_board::current_dir()));
    let senses = |r: &Rig| r.p4_lines().iter().filter(|l| l.contains("senses:")).map(|l| l.to_string()).collect::<Vec<_>>();
    while senses(&r).is_empty() && r.cs.now_ns < 30 * S { let t = r.cs.now_ns + S; r.run_to(t); }
    // LP_CLKRST HPCPU_RESET_CTRL0.HPCORE0_SW_RESET (0x50111014 bit 13, lp_clkrst_reg.h): a CPU
    // reset with the clocks left as the application set them, as a watchdog's reset leaves them
    emu_core::Bus::write32(&mut r.cs.p4.bus, 0x5011_1014, 1 << 13).unwrap();
    let restart = r.cs.now_ns;
    while senses(&r).len() < 2 && r.cs.now_ns < restart + 40 * S { let t = r.cs.now_ns + S; r.run_to(t); }
    let s = senses(&r);
    for l in &s { eprintln!("[hazard] {}", l.split("senses:").nth(1).unwrap_or(l).split("quiet").next().unwrap_or("")); }
    assert_eq!(r.cs.summary.p4_resets, 1, "the P4 restarted");
    assert!(s.len() >= 2 && s.iter().all(|l| l.contains("temp_c=Some(25)")), "a temperature before and after the restart");
}
