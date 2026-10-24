//! oxidrive on an exFAT card (esp32p4.md, "An exFAT car"): the owner's firmware (not in this
//! repository) with exFAT archive windows, on the rig of `oxidrive_services` (`oxidrive_rig`), and
//! a car that writes exFAT as Tesla's Linux does (`esp_soc::usb_host::exfat`) on its USB port.
//!
//! - `OXIDRIVE_EXFAT_DIR` (default the fixtures' `oxidrive-exfat`): the plain build, with exFAT
//!   windows, for the window, the window gate and the shared-pass refusal.
//! - `OXIDRIVE_EXFAT_TRACE_DIR` (default `oxidrive-exfat-trace`): the SD_TRACE build, which
//!   uploads every car write to `oxidrive-trace/` on the share; `OXI_BIN`, the `oxi` CLI built from
//!   oxidrive's repository, classifies the trace and is checked against the car's ground truth;
//!   `OXIDRIVE_EXFAT_OUT` keeps the trace and the card image (never commit either).
//!
//! The card is a fresh exFAT image (`sd_image::exfat`), `OXIDRIVE_CARD_MIB` (4096) with
//! `OXIDRIVE_EXFAT_CLUSTER_KIB` (32 KiB) clusters. The SD bus is timed and the card programs for
//! `OXIDRIVE_SD_PRG_US` (1500 us, assumed) after each write, as in `oxidrive_hazards`.
#[path = "../../tests/common.rs"]
mod common;
#[path = "../../tests/sd_image.rs"]
mod sd_image;
#[path = "../../tests/oxidrive_board.rs"]
mod oxidrive_board;
#[path = "../../tests/services.rs"]
mod services;
#[path = "../../tests/hosted_slave.rs"]
mod hosted_slave;
#[path = "../../tests/oxidrive_rig.rs"]
mod oxidrive_rig;
use esp_soc::usb_host::car::{Car, CarCfg, CarFile, Recorded};
use esp_soc::usb_host::exfat::{ExFat, Truth};
use esp_soc::usb_host::fat32::{mp4_bytes, BlockDevice};
use esp_soc::usb_host::msc::Msc;
use esp_soc::usb_host::pagecache::{Landing, PageCache, WritebackCfg};
use esp_soc::usb_host::recorder::Sentry;
use esp_soc::usb_host::writelog::{self, BootClass, Region};
use esp_soc::usb_host::UsbHost;
use oxidrive_rig::*;
use std::collections::{BTreeMap, HashMap};
use std::time::Duration;

const S: u64 = 1_000_000_000;
const MS: u64 = 1_000_000;

fn env_u64(name: &str, default: u64) -> u64 { std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default) }
fn secs(ns: u64) -> String { format!("{:.3} s", ns as f64 / 1e9) }

fn fixture(var: &str, name: &str) -> String {
    std::env::var(var).unwrap_or_else(|_| format!("{}/Code/dlp/sim-fixtures/esp32p4/{name}", std::env::var("HOME").unwrap_or_default()))
}
fn exfat_dir() -> String { fixture("OXIDRIVE_EXFAT_DIR", "oxidrive-exfat") }
fn trace_dir() -> String { fixture("OXIDRIVE_EXFAT_TRACE_DIR", "oxidrive-exfat-trace") }

/// The rig on `dir`'s build with a fresh exFAT card, a car on its port, the card's writes logged.
fn exfat_rig(dir: String, card: &str) -> Rig {
    let prg = env_u64("OXIDRIVE_SD_PRG_US", 1500);
    let timed = std::env::var("OXIDRIVE_SD_TIMING").map_or(true, |v| v != "0");
    let (mib, kib) = (env_u64("OXIDRIVE_CARD_MIB", 4096), env_u64("OXIDRIVE_EXFAT_CLUSTER_KIB", 32));
    let proto = common::tmp(&format!("{card}.proto"));
    sd_image::exfat(&proto, mib << 20, (kib << 10) as u32, "ESP32SIM").unwrap();
    eprintln!("[exfat] {dir}: a {mib} MiB exFAT card of {kib} KiB clusters; SD bus {}, programming {prg} us (assumed)", if timed { "timed" } else { "untimed" });
    rig_with(RigOpts { card_name: card.into(), card_bytes: mib << 20, car: true, image: Some(proto), prg_us: Some(prg), timed: Some(timed), log_writes: true, ..RigOpts::new(dir) })
}

/// The car as Linux 5.8's exfat writes: VolumeFlags kept, the entry set rewritten as the file
/// grows, through `disk`; enumerated, ready and mounted.
fn exfat_car<'p>(r: &'p mut Rig, disk: impl FnOnce(Msc<'p>) -> PageCache<Msc<'p>>, cfg: CarCfg) -> Car<Msc<'p>, ExFat> { exfat_car_on(drive(r), disk, cfg) }

/// Enumerate, open the mass-storage interface, wait for the drive.
fn drive(r: &mut Rig) -> Msc<'_> {
    let mut host = UsbHost::new(r);
    host.log = std::env::var("OXIDRIVE_USB_LOG").is_ok_and(|v| v == "1");
    host.enumerate(10 * S).unwrap_or_else(|e| panic!("enumerate: {e}"));
    let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
    msc.wait_ready(100 * MS, 10 * S).unwrap_or_else(|e| panic!("never ready: {e}"));
    msc
}

fn exfat_car_on<'p>(msc: Msc<'p>, disk: impl FnOnce(Msc<'p>) -> PageCache<Msc<'p>>, cfg: CarCfg) -> Car<Msc<'p>, ExFat> {
    let mut car = Car::<_, ExFat>::on(disk(msc), CarCfg { linux_entries: true, dirty_flag: true, ..cfg });
    car.echo = true;
    car.disk.record = true;
    car.mount().unwrap_or_else(|e| panic!("mount: {e}"));
    car
}

fn writeback(m: Msc<'_>) -> PageCache<Msc<'_>> { PageCache::writeback(m, WritebackCfg::default()) }

/// Tesla's names: an event's single combined clip, stamped to the minute, which oxidrive takes as
/// a whole event by itself.
fn event_path(folder: &str, minute: u32) -> String { format!("TeslaCam/{folder}/2026-01-01_{:02}-{:02}-00/event.mp4", minute / 60, minute % 60) }

/// Idle until the console has a line holding `what`, polling as the car does; when it came.
fn idle_until_line(car: &mut Car<Msc<'_>, ExFat>, shared: &std::rc::Rc<std::cell::RefCell<Shared>>, what: &str, after_ns: u64, step_ns: u64, deadline_ns: u64) -> Option<u64> {
    while car.now() < deadline_ns {
        if let Some(t) = shared.borrow().all(what).into_iter().find(|&t| t >= after_ns) { return Some(t); }
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
            if by_lba.get(&lba).is_some_and(|v| v.iter().any(|&t| t + S >= w.at_ns && t <= w.at_ns + S)) { continue; }
            match out.last_mut() {
                Some(o) if o.at_ns == w.at_ns && o.lba + o.count == lba => o.count += 1,
                _ => out.push(CardWrite { at_ns: w.at_ns, lba, count: 1 }),
            }
        }
    }
    out
}

/// Window intervals from the console: card taken, card returned.
fn windows(shared: &Shared) -> Vec<(u64, u64)> {
    let taken = shared.all("supervisor: card taken for archiving");
    let back = shared.all("supervisor: card returned to the car");
    taken.iter().map(|&t| (t, back.iter().copied().find(|&b| b >= t).unwrap_or(u64::MAX))).collect()
}

/// The firmware's lines about windows, passes, the card and the archive.
fn console(sh: &Shared) {
    for (t, l) in &sh.lines {
        if ["supervisor", "window", "shared", "scan:", "archiver", "msc:", "usb:", "smb-> finish", "smb-> begin_part", "fs:", "sdwrap", "exFAT", "layout", "trace"].iter().any(|k| l.contains(k)) { eprintln!("[console {:>9}] {l}", secs(*t)); }
    }
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

/// A copy of the card as it is now (a clone on APFS), for checking mid-run.
fn snapshot(card: &std::path::Path, tag: &str) -> std::path::PathBuf {
    let p = common::tmp(&format!("{tag}.img"));
    std::fs::copy(card, &p).unwrap();
    p
}

/// What an exFAT card holds, read as the car reads it: whether it mounts, its dirty flag, the
/// folders under TeslaCam, and every file the car closed and did not delete, compared.
fn check_card(img: &std::path::Path, files: &BTreeMap<String, CarFile>) -> (Vec<String>, bool, Vec<String>) {
    let mut d = FileDisk(std::fs::File::open(img).unwrap());
    let mut fs = ExFat::mount(&mut d).unwrap_or_else(|e| panic!("{}: the car cannot mount it: {e}", img.display()));
    let dirty = fs.dirty(&mut d).unwrap();
    let folders: Vec<String> = fs.list(&mut d, "TeslaCam").unwrap_or_default().into_iter().map(|e| e.name).collect();
    let mut bad = Vec::new();
    for (p, f) in files.iter().filter(|f| f.1.complete && !f.1.deleted) {
        match fs.read_path(&mut d, p) { Ok(b) if b == f.data => {}, Ok(b) => bad.push(format!("{p}: {} bytes differ from the car's {}", b.len(), f.data.len())), Err(e) => bad.push(format!("{p}: {e}")) }
    }
    (folders, dirty, bad)
}

/// The regions of the writes oxidrive made itself, from the card's write log.
fn firmware_writes(card: &[CardWrite], car: &[Landing], within: &[(u64, u64)], img: &std::path::Path) -> Vec<(u64, u64, u64, String)> {
    // the directories as the image holds them at the end: the root, TeslaCam and the folders in it
    let mut d = FileDisk(std::fs::File::open(img).unwrap());
    let log = esp_soc::usb_host::exfat::TruthLog::default();
    if let Ok(mut fs) = ExFat::mount(&mut d) {
        fs.set_truth(log.clone());
        for f in fs.list(&mut d, "TeslaCam").unwrap_or_default() {
            let p = format!("TeslaCam/{}", f.name);
            for e in fs.list(&mut d, &p).unwrap_or_default() { let _ = fs.list(&mut d, &format!("{p}/{}", e.name)); }
        }
    }
    let t: Truth = log.lock().unwrap().clone();
    device_writes(card, car).into_iter().filter(|w| within.iter().any(|&(a, b)| w.at_ns > a && w.at_ns <= b.saturating_add(S))).map(|w| {
        let r = t.layout.map_or("?".to_string(), |l| {
            let mut v: Vec<(Region, u32)> = Vec::new();
            for s in w.lba..w.lba + w.count { let r = l.region(s, &t.root_clusters, &t.dir_clusters); match v.iter_mut().find(|x| x.0 == r) { Some(x) => x.1 += 1, None => v.push((r, 1)) } }
            v.iter().map(|(r, n)| format!("{}:{n}", r.name())).collect::<Vec<_>>().join(",")
        });
        (w.at_ns, w.lba, w.count, r)
    }).collect()
}

/// Force a window: `force_open` on the command topic every 2 s, as `oxidrive_hazards` does, until
/// the card is taken (the policy's 1,200 s cooldown would hold it back otherwise), then wait until
/// it is returned and the car has the medium again. The window's span. Unless `now`, first wait
/// until everything the car holds has landed and 5 s more: a window forced sooner refuses the
/// writeback and the car drops its pages (`external_oxidrive_forced_window_drops_a_writeback_cars_metadata`).
fn force_window(car: &mut Car<Msc<'_>, ExFat>, shared: &std::rc::Rc<std::cell::RefCell<Shared>>, mqtt: &services::MqttClient, cmd: &str, deadline: u64, now: bool) -> (u64, u64) {
    if !now {
        while car.disk.dirty_pages() > 0 && car.now() < deadline { car.idle(S).unwrap(); }
        car.idle(5 * S).unwrap();
    }
    let before = shared.borrow().all("supervisor: card taken for archiving").len();
    while shared.borrow().all("supervisor: card taken for archiving").len() == before && car.now() < deadline {
        mqtt.publish(cmd, "force_open").unwrap();
        car.idle(2 * S).unwrap();
    }
    let taken = *shared.borrow().all("supervisor: card taken for archiving").get(before).expect("force_open opened no window");
    let back = idle_until_line(car, shared, "supervisor: card returned to the car", taken, 100 * MS, deadline).expect("the card never came back");
    car.idle(2 * S + 100 * MS).unwrap();
    while !car.present && car.now() < deadline { car.idle(100 * MS).unwrap(); }
    (taken, back)
}

/// (a) Archive windows on an exFAT card. A fresh card: the car mounts it and polls; oxidrive's
/// first window, a minute in, finds no TeslaCam and makes it with its exFAT FatFs. The card as
/// that window left it must pass fsck and mount as the car mounts exFAT; the car, meeting UNIT
/// ATTENTION, reads the volume again and records two saved events into the window's folders. A
/// forced window archives the older one byte for byte and holds the newest back, as oxidrive's
/// archiver holds back the newest event; the car sees MEDIUM NOT PRESENT in it and UNIT ATTENTION
/// after, and records a third event. A second forced window archives the second event. The car
/// unmounts; fsck is clean, the car mounts the card again, and every clip it closed is on it as it
/// wrote it. The car is Linux's: writeback, VolumeFlags kept.
#[test]
#[ignore = "needs an oxidrive build with exFAT windows and nvs.csv under OXIDRIVE_EXFAT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and an exFAT checker"]
fn external_oxidrive_archives_an_exfat_card_through_windows() {
    let mut r = exfat_rig(exfat_dir(), "exfat-window.img");
    let shared = r.shared.clone();
    let card = r.card.clone();
    let share = r.smb.path();
    let (cmd, mqtt) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    let wall = std::time::Instant::now();
    let deadline = env_u64("OXIDRIVE_SECONDS", 420) * S;
    let mib = env_u64("OXIDRIVE_CLIP_MIB", 2) as usize;
    let on_share = |m: u32| share.join(event_path("SavedClips", m).strip_prefix("TeslaCam/").unwrap()).exists();
    let (files, landings, stats, after_first, log, held) = {
        let mut car = exfat_car(&mut r, writeback, CarCfg::default());
        assert!(car.fs.clone().unwrap().stat(&mut car.disk, "TeslaCam").unwrap().is_none(), "a fresh card");
        let t1 = idle_until_line(&mut car, &shared, "supervisor: card returned to the car", 0, 100 * MS, deadline).expect("no first window");
        eprintln!("[exfat] the first window returned the card at {}", secs(t1));
        let after_first = snapshot(&card, "after-first-window");
        // the next command meets the unit attention; the car reads the volume again
        car.idle(2 * S + 100 * MS).unwrap();
        while !car.present && car.now() < deadline { car.idle(100 * MS).unwrap(); }
        for (i, m) in [0u32, 1].iter().enumerate() {
            let p = event_path("SavedClips", *m);
            let st = car.record(&p, &mp4_bytes(&p, (mib << 20) + i * 4096)).unwrap();
            assert!(matches!(st, Recorded::Closed { .. }), "{p}: {st:?}");
        }
        car.idle(15 * S).unwrap();
        let w2 = force_window(&mut car, &shared, &mqtt, &cmd, deadline, false);
        let held = (on_share(0), on_share(1));
        eprintln!("[exfat] the second window {} to {}; on the share after it: event 0 {}, event 1 {}", secs(w2.0), secs(w2.1), held.0, held.1);
        let p = event_path("SavedClips", 5);
        let st = car.record(&p, &mp4_bytes(&p, mib << 20)).unwrap();
        assert!(matches!(st, Recorded::Closed { .. }), "the car records after the window: {st:?}");
        car.idle(15 * S).unwrap();
        let w3 = force_window(&mut car, &shared, &mqtt, &cmd, deadline, false);
        eprintln!("[exfat] the third window {} to {}", secs(w3.0), secs(w3.1));
        car.idle(40 * S).unwrap();
        car.unmount().unwrap();
        car.idle(2 * S).unwrap();
        (car.files.clone(), car.disk.landings.clone(), car.stats, after_first, car.log.clone(), held)
    };
    r.report(20);
    eprintln!("[exfat] {:.0} s wall for {} emulated; car: {stats:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    console(&shared.borrow());
    for (t, l) in log.iter().filter(|l| l.1.contains("TEST UNIT READY") || l.1.contains("cut")) { eprintln!("[car {:>9}] {l}", secs(*t)); }
    let sh = shared.borrow();
    let ws = windows(&sh);
    for (a, b) in &ws { eprintln!("[exfat] window {} to {} ({})", secs(*a), secs(*b), secs(b.saturating_sub(*a))); }
    let fw = firmware_writes(&sh.writes, &landings, &ws, &card);
    eprintln!("[exfat] oxidrive's own writes in its windows: {} runs, {} blocks", fw.len(), fw.iter().map(|w| w.2).sum::<u64>());
    for w in fw.iter().take(40) { eprintln!("[exfat]   {} {} blocks at {} ({})", secs(w.0), w.2, w.1, w.3); }
    // the first window's own writes: fsck and the car's mount
    let first = sd_image::fsck(&after_first);
    eprintln!("[exfat] fsck after the first window: {}", first.as_ref().unwrap_or_else(|e| e).trim().replace('\n', "; "));
    let (folders, dirty_first, _) = check_card(&after_first, &BTreeMap::new());
    eprintln!("[exfat] after the first window TeslaCam holds {folders:?}; VolumeFlags {}", if dirty_first { "dirty" } else { "clean" });
    // the share and the card at the end
    let ups = archived(&share, &files);
    for (p, v) in &ups { eprintln!("[exfat] {p}: {v:?}"); }
    let last = sd_image::fsck(&card);
    eprintln!("[exfat] fsck at the end: {}", last.as_ref().unwrap_or_else(|e| e).trim().replace('\n', "; "));
    let (folders_end, dirty_end, bad) = check_card(&card, &files);
    let refusals = sh.all("shared: the card is exFAT").len();
    let failed = ws.iter().any(|&(a, b)| sh.when("window: finished with failures").is_some_and(|t| (a..=b).contains(&t)));
    assert!(ws.len() >= 3, "three windows: {ws:?}");
    first.unwrap_or_else(|e| panic!("the card as the first window left it fails fsck: {e}"));
    assert!(["RecentClips", "SavedClips", "SentryClips"].iter().all(|f| folders.iter().any(|x| x == f)), "the first window made TeslaCam's folders: {folders:?}");
    assert!(stats.unit_attentions >= 3 && stats.not_ready >= 2, "the car met NOT READY in the windows and UNIT ATTENTION after each: {stats:?}");
    assert_eq!(held, (true, false), "the second window archives the older event and holds the newest back");
    for m in [0, 1] { assert_eq!(ups.iter().find(|f| f.0 == event_path("SavedClips", m)).map(|f| f.1.clone()), Some(Archived::Same), "event {m} archived as the car wrote it"); }
    assert!(ups.iter().all(|(_, v)| matches!(v, Archived::Same | Archived::Absent)), "an upload differs from the car's file");
    assert!(!failed, "the windows finished clean");
    assert!(bad.is_empty(), "the card at the end: {bad:?}");
    assert!(!dirty_end && folders_end.len() >= 3, "a clean unmount, the folders kept: {folders_end:?}");
    assert!(refusals <= 1, "the shared-pass refusal said at most once: {refusals}");
    last.unwrap_or_else(|e| panic!("fsck at the end: {e}"));
}

/// A window forced while a writeback car still holds metadata: the car records two saved events
/// on a card whose TeslaCam the first window made, and 15 s after its last write, with its
/// directory entries, bitmap and VolumeFlags still waiting out the 30 s metadata lag, `force_open`
/// opens a window. Asserted, as oxidrive 037647d behaves: the window finds no event (their folders
/// have not landed) and ends at once; the gate refuses the car's writeback, and the car, meeting
/// MEDIUM NOT PRESENT, drops the pages it held, as Linux drops writes to a medium gone; so the
/// events never reach the card, though their data clusters did. A window that opens on its own
/// waits for a quiet minute, longer than the lag, and does not meet this.
#[test]
#[ignore = "needs an oxidrive build with exFAT windows and nvs.csv under OXIDRIVE_EXFAT_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_forced_window_drops_a_writeback_cars_metadata() {
    let mut r = exfat_rig(exfat_dir(), "exfat-forced.img");
    let shared = r.shared.clone();
    let card = r.card.clone();
    let (cmd, mqtt) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    let deadline = env_u64("OXIDRIVE_SECONDS", 300) * S;
    let wall = std::time::Instant::now();
    let (files, log, stats, cache, held_before) = {
        let mut car = exfat_car(&mut r, writeback, CarCfg::default());
        idle_until_line(&mut car, &shared, "supervisor: card returned to the car", 0, 100 * MS, deadline).expect("no first window");
        car.idle(2 * S + 100 * MS).unwrap();
        while !car.present && car.now() < deadline { car.idle(100 * MS).unwrap(); }
        for m in [0u32, 1] {
            let p = event_path("SavedClips", m);
            assert!(matches!(car.record(&p, &mp4_bytes(&p, 2 << 20)).unwrap(), Recorded::Closed { .. }));
        }
        car.idle(15 * S).unwrap();
        let held_before = car.disk.dirty_pages();
        let w = force_window(&mut car, &shared, &mqtt, &cmd, deadline, true);
        eprintln!("[exfat] forced window {} to {}, the car holding {held_before} dirty pages when it was forced", secs(w.0), secs(w.1));
        car.idle(40 * S).unwrap();
        (car.files.clone(), car.log.clone(), car.stats, car.disk.stats, held_before)
    };
    r.report(10);
    eprintln!("[exfat] {:.0} s wall for {} emulated; car: {stats:?}; its cache: {cache:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    console(&shared.borrow());
    for (t, l) in log.iter().filter(|l| l.1.contains("TEST UNIT READY") || l.1.contains("dropped")) { eprintln!("[car {:>9}] {l}", secs(*t)); }
    let sh = shared.borrow();
    let forced = sh.lines.iter().filter(|l| l.1.contains("window: 0 uploaded")).count();
    let dropped: u64 = log.iter().filter_map(|l| l.1.split("medium gone, ").nth(1)?.split(' ').next()?.parse::<u64>().ok()).sum();
    let (_, _, bad) = check_card(&card, &files);
    for b in &bad { eprintln!("[exfat] lost: {b}"); }
    assert!(held_before > 0, "the car still held metadata when the window was forced");
    assert!(forced >= 2, "the first window and the forced one found nothing to upload");
    assert!(dropped > 0 || cache.write_errors > 0, "the car lost pages to the window: {dropped} dropped, {} refused", cache.write_errors);
    assert_eq!(bad.len(), 2, "both events lost on the card: {bad:?}");
}

/// (b) Shared passes refuse an exFAT card and say so once. The car records a small clip every 20 s
/// for `OXIDRIVE_SECONDS` (200), so the card is never quiet for a minute and no window opens; the
/// firmware's passes, due every minute, each find the card exFAT. Asserted: the console says so
/// once, MQTT's state carries `shared.aborted` = "card is exFAT: not read", nothing reaches the
/// share, no window opens, and oxidrive writes no block.
#[test]
#[ignore = "needs an oxidrive build with exFAT support and nvs.csv under OXIDRIVE_EXFAT_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_refuses_shared_passes_on_exfat_and_says_so_once() {
    let mut r = exfat_rig(exfat_dir(), "exfat-shared.img");
    let shared = r.shared.clone();
    let end = env_u64("OXIDRIVE_SECONDS", 200) * S;
    let state_topic = format!("oxidrive/{}/state", r.id);
    let mqtt = r.mqtt.client.clone();
    let wall = std::time::Instant::now();
    let (landings, stats) = {
        let mut car = exfat_car(&mut r, writeback, CarCfg::default());
        let mut m = 0;
        while car.now() < end {
            let p = format!("TeslaCam/RecentClips/2026-01-01_00-{m:02}-00-front.mp4");
            let st = car.record(&p, &mp4_bytes(&p, 256 << 10)).unwrap();
            assert!(matches!(st, Recorded::Closed { .. }), "{st:?}");
            m += 1;
            car.idle(20 * S).unwrap();
        }
        (car.disk.landings.clone(), car.stats)
    };
    let retained = mqtt.retained(&[&state_topic], Duration::from_secs(2)).unwrap_or_default();
    r.report(10);
    eprintln!("[exfat] {:.0} s wall for {} emulated; car: {stats:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    console(&shared.borrow());
    let sh = shared.borrow();
    let said = sh.all("shared: the card is exFAT");
    let state = retained.iter().find(|(t, _)| *t == state_topic).map(|x| x.1.clone()).unwrap_or_default();
    eprintln!("[exfat] the refusal said at {:?}; state: {}", said.iter().map(|&t| secs(t)).collect::<Vec<_>>(), &state[..state.len().min(600)]);
    let ours = device_writes(&sh.writes, &landings);
    let on_share: Vec<_> = walk(&r.smb.path());
    eprintln!("[exfat] the share holds {on_share:?}; oxidrive wrote {} blocks itself", ours.iter().map(|w| w.count).sum::<u64>());
    assert_eq!(said.len(), 1, "said once per boot");
    assert!(r.cs.now_ns >= said[0] + 120 * S, "two passes due after the first refusal ran in the run");
    assert!(state.contains("card is exFAT: not read"), "MQTT's shared.aborted: {state}");
    assert!(windows(&sh).is_empty(), "no window opened");
    assert!(on_share.iter().all(|p| p.starts_with('.') ), "nothing archived: {on_share:?}");
    assert!(ours.is_empty(), "oxidrive wrote no block: {ours:?}");
}

/// Every file under `dir`, relative.
fn walk(dir: &std::path::Path) -> Vec<String> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).into_iter().flatten().flatten() {
            let p = e.path();
            if p.is_dir() { stack.push(p); } else { out.push(p.strip_prefix(dir).unwrap().display().to_string()); }
        }
    }
    out.sort();
    out
}

/// (c) The window gate on exFAT (oxidrive_hazards' `external_oxidrive_refuses_car_writes_in_a_window`
/// with an exFAT car): the car records two saved events, idles until a window takes the card, and
/// starts a Sentry event the moment it does. Its first command, the boot sector written dirty at
/// once as 5.8 writes it, is refused with 2/3A/00; none of its writes reaches the card while
/// oxidrive holds it; the window archives the older event as the car wrote it (holding the newest
/// back) and finishes clean;
/// after it the car meets UNIT ATTENTION, reads the volume again and records; every closed clip is
/// on the card as written, fsck is clean and the car mounts the card again.
#[test]
#[ignore = "needs an oxidrive build with exFAT windows and nvs.csv under OXIDRIVE_EXFAT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and an exFAT checker"]
fn external_oxidrive_refuses_exfat_car_writes_in_a_window() {
    let mut r = exfat_rig(exfat_dir(), "exfat-gate.img");
    let shared = r.shared.clone();
    let wall = std::time::Instant::now();
    let (files, landings, gone_at, sentry, stats, rec, refused_in) = {
        let mut car = exfat_car(&mut r, PageCache::new, CarCfg::default());
        for m in 0..2 {
            let p = event_path("SavedClips", m);
            let st = car.record(&p, &mp4_bytes(&p, 2 << 20)).unwrap();
            assert!(matches!(st, Recorded::Closed { .. }), "{st:?}");
        }
        let taken = idle_until_line(&mut car, &shared, "supervisor: card taken for archiving", 0, 10 * MS, 300 * S).expect("no window within 300 s");
        let sentry = event_path("SentryClips", 2);
        let start = car.now();
        eprintln!("[exfat] window taken at {}; the car starts a Sentry clip at {}", secs(taken), secs(start));
        let rec = car.record(&sentry, &mp4_bytes(&sentry, 16 << 20));
        let refused_in = car.now() - start;
        eprintln!("[exfat] the Sentry clip: {rec:?} after {}", secs(refused_in));
        idle_until_line(&mut car, &shared, "supervisor: card returned to the car", start, 100 * MS, 500 * S).expect("the card never came back");
        car.idle(2 * S + 100 * MS).unwrap();
        while !car.present && car.now() < 500 * S { car.idle(100 * MS).unwrap(); }
        let gone_at = car.log.iter().find(|l| l.0 >= start && l.1.contains("medium gone")).map_or(u64::MAX, |l| l.0);
        assert!(car.present, "the medium never came back");
        car.idle(5 * S).unwrap();
        let after = event_path("SavedClips", 5);
        let st = car.record(&after, &mp4_bytes(&after, 2 << 20)).unwrap();
        assert!(matches!(st, Recorded::Closed { .. }), "the car records after the window: {st:?}");
        car.idle(4 * S).unwrap();
        car.unmount().unwrap();
        (car.files.clone(), car.disk.landings.clone(), gone_at, sentry, car.stats, rec, refused_in)
    };
    r.report(10);
    eprintln!("[exfat] {:.0} s wall for {} emulated; car: {stats:?}; the medium seen gone at {}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns), secs(gone_at));
    console(&shared.borrow());
    let sh = shared.borrow();
    let ws = windows(&sh);
    let (taken, returned) = *ws.first().expect("a window");
    let held = |t: u64| t > taken && t < returned;
    let into: Vec<&Landing> = landings.iter().filter(|l| held(l.at_ns)).collect();
    let on_card: u64 = sh.writes.iter().filter(|w| held(w.at_ns)).map(|w| w.count).sum();
    let fw = firmware_writes(&sh.writes, &landings, &ws, &r.card);
    eprintln!("[exfat] the window: {} to {}; the card took {on_card} blocks in it, oxidrive's own: {fw:?}", secs(taken), secs(returned));
    let ups = archived(&r.smb.path(), &files);
    for (p, v) in &ups { eprintln!("[exfat] {p}: {v:?}"); }
    let failed = sh.when("window: finished with failures").is_some_and(|t| (taken..=returned).contains(&t));
    let (clean, out) = match sd_image::fsck(&r.card) { Ok(o) => (true, o), Err(e) => (false, e) };
    eprintln!("[exfat] fsck: {}", out.trim().replace('\n', "; "));
    let (_, dirty, bad) = check_card(&r.card, &files);
    let mut d = FileDisk(std::fs::File::open(&r.card).unwrap());
    let refused_on_card = ExFat::mount(&mut d).unwrap().stat(&mut d, &sentry).unwrap_or(None);
    let refusal = match &rec { Ok(Recorded::WriteFailed { written, error }) => (*written, error.clone()), Err(e) => (0, e.clone()), Ok(r) => panic!("the Sentry clip was not refused: {r:?}") };
    assert!(refusal.1.contains("2/3a/00") && refusal.0 == 0 && refused_in < 100 * MS, "the car's first command after the take is refused: {refusal:?} after {}", secs(refused_in));
    assert!(into.is_empty() && on_card == 0, "none of the car's writes reached the card while oxidrive held it: {} landings, {on_card} blocks", into.len());
    assert!(refused_on_card.as_ref().is_none_or(|e| e.size == 0), "nothing of the refused clip on the card: {refused_on_card:?}");
    assert_eq!(ups.iter().find(|f| f.0 == event_path("SavedClips", 0)).map(|f| f.1.clone()), Some(Archived::Same), "the older event archived as the car wrote it (the newest is held back)");
    assert!(ups.iter().all(|(_, v)| matches!(v, Archived::Same | Archived::Absent)), "an upload differs from the car's file");
    assert!(!failed, "the window finished clean");
    assert!(stats.unit_attentions >= 1, "the car met UNIT ATTENTION after the window: {stats:?}");
    assert!(bad.is_empty() && !dirty, "every closed clip on the card, the volume clean: {bad:?}");
    assert!(clean, "fsck");
}

/// The car formats the card exFAT over USB, as exfatprogs' `mkfs.exfat` lays a volume out
/// (`sd_image::exfat`, `cluster` bytes) and in its order (mkfs/mkfs.c `make_exfat`): the MBR, the
/// volume's first sectors up to the root directory's cluster zeroed (the quick format's
/// `exfat_zero_out_disk`), then the main and backup boot regions, the FAT, the bitmap, the up-case
/// table and the root. How the car's own format writes is not known; this is mkfs's.
fn format_exfat(msc: &mut Msc<'_>, cluster: u32) {
    let bytes = msc.blocks * 512;
    let img = common::tmp("car-format.img");
    sd_image::exfat(&img, bytes, cluster, "").unwrap();
    let l = sd_image::exfat_layout(bytes, cluster);
    let mut src = FileDisk(std::fs::File::open(&img).unwrap());
    let part = sd_image::PARTITION_LBA;
    let end = part + l.heap_offset as u64 + (l.root_cluster as u64 - 1) * (cluster as u64 / 512);
    let mut put = |lba: u64, n: u64, zero: bool| {
        let mut b = vec![0u8; n as usize * 512];
        if !zero { src.read_blocks(lba, &mut b).unwrap(); }
        for (i, c) in b.chunks(64 << 10).enumerate() { msc.write10(lba + i as u64 * 128, c).unwrap_or_else(|e| panic!("format: WRITE(10) at {}: {e}", lba + i as u64 * 128)); }
    };
    put(0, 1, false);
    put(part, end - part, true);
    put(part, 12, false);
    put(part + 12, 12, false);
    put(part + l.fat_offset as u64, l.fat_length as u64, false);
    put(part + l.heap_offset as u64, end - part - l.heap_offset as u64, false);
    let _ = std::fs::remove_file(&img);
}

/// (e) The owner's procedure: the board runs with a FAT32 card, the car records on it, then
/// formats the card exFAT while the board runs, records on exFAT, and the next window archives
/// it. oxidrive reads the layout again when the car writes LBA 0 or the boot region and follows
/// it (`fs: the car rewrote the volume's layout`). Asserted: the firmware reports the volume
/// exFAT after the format; the next window archives the older exFAT event byte for byte, holding
/// the newest back, does not upload the FAT32 event the format erased, and finishes clean;
/// the car meets NOT READY then UNIT ATTENTION, records again and unmounts; fsck is clean and the
/// car mounts the card again.
#[test]
#[ignore = "needs an oxidrive build that follows a reformat (037647d or later) and nvs.csv under OXIDRIVE_EXFAT_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and an exFAT checker"]
fn external_oxidrive_archives_a_card_the_car_formats_exfat() {
    let prg = env_u64("OXIDRIVE_SD_PRG_US", 1500);
    let mut r = rig_with(RigOpts { card_name: "exfat-format.img".into(), card_bytes: env_u64("OXIDRIVE_CARD_MIB", 4096) << 20, car: true, prg_us: Some(prg), timed: Some(true), log_writes: true, ..RigOpts::new(exfat_dir()) });
    let shared = r.shared.clone();
    let card = r.card.clone();
    let deadline = env_u64("OXIDRIVE_SECONDS", 400) * S;
    let wall = std::time::Instant::now();
    let (files, landings, stats, formatted) = {
        let mut fat = Car::new(PageCache::new(drive(&mut r)), CarCfg { dirty_flag: true, ..CarCfg::default() });
        fat.echo = true;
        fat.mount().unwrap();
        let p = event_path("SavedClips", 0);
        assert!(matches!(fat.record(&p, &mp4_bytes(&p, 1 << 20)).unwrap(), Recorded::Closed { .. }));
        fat.unmount().unwrap();
        let mut msc = fat.disk.dev;
        let t0 = msc.host.now_ns();
        format_exfat(&mut msc, (env_u64("OXIDRIVE_EXFAT_CLUSTER_KIB", 32) << 10) as u32);
        let formatted = msc.host.now_ns();
        eprintln!("[exfat] the car formatted the card exFAT from {} to {}", secs(t0), secs(formatted));
        let mut car = exfat_car_on(msc, writeback, CarCfg::default());
        for m in [1u32, 2] {
            let p = event_path("SavedClips", m);
            let st = car.record(&p, &mp4_bytes(&p, 2 << 20)).unwrap();
            assert!(matches!(st, Recorded::Closed { .. }), "{p}: {st:?}");
        }
        let quiet = car.now();
        idle_until_line(&mut car, &shared, "supervisor: card returned to the car", quiet, 100 * MS, deadline).expect("no window after the format");
        car.idle(2 * S + 100 * MS).unwrap();
        while !car.present && car.now() < deadline { car.idle(100 * MS).unwrap(); }
        let p = event_path("SavedClips", 5);
        assert!(matches!(car.record(&p, &mp4_bytes(&p, 2 << 20)).unwrap(), Recorded::Closed { .. }), "the car records after the window");
        car.idle(40 * S).unwrap();
        car.unmount().unwrap();
        car.idle(2 * S).unwrap();
        (car.files.clone(), car.disk.landings.clone(), car.stats, formatted)
    };
    r.report(20);
    eprintln!("[exfat] {:.0} s wall for {} emulated; car: {stats:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    console(&shared.borrow());
    let sh = shared.borrow();
    let ws = windows(&sh);
    for (a, b) in &ws { eprintln!("[exfat] window {} to {}", secs(*a), secs(*b)); }
    let relayout: Vec<(u64, String)> = sh.lines.iter().filter(|l| l.1.contains("rewrote the volume's layout")).cloned().collect();
    for (t, l) in &relayout { eprintln!("[exfat] {}: {l}", secs(*t)); }
    let fw = firmware_writes(&sh.writes, &landings, &ws, &card);
    eprintln!("[exfat] oxidrive's own writes in its windows: {fw:?}");
    let ups = archived(&r.smb.path(), &files);
    for (p, v) in &ups { eprintln!("[exfat] {p}: {v:?}"); }
    let fsck = sd_image::fsck(&card);
    eprintln!("[exfat] fsck: {}", fsck.as_ref().unwrap_or_else(|e| e).trim().replace('\n', "; "));
    let (_, dirty, bad) = check_card(&card, &files);
    let failed = ws.iter().any(|&(a, b)| sh.when("window: finished with failures").is_some_and(|t| (a..=b).contains(&t)));
    assert!(relayout.last().is_some_and(|l| l.1.contains("ExFat") && l.0 >= formatted.saturating_sub(5 * S)), "the firmware follows the format to exFAT: {relayout:?}");
    assert!(ws.iter().any(|&(a, _)| a > formatted), "a window after the format: {ws:?}");
    assert_eq!(ups.iter().find(|f| f.0 == event_path("SavedClips", 1)).map(|f| f.1.clone()), Some(Archived::Same), "the older exFAT event archived as the car wrote it (the newest is held back)");
    assert!(!ups.iter().any(|f| f.0 == event_path("SavedClips", 0) && f.1 != Archived::Absent), "the FAT32 event the format erased is not archived");
    assert!(ups.iter().all(|(_, v)| matches!(v, Archived::Same | Archived::Absent)), "an upload differs from the car's file");
    assert!(!failed, "the window finished clean");
    assert!(stats.unit_attentions >= 1, "UNIT ATTENTION after the window: {stats:?}");
    assert!(bad.is_empty() && !dirty, "every closed clip on the card, the volume clean: {bad:?}");
    fsck.unwrap_or_else(|e| panic!("fsck: {e}"));
}

// ---- (d) the trace against the car's ground truth ----

/// One `sdtrace` line, and the file it came in.
#[derive(Clone, Debug)]
struct TraceWrite { file: String, seq: u64, t_ms: u64, lba: u64, count: u64 }

/// A boot's trace: its writes in time order, the boot-sector contents by (t_ms, lba), and each
/// file's `sdtrace-end` counts.
struct Trace { writes: Vec<TraceWrite>, bootw: HashMap<(u64, u64), Vec<u8>>, ends: Vec<String> }

/// The `sdtrace` and `sdtrace-bootw` lines of a boot's files (docs/trace-capture.md in oxidrive's
/// repository, "The file format, version 2"); a bootw is joined to its write on (t_ms, lba).
fn parse_trace(files: &[std::path::PathBuf]) -> Trace {
    let (mut writes, mut bootw, mut ends) = (Vec::new(), HashMap::new(), Vec::new());
    for f in files {
        let name = f.file_name().unwrap().to_string_lossy().to_string();
        for line in std::fs::read_to_string(f).unwrap().lines() {
            let c: Vec<&str> = line.split('\t').collect();
            let n = |i: usize| c.get(i).and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
            match c[0] {
                "sdtrace" => writes.push(TraceWrite { file: name.clone(), seq: n(1), t_ms: n(2), lba: n(3), count: n(4) }),
                "sdtrace-bootw" => { let hex = c.get(3).copied().unwrap_or(""); let b: Vec<u8> = (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap()).collect(); bootw.insert((n(1), n(2)), b); }
                "sdtrace-end" => ends.push(format!("{name}: {}", c[1..].join(" "))),
                _ => {}
            }
        }
    }
    writes.sort_by_key(|w| (w.t_ms, w.file.clone(), w.seq));
    Trace { writes, bootw, ends }
}

/// One line of `oxi trace --writes`: n, file (its index among the files given), seq, t_ms, lba,
/// count, service_us, regions, boot class, layout_t_ms.
#[derive(Clone, Debug)]
struct OxiWrite { file: String, seq: u64, t_ms: u64, lba: u64, count: u64, regions: String, boot: String }

fn oxi_writes(bin: &str, image: Option<&std::path::Path>, files: &[std::path::PathBuf]) -> Vec<OxiWrite> {
    let mut cmd = std::process::Command::new(bin);
    cmd.arg("trace").arg("--writes").arg("--wait").arg("0");
    if let Some(i) = image { cmd.arg("--image").arg(i); }
    cmd.args(files);
    let out = cmd.output().unwrap_or_else(|e| panic!("{bin}: {e}"));
    assert!(out.status.success(), "{bin} trace --writes: {}\n{}", out.status, String::from_utf8_lossy(&out.stderr));
    String::from_utf8_lossy(&out.stdout).lines().filter_map(|l| {
        let c: Vec<&str> = l.split('\t').collect();
        if c.len() < 9 || c[0].parse::<u64>().is_err() { return None; }
        // the file: its index among the files passed, or its path
        let name = |p: &std::path::Path| p.file_name().map_or(String::new(), |f| f.to_string_lossy().to_string());
        let file = c[1].parse::<usize>().ok().and_then(|i| files.get(i)).map_or_else(|| name(std::path::Path::new(c[1])), |p| name(p));
        Some(OxiWrite { file, seq: c[2].parse().ok()?, t_ms: c[3].parse().ok()?, lba: c[4].parse().ok()?, count: c[5].parse().ok()?, regions: c[7].to_string(), boot: c[8].to_string() })
    }).collect()
}

/// `fat:1,data:63` as region counts.
fn regions_of(s: &str) -> BTreeMap<String, u64> {
    s.split(',').filter(|x| !x.is_empty()).filter_map(|x| { let (r, n) = x.split_once(':')?; Some((r.to_string(), n.parse().ok()?)) }).collect()
}

/// (d) The SD_TRACE build records every write the car makes and uploads the trace to the share;
/// `oxi trace --writes` (`OXI_BIN`) classifies it. A writeback exFAT car records the Sentry-like
/// workload of `recorder::Sentry` (2 MiB clips, a set every 20 s) for `OXIDRIVE_SECONDS` (200),
/// writing all along so no window opens, and the run goes on until the trace has been flushed past
/// the car's last write. Every trace write is checked against the car's own writes and ground
/// truth (`writelog`): it lies within a write the car landed at that time, its regions are the
/// ones the car's layout gives its sectors, and a boot-sector write is a VolumeFlags change
/// (`state`) where the car changed only VolumeFlags. With `--image` the dir and data regions are
/// compared too; without it oxi cannot tell a directory cluster from data.
#[test]
#[ignore = "needs the oxidrive SD_TRACE build and nvs.csv under OXIDRIVE_EXFAT_TRACE_DIR, OXI_BIN (oxi trace from oxidrive's repository), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_exfat_trace_matches_the_cars_ground_truth() {
    let oxi = std::env::var("OXI_BIN").unwrap_or_else(|_| panic!("set OXI_BIN to the oxi CLI built from oxidrive's repository (cargo build -p oxidrive-cli)"));
    let mut r = exfat_rig(trace_dir(), "exfat-trace.img");
    let shared = r.shared.clone();
    let end = env_u64("OXIDRIVE_SECONDS", 200) * S;
    let share = r.smb.path();
    let wall = std::time::Instant::now();
    let traces = |share: &std::path::Path| -> Vec<std::path::PathBuf> { let mut v: Vec<_> = std::fs::read_dir(share.join("oxidrive-trace")).into_iter().flatten().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "tsv")).collect(); v.sort(); v };
    let (landings, meta, truth, stats, boot0, last_write) = {
        let mut car = exfat_car(&mut r, writeback, CarCfg::default());
        let mut boot0 = [0u8; 512];
        car.disk.dev.read_blocks(car.fs.as_ref().unwrap().layout.volume_lba, &mut boot0).unwrap();
        let mut sentry = Sentry::default();
        sentry.clip = (env_u64("OXIDRIVE_CLIP_MIB", 2) << 20) as usize;
        while car.now() < end { sentry.step(&mut car).unwrap(); }
        // everything lands, then the car keeps the card busy with small writes until a flush has
        // come after its last landing
        car.idle(40 * S).unwrap();
        let last_write = car.disk.landings.last().map_or(0, |l| l.at_ns);
        let mut k = 0;
        loop {
            let flushed = traces(&share).len();
            let t0 = car.now();
            let p = format!("TeslaCam/RecentClips/keepalive-{k}.mp4");
            car.record(&p, &mp4_bytes(&p, 32 << 10)).unwrap();
            car.disk.sync().unwrap();
            k += 1;
            car.idle(15 * S).unwrap();
            if traces(&share).len() > flushed && t0 > last_write + 60 * S || car.now() > end + 240 * S { break; }
        }
        let t = car.truth.lock().unwrap().clone();
        (car.disk.landings.clone(), car.disk.meta_landed.clone(), t, car.stats, boot0, last_write)
    };
    r.report(10);
    eprintln!("[exfat] {:.0} s wall for {} emulated; car: {stats:?}", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    console(&shared.borrow());
    let files = traces(&share);
    assert!(!files.is_empty(), "no trace reached the share");
    // `OXIDRIVE_EXFAT_OUT` keeps the trace and the card for `oxi trace` by hand; neither is to be committed
    if let Ok(out) = std::env::var("OXIDRIVE_EXFAT_OUT") {
        let out = std::path::Path::new(&out);
        std::fs::create_dir_all(out).unwrap();
        for f in &files { std::fs::copy(f, out.join(f.file_name().unwrap())).unwrap(); }
        std::fs::copy(&r.card, out.join("card.img")).unwrap();
        eprintln!("[exfat] the trace and the card kept in {}", out.display());
    }
    let Trace { writes: tw, bootw, ends } = parse_trace(&files);
    for e in &ends { eprintln!("[exfat] trace end: {e}"); }
    eprintln!("[exfat] {} trace files, {} writes, {} boot-sector contents; the car landed {} writes, its last at {}", files.len(), tw.len(), bootw.len(), landings.len(), secs(last_write));
    let layout = truth.layout.unwrap();
    // the firmware's clock against the emulator's: the median gap from a trace write to the car's
    // landing of the same blocks
    let mut by_lba: HashMap<u64, Vec<u64>> = HashMap::new();
    for l in &landings { by_lba.entry(l.lba).or_default().push(l.at_ns); }
    let mut gaps: Vec<i64> = tw.iter().filter_map(|w| by_lba.get(&w.lba).and_then(|v| v.iter().map(|&t| t as i64 - (w.t_ms * MS) as i64).min_by_key(|g| g.abs()))).collect();
    gaps.sort();
    let offset = gaps.get(gaps.len() / 2).copied().unwrap_or(0);
    eprintln!("[exfat] the emulator's clock is the firmware's plus {:.1} ms (median of {})", offset as f64 / 1e6, gaps.len());
    // each trace write within a car landing near its time
    let covered = |w: &TraceWrite| landings.iter().any(|l| l.lba <= w.lba && w.lba + w.count <= l.lba + l.count as u64 && ((l.at_ns as i64) - (w.t_ms * MS) as i64 - offset).abs() < 2 * S as i64);
    let stray: Vec<&TraceWrite> = tw.iter().filter(|w| !covered(w)).collect();
    eprintln!("[exfat] trace writes no car write accounts for: {}", stray.len());
    for w in stray.iter().take(10) { eprintln!("[exfat]   {w:?}"); }
    // the car's writes the trace should hold: those it landed before the last trace write
    let horizon = tw.last().map_or(0, |w| (w.t_ms * MS) as i64 + offset) as u64;
    let traced_sectors: u64 = tw.iter().map(|w| w.count).sum();
    let landed_sectors: u64 = landings.iter().filter(|l| l.at_ns + S < horizon).map(|l| l.count as u64).sum();
    eprintln!("[exfat] sectors: the trace {traced_sectors}, the car landed {landed_sectors} before {}", secs(horizon));
    // regions: oxi's against the car's layout, with and without the image
    let truth_regions = |w: &TraceWrite, dirs: bool| -> BTreeMap<String, u64> {
        let mut m = BTreeMap::new();
        for s in w.lba..w.lba + w.count {
            let r = if dirs { layout.region(s, &truth.root_clusters, &truth.dir_clusters) } else { layout.region(s, &truth.root_clusters, &BTreeMap::new()) };
            *m.entry(r.name().to_string()).or_insert(0) += 1;
        }
        m
    };
    let key: HashMap<(String, u64), &TraceWrite> = tw.iter().map(|w| ((w.file.clone(), w.seq), w)).collect();
    let image = snapshot(&r.card, "trace-image");
    let mut report = Vec::new();
    for (label, img) in [("without the image", None), ("with the image", Some(image.as_path()))] {
        let ox = oxi_writes(&oxi, img, &files);
        let mut mismatch = Vec::new();
        let mut boot_cmp = (0, 0);
        let mut missing = 0;
        for o in &ox {
            let Some(w) = key.get(&(o.file.clone(), o.seq)) else { missing += 1; continue };
            assert_eq!((o.t_ms, o.lba, o.count), (w.t_ms, w.lba, w.count), "oxi's write against the file's");
            let ours = truth_regions(w, img.is_some());
            let theirs = regions_of(&o.regions);
            // oxi without an image calls the heap's directory clusters data
            let ours = if img.is_none() { ours.into_iter().map(|(r, n)| (if r == "dir" { "data".to_string() } else { r }, n)).fold(BTreeMap::new(), |mut m, (r, n)| { *m.entry(r).or_insert(0) += n; m }) } else { ours };
            if ours != theirs { mismatch.push(format!("{} seq {} t={} lba {} +{}: oxi {} ours {:?}", o.file, o.seq, o.t_ms, o.lba, o.count, o.regions, ours)); }
            if o.boot != "-" {
                boot_cmp.0 += 1;
                let content = bootw.get(&(w.t_ms, layout.volume_lba));
                let want = content.map(|b| class_of(&boot0, &meta, b, layout.volume_lba, (w.t_ms * MS) as i64 + offset));
                if want.map(|c| c.name()) == Some(o.boot.as_str()) || (content.is_none() && o.boot == "no-content") { boot_cmp.1 += 1; } else { mismatch.push(format!("{} seq {} boot: oxi {} ours {:?}", o.file, o.seq, o.boot, want)); }
            }
        }
        eprintln!("[exfat] oxi {label}: {} writes, {} not in the files, {} differ from the ground truth; boot-sector writes {} ({} agree)", ox.len(), missing, mismatch.len(), boot_cmp.0, boot_cmp.1);
        for m in mismatch.iter().take(20) { eprintln!("[exfat]   {m}"); }
        report.push((label, ox.len(), missing, mismatch.len()));
    }
    // the car's ground truth for the record
    let w = writelog::exfat_writes(&truth, &landings, &meta, &boot0);
    let (dirty, clean, sync) = writelog::boot_writes(&w);
    let chained = truth.files.values().filter(|f| !f.dir && f.is_chained()).count();
    eprintln!("[exfat] ground truth: {} writes, boot sector {dirty} dirty + {clean} clean ({sync} at once), {chained} of {} files chained", w.len(), truth.files.values().filter(|f| !f.dir).count());
    assert!(stray.is_empty(), "every trace write is one the car made");
    for (label, n, missing, bad) in report { assert!(n == tw.len() && missing == 0 && bad == 0, "oxi {label}: {n} writes for the files' {}, {missing} unmatched, {bad} differ", tw.len()); }
}

/// What the car's write left in the boot sector against what was there before it: the car's
/// landed boot sectors in order, from the one it mounted.
fn class_of(boot0: &[u8; 512], meta: &[(u64, u64, Vec<u8>)], content: &[u8], vbr: u64, at_ns: i64) -> BootClass {
    let mut prev = boot0.to_vec();
    for (t, lba, d) in meta {
        if (*t as i64) >= at_ns - (S / 2) as i64 { break; }
        if *lba <= vbr && vbr < lba + d.len() as u64 / 512 { let o = (vbr - lba) as usize * 512; prev = d[o..o + 512].to_vec(); }
    }
    if (0..512).all(|i| matches!(i, 106 | 107 | 112) || prev[i] == content[i]) { BootClass::State } else { BootClass::Structural }
}
