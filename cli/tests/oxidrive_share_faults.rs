//! The owner's oxidrive firmware (not in this repository) against a share that misbehaves in one
//! way at a time: a manifest save that dies between setting the old copy aside and renaming the
//! new one in, a manifest too large to load or with a byte that is not UTF-8, a slow link, a NAS
//! that drops every session at its first request, an event directory it will not let the device
//! into, a rename whose reply is lost before an archived copy is damaged, and an SMB connect that
//! straddles a second boundary with libsmb2's timeout off. Each test prints what the firmware did
//! and asserts what a firmware that handles the fault does, so an older build shows its failure.
//! The firmware is `OXIDRIVE_DIR`; the scenario mirrors `smb_fault` in `oxidrive_services.rs`: a
//! car that keeps writing a RecentClips clip, so the card is never quiet and the firmware
//! archives in shared passes, a minute apart, the first forced over MQTT.
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
use esp_soc::usb_host::fat32::{clip_bytes, mp4_bytes, write_clip, Fat32};
use esp_soc::usb_host::msc::Msc;
use esp_soc::usb_host::UsbHost;
use oxidrive_rig::*;
use services::{smb2, Relay, Trap};
use std::cell::RefCell;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::time::Duration;

const S: u64 = 1_000_000_000;
const MS: u64 = 1_000_000;
const INDEX: &str = ".oxidrive-index.tsv";

fn crc32(data: &[u8]) -> u32 {
    !data.iter().fold(!0u32, |c, &b| (0..8).fold(c ^ b as u32, |c, _| if c & 1 != 0 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 }))
}
fn secs(ns: u64) -> String { format!("{:.3} s", ns as f64 / 1e9) }
fn stamp(i: usize) -> String { format!("2026-01-01_00-{i:02}-00") }
fn card_path(stamp: &str) -> String { format!("TeslaCam/SavedClips/{stamp}/event.mp4") }
fn share_path(stamp: &str) -> String { format!("SavedClips/{stamp}/event.mp4") }

/// The manifest's rows: path, size and checksum.
fn rows(text: &str) -> Vec<(String, u64, Option<u32>)> {
    text.lines().filter_map(|l| {
        let mut f = l.split('\t');
        let size = f.next()?.parse().ok()?;
        let path = f.next()?.to_string();
        Some((path, size, f.next().and_then(|c| u32::from_str_radix(c, 16).ok())))
    }).collect()
}
fn manifest(share: &Path) -> Option<String> { std::fs::read(share.join(INDEX)).ok().map(|b| String::from_utf8_lossy(&b).into_owned()) }

/// Each event the test records: its stamp and the clip's bytes.
struct Events { list: Vec<(String, Vec<u8>)> }
impl Events {
    fn new(n: usize, kib: usize) -> Events { Events { list: (0..n).map(|i| { let s = stamp(i); let c = mp4_bytes(&s, kib << 10); (s, c) }).collect() } }
    fn on_share(&self, share: &Path, i: usize) -> bool { std::fs::read(share.join(share_path(&self.list[i].0))).is_ok_and(|b| b == self.list[i].1) }
    fn archived(&self, share: &Path, range: std::ops::Range<usize>) -> usize { range.filter(|&i| self.on_share(share, i)).count() }
}

/// What a test sees and does each tenth of a second while the car records.
struct Tick<'a, 'p> { fs: &'a mut Fat32, msc: &'a mut Msc<'p>, now: u64 }
impl Tick<'_, '_> {
    fn record(&mut self, events: &Events, range: std::ops::Range<usize>) {
        for (stamp, clip) in &events.list[range.clone()] {
            let path = card_path(stamp);
            self.fs.mkdir_all(self.msc, path.rsplit_once('/').unwrap().0).unwrap();
            write_clip(self.fs, self.msc, &path, clip, 64 << 10, 0, 1).unwrap_or_else(|e| panic!("{path}: {e}"));
        }
        eprintln!("[faults] t={}: the car recorded events {range:?}", secs(self.msc.host.now_ns()));
        self.now = self.msc.host.now_ns();
    }
}

/// The car on the rig until `tick` returns false or `deadline_ns` passes, writing `recent` (a
/// RecentClips name of its own each time) 64 KiB every 500 ms.
fn drive(r: &mut Rig, recent: &str, deadline_ns: u64, mut tick: impl FnMut(&mut Tick) -> bool) {
    let mut host = UsbHost::new(r);
    host.enumerate(10 * S).unwrap_or_else(|e| panic!("enumerate: {e}"));
    let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
    msc.wait_ready(100 * MS, 10 * S).unwrap_or_else(|e| panic!("never ready: {e}"));
    msc.read_capacity().unwrap();
    let mut fs = Fat32::mount(&mut msc).unwrap();
    fs.mkdir_all(&mut msc, recent.rsplit_once('/').unwrap().0).unwrap();
    let mut rec = fs.create_path(&mut msc, recent).unwrap();
    let chunk = clip_bytes(recent, 64 << 10);
    let mut step = 0u64;
    while msc.host.now_ns() < deadline_ns {
        if step.is_multiple_of(5) { rec.write(&mut fs, &mut msc, &chunk).unwrap(); }
        step += 1;
        msc.host.idle_ns(100 * MS).unwrap();
        let now = msc.host.now_ns();
        if !tick(&mut Tick { fs: &mut fs, msc: &mut msc, now }) { break; }
    }
    rec.close(&mut fs, &mut msc).unwrap();
}

fn recent(n: usize) -> String { format!("TeslaCam/RecentClips/2026-01-01_01-00-0{n}-front.mp4") }

/// The P4 loses power and comes back, the car's host dropped first.
fn power_cut(r: &mut Rig) {
    esp_soc::SocBus::request_reset(&mut r.cs.p4.bus, esp_periph::RST_POWERON);
    let t = r.cs.now_ns + 10 * MS;
    r.run_to(t);
}

/// The P4's console from line `from` on, with times.
fn console(shared: &Rc<RefCell<Shared>>, from: usize) -> Vec<(u64, String)> { shared.borrow().lines[from..].to_vec() }
fn count(lines: &[(u64, String)], what: &str) -> usize { lines.iter().filter(|l| l.1.contains(what)).count() }
fn times(lines: &[(u64, String)], what: &str) -> Vec<u64> { lines.iter().filter(|l| l.1.contains(what)).map(|l| l.0).collect() }
fn show(lines: &[(u64, String)], keys: &[&str]) {
    for l in lines.iter().filter(|l| keys.iter().any(|k| l.1.contains(k))) { eprintln!("[faults]   {:8.3} {}", l.0 as f64 / 1e9, l.1); }
}

/// The firmware's console lines that say what a pass did, for every test.
const PASS_LINES: [&str; 9] = ["scan: shared", "shared: ", "archiver: ", "index: ", "smb: connected", "refused", "deferring", "does not match", "WARN"];

/// A rig with the share behind a relay, and what a test needs while the car holds it.
struct Setup { r: Rig, relay: Relay, shared: Rc<RefCell<Shared>>, share: PathBuf, cmd: String, mqtt: services::MqttClient }
fn setup(card: &str, shape: impl FnOnce(&mut RigOpts)) -> Setup {
    let mut o = RigOpts { card_name: card.into(), car: true, smb_relay: true, ..RigOpts::new(oxidrive_board::dir()) };
    shape(&mut o);
    eprintln!("[faults] firmware {}", o.dir);
    let r = rig_with(o);
    let relay = r.smb_relay.clone().unwrap();
    let (shared, share, cmd, mqtt) = (r.shared.clone(), r.smb.path(), format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    Setup { r, relay, shared, share, cmd, mqtt }
}

/// The retained state document's top-level number `key`, if it is there. The firmware publishes
/// it only once the card has been quiet for 2 s, so the car must have stopped.
fn state_num(s: &Setup, key: &str) -> Option<i64> {
    let topic = format!("oxidrive/{}/state", s.r.id);
    let doc = s.r.mqtt.client.retained(&[&topic], Duration::from_secs(1)).ok()?.into_iter().next()?.1;
    let j = esp_soc::json::parse_json(&doc).ok()?;
    j.get(key)?.as_i64()
}

/// A manifest save that dies at the rename that puts the new manifest in place, then a power
/// cut before any later save. After a clean pass archives two events and saves their checksums,
/// the relay ends the connection on the rename onto `.oxidrive-index.tsv` without passing it on,
/// once; the P4 then loses power. The state on the share is what a refused rename leaves, which
/// samba cannot be made to produce on its own: it refuses that rename only by refusing the
/// unlink before it too. After the reboot every event archived before the cut must have its
/// checksum in the manifest. A firmware that removed the manifest before the rename, and took
/// an absent manifest as loaded, saves one with none.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_keeps_its_checksums_when_a_manifest_save_dies_at_the_rename() {
    let mut s = setup("oxidrive-manifest-rename.img", |_| {});
    let ev = Events::new(6, 1536);
    let (relay, shared, share, cmd, mqtt) = (s.relay.clone(), s.shared.clone(), s.share.clone(), s.cmd.clone(), s.mqtt.clone());
    let (mut forced, mut armed, mut saved_at) = (false, None, None);
    drive(&mut s.r, &recent(0), 300 * S, |t| {
        if !forced && shared.borrow().when("session ready").is_some() {
            t.record(&ev, 0..3);
            mqtt.publish(&cmd, "force_open").unwrap();
            forced = true;
        }
        let saved = manifest(&share).map(|m| rows(&m).iter().filter(|r| r.2.is_some() && !r.0.ends_with(".part")).count()).unwrap_or(0);
        if saved_at.is_none() && saved >= 2 { saved_at = Some(t.now); }
        // Armed once the pass that saved them has ended, so the trap meets the next pass's save.
        let ended = saved_at.is_some_and(|at| shared.borrow().all("scan: shared").iter().any(|&p| p >= at));
        if forced && armed.is_none() && ended {
            relay.trap(Trap { pred: Box::new(smb2::renames_to(INDEX)), after: None, times: 1 });
            t.record(&ev, 3..6);
            armed = Some(t.now);
            eprintln!("[faults] t={}: {saved} checksums saved; the manifest's rename is trapped", secs(t.now));
        }
        relay.sprung() == 0
    });
    assert_eq!(relay.sprung(), 1, "the trap never fired");
    let cut_at = s.r.cs.now_ns;
    let before: Vec<usize> = (0..6).filter(|&i| ev.on_share(&share, i)).collect();
    let on_share: Vec<String> = std::fs::read_dir(&share).unwrap().flatten().map(|e| e.file_name().to_string_lossy().into_owned()).filter(|n| n.starts_with(".oxidrive")).collect();
    eprintln!("[faults] t={}: the save died at its rename; events on the share {before:?}; manifest files {on_share:?}; power cut", secs(cut_at));
    relay.untrap();
    let mark = shared.borrow().lines.len();
    power_cut(&mut s.r);
    let mut settled = None;
    drive(&mut s.r, &recent(1), cut_at + 300 * S, |t| {
        let sh = shared.borrow();
        let after = &sh.lines[mark..];
        let passes = after.iter().filter(|l| l.1.contains("scan: shared")).count();
        if settled.is_none() && passes >= 2 && ev.archived(&share, 0..5) == 5 { settled = Some(t.now); }
        settled.is_none_or(|at| t.now < at + 5 * S)
    });
    let lines = console(&shared, 0);
    show(&lines, &PASS_LINES);
    let text = manifest(&share).unwrap_or_default();
    let have = rows(&text);
    eprintln!("[faults] the manifest after the reboot:\n{text}");
    let mut missing = Vec::new();
    for &i in &before {
        let path = share_path(&ev.list[i].0);
        let crc = have.iter().find(|r| r.0 == path).and_then(|r| r.2);
        eprintln!("[faults] {path}: crc {crc:08x?}, want {:08x}", crc32(&ev.list[i].1));
        if crc != Some(crc32(&ev.list[i].1)) { missing.push(path); }
    }
    s.r.report(10);
    assert!(before.len() >= 3, "fewer than three events archived before the cut: {before:?}");
    assert!(missing.is_empty(), "checksums lost across the reboot: {missing:?}");
}

/// A manifest on the share is read, merged and saved back; one the firmware cannot use must not
/// cost its rows. Boots against a share holding `index`, archives two events, and returns the
/// manifest as it is after two more passes, and the console.
fn with_manifest(card: &str, index: &[u8]) -> (Vec<u8>, Vec<(u64, String)>) {
    let mut s = setup(card, |_| {});
    std::fs::write(s.share.join(INDEX), index).unwrap();
    let ev = Events::new(3, 1536);
    let (shared, share, cmd, mqtt) = (s.shared.clone(), s.share.clone(), s.cmd.clone(), s.mqtt.clone());
    let mut forced = false;
    drive(&mut s.r, &recent(0), 300 * S, |t| {
        if !forced && shared.borrow().when("session ready").is_some() {
            t.record(&ev, 0..3);
            mqtt.publish(&cmd, "force_open").unwrap();
            forced = true;
        }
        !(shared.borrow().all("scan: shared").len() >= 3 && ev.archived(&share, 0..3) == 3)
    });
    let lines = console(&shared, 0);
    show(&lines, &PASS_LINES);
    assert_eq!(ev.archived(&share, 0..3), 3, "the events did not archive");
    (std::fs::read(share.join(INDEX)).unwrap_or_default(), lines)
}

/// Rows for clips long gone from the card, as months of archiving leave.
fn old_rows(n: usize) -> String {
    (0..n).map(|i| format!("{}\tSentryClips/2025-{:02}-{:02}_{:02}-{:02}-00/event-{i}.mp4\t{:08x}\n", 1000 + i, 1 + i % 12, 1 + i % 28, i % 24, i % 60, crc32(&i.to_le_bytes()))).collect()
}

/// A manifest larger than the firmware loads (4 MiB) is neither loaded nor saved over: the
/// events still archive, and the file is byte for byte what it was.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_does_not_save_over_a_manifest_too_large_to_load() {
    let mut text = old_rows(60_000);
    while text.len() <= 4 << 20 { text.push_str(&old_rows(1000)); }
    let (after, _) = with_manifest("oxidrive-manifest-large.img", text.as_bytes());
    eprintln!("[faults] the manifest was {} bytes and is {} bytes", text.len(), after.len());
    assert!(after == text.as_bytes(), "the manifest was saved over: {} bytes now", after.len());
}

/// A manifest with one row that is not UTF-8 keeps every other row through the next save.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_keeps_the_rows_of_a_manifest_with_a_bad_byte() {
    let good = old_rows(500);
    let mut text = good.as_bytes().to_vec();
    text.extend_from_slice(b"1234\tSentryClips/2025-12-31_23-59-00/\xff\xfe-front.mp4\t0badf00d\n");
    text.extend_from_slice(old_rows(10).replace("2025-", "2024-").as_bytes());
    let (after, _) = with_manifest("oxidrive-manifest-bad-byte.img", &text);
    let have = rows(&String::from_utf8_lossy(&after));
    let want = rows(&good);
    let kept = want.iter().filter(|w| have.iter().any(|h| h.0 == w.0 && h.2 == w.2)).count();
    eprintln!("[faults] {kept} of {} rows kept; the manifest has {} rows", want.len(), have.len());
    assert_eq!(kept, want.len(), "rows lost");
}

/// A slow link: the share at `OXIDRIVE_SLOW_RATE` (100 kB/s each way, in emulated time) and an
/// event of `OXIDRIVE_SLOW_MIB` (20) MiB, then a small one, which goes first. The large one's
/// transfer spans several passes, and the read-back that a resumed part always gets digests the
/// whole clip in one SMB call, about 210 s at this rate: longer than the 120 s a call may go
/// without an answer. It must finish, and the event reach the share byte for byte with its
/// checksum in the manifest.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_verifies_a_large_clip_over_a_slow_link() {
    let rate = Rig::env_secs("OXIDRIVE_SLOW_RATE", 100_000);
    let mib = Rig::env_secs("OXIDRIVE_SLOW_MIB", 20) as usize;
    let mut s = setup("oxidrive-slow.img", |o| o.smb_rate = Some(rate));
    let mut ev = Events::new(2, mib << 10);
    ev.list[1].1 = mp4_bytes(&ev.list[1].0, 1536 << 10);
    let (shared, share, cmd, mqtt) = (s.shared.clone(), s.share.clone(), s.cmd.clone(), s.mqtt.clone());
    let deadline = Rig::env_secs("OXIDRIVE_SECONDS", 900) * S;
    let mut forced = false;
    let mut done = None;
    drive(&mut s.r, &recent(0), deadline, |t| {
        if !forced && shared.borrow().when("session ready").is_some() {
            t.record(&ev, 0..2);
            mqtt.publish(&cmd, "force_open").unwrap();
            forced = true;
        }
        if done.is_none() && ev.on_share(&share, 0) { done = Some(t.now); }
        done.is_none_or(|d| t.now < d + 30 * S)
    });
    let lines = console(&shared, 0);
    show(&lines, &PASS_LINES);
    let text = manifest(&share).unwrap_or_default();
    let crc = rows(&text).into_iter().find(|r| r.0 == share_path(&ev.list[0].0)).and_then(|r| r.2);
    let stuck = count(&lines, "has not answered");
    eprintln!("[faults] {} MiB at {rate} B/s: on the share {}, crc {crc:08x?}; {stuck} calls given up as stuck; {} lost sessions; {} late-reply warnings",
              mib, done.map_or("never".into(), secs), count(&lines, "archiver: connection lost"), count(&lines, "waiting on a reply"));
    s.r.report(10);
    assert_eq!(stuck, 0, "a call was given up as stuck");
    assert!(done.is_some(), "the event never reached the share");
    assert_eq!(crc, Some(crc32(&ev.list[0].1)), "no checksum for it");
}

/// A NAS that drops every session at its first request: after a clean pass, the relay ends each
/// connection at its first CREATE (`OXIDRIVE_DROP_SECS`, 300 s), which is a pass's first stat
/// of a clip. Nothing of any clip is sent, so no clip may collect strikes or be deferred, and
/// the passes must space out. Once the NAS behaves, every event archives.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_backs_off_from_a_nas_that_drops_every_session() {
    let mut s = setup("oxidrive-drops.img", |_| {});
    let ev = Events::new(6, 1536);
    let (relay, shared, share, cmd, mqtt) = (s.relay.clone(), s.shared.clone(), s.share.clone(), s.cmd.clone(), s.mqtt.clone());
    let hold = Rig::env_secs("OXIDRIVE_DROP_SECS", 300) * S;
    let (mut forced, mut armed, mut released, mut done) = (false, None, None, None);
    drive(&mut s.r, &recent(0), Rig::env_secs("OXIDRIVE_SECONDS", 900) * S, |t| {
        if !forced && shared.borrow().when("session ready").is_some() {
            t.record(&ev, 0..2);
            mqtt.publish(&cmd, "force_open").unwrap();
            forced = true;
        }
        // Once a pass has uploaded, so the manifest is loaded and a pass's first request is a
        // clip's stat.
        let uploaded = shared.borrow().lines.iter().any(|l| l.1.contains("oxidrive::archiver: shared: ") && !l.1.contains("shared: 0 uploaded"));
        if forced && armed.is_none() && uploaded {
            relay.trap(Trap { pred: Box::new(smb2::is_create), after: None, times: u32::MAX });
            t.record(&ev, 2..6);
            armed = Some(t.now);
            eprintln!("[faults] t={}: every session now ends at its first CREATE", secs(t.now));
        }
        if let (Some(a), None) = (armed, released) {
            if t.now >= a + hold {
                relay.untrap();
                released = Some(t.now);
                eprintln!("[faults] t={}: the NAS behaves again", secs(t.now));
            }
        }
        if released.is_some() && done.is_none() && ev.archived(&share, 0..5) == 5 { done = Some(t.now); }
        done.is_none_or(|d| t.now < d + 2 * S)
    });
    let (armed, released) = (armed.expect("never armed"), released.expect("never released"));
    let lines = console(&shared, 0);
    show(&lines, &PASS_LINES);
    let during: Vec<(u64, String)> = lines.iter().filter(|l| l.0 >= armed && l.0 < released).cloned().collect();
    let lost = times(&during, "archiver: connection lost");
    let gaps: Vec<f64> = lost.windows(2).map(|w| (w[1] - w[0]) as f64 / 1e9).collect();
    let deferred = count(&lines, "lost sessions in a row; deferring it");
    eprintln!("[faults] while the NAS dropped sessions: {} passes lost their session (gaps {gaps:.0?} s), {} connections ended by the relay; {deferred} clips deferred on strikes; all archived {}",
              lost.len(), relay.sprung(), done.map_or("never".into(), secs));
    s.r.report(10);
    assert_eq!(deferred, 0, "a clip deferred on strikes it never earned");
    assert!(gaps.iter().any(|&g| g > 100.0), "the passes did not space out: {gaps:?}");
    assert!(done.is_some(), "not every event archived once the NAS behaved");
}

/// One event directory on the share the server will not let the device into (mode 000), made
/// before the event is recorded: every stat of its clip is refused. The other events archive;
/// the refused one is tried no more than three passes running before it waits out a cooldown;
/// and the retry backlog that keeps the car awake (`backlog_retry`) does not count it.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_waits_out_a_clip_whose_stat_is_refused() {
    use std::os::unix::fs::PermissionsExt;
    let mut s = setup("oxidrive-refused-stat.img", |_| {});
    let ev = Events::new(4, 1536);
    let locked = s.share.join(share_path(&ev.list[0].0)).parent().unwrap().to_path_buf();
    std::fs::create_dir_all(&locked).unwrap();
    std::fs::set_permissions(&locked, std::fs::Permissions::from_mode(0o000)).unwrap();
    let (shared, share, cmd, mqtt) = (s.shared.clone(), s.share.clone(), s.cmd.clone(), s.mqtt.clone());
    let mut forced = false;
    let passes = Rig::env_secs("OXIDRIVE_REFUSED_PASSES", 6) as usize;
    drive(&mut s.r, &recent(0), Rig::env_secs("OXIDRIVE_SECONDS", 600) * S, |t| {
        if !forced && shared.borrow().when("session ready").is_some() {
            t.record(&ev, 0..4);
            mqtt.publish(&cmd, "force_open").unwrap();
            forced = true;
        }
        shared.borrow().all("scan: shared").len() < passes
    });
    let t = s.r.cs.now_ns + 10 * S;
    s.r.run_to(t);
    std::fs::set_permissions(&locked, std::fs::Permissions::from_mode(0o755)).unwrap();
    let lines = console(&shared, 0);
    show(&lines, &PASS_LINES);
    let refused = count(&lines, &format!("stat {} refused", share_path(&ev.list[0].0)));
    let deferred = count(&lines, "refused 3 passes in a row; deferring it");
    let (backlog, retry) = (state_num(&s, "backlog"), state_num(&s, "backlog_retry"));
    eprintln!("[faults] over {} passes: its stat refused {refused} times, deferred {deferred} times; {} of the other 3 archived; state backlog {backlog:?}, backlog_retry {retry:?}",
              count(&lines, "scan: shared"), ev.archived(&share, 1..4));
    s.r.report(10);
    assert_eq!(ev.archived(&share, 1..4), 3, "the other events did not archive");
    assert!(refused <= 3, "its stat was refused {refused} times in {passes} passes");
    assert_eq!(retry, Some(0), "the retry backlog counts it");
    assert!(backlog.is_some_and(|b| b >= 1), "the backlog lost it: {backlog:?}");
}

/// A rename that lands with its reply lost, and the archived copy then damaged on the share: the
/// relay passes the event's rename on and closes the connection before the reply, so the
/// firmware finds the clip archived with no checksum of its own and its part's checksum in
/// memory. A byte of the archived copy is flipped before the next pass. The firmware must find
/// the copy does not match, remove it, and send the clip again, so it ends byte for byte.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_replaces_an_archived_copy_that_does_not_match() {
    let mut s = setup("oxidrive-adopt.img", |_| {});
    let ev = Events::new(3, 1536);
    let (relay, shared, share, cmd, mqtt) = (s.relay.clone(), s.shared.clone(), s.share.clone(), s.cmd.clone(), s.mqtt.clone());
    let target = share_path(&ev.list[0].0);
    let (mut forced, mut flipped, mut done) = (false, None, None);
    relay.trap(Trap { pred: Box::new(smb2::renames_to(&target)), after: Some(Duration::from_millis(500)), times: 1 });
    drive(&mut s.r, &recent(0), Rig::env_secs("OXIDRIVE_SECONDS", 400) * S, |t| {
        if !forced && shared.borrow().when("session ready").is_some() {
            t.record(&ev, 0..3);
            mqtt.publish(&cmd, "force_open").unwrap();
            forced = true;
        }
        if flipped.is_none() && relay.sprung() == 1 {
            let p = share.join(&target);
            if let Ok(mut b) = std::fs::read(&p) {
                if b.len() == ev.list[0].1.len() {
                    let at = b.len() / 2;
                    b[at] ^= 0x01;
                    std::fs::write(&p, &b).unwrap();
                    flipped = Some(t.now);
                    eprintln!("[faults] t={}: the rename landed, its reply was lost; byte {at} of the archived copy flipped", secs(t.now));
                }
            }
        }
        if flipped.is_some() && done.is_none() && ev.archived(&share, 0..2) == 2 && manifest(&share).is_some_and(|m| rows(&m).iter().any(|r| r.0 == target && r.2.is_some())) { done = Some(t.now); }
        done.is_none_or(|d| t.now < d + 2 * S)
    });
    let lines = console(&shared, 0);
    show(&lines, &PASS_LINES);
    let crc = manifest(&share).and_then(|m| rows(&m).into_iter().find(|r| r.0 == target)).and_then(|r| r.2);
    eprintln!("[faults] flipped {}; the copy matches the card {}; crc {crc:08x?} (want {:08x}); {} mismatch lines",
              flipped.map_or("never".into(), secs), ev.on_share(&share, 0), crc32(&ev.list[0].1), count(&lines, "does not match its part's checksum"));
    s.r.report(10);
    assert!(flipped.is_some(), "the rename's reply was never lost");
    assert!(ev.on_share(&share, 0), "the damaged copy is still on the share");
    assert_eq!(crc, Some(crc32(&ev.list[0].1)));
}

/// libsmb2's timeout off (a build with `SMB_TIMEOUT=0`) and an SMB handshake that takes
/// `OXIDRIVE_CONNECT_DELAY_MS` (350) ms of emulated time, so about a third of connects straddle
/// a second boundary; the firmware's NAS probe gives up at 400 ms, which bounds the delay. The P4
/// boots `OXIDRIVE_CONNECTS` (10) times, a power cut after each boot's first connect, so every
/// connect is a first one. Every connect must succeed. The first boot's can also meet the
/// clock's first SNTP step, which libsmb2's connect wait reads as its timeout passing.
#[test]
#[ignore = "needs an oxidrive build with SMB_TIMEOUT=0 (images and nvs.csv under OXIDRIVE_DIR), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_connects_with_its_request_timeout_off() {
    let delay = Rig::env_secs("OXIDRIVE_CONNECT_DELAY_MS", 350);
    let boots = Rig::env_secs("OXIDRIVE_CONNECTS", 10) as usize;
    let mut s = setup("oxidrive-no-timeout.img", |o| o.smb_connect_delay_ms = delay);
    let ev = Events::new(2, 512);
    let (shared, cmd, mqtt) = (s.shared.clone(), s.cmd.clone(), s.mqtt.clone());
    let mut tries: Vec<(u64, bool, String)> = Vec::new();
    for boot in 0..boots {
        let mark = shared.borrow().lines.len();
        let mut forced = false;
        let deadline = s.r.cs.now_ns + 120 * S;
        drive(&mut s.r, &recent(boot), deadline, |t| {
            let sh = shared.borrow();
            let after = &sh.lines[mark..];
            if !forced && after.iter().any(|l| l.1.contains("session ready")) {
                drop(sh);
                if boot == 0 { t.record(&ev, 0..2); }
                mqtt.publish(&cmd, "force_open").unwrap();
                forced = true;
                return true;
            }
            !after.iter().any(|l| l.1.contains("smb: connected to") || l.1.contains("archiver: no SMB session"))
        });
        let first = shared.borrow().lines[mark..].iter().find(|l| l.1.contains("smb: connected to") || l.1.contains("archiver: no SMB session")).cloned();
        let Some((at, line)) = first else { panic!("boot {boot}: no connect within 120 s") };
        let ok = line.contains("smb: connected to");
        eprintln!("[faults] boot {boot}: {} at {}: {}", if ok { "connected" } else { "failed" }, secs(at), line.split("; ").last().unwrap_or(""));
        tries.push((at, ok, line));
        power_cut(&mut s.r);
    }
    let failed = tries.iter().filter(|t| !t.1).count();
    eprintln!("[faults] {delay} ms handshakes, libsmb2's timeout off: {} of {boots} first connects failed", failed);
    s.r.report(10);
    assert_eq!(failed, 0, "connects failed");
}
