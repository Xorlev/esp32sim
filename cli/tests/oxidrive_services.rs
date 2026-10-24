//! The owner's oxidrive firmware (not in this repository) against real local services: an MQTT
//! broker and an SMB server (`services`), reached from the P4 through the C6's radio, the virtual
//! access point and forwards of the virtual network (`--net none,fwd=...`). The fixture's own tests
//! run without firmware or servers.
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
use common::*;
use oxidrive_rig::*;
use std::time::Duration;

/// Goal 1 of milestone 4b: oxidrive's MQTT session comes up against the broker and it publishes
/// its retained state and availability; its NAS probe finds the SMB server. Waits up to
/// `OXIDRIVE_SECONDS` (40) of emulated time.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_meets_its_broker_and_its_nas() {
    let mut r = rig("oxidrive-services.img", 4 << 30, false);
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 40) * 1_000_000_000;
    let t0 = std::time::Instant::now();
    let (avail, state) = (format!("oxidrive/{}/avail", r.id), format!("oxidrive/{}/state", r.id));
    let mut retained = Vec::new();
    while r.cs.now_ns < end {
        let next = r.cs.now_ns + 1_000_000_000;
        r.run_to(next);
        if r.p4_has("session ready") {
            retained = r.mqtt.client.retained(&[&avail, &state], Duration::from_secs(1)).unwrap();
            if retained.iter().any(|(t, _)| *t == state) { break; }
        }
    }
    r.report(30);
    eprintln!("[oxidrive] {:.1} s wall", t0.elapsed().as_secs_f64());
    for (t, p) in &retained { eprintln!("[oxidrive] retained {t}: {}", &p[..p.len().min(400)]); }
    let on_share: Vec<_> = std::fs::read_dir(r.smb.path()).unwrap().flatten().map(|e| e.file_name()).collect();
    eprintln!("[oxidrive] the share holds {on_share:?}; the card is {}", r.card.display());
    assert!(r.p4_has("oxidrive::net::mqtt: session up"), "no MQTT session");
    assert!(r.p4_has("senses: nas_reachable=true"), "the NAS probe did not reach the SMB server");
    assert!(retained.iter().any(|(t, p)| *t == avail && p == "online"), "no retained {avail} = online");
    assert!(retained.iter().any(|(t, p)| *t == state && p.starts_with('{')), "no retained {state} document");
}

/// Saved events as the car names them; the last is recorded after the window.
const EVENTS: [&str; 3] = ["2026-01-01_00-00-00", "2026-01-01_00-01-00", "2026-01-01_00-05-00"];
fn event_clip(stamp: &str) -> String { format!("TeslaCam/SavedClips/{stamp}/event.mp4") }

/// Goal 2 of milestone 4b: an archive window end to end. The car records two saved events
/// (`event.mp4` each, `OXIDRIVE_CLIP_MIB` MiB, a 4.1 MB/s camera) and then idles, polling TEST
/// UNIT READY every 2 s as udev polls a removable disk. oxidrive's own policy opens the window
/// once the card has been quiet for a minute (`OXIDRIVE_FORCE_OPEN=1` publishes `force_open`
/// instead); the car sees MEDIUM NOT PRESENT, oxidrive uploads, renames and writes its manifest,
/// and gives the card back; the car reads the volume again and records a third event. Then: every
/// clip read back over USB and off the image, fsck on the image, and every upload on the share
/// compared byte for byte with the card.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba, mosquitto and a FAT checker"]
fn external_oxidrive_archives_a_window() {
    use esp_soc::usb_host::{fat32::{mp4_bytes, write_clip, Fat32}, msc::{Msc, MscError}, UsbHost};
    let mut r = rig("oxidrive-window.img", 4 << 30, true);
    let mib = Rig::env_secs("OXIDRIVE_CLIP_MIB", 2);
    // the newest of the first two is held back for one pass, so the window is the one to take it
    let window_mib = Rig::env_secs("OXIDRIVE_WINDOW_CLIP_MIB", mib);
    let force = std::env::var("OXIDRIVE_FORCE_OPEN").is_ok_and(|v| v == "1");
    let deadline = Rig::env_secs("OXIDRIVE_SECONDS", 200) * 1_000_000_000;
    let clips: Vec<Vec<u8>> = EVENTS.iter().enumerate().map(|(i, s)| mp4_bytes(s, ((if i == 1 { window_mib } else { mib }) << 20) as usize)).collect();
    let gap_ns = 16_000_000;
    let (cmd, mqtt) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    let wall = std::time::Instant::now();
    let (mut gone_at, mut back_at) = (None, None);
    {
        let mut host = UsbHost::new(&mut r);
        host.log = true;
        host.enumerate(10_000_000_000).unwrap_or_else(|e| panic!("enumerate: {e}"));
        let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
        msc.wait_ready(100_000_000, 10_000_000_000).unwrap_or_else(|e| panic!("never ready: {e}"));
        msc.read_capacity().unwrap();
        let mut fs = Fat32::mount(&mut msc).unwrap();
        for (stamp, clip) in EVENTS[..2].iter().zip(&clips) {
            let path = event_clip(stamp);
            fs.mkdir_all(&mut msc, path.rsplit_once('/').unwrap().0).unwrap();
            let st = write_clip(&mut fs, &mut msc, &path, clip, 64 << 10, gap_ns, 1).unwrap_or_else(|e| panic!("{path}: {e}"));
            eprintln!("[oxidrive] car: {path}: {}", st.summary(gap_ns));
        }
        let quiet_from = msc.host.now_ns();
        while msc.host.now_ns() < deadline {
            let now = msc.host.now_ns();
            // a media poll as Linux's sd_check_events makes one: the flag a UNIT ATTENTION set is taken
            let ready = msc.test_unit_ready();
            msc.take_changed();
            match ready {
                Ok(()) if gone_at.is_some() => { back_at = Some(now); break; }
                Ok(()) => {}
                Err(MscError::Check(s)) if gone_at.is_none() => { eprintln!("[oxidrive] car: t={:.3} TEST UNIT READY: {s}", now as f64 / 1e9); gone_at = Some(now); }
                Err(MscError::Check(_)) => {}
                Err(e) => panic!("the car's poll failed: {e}"),
            }
            if force && gone_at.is_none() && now > quiet_from + 15_000_000_000 { mqtt.publish(&cmd, "force_open").unwrap(); }
            msc.host.idle_ns(2_000_000_000).unwrap();
        }
        let (gone, back) = (gone_at.expect("the car never saw the medium go"), back_at.expect("the medium never came back"));
        eprintln!("[oxidrive] car: the medium went at {:.3} s and was back at {:.3} s", gone as f64 / 1e9, back as f64 / 1e9);
        // a medium change: the car reads the volume again before it writes
        msc.read_capacity().unwrap();
        let mut fs = Fat32::mount(&mut msc).unwrap();
        let path = event_clip(EVENTS[2]);
        fs.mkdir_all(&mut msc, path.rsplit_once('/').unwrap().0).unwrap();
        let st = write_clip(&mut fs, &mut msc, &path, &clips[2], 64 << 10, gap_ns, 1).unwrap_or_else(|e| panic!("{path}: {e}"));
        eprintln!("[oxidrive] car: {path}: {}", st.summary(gap_ns));
        for (stamp, clip) in EVENTS.iter().zip(&clips) {
            assert!(fs.read_path(&mut msc, &event_clip(stamp)).unwrap() == *clip, "{stamp}: the clip read back over USB differs");
        }
        let end = msc.host.now_ns() + 4_000_000_000;
        while msc.host.now_ns() < end { msc.test_unit_ready().unwrap(); msc.host.idle_ns(2_000_000_000).unwrap(); }
    }
    r.report(60);
    eprintln!("[oxidrive] {:.1} s wall for {:.1} s emulated", wall.elapsed().as_secs_f64(), r.cs.now_ns as f64 / 1e9);
    let took = r.lines.iter().find(|l| !l.1 && l.2.contains("supervisor: card taken for archiving")).map(|l| l.0);
    let returned = r.lines.iter().find(|l| !l.1 && l.2.contains("supervisor: card returned to the car")).map(|l| l.0);
    eprintln!("[oxidrive] the window: taken {took:?} ns, returned {returned:?} ns");
    for l in r.p4_lines().iter().filter(|l| l.contains("shared:") || l.contains("window") || l.contains("upload") || l.contains("supervisor") || l.contains("smb:")) { eprintln!("[oxidrive]   {l}"); }
    assert!(took.is_some() && returned.is_some(), "no window: the card was never taken and returned");
    for u in upload_phases(&r.p4_lines()) {
        let mb = |ms: u64| u.bytes as f64 / 1e3 / ms.max(1) as f64;
        eprintln!("[oxidrive] upload {}: {} bytes written in {} ms ({:.2} MB/s), read back and renamed in {} ms ({:.2} MB/s)",
                  u.path, u.bytes, u.written - u.begun, mb(u.written - u.begun), u.renamed - u.written, mb(u.renamed - u.written));
        eprintln!("[oxidrive]   writing: {}; reading back: {}", r.rates(u.begun, u.written), r.rates(u.written, u.renamed));
    }
    // the card: fsck, and every clip as the car wrote it
    let fsck = sd_image::fsck(&r.card).unwrap_or_else(|e| panic!("fsck: {e}"));
    eprintln!("[oxidrive] fsck: {}", fsck.trim().replace('\n', "; "));
    let mut disk = FileDisk(std::fs::File::open(&r.card).unwrap());
    let fs = Fat32::mount(&mut disk).unwrap();
    for (stamp, clip) in EVENTS.iter().zip(&clips) {
        assert!(fs.read_path(&mut disk, &event_clip(stamp)).unwrap() == *clip, "{stamp}: the clip on the image differs");
    }
    // the share: each upload byte for byte as on the card, no part left, the manifest
    let share = r.smb.path();
    let mut uploaded = Vec::new();
    for (stamp, clip) in EVENTS.iter().zip(&clips) {
        let p = share.join(format!("SavedClips/{stamp}/event.mp4"));
        match std::fs::read(&p) {
            Ok(b) => { assert!(b == *clip, "{}: differs from the card", p.display()); uploaded.push(*stamp); eprintln!("[oxidrive] share: SavedClips/{stamp}/event.mp4 matches the card ({} bytes)", b.len()); }
            Err(_) => eprintln!("[oxidrive] share: SavedClips/{stamp}/event.mp4 not uploaded"),
        }
        assert!(!share.join(format!("SavedClips/{stamp}/event.mp4.part")).exists(), "{stamp}: a .part left on the share");
    }
    let manifest = std::fs::read_to_string(share.join(".oxidrive-index.tsv")).unwrap_or_default();
    eprintln!("[oxidrive] share: manifest {:?}", manifest);
    assert!(!uploaded.is_empty(), "nothing reached the share");
    assert!(uploaded.iter().all(|s| manifest.contains(s)), "the manifest lacks an upload");
}

/// A clip the car keeps recording while a pass runs, so the card is never quiet and no window opens.
const RECENT: &str = "TeslaCam/RecentClips/2026-01-01_01-00-00-front.mp4";

/// What the firmware did after a fault on its SMB link, from its console.
struct LinkFault {
    n: usize,
    archived: usize,
    first: Option<u64>,
    second: Option<u64>,
    fault: u64,
    restored: Option<u64>,
    done: Option<u64>,
    /// The faulted pass's `scan: shared` line, and the next session's.
    pass_end: Option<u64>,
    reconnect: Option<u64>,
    lost: usize,
    failures: usize,
    opens: usize,
    saves: usize,
    save_begins: usize,
    calls: usize,
    /// The longest main-loop iteration the firmware reported after the fault, s.
    longest_loop: f64,
    /// Passes that ran while the fault held, after the faulted one: (end, seconds).
    later: Vec<(u64, f64)>,
    /// Passes that found no session after the link was restored.
    unreachable_after: usize,
}

/// The scenario both link-fault tests share. The car records `OXIDRIVE_CUT_EVENTS` (4) saved
/// events of `OXIDRIVE_CUT_CLIP_KIB` (1536) KiB and goes on writing a RecentClips clip, 64 KiB
/// every 500 ms, so no window opens. `force_open` starts a pass once the MQTT session is up,
/// which archives all but the newest. The car then records as many newer events, and the
/// firmware's own pass a minute later uploads those, newest first, and then checks the ones
/// already archived. (A second `force_open` would take the card if it met the supervisor
/// probing.) `fault` is applied to the relay in front of the share while that pass writes its
/// second clip, and the relay restored `hold_s` later; the run ends two seconds after every clip
/// is on the share, or at `OXIDRIVE_SECONDS` (`deadline_s`).
fn smb_fault(card: &str, fault: impl Fn(&services::Relay), hold_s: u64, deadline_s: u64) -> LinkFault {
    use esp_soc::usb_host::{fat32::{clip_bytes, mp4_bytes, write_clip, Fat32}, msc::Msc, UsbHost};
    const S: u64 = 1_000_000_000;
    let per = Rig::env_secs("OXIDRIVE_CUT_EVENTS", 4) as usize;
    let kib = Rig::env_secs("OXIDRIVE_CUT_CLIP_KIB", 1536) as usize;
    let hold = hold_s * S;
    let deadline = Rig::env_secs("OXIDRIVE_SECONDS", deadline_s) * S;
    let mut r = rig_with(RigOpts { card_name: card.into(), car: true, smb_relay: true, ..RigOpts::new(oxidrive_board::dir()) });
    let relay = r.smb_relay.clone().unwrap();
    let shared = r.shared.clone();
    let (cmd, mqtt, share) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone(), r.smb.path());
    let n = 2 * per;
    let stamps: Vec<String> = (0..n).map(|i| format!("2026-01-01_00-{i:02}-00")).collect();
    let clips: Vec<Vec<u8>> = stamps.iter().map(|s| mp4_bytes(s, kib << 10)).collect();
    let archived = || stamps.iter().zip(&clips).filter(|(s, c)| std::fs::read(share.join(format!("SavedClips/{s}/event.mp4"))).is_ok_and(|b| b == **c)).count();
    // The console line from which the faulted pass's uploads are counted, once batch two is recorded.
    let (mut first, mut second, mut mark) = (None, None, None);
    let (mut fault_at, mut restored, mut done) = (None, None, None);
    let wall = std::time::Instant::now();
    {
        let mut host = UsbHost::new(&mut r);
        host.enumerate(10 * S).unwrap_or_else(|e| panic!("enumerate: {e}"));
        let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
        msc.wait_ready(100_000_000, 10 * S).unwrap_or_else(|e| panic!("never ready: {e}"));
        msc.read_capacity().unwrap();
        let mut fs = Fat32::mount(&mut msc).unwrap();
        let record = |fs: &mut Fat32, msc: &mut Msc, batch: std::ops::Range<usize>| {
            let end = batch.end;
            for (stamp, clip) in stamps[batch.clone()].iter().zip(&clips[batch]) {
                let path = event_clip(stamp);
                fs.mkdir_all(msc, path.rsplit_once('/').unwrap().0).unwrap();
                write_clip(fs, msc, &path, clip, 64 << 10, 0, 1).unwrap_or_else(|e| panic!("{path}: {e}"));
            }
            eprintln!("[oxidrive] t={:.3} s: the car has recorded {end} saved events", msc.host.now_ns() as f64 / 1e9);
        };
        record(&mut fs, &mut msc, 0..per);
        fs.mkdir_all(&mut msc, RECENT.rsplit_once('/').unwrap().0).unwrap();
        let mut rec = fs.create_path(&mut msc, RECENT).unwrap();
        let chunk = clip_bytes(RECENT, 64 << 10);
        let mut step = 0u64;
        while msc.host.now_ns() < deadline {
            if step.is_multiple_of(5) { rec.write(&mut fs, &mut msc, &chunk).unwrap(); }
            step += 1;
            msc.host.idle_ns(100_000_000).unwrap();
            let now = msc.host.now_ns();
            let (ready, passes, lines, second_written) = {
                let sh = shared.borrow();
                let from = mark.unwrap_or(usize::MAX).min(sh.lines.len());
                let begun: Vec<usize> = (from..sh.lines.len()).filter(|&i| sh.lines[i].1.contains("smb-> begin_part SavedClips/")).collect();
                (sh.when("session ready").is_some(), sh.all("scan: shared").len(), sh.lines.len(),
                 begun.len() >= 2 && sh.lines[begun[1]..].iter().any(|l| l.1.contains("smb-> write_part")))
            };
            if first.is_none() && ready {
                mqtt.publish(&cmd, "force_open").unwrap();
                first = Some(now);
            }
            if second.is_none() && first.is_some() && passes >= 1 {
                record(&mut fs, &mut msc, per..n);
                mark = Some(lines);
                second = Some(msc.host.now_ns());
            }
            if fault_at.is_none() && second_written {
                fault(&relay);
                fault_at = Some(now);
                eprintln!("[oxidrive] t={:.3} s: the SMB link faulted", now as f64 / 1e9);
            }
            if let (Some(f), None) = (fault_at, restored) {
                if now >= f + hold {
                    relay.restore();
                    restored = Some(now);
                    eprintln!("[oxidrive] t={:.3} s: the SMB relay restored", now as f64 / 1e9);
                }
            }
            if restored.is_some() && done.is_none() && archived() == n { done = Some(now); }
            if done.is_some_and(|d| now > d + 2 * S) { break; }
        }
        rec.close(&mut fs, &mut msc).unwrap();
    }
    eprintln!("[oxidrive] {:.1} s wall for {:.1} s emulated", wall.elapsed().as_secs_f64(), r.cs.now_ns as f64 / 1e9);
    let fault = fault_at.expect("the second pass never wrote a second clip");
    let secs = |t: Option<u64>| t.map_or("never".to_string(), |t| format!("{:.3} s", t as f64 / 1e9));
    // From the 100 ms slice that held the fault to the next session the firmware opened.
    let after: Vec<(u64, &str)> = r.lines.iter().filter(|l| !l.1 && l.0 >= fault).map(|l| (l.0, l.2.as_str())).collect();
    let reconnect = after.iter().position(|l| l.1.contains("smb: connected to"));
    let dead = &after[..reconnect.unwrap_or(after.len())];
    let count = |what: &str| dead.iter().filter(|l| l.1.contains(what)).count();
    let failures: Vec<&str> = dead.iter().map(|l| l.1).filter(|l| !l.contains("connection lost") && (l.contains(" failed") || l.contains("smb: unlink") || l.contains("could not"))).collect();
    let scans: Vec<(u64, f64)> = after.iter().filter(|l| l.1.contains("scan: shared"))
        .map(|l| (l.0, l.1.split("scan: shared").nth(1).and_then(|r| r.trim_start_matches(" full").trim().split('s').next()).and_then(|x| x.parse().ok()).unwrap_or(f64::NAN))).collect();
    let longest_loop = after.iter().filter_map(|l| l.1.split("last iteration took ").nth(1)?.split('s').next()?.parse::<f64>().ok()).fold(0.0, f64::max);
    let f = LinkFault {
        n, archived: archived(), first, second, fault, restored, done,
        pass_end: scans.first().map(|s| s.0), reconnect: reconnect.map(|i| after[i].0),
        lost: after.iter().filter(|l| l.1.contains("archiver: connection lost")).count(),
        failures: failures.len(), opens: count("begin_part failed"), saves: count("index: could not save"),
        save_begins: count("smb-> begin_part .oxidrive-index.tsv"), calls: count("smb-> "), longest_loop,
        later: scans.iter().skip(1).filter(|s| restored.is_none_or(|r| s.0 < r)).copied().collect(),
        unreachable_after: restored.map_or(0, |r| after.iter().filter(|l| l.0 >= r && l.1.contains("archiver: no SMB session")).count()),
    };
    eprintln!("[oxidrive] forced {}, newer events recorded {}, fault {}, the pass ended {}, restored {}, reconnected {}, all archived {}",
              secs(f.first), secs(f.second), secs(Some(f.fault)), secs(f.pass_end), secs(f.restored), secs(f.reconnect), secs(f.done));
    eprintln!("[oxidrive] before the next session: {} SMB calls logged, {} failure lines ({} failed clip opens, {} failed manifest saves), {} manifest saves begun; {} \"archiver: connection lost\"; longest main-loop iteration after the fault {:.1} s",
              f.calls, f.failures, f.opens, f.saves, f.save_begins, f.lost, f.longest_loop);
    for (t, d) in &f.later { eprintln!("[oxidrive] a later pass while the fault held ended at {:.3} s after {d:.1} s", *t as f64 / 1e9); }
    eprintln!("[oxidrive] {} of {n} clips on the share byte for byte; {} passes with no session after the restore", f.archived, f.unreachable_after);
    for l in r.lines.iter().filter(|l| !l.1 && (l.2.contains("shared: ") || l.2.contains("last iteration took") || l.2.contains("no SMB session"))) { eprintln!("[oxidrive]   {:8.3} {}", l.0 as f64 / 1e9, l.2); }
    for l in dead.iter().take(40) { eprintln!("[oxidrive]   {:8.3} {}", l.0 as f64 / 1e9, l.1); }
    r.report(10);
    for s in &stamps { assert!(!share.join(format!("SavedClips/{s}/event.mp4.part")).exists() || f.archived < n, "{s}: a .part left on the share"); }
    f
}

/// A shared pass whose SMB link is cut mid-upload, as a home network that blinks: the relay
/// closes every connection while the pass writes its second clip and passes again 5 s
/// (`OXIDRIVE_CUT_SECS`) later. The firmware should end the pass at once with one
/// "archiver: connection lost", at most a couple of failed calls and no manifest save on the
/// dead session; its next pass reconnects and every clip reaches the share byte for byte. The
/// counts print either way, so a firmware without the fix shows what it does instead.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_shared_pass_stops_once_when_its_smb_link_is_cut() {
    let f = smb_fault("oxidrive-smb-cut.img", services::Relay::cut, Rig::env_secs("OXIDRIVE_CUT_SECS", 5), 240);
    assert_eq!(f.lost, 1, "not one \"archiver: connection lost\"");
    assert!(f.failures <= 2, "{} failure lines on the dead session", f.failures);
    assert_eq!(f.saves + f.save_begins, 0, "a manifest save on the dead session");
    assert!(f.reconnect.is_some(), "no session after the cut");
    assert_eq!(f.archived, f.n, "not every clip reached the share");
}

/// The same pass with its link half-open: the relay drops everything both ways and closes
/// nothing, as when the NAS vanishes without a reset, for 100 s (`OXIDRIVE_SILENT_SECS`), new
/// connections included. Nothing on the wire ends the pass; the firmware's own timeouts must.
/// It should end the faulted pass within 90 s (libsmb2's 60 s request timeout and margin),
/// with no main-loop iteration longer, and a pass that runs while the link is still silent
/// should give up within 30 s (the 15 s connect timeout and margin). Once the link passes
/// again, the next pass finds a session, with no reply left from the fault mistaken for its own,
/// and every clip reaches the share.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_shared_pass_ends_when_its_smb_link_goes_half_open() {
    let f = smb_fault("oxidrive-smb-silent.img", services::Relay::half_open, Rig::env_secs("OXIDRIVE_SILENT_SECS", 100), 360);
    let ended = f.pass_end.map(|t| (t - f.fault) as f64 / 1e9);
    eprintln!("[oxidrive] the faulted pass ended {ended:?} s after the fault; later passes while silent: {:?}", f.later);
    assert!(ended.is_some_and(|s| s <= 90.0), "the faulted pass did not end within 90 s: {ended:?}");
    assert!(f.longest_loop <= 90.0, "a main-loop iteration of {:.1} s", f.longest_loop);
    assert_eq!(f.lost, 1, "not one \"archiver: connection lost\"");
    assert_eq!(f.saves + f.save_begins, 0, "a manifest save on the dead session");
    assert!(!f.later.is_empty(), "no pass ran while the link was silent");
    assert!(f.later.iter().all(|l| l.1 <= 30.0), "a pass on the silent link took {:?}", f.later);
    assert_eq!(f.unreachable_after, 0, "a pass after the restore found no session");
    assert_eq!(f.archived, f.n, "not every clip reached the share");
}

/// CRC-32 (IEEE 802.3, reflected), as the update's manifest carries it.
fn crc32(data: &[u8]) -> u32 {
    !data.iter().fold(!0u32, |c, &b| (0..8).fold(c ^ b as u32, |c, _| if c & 1 != 0 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 }))
}

/// An update for `version` on the share: an image and a manifest naming its true size, CRC-32
/// and SHA-256. `signed`: signed with the throwaway key in `OXI_OTA_KEY_DIR`, the one the
/// fixture's build embeds, so the manifest verifies and the card is taken; the download is
/// not an image and fails later. Otherwise the signature is one no key made, refused before the
/// card is taken.
fn stage_update(r: &Rig, version: &str, signed: bool) {
    use esp_soc::usb_host::fat32::clip_bytes;
    let file = format!("oxidrive-{version}.bin");
    let image = clip_bytes("not an image", 256 << 10);
    let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
    let statement = format!("oxidrive-ota-v1\nversion={version}\nsize={}\nsha256={}\n", image.len(), hex(&sha256(&image)));
    let sig = if signed { hex(&sign_with_test_key(statement.as_bytes())) } else { hex(&[sha256(b"r"), sha256(b"s")].concat()) };
    let dir = r.smb.path().join("oxidrive-ota");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(&file), &image).unwrap();
    std::fs::write(dir.join("manifest"), format!("version={version}\nfile={file}\nsize={}\ncrc32={:08x}\nsha256={}\nsig={sig}\n", image.len(), crc32(&image), hex(&sha256(&image)))).unwrap();
}

/// ECDSA P-256 over SHA-256 of `msg` with the throwaway key in `OXI_OTA_KEY_DIR`, by openssl, as
/// the manifest carries it: `r || s`.
fn sign_with_test_key(msg: &[u8]) -> [u8; 64] {
    let dir = std::env::var("OXI_OTA_KEY_DIR").expect("set OXI_OTA_KEY_DIR to the throwaway OTA key the fixture's build embeds");
    let path = tmp("ota-statement");
    std::fs::write(&path, msg).unwrap();
    let out = std::process::Command::new("openssl").args(["dgst", "-sha256", "-sign"]).arg(format!("{dir}/ota-signing-key.pem")).arg(&path).output()
        .unwrap_or_else(|e| panic!("openssl: {e}"));
    assert!(out.status.success(), "openssl: {}", String::from_utf8_lossy(&out.stderr));
    // SEQUENCE { INTEGER r, INTEGER s }, each big-endian, minimal, perhaps with a leading zero.
    let der = out.stdout;
    assert_eq!(der.first(), Some(&0x30), "not a DER signature");
    let (mut at, mut rs) = (2, [0u8; 64]);
    for half in rs.chunks_mut(32) {
        assert_eq!(der[at], 0x02, "not a DER integer");
        let len = der[at + 1] as usize;
        let mut v = &der[at + 2..at + 2 + len];
        while v.len() > 32 && v[0] == 0 { v = &v[1..]; }
        half[32 - v.len()..].copy_from_slice(v);
        at += 2 + len;
    }
    rs
}

/// Goal 4 of milestone 4b, the refusal half: an update whose manifest names the image's true
/// size, CRC-32 and SHA-256 but carries a signature no key made. oxidrive refuses it on the
/// signature over the manifest alone, before the card is taken, and nothing boots.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_refuses_an_unsigned_update() {
    let mut r = rig("oxidrive-ota.img", 4 << 30, false);
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 60) * 1_000_000_000;
    while r.cs.now_ns < end && !r.p4_has("session ready") { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    assert!(r.p4_has("session ready"), "no MQTT session");
    let version = "0.1.0-0000001";
    stage_update(&r, version, false);
    r.mqtt.client.publish(&format!("oxidrive/{}/cmd", r.id), &format!("update {version}")).unwrap();
    while r.cs.now_ns < end && !r.p4_has("signature did not verify") { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    let t = r.cs.now_ns + 3_000_000_000;
    r.run_to(t);
    for l in r.p4_lines().iter().filter(|l| l.contains("ota") || l.contains("supervisor")) { eprintln!("[oxidrive]   {l}"); }
    r.report(10);
    assert!(r.p4_has("signature did not verify"), "not refused on the signature");
    assert!(!r.p4_has("supervisor: card taken for a firmware update"), "the forged update took the card");
    assert_eq!(r.cs.summary.p4_resets, 0, "nothing booted");
}

// Commands sent while the device is away. oxidrive keeps a persistent MQTT session, so the broker
// queues QoS 1 commands for it across a reboot; the publisher's MQTT 5 message expiry bounds how
// late one may arrive. The rig's relay stands between the firmware and the broker.

/// The P4's console lines from index `from` of `r.lines` on.
fn since(r: &Rig, from: usize) -> Vec<&str> { r.lines[from..].iter().filter(|l| !l.1).map(|l| l.2.as_str()).collect() }
fn count_since(r: &Rig, from: usize, what: &str) -> usize { since(r, from).iter().filter(|l| l.contains(what)).count() }

/// Run a second at a time until a P4 line from `from` on holds `what`, or until `end_ns`.
fn run_until(r: &mut Rig, from: usize, what: &str, end_ns: u64) -> bool {
    while r.cs.now_ns < end_ns && count_since(r, from, what) == 0 { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    count_since(r, from, what) > 0
}

/// A rig whose broker connection goes through the relay, run until its MQTT session is ready.
fn queue_rig(card: &str, end_ns: u64) -> Rig {
    let mut r = rig_with(RigOpts { card_name: card.into(), mqtt_relay: true, ..RigOpts::new(oxidrive_board::dir()) });
    assert!(run_until(&mut r, 0, "session ready", end_ns), "no MQTT session");
    r
}

/// The device away, as after a radio wedge: its broker connection cut until the broker has
/// published its will, then the P4 reset. `publish` runs while it is away, with the broker's
/// client and the command topic; the relay is restored after it, for the reboot to find.
fn away(r: &mut Rig, publish: impl FnOnce(&services::MqttClient, &str)) {
    let relay = r.relay.as_ref().expect("a rig with mqtt_relay");
    relay.cut();
    let avail = format!("oxidrive/{}/avail", r.id);
    let t0 = std::time::Instant::now();
    while !r.mqtt.client.retained(&[&avail], Duration::from_secs(1)).unwrap().iter().any(|(_, p)| p == "offline") {
        assert!(t0.elapsed() < Duration::from_secs(15), "the broker never saw the device go");
    }
    eprintln!("[oxidrive] t={:.3} s: the broker has the device offline; resetting the P4", r.cs.now_ns as f64 / 1e9);
    r.cs.p4.reboot();
    publish(&r.mqtt.client, &format!("oxidrive/{}/cmd", r.id));
    relay.restore();
}

/// (a) and (b): two commands published while the P4 reboots with its broker unreachable. The one
/// whose 2 s expiry passes first (`ble_probe`) is dropped by the broker; the other
/// (`update_status`) is executed once after the reboot, on the session the broker kept.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with a persistent MQTT session), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_runs_a_command_queued_across_a_reboot_and_drops_an_expired_one() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 120) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-queue.img", end);
    let mark = r.lines.len();
    away(&mut r, |mq, cmd| {
        mq.publish_v5(cmd, "ble_probe", Some(2)).unwrap();
        mq.publish_v5(cmd, "update_status", Some(600)).unwrap();
        // the broker expires by its own clock, which runs while the emulator waits
        std::thread::sleep(Duration::from_secs(4));
    });
    let ran = run_until(&mut r, mark, "ota: status:", end);
    let t = r.cs.now_ns + 3_000_000_000;
    r.run_to(t);
    for l in since(&r, mark).iter().filter(|l| l.contains("mqtt") || l.contains("command") || l.contains("ota:")) { eprintln!("[oxidrive]   {l}"); }
    r.report(10);
    assert!(count_since(&r, mark, "session up (resumed)") > 0, "the broker did not keep the session across the reboot");
    assert!(ran, "the queued update_status was not executed after the reboot");
    assert_eq!(count_since(&r, mark, "ota: status:"), 1, "update_status ran more than once");
    assert_eq!(count_since(&r, mark, "command BleProbe"), 0, "the expired ble_probe was delivered");
}

/// (c): a redelivery. The relay drops what the device sends, so its PUBACK for `update_status`
/// never reaches the broker; the connection is then cut, and the broker sends the same message
/// (same packet id) again on the resumed session. The device recognises it and acts once.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with a persistent MQTT session), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_acts_once_on_a_redelivered_command() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 120) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-redeliver.img", end);
    let mark = r.lines.len();
    let cmd = format!("oxidrive/{}/cmd", r.id);
    r.relay.as_ref().unwrap().deafen();
    r.mqtt.client.publish_v5(&cmd, "update_status", Some(600)).unwrap();
    assert!(run_until(&mut r, mark, "ota: status:", end), "update_status was not executed");
    let relay = r.relay.as_ref().unwrap();
    relay.cut();
    relay.restore();
    eprintln!("[oxidrive] t={:.3} s: the PUBACK swallowed and the connection cut", r.cs.now_ns as f64 / 1e9);
    let again = run_until(&mut r, mark, "delivered again", end);
    let t = r.cs.now_ns + 3_000_000_000;
    r.run_to(t);
    for l in since(&r, mark).iter().filter(|l| l.contains("mqtt") || l.contains("command") || l.contains("ota:")) { eprintln!("[oxidrive]   {l}"); }
    r.report(10);
    assert!(count_since(&r, mark, "session up (resumed)") > 0, "no resumed session after the cut");
    assert!(again, "the broker's redelivery was not recognised as one");
    assert_eq!(count_since(&r, mark, "ota: status:"), 1, "the redelivered update_status ran twice");
}

/// The retained state document's `ota` object, waiting up to `secs` of emulated time for one
/// whose phase is `phase`.
fn ota_state(r: &mut Rig, phase: &str, secs: u64) -> Option<String> {
    let state = format!("oxidrive/{}/state", r.id);
    let want = format!("\"phase\":\"{phase}\"");
    for _ in 0..secs {
        let doc = r.mqtt.client.retained(&[&state], Duration::from_secs(1)).unwrap().into_iter().next().map(|(_, p)| p);
        if let Some(ota) = doc.as_deref().and_then(|d| d.split("\"ota\":").nth(1)).map(|o| o.split('}').next().unwrap_or("").to_string()) {
            if ota.contains(&want) { return Some(ota); }
        }
        let t = r.cs.now_ns + 1_000_000_000;
        r.run_to(t);
    }
    None
}

/// (d): `update V now` and then `update_cancel V`, both queued while the P4 reboots. They arrive
/// back to back after CONNACK and the cancel wins: the card is never taken and the state says
/// `cancelled`. Then the same update, sent after the cancel, takes the card: the cancel withdrew
/// only what came before it, and without it the update would have gone ahead.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with update_cancel), OXI_OTA_KEY_DIR (the throwaway key that build embeds) and openssl, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_cancel_queued_behind_an_update_now_wins() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 150) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-cancel.img", end);
    let version = "0.1.0-0000002";
    stage_update(&r, version, true);
    let mark = r.lines.len();
    away(&mut r, |mq, cmd| {
        mq.publish_v5(cmd, &format!("update {version} now"), Some(300)).unwrap();
        mq.publish_v5(cmd, &format!("update_cancel {version}"), Some(300)).unwrap();
    });
    let cancelled = run_until(&mut r, mark, &format!("update to {version} cancelled"), end);
    let ota = ota_state(&mut r, "cancelled", 10);
    let taken_while_cancelled = count_since(&r, mark, "card taken for a firmware update");
    for l in since(&r, mark).iter().filter(|l| l.contains("mqtt") || l.contains("command") || l.contains("ota") || l.contains("supervisor")) { eprintln!("[oxidrive]   {l}"); }
    eprintln!("[oxidrive] the state's ota after the cancel: {ota:?}");
    assert!(count_since(&r, mark, "session up (resumed)") > 0, "the broker did not keep the session across the reboot");
    assert!(cancelled, "the queued update was not cancelled");
    assert_eq!(taken_while_cancelled, 0, "the cancelled update took the card");
    assert!(ota.is_some(), "the state document does not say cancelled");
    let after = r.lines.len();
    r.mqtt.client.publish_v5(&format!("oxidrive/{}/cmd", r.id), &format!("update {version} now"), Some(300)).unwrap();
    let taken = run_until(&mut r, after, "supervisor: card taken for a firmware update", end);
    for l in since(&r, after).iter().filter(|l| l.contains("command") || l.contains("ota") || l.contains("supervisor")) { eprintln!("[oxidrive]   {l}"); }
    r.report(10);
    assert!(taken, "the update sent after the cancel did not take the card");
    assert_eq!(r.cs.summary.p4_resets, 0, "nothing was installed");
}

/// (e): `update V now` with a 2 s expiry, published while the P4 reboots and kept from it past
/// the expiry. The broker drops it; `update_status`, published after it, is delivered and shows
/// the queue reached the device.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with a persistent MQTT session), OXI_OTA_KEY_DIR (the throwaway key that build embeds) and openssl, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_never_sees_an_update_now_that_expired_while_it_was_away() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 120) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-expired.img", end);
    let version = "0.1.0-0000003";
    stage_update(&r, version, true);
    let mark = r.lines.len();
    away(&mut r, |mq, cmd| {
        mq.publish_v5(cmd, &format!("update {version} now"), Some(2)).unwrap();
        mq.publish_v5(cmd, "update_status", Some(600)).unwrap();
        std::thread::sleep(Duration::from_secs(4));
    });
    let delivered = run_until(&mut r, mark, "ota: status:", end);
    let t = r.cs.now_ns + 5_000_000_000;
    r.run_to(t);
    for l in since(&r, mark).iter().filter(|l| l.contains("mqtt") || l.contains("command") || l.contains("ota") || l.contains("supervisor")) { eprintln!("[oxidrive]   {l}"); }
    r.report(10);
    assert!(count_since(&r, mark, "session up (resumed)") > 0, "the broker did not keep the session across the reboot");
    assert!(delivered, "the queue never reached the device");
    assert_eq!(count_since(&r, mark, "Update {"), 0, "the expired update was delivered");
    assert_eq!(count_since(&r, mark, "card taken for a firmware update"), 0, "the expired update took the card");
}

/// A cancel that arrives while the update holds the card: `update V now` sent while connected,
/// `update_cancel V` once the card is taken. The supervisor stops the update and gives the card
/// back at once. The download itself never finishes here: the firmware hashes it with the SHA
/// accelerator in DMA mode, which on the P4 goes through the AXI GDMA (not modelled), so
/// SHA_BUSY (0x50091018) never clears. The checks the SMB thread makes itself, between reads and
/// before it sets the boot partition, are not reached.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with update_cancel), OXI_OTA_KEY_DIR (the throwaway key that build embeds) and openssl, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_cancel_gives_back_the_card_an_update_holds() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 120) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-cancel-writing.img", end);
    let version = "0.1.0-0000004";
    stage_update(&r, version, true);
    let cmd = format!("oxidrive/{}/cmd", r.id);
    let mark = r.lines.len();
    r.mqtt.client.publish_v5(&cmd, &format!("update {version} now"), Some(300)).unwrap();
    assert!(run_until(&mut r, mark, "card taken for a firmware update", end), "the update never took the card");
    let (taken, sent_ns) = (r.lines.len(), r.cs.now_ns);
    r.mqtt.client.publish_v5(&cmd, &format!("update_cancel {version}"), Some(300)).unwrap();
    let returned = run_until(&mut r, taken, "card returned to the car", end);
    let back_ns = r.lines[taken..].iter().find(|l| !l.1 && l.2.contains("card returned to the car")).map(|l| l.0);
    for l in since(&r, mark).iter().filter(|l| l.contains("command") || l.contains("ota") || l.contains("supervisor")) { eprintln!("[oxidrive]   {l}"); }
    eprintln!("[oxidrive] the card back {:?} ms of emulated time after the cancel was published (to the 100 ms slice)", back_ns.map(|t| (t - sent_ns) / 1_000_000));
    r.report(10);
    assert!(count_since(&r, taken, &format!("cancelling the update to {version}")) > 0, "the cancel did not stop the update");
    assert!(returned, "the card did not come back after the cancel");
    assert_eq!(r.cs.summary.p4_resets, 0, "nothing was installed");
}

/// The coredump partition's offset and size in a partition table: a data partition of subtype 3.
fn coredump_partition(table: &[u8]) -> Option<(usize, usize)> {
    table.as_chunks::<32>().0.iter().take_while(|e| e[..2] == [0xaa, 0x50]).find(|e| e[2] == 0x01 && e[3] == 0x03).map(|e| {
        let word = |i: usize| u32::from_le_bytes(e[i..i + 4].try_into().unwrap()) as usize;
        (word(4), word(8))
    })
}

/// Run until a P4 line after line `from` holds `s`, for at most `secs` more of emulated time.
fn run_until_line(r: &mut Rig, from: usize, s: &str, secs: u64) -> bool {
    let end = r.cs.now_ns + secs * 1_000_000_000;
    loop {
        if r.lines[from..].iter().any(|l| !l.1 && l.2.contains(s)) { return true; }
        if r.cs.now_ns >= end { return false; }
        let t = r.cs.now_ns + 500_000_000;
        r.run_to(t);
    }
}

fn p4_lines_since(r: &Rig, from: usize) -> Vec<String> { r.lines[from..].iter().filter(|l| !l.1).map(|l| l.2.clone()).collect() }

/// Stub `stubs` (function entry, return value) until the P4 resets, in steps small enough that
/// the next boot does not reach them, then remove them.
fn crash_once(r: &mut Rig, stubs: &[(u32, u32)]) {
    let resets = r.cs.summary.p4_resets;
    for &(at, v) in stubs { r.cs.p4.stubs.insert(at, v); }
    let end = r.cs.now_ns + 20_000_000_000;
    while r.cs.summary.p4_resets == resets && r.cs.now_ns < end { let t = r.cs.now_ns + 20_000_000; r.run_to(t); }
    for (at, _) in stubs { r.cs.p4.stubs.remove(at); }
    assert_eq!(r.cs.summary.p4_resets, resets + 1, "the stubs did not reset the P4 once");
}

/// Ask `diag crash` with request id `id` and return the reply, run on 3 s for it.
fn diag_crash(r: &mut Rig, heard: &services::Listener, id: u32) -> Option<String> {
    r.mqtt.client.publish(&format!("oxidrive/{}/diag/req", r.id), &format!(r#"{{"id":{id},"cmd":"crash"}}"#)).unwrap();
    let t = r.cs.now_ns + 3_000_000_000;
    r.run_to(t);
    heard.messages().into_iter().find(|(t, p)| t.ends_with("/diag/rsp") && p.contains(&format!(r#""id":{id},"#))).map(|m| m.1)
}

/// A crash and its report, over eight boots of a firmware that reports its core dumps. 1: the
/// coredump partition is blank, nothing is reported and `diag crash` answers `none`. 2:
/// `smb2_init_context` stubbed to return NULL fails the firmware's link check at boot (a Rust
/// `assert!`, so `abort()`), and the panic handler writes its core dump to flash. 3: the stub
/// gone, the next boot finds the dump and reports it: a `crash:` line, a `crash` event on MQTT,
/// the partition's bytes and a `.txt` under `oxidrive-crash/` on the share, and `diag crash`
/// answering `reported`. 4: a reset by the test; the same dump is reported already and nothing is
/// said or uploaded again. 5: `clock_gettime` stubbed to fail once the firmware runs, so std's
/// `Instant::now` panics with a message. 6: the report of that dump carries the message. 7: the
/// clock fails again with `esp_core_dump_write_prepare` stubbed to ESP_ERR_NO_MEM, as for a dump
/// too big for the partition, and flash keeps the last dump. 8: that crash is reported lost,
/// naming the dump flash still holds. `app.elf` beside the images names the stubbed functions.
/// `OXIDRIVE_CRASH_DIR` keeps the uploaded image and `.txt` there, for oxidrive's decoder.
#[test]
#[ignore = "needs a crash-reporting oxidrive (images, app.elf and nvs.csv under OXIDRIVE_DIR), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_reports_its_crash_to_the_nas() {
    let mut r = rig("oxidrive-crash.img", 4 << 30, false);
    let dir = oxidrive_board::dir();
    let elf = std::fs::read(format!("{dir}/app.elf")).unwrap_or_else(|e| panic!("{dir}/app.elf: {e}"));
    r.cs.p4.add_symbols(&elf).unwrap();
    let link_check = r.cs.p4.sym_addr("smb2_init_context").expect("smb2_init_context in app.elf");
    let table = std::fs::read(format!("{dir}/partition-table.bin")).unwrap();
    let (core_off, core_size) = coredump_partition(&table).expect("a coredump partition in partition-table.bin");
    let base = format!("oxidrive/{}", r.id);
    let heard = r.mqtt.client.listen(&[&format!("{base}/event"), &format!("{base}/diag/rsp")]).unwrap();
    let wall = std::time::Instant::now();

    // 1: a blank partition
    assert!(run_until_line(&mut r, 0, "session ready", 40), "no MQTT session");
    let reply = diag_crash(&mut r, &heard, 1);
    eprintln!("[oxidrive] boot 1: diag crash -> {reply:?}");
    assert!(reply.as_deref().is_some_and(|p| p.contains(r#""present":false,"err":"none""#)), "diag crash on a blank partition: {reply:?}");
    assert!(r.cs.p4.bus.flash[core_off..core_off + 4] == [0xff; 4], "the coredump partition was not blank");
    let said: Vec<&str> = r.p4_lines().into_iter().filter(|l| l.contains("crash") || l.contains("core dump")).collect();
    assert!(said.iter().all(|l| !l.contains("oxidrive::crash")), "a blank partition was reported: {said:?}");

    // 2: the panic
    r.cs.p4.stubs.insert(link_check, 0);
    let mark = r.lines.len();
    r.cs.p4.reboot();
    let end = r.cs.now_ns + 20_000_000_000;
    // small steps: the next boot reaches the same panic about 250 ms after the reset
    while r.cs.summary.p4_resets == 0 && r.cs.now_ns < end { let t = r.cs.now_ns + 20_000_000; r.run_to(t); }
    r.cs.p4.stubs.remove(&link_check);
    for l in p4_lines_since(&r, mark).iter().filter(|l| l.contains("anic") || l.contains("ore dump") || l.contains("abort") || l.contains("libsmb2")) { eprintln!("[oxidrive] boot 2: {l}"); }
    assert_eq!(r.cs.summary.p4_resets, 1, "the stubbed link check did not reset the P4");
    let len = u32::from_le_bytes(r.cs.p4.bus.flash[core_off..core_off + 4].try_into().unwrap()) as usize;
    assert!(len > 4 && len <= core_size, "no core dump in the partition (length word {len:#x})");
    let image = r.cs.p4.bus.flash[core_off..core_off + len].to_vec();
    assert_eq!(crc32(&image[..len - 4]), u32::from_le_bytes(image[len - 4..].try_into().unwrap()), "the dump fails its CRC");
    eprintln!("[oxidrive] boot 2: a {len}-byte core dump in the {core_size}-byte partition");

    // 3: the report
    let mark = r.lines.len();
    let uploaded = run_until_line(&mut r, mark, "crash: the core dump is on the NAS at", 60);
    // the upload can beat the MQTT session, which announces the crash when it comes up
    assert!(run_until_line(&mut r, mark, "session ready", 20), "no MQTT session after the crash");
    let t = r.cs.now_ns + 2_000_000_000;
    r.run_to(t);
    let boot3 = p4_lines_since(&r, mark);
    for l in boot3.iter().filter(|l| l.contains("crash") || l.contains("core dump")) { eprintln!("[oxidrive] boot 3: {l}"); }
    assert!(uploaded, "the dump never reached the NAS");
    assert!(boot3.iter().any(|l| l.contains("crash: abort in main pc=0x") && l.contains(" why: abort() was called at PC 0x")), "no crash line naming the abort");
    let crash_dir = r.smb.path().join("oxidrive-crash");
    let mut names: Vec<String> = std::fs::read_dir(&crash_dir).unwrap().flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect();
    names.sort();
    eprintln!("[oxidrive] share: oxidrive-crash/ holds {names:?}");
    assert_eq!(names.len(), 2, "one image and one .txt");
    let core = names.iter().find(|n| n.ends_with(".core")).expect("an image");
    assert!(core.contains("-boot") && core.ends_with(&format!("-{:08x}.core", u32::from_le_bytes(image[len - 4..].try_into().unwrap()))), "{core}: not <fw>-boot<N>-<crc>.core");
    assert!(std::fs::read(crash_dir.join(core)).unwrap() == image, "the uploaded image differs from the partition");
    let txt = std::fs::read_to_string(crash_dir.join(core.replace(".core", ".txt"))).unwrap();
    eprintln!("[oxidrive] share: the .txt:\n{txt}");
    let elf_name = txt.lines().find_map(|l| l.strip_prefix("elf: ")).expect("an elf line");
    assert!(elf_name.len() >= 8 && sha256_hex(&elf).starts_with(elf_name), "the .txt's elf {elf_name} is not app.elf's hash");
    assert!(txt.contains("\nfound in: the running image\n") && txt.contains("\nkind: abort\n") && txt.contains("\ntask: main\n"));
    if let Some(keep) = std::env::var_os("OXIDRIVE_CRASH_DIR") {
        std::fs::create_dir_all(&keep).unwrap();
        for n in &names { std::fs::copy(crash_dir.join(n), std::path::Path::new(&keep).join(n)).unwrap(); }
    }
    let fw = txt.lines().find_map(|l| l.strip_prefix("fw: ")).unwrap();
    assert!(core.starts_with(&format!("{fw}-boot")), "{core} is not named for {fw}");
    let events: Vec<String> = heard.messages().into_iter().filter(|(t, p)| t.ends_with("/event") && p.contains(r#""kind":"crash""#)).map(|m| m.1).collect();
    for e in &events { eprintln!("[oxidrive] event: {e}"); }
    assert!(events.iter().any(|e| e.contains("abort in main pc=")), "no crash event with the summary");
    assert!(events.iter().any(|e| e.contains("uploaded oxidrive-crash/")), "no event for the upload");
    let reply = diag_crash(&mut r, &heard, 2);
    eprintln!("[oxidrive] boot 3: diag crash -> {reply:?}");
    let reply = reply.expect("no diag crash reply");
    assert!(reply.contains(r#""present":true,"reported":true"#) && reply.contains(r#""task":"main""#) && reply.contains(r#""kind":"abort""#), "{reply}");

    // 4: the same dump, reported already
    let mark = r.lines.len();
    r.cs.p4.reboot();
    assert!(run_until_line(&mut r, mark, "session ready", 40), "no MQTT session after the reset");
    let t = r.cs.now_ns + 5_000_000_000;
    r.run_to(t);
    let boot4 = p4_lines_since(&r, mark);
    for l in boot4.iter().filter(|l| l.contains("crash") || l.contains("core dump")) { eprintln!("[oxidrive] boot 4: {l}"); }
    assert!(boot4.iter().any(|l| l.contains("core dump") && l.contains("already on the NAS")), "boot 4 did not find the dump reported");
    assert!(!boot4.iter().any(|l| l.contains("crash: abort in main") || l.contains("crash: the core dump is on the NAS")), "boot 4 reported the dump again");
    assert_eq!(std::fs::read_dir(&crash_dir).unwrap().count(), 2, "boot 4 uploaded again");

    // 5: a Rust panic with a message, after the journal is open
    let clock = r.cs.p4.sym_addr("clock_gettime").expect("clock_gettime in app.elf");
    let mark = r.lines.len();
    crash_once(&mut r, &[(clock, u32::MAX)]);
    for l in p4_lines_since(&r, mark).iter().filter(|l| l.contains("panicked") || l.contains("ore dump has")) { eprintln!("[oxidrive] boot 5: {l}"); }

    // 6: its report carries the message
    let mark = r.lines.len();
    assert!(run_until_line(&mut r, mark, "crash: the core dump is on the NAS at", 60), "the second dump never reached the NAS");
    let boot6 = p4_lines_since(&r, mark);
    for l in boot6.iter().filter(|l| l.contains("crash")) { eprintln!("[oxidrive] boot 6: {l}"); }
    assert!(boot6.iter().any(|l| l.contains("crash: abort in ") && l.contains(" why: pal/unix/time.rs:")), "no crash line carrying the panic message");
    let txt6 = std::fs::read_dir(&crash_dir).unwrap().flatten().map(|e| e.path()).find(|p| p.extension().is_some_and(|x| x == "txt") && !names.iter().any(|n| p.ends_with(n))).expect("a second .txt");
    let txt6 = std::fs::read_to_string(txt6).unwrap();
    let panic_line = txt6.lines().find(|l| l.starts_with("panic: ")).unwrap_or_else(|| panic!("no panic line in\n{txt6}"));
    eprintln!("[oxidrive] boot 6: the .txt's {panic_line}");
    assert!(panic_line.contains("pal/unix/time.rs:") && panic_line.contains("unwrap()"), "{panic_line}");
    if let Some(keep) = std::env::var_os("OXIDRIVE_CRASH_DIR") {
        for e in std::fs::read_dir(&crash_dir).unwrap().flatten() { std::fs::copy(e.path(), std::path::Path::new(&keep).join(e.file_name())).unwrap(); }
    }

    // 7: a panic whose dump IDF refuses before it erases anything
    assert!(run_until_line(&mut r, mark, "session ready", 20), "no MQTT session in boot 6");
    let prepare = ["esp_core_dump_write_prepare", "esp_core_dump_flash_write_prepare"].iter().find_map(|n| r.cs.p4.sym_addr(n)).expect("esp_core_dump_write_prepare in app.elf");
    let held = r.cs.p4.bus.flash[core_off..core_off + core_size].to_vec();
    crash_once(&mut r, &[(clock, u32::MAX), (prepare, 0x101)]);
    assert!(r.cs.p4.bus.flash[core_off..core_off + core_size] == held[..], "the refused dump changed the partition");

    // 8: reported lost, naming what flash holds
    let mark = r.lines.len();
    assert!(run_until_line(&mut r, mark, "session ready", 40), "no MQTT session after the lost dump");
    let t = r.cs.now_ns + 3_000_000_000;
    r.run_to(t);
    let boot8 = p4_lines_since(&r, mark);
    for l in boot8.iter().filter(|l| l.contains("crash")) { eprintln!("[oxidrive] boot 8: {l}"); }
    assert!(boot8.iter().any(|l| l.contains("crash: lost: boot ") && l.contains("followed a panic and flash kept no dump of it; flash holds the dump ") && l.ends_with(", on the NAS")), "no lost line naming the dump flash holds");
    let reply = diag_crash(&mut r, &heard, 3).expect("no diag crash reply in boot 8");
    eprintln!("[oxidrive] boot 8: diag crash -> {reply}");
    assert!(reply.contains(r#""present":true,"err":"lost","reported":true"#), "{reply}");
    assert!(heard.messages().iter().any(|(t, p)| t.ends_with("/event") && p.contains(r#""detail":"lost: boot "#)), "no lost event");
    assert_eq!(std::fs::read_dir(&crash_dir).unwrap().count(), 4, "boot 8 uploaded something");
    r.report(10);
    eprintln!("[oxidrive] {:.1} s wall for {:.1} s emulated", wall.elapsed().as_secs_f64(), r.cs.now_ns as f64 / 1e9);
}
