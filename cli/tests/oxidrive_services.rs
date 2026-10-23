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

/// CRC-32 (IEEE 802.3, reflected), as the update's manifest carries it.
fn crc32(data: &[u8]) -> u32 {
    !data.iter().fold(!0u32, |c, &b| (0..8).fold(c ^ b as u32, |c, _| if c & 1 != 0 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 }))
}

/// Goal 4 of milestone 4b, the refusal half: an update fetched over the same SMB path, whose
/// manifest names the image's true size, CRC-32 and SHA-256 but carries a signature no key made.
/// oxidrive should take the card for the update, download and check the image, refuse it on the
/// signature, and give the card back; the running image stays. A signed image needs the owner's
/// private key, which no test uses. It does not pass yet: the firmware hashes the download with
/// the SHA accelerator in DMA mode, which on the P4 goes through the AXI GDMA (not modelled), so
/// SHA_BUSY (0x50091018) never clears and the update stops after taking the card.
/// An update for `version` on the share: an image and a manifest naming its true size, CRC-32
/// and SHA-256, with a signature no key made. The checks pass and the card is taken; the
/// signature fails only after the download.
fn stage_update(r: &Rig, version: &str) {
    use esp_soc::usb_host::fat32::clip_bytes;
    let file = format!("oxidrive-{version}.bin");
    let image = clip_bytes("not an image", 256 << 10);
    let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
    let sig = hex(&[sha256(b"r"), sha256(b"s")].concat());
    let dir = r.smb.path().join("oxidrive-ota");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(&file), &image).unwrap();
    std::fs::write(dir.join("manifest"), format!("version={version}\nfile={file}\nsize={}\ncrc32={:08x}\nsha256={}\nsig={sig}\n", image.len(), crc32(&image), hex(&sha256(&image)))).unwrap();
}

#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto; fails until the P4's AXI GDMA serves SHA's DMA mode"]
fn external_oxidrive_refuses_an_unsigned_update() {
    let mut r = rig("oxidrive-ota.img", 4 << 30, false);
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 60) * 1_000_000_000;
    while r.cs.now_ns < end && !r.p4_has("session ready") { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    assert!(r.p4_has("session ready"), "no MQTT session");
    let version = "0.1.0-0000001";
    stage_update(&r, version);
    r.mqtt.client.publish(&format!("oxidrive/{}/cmd", r.id), &format!("update {version}")).unwrap();
    while r.cs.now_ns < end && !r.p4_has("signature did not verify") { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    let t = r.cs.now_ns + 3_000_000_000;
    r.run_to(t);
    for l in r.p4_lines().iter().filter(|l| l.contains("ota") || l.contains("supervisor")) { eprintln!("[oxidrive]   {l}"); }
    r.report(10);
    assert!(r.p4_has("supervisor: card taken for a firmware update"), "the update never took the card");
    assert!(r.p4_has("signature did not verify"), "not refused on the signature (SHA's DMA mode on the P4 is not modelled)");
    assert!(r.p4_has("supervisor: card returned to the car"), "the card did not come back");
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
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with update_cancel), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_cancel_queued_behind_an_update_now_wins() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 150) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-cancel.img", end);
    let version = "0.1.0-0000002";
    stage_update(&r, version);
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
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with a persistent MQTT session), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_never_sees_an_update_now_that_expired_while_it_was_away() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 120) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-expired.img", end);
    let version = "0.1.0-0000003";
    stage_update(&r, version);
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
/// back at once. The download itself never finishes here (the P4 model does not serve SHA's DMA
/// mode; see `external_oxidrive_refuses_an_unsigned_update`), so the checks the SMB thread makes
/// itself, between reads and before it sets the boot partition, are not reached.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR (a build with update_cancel), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_cancel_gives_back_the_card_an_update_holds() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 120) * 1_000_000_000;
    let mut r = queue_rig("oxidrive-cancel-writing.img", end);
    let version = "0.1.0-0000004";
    stage_update(&r, version);
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
