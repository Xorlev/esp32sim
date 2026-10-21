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
            match msc.test_unit_ready() {
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
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto; fails until the P4's AXI GDMA serves SHA's DMA mode"]
fn external_oxidrive_refuses_an_unsigned_update() {
    use esp_soc::usb_host::fat32::clip_bytes;
    let mut r = rig("oxidrive-ota.img", 4 << 30, false);
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 60) * 1_000_000_000;
    while r.cs.now_ns < end && !r.p4_has("session ready") { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    assert!(r.p4_has("session ready"), "no MQTT session");
    let (version, file) = ("0.1.0-0000001", "oxidrive-0.1.0-0000001.bin");
    let image = clip_bytes("not an image", 256 << 10);
    let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
    let sig = hex(&[sha256(b"r"), sha256(b"s")].concat());
    let dir = r.smb.path().join("oxidrive-ota");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(file), &image).unwrap();
    std::fs::write(dir.join("manifest"), format!("version={version}\nfile={file}\nsize={}\ncrc32={:08x}\nsha256={}\nsig={sig}\n", image.len(), crc32(&image), hex(&sha256(&image)))).unwrap();
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
