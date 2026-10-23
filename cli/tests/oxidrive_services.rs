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
