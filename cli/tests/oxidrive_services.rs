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
use common::*;
use esp32sim::cosim::{CoSim, Config};
use esp_soc::nat::{Forward, Nat};
use services::{Mosquitto, Samba};
use std::net::{Ipv4Addr, SocketAddrV4};
use std::time::Duration;

/// The services' test accounts; `OXIDRIVE_DIR/nvs.csv` carries the same.
const SHARE_USER: &str = "esp32sim";
const SHARE_PASS: &str = "esp32sim-smb";
const BROKER_USER: &str = "esp32sim";
const BROKER_PASS: &str = "esp32sim-mqtt";
/// The network's clock at power-on, 2026-01-01 00:00 UTC, as in the goldens.
const EPOCH_S: u64 = 1_767_225_600;
const FW: &str = "web/wasm/fw/public";

/// Where a URL in nvs.csv points the firmware: the guest address and port it connects to, and
/// the path after them. A host name resolves to 10.0.2.3, the virtual network's DNS answer
/// under `--net none`.
#[derive(Debug)]
struct Endpoint { guest: SocketAddrV4, path: String }

fn endpoint(csv: &str, scheme: &str, default_port: u16) -> Endpoint {
    let prefix = format!("{scheme}://");
    let url = csv.lines().filter_map(|l| l.rsplit(',').next()).find(|v| v.starts_with(&prefix))
        .unwrap_or_else(|| panic!("nvs.csv has no {prefix} value"));
    let rest = &url[prefix.len()..];
    let (authority, path) = rest.split_once('/').unwrap_or((rest, ""));
    let host_port = authority.rsplit('@').next().unwrap();
    let (host, port) = match host_port.rsplit_once(':') { Some((h, p)) => (h, p.parse().unwrap()), None => (host_port, default_port) };
    let ip = host.parse::<Ipv4Addr>().unwrap_or(Ipv4Addr::new(10, 0, 2, 3));
    Endpoint { guest: SocketAddrV4::new(ip, port), path: path.to_string() }
}

/// The committed ESP-Hosted slave on a C6 booted from its mask ROM, the virtual access point on
/// its radio and `nat` behind it, the network's clock fixed.
fn hosted_slave(nat: Nat) -> esp32c6::Machine {
    let read = |name: &str| std::fs::read(root().join(FW).join(name)).unwrap_or_else(|e| panic!("{FW}/{name}: {e}"));
    let mut c6 = esp32c6::machine([0x40, 0x4c, 0xca, 0x00, 0x11, 0x22], 4 << 20);
    esp_soc::SocBus::set_flash_size(&mut c6.bus, 4 << 20);
    c6.load_rom(&std::fs::read(rom("esp32c6_rev0")).unwrap()).unwrap();
    for (off, f) in [(0, "c6-hosted-bootloader.bin"), (0x8000, "c6-hosted-ptable.bin"), (0xd000, "c6-hosted-otadata.bin"), (0x10000, "c6-hosted-slave.bin")] { c6.write_flash(off, &read(f)).unwrap(); }
    // bb_init, the PHY's baseband calibration (riscv32-esp-elf-nm on network_adapter.elf)
    c6.stubs.insert(0x420b_ac5c, 0);
    let ap = esp_soc::wifi::ApConfig::parse(&format!("ssid=esp32sim,psk=esp32sim-pass,epoch={EPOCH_S}")).unwrap();
    esp_soc::SocBus::attach_wifi(&mut c6.bus, ap, Some(nat)).unwrap();
    c6.boot_rom();
    c6
}

/// LP_TSENS CTRL (0x5012F000; IDF v5.5.3 soc/esp32p4/register/hw_ver1/soc/tsens_struct.h): OUT in
/// bits 7:0. The block is register RAM here; the driver only read-modify-writes the other fields,
/// so a seeded OUT stays. 104 reads as 25 C in the -10..80 C range oxidrive asks for (offset 0:
/// 0.4386 * OUT - 20.52, temperature_sensor_ll.h). Without a reading every window is vetoed.
const TSENS_CTRL: u32 = 0x5012_f000;
const TSENS_OUT_25C: u32 = 104;

/// The firmware, its card and both services, the pair joined: the C6's network forwards the
/// two guest addresses nvs.csv names to the services, and refuses everything else.
struct Rig { cs: CoSim, smb: Samba, mqtt: Mosquitto, card: std::path::PathBuf, id: String, lines: Vec<(u64, bool, String)> }

fn rig(card_name: &str, card_bytes: u64, car: bool) -> Rig {
    let dir = oxidrive_board::dir();
    let csv_path = std::path::Path::new(&dir).join("nvs.csv");
    let csv = std::fs::read_to_string(&csv_path).unwrap_or_else(|e| panic!("{}: {e}", csv_path.display()));
    let (smb_at, mqtt_at) = (endpoint(&csv, "smb", 445), endpoint(&csv, "mqtt", 1883));
    let share = smb_at.path.split('/').next().filter(|s| !s.is_empty()).expect("a share in the smb:// URL").to_string();
    let smb = Samba::start(&share, SHARE_USER, SHARE_PASS).unwrap_or_else(|e| panic!("samba: {e}"));
    let mqtt = Mosquitto::start(BROKER_USER, BROKER_PASS).unwrap_or_else(|e| panic!("mosquitto: {e}"));
    let (mut p4, board_log, card) = oxidrive_board::boot_card(card_name, true, Some(card_bytes));
    let table = std::fs::read(format!("{dir}/partition-table.bin")).unwrap();
    let (off, size) = oxidrive_board::nvs_partition(&table).expect("an NVS partition in partition-table.bin");
    p4.write_flash(off, &oxidrive_board::nvs_image(&csv_path, size)).unwrap();
    emu_core::Bus::write32(&mut p4.bus, TSENS_CTRL, TSENS_OUT_25C).unwrap();
    if car { esp_soc::SocBus::attach_usb_host(&mut p4.bus, Default::default()).unwrap(); }
    // the firmware's id: the factory MAC's last three bytes (its main.rs)
    let mac = oxidrive_board::board_mac(&board_log);
    let id = format!("{:02x}{:02x}{:02x}", mac[3], mac[4], mac[5]);
    let log = std::env::var("OXIDRIVE_NET_LOG").is_ok_and(|v| v == "1");
    let forwards = vec![Forward { guest: smb_at.guest, host: smb.svc.addr() }, Forward { guest: mqtt_at.guest, host: mqtt.svc.addr() }];
    for f in &forwards { eprintln!("[oxidrive] forwarding {} to {}", f.guest, f.host); }
    let cs = CoSim::new(p4, hosted_slave(Nat::forward_only(log, forwards)), Config::default());
    Rig { cs, smb, mqtt, card, id, lines: Vec::new() }
}

impl Rig {
    /// Run to `ns`, keeping every console line with the time its slice ended.
    fn advance(&mut self, ns: u64) -> Result<(), String> {
        while self.cs.now_ns < ns {
            let r = self.cs.run_until((self.cs.now_ns + 100_000_000).min(ns));
            let t = self.cs.now_ns;
            for (c6, l) in self.cs.lines.drain(..) {
                if std::env::var("OXIDRIVE_ECHO").is_ok_and(|v| v == "1") { eprintln!("[{:8.3}] {}{l}", t as f64 / 1e9, if c6 { "[c6] " } else { "" }); }
                self.lines.push((t, c6, l));
            }
            r.map_err(|s| format!("the P4 stopped at {:.3} s: {s:?}", t as f64 / 1e9))?;
        }
        Ok(())
    }
    fn run_to(&mut self, ns: u64) { self.advance(ns).unwrap_or_else(|e| panic!("{e}")) }
    fn p4_has(&self, s: &str) -> bool { self.lines.iter().any(|(_, c6, l)| !c6 && l.contains(s)) }
    fn p4_lines(&self) -> Vec<&str> { self.lines.iter().filter(|l| !l.1).map(|l| l.2.as_str()).collect() }
    fn report(&self, last: usize) {
        let c = self.cs.c6.borrow();
        eprintln!("[oxidrive] {:.3} s emulated; {}", self.cs.now_ns as f64 / 1e9, esp_soc::SocBus::report(&c.m.bus).replace('\n', "\n[oxidrive] "));
        eprintln!("[oxidrive] the P4's last lines:");
        let p4 = self.p4_lines();
        for l in &p4[p4.len().saturating_sub(last)..] { eprintln!("[oxidrive]   {l}"); }
    }
    fn env_secs(name: &str, default: u64) -> u64 { std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default) }
}

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

/// The car on the P4's USB port, through the pair; the console lines kept as `run_to` keeps them.
impl esp_soc::usb_host::UsbPort for Rig {
    fn link(&mut self) -> Option<&mut esp_periph::usb_otg::UsbLink> { esp_soc::SocBus::usb_link(&mut self.cs.p4.bus) }
    fn run_until(&mut self, t_ns: u64) -> Result<(), String> { self.advance(t_ns) }
}

/// A card image file as a block device, for reading the card after the run.
struct FileDisk(std::fs::File);
impl esp_soc::usb_host::fat32::BlockDevice for FileDisk {
    fn block_size(&self) -> u32 { 512 }
    fn blocks(&self) -> u64 { self.0.metadata().unwrap().len() / 512 }
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String> {
        use std::io::{Read, Seek};
        self.0.seek(std::io::SeekFrom::Start(lba * 512)).map_err(|e| e.to_string())?;
        self.0.read_exact(buf).map_err(|e| e.to_string())
    }
    fn write_blocks(&mut self, _: u64, _: &[u8]) -> Result<(), String> { Err("read only".into()) }
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
    let force = std::env::var("OXIDRIVE_FORCE_OPEN").is_ok_and(|v| v == "1");
    let deadline = Rig::env_secs("OXIDRIVE_SECONDS", 200) * 1_000_000_000;
    let clips: Vec<Vec<u8>> = EVENTS.iter().map(|s| mp4_bytes(s, (mib << 20) as usize)).collect();
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
