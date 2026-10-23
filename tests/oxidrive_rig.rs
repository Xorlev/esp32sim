//! The owner's oxidrive firmware (not in this repository) on the P4 with ESP-Hosted's slave on the
//! C6, its MQTT broker and SMB server running on the host (`services`), and a host on its USB port:
//! the rig the `oxidrive_*` tests in `cli/tests` share, included with `#[path]` next to `common`,
//! `sd_image`, `oxidrive_board` and `services`.
#![allow(dead_code)]
use crate::common::*;
use crate::oxidrive_board;
use crate::services::{Mosquitto, Relay, Samba};
use esp32sim::cosim::{CoSim, Config};
use esp_soc::nat::{Forward, Nat};
use std::cell::RefCell;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::rc::Rc;

/// The services' test accounts; `OXIDRIVE_DIR/nvs.csv` carries the same.
pub const SHARE_USER: &str = "esp32sim";
pub const SHARE_PASS: &str = "esp32sim-smb";
pub const BROKER_USER: &str = "esp32sim";
pub const BROKER_PASS: &str = "esp32sim-mqtt";
/// The network's clock at power-on, 2026-01-01 00:00 UTC, as in the goldens.
pub const EPOCH_S: u64 = 1_767_225_600;
pub const FW: &str = "web/wasm/fw/public";

/// Where a URL in nvs.csv points the firmware: the guest address and port it connects to, and
/// the path after them. A host name resolves to 10.0.2.3, the virtual network's DNS answer
/// under `--net none`.
#[derive(Debug)]
pub struct Endpoint { pub guest: SocketAddrV4, pub path: String }

pub fn endpoint(csv: &str, scheme: &str, default_port: u16) -> Endpoint {
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
pub fn hosted_slave(nat: Nat) -> esp32c6::Machine {
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

/// The firmware, its card and both services, the pair joined: the C6's network forwards the
/// two guest addresses nvs.csv names to the services, and refuses everything else.
pub struct Rig {
    pub cs: CoSim, pub smb: Samba, pub mqtt: Mosquitto, pub card: std::path::PathBuf, pub id: String, pub lines: Vec<(u64, bool, String)>, pub samples: Vec<Sample>,
    /// the firmware's way to the broker, with `RigOpts::mqtt_relay`
    pub relay: Option<Relay>,
    /// the P4's console lines and the card's writes, readable while a car holds the rig as its port
    pub shared: Rc<RefCell<Shared>>,
    card_log: Option<Rc<RefCell<Vec<u64>>>>,
}

/// What a test reads while a car drives the rig: the P4's console lines and the card's writes,
/// each with the time of the slice it came in (a microframe while the car moves data, 100 ms
/// otherwise).
#[derive(Default)]
pub struct Shared { pub lines: Vec<(u64, String)>, pub writes: Vec<CardWrite> }

/// A run of blocks the card was written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CardWrite { pub at_ns: u64, pub lba: u64, pub count: u64 }

impl Shared {
    /// When the first console line holding `s` came, if one has.
    pub fn when(&self, s: &str) -> Option<u64> { self.lines.iter().find(|l| l.1.contains(s)).map(|l| l.0) }
    /// Every time a line holding `s` came.
    pub fn all(&self, s: &str) -> Vec<u64> { self.lines.iter().filter(|l| l.1.contains(s)).map(|l| l.0).collect() }
}

/// The air, the NAT and the CPUs every 100 ms: frames the station sent and received, bytes to
/// the host and back, instructions each chip retired (one a cycle; a core waiting in WFI retires
/// none).
#[derive(Clone, Copy, Debug, Default)]
pub struct Sample { pub ns: u64, pub tx_frames: u64, pub rx_frames: u64, pub to_host: u64, pub to_guest: u64, pub p4_insns: [u64; 2], pub c6_insns: u64 }

pub fn rig(card_name: &str, card_bytes: u64, car: bool) -> Rig {
    rig_with(RigOpts { card_name: card_name.into(), card_bytes, car, ..RigOpts::new(oxidrive_board::dir()) })
}

/// The rig's firmware, card and car.
pub struct RigOpts {
    /// the firmware's images and nvs.csv
    pub dir: String,
    pub card_name: String,
    pub card_bytes: u64,
    /// a host on the USB port
    pub car: bool,
    /// a card image prepared beforehand, the card's programming time, the SD bus timed
    pub image: Option<std::path::PathBuf>,
    pub prg_us: Option<u64>,
    pub timed: Option<bool>,
    /// log every block written to the card (`Shared::writes`)
    pub log_writes: bool,
    /// put a `Relay` between the firmware and the broker (`Rig::relay`)
    pub mqtt_relay: bool,
}
impl RigOpts {
    pub fn new(dir: String) -> Self { RigOpts { dir, card_name: "oxidrive.img".into(), card_bytes: 4 << 30, car: false, image: None, prg_us: None, timed: None, log_writes: false, mqtt_relay: false } }
}

pub fn rig_with(o: RigOpts) -> Rig {
    let dir = o.dir.clone();
    let csv_path = std::path::Path::new(&dir).join("nvs.csv");
    let csv = std::fs::read_to_string(&csv_path).unwrap_or_else(|e| panic!("{}: {e}", csv_path.display()));
    let (smb_at, mqtt_at) = (endpoint(&csv, "smb", 445), endpoint(&csv, "mqtt", 1883));
    let share = smb_at.path.split('/').next().filter(|s| !s.is_empty()).expect("a share in the smb:// URL").to_string();
    let smb = Samba::start(&share, SHARE_USER, SHARE_PASS).unwrap_or_else(|e| panic!("samba: {e}"));
    let mqtt = Mosquitto::start(BROKER_USER, BROKER_PASS).unwrap_or_else(|e| panic!("mosquitto: {e}"));
    let card_log = o.log_writes.then(|| Rc::new(RefCell::new(Vec::new())));
    let opts = oxidrive_board::CardOpts { size: Some(o.card_bytes), image: o.image.clone(), prg_us: o.prg_us, timed: o.timed, log: card_log.clone() };
    let (mut p4, board_log, card) = oxidrive_board::boot_in(&dir, &o.card_name, true, opts);
    let table = std::fs::read(format!("{dir}/partition-table.bin")).unwrap();
    let (off, size) = oxidrive_board::nvs_partition(&table).expect("an NVS partition in partition-table.bin");
    p4.write_flash(off, &oxidrive_board::nvs_image(&csv_path, size)).unwrap();
    if std::env::var_os("ESP_EMU_DEBUG").is_some() { p4.set_debug(&esp_soc::DebugFlags::from_env()); }
    if o.car { esp_soc::SocBus::attach_usb_host(&mut p4.bus, Default::default()).unwrap(); }
    // the firmware's id: the factory MAC's last three bytes (its main.rs)
    let mac = oxidrive_board::board_mac(&board_log);
    let id = format!("{:02x}{:02x}{:02x}", mac[3], mac[4], mac[5]);
    let log = std::env::var("OXIDRIVE_NET_LOG").is_ok_and(|v| v == "1");
    let relay = o.mqtt_relay.then(|| Relay::start(mqtt.svc.addr()).unwrap_or_else(|e| panic!("{e}")));
    let broker = relay.as_ref().map_or(mqtt.svc.addr(), Relay::addr);
    let forwards = vec![Forward { guest: smb_at.guest, host: smb.svc.addr() }, Forward { guest: mqtt_at.guest, host: broker }];
    for f in &forwards { eprintln!("[oxidrive] forwarding {} to {}", f.guest, f.host); }
    let cs = CoSim::new(p4, hosted_slave(Nat::forward_only(log, forwards)), Config::default());
    Rig { cs, smb, mqtt, card, id, lines: Vec::new(), samples: Vec::new(), relay, shared: Rc::new(RefCell::new(Shared::default())), card_log }
}

impl Rig {
    /// Run to `ns`, keeping every console line with the time its slice ended.
    pub fn advance(&mut self, ns: u64) -> Result<(), String> {
        while self.cs.now_ns < ns {
            let r = self.cs.run_until((self.cs.now_ns + 100_000_000).min(ns));
            let t = self.cs.now_ns;
            let mut shared = self.shared.borrow_mut();
            for (c6, l) in self.cs.lines.drain(..) {
                if std::env::var("OXIDRIVE_ECHO").is_ok_and(|v| v == "1") { eprintln!("[{:8.3}] {}{l}", t as f64 / 1e9, if c6 { "[c6] " } else { "" }); }
                if !c6 { shared.lines.push((t, l.clone())); }
                self.lines.push((t, c6, l));
            }
            if let Some(log) = &self.card_log {
                for lba in log.borrow_mut().drain(..) {
                    match shared.writes.last_mut() {
                        Some(w) if w.at_ns == t && w.lba + w.count == lba => w.count += 1,
                        _ => shared.writes.push(CardWrite { at_ns: t, lba, count: 1 }),
                    }
                }
            }
            drop(shared);
            r.map_err(|s| format!("the P4 stopped at {:.3} s: {s:?}", t as f64 / 1e9))?;
            let c = self.cs.c6.borrow();
            let link = &c.m.bus.periph.wifi_mac.link;
            let nat = link.net().and_then(|n| n.nat.as_ref());
            self.samples.push(Sample { ns: t, tx_frames: link.tx_frames(), rx_frames: link.rx_frames(),
                                       to_host: nat.map_or(0, |n| n.bytes_to_host), to_guest: nat.map_or(0, |n| n.bytes_to_guest),
                                       p4_insns: [0, 1].map(|i| self.cs.p4.cores.get(i).map_or(0, |k| k.retired_count)), c6_insns: c.m.cores.iter().map(|k| k.retired_count).sum() });
        }
        Ok(())
    }

    /// Rates between emulated milliseconds `from` and `to`, from the samples: frames a second
    /// each way and MB/s each way.
    pub fn rates(&self, from: u64, to: u64) -> String {
        let at = |ms: u64| self.samples.iter().find(|s| s.ns >= ms * 1_000_000).copied().unwrap_or_default();
        let (a, b) = (at(from), at(to));
        let secs = (b.ns.saturating_sub(a.ns)) as f64 / 1e9;
        let busy = |n: u64, hz: f64| n as f64 / (secs * hz) * 100.0;
        format!("station {:.0} frames/s out, {:.0} in; NAT {:.2} MB/s to the host, {:.2} back; busy: P4 core 0 {:.0}%, core 1 {:.0}%, C6 {:.0}%",
                (b.tx_frames - a.tx_frames) as f64 / secs, (b.rx_frames - a.rx_frames) as f64 / secs,
                (b.to_host - a.to_host) as f64 / 1e6 / secs, (b.to_guest - a.to_guest) as f64 / 1e6 / secs,
                busy(b.p4_insns[0] - a.p4_insns[0], 360e6), busy(b.p4_insns[1] - a.p4_insns[1], 360e6), busy(b.c6_insns - a.c6_insns, 160e6))
    }
    pub fn run_to(&mut self, ns: u64) { self.advance(ns).unwrap_or_else(|e| panic!("{e}")) }
    pub fn p4_has(&self, s: &str) -> bool { self.lines.iter().any(|(_, c6, l)| !c6 && l.contains(s)) }
    pub fn p4_lines(&self) -> Vec<&str> { self.lines.iter().filter(|l| !l.1).map(|l| l.2.as_str()).collect() }
    pub fn report(&self, last: usize) {
        let c = self.cs.c6.borrow();
        eprintln!("[oxidrive] {:.3} s emulated; {}", self.cs.now_ns as f64 / 1e9, esp_soc::SocBus::report(&c.m.bus).replace('\n', "\n[oxidrive] "));
        eprintln!("[oxidrive] the P4's last lines:");
        let p4 = self.p4_lines();
        for l in &p4[p4.len().saturating_sub(last)..] { eprintln!("[oxidrive]   {l}"); }
    }
    pub fn env_secs(name: &str, default: u64) -> u64 { std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default) }
}

/// The car on the P4's USB port, through the pair; the console lines kept as `run_to` keeps them.
impl esp_soc::usb_host::UsbPort for Rig {
    fn link(&mut self) -> Option<&mut esp_periph::usb_otg::UsbLink> { esp_soc::SocBus::usb_link(&mut self.cs.p4.bus) }
    fn run_until(&mut self, t_ns: u64) -> Result<(), String> { self.advance(t_ns) }
}

/// A card image file as a block device: the card read after a run, or prepared before one.
pub struct FileDisk(pub std::fs::File);
impl esp_soc::usb_host::fat32::BlockDevice for FileDisk {
    fn block_size(&self) -> u32 { 512 }
    fn blocks(&self) -> u64 { self.0.metadata().unwrap().len() / 512 }
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String> {
        use std::io::{Read, Seek};
        self.0.seek(std::io::SeekFrom::Start(lba * 512)).map_err(|e| e.to_string())?;
        self.0.read_exact(buf).map_err(|e| e.to_string())
    }
    fn write_blocks(&mut self, lba: u64, data: &[u8]) -> Result<(), String> {
        use std::io::{Seek, Write};
        self.0.seek(std::io::SeekFrom::Start(lba * 512)).map_err(|e| e.to_string())?;
        self.0.write_all(data).map_err(|e| e.to_string())
    }
}

/// One clip's way to the share, from the firmware's SMB log lines (`smb-> ...`), in its own
/// milliseconds: `begin_part`, the `.part` size check that follows the last write, `finish`.
pub struct Upload { pub path: String, pub bytes: u64, pub begun: u64, pub written: u64, pub renamed: u64 }

pub fn upload_phases(lines: &[&str]) -> Vec<Upload> {
    let ms = |l: &str| l.get(3..).and_then(|t| t.split(')').next()).and_then(|t| t.parse::<u64>().ok());
    let mut out = Vec::new();
    for (i, l) in lines.iter().enumerate() {
        let Some(path) = l.split("smb-> begin_part ").nth(1).and_then(|r| r.strip_suffix(" @0")) else { continue };
        if path.starts_with('.') { continue; }
        let after = &lines[i..];
        let find = |pat: &str| after.iter().find(|x| x.ends_with(pat)).and_then(|x| ms(x));
        let bytes = after.iter().take_while(|x| !x.contains("smb-> end_part")).filter_map(|x| x.split("smb-> write_part ").nth(1)?.strip_suffix(" bytes")?.parse::<u64>().ok()).sum();
        if let (Some(begun), Some(written), Some(renamed)) = (ms(l), find(&format!("smb-> size_of {path}.part")), find(&format!("smb-> finish {path}"))) {
            out.push(Upload { path: path.to_string(), bytes, begun, written, renamed });
        }
    }
    out
}

