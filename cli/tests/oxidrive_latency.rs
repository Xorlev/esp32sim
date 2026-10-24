//! Car write latency under a shared reader (milestone 4c, item 5): the owner's oxidrive firmware
//! (not in this repository) runs a shared pass, reading the card on slot 0 of the SDMMC host while
//! the car keeps recording to it over USB, and the car times every WRITE(10) (`Msc::times`). The SD
//! bus is timed on both slots and the card programs for `OXIDRIVE_SD_PRG_US` after each write; a
//! tap on each SD device records every command with its emulated time, which says when the
//! firmware's own card reads (the shared reader) and ESP-Hosted's SDIO transfers held the bus.
//!
//! The servers answer in host time, so two runs of one variant differ a little; everything else
//! (the car's schedule, the firmware, the network's clock) is fixed. [esp32p4.md](../../docs/esp32p4.md),
//! "Car write latency under a shared reader", has the method and the results, and "The fixed
//! reader" what `external_a_shared_pass_completes_without_racing_the_car` asserts.
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
use common::*;
use esp32sim::cosim::{CoSim, Config};
use esp_periph::sdmmc::{SdCmdError, SdDataError, SdDevice, SdResponse};
use esp_soc::nat::{Forward, Nat};
use esp_soc::usb_host::timing::{overlap_ns, CmdTime, Latency, MS_EDGES};
use services::{Mosquitto, Samba};
use std::cell::RefCell;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::rc::Rc;

// The services' accounts and the network's clock, as in oxidrive_services.rs.
const SHARE_USER: &str = "esp32sim";
const SHARE_PASS: &str = "esp32sim-smb";
const BROKER_USER: &str = "esp32sim";
const BROKER_PASS: &str = "esp32sim-mqtt";
const EPOCH_S: u64 = 1_767_225_600;
/// LP_TSENS CTRL's OUT seeded to read 25 C: oxidrive vetoes every pass without a temperature
/// (oxidrive_services.rs has the derivation).
const TSENS_CTRL: u32 = 0x5012_f000;
const TSENS_OUT_25C: u32 = 104;

/// The camera: 64 KiB due every 16 ms (4.1 MB/s).
const CHUNK: usize = 64 << 10;
const GAP_NS: u64 = 16_000_000;
/// The shared pass is forced at the first moment after this with the MQTT session up; the
/// firmware's own first pass is due 60 s after boot, after the window ends.
const FORCE_AT_NS: u64 = 9_000_000_000;
/// SCSI WRITE(10) and the SD commands the analysis looks at (SD Physical Layer Simplified
/// Specification v9.10 §4.7.4).
const WRITE10: u8 = 0x2a;
const READ10: u8 = 0x28;
const CMD_READ_SINGLE: u8 = 17;
const CMD_READ_MULTIPLE: u8 = 18;
const CMD_WRITE_MULTIPLE: u8 = 25;

#[derive(Debug)]
struct Endpoint { guest: SocketAddrV4, path: String }

/// Where a URL in nvs.csv points the firmware; a host name resolves to 10.0.2.3 under `--net none`.
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

/// The slave (`hosted_slave::Slave::from_env`) on a C6, the virtual access point and `nat` behind
/// it (`OXIDRIVE_NTP_FROM_S` holds its NTP answers back until then, as in `oxidrive_rig`).
fn hosted_slave(nat: Nat) -> esp32c6::Machine {
    let slave = hosted_slave::Slave::from_env();
    let mut c6 = esp32c6::machine([0x40, 0x4c, 0xca, 0x00, 0x11, 0x22], 4 << 20);
    esp_soc::SocBus::set_flash_size(&mut c6.bus, 4 << 20);
    c6.load_rom(&std::fs::read(rom("esp32c6_rev0")).unwrap()).unwrap();
    slave.flash(&mut c6);
    hosted_slave::stub_booted(&mut c6, &hosted_slave::known(&slave));
    let ap = esp_soc::wifi::ApConfig::parse(&format!("ssid=esp32sim,psk=esp32sim-pass,epoch={EPOCH_S}")).unwrap();
    esp_soc::SocBus::attach_wifi(&mut c6.bus, ap, Some(nat)).unwrap();
    let from_s: u64 = std::env::var("OXIDRIVE_NTP_FROM_S").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    if let Some(net) = c6.bus.periph.wifi_mac.link.net_mut() { net.ntp_from_us = from_s * 1_000_000; }
    c6.boot_rom();
    c6
}

/// Each P4 core's PC, `mstatus.MIE` and CLIC threshold at the end of a run step
/// (`LATENCY_PCS=1`), for finding what holds the car's commands up.
#[derive(Clone, Copy, Debug)]
struct Sample { t: u64, pc: [u32; 2], mie: [u32; 2], thresh: [u32; 2] }

/// One command on an SD device as the device saw it: sent at `start_ns`, its data phase over at
/// `end_ns` (`start_ns` without data; the timed bus hands each block over at the data phase's end),
/// and its short response (0 for none or a long one).
#[derive(Clone, Copy, Debug)]
struct BusOp { sdio: bool, cmd: u8, arg: u32, start_ns: u64, end_ns: u64, bytes: u32, answered: bool, resp: u32 }

/// An SD device with every command it takes logged.
struct Tap { inner: Box<dyn SdDevice>, sdio: bool, ops: Rc<RefCell<Vec<BusOp>>> }

impl Tap {
    fn data(&self, now: u64, n: usize) {
        if let Some(op) = self.ops.borrow_mut().iter_mut().rev().find(|o| o.sdio == self.sdio) { op.end_ns = op.end_ns.max(now); op.bytes += n as u32; }
    }
}

impl SdDevice for Tap {
    fn command(&mut self, now: u64, cmd: u8, arg: u32) -> Result<SdResponse, SdCmdError> {
        let r = self.inner.command(now, cmd, arg);
        let resp = match r { Ok(SdResponse::Short(v)) => v, _ => 0 };
        self.ops.borrow_mut().push(BusOp { sdio: self.sdio, cmd, arg, start_ns: now, end_ns: now, bytes: 0, answered: r != Err(SdCmdError::NoResponse), resp });
        r
    }
    fn read_data(&mut self, now: u64, buf: &mut [u8]) -> Result<(), SdDataError> { let r = self.inner.read_data(now, buf); self.data(now, buf.len()); r }
    fn write_data(&mut self, now: u64, data: &[u8]) -> Result<(), SdDataError> { let r = self.inner.write_data(now, data); self.data(now, data.len()); r }
    fn busy(&self, now: u64) -> bool { self.inner.busy(now) }
    fn busy_until(&self) -> Option<u64> { self.inner.busy_until() }
    fn bus_width(&self) -> Option<u8> { self.inner.bus_width() }
    fn interrupt(&self) -> bool { self.inner.interrupt() }
    fn io(&self) -> bool { self.inner.io() }
    fn edge_lost(&self) -> bool { self.inner.edge_lost() }
    fn name(&self) -> &str { self.inner.name() }
    fn set_log(&mut self, on: bool) { self.inner.set_log(on) }
}

/// Stands in for a device while it is moved into a `Tap`.
struct Nothing;
impl SdDevice for Nothing {
    fn command(&mut self, _: u64, _: u8, _: u32) -> Result<SdResponse, SdCmdError> { Err(SdCmdError::NoResponse) }
    fn read_data(&mut self, _: u64, _: &mut [u8]) -> Result<(), SdDataError> { Err(SdDataError::Timeout) }
    fn write_data(&mut self, _: u64, _: &[u8]) -> Result<(), SdDataError> { Err(SdDataError::Timeout) }
    fn name(&self) -> &str { "nothing" }
}

/// The firmware, its card (timed, programming for `prg_ns` after each write) and both services,
/// the pair joined; the SD devices tapped. `forced_at` is the first moment past `FORCE_AT_NS`
/// with the MQTT session up; `force` publishes `force_open` then, otherwise it is only noted.
struct Rig { cs: CoSim, smb: Samba, mqtt: Mosquitto, card: std::path::PathBuf, id: String, lines: Vec<(u64, String)>, bus: Rc<RefCell<Vec<BusOp>>>,
             force: bool, forced_at: Option<u64>, pcs: Option<Vec<Sample>> }

fn rig(dir: &str, prg_ns: u64, force: bool) -> Rig {
    let csv_path = std::path::Path::new(&dir).join("nvs.csv");
    let csv = std::fs::read_to_string(&csv_path).unwrap_or_else(|e| panic!("{}: {e}", csv_path.display()));
    let (smb_at, mqtt_at) = (endpoint(&csv, "smb", 445), endpoint(&csv, "mqtt", 1883));
    let share = smb_at.path.split('/').next().filter(|s| !s.is_empty()).expect("a share in the smb:// URL").to_string();
    let smb = Samba::start(&share, SHARE_USER, SHARE_PASS).unwrap_or_else(|e| panic!("samba: {e}"));
    let mqtt = Mosquitto::start(BROKER_USER, BROKER_PASS).unwrap_or_else(|e| panic!("mosquitto: {e}"));
    let (mut p4, board_log, card) = oxidrive_board::boot_in(dir, "oxidrive-latency.img", true, oxidrive_board::CardOpts { size: Some(4 << 30), ..Default::default() });
    let table = std::fs::read(format!("{dir}/partition-table.bin")).unwrap();
    let (off, size) = oxidrive_board::nvs_partition(&table).expect("an NVS partition in partition-table.bin");
    p4.write_flash(off, &oxidrive_board::nvs_image(&csv_path, size)).unwrap();
    emu_core::Bus::write32(&mut p4.bus, TSENS_CTRL, TSENS_OUT_25C).unwrap();
    // The card again, before the first instruction: timed, with its programming time, tapped.
    let bus = Rc::new(RefCell::new(Vec::new()));
    let store = esp_periph::sdmmc::FileStore::write_through(card.to_str().unwrap()).unwrap();
    let config = esp_periph::sdmmc::SdCardConfig { prg_ns, timed_bus: true, ..Default::default() };
    let sd = esp_periph::sdmmc::SdCard::with_config(Box::new(store), config).unwrap();
    p4.bus.periph.sdmmc.timed = true;
    p4.bus.periph.attach_sd(esp32p4::periph::SdPins::SOCKET, Box::new(Tap { inner: Box::new(sd), sdio: false, ops: bus.clone() }));
    esp_soc::SocBus::attach_usb_host(&mut p4.bus, Default::default()).unwrap();
    let mac = oxidrive_board::board_mac(&board_log);
    let id = format!("{:02x}{:02x}{:02x}", mac[3], mac[4], mac[5]);
    let forwards = vec![Forward { guest: smb_at.guest, host: smb.svc.addr() }, Forward { guest: mqtt_at.guest, host: mqtt.svc.addr() }];
    let mut cs = CoSim::new(p4, hosted_slave(Nat::forward_only(false, forwards)), Config::default());
    let periph = &mut cs.p4.bus.periph;
    let i = periph.sd_pins.iter().position(|&p| p == esp32p4::periph::SdPins::HOSTED).expect("the C6 on slot 1");
    let link = std::mem::replace(&mut periph.sdmmc.devices[i], Box::new(Nothing));
    periph.sdmmc.devices[i] = Box::new(Tap { inner: link, sdio: true, ops: bus.clone() });
    let pcs = std::env::var("LATENCY_PCS").is_ok_and(|v| v == "1").then(Vec::new);
    Rig { cs, smb, mqtt, card, id, lines: Vec::new(), bus, force, forced_at: None, pcs }
}

impl Rig {
    /// Run to `ns`, keeping every P4 console line with the time its slice ended, and forcing
    /// the pass when it is due.
    fn advance(&mut self, ns: u64) -> Result<(), String> {
        while self.cs.now_ns < ns {
            let r = self.cs.run_until((self.cs.now_ns + 100_000_000).min(ns));
            let t = self.cs.now_ns;
            for (c6, l) in self.cs.lines.drain(..) {
                if std::env::var("OXIDRIVE_ECHO").is_ok_and(|v| v == "1") { eprintln!("[{:8.3}] {}{l}", t as f64 / 1e9, if c6 { "[c6] " } else { "" }); }
                if !c6 { self.lines.push((t, l)); }
            }
            r.map_err(|s| format!("the P4 stopped at {:.3} s: {s:?}", t as f64 / 1e9))?;
            if let Some(p) = self.pcs.as_mut() {
                let core = |i: usize| self.cs.p4.cores.get(i).map_or((0, 0), |k| (k.pc, k.mstatus >> 3 & 1));
                let clic = &self.cs.p4.bus.periph.clic.core;
                p.push(Sample { t, pc: [core(0).0, core(1).0], mie: [core(0).1, core(1).1], thresh: [clic[0].thresh >> 24, clic[1].thresh >> 24] });
            }
            if self.forced_at.is_none() && t >= FORCE_AT_NS && self.p4_has("session ready") {
                self.forced_at = Some(t);
                if self.force { self.mqtt.client.publish(&format!("oxidrive/{}/cmd", self.id), "force_open").map_err(|e| format!("force_open: {e}"))?; }
            }
        }
        Ok(())
    }
    fn p4_has(&self, s: &str) -> bool { self.lines.iter().any(|(_, l)| l.contains(s)) }
}

impl esp_soc::usb_host::UsbPort for Rig {
    fn link(&mut self) -> Option<&mut esp_periph::usb_otg::UsbLink> { esp_soc::SocBus::usb_link(&mut self.cs.p4.bus) }
    fn run_until(&mut self, t_ns: u64) -> Result<(), String> { self.advance(t_ns) }
}

/// A card image file as a block device, read only.
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

fn env_u64(name: &str, default: u64) -> u64 { std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default) }

/// What one car WRITE(10) met: its time, and the bus time the shared reader and the SDIO link
/// took inside it.
struct Write { t: CmdTime, reader_ns: u64, sdio_ns: u64 }

/// The run's findings, printed as `[latency]` lines.
struct Report { writes: Vec<Write>, others: Vec<CmdTime>, reader: Vec<BusOp>, pass: Option<(u64, u64)>, window: (u64, u64) }

impl Report {
    /// Split the card's reads into the car's (inside one of its READ(10)s, at its blocks) and the
    /// firmware's own, and measure each WRITE(10). `cmds` are in submission order.
    fn new(cmds: &[CmdTime], bus: &[BusOp], window: (u64, u64), pass: Option<(u64, u64)>) -> Report {
        let reader: Vec<BusOp> = bus.iter().filter(|o| !o.sdio && is_read(o) && !car_read(cmds, o)).copied().collect();
        let spans = |ops: &mut dyn Iterator<Item = &BusOp>| ops.map(|o| (o.start_ns, o.end_ns.max(o.start_ns + 1))).collect::<Vec<_>>();
        let reader_spans = spans(&mut reader.iter());
        let sdio_spans = spans(&mut bus.iter().filter(|o| o.sdio));
        let writes = cmds.iter().filter(|c| c.op == WRITE10 && (window.0..window.1).contains(&c.submit_ns))
            .map(|&t| Write { t, reader_ns: overlap_ns(&reader_spans, t.submit_ns, t.done_ns), sdio_ns: overlap_ns(&sdio_spans, t.submit_ns, t.done_ns) }).collect();
        let others = cmds.iter().filter(|c| c.op != WRITE10 && (window.0..window.1).contains(&c.submit_ns)).copied().collect();
        Report { writes, others, reader, pass, window }
    }

    fn print(&self, label: &str, bus: &[BusOp]) {
        let ms = |ns: u64| ns as f64 / 1e6;
        let lat = |f: &dyn Fn(&Write) -> bool| Latency::new(self.writes.iter().filter(|w| f(w)).map(|w| w.t.latency_ns()).collect());
        let data = |w: &Write| w.t.blocks as usize * 512 == CHUNK;
        let in_pass = |w: &Write| self.pass.is_some_and(|(a, b)| (a..b).contains(&w.t.submit_ns));
        eprintln!("[latency] {label}: window {:.1}-{:.1} s; the shared pass {}", self.window.0 as f64 / 1e9, self.window.1 as f64 / 1e9,
                  self.pass.map_or("did not run".to_string(), |(a, b)| format!("{:.2}-{:.2} s", a as f64 / 1e9, b as f64 / 1e9)));
        for (what, f) in [("all WRITE(10)", &(|_: &Write| true) as &dyn Fn(&Write) -> bool), ("data WRITE(10) (64 KiB)", &data),
                          ("data, in the pass", &|w: &Write| data(w) && in_pass(w)), ("data, a reader transfer inside", &|w: &Write| data(w) && w.reader_ns > 0),
                          ("data, no reader transfer inside", &|w: &Write| data(w) && w.reader_ns == 0)] {
            let l = lat(f);
            eprintln!("[latency]   {what}: {}", l.summary());
            if !l.is_empty() { eprintln!("[latency]     histogram {}", l.histogram_line(&MS_EDGES)); }
        }
        let reads = Latency::new(self.others.iter().filter(|c| c.op == READ10).map(|c| c.latency_ns()).collect());
        eprintln!("[latency]   READ(10): {}", reads.summary());
        let all: Vec<u64> = self.writes.iter().map(|w| w.t.latency_ns()).chain(self.others.iter().map(|c| c.latency_ns())).collect();
        let stalled = |ms: u64| all.iter().filter(|&&v| v >= ms * 1_000_000).count();
        eprintln!("[latency]   any car command: n={} max {:.2} ms; at or over 20 ms: {}, 50 ms: {}, 100 ms: {}", all.len(), ms(all.iter().copied().max().unwrap_or(0)), stalled(20), stalled(50), stalled(100));
        let (mut run, mut longest) = (0, 0);
        for w in self.writes.iter().filter(|w| data(w)) { run = if w.reader_ns > 0 { run + 1 } else { 0 }; longest = longest.max(run); }
        eprintln!("[latency]   longest run of consecutive data WRITE(10)s with reader bus time inside: {longest}");
        let behind: Vec<u64> = self.writes.iter().filter(|w| data(w) && w.reader_ns > 0).map(|w| w.reader_ns).collect();
        let n_data = self.writes.iter().filter(|w| data(w)).count();
        let b = Latency::new(behind);
        eprintln!("[latency]   data WRITE(10)s with reader bus time inside: {} of {} ({:.1}%); that bus time median {:.3} p99 {:.3} max {:.3} ms",
                  b.len(), n_data, b.len() as f64 * 100.0 / n_data.max(1) as f64, ms(b.median()), ms(b.percentile(99.0)), ms(b.max()));
        let s = Latency::new(self.writes.iter().filter(|w| data(w)).map(|w| w.sdio_ns).collect());
        eprintln!("[latency]   SDIO bus time inside data WRITE(10)s: median {:.3} p99 {:.3} max {:.3} ms", ms(s.median()), ms(s.percentile(99.0)), ms(s.max()));
        let in_window: Vec<&BusOp> = self.reader.iter().filter(|o| (self.window.0..self.window.1).contains(&o.start_ns)).collect();
        let bytes: u64 = in_window.iter().filter(|o| o.answered).map(|o| o.bytes as u64).sum();
        let mut sizes: Vec<(u8, u32, usize)> = Vec::new();
        for o in &in_window {
            let blocks = o.bytes.div_ceil(512);
            match sizes.iter_mut().find(|s| s.0 == o.cmd && s.1 == blocks) { Some(s) => s.2 += 1, None => sizes.push((o.cmd, blocks, 1)) }
        }
        sizes.sort();
        let secs = |(a, b): (u64, u64)| (b - a) as f64 / 1e9;
        let pass_bytes: u64 = self.pass.map_or(0, |(a, b)| self.reader.iter().filter(|o| o.answered && (a..b).contains(&o.start_ns)).map(|o| o.bytes as u64).sum());
        eprintln!("[latency]   firmware card reads in the window: {} commands, {} bytes; in the pass {} bytes, {:.3} MB/s over it",
                  in_window.len(), bytes, pass_bytes, self.pass.map_or(0.0, |p| pass_bytes as f64 / 1e6 / secs(p).max(1e-9)));
        eprintln!("[latency]   their sizes (CMD, blocks: count): {}", sizes.iter().map(|(c, b, n)| format!("CMD{c} {b}: {n}")).collect::<Vec<_>>().join(", "));
        let unanswered: Vec<&BusOp> = self.reader.iter().filter(|o| !o.answered).collect();
        eprintln!("[latency]   firmware reads the card did not answer: {}", unanswered.len());
        for o in unanswered.iter().take(10) {
            // the status the next command on the card was answered with
            let next = bus.iter().find(|n| !n.sdio && n.start_ns >= o.start_ns && n.answered);
            eprintln!("[latency]     t={:.6} CMD{} arg {:#x}; next: {}", o.start_ns as f64 / 1e9, o.cmd, o.arg,
                      next.map_or("none".to_string(), |n| format!("CMD{} at {:.6} answered {:#x}", n.cmd, n.start_ns as f64 / 1e9, n.resp)));
        }
    }

    /// Per-write, per-read and per-card-command rows for offline analysis, when `LATENCY_OUT`
    /// names a directory.
    fn save(&self, dir: &std::path::Path, label: &str, bus: &[BusOp], cmds: &[CmdTime], pcs: &[Sample]) {
        std::fs::create_dir_all(dir).unwrap();
        let mut w = String::from("submit_ns\tlatency_ns\tlba\tblocks\tok\treader_ns\tsdio_ns\n");
        for x in &self.writes { w += &format!("{}\t{}\t{}\t{}\t{}\t{}\t{}\n", x.t.submit_ns, x.t.latency_ns(), x.t.lba, x.t.blocks, x.t.ok as u8, x.reader_ns, x.sdio_ns); }
        std::fs::write(dir.join(format!("{label}.writes.tsv")), w).unwrap();
        let mut r = String::from("start_ns\tend_ns\tcmd\targ\tbytes\tanswered\n");
        for o in &self.reader { r += &format!("{}\t{}\t{}\t{}\t{}\t{}\n", o.start_ns, o.end_ns, o.cmd, o.arg, o.bytes, o.answered as u8); }
        std::fs::write(dir.join(format!("{label}.reads.tsv")), r).unwrap();
        let mut c = String::from("start_ns\tend_ns\tcmd\targ\tbytes\tanswered\tresp\n");
        for o in bus.iter().filter(|o| !o.sdio) { c += &format!("{}\t{}\t{}\t{}\t{}\t{}\t{:#x}\n", o.start_ns, o.end_ns, o.cmd, o.arg, o.bytes, o.answered as u8, o.resp); }
        std::fs::write(dir.join(format!("{label}.card.tsv")), c).unwrap();
        let mut m = String::from("submit_ns\tlatency_ns\top\tlba\tblocks\tok\n");
        for t in cmds { m += &format!("{}\t{}\t{}\t{}\t{}\t{}\n", t.submit_ns, t.latency_ns(), t.op, t.lba, t.blocks, t.ok as u8); }
        std::fs::write(dir.join(format!("{label}.cmds.tsv")), m).unwrap();
        if !pcs.is_empty() {
            let mut p = String::from("t_ns\tpc0\tpc1\tmie0\tmie1\tthresh0\tthresh1\n");
            for x in pcs { p += &format!("{}\t{:08x}\t{:08x}\t{}\t{}\t{}\t{}\n", x.t, x.pc[0], x.pc[1], x.mie[0], x.mie[1], x.thresh[0], x.thresh[1]); }
            std::fs::write(dir.join(format!("{label}.pcs.tsv")), p).unwrap();
        }
    }
}

fn is_read(o: &BusOp) -> bool { matches!(o.cmd, CMD_READ_SINGLE | CMD_READ_MULTIPLE) }

/// The car's command in flight at `t`, if any: the last one submitted at or before it.
fn car_command_at(cmds: &[CmdTime], t: u64) -> Option<&CmdTime> {
    cmds[..cmds.partition_point(|c| c.submit_ns <= t)].last().filter(|c| t <= c.done_ns)
}

/// A card read made for one of the car's READ(10)s.
fn car_read(cmds: &[CmdTime], o: &BusOp) -> bool {
    car_command_at(cmds, o.start_ns).is_some_and(|c| c.op == READ10 && (c.lba..c.lba + c.blocks as u32).contains(&o.arg))
}

/// Saved events the car records before it starts the long clip; the firmware holds the newest
/// back for one pass and reads the others alongside the car.
const EVENTS: [&str; 3] = ["2026-01-01_00-00-00", "2026-01-01_00-01-00", "2026-01-01_00-02-00"];
const RECENT: &str = "TeslaCam/RecentClips/2026-01-01_00-03-00-front.mp4";

/// The scenario: the car records the saved events, then one long clip at the camera's pace
/// through the window; `shared` forces a pass at `FORCE_AT_NS`, otherwise none runs before the
/// window ends. `OXIDRIVE_SD_PRG_US` (1000) is the card's programming time,
/// `LATENCY_SECONDS` (40) the window, `OXIDRIVE_EVENT_MIB` (4) each saved event's size,
/// `LATENCY_LABEL` names the run and `LATENCY_OUT` a directory for its rows. `fixed` asserts what
/// a reader that shares a lock with the car's writes must do (`FixedChecks`).
fn car_write_latency(dir: &str, shared: bool, fixed: bool) {
    use esp_soc::usb_host::{fat32::{mp4_bytes, write_clip, Fat32}, msc::Msc, UsbHost};
    let prg_ns = env_u64("OXIDRIVE_SD_PRG_US", 1000) * 1000;
    let window_ns = env_u64("LATENCY_SECONDS", 40) * 1_000_000_000;
    let event_mib = env_u64("OXIDRIVE_EVENT_MIB", 4) as usize;
    let label = std::env::var("LATENCY_LABEL").unwrap_or_else(|_| if shared { "shared".into() } else { "alone".into() });
    let mut r = rig(dir, prg_ns, shared);
    eprintln!("[latency] {label}: the card timed on a timed bus, programming {} us after each write", prg_ns / 1000);
    let events: Vec<Vec<u8>> = EVENTS.iter().map(|s| mp4_bytes(s, event_mib << 20)).collect();
    let wall = std::time::Instant::now();
    let (clip, window, stats, cmds);
    {
        let mut host = UsbHost::new(&mut r);
        host.enumerate(10_000_000_000).unwrap_or_else(|e| panic!("enumerate: {e}"));
        let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
        msc.wait_ready(100_000_000, 10_000_000_000).unwrap_or_else(|e| panic!("never ready: {e}"));
        msc.read_capacity().unwrap();
        let mut fs = Fat32::mount(&mut msc).unwrap();
        for (stamp, clip) in EVENTS.iter().zip(&events) {
            let path = format!("TeslaCam/SavedClips/{stamp}/event.mp4");
            fs.mkdir_all(&mut msc, path.rsplit_once('/').unwrap().0).unwrap();
            write_clip(&mut fs, &mut msc, &path, clip, CHUNK, GAP_NS, 1).unwrap_or_else(|e| panic!("{path}: {e}"));
        }
        // The long clip runs past the window's end with the session up to 11 s late.
        let start = msc.host.now_ns();
        let end = FORCE_AT_NS + 12_000_000_000 + window_ns;
        let chunks = (end - start).div_ceil(GAP_NS) as usize;
        clip = mp4_bytes(RECENT, chunks * CHUNK);
        fs.mkdir_all(&mut msc, RECENT.rsplit_once('/').unwrap().0).unwrap();
        msc.times = Some(Vec::with_capacity(chunks * 8));
        stats = write_clip(&mut fs, &mut msc, RECENT, &clip, CHUNK, GAP_NS, 1).unwrap_or_else(|e| panic!("{RECENT}: {e}"));
        cmds = msc.times.take().unwrap();
        drop(msc);
        let at = r.forced_at.expect("the MQTT session never came up");
        window = (at + 500_000_000, at + 500_000_000 + window_ns);
        // The firmware's own first pass is due 60 s after boot, the next one 60 s after a forced one.
        assert!(window.1 < 60_000_000_000, "the window ends at {:.1} s, past the firmware's own pass at 60 s: lower LATENCY_SECONDS", window.1 as f64 / 1e9);
        assert!(cmds.last().is_some_and(|c| c.submit_ns >= window.1), "the clip ended before the window did");
        // Past the window: the pass's summary, if it runs that long.
        let t = r.cs.now_ns + 30_000_000_000;
        while r.cs.now_ns < t && shared && !r.p4_has("scan: shared") { let n = r.cs.now_ns + 1_000_000_000; r.advance(n).unwrap_or_else(|e| panic!("{e}")); }
    }
    eprintln!("[latency] {label}: prg {} us, {:.1} s wall for {:.1} s emulated; the car: {}", prg_ns / 1000, wall.elapsed().as_secs_f64(), r.cs.now_ns as f64 / 1e9, stats.summary(GAP_NS));
    if r.pcs.is_some() {
        // the SDMMC (23) and USB-OTG (93) sources: each core's CPU line and its CLIC control word
        let p = &r.cs.p4.bus.periph;
        for src in [23usize, 93] { eprintln!("[latency] source {src}: {:?}", [0, 1].map(|c| { let line = p.intmtx.map[c][src]; (line, p.clic.core[c].ctrl.get(line as usize + 16).copied()) })); }
    }
    let bus = r.bus.borrow().clone();
    let forced = r.forced_at.unwrap();
    // The pass: from the firmware's first card read after the force to its `scan: shared` line,
    // or to the run's end while it still reads; none if it read once and stopped.
    let reads: Vec<u64> = bus.iter().filter(|o| !o.sdio && o.start_ns > forced && is_read(o) && !car_read(&cmds, o)).map(|o| o.start_ns).collect();
    let pass_end = r.lines.iter().find(|(t, l)| *t > forced && l.contains("scan: shared")).map(|l| l.0);
    let pass = match (reads.first(), pass_end) {
        (Some(&a), Some(b)) => Some((a, b)),
        (Some(&a), None) if reads.last() > Some(&(a + 1_000_000_000)) => Some((a, r.cs.now_ns)),
        _ => None,
    };
    let report = Report::new(&cmds, &bus, window, pass.filter(|_| shared));
    report.print(&label, &bus);
    if let Some(dir) = std::env::var_os("LATENCY_OUT") { report.save(std::path::Path::new(&dir), &label, &bus, &cmds, r.pcs.as_deref().unwrap_or(&[])); }
    let all = std::env::var("LATENCY_CONSOLE").is_ok_and(|v| v == "1");
    let shown = |l: &str| all || ["shared", "0x107", "sdmmc", "smb: ", "upload", "scan:"].iter().any(|k| l.contains(k));
    for (t, l) in r.lines.iter().filter(|(t, l)| *t + 1_000_000_000 > forced && shown(l)).take(if all { usize::MAX } else { 60 }) {
        eprintln!("[latency]   {:8.3} {l}", *t as f64 / 1e9);
    }
    // The card afterwards: every clip as the car wrote it.
    let mut disk = FileDisk(std::fs::File::open(&r.card).unwrap());
    let fs = Fat32::mount(&mut disk).unwrap();
    assert!(fs.read_path(&mut disk, RECENT).unwrap() == clip, "the long clip on the image differs from what the car wrote");
    for (stamp, ev) in EVENTS.iter().zip(&events) {
        assert!(fs.read_path(&mut disk, &format!("TeslaCam/SavedClips/{stamp}/event.mp4")).unwrap() == *ev, "{stamp}: the event on the image differs");
    }
    let data_writes = report.writes.iter().filter(|w| w.t.blocks as usize * 512 == CHUNK).count();
    assert!(data_writes as u64 >= window_ns / GAP_NS - 2, "{data_writes} data WRITE(10)s timed in the window");
    assert!(report.writes.iter().all(|w| w.t.ok), "a WRITE(10) failed");
    if shared { assert!(!reads.is_empty(), "the firmware never read the card after the force"); } else { assert!(report.reader.iter().all(|o| o.start_ns < window.0 || o.start_ns >= window.1), "the firmware read the card in the control's window"); }
    if std::env::var("LATENCY_REPORT").is_ok_and(|v| v == "1") { car_report(&mut r); }
    if fixed {
        let after: Vec<&str> = r.lines.iter().filter(|(t, _)| *t > forced).map(|(_, l)| l.as_str()).collect();
        let held_back = EVENTS.len() - 1;
        let archived = EVENTS[..held_back].iter().zip(&events)
            .filter(|(stamp, ev)| std::fs::read(r.smb.path().join(format!("SavedClips/{stamp}/event.mp4"))).is_ok_and(|b| b == **ev)).count();
        let checks = FixedChecks { report: &report, pass_ended: pass_end.is_some(), stopped_early: after.iter().any(|l| l.contains("stopped early")), archived, expected: held_back };
        eprintln!("[latency] {label}: {}", checks.summary());
        if let Err(e) = checks.verdict() { panic!("{label}: {e}; {}", checks.summary()); }
    }
}

/// `LATENCY_REPORT=1`: what the firmware itself counted over the run, from its car report
/// (oxidrive's `car_report`): idle past the minute after which it saves the report, cut the
/// power, and read the boot's report from the broker when the next boot publishes it. Its lines
/// from the C components (`H_SDIO_DRV: task still writing Rx data to queue!` and the rest) are
/// counted from its log hook, which the console does not show past the start of `main`.
fn car_report(r: &mut Rig) {
    let idle = r.cs.now_ns + 80_000_000_000;
    r.advance(idle).unwrap_or_else(|e| panic!("{e}"));
    let cut = r.lines.len();
    esp_soc::SocBus::request_reset(&mut r.cs.p4.bus, esp_periph::RST_POWERON);
    let by = r.cs.now_ns + 120_000_000_000;
    while r.cs.now_ns < by && !r.lines[cut..].iter().any(|(_, l)| l.contains("car_report: car 1/")) {
        let t = r.cs.now_ns + 1_000_000_000;
        r.advance(t).unwrap_or_else(|e| panic!("{e}"));
    }
    let topic = format!("oxidrive/{}/car_report/1", r.id);
    let got = r.mqtt.client.retained(&[&topic], std::time::Duration::from_secs(1)).unwrap();
    let Some((_, doc)) = got.into_iter().next() else { eprintln!("[latency] no car report published"); return };
    let j = esp_soc::json::parse_json(&doc).unwrap_or_else(|e| panic!("{e}: {doc}"));
    let num = |path: &str| {
        let mut at = &j;
        for k in path.split('.') { match at.get(k) { Some(v) => at = v, None => return None } }
        at.as_i64()
    };
    let fields = ["radio.rxq_full", "radio.sdio_errors", "radio.dead", "radio.link_lost", "net.smb_stalls", "net.mqtt_reconnects", "health.heap_min", "health.psram_min", "writes.max_us", "writes.slow"];
    eprintln!("[latency] car report: {}", fields.iter().map(|f| format!("{f} {}", num(f).map_or("-".into(), |v| v.to_string()))).collect::<Vec<_>>().join(", "));
}

/// What a shared reader that shares a lock with the car's writes, and keeps libsmb2 off its core,
/// must do (oxidrive's fix-shared-reader): every card read it sends is answered (none meets the
/// card programming, `0x107`), no car command takes 100 ms or more, and the pass runs to its end
/// with the events it was offered archived as the car wrote them.
struct FixedChecks<'a> { report: &'a Report, pass_ended: bool, stopped_early: bool, archived: usize, expected: usize }

impl FixedChecks<'_> {
    fn unanswered(&self) -> usize { self.report.reader.iter().filter(|o| !o.answered).count() }
    fn slow(&self) -> Vec<CmdTime> {
        self.report.writes.iter().map(|w| w.t).chain(self.report.others.iter().copied()).filter(|c| c.latency_ns() >= 100_000_000).collect()
    }
    fn summary(&self) -> String {
        format!("fixed-reader checks: {} firmware reads unanswered, {} car commands of 100 ms or more, the pass {}{}, {} of {} events archived",
                self.unanswered(), self.slow().len(), if self.pass_ended { "ended" } else { "did not end" }, if self.stopped_early { " early" } else { "" }, self.archived, self.expected)
    }
    fn verdict(&self) -> Result<(), String> {
        if self.unanswered() > 0 { return Err("the card did not answer a firmware read: it met the card programming the car's write".into()); }
        if !self.slow().is_empty() { return Err(format!("car commands of 100 ms or more: {:?}", self.slow())); }
        if !self.pass_ended || self.stopped_early { return Err("the pass did not run to its end".into()); }
        if self.archived != self.expected { return Err("not every event offered was archived as the car wrote it".into()); }
        Ok(())
    }
}

/// The firmware `fixed` holds to: `OXIDRIVE_READER_DIR`, else oxidrive's fix-shared-reader build in
/// the fixtures.
fn reader_dir() -> String {
    std::env::var("OXIDRIVE_READER_DIR").unwrap_or_else(|_| format!("{}/Code/dlp/sim-fixtures/esp32p4/oxidrive-fix-reader", std::env::var("HOME").unwrap_or_default()))
}

/// The shared pass reading alongside the car; `OXIDRIVE_DIR` picks the firmware (a reader chunk
/// variant or the baseline).
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_car_write_latency_under_a_shared_reader() { car_write_latency(&oxidrive_board::dir(), true, false); }

/// The control: the same car and firmware, no pass in the window.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_car_write_latency_alone() { car_write_latency(&oxidrive_board::dir(), false, false); }

/// The shared pass of oxidrive's fix-shared-reader (`OXIDRIVE_READER_DIR`): no read meets the card
/// programming, no car command of 100 ms or more, and the pass archives both events it was
/// offered. oxidrive 728b71f fails it at every `OXIDRIVE_SD_PRG_US`. At 2000 the pass's summary
/// came at 62 s (74 s with the SMB thread pinned to core 1), inside the 30 s the run waits after
/// the car's last write (61 s); a slower SMB path could push it past that.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under OXIDRIVE_READER_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_a_shared_pass_completes_without_racing_the_car() { car_write_latency(&reader_dir(), true, true); }

/// The analysis on a made-up run: the car's READ(10) transfers are not the reader's, and a
/// WRITE(10) counts the reader's and the link's bus time inside it.
#[test]
fn the_reader_is_what_the_car_did_not_ask_for() {
    let cmd = |op, lba, blocks, submit_ns, done_ns| CmdTime { op, lba, blocks, submit_ns, done_ns, ok: true, sense: None };
    let op = |sdio, cmd, arg, start_ns, end_ns| BusOp { sdio, cmd, arg, start_ns, end_ns, bytes: ((end_ns - start_ns) / 10) as u32, answered: true, resp: 0 };
    let cmds = [cmd(READ10, 50, 1, 0, 100), cmd(WRITE10, 1000, 128, 200, 1000), cmd(WRITE10, 2000, 128, 1200, 2000)];
    let bus = [op(false, CMD_READ_SINGLE, 50, 10, 20), op(false, CMD_READ_MULTIPLE, 9000, 150, 300), op(false, CMD_WRITE_MULTIPLE, 1000, 400, 900),
               op(true, 53, 0, 500, 550), op(false, CMD_READ_SINGLE, 60, 1500, 1520)];
    let rep = Report::new(&cmds, &bus, (0, 1500), Some((150, 1600)));
    assert_eq!(rep.reader.iter().map(|o| o.arg).collect::<Vec<_>>(), [9000, 60], "the car's own READ(10) at block 50 is not the reader's");
    assert_eq!(rep.writes.len(), 2, "both WRITE(10)s start in the window");
    assert_eq!(rep.writes.iter().map(|w| (w.reader_ns, w.sdio_ns)).collect::<Vec<_>>(), [(100, 50), (20, 0)]);
}

/// The fixed-reader checks on made-up runs: each of an unanswered read, a slow car command, a pass
/// that stopped early and a missing event fails them.
#[test]
fn the_fixed_reader_checks_fail_on_each_hazard() {
    let cmd = |op, submit_ns, done_ns| CmdTime { op, lba: 1000, blocks: 128, submit_ns, done_ns, ok: true, sense: None };
    let read = |answered| BusOp { sdio: false, cmd: CMD_READ_MULTIPLE, arg: 9000, start_ns: 10, end_ns: 20, bytes: 32768, answered, resp: 0 };
    let report = |slow: bool, answered| Report::new(&[cmd(WRITE10, 0, if slow { 100_000_000 } else { 2_620_000 })], &[read(answered)], (0, 1_000_000_000), Some((0, 1_000_000_000)));
    let fails = |c: FixedChecks| c.verdict().is_err();
    let (good, slow, refused) = (report(false, true), report(true, true), report(false, false));
    let checks = |report, pass_ended, stopped_early, archived| FixedChecks { report, pass_ended, stopped_early, archived, expected: 2 };
    assert!(!fails(checks(&good, true, false, 2)), "a clean run passes");
    assert!(fails(checks(&refused, true, false, 2)), "an unanswered read");
    assert!(fails(checks(&slow, true, false, 2)), "a car command of 100 ms");
    assert!(fails(checks(&good, false, false, 2)), "a pass that never ended");
    assert!(fails(checks(&good, true, true, 2)), "a pass that stopped early");
    assert!(fails(checks(&good, true, false, 1)), "an event not archived");
}
