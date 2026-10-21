//! The host against a mass-storage device written in Rust at the transaction level: a fake that
//! answers like TinyUSB's MSC class, with knobs for the failures BOT's host side must handle.
#[path = "../../../tests/sd_image.rs"]
mod sd_image;
use super::fat32::{clip_bytes, write_clip, Fat32, MemDisk};
use super::msc::{op, sense, Dir, Msc, MscError, Recovery, Sense};
use super::*;
use emu_core::Fault;
use esp_periph::usb_otg::{BusEvent, DmaMemory, Handshake, InReply, UsbFunction, UsbTiming};

struct NoMem;
impl DmaMemory for NoMem {
    fn read(&mut self, _: u32, _: &mut [u8]) -> Result<(), Fault> { Err(Fault::Unmapped) }
    fn write(&mut self, _: u32, _: &[u8]) -> Result<(), Fault> { Err(Fault::Unmapped) }
}

#[derive(Debug)]
enum Bot { Idle, DataIn(Vec<u8>, usize), DataOut(usize, Vec<u8>), Status([u8; 13]) }

#[derive(Default)]
struct Knobs { not_ready: u32, phase_on: Option<u8>, bad_tag_on: Option<u8>, short_on: Option<(u8, usize)>, nak_in: u32, zlp_before_csw_on: Option<u8> }

struct FakeMsc {
    disk: Vec<u8>,
    vbus: bool,
    setup: [u8; 8],
    ctrl_in: Vec<u8>,
    ctrl_pos: usize,
    ctrl_stall: bool,
    ep0_pid: bool,
    configured: bool,
    /// next data PID: [OUT, IN] of endpoint 1
    pid: [bool; 2],
    halt: [bool; 2],
    bot: Bot,
    tag: u32,
    want: u32,
    sense: Sense,
    knobs: Knobs,
    bot_resets: u32,
    clear_halts: u32,
    commands: Vec<u8>,
    write_lba: usize,
    writes: Vec<(usize, usize)>,
    bus_resets: u32,
    /// stop answering bulk IN after this many more IN data packets
    hang_after: Option<u32>,
    /// the port's time, set before each service
    now: u64,
    /// when the application holds the card
    windows: Vec<FakeWindow>,
    /// when each WRITE(10) of `writes` came
    write_at: Vec<u64>,
    /// Serve `windows` as oxidrive's gated esp_tinyusb does: READ CAPACITY, READ(10) and WRITE(10)
    /// refused with MEDIUM NOT PRESENT as TEST UNIT READY is, and after each window UNIT ATTENTION
    /// 6/28/00 for the first command but INQUIRY and REQUEST SENSE.
    gated: bool,
    /// windows whose unit attention has been reported
    attended: usize,
}

/// A window as esp_tinyusb serves one: while the application holds the card TEST UNIT READY
/// answers MEDIUM NOT PRESENT, and READ(10) and WRITE(10) still reach the card (tinyusb_msc.c
/// `tud_msc_test_unit_ready_cb`, `tud_msc_read10_cb`, `tud_msc_write10_cb`). `app` changes the card
/// once, at the window's start.
struct FakeWindow { from: u64, to: u64, app: Option<AppWrite> }
/// What the application does to the card's bytes.
type AppWrite = Box<dyn FnMut(&mut Vec<u8>)>;

const DEVICE: [u8; 18] = [18, 1, 0x00, 0x02, 0xef, 2, 1, 64, 0x3a, 0x30, 0x02, 0x40, 0x00, 0x01, 1, 2, 3, 1];
fn config() -> Vec<u8> {
    let mut c = vec![9, 2, 32, 0, 1, 1, 0, 0xa0, 50];
    c.extend_from_slice(&[9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0]);
    c.extend_from_slice(&[7, 5, 0x01, 2, 0x00, 0x02, 0]);
    c.extend_from_slice(&[7, 5, 0x81, 2, 0x00, 0x02, 0]);
    c
}
fn string(s: &str) -> Vec<u8> { let mut v = vec![2 + 2 * s.len() as u8, 3]; for u in s.encode_utf16() { v.extend_from_slice(&u.to_le_bytes()); } v }

impl FakeMsc {
    fn new(blocks: usize) -> Self {
        FakeMsc { disk: vec![0; blocks * 512], vbus: false, setup: [0; 8], ctrl_in: Vec::new(), ctrl_pos: 0, ctrl_stall: false, ep0_pid: true, configured: false,
                  pid: [false; 2], halt: [false; 2], bot: Bot::Idle, tag: 0, want: 0, sense: Sense::default(), knobs: Knobs::default(), bot_resets: 0, clear_halts: 0, commands: Vec::new(), write_lba: 0, writes: Vec::new(), bus_resets: 0, hang_after: None, now: 0, windows: Vec::new(), write_at: Vec::new(), gated: false, attended: 0 }
    }
    fn csw(&self, residue: u32, status: u8) -> [u8; 13] {
        let mut c = [0u8; 13];
        c[..4].copy_from_slice(&msc::CSW_SIGNATURE.to_le_bytes());
        c[4..8].copy_from_slice(&self.tag.to_le_bytes());
        c[8..12].copy_from_slice(&residue.to_le_bytes());
        c[12] = status;
        c
    }
    fn fail(&mut self, key: u8, asc: u8, data_in: bool) {
        self.sense = Sense { key, asc, ascq: 0 };
        if data_in && self.want > 0 { self.halt[1] = true; }
        self.bot = Bot::Status(self.csw(self.want, 1));
    }
    fn cbw(&mut self, d: &[u8]) {
        if d.len() != 31 || d[..4] != msc::CBW_SIGNATURE.to_le_bytes() { self.halt = [true, true]; return; }
        self.tag = u32::from_le_bytes(d[4..8].try_into().unwrap());
        self.want = u32::from_le_bytes(d[8..12].try_into().unwrap());
        let (dir_in, cb) = (d[12] & 0x80 != 0, &d[15..31]);
        let opc = cb[0];
        self.commands.push(opc);
        let lba = u32::from_be_bytes(cb[2..6].try_into().unwrap()) as usize;
        let n = u16::from_be_bytes([cb[7], cb[8]]) as usize;
        if self.knobs.phase_on == Some(opc) { self.bot = Bot::Status(self.csw(0, 2)); return; }
        if self.gated {
            let ended = self.windows.iter().filter(|w| w.to <= self.now).count();
            if ended > self.attended && !matches!(opc, op::INQUIRY | op::REQUEST_SENSE) { self.attended = ended; return self.fail(sense::UNIT_ATTENTION, 0x28, dir_in); }
            let held = self.windows.iter().any(|w| (w.from..w.to).contains(&self.now));
            if held && matches!(opc, op::READ_CAPACITY10 | op::READ10 | op::WRITE10) { return self.fail(sense::NOT_READY, sense::MEDIUM_NOT_PRESENT, dir_in); }
        }
        let reply = match opc {
            op::TEST_UNIT_READY => {
                if self.knobs.not_ready > 0 { self.knobs.not_ready -= 1; return self.fail(sense::NOT_READY, sense::MEDIUM_NOT_PRESENT, false); }
                if self.windows.iter().any(|w| (w.from..w.to).contains(&self.now)) { return self.fail(sense::NOT_READY, sense::MEDIUM_NOT_PRESENT, false); }
                None
            }
            op::REQUEST_SENSE => { let mut s = vec![0u8; 18]; s[0] = 0x70; s[2] = self.sense.key; s[7] = 10; s[12] = self.sense.asc; s[13] = self.sense.ascq; self.sense = Sense::default(); Some(s) }
            op::INQUIRY => { let mut s = vec![0u8; 36]; s[1] = 0x80; s[4] = 31; s[8..16].copy_from_slice(b"TinyUSB "); s[16..32].copy_from_slice(b"TEST MSC Storage"); s[32..36].copy_from_slice(b"0.1 "); Some(s) }
            op::READ_CAPACITY10 => { let mut s = ((self.disk.len() / 512 - 1) as u32).to_be_bytes().to_vec(); s.extend_from_slice(&512u32.to_be_bytes()); Some(s) }
            op::MODE_SENSE6 => Some(vec![3, 0, 0, 0]),
            op::PREVENT_ALLOW | op::START_STOP_UNIT | op::SYNCHRONIZE_CACHE10 => None,
            op::READ10 => { if (lba + n) * 512 > self.disk.len() { return self.fail(sense::ILLEGAL_REQUEST, 0x21, true); } Some(self.disk[lba * 512..(lba + n) * 512].to_vec()) }
            op::WRITE10 => { self.write_lba = lba; self.writes.push((lba, n)); self.write_at.push(self.now); self.bot = Bot::DataOut(n * 512, Vec::new()); return; }
            _ => return self.fail(sense::ILLEGAL_REQUEST, sense::INVALID_OPCODE, dir_in),
        };
        match reply {
            Some(mut d) if dir_in => {
                d.truncate(self.want as usize);
                if let Some((o, k)) = self.knobs.short_on { if o == opc { d.truncate(k); } }
                if d.is_empty() { let r = self.want; self.finish(r, 0); } else { self.bot = Bot::DataIn(d, 0); }
            }
            _ => self.finish(self.want, 0),
        }
    }
    fn finish(&mut self, residue: u32, status: u8) {
        let mut c = self.csw(residue, status);
        if let Some(o) = self.knobs.bad_tag_on { if self.commands.last() == Some(&o) { c[4] ^= 1; } }
        self.bot = Bot::Status(c);
    }
    fn control(&mut self) {
        let s = self.setup;
        let (value, index, len) = (u16::from_le_bytes([s[2], s[3]]), u16::from_le_bytes([s[4], s[5]]), u16::from_le_bytes([s[6], s[7]]) as usize);
        self.ctrl_stall = false;
        self.ctrl_in.clear();
        self.ctrl_pos = 0;
        match (s[0], s[1]) {
            (0x80, req::GET_DESCRIPTOR) => match (value >> 8) as u8 {
                req::DEVICE => self.ctrl_in = DEVICE.to_vec(),
                req::CONFIGURATION => self.ctrl_in = config(),
                req::STRING => self.ctrl_in = match value & 0xff { 0 => vec![4, 3, 9, 4], 1 => string("TinyUSB"), 2 => string("TinyUSB Device"), 3 => string("123456"), _ => { self.ctrl_stall = true; vec![] } },
                _ => self.ctrl_stall = true,
            },
            (0x00, req::SET_ADDRESS) => {}
            (0x00, req::SET_CONFIGURATION) => { self.configured = value == 1; self.pid = [false; 2]; }
            (0x02, req::CLEAR_FEATURE) => { let i = (index >> 7) as usize & 1; self.halt[i] = false; self.pid[i] = false; self.clear_halts += 1; }
            (0xa1, msc::GET_MAX_LUN) => self.ctrl_in = vec![0],
            (0x40, 0x01) => {}
            // vendor: NAK bulk IN after wValue more packets, until a bus reset
            (0x40, 0x02) => self.hang_after = Some(u16::from_le_bytes([self.setup[2], self.setup[3]]) as u32),
            (0x21, msc::BULK_ONLY_RESET) => { self.bot = Bot::Idle; self.bot_resets += 1; }
            _ => self.ctrl_stall = true,
        }
        self.ctrl_in.truncate(len);
    }
}

impl UsbFunction for FakeMsc {
    fn vbus(&mut self, _now: u64, on: bool) { self.vbus = on; }
    fn pullup(&self) -> bool { self.vbus }
    fn speed_capability(&self) -> Speed { Speed::High }
    fn bus_reset(&mut self, _now: u64) { self.configured = false; self.bot = Bot::Idle; self.bus_resets += 1; self.hang_after = None; }
    fn enumerated(&mut self, _now: u64, _speed: Speed) {}
    fn bus_event(&mut self, _now: u64, _e: BusEvent) {}
    fn setup(&mut self, _mem: &mut dyn DmaMemory, pkt: [u8; 8]) { self.setup = pkt; self.ep0_pid = true; self.control(); }
    fn token_in(&mut self, _mem: &mut dyn DmaMemory, ep: u8, max: usize) -> InReply {
        if ep == 0 {
            if self.ctrl_stall { return InReply::Stall; }
            let n = (self.ctrl_in.len() - self.ctrl_pos).min(max);
            let d = self.ctrl_in[self.ctrl_pos..self.ctrl_pos + n].to_vec();
            self.ctrl_pos += n;
            let pid = self.ep0_pid;
            self.ep0_pid = !pid;
            return InReply::Data(d, pid);
        }
        if self.halt[1] { return InReply::Stall; }
        if self.knobs.nak_in > 0 { self.knobs.nak_in -= 1; return InReply::Nak; }
        match &mut self.hang_after { Some(0) => return InReply::Nak, Some(n) => *n -= 1, None => {} }
        let pid = self.pid[1];
        let reply = match &mut self.bot {
            Bot::DataIn(d, pos) => {
                let n = (d.len() - *pos).min(max);
                let chunk = d[*pos..*pos + n].to_vec();
                *pos += n;
                if *pos == d.len() {
                    let r = self.want - d.len() as u32;
                    // less than asked for, ending on a full packet: STALL to end the data stage (BOT §6.7.2)
                    if r > 0 && n == max { self.halt[1] = true; }
                    self.finish(r, 0);
                }
                chunk
            }
            Bot::Status(_) if self.knobs.zlp_before_csw_on.is_some() && self.knobs.zlp_before_csw_on == self.commands.last().copied() => { self.knobs.zlp_before_csw_on = None; Vec::new() }
            Bot::Status(c) => { let c = c.to_vec(); self.bot = Bot::Idle; c }
            _ => return InReply::Nak,
        };
        self.pid[1] = !pid;
        InReply::Data(reply, pid)
    }
    fn token_out(&mut self, _mem: &mut dyn DmaMemory, ep: u8, data1: bool, data: &[u8]) -> Handshake {
        if ep == 0 { return if self.ctrl_stall { Handshake::Stall } else { Handshake::Ack }; }
        if self.halt[0] { return Handshake::Stall; }
        if data1 != self.pid[0] { return Handshake::Ack; }
        self.pid[0] = !data1;
        match &mut self.bot {
            Bot::Idle => self.cbw(data),
            Bot::DataOut(need, got) => {
                got.extend_from_slice(data);
                if got.len() >= *need {
                    let lba = self.write_lba;
                    let d = std::mem::take(got);
                    self.disk[lba * 512..lba * 512 + d.len()].copy_from_slice(&d);
                    self.finish(0, 0);
                }
            }
            _ => return Handshake::Stall,
        }
        Handshake::Ack
    }
}
struct FakePort { link: UsbLink, dev: FakeMsc, now: u64, pullup: bool }
impl FakePort {
    fn new(dev: FakeMsc) -> Self { FakePort { link: UsbLink::new(UsbTiming::default(), 1_000_000_000, 0), dev, now: 0, pullup: false } }
}
impl UsbPort for FakePort {
    fn link(&mut self) -> Option<&mut UsbLink> { Some(&mut self.link) }
    fn run_until(&mut self, t_ns: u64) -> Result<(), String> {
        loop {
            self.dev.now = self.now;
            for w in self.dev.windows.iter_mut().filter(|w| w.from <= self.now) { if let Some(mut app) = w.app.take() { app(&mut self.dev.disk); } }
            if self.link.tick_to(self.now) { self.link.service(self.now, &mut self.dev, &mut NoMem); }
            let p = self.dev.pullup();
            if p != self.pullup { self.pullup = p; self.link.device_pullup(self.now, p); continue; }
            if self.now >= t_ns { return Ok(()); }
            let next_window = self.dev.windows.iter().filter(|w| w.app.is_some() && w.from > self.now).map(|w| w.from).min().unwrap_or(u64::MAX);
            self.now = self.link.wake_at().map_or(t_ns, |w| w.max(self.now + 1)).min(t_ns).min(next_window);
        }
    }
}

fn msc(port: &mut FakePort) -> Msc<'_> {
    let mut h = UsbHost::new(port);
    h.enumerate(1_000_000_000).unwrap();
    Msc::open(h).unwrap()
}

#[test]
fn enumeration_follows_chapter_9_in_emulated_time() {
    let mut port = FakePort::new(FakeMsc::new(2048));
    let mut h = UsbHost::new(&mut port);
    h.enumerate(1_000_000_000).unwrap();
    let d = h.device.clone().unwrap();
    assert_eq!((d.vendor, d.product, d.mps0, d.class), (0x303a, 0x4002, 64, 0xef));
    let c = h.config.clone().unwrap();
    assert_eq!(c.interfaces[0].endpoints.iter().map(|e| (e.address, e.mps)).collect::<Vec<_>>(), [(0x01, 512), (0x81, 512)]);
    assert_eq!(h.strings, [(1, "TinyUSB".to_string()), (2, "TinyUSB Device".to_string()), (3, "123456".to_string())]);
    assert_eq!(h.address, 1);
    let t = h.now_ns();
    assert!((162_000_000..164_000_000).contains(&t), "debounce 100 ms + reset 50 ms + recovery 10 ms + SET_ADDRESS 2 ms and a few microframes, took {t} ns");
    assert!(port.dev.configured);
}

#[test]
fn test_unit_ready_until_the_medium_is_present() {
    let mut dev = FakeMsc::new(2048);
    dev.knobs.not_ready = 3;
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    assert_eq!(m.test_unit_ready(), Err(MscError::Check(Sense { key: sense::NOT_READY, asc: sense::MEDIUM_NOT_PRESENT, ascq: 0 })), "CHECK CONDITION, then REQUEST SENSE says 3A/00");
    let t0 = m.host.now_ns();
    assert_eq!(m.wait_ready(100_000_000, 2_000_000_000), Ok(2));
    assert!(m.host.now_ns() - t0 >= 200_000_000, "two polls 100 ms apart failed first");
    assert_eq!(m.stats.check_conditions, 3);
    let i = m.inquiry().unwrap();
    assert_eq!((i.vendor.as_str(), i.product.as_str(), i.removable), ("TinyUSB", "TEST MSC Storage", true));
    assert_eq!(m.read_capacity().unwrap(), (2048, 512));
    drop(m);
    assert_eq!(port.dev.commands.iter().filter(|&&c| c == op::REQUEST_SENSE).count(), 3);
}

#[test]
fn reads_writes_and_residues() {
    let mut port = FakePort::new(FakeMsc::new(4096));
    let mut m = msc(&mut port);
    m.read_capacity().unwrap();
    let data: Vec<u8> = (0..300 * 512).map(|i| (i % 251) as u8).collect();
    m.write10(100, &data).unwrap();
    assert_eq!(m.read10(100, 300).unwrap(), data, "300 blocks in commands of 240 at most");
    assert!(!m.mode_sense().unwrap(), "four bytes of the 192 asked for: a residue, not an error");
    let r = m.command(&[op::MODE_SENSE6, 0, 0x3f, 0, 192, 0], Dir::In(192)).unwrap();
    assert_eq!((r.data.len(), r.csw.residue, r.csw.status), (4, 188, 0));
    drop(m);
    assert_eq!(&port.dev.disk[100 * 512..400 * 512], &data[..]);
    assert_eq!(port.dev.commands.iter().filter(|&&c| c == op::WRITE10).count(), 2);
}

#[test]
fn a_short_data_stage_and_a_stalled_unknown_command() {
    let mut dev = FakeMsc::new(2048);
    dev.knobs.short_on = Some((op::READ10, 1024));
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    m.read_capacity().unwrap();
    assert_eq!(m.read10(0, 4), Err(MscError::Short { want: 2048, got: 1024, residue: 1024 }));
    // an opcode the device does not know: the data-in STALLs, the host clears it, reads the
    // CSW (failed) and asks for the sense
    assert_eq!(m.scsi(&[0x5a, 0, 0, 0, 0, 0, 0, 0, 252, 0], Dir::In(252)), Err(MscError::Check(Sense { key: sense::ILLEGAL_REQUEST, asc: sense::INVALID_OPCODE, ascq: 0 })));
    assert_eq!(m.stats.stalls, 2, "the short READ(10) ended in a STALL too");
    m.test_unit_ready().unwrap();
    drop(m);
    assert_eq!(port.dev.clear_halts, 2);
}

#[test]
fn a_phase_error_or_a_wrong_tag_ends_in_bot_reset_recovery() {
    let mut dev = FakeMsc::new(2048);
    dev.knobs.phase_on = Some(op::SYNCHRONIZE_CACHE10);
    dev.knobs.bad_tag_on = Some(op::PREVENT_ALLOW);
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    m.recovery = Recovery::Bot;
    assert_eq!(m.sync_cache(), Err(MscError::Phase));
    assert_eq!(m.prevent_removal(true), Err(MscError::BadCsw("tag mismatch".into())));
    assert_eq!(m.stats.resets, 2);
    m.test_unit_ready().unwrap();
    drop(m);
    assert_eq!((port.dev.bot_resets, port.dev.clear_halts), (2, 4), "Bulk-Only Mass Storage Reset and a CLEAR_FEATURE(ENDPOINT_HALT) on each bulk endpoint");
}

#[test]
fn nak_waits_a_microframe() {
    let mut dev = FakeMsc::new(2048);
    dev.knobs.nak_in = 8;
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    let t0 = m.host.now_ns();
    m.test_unit_ready().unwrap();
    let dt = m.host.now_ns() - t0;
    assert!((1_000_000..1_400_000).contains(&dt), "8 NAKs on the CSW, one retry a microframe: {dt} ns");
}

/// Each command's submission and completion, kept only when asked for: a WRITE(10) split at
/// 240 blocks is two commands, a CHECK CONDITION is the failed command and its REQUEST SENSE, and
/// the NAKed CSW shows in its command's latency.
#[test]
fn command_times_are_kept_on_request() {
    let mut dev = FakeMsc::new(4096);
    dev.knobs.not_ready = 1;
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    assert_eq!(m.times, None, "nothing kept by default");
    m.times = Some(Vec::new());
    let t0 = m.host.now_ns();
    m.test_unit_ready().unwrap_err();
    m.write10(100, &vec![0x5a; 300 * 512]).unwrap();
    m.read10(100, 8).unwrap();
    let t1 = m.host.now_ns();
    m.test_unit_ready().unwrap();
    let times = m.times.take().unwrap();
    assert_eq!(times.iter().map(|t| (t.op, t.lba, t.blocks, t.ok)).collect::<Vec<_>>(),
               [(op::TEST_UNIT_READY, 0, 0, false), (op::REQUEST_SENSE, 0, 0, true), (op::WRITE10, 100, 240, true), (op::WRITE10, 340, 60, true),
                (op::READ10, 100, 8, true), (op::TEST_UNIT_READY, 0, 0, true)]);
    assert_eq!((times[0].submit_ns, times[4].done_ns), (t0, t1));
    assert!(times.windows(2).all(|w| w[0].submit_ns < w[0].done_ns && w[0].done_ns <= w[1].submit_ns), "one command at a time: {times:?}");
    assert!(times[2].latency_ns() > times[3].latency_ns(), "240 blocks take longer than 60");
    let mut dev = FakeMsc::new(2048);
    dev.knobs.nak_in = 8;
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    m.times = Some(Vec::new());
    m.test_unit_ready().unwrap();
    let t = m.times.take().unwrap()[0];
    assert!((1_000_000..1_400_000).contains(&t.latency_ns()), "8 NAKs on the CSW, one retry a microframe: {t:?}");
}

#[test]
fn fat32_files_written_like_linux_and_read_back() {
    let img = std::env::temp_dir().join(format!("usbhost-fat-{}.img", std::process::id()));
    sd_image::fat32(&img, 64 << 20, 512, "").unwrap();
    let mut dev = FakeMsc::new(0);
    dev.disk = std::fs::read(&img).unwrap();
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    m.read_capacity().unwrap();
    let mut fs = Fat32::mount(&mut m).unwrap();
    assert_eq!((fs.volume_lba, fs.cluster_bytes()), (2048, 512));
    let clip = clip_bytes("CLIP0001.MP4", 200_000);
    let mut f = fs.create(&mut m, "CLIP0001.MP4").unwrap();
    for part in clip.chunks(64 << 10) { f.write(&mut fs, &mut m, part).unwrap(); }
    assert_eq!(f.close(&mut fs, &mut m).unwrap(), 200_000);
    assert_eq!(fs.read_file(&mut m, "clip0001.mp4").unwrap(), clip);
    assert_eq!(fs.list_root(&mut m).unwrap().iter().map(|e| (e.name.as_str(), e.size)).collect::<Vec<_>>(), [("CLIP0001.MP4", 200_000)]);
    drop(m);
    std::fs::write(&img, &port.dev.disk).unwrap();
    let files = sd_image::Volume::open(&img).unwrap().root_files();
    assert_eq!(files, vec![("CLIP0001.MP4".to_string(), clip)], "the hand-written reader agrees");
    // the write pattern: the entry, then per chunk its data and the FAT sectors its clusters'
    // entries are in, each in both FATs, then the entry again and FSInfo
    let w = &port.dev.writes;
    let (fat1, fat2, root) = (2048 + 32, 2048 + 32 + fs.fat_sectors as usize, fs.data_lba as usize);
    assert_eq!(w[0], (root, 1), "the directory entry first");
    assert_eq!(&w[w.len() - 2..], &[(root, 1), (2049, 1)], "the entry's size and cluster, then FSInfo");
    let data: Vec<_> = w.iter().filter(|x| x.0 > root).collect();
    assert_eq!(data.iter().map(|x| x.1).collect::<Vec<_>>(), [128, 128, 128, 7], "64 KiB chunks, one WRITE(10) each");
    let fat: Vec<_> = w.iter().filter(|x| (fat1..root).contains(&x.0)).collect();
    assert!(fat.iter().all(|x| x.1 == 1) && fat.chunks(2).all(|p| p[0].0 + fs.fat_sectors as usize == p[1].0 && p[0].0 < fat2), "FAT sectors in pairs, the first copy then the second: {fat:?}");
    assert_eq!(w.len(), 2 + data.len() + fat.len() + 1);
    // FSInfo: the free count down by the clip's clusters, the hint the last cluster allocated
    let info = &port.dev.disk[(2048 + 1) * 512..(2048 + 2) * 512];
    let (free, hint) = (u32::from_le_bytes(info[488..492].try_into().unwrap()), u32::from_le_bytes(info[492..496].try_into().unwrap()));
    assert_eq!((free, hint), (fs.clusters - 1 - 391, 3 + 390), "200,000 bytes are 391 clusters from cluster 3");
    let mut mem = MemDisk(port.dev.disk.clone());
    let fs = Fat32::mount(&mut mem).unwrap();
    assert_eq!(fs.list_root(&mut mem).unwrap().len(), 1);
    let _ = std::fs::remove_file(&img);
}

#[test]
fn a_script_parses_and_runs_on_the_fake() {
    let cmds = script::parse("attach\nmsc\nready 100 1000\ncapacity # 2048 blocks\nwrite 10 2 a5\nverify 10 2 a5\nread 10 2\nsuspend\nidle 5\nresume\nuart0 expose\n").unwrap();
    assert_eq!(cmds.len(), 11);
    assert_eq!(cmds[3], (4, script::Cmd::Capacity));
    assert!(script::parse("frobnicate").is_err());
    let mut port = FakePort::new(FakeMsc::new(2048));
    let mut r = script::Runner::new(&mut port);
    r.run(&cmds).unwrap();
    let mut r = script::Runner::new(&mut port);
    assert_eq!(r.run(&script::parse("capacity").unwrap()), Err((1, "no mass-storage interface open (msc)".into())));
}

#[test]
fn control_writes_end_in_a_data1_status() {
    let mut port = FakePort::new(FakeMsc::new(2048));
    let mut h = UsbHost::new(&mut port);
    h.enumerate(1_000_000_000).unwrap();
    for len in [7u16, 70, 128] {
        h.control(setup_packet(0x40, 0x01, 0, 0, len), vec![0xa5; len as usize]).unwrap();
    }
    assert_eq!(h.port().link().unwrap().stats.toggle_errors, 0);
}

/// Linux's usb-storage recovers from a transport error with a port reset and re-enumeration
/// (usb_stor_invoke_transport, Handle_Errors), not the class reset.
#[test]
fn a_phase_error_ends_in_a_port_reset_under_linux() {
    let mut dev = FakeMsc::new(2048);
    dev.knobs.phase_on = Some(op::SYNCHRONIZE_CACHE10);
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    assert_eq!(m.recovery, Recovery::Linux);
    assert_eq!(m.sync_cache(), Err(MscError::Phase));
    assert_eq!((m.stats.port_resets, m.stats.resets), (1, 0));
    assert_eq!(m.host.address, 1, "the device keeps its address");
    m.test_unit_ready().unwrap();
    drop(m);
    assert_eq!((port.dev.bus_resets, port.dev.bot_resets), (3, 0), "the enumeration's reset and the recovery's two");
    assert!(port.dev.configured, "SET_CONFIGURATION restored it");
}

/// A command that outlives its timeout is aborted; Linux then resets the port and the next
/// command works.
#[test]
fn a_command_timeout_aborts_and_resets_the_port() {
    let mut port = FakePort::new(FakeMsc::new(2048));
    let mut m = msc(&mut port);
    m.read_capacity().unwrap();
    m.host.control(setup_packet(0x40, 0x02, 4, 0, 0), Vec::new()).unwrap();
    m.timeout_ns = 50_000_000;
    let t0 = m.host.now_ns();
    assert_eq!(m.read10(0, 16), Err(MscError::Usb(UsbError::Timeout)));
    assert!(m.host.now_ns() - t0 >= 50_000_000);
    assert_eq!((m.stats.aborts, m.stats.port_resets), (1, 1));
    m.timeout_ns = 30_000_000_000;
    assert_eq!(m.read10(0, 16).unwrap().len(), 16 * 512);
}

/// A zero-length packet where the CSW should be: Linux reads the CSW again
/// (usb_stor_Bulk_transport), BOT's host treats it as an invalid CSW and resets.
#[test]
fn a_zero_length_packet_before_the_csw_is_read_past_under_linux() {
    for (recovery, ok) in [(Recovery::Linux, true), (Recovery::Bot, false)] {
        let mut dev = FakeMsc::new(2048);
        dev.knobs.zlp_before_csw_on = Some(op::TEST_UNIT_READY);
        let mut port = FakePort::new(dev);
        let mut m = msc(&mut port);
        m.recovery = recovery;
        assert_eq!(m.test_unit_ready().is_ok(), ok, "{recovery:?}");
        assert_eq!(m.stats.resets, u64::from(!ok));
        m.test_unit_ready().unwrap();
    }
}

/// A clip paced to a camera: chunk i is due at t0 + i·gap. A drive faster than the camera keeps
/// up and the clip takes the camera's time; a camera faster than the drive leaves chunks a whole
/// gap or more behind.
#[test]
fn a_clip_is_paced_to_the_camera_and_lateness_counted() {
    let img = std::env::temp_dir().join(format!("usbhost-pace-{}.img", std::process::id()));
    sd_image::fat32(&img, 64 << 20, 512, "").unwrap();
    let mut dev = FakeMsc::new(0);
    dev.disk = std::fs::read(&img).unwrap();
    std::fs::remove_file(&img).ok();
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
    m.read_capacity().unwrap();
    let mut fs = Fat32::mount(&mut m).unwrap();
    let clip = clip_bytes("A.MP4", 8 << 16);
    let st = write_clip(&mut fs, &mut m, "A.MP4", &clip, 64 << 10, 16_000_000, 1).unwrap();
    assert_eq!((st.bytes, st.chunks), (8 << 16, 8));
    assert!(st.kept_up(16_000_000) && st.worst_late_ns < 16_000_000, "{st:?}");
    assert!(st.elapsed_ns >= 7 * 16_000_000, "the last chunk is not ready before 112 ms: {st:?}");
    assert!(st.summary(16_000_000).contains("kept up"));
    let st = write_clip(&mut fs, &mut m, "B.MP4", &clip_bytes("B.MP4", 8 << 16), 64 << 10, 100_000, 1).unwrap();
    assert!(!st.kept_up(100_000) && st.behind > 0 && st.worst_chunk > 0, "{st:?}");
    assert_eq!(fs.read_file(&mut m, "A.MP4").unwrap(), clip);
}

#[test]
fn script_lines_parse_with_their_tolerance_and_faults() {
    use super::script::{parse, Cmd};
    let c = parse("? uart0 status\nrecovery bot\ntimeout 5\ninject vbus-off 10\nport-reset # again\n? fat-write A.MP4 100 64 16").unwrap();
    assert_eq!(c, vec![(1, Cmd::Tolerate(Box::new(Cmd::Uart0("status".into())))), (2, Cmd::Recovery(Recovery::Bot)), (3, Cmd::Timeout(5)), (4, Cmd::InjectVbusOff(10)), (5, Cmd::PortReset),
                       (6, Cmd::Tolerate(Box::new(Cmd::FatWrite { name: "A.MP4".into(), bytes: 100, chunk: 64 << 10, gap_ms: 16, fat_every: 1 })))]);
    assert!(parse("inject reset 5").is_err() && parse("recovery windows").is_err());
}

/// A tree as a Tesla car writes it: three directory levels with long names, a lower-case file
/// name under a long entry, two directories whose long names share a short basis, and the files
/// read back by path; the root keeps its 8.3 view.
#[test]
fn fat32_directories_and_long_names_round_trip() {
    use super::fat32::mp4_bytes;
    let img = std::env::temp_dir().join(format!("usbhost-fat-tree-{}.img", std::process::id()));
    sd_image::fat32(&img, 64 << 20, 512, "").unwrap();
    let mut mem = MemDisk(std::fs::read(&img).unwrap());
    let _ = std::fs::remove_file(&img);
    let mut fs = Fat32::mount(&mut mem).unwrap();
    let (a, b) = ("TeslaCam/SavedClips/2026-01-01_00-00-00", "TeslaCam/SavedClips/2026-01-01_00-01-00");
    let da = fs.mkdir_all(&mut mem, a).unwrap();
    let db = fs.mkdir_all(&mut mem, b).unwrap();
    assert_eq!(fs.mkdir_all(&mut mem, a).unwrap(), da, "an existing directory is found, not made again");
    let clip = mp4_bytes("event-a", 100_000);
    assert_eq!((&clip[4..8], &clip[28..32], &clip[clip.len() - 12..clip.len() - 8]), (&b"ftyp"[..], &b"mdat"[..], &b"moov"[..]));
    assert_eq!(u32::from_be_bytes(clip[24..28].try_into().unwrap()) as usize + 24 + 16, clip.len(), "the boxes tile the file");
    write_clip(&mut fs, &mut mem, &format!("{a}/event.mp4"), &clip, 64 << 10, 0, 1).unwrap();
    assert_eq!(fs.read_path(&mut mem, &format!("{a}/event.mp4")).unwrap(), clip);
    assert_eq!(fs.read_path(&mut mem, &format!("{a}/EVENT.MP4")).unwrap(), clip, "names compare without case");
    let names = |fs: &Fat32, mem: &mut MemDisk, p: &str| fs.list(mem, p).unwrap().into_iter().map(|e| e.name).collect::<Vec<_>>();
    assert_eq!(names(&fs, &mut mem, "TeslaCam/SavedClips"), ["2026-01-01_00-00-00", "2026-01-01_00-01-00"]);
    assert_eq!(names(&fs, &mut mem, a), ["event.mp4"]);
    assert!(names(&fs, &mut mem, b).is_empty());
    assert_eq!(fs.list_root(&mut mem).unwrap().iter().map(|e| e.name.as_str()).collect::<Vec<_>>(), ["TESLACAM"], "the root's short view");
    assert!(fs.create_path(&mut mem, &format!("{a}/event.mp4")).is_err(), "an existing file");
    assert!(fs.create_path(&mut mem, "TeslaCam/RecentClips/x.mp4").is_err(), "a missing directory");
    assert!(fs.mkdir_all(&mut mem, &format!("{a}/event.mp4/x")).is_err(), "a file in the way");
    // the event directories: `.` is itself, `..` the parent; the parent's entries a long name
    // and the short ones 2026-0~1 and 2026-0~2
    let saved = fs.stat(&mut mem, "TeslaCam/SavedClips").unwrap().unwrap().cluster;
    let tesla = fs.stat(&mut mem, "TeslaCam").unwrap().unwrap().cluster;
    let cluster = |c: u32| { let o = (fs.data_lba + (c as u64 - 2) * fs.sectors_per_cluster as u64) as usize * 512; mem.0[o..o + 512].to_vec() };
    let d = cluster(db);
    assert_eq!((&d[..11], u16::from_le_bytes([d[26], d[27]]) as u32), (&b".          "[..], db));
    assert_eq!((&d[32..43], u16::from_le_bytes([d[58], d[59]]) as u32), (&b"..         "[..], saved));
    let s = cluster(saved);
    let shorts: Vec<&[u8]> = s.chunks(32).filter(|e| e[0] != 0 && e[11] != 0x0f).map(|e| &e[..11]).collect();
    assert_eq!(shorts, [&b".          "[..], b"..         ", b"2026-0~1   ", b"2026-0~2   "]);
    let top = cluster(tesla);
    assert_eq!(u16::from_le_bytes([top[58], top[59]]), 0, "`..` of a directory in the root is cluster 0");
}

/// The same tree on a 3 GiB card with 32 KiB clusters, checked by the system's FAT checker
/// (`sd_image::fsck`): the long names, their checksums, `.` and `..`, the chains and the free
/// count must all satisfy it.
#[test]
#[ignore = "needs a FAT checker: fsck_msdos (macOS) or fsck.fat (Linux), or FSCK_FAT"]
fn external_fat32_tree_passes_fsck() {
    use super::fat32::mp4_bytes;
    let img = std::env::temp_dir().join(format!("usbhost-fat-fsck-{}.img", std::process::id()));
    sd_image::fat32(&img, 3 << 30, 32 << 10, "ESP32SIM").unwrap();
    let mut file = FileDisk(std::fs::OpenOptions::new().read(true).write(true).open(&img).unwrap());
    let mut fs = Fat32::mount(&mut file).unwrap();
    for (i, stamp) in ["2026-01-01_00-00-00", "2026-01-01_00-01-00", "2026-01-01_00-02-00"].iter().enumerate() {
        let dir = format!("TeslaCam/{}/{stamp}", if i == 2 { "SentryClips" } else { "SavedClips" });
        fs.mkdir_all(&mut file, &dir).unwrap();
        let clip = mp4_bytes(stamp, 300_000 + i * 70_000);
        write_clip(&mut fs, &mut file, &format!("{dir}/event.mp4"), &clip, 64 << 10, 0, 1).unwrap();
        assert_eq!(fs.read_path(&mut file, &format!("{dir}/event.mp4")).unwrap(), clip);
    }
    let mut f = fs.create(&mut file, "README.TXT").unwrap();
    f.write(&mut fs, &mut file, b"8.3 in the root").unwrap();
    f.close(&mut fs, &mut file).unwrap();
    drop(file);
    let r = sd_image::fsck(&img);
    let _ = std::fs::remove_file(&img);
    let out = r.unwrap_or_else(|e| panic!("{e}"));
    eprintln!("{out}");
}

/// A card image file as a block device.
struct FileDisk(std::fs::File);
impl super::fat32::BlockDevice for FileDisk {
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

use super::car::{Car, CarCfg, Offline, Recorded};
use super::fat32::{mp4_bytes, FsckWrite};
use super::pagecache::{PageCache, WritebackCfg};

/// A 64 MiB FAT32 image of 512-byte clusters, in memory.
fn small_volume(tag: &str) -> Vec<u8> {
    let img = std::env::temp_dir().join(format!("usbhost-{tag}-{}.img", std::process::id()));
    sd_image::fat32(&img, 64 << 20, 512, "").unwrap();
    let d = std::fs::read(&img).unwrap();
    let _ = std::fs::remove_file(&img);
    d
}

/// The application's FatFs making a directory on the card, as oxidrive's TeslaCam self-heal does
/// in a window: a cluster from FSInfo's hint, the entry in the root, FSInfo.
fn app_mkdir(path: &'static str) -> AppWrite {
    Box::new(move |disk: &mut Vec<u8>| {
        let mut m = MemDisk(std::mem::take(disk));
        let mut fs = Fat32::mount(&mut m).unwrap();
        fs.mkdir_all(&mut m, path).unwrap();
        *disk = m.0;
    })
}

/// esp_tinyusb answers TEST UNIT READY with MEDIUM NOT PRESENT while the application holds the
/// card but writes every WRITE(10) to the card all the same, so a car recording when a window
/// opens keeps writing into it until its next poll: here 2 s polls, a window from 0.5 s, the
/// writes between it and the poll at 2.16 s on the card, the clip cut there. When the window ends
/// the car sees the medium again and reads the volume anew.
#[test]
fn a_car_keeps_writing_into_a_window_until_its_next_poll() {
    let mut dev = FakeMsc::new(0);
    dev.disk = small_volume("window-writes");
    dev.windows.push(FakeWindow { from: 500_000_000, to: 3_000_000_000, app: None });
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut car = Car::new(PageCache::new(m), CarCfg::default());
    car.mount().unwrap();
    let mounted = car.now();
    let clip = mp4_bytes("event", 8 << 20);
    let r = car.record("TeslaCam/RecentClips/clip.mp4", &clip).unwrap();
    let Recorded::MediumGone { written } = r else { panic!("{r:?}") };
    let gone = car.log.iter().find(|l| l.1.contains("medium gone")).unwrap().0;
    assert!((mounted + 2_000_000_000..mounted + 2_010_000_000).contains(&gone), "the poll 2 s after the mount saw it: {gone}");
    assert!(written > 6 << 20 && written < 8 << 20, "about 2 s of a 4.1 MB/s camera written: {written}");
    assert!(!car.present && car.fs.is_none());
    car.idle(2_000_000_000).unwrap();
    assert!(car.present && car.stats.mounts == 2 && car.stats.media_changes == 2, "{:?}", car.stats);
    drop(car);
    let in_window: Vec<_> = port.dev.writes.iter().zip(&port.dev.write_at).filter(|(_, &t)| (500_000_000..gone).contains(&t)).collect();
    let blocks: usize = in_window.iter().map(|(w, _)| w.1).sum();
    assert!(in_window.len() > 90 && blocks * 512 > 6 << 20, "{} WRITE(10)s, {blocks} blocks reached the card under the application", in_window.len());
    // vfat-like order: the cut clip's entry still says 0 bytes and no cluster, its data orphaned
    let mut mem = MemDisk(port.dev.disk.clone());
    let fs = Fat32::mount(&mut mem).unwrap();
    let e = fs.stat(&mut mem, "TeslaCam/RecentClips/clip.mp4").unwrap().unwrap();
    assert_eq!((e.size, e.cluster), (0, 0));
}

/// A car that keeps the FAT and the directories it read (Linux's buffer cache), polling every 5 s,
/// does not see a window that opens and closes between two polls: the application's directory,
/// made in the window, is not in the car's cached root and FAT, so the car's own directory takes
/// the same root slot and the same cluster. A car that reads the volume again sees it and uses it.
#[test]
fn a_caching_car_writes_over_what_a_window_between_its_polls_wrote() {
    for keep in [true, false] {
        let mut dev = FakeMsc::new(0);
        dev.disk = small_volume("window-short");
        dev.windows.push(FakeWindow { from: 1_000_000_000, to: 1_500_000_000, app: Some(app_mkdir("TeslaCam/SavedClips")) });
        let mut port = FakePort::new(dev);
        let m = msc(&mut port);
        let disk = if keep { PageCache::keeping_meta(m) } else { PageCache::new(m) };
        let mut car = Car::new(disk, CarCfg { poll_ns: 5_000_000_000, ..CarCfg::default() });
        car.mount().unwrap();
        // the car looks for its folder before the window: the root and the FAT read
        assert!(car.fs.clone().unwrap().stat(&mut car.disk, "TeslaCam").unwrap().is_none());
        car.idle(6_000_000_000).unwrap();
        assert_eq!((car.stats.polls, car.stats.media_changes), (1, 0), "the one poll, at 5.16 s, found the medium there");
        let fs = car.fs.as_mut().unwrap();
        fs.mkdir_all(&mut car.disk, "TeslaCam/RecentClips").unwrap();
        drop(car);
        let mut mem = MemDisk(port.dev.disk.clone());
        let fs = Fat32::mount(&mut mem).unwrap();
        let names = |p: &str, mem: &mut MemDisk| fs.list(mem, p).map(|v| v.into_iter().map(|e| e.name).collect::<Vec<_>>());
        if keep {
            assert_eq!(names("TeslaCam", &mut mem).unwrap(), ["RecentClips"], "the application's SavedClips is gone: the car's TeslaCam took its cluster");
            assert_eq!(fs.list_root(&mut mem).unwrap().len(), 1, "one TESLACAM in the root, in the same slot");
        } else {
            assert_eq!(names("TeslaCam", &mut mem).unwrap(), ["SavedClips", "RecentClips"]);
        }
    }
}

/// The window of `a_car_keeps_writing_into_a_window_until_its_next_poll`, gated: the car's first
/// WRITE(10) after the window opens is refused with MEDIUM NOT PRESENT, so nothing of it reaches
/// the card and the clip is cut at once; its poll then finds the medium gone, and the first poll
/// after the window meets the UNIT ATTENTION and revalidates (READ CAPACITY, MODE SENSE, the
/// volume).
#[test]
fn a_gated_window_refuses_the_car_and_its_unit_attention_revalidates() {
    let mut dev = FakeMsc::new(0);
    dev.disk = small_volume("window-gated");
    dev.windows.push(FakeWindow { from: 500_000_000, to: 3_000_000_000, app: None });
    dev.gated = true;
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut car = Car::new(PageCache::new(m), CarCfg::default());
    car.mount().unwrap();
    let clip = mp4_bytes("event", 8 << 20);
    let r = car.record("TeslaCam/RecentClips/clip.mp4", &clip).unwrap();
    let Recorded::WriteFailed { written, error } = r else { panic!("{r:?}") };
    assert!(error.contains("2/3a/00") && written < 2 << 20, "refused at the window's start, about 0.33 s of the camera written: {written}, {error}");
    car.idle(4_500_000_000).unwrap();
    assert_eq!((car.stats.unit_attentions, car.stats.media_changes, car.stats.mounts), (1, 2, 2), "gone at a poll, back at the unit attention: {:?}", car.stats);
    assert!(car.present && car.log.iter().any(|l| l.1.contains("NOT READY TO READY CHANGE")));
    drop(car);
    assert!(port.dev.write_at.iter().all(|&t| !(500_000_000..3_000_000_000).contains(&t)), "no WRITE(10) reached the card in the window");
    let after: Vec<u8> = port.dev.commands.iter().rev().take(12).copied().collect();
    assert!(after.contains(&op::MODE_SENSE6) && after.contains(&op::READ_CAPACITY10), "the revalidation's commands: {after:02x?}");
}

/// `a_caching_car_writes_over_what_a_window_between_its_polls_wrote`, gated: the window still
/// falls between the 5 s car's polls, but that poll meets the UNIT ATTENTION, so the car drops
/// its cached root and FAT and reads the volume again before it makes its folder.
#[test]
fn a_caching_car_reads_again_what_a_gated_window_between_its_polls_wrote() {
    let mut dev = FakeMsc::new(0);
    dev.disk = small_volume("window-short-gated");
    dev.windows.push(FakeWindow { from: 1_000_000_000, to: 1_500_000_000, app: Some(app_mkdir("TeslaCam/SavedClips")) });
    dev.gated = true;
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut car = Car::new(PageCache::keeping_meta(m), CarCfg { poll_ns: 5_000_000_000, ..CarCfg::default() });
    car.mount().unwrap();
    assert!(car.fs.clone().unwrap().stat(&mut car.disk, "TeslaCam").unwrap().is_none());
    car.idle(6_000_000_000).unwrap();
    assert_eq!((car.stats.polls, car.stats.unit_attentions, car.stats.media_changes), (1, 1, 1), "the one poll met the unit attention");
    let fs = car.fs.as_mut().unwrap();
    fs.mkdir_all(&mut car.disk, "TeslaCam/RecentClips").unwrap();
    drop(car);
    let mut mem = MemDisk(port.dev.disk.clone());
    let fs = Fat32::mount(&mut mem).unwrap();
    let names: Vec<String> = fs.list(&mut mem, "TeslaCam").unwrap().into_iter().map(|e| e.name).collect();
    assert_eq!(names, ["SavedClips", "RecentClips"], "the application's folder kept");
}

/// A writeback car whose pages land while a gated window holds the card, and after it: the first
/// landing after the window meets the UNIT ATTENTION, which marks the device changed, so the rest
/// are refused by the host itself (sd.c:1340-1343) and lost, none reaching the card, and the next
/// poll revalidates. A landing the drive refuses is not the error of the pause that woke it.
#[test]
fn a_writeback_car_loses_its_pages_to_a_gated_window_and_revalidates() {
    let mut dev = FakeMsc::new(0);
    dev.disk = small_volume("window-writeback-gated");
    dev.windows.push(FakeWindow { from: 1_000_000_000, to: 1_500_000_000, app: None });
    dev.gated = true;
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut car = Car::new(PageCache::writeback(m, WritebackCfg::default()), CarCfg { poll_ns: 5_000_000_000, linux_entries: true, ..CarCfg::default() });
    car.mount().unwrap();
    let r = car.record("TeslaCam/RecentClips/clip.mp4", &mp4_bytes("c", 1 << 20)).unwrap();
    assert!(matches!(r, Recorded::Closed { .. }), "{r:?}");
    car.idle(6_000_000_000).unwrap();
    assert_eq!((car.stats.unit_attentions, car.stats.media_changes), (1, 1), "{:?}", car.stats);
    assert!(car.disk.stats.write_errors > 1 && car.disk.dirty_pages() == 0, "{:?}", car.disk.stats);
    drop(car);
    assert!(port.dev.write_at.iter().all(|&t| t < 1_000_000_000), "nothing landed in or after the window: {:?}", port.dev.write_at);
}

/// A recorder on Linux's writeback, the firmware's `WritebackCfg` defaults: each chunk's data in
/// one batch 50 ms to 5 s after it was written, metadata pages held back 30 s about 96 times in
/// 100, the entry rewritten as the file grows, so the medium sees intermediate sizes; after a
/// sync the medium is what the car believes.
#[test]
fn writeback_lands_data_in_batches_and_holds_the_metadata_back() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(small_volume("writeback")), now: 0 }, WritebackCfg::default());
    // 64 KiB a second: each clip records for 48 s, longer than the metadata lag
    let mut car = Car::new(disk, CarCfg { poll_ns: 0, linux_entries: true, gap_ns: 1_000_000_000, ..CarCfg::default() });
    car.disk.record = true;
    car.mount().unwrap();
    let clips: Vec<(String, Vec<u8>)> = (0..6).map(|i| (format!("TeslaCam/SavedClips/e{i}/front.mp4"), mp4_bytes(&format!("c{i}"), 3 << 20))).collect();
    for (p, c) in &clips { assert!(matches!(car.record(p, c).unwrap(), Recorded::Closed { .. })); }
    car.idle(40_000_000_000).unwrap();
    assert_eq!(car.disk.dirty_pages(), 0, "everything landed within the lag and a flusher wake");
    let fs = car.fs.clone().unwrap();
    let data: Vec<_> = car.disk.landings.iter().filter(|l| !l.meta).collect();
    let lag = |l: &&super::pagecache::Landing| l.at_ns - l.dirtied_ns;
    assert!(data.iter().all(|l| (50_000_000..=5_100_000_000).contains(&lag(l))), "data lands within a flusher wake");
    assert!(data.iter().all(|l| l.count >= 128 || l.lba + l.count as u64 > fs.data_lba), "a 64 KiB chunk lands in one write");
    let meta: Vec<_> = car.disk.landings.iter().filter(|l| l.meta).collect();
    let held = meta.iter().filter(|l| lag(l) >= 30_000_000_000).count();
    assert!(held * 100 >= meta.len() * 85, "{held} of {} metadata writes trailed 30 s", meta.len());
    let (first, entry_first) = super::pagecache::data_first(&car.disk.landings, &fs);
    eprintln!("data first in {first} clusters, the entry first in {entry_first} ({:.1}% data first); {held} of {} metadata writes held back", first as f64 * 100.0 / (first + entry_first) as f64, meta.len());
    assert!(first > 0 && entry_first > 0 && first * 100 > (first + entry_first) * 95, "data before its FAT entry in {first} clusters, the entry first in {entry_first}");
    // the medium mid-recording: some entry sizes landed between 0 and the clip's size
    let sizes = landed_entry_sizes(&car.disk.meta_landed);
    assert!(sizes.iter().any(|&(_, s)| s > 0 && s < 3 << 20), "an intermediate size reached the medium: {sizes:?}");
    car.disk.sync().unwrap();
    let mut m = std::mem::replace(&mut car.disk.dev.dev, MemDisk(Vec::new()));
    let fs = Fat32::mount(&mut m).unwrap();
    for (p, c) in &clips { assert_eq!(fs.read_path(&mut m, p).unwrap(), *c, "{p}"); }
}

/// Power gone mid-recording: what had not landed is lost. The medium keeps a torn volume, which
/// is dirty because the car never unmounted it; fsck cuts the files to what their chains hold,
/// frees the clusters nothing reaches, and a second run finds nothing to do.
#[test]
fn a_power_cut_leaves_a_dirty_torn_volume_that_fsck_repairs() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(small_volume("powercut")), now: 0 }, WritebackCfg::default());
    let mut car = Car::new(disk, CarCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, ..CarCfg::default() });
    car.mount().unwrap();
    for i in 0..3 { car.record(&format!("TeslaCam/RecentClips/c{i}.mp4"), &mp4_bytes(&format!("c{i}"), 2 << 20)).unwrap(); }
    car.idle(3_000_000_000).unwrap();
    let lost = car.power_cut();
    assert!(lost > 0);
    let mut m = Offline { dev: std::mem::replace(&mut car.disk.dev.dev, MemDisk(Vec::new())), now: 0 };
    let mut fs = Fat32::mount(&mut m).unwrap();
    assert!(fs.dirty(&mut m).unwrap(), "no clean unmount");
    let r = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(r.dirty_cleared && r.lost_clusters + r.truncated.len() as u32 + r.resized.len() as u32 > 0, "{}", r.summary());
    let again = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(!again.dirty_cleared && again.lost_clusters == 0 && again.truncated.is_empty() && again.resized.is_empty() && again.fat_sectors_changed == 0, "{}", again.summary());
}

/// A car's RecentClips ring on a full volume: deleting the oldest clip frees its clusters, and the
/// next clip, allocated from the hint as Linux's `fat_alloc_clusters` does once it has come round,
/// takes them.
#[test]
fn rotation_on_a_full_volume_reuses_the_clusters_it_frees() {
    let mut mem = MemDisk(small_volume("rotation"));
    let mut fs = Fat32::mount(&mut mem).unwrap();
    // fill the volume but for three clips and the two directories they go in
    let fill = fs.clusters as usize - 1 - (3 * (1 << 20) / 512 + 2);
    let mut w = fs.create(&mut mem, "FILLER.BIN").unwrap();
    for n in (0..fill).step_by(2048).map(|at| (fill - at).min(2048)) { w.write(&mut fs, &mut mem, &vec![0u8; n * 512]).unwrap(); }
    w.close(&mut fs, &mut mem).unwrap();
    let mut car = Car::new(PageCache::new(Offline { dev: mem, now: 0 }), CarCfg { poll_ns: 0, gap_ns: 0, ..CarCfg::default() });
    car.mount().unwrap();
    let path = |i: usize| format!("TeslaCam/RecentClips/c{i}.mp4");
    let mut chains = Vec::new();
    for i in 0..6 {
        if i >= 3 { car.delete(&path(i - 3)).unwrap(); }
        car.record(&path(i), &mp4_bytes(&path(i), 1 << 20)).unwrap();
        let fs = car.fs.clone().unwrap();
        let e = fs.stat(&mut car.disk, &path(i)).unwrap().unwrap();
        chains.push(fs.chain(&mut car.disk, e.cluster).unwrap());
    }
    for i in 3..6 { assert_eq!(chains[i], chains[i - 3], "clip {i} takes clip {}'s clusters", i - 3); }
    assert_eq!(car.read(&path(5)).unwrap(), mp4_bytes(&path(5), 1 << 20));
}

/// The (name, size) of every file entry in the directory sectors that landed, in landing order.
fn landed_entry_sizes(landed: &[(u64, u64, Vec<u8>)]) -> Vec<(String, u32)> {
    landed.iter().flat_map(|(_, _, d)| d.chunks(32).filter(|e| e[0] != 0 && e[0] != 0xe5 && e[11] == 0x20).map(|e| (String::from_utf8_lossy(&e[..11]).to_string(), u32::from_le_bytes(e[28..32].try_into().unwrap()))).collect::<Vec<_>>()).collect()
}

/// A RecentClips ring of long-named clips in one directory: each deletion leaves its entries
/// free (0xE5), and the next clip's long name and short entry take them, so the directory's one
/// cluster holds the ring for good.
#[test]
fn a_ring_of_long_names_reuses_its_directory_slots() {
    let img = std::env::temp_dir().join(format!("usbhost-ring-{}.img", std::process::id()));
    sd_image::fat32(&img, 3 << 30, 32 << 10, "").unwrap();
    let file = FileDisk(std::fs::OpenOptions::new().read(true).write(true).open(&img).unwrap());
    let mut car = Car::new(PageCache::writeback(Offline { dev: file, now: 0 }, WritebackCfg::default()), CarCfg { poll_ns: 0, gap_ns: 0, linux_entries: true, ..CarCfg::default() });
    car.mount().unwrap();
    let cams = ["front", "back", "left_repeater", "right_repeater"];
    let mut ring = std::collections::VecDeque::new();
    for m in 0..300u32 {
        let paths: Vec<String> = cams.iter().map(|c| format!("TeslaCam/RecentClips/2026-01-01_{:02}-{:02}-00-{c}.mp4", m / 60, m % 60)).collect();
        let datas: Vec<Vec<u8>> = paths.iter().map(|p| mp4_bytes(p, 40_000)).collect();
        let set: Vec<(&str, &[u8])> = paths.iter().map(|p| p.as_str()).zip(datas.iter().map(|d| d.as_slice())).collect();
        car.record_set(&set, 100_000_000).unwrap_or_else(|e| panic!("set {m}: {e}"));
        ring.push_back(paths);
        if ring.len() > 12 { for p in ring.pop_front().unwrap() { car.delete(&p).unwrap(); } }
    }
    car.disk.sync().unwrap();
    let fs = car.fs.clone().unwrap();
    let e = fs.stat(&mut car.disk, "TeslaCam/RecentClips").unwrap().unwrap();
    assert_eq!(fs.chain(&mut car.disk, e.cluster).unwrap().len(), 1, "one cluster");
    assert_eq!(fs.list(&mut car.disk, "TeslaCam/RecentClips").unwrap().len(), 48);
    drop(car);
    let _ = std::fs::remove_file(&img);
}

/// A USB reset without power loss, then the car's fsck with the FATs rewritten whole: the port
/// reset as Linux's `usb_reset_device` does it, every sector of both FATs written, FAT1 first, in
/// commands of 240 blocks, and the volume read again afterwards.
#[test]
fn a_reset_then_a_bulk_fsck_rewrites_both_fats_whole() {
    let mut dev = FakeMsc::new(0);
    dev.disk = small_volume("bulk-fsck");
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut car = Car::new(PageCache::new(m), CarCfg { poll_ns: 0, gap_ns: 0, ..CarCfg::default() });
    car.mount().unwrap();
    car.record("TeslaCam/RecentClips/a.mp4", &mp4_bytes("a", 300_000)).unwrap();
    car.disk.dev.host.reset_device().unwrap();
    let fs = car.fs.clone().unwrap();
    let r = car.fsck_now(super::fat32::FsckWrite::Bulk).unwrap();
    assert_eq!((r.files, r.lost_clusters, r.fat_bytes_written), (1, 0, 2 * fs.fat_sectors as u64 * 512), "{}", r.summary());
    assert_eq!(car.read("TeslaCam/RecentClips/a.mp4").unwrap(), mp4_bytes("a", 300_000));
    drop(car);
    let fat1 = 2048 + fs.reserved as usize;
    let fat2 = fat1 + fs.fat_sectors as usize;
    let w = &port.dev.writes;
    let first = w.iter().position(|x| x.0 == fat1 && x.1 == 240).expect("FAT1 rewritten from its first sector");
    let blocks = |lo: usize, hi: usize| w[first..].iter().filter(|x| (lo..hi).contains(&x.0)).map(|x| x.1).sum::<usize>();
    assert_eq!((blocks(fat1, fat2), blocks(fat2, fat2 + fs.fat_sectors as usize)), (fs.fat_sectors as usize, fs.fat_sectors as usize));
    assert!(w[first..].iter().take_while(|x| x.0 < fat2).all(|x| (fat1..fat2).contains(&x.0)), "FAT1 whole before FAT2");
    assert_eq!(port.dev.bus_resets, 3, "the enumeration's reset and the port reset's two");
}

mod exfat;
