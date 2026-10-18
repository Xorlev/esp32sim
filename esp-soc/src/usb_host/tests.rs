//! The host against a mass-storage device written in Rust at the transaction level: a fake that
//! answers like TinyUSB's MSC class, with knobs for the failures BOT's host side must handle.
#[path = "../../../tests/sd_image.rs"]
mod sd_image;
use super::fat32::{clip_bytes, Fat32, MemDisk};
use super::msc::{op, sense, Dir, Msc, MscError, Sense};
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
struct Knobs { not_ready: u32, phase_on: Option<u8>, bad_tag_on: Option<u8>, short_on: Option<(u8, usize)>, nak_in: u32 }

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
}

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
                  pid: [false; 2], halt: [false; 2], bot: Bot::Idle, tag: 0, want: 0, sense: Sense::default(), knobs: Knobs::default(), bot_resets: 0, clear_halts: 0, commands: Vec::new(), write_lba: 0, writes: Vec::new() }
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
        let reply = match opc {
            op::TEST_UNIT_READY => {
                if self.knobs.not_ready > 0 { self.knobs.not_ready -= 1; return self.fail(sense::NOT_READY, sense::MEDIUM_NOT_PRESENT, false); }
                None
            }
            op::REQUEST_SENSE => { let mut s = vec![0u8; 18]; s[0] = 0x70; s[2] = self.sense.key; s[7] = 10; s[12] = self.sense.asc; s[13] = self.sense.ascq; self.sense = Sense::default(); Some(s) }
            op::INQUIRY => { let mut s = vec![0u8; 36]; s[1] = 0x80; s[4] = 31; s[8..16].copy_from_slice(b"TinyUSB "); s[16..32].copy_from_slice(b"TEST MSC Storage"); s[32..36].copy_from_slice(b"0.1 "); Some(s) }
            op::READ_CAPACITY10 => { let mut s = ((self.disk.len() / 512 - 1) as u32).to_be_bytes().to_vec(); s.extend_from_slice(&512u32.to_be_bytes()); Some(s) }
            op::MODE_SENSE6 => Some(vec![3, 0, 0, 0]),
            op::PREVENT_ALLOW | op::START_STOP_UNIT | op::SYNCHRONIZE_CACHE10 => None,
            op::READ10 => { if (lba + n) * 512 > self.disk.len() { return self.fail(sense::ILLEGAL_REQUEST, 0x21, true); } Some(self.disk[lba * 512..(lba + n) * 512].to_vec()) }
            op::WRITE10 => { self.write_lba = lba; self.writes.push((lba, n)); self.bot = Bot::DataOut(n * 512, Vec::new()); return; }
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
    fn bus_reset(&mut self, _now: u64) { self.configured = false; self.bot = Bot::Idle; }
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
            if self.link.tick_to(self.now) { self.link.service(self.now, &mut self.dev, &mut NoMem); }
            let p = self.dev.pullup();
            if p != self.pullup { self.pullup = p; self.link.device_pullup(self.now, p); continue; }
            if self.now >= t_ns { return Ok(()); }
            self.now = self.link.wake_at().map_or(t_ns, |w| w.max(self.now + 1)).min(t_ns);
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
fn a_phase_error_or_a_wrong_tag_ends_in_reset_recovery() {
    let mut dev = FakeMsc::new(2048);
    dev.knobs.phase_on = Some(op::SYNCHRONIZE_CACHE10);
    dev.knobs.bad_tag_on = Some(op::PREVENT_ALLOW);
    let mut port = FakePort::new(dev);
    let mut m = msc(&mut port);
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
