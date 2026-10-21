//! Shared-pass refusals by cause (milestone 4c): how many of oxidrive's shared-pass refusals and
//! discards a per-cluster write map with FAT-entry and directory-entry comparison (the reduced
//! snapshot design of oxidrive's shared-reading plan) would remove. A scratch build of the firmware
//! (`OXIDRIVE_REFUSAL_DIR`, branch sim-refusal-log: 728b71f plus `RLOG` log lines, no change of
//! behaviour) logs every listing refusal, every watch it arms and every re-check, with the dirty
//! log's state and the retained writes that hit the watch. A Sentry-like writeback car records on a
//! nearly full card for minutes while oxidrive's own shared passes run. Each discard is then judged
//! against the medium's history (the card image after the car's mount, and the bytes of every
//! metadata page the car landed, with its time): whether the clip's data clusters were written
//! during the read, whether its chain's FAT entries changed, whether its directory entry or long
//! name changed. A discard where none of those happened was caused by the ring wrapping, by false
//! sharing of a FAT or directory sector, or by a watch widened past the clip.
//!
//! The SD bus is timed; the card's programming time is `OXIDRIVE_SD_PRG_US`, 0 by default here: with
//! programming time the bus race (a read meeting the card in programming is not answered) ends
//! nearly every pass before refusals accumulate. [esp32p4.md](../../docs/esp32p4.md), "Shared-pass
//! refusals by cause", has the method, the taxonomy and the results.
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
use esp_soc::usb_host::car::{Car, CarCfg, CarFile, Recorded};
use esp_soc::usb_host::fat32::{mp4_bytes, BlockDevice, Fat32};
use esp_soc::usb_host::msc::Msc;
use esp_soc::usb_host::pagecache::{Landing, PageCache, WritebackCfg};
use esp_soc::usb_host::UsbHost;
use oxidrive_rig::*;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};

const S: u64 = 1_000_000_000;
const MS: u64 = 1_000_000;

fn env_u64(name: &str, default: u64) -> u64 { std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default) }
fn secs(ns: u64) -> String { format!("{:.3} s", ns as f64 / 1e9) }
fn pct(n: usize, d: usize) -> String { if d == 0 { "-".into() } else { format!("{:.1}%", n as f64 * 100.0 / d as f64) } }

/// The scratch build: `OXIDRIVE_REFUSAL_DIR`, else the fixtures' default.
fn refusal_dir() -> String {
    std::env::var("OXIDRIVE_REFUSAL_DIR").unwrap_or_else(|_| format!("{}/Code/dlp/sim-fixtures/esp32p4/oxidrive-refusal-log", std::env::var("HOME").unwrap_or_default()))
}

/// Enumerate, open the mass-storage interface, wait for the drive, and mount the volume.
fn car_on<'p>(r: &'p mut Rig, cfg: CarCfg, wb: WritebackCfg) -> Car<Msc<'p>> {
    let mut host = UsbHost::new(r);
    host.log = std::env::var("OXIDRIVE_USB_LOG").is_ok_and(|v| v == "1");
    host.enumerate(10 * S).unwrap_or_else(|e| panic!("enumerate: {e}"));
    let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
    msc.wait_ready(100 * MS, 10 * S).unwrap_or_else(|e| panic!("never ready: {e}"));
    let mut car = Car::new(PageCache::writeback(msc, wb), cfg);
    car.echo = std::env::var("OXIDRIVE_CAR_ECHO").is_ok_and(|v| v == "1");
    car.disk.record = true;
    car.mount().unwrap_or_else(|e| panic!("mount: {e}"));
    car
}

/// Tesla's names: stamped to the minute of the recorder's clock.
fn stamp(minute: u32) -> String { format!("2026-01-01_{:02}-{:02}-00", minute / 60, minute % 60) }
const CAMS: [&str; 4] = ["front", "back", "left_repeater", "right_repeater"];

/// A Sentry-like recorder. Every step records a set of four camera clips into RecentClips, chunk i
/// of every clip due at t0 + i * gap. Every `event_every` sets a Sentry event: its folder, a small
/// `event.json` and `thumb.png`, and copies of the last `preroll` sets and of the trigger set,
/// written `copies` clips a step alongside the live set (the folder is a live write target for
/// several steps, at twice the camera rate). RecentClips keeps `ring` sets and SentryClips
/// `events_kept` events, the oldest deleted as vfat unlinks, so on a nearly full card the next
/// clips take the freed clusters.
struct Sentry { set_ns: u64, clip: usize, ring: usize, event_every: u32, preroll: usize, copies: usize, events_kept: usize, minute: u32, sets: u32, events: u32, recent: VecDeque<Vec<String>>, kept: VecDeque<(String, Vec<String>)>, pending: VecDeque<(String, String)>, trigger_dir: Option<String> }

impl Sentry {
    fn from_env() -> Self {
        Sentry { set_ns: env_u64("REFUSAL_SET_SECS", 20) * S, clip: (env_u64("REFUSAL_CLIP_KIB", 8192) << 10) as usize, ring: env_u64("REFUSAL_RING", 6) as usize,
                 event_every: env_u64("REFUSAL_EVENT_EVERY", 4) as u32, preroll: env_u64("REFUSAL_PREROLL", 2) as usize, copies: env_u64("REFUSAL_COPIES", 4) as usize,
                 events_kept: env_u64("REFUSAL_EVENTS_KEPT", 6) as usize, minute: 0, sets: 0, events: 0, recent: VecDeque::new(), kept: VecDeque::new(), pending: VecDeque::new(), trigger_dir: None }
    }

    /// Free space the recorder needs at most, with a margin: the ring and one set more, the kept
    /// events and one more.
    fn needs(&self) -> u64 { let set = (4 * self.clip) as u64; (self.ring as u64 + 2) * set + (self.events_kept as u64 + 1) * ((self.preroll as u64 + 1) * set + (64 << 10)) + (64 << 20) }

    fn step<D: esp_soc::usb_host::car::Drive>(&mut self, car: &mut Car<D>) -> Result<Recorded, String> {
        let m = self.minute;
        self.minute += 1;
        self.sets += 1;
        if self.sets.is_multiple_of(self.event_every) {
            self.events += 1;
            let dir = format!("TeslaCam/SentryClips/{}", stamp(m));
            let json = format!("{{\"timestamp\":\"{}\",\"city\":\"Testville\",\"reason\":\"sentry_aware_object_detection\",\"camera\":\"0\"}}", stamp(m)).into_bytes();
            let thumb = mp4_bytes(&format!("{dir}/thumb.png"), 16 << 10);
            let small = [(format!("{dir}/event.json"), json), (format!("{dir}/thumb.png"), thumb)];
            let r = car.record_set(&small.iter().map(|(p, d)| (p.as_str(), d.as_slice())).collect::<Vec<_>>(), 0)?;
            let mut files: Vec<String> = small.iter().map(|s| s.0.clone()).collect();
            for set in self.recent.iter().rev().take(self.preroll).rev() {
                for p in set { let dst = format!("{dir}/{}", p.rsplit('/').next().unwrap()); self.pending.push_back((p.clone(), dst.clone())); files.push(dst); }
            }
            self.kept.push_back((dir.clone(), files));
            self.trigger_dir = Some(dir);
            if !matches!(r, Recorded::Closed { .. }) { return Ok(r); }
        }
        let live: Vec<String> = CAMS.iter().map(|c| format!("TeslaCam/RecentClips/{}-{c}.mp4", stamp(m))).collect();
        let mut clips: Vec<(String, Vec<u8>)> = live.iter().map(|p| (p.clone(), mp4_bytes(p, self.clip))).collect();
        for _ in 0..self.copies {
            let Some((src, dst)) = self.pending.pop_front() else { break };
            clips.push((dst, car.files[&src].data.clone()));
        }
        let chunks = self.clip.div_ceil(car.cfg.chunk) as u64;
        let r = car.record_set(&clips.iter().map(|(p, d)| (p.as_str(), d.as_slice())).collect::<Vec<_>>(), self.set_ns / chunks.max(1))?;
        self.recent.push_back(live.clone());
        // the trigger set is copied into its event after it is recorded
        if let Some(dir) = self.trigger_dir.take() {
            for p in &live {
                let dst = format!("{dir}/{}", p.rsplit('/').next().unwrap());
                self.pending.push_back((p.clone(), dst.clone()));
                if let Some(k) = self.kept.iter_mut().find(|k| k.0 == dir) { k.1.push(dst); }
            }
        }
        if !matches!(r, Recorded::Closed { .. }) { return Ok(r); }
        while self.recent.len() > self.ring { for p in self.recent.pop_front().unwrap() { car.delete(&p)?; } }
        while self.kept.len() > self.events_kept {
            let (dir, files) = self.kept.pop_front().unwrap();
            self.pending.retain(|(_, d)| !d.starts_with(&dir));
            for p in files { if car.files.get(&p).is_some_and(|f| !f.deleted) { car.delete(&p)?; } }
        }
        Ok(r)
    }
}

/// Fill all of a volume but `leave` bytes with one file, FILLER.BIN, its chain written straight
/// into both FATs (as oxidrive_hazards.rs does): a card as full as a car keeps it.
fn fill_volume(image: &std::path::Path, leave: u64) {
    let mut f = FileDisk(std::fs::OpenOptions::new().read(true).write(true).open(image).unwrap());
    let mut fs = Fat32::mount(&mut f).unwrap();
    let cb = fs.cluster_bytes() as u64;
    let n = fs.clusters as u64 - 1 - leave / cb;
    let fat = fs.volume_lba + fs.reserved as u64;
    let last = 3 + n - 1;
    for sec in (3 / 128)..=(last / 128) {
        let mut s = [0u8; 512];
        f.read_blocks(fat + sec, &mut s).unwrap();
        for c in (sec * 128).max(3)..((sec + 1) * 128).min(last + 1) {
            let v: u32 = if c == last { 0x0fff_ffff } else { c as u32 + 1 };
            s[(c % 128) as usize * 4..(c % 128) as usize * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        for copy in 0..fs.fats as u64 { f.write_blocks(fat + copy * fs.fat_sectors as u64 + sec, &s).unwrap(); }
    }
    drop(fs.create(&mut f, "FILLER.BIN").unwrap());
    let root = fs.data_lba;
    let mut s = [0u8; 512];
    f.read_blocks(root, &mut s).unwrap();
    let slot = s.chunks(32).position(|e| &e[..11] == b"FILLER  BIN").unwrap();
    let e = &mut s[slot * 32..slot * 32 + 32];
    e[20..22].copy_from_slice(&0u16.to_le_bytes());
    e[26..28].copy_from_slice(&3u16.to_le_bytes());
    e[28..32].copy_from_slice(&((n * cb).min(u32::MAX as u64) as u32).to_le_bytes());
    f.write_blocks(root, &s).unwrap();
    let mut info = [0u8; 512];
    f.read_blocks(fs.volume_lba + fs.fsinfo as u64, &mut info).unwrap();
    let free = fs.clusters - 1 - n as u32;
    info[488..492].copy_from_slice(&free.to_le_bytes());
    info[492..496].copy_from_slice(&(last as u32).to_le_bytes());
    f.write_blocks(fs.volume_lba + fs.fsinfo as u64, &info).unwrap();
}

// ---- the firmware's RLOG lines ----

/// One `RLOG` line: its kind, its key=value fields (a value may hold spaces, as a `Debug` of a
/// struct variant does), and when the console had it.
#[derive(Clone, Debug)]
struct Rlog { at_ns: u64, kind: String, f: BTreeMap<String, String> }
impl Rlog {
    fn get(&self, k: &str) -> &str { self.f.get(k).map_or("", |s| s.as_str()) }
    fn num(&self, k: &str) -> u64 { self.get(k).parse().unwrap_or(0) }
    fn t(&self) -> u64 { self.num("t") }
}

fn rlogs(lines: &[(u64, String)]) -> Vec<Rlog> {
    let mut out = Vec::new();
    for (at, l) in lines {
        let Some(rest) = l.split("RLOG ").nth(1) else { continue };
        let mut it = rest.split(' ');
        let kind = it.next().unwrap_or("").to_string();
        let mut f: BTreeMap<String, String> = BTreeMap::new();
        let mut last: Option<String> = None;
        for tok in it {
            match tok.split_once('=') {
                Some((k, v)) if !k.is_empty() && k.chars().all(|c| c.is_ascii_lowercase() || c == '_') => { f.insert(k.to_string(), v.to_string()); last = Some(k.to_string()); }
                _ => if let Some(k) = &last { let v = f.get_mut(k).unwrap(); v.push(' '); v.push_str(tok); },
            }
        }
        out.push(Rlog { at_ns: *at, kind, f });
    }
    out
}

/// `lba+count,...`
fn spans(s: &str) -> Vec<(u64, u64)> { s.split(',').filter_map(|x| { let (a, b) = x.split_once('+')?; Some((a.parse().ok()?, b.parse().ok()?)) }).collect() }

/// `t:lba+count@i,...`: the retained writes that hit the watch.
fn hits(s: &str) -> Vec<(u64, u64, u64, usize)> {
    s.split(',').filter_map(|x| { let (t, r) = x.split_once(':')?; let (span, i) = r.split_once('@')?; let (a, b) = span.split_once('+')?; Some((t.parse().ok()?, a.parse().ok()?, b.parse().ok()?, i.parse().ok()?)) }).collect()
}

// ---- the medium's history ----

/// The card's sectors at any time: the image after the car's mount, then every landing of the car
/// in order. Metadata landings carry their bytes (`PageCache::meta_landed`); a data landing does
/// not, so a sector whose last write was data answers `None`.
/// When a sector was written, and where its bytes are: a metadata landing and the sector within it, or
/// `None` for a data landing.
type Written = (u64, Option<(usize, usize)>);
struct Medium { base: FileDisk, idx: HashMap<u64, Vec<Written>>, meta: Vec<(u64, u64, Vec<u8>)> }

impl Medium {
    fn new(base: &std::path::Path, landings: &[Landing], meta: Vec<(u64, u64, Vec<u8>)>) -> Self {
        let mut idx: HashMap<u64, Vec<Written>> = HashMap::new();
        for (k, (at, lba, data)) in meta.iter().enumerate() {
            for i in 0..data.len() / 512 { idx.entry(lba + i as u64).or_default().push((*at, Some((k, i)))); }
        }
        for l in landings.iter().filter(|l| !l.meta) {
            for lba in l.lba..l.lba + l.count as u64 { if let Some(v) = idx.get_mut(&lba) { v.push((l.at_ns, None)); } }
        }
        for v in idx.values_mut() { v.sort_by_key(|x| x.0); }
        Medium { base: FileDisk(std::fs::File::open(base).unwrap()), idx, meta }
    }

    fn sector(&mut self, lba: u64, t_ns: u64) -> Option<[u8; 512]> {
        if let Some(v) = self.idx.get(&lba) {
            if let Some(&(_, src)) = v.iter().rev().find(|x| x.0 <= t_ns) {
                let (k, i) = src?;
                return Some(self.meta[k].2[i * 512..i * 512 + 512].try_into().unwrap());
            }
        }
        let mut s = [0u8; 512];
        self.base.read_blocks(lba, &mut s).ok()?;
        Some(s)
    }
}

/// The volume's layout, from the car's mount.
#[derive(Clone, Copy)]
struct Geo { fat0: u64, fat_sectors: u64, fats: u64, data: u64, spc: u64 }
impl Geo {
    fn of(fs: &Fat32) -> Self { Geo { fat0: fs.volume_lba + fs.reserved as u64, fat_sectors: fs.fat_sectors as u64, fats: fs.fats as u64, data: fs.data_lba, spc: fs.sectors_per_cluster as u64 } }
    fn fat_lba(&self, copy: u64, c: u32) -> u64 { self.fat0 + copy * self.fat_sectors + c as u64 / 128 }
    fn is_fat(&self, lba: u64) -> bool { lba >= self.fat0 && lba < self.fat0 + self.fats * self.fat_sectors }
    fn cluster_of(&self, lba: u64) -> Option<u32> { (lba >= self.data).then(|| ((lba - self.data) / self.spc) as u32 + 2) }
}

fn entry32(s: &[u8; 512], c: u32) -> u32 { let at = (c % 128) as usize * 4; u32::from_le_bytes(s[at..at + 4].try_into().unwrap()) & 0x0fff_ffff }

/// The cluster chain from `first` in FAT copy 0 as it stood at `t`.
fn chain_at(m: &mut Medium, g: Geo, first: u32, t: u64) -> Option<Vec<u32>> {
    let mut out = Vec::new();
    let mut c = first;
    while (2..0x0fff_fff8).contains(&c) {
        if out.len() > 1 << 20 { return None; }
        out.push(c);
        c = entry32(&m.sector(g.fat_lba(0, c), t)?, c);
    }
    Some(out)
}

/// The slots of a file's entry (its long-name entries, then its 8.3 entry) at `t`, from where the
/// firmware listed it: the 8.3 entry at `offset` of `lba`, the long name back to `first_lba`.
fn entry_slots(m: &mut Medium, lba: u64, offset: u64, first_lba: u64, t: u64) -> Vec<(u64, usize)> {
    let mut slots = vec![(lba, offset as usize / 32)];
    if first_lba > lba || lba - first_lba > 4 { return slots; }
    let mut secs: Vec<(u64, [u8; 512])> = Vec::new();
    for s in first_lba..=lba { if let Some(b) = m.sector(s, t) { secs.push((s, b)); } else { return slots; } }
    let flat: Vec<(u64, usize, [u8; 32])> = secs.iter().flat_map(|(s, b)| (0..16).map(move |i| (*s, i, b[i * 32..i * 32 + 32].try_into().unwrap()))).collect();
    let Some(at) = flat.iter().position(|x| x.0 == lba && x.1 == offset as usize / 32) else { return slots };
    for j in (0..at).rev() {
        let e = &flat[j].2;
        if e[0] == 0xe5 || e[11] & 0x3f != 0x0f { break; }
        slots.insert(0, (flat[j].0, flat[j].1));
        if e[0] & 0x40 != 0 { break; }
    }
    slots
}

fn slot_bytes(m: &mut Medium, slots: &[(u64, usize)], t: u64) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    for &(lba, i) in slots { out.extend_from_slice(&m.sector(lba, t)?[i * 32..i * 32 + 32]); }
    Some(out)
}

// ---- classification ----

/// The causes a refusal or discard is put down to, each exactly one (esp32p4.md, "Shared-pass
/// refusals by cause", defines them).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
enum Cause {
    /// the dirty log answered Unknown (wrapped past the read's start, or no answer) and nothing the
    /// reduced design counts as a change happened
    A,
    /// a watched FAT sector was written, the clip's chain entries in it unchanged
    B,
    /// the watched directory sector was written, the clip's 8.3 entry and long name unchanged
    C,
    /// the clip changed during the read: a data cluster of its chain written, its chain entries or
    /// its entry changed
    D,
    /// refused at listing: the entry and the chain disagree, the walk failed; or the clip was not a
    /// finished MP4 when opened
    E,
    /// a hit inside a watch widened past the clip (a bounding run, or a listing-time span the chain
    /// no longer holds), the clip unchanged
    G,
}
impl Cause {
    fn label(self) -> &'static str {
        match self { Cause::A => "(a) ring wrapped / no answer", Cause::B => "(b) FAT sector false sharing", Cause::C => "(c) directory sector false sharing", Cause::D => "(d) the clip changed during the read", Cause::E => "(e) structural / in flight at listing or open", Cause::G => "(g) widened watch, outside the clip" }
    }
    fn removable(self) -> bool { matches!(self, Cause::A | Cause::B | Cause::C | Cause::G) }
}

/// One refusal or discard, judged.
#[derive(Clone, Debug)]
struct Judged { at_ns: u64, pass: usize, path: String, cause: Cause, why: String, detail: String, since_ns: u64 }

/// What the medium says changed for one read, `t0` to `t1`.
#[derive(Default, Debug)]
struct Change { data: usize, data_by_deletion: bool, chain_fat1: bool, chain_fat2: bool, entry: bool, entry_unknown: bool, chain_unknown: bool, chain_len: usize }
impl Change { fn genuine(&self) -> bool { self.data > 0 || self.chain_fat1 || self.chain_fat2 || self.entry || self.chain_unknown || self.entry_unknown } }

/// What the car believed of a file when the run ended.
#[derive(Clone, Debug)]
struct Truth { closed_ns: Option<u64>, complete: bool, deleted: bool, started_ns: Option<u64> }

/// Everything the judging reads, kept so a run can be judged again: `REFUSAL_OUT` writes it,
/// `external_oxidrive_refusals_judged_again` reads it back from `REFUSAL_IN`.
struct RunData {
    /// the P4's console, each line with the time of the slice it came in
    lines: Vec<(u64, String)>,
    /// every run of blocks the card was written
    writes: Vec<CardWrite>,
    /// the car's writes as they reached the drive, and the bytes of its metadata ones
    landings: Vec<Landing>,
    meta: Vec<(u64, u64, Vec<u8>)>,
    truth: BTreeMap<String, Truth>,
    /// every clip on the share (its path under TeslaCam), and whether it equals the car's file
    share: BTreeMap<String, bool>,
    /// the card image after the car's mount
    baseline: std::path::PathBuf,
    geo: Geo,
    rec_start: u64,
    rec_end: u64,
}

fn dump(d: &RunData, dir: &std::path::Path) {
    use std::io::Write;
    std::fs::create_dir_all(dir).unwrap();
    let tsv = |name: &str, rows: String| std::fs::write(dir.join(name), rows).unwrap();
    tsv("console.tsv", d.lines.iter().map(|(t, l)| format!("{t}\t{}\n", l.replace(['\t', '\n'], " "))).collect());
    tsv("writes.tsv", d.writes.iter().map(|w| format!("{}\t{}\t{}\n", w.at_ns, w.lba, w.count)).collect());
    tsv("landings.tsv", d.landings.iter().map(|l| format!("{}\t{}\t{}\t{}\t{}\t{}\n", l.at_ns, l.lba, l.count, l.meta as u8, l.dirtied_ns, l.due_ns)).collect());
    tsv("truth.tsv", d.truth.iter().map(|(p, t)| format!("{p}\t{}\t{}\t{}\t{}\n", t.closed_ns.map_or("-".into(), |c| c.to_string()), t.complete as u8, t.deleted as u8, t.started_ns.map_or("-".into(), |c| c.to_string()))).collect());
    tsv("share.tsv", d.share.iter().map(|(p, s)| format!("{p}\t{}\n", *s as u8)).collect());
    let g = d.geo;
    tsv("run.txt", format!("{} {} {} {} {} {} {}\n", g.fat0, g.fat_sectors, g.fats, g.data, g.spc, d.rec_start, d.rec_end));
    let mut f = std::io::BufWriter::new(std::fs::File::create(dir.join("meta.bin")).unwrap());
    for (at, lba, data) in &d.meta { f.write_all(&at.to_le_bytes()).unwrap(); f.write_all(&lba.to_le_bytes()).unwrap(); f.write_all(&(data.len() as u32).to_le_bytes()).unwrap(); f.write_all(data).unwrap(); }
    drop(f);
    // an APFS clone where the file system makes one
    if d.baseline != dir.join("baseline.img") { std::fs::copy(&d.baseline, dir.join("baseline.img")).unwrap(); }
}

fn load(dir: &std::path::Path) -> RunData {
    let read = |name: &str| std::fs::read_to_string(dir.join(name)).unwrap_or_else(|e| panic!("{}: {e}", dir.join(name).display()));
    let cols = |l: &str| l.split('\t').map(str::to_string).collect::<Vec<_>>();
    let n = |s: &str| s.parse::<u64>().unwrap();
    let lines = read("console.tsv").lines().filter_map(|l| { let (t, x) = l.split_once('\t')?; Some((n(t), x.to_string())) }).collect();
    let writes = read("writes.tsv").lines().map(|l| { let c = cols(l); CardWrite { at_ns: n(&c[0]), lba: n(&c[1]), count: n(&c[2]) } }).collect();
    let landings = read("landings.tsv").lines().map(|l| { let c = cols(l); Landing { at_ns: n(&c[0]), lba: n(&c[1]), count: n(&c[2]) as u32, meta: c[3] == "1", dirtied_ns: n(&c[4]), due_ns: n(&c[5]) } }).collect();
    let truth = read("truth.tsv").lines().map(|l| { let c = cols(l); (c[0].clone(), Truth { closed_ns: c[1].parse().ok(), complete: c[2] == "1", deleted: c[3] == "1", started_ns: c.get(4).and_then(|s| s.parse().ok()) }) }).collect();
    let share = read("share.tsv").lines().map(|l| { let c = cols(l); (c[0].clone(), c[1] == "1") }).collect();
    let r: Vec<u64> = read("run.txt").split_whitespace().map(n).collect();
    let bytes = std::fs::read(dir.join("meta.bin")).unwrap();
    let (mut meta, mut at) = (Vec::new(), 0usize);
    while at + 20 <= bytes.len() {
        let t = u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap());
        let lba = u64::from_le_bytes(bytes[at + 8..at + 16].try_into().unwrap());
        let len = u32::from_le_bytes(bytes[at + 16..at + 20].try_into().unwrap()) as usize;
        meta.push((t, lba, bytes[at + 20..at + 20 + len].to_vec()));
        at += 20 + len;
    }
    RunData { lines, writes, landings, meta, truth, share, baseline: dir.join("baseline.img"), geo: Geo { fat0: r[0], fat_sectors: r[1], fats: r[2], data: r[3], spc: r[4] }, rec_start: r[5], rec_end: r[6] }
}

/// The share's clips, by their path under TeslaCam, and whether each equals the car's file.
fn share_clips(root: &std::path::Path, files: &BTreeMap<String, CarFile>) -> BTreeMap<String, bool> {
    let mut out = BTreeMap::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).into_iter().flatten().flatten() {
            let p = e.path();
            if p.is_dir() { stack.push(p); continue; }
            let rel = p.strip_prefix(root).unwrap().to_string_lossy().to_string();
            if rel.starts_with('.') || rel.ends_with(".part") || rel.ends_with(".tsv") { continue; }
            let same = files.get(&format!("TeslaCam/{rel}")).is_some_and(|f| std::fs::read(&p).is_ok_and(|b| b == f.data));
            out.insert(rel, same);
        }
    }
    out
}

/// What a judged run sums to, for the assertions.
struct Summary { passes: usize, own: u64, windows: usize, torn: usize, total: usize, classified: usize, unmatched: usize }

#[test]
#[ignore = "needs the oxidrive sim-refusal-log images and nvs.csv under OXIDRIVE_REFUSAL_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_shared_pass_refusals_by_cause() {
    let card_mib = env_u64("REFUSAL_CARD_MIB", 4096);
    let record_secs = env_u64("REFUSAL_RECORD_SECS", 720);
    let mut sentry = Sentry::from_env();
    let free = env_u64("REFUSAL_FREE_MIB", sentry.needs() >> 20);
    let prg = env_u64("OXIDRIVE_SD_PRG_US", 0);
    let proto = common::tmp("refusals-proto.img");
    sd_image::fat32(&proto, card_mib << 20, 32 << 10, "ESP32SIM").unwrap();
    fill_volume(&proto, free << 20);
    eprintln!("[refusals] card {card_mib} MiB, {free} MiB free; SD bus timed, programming {prg} us; firmware {}", refusal_dir());
    eprintln!("[refusals] recorder: 4 cameras, {} KiB clips every {}, ring {} sets, an event every {} sets ({} sets of pre-roll and the trigger set, {} copies a step), {} events kept; {} s of recording",
              sentry.clip >> 10, secs(sentry.set_ns), sentry.ring, sentry.event_every, sentry.preroll, sentry.copies, sentry.events_kept, record_secs);
    let o = RigOpts { card_name: "refusals.img".into(), card_bytes: card_mib << 20, car: true, image: Some(proto.clone()), prg_us: Some(prg), timed: Some(true), log_writes: true, ..RigOpts::new(refusal_dir()) };
    let mut r = rig_with(o);
    // The NAT reaps a flow idle for 120 s without telling either end; the firmware keeps its SMB
    // session between passes, and a NAS on its LAN would keep it open.
    let idle_s = env_u64("REFUSAL_NAT_IDLE_S", 3600);
    if let Some(nat) = r.cs.c6.borrow_mut().m.bus.periph.wifi_mac.link.net_mut().and_then(|n| n.nat.as_mut()) { nat.idle_close_us = idle_s * 1_000_000; }
    let shared = r.shared.clone();
    let card = r.card.clone();
    let out = std::env::var("REFUSAL_OUT").ok().map(std::path::PathBuf::from);
    let baseline = match &out { Some(d) => { std::fs::create_dir_all(d).unwrap(); d.join("baseline.img") } None => common::tmp("refusals-baseline.img") };
    let wall = std::time::Instant::now();
    let (files, landings, meta, fs, rec_start, rec_end) = {
        let mut car = car_on(&mut r, CarCfg { linux_entries: true, dirty_flag: true, ..CarCfg::default() }, WritebackCfg::default());
        std::fs::copy(&card, &baseline).unwrap();
        let rec_start = car.now();
        let end = rec_start + record_secs * S;
        let mut last_note = 0;
        while car.now() < end {
            let st = sentry.step(&mut car).unwrap_or_else(|e| panic!("the recorder: {e}"));
            if !matches!(st, Recorded::Closed { .. }) { eprintln!("[refusals] a set was cut at {}: {st:?}", secs(car.now())); }
            if car.now() >= last_note + 60 * S {
                last_note = car.now();
                let sh = shared.borrow();
                eprintln!("[refusals] {} emulated ({:.0} s wall): {} sets, {} events, {} passes ended, {} clips uploaded", secs(car.now()), wall.elapsed().as_secs_f64(), sentry.sets, sentry.events,
                          sh.all("RLOG pass-end").len(), sh.lines.iter().filter(|l| l.1.contains("RLOG clip") && l.1.contains("outcome=uploaded")).count());
            }
        }
        let rec_end = car.now();
        eprintln!("[refusals] car: {:?}; its cache: {:?}", car.stats, car.disk.stats);
        let fs = car.fs.clone().expect("a mounted volume");
        (car.files.clone(), std::mem::take(&mut car.disk.landings), std::mem::take(&mut car.disk.meta_landed), fs, rec_start, rec_end)
    };
    r.report(10);
    eprintln!("[refusals] {:.0} s wall for {} emulated", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));
    let sh = shared.borrow();
    let data = RunData {
        lines: sh.lines.clone(), writes: sh.writes.clone(), landings, meta,
        truth: files.iter().map(|(p, f)| (p.clone(), Truth { closed_ns: f.closed_ns, complete: f.complete, deleted: f.deleted, started_ns: Some(f.started_ns) })).collect(),
        share: share_clips(&r.smb.path(), &files), baseline, geo: Geo::of(&fs), rec_start, rec_end,
    };
    if let Some(d) = &out { dump(&data, d); eprintln!("[refusals] the run's data in {}", d.display()); }
    let s = analyse(&data, out.as_deref());
    assert!(s.passes > 0, "no shared pass ended");
    assert_eq!(s.own, 0, "oxidrive wrote the card");
    assert_eq!(s.windows, 0, "a window opened: the car stopped recording for a minute");
    assert_eq!(s.torn, 0, "a clip on the share differs from the car's file");
    assert_eq!(s.classified, s.total, "every refusal in exactly one class");
    assert_eq!(s.unmatched, 0, "a re-check with no open before it");
}

/// The same judging, on a run kept with `REFUSAL_OUT`, read back from `REFUSAL_IN`.
#[test]
#[ignore = "needs a run kept by external_oxidrive_shared_pass_refusals_by_cause under REFUSAL_IN"]
fn external_oxidrive_refusals_judged_again() {
    let dir = std::path::PathBuf::from(std::env::var("REFUSAL_IN").expect("REFUSAL_IN: a directory REFUSAL_OUT wrote"));
    let data = load(&dir);
    let out = std::env::var("REFUSAL_OUT").ok().map(std::path::PathBuf::from);
    let s = analyse(&data, out.as_deref());
    assert_eq!(s.classified, s.total, "every refusal in exactly one class");
}

fn analyse(data: &RunData, out: Option<&std::path::Path>) -> Summary {
    let g = data.geo;
    let logs = rlogs(&data.lines);
    let q = |v: &[u64], f: f64| v.get(((v.len() as f64 * f) as usize).min(v.len().saturating_sub(1))).copied().unwrap_or(0);

    // oxidrive's own writes: blocks the car's landings do not account for within a second
    let mut car_blocks: HashMap<u64, Vec<u64>> = HashMap::new();
    for l in &data.landings { for lba in l.lba..l.lba + l.count as u64 { car_blocks.entry(lba).or_default().push(l.at_ns); } }
    let own: u64 = data.writes.iter().filter(|w| w.at_ns >= data.rec_start).map(|w| (w.lba..w.lba + w.count).filter(|lba| !car_blocks.get(lba).is_some_and(|v| v.iter().any(|&t| t + S >= w.at_ns && t <= w.at_ns + S))).count() as u64).sum();
    drop(car_blocks);
    let windows = data.lines.iter().filter(|l| l.1.contains("supervisor: card taken for archiving")).count();
    eprintln!("[refusals] recording {} to {}; blocks written that the car did not land: {own}; windows: {windows}", secs(data.rec_start), secs(data.rec_end));

    // the firmware's clock against the emulator's: every write the firmware logged (the hits, and
    // the last writes its log held at a re-check) is a run of blocks the card took; the median
    // distance to the nearest such run in the card's log, which the rig times to its slice
    let mut d: Vec<i64> = Vec::new();
    let w = &data.writes;
    for x in logs.iter().filter(|x| x.kind == "reval") {
        let entries: Vec<(u64, u64)> = hits(x.get("hits")).into_iter().map(|h| (h.0, h.1)).chain(x.get("last").split(',').filter_map(|e| { let (t, r) = e.split_once(':')?; Some((t.parse().ok()?, r.split('+').next()?.parse().ok()?)) })).collect();
        for (t, lba) in entries {
            let fw = (t * MS) as i64;
            let lo = w.partition_point(|c| (c.at_ns as i64) < fw - 2_000_000_000);
            let near = w[lo..].iter().take_while(|c| (c.at_ns as i64) < fw + 2_000_000_000).filter(|c| c.lba <= lba && lba < c.lba + c.count).map(|c| c.at_ns as i64 - fw).min_by_key(|x| x.abs());
            if let Some(n) = near { d.push(n); }
        }
    }
    d.sort_unstable();
    let line_lag: Vec<i64> = logs.iter().filter(|x| x.f.contains_key("t")).map(|x| x.at_ns as i64 - (x.t() * MS) as i64).collect();
    let off = d.get(d.len() / 2).copied().unwrap_or_else(|| line_lag.iter().copied().min().unwrap_or(0));
    let pick = |f: f64| d.get(((d.len() as f64 * f) as usize).min(d.len().saturating_sub(1))).map_or(0.0, |&x| x as f64 / 1e6);
    eprintln!("[refusals] clock: the firmware's ms + {:.3} ms is the emulator's ({} logged writes matched to the card's; card minus firmware p10/p50/p90 {:.3}/{:.3}/{:.3} ms; console line minus firmware min {:.3} ms)",
              off as f64 / 1e6, d.len(), pick(0.1), pick(0.5), pick(0.9), line_lag.iter().copied().min().unwrap_or(0) as f64 / 1e6);
    let emu = |t_fw: u64| ((t_fw * MS) as i64 + off).max(0) as u64;

    let mut medium = Medium::new(&data.baseline, &data.landings, data.meta.clone());
    let mut lands = data.landings.clone();
    lands.sort_by_key(|l| l.at_ns);

    // passes, by their start lines; where each stopped early
    let starts: Vec<u64> = logs.iter().filter(|x| x.kind == "pass-start").map(|x| x.t()).collect();
    let pass_of = |t_fw: u64| starts.iter().rposition(|&s| s <= t_fw).map_or(0, |i| i + 1);
    let stop_at: BTreeMap<usize, (u64, String)> = {
        let mut m = BTreeMap::new();
        for (t, l) in &data.lines {
            if l.contains("card went slow for the car") { let fw = (*t as i64 - off).max(0) as u64 / MS; m.entry(pass_of(fw)).or_insert((fw, "card went slow".to_string())); }
        }
        for x in logs.iter().filter(|x| x.kind == "stall") { m.entry(pass_of(x.t())).or_insert((x.t(), "card busy".to_string())); }
        m
    };
    // the warning's time is its slice's, up to 100 ms after the firmware wrote it
    let after_stop = |t_fw: u64| stop_at.get(&pass_of(t_fw)).is_some_and(|s| t_fw + 200 >= s.0);

    let mut judged: Vec<Judged> = Vec::new();
    let mut accepted: Vec<(u64, String)> = Vec::new();
    let mut opens: HashMap<String, Rlog> = HashMap::new();
    let mut revalled: HashSet<(String, u64)> = HashSet::new();
    let mut watch_ms: Vec<u64> = Vec::new();
    let mut coverage: Vec<(u64, u64)> = Vec::new();
    let mut combos: BTreeMap<String, usize> = BTreeMap::new();
    let mut d_kinds: BTreeMap<&'static str, usize> = BTreeMap::new();
    let (mut stops_in_fail, mut unmatched, mut failed_other, mut multi, mut removable_torn) = (0usize, 0usize, 0usize, 0usize, 0usize);
    let set_ns = env_u64("REFUSAL_SET_SECS", 20) * S;
    for x in &logs {
        match x.kind.as_str() {
            "open" => { opens.insert(x.get("path").to_string(), x.clone()); }
            "list-refuse" => {
                let why = x.get("why").to_string();
                let stopped = x.get("stalled") == "true" || (x.get("site") != "verdict" && after_stop(x.t()));
                let note = if stopped { " (after the pass's stop)" } else { "" };
                judged.push(Judged { at_ns: emu(x.t()), pass: pass_of(x.t()), path: x.get("path").to_string(), cause: Cause::E, why: format!("listing: {}{note}", why.split([' ', '(']).next().unwrap_or("")),
                                     detail: format!("site={} why={why} size={} first={} clusters={} end={}", x.get("site"), x.get("size"), x.get("first"), x.get("clusters"), x.get("end")), since_ns: 0 });
            }
            "reval" => {
                let path = x.get("path").to_string();
                revalled.insert((path.clone(), opens.get(&path).map_or(0, |o| o.t())));
                let since_fw = x.num("since");
                watch_ms.push(x.t().saturating_sub(since_fw));
                if x.num("lost") > 0 { if let (Ok(o), Ok(n)) = (x.get("oldest").parse::<u64>(), x.get("newest").parse::<u64>()) { coverage.push((n.saturating_sub(o), x.t().saturating_sub(since_fw))); } }
                let verdict = x.get("verdict");
                if verdict == "Clean" { accepted.push((emu(x.t()), path)); continue; }
                let Some(o) = opens.get(&path).cloned() else { unmatched += 1; continue };
                let (t0, t1) = (emu(since_fw), emu(x.t()));
                let first: u32 = o.num("first") as u32;
                let parts: Vec<u64> = o.get("dirent").split(':').filter_map(|v| v.parse().ok()).collect();
                let (dlba, doff, dfirst) = match parts[..] { [a, b, c] => (a, b, c), _ => { unmatched += 1; continue } };
                // what changed, by the medium
                let mut ch = Change::default();
                let chain = chain_at(&mut medium, g, first, t0);
                match &chain {
                    None => ch.chain_unknown = true,
                    Some(chain) => {
                        ch.chain_len = chain.len();
                        let set: HashSet<u32> = chain.iter().copied().collect();
                        let lo = lands.partition_point(|l| l.at_ns <= t0);
                        for l in lands[lo..].iter().take_while(|l| l.at_ns <= t1) {
                            if (l.lba..l.lba + l.count as u64).any(|s| g.cluster_of(s).is_some_and(|k| set.contains(&k))) { ch.data += 1; }
                        }
                        if ch.data > 0 {
                            // the chain freed: its first cluster's entry free after the read
                            ch.data_by_deletion = medium.sector(g.fat_lba(0, first), t1).is_some_and(|s| entry32(&s, first) == 0);
                        }
                        for copy in 0..g.fats {
                            for &c in chain {
                                match (medium.sector(g.fat_lba(copy, c), t0), medium.sector(g.fat_lba(copy, c), t1)) {
                                    (Some(a), Some(b)) => if entry32(&a, c) != entry32(&b, c) { if copy == 0 { ch.chain_fat1 = true } else { ch.chain_fat2 = true } },
                                    _ => ch.chain_unknown = true,
                                }
                            }
                        }
                    }
                }
                let slots = entry_slots(&mut medium, dlba, doff, dfirst, t0);
                match (slot_bytes(&mut medium, &slots, t0), slot_bytes(&mut medium, &slots, t1)) { (Some(a), Some(b)) => ch.entry = a != b, _ => ch.entry_unknown = true }
                let entry_sectors: BTreeSet<u64> = slots.iter().map(|s| s.0).collect();
                let hs = hits(x.get("hits"));
                let chain_set: HashSet<u32> = chain.clone().unwrap_or_default().into_iter().collect();
                let fat_with_entries: HashSet<u64> = (0..g.fats).flat_map(|copy| chain_set.iter().map(move |&c| (copy, c))).map(|(copy, c)| g.fat_lba(copy, c)).collect();
                // each hit sector inside the watch: a FAT sector holding a chain entry or only
                // inside the watched range, the entry's sectors, the clip's own data, or elsewhere
                let ws = spans(x.get("watch"));
                let mut kinds: BTreeSet<&'static str> = BTreeSet::new();
                for &(_, lba, count, _) in &hs {
                    for s in (lba..lba + count).filter(|s| ws.iter().any(|w| w.0 <= *s && *s < w.0 + w.1)) {
                        kinds.insert(if g.is_fat(s) { if fat_with_entries.contains(&s) { "fat" } else { "fat-range" } }
                                     else if entry_sectors.contains(&s) || (s >= dfirst.min(dlba) && s <= dlba) { "dirent" }
                                     else if g.cluster_of(s).is_some_and(|c| chain_set.contains(&c)) { "own-data" }
                                     else { "outside" });
                    }
                }
                let combo = kinds.iter().copied().collect::<Vec<_>>().join("+");
                let (cause, why) = if ch.genuine() {
                    let k = if ch.data > 0 { if ch.data_by_deletion { "data written: clusters freed and reused" } else { "data written: a late landing on the clip's clusters" } }
                            else if ch.chain_fat1 || ch.chain_fat2 { "chain entries changed" } else if ch.entry { "entry or long name changed" } else { "history unknown (counted as changed)" };
                    *d_kinds.entry(k).or_default() += 1;
                    (Cause::D, format!("{verdict}: {k}"))
                } else if verdict != "Dirty(Yes)" {
                    (Cause::A, format!("{verdict}: lost={} oldest={} since={}", x.get("lost"), x.get("oldest"), since_fw))
                } else if kinds.contains("own-data") {
                    // a hit on the clip's own data the medium does not show inside the window:
                    // counted as changed, conservatively
                    *d_kinds.entry("own-data hit at the window's edge").or_default() += 1;
                    (Cause::D, "Dirty(Yes): own-data hit at the window's edge".to_string())
                } else if kinds.contains("fat") || kinds.contains("fat-range") {
                    (Cause::B, format!("Dirty(Yes): {combo}"))
                } else if kinds.contains("dirent") {
                    (Cause::C, format!("Dirty(Yes): {combo}"))
                } else {
                    (Cause::G, format!("Dirty(Yes): {combo}{}", if x.get("bounded") == "true" { ", bounded" } else { ", stale listing span" }))
                };
                // would the reduced design have published a whole clip? The car had closed the
                // file by the read's start, and every cluster of the chain then had data landed
                // on it after the file was begun (a landing before is a previous owner's bytes).
                let mut torn = String::new();
                if cause.removable() {
                    *combos.entry(combo.clone()).or_default() += 1;
                    if kinds.iter().filter(|k| ["fat", "fat-range", "dirent", "outside"].contains(k)).map(|k| if k.starts_with("fat") { "fat" } else { k }).collect::<BTreeSet<_>>().len() > 1 { multi += 1; }
                    let t = data.truth.get(&format!("TeslaCam/{path}"));
                    let closed = t.and_then(|t| t.closed_ns).is_some_and(|c| c <= t0);
                    let started = t.and_then(|t| t.started_ns.or_else(|| t.closed_ns.map(|c| c.saturating_sub(set_ns + 5 * S)))).unwrap_or(0);
                    let lo = lands.partition_point(|l| l.at_ns < started);
                    let mut landed: HashSet<u32> = HashSet::new();
                    for l in lands[lo..].iter().take_while(|l| l.at_ns <= t0).filter(|l| !l.meta) {
                        for s in (l.lba..l.lba + l.count as u64).step_by(g.spc as usize) { if let Some(c) = g.cluster_of(s) { if chain_set.contains(&c) { landed.insert(c); } } }
                    }
                    let missing = chain_set.len() - landed.len();
                    if !closed || missing > 0 || chain_set.is_empty() {
                        torn = format!(" TORN-RISK closed_by_read={closed} clusters_not_landed={missing}");
                        removable_torn += 1;
                    }
                }
                judged.push(Judged { at_ns: t1, pass: pass_of(x.t()), path, cause, why, since_ns: t0,
                                     detail: format!("watch {} ms bounded={} watch=[{}] nhits={} hits=[{}] chain={} data_writes={} fat1={} fat2={} entry={}{torn}", x.t() - since_fw, x.get("bounded"), x.get("watch"), x.get("nhits"), x.get("hits"), ch.chain_len, ch.data, ch.chain_fat1, ch.chain_fat2, ch.entry) });
            }
            "clip" => {
                let path = x.get("path").to_string();
                let o = opens.get(&path);
                let had_reval = o.is_some_and(|o| revalled.contains(&(path.clone(), o.t())));
                match x.get("outcome") {
                    "incomplete" if !had_reval => judged.push(Judged { at_ns: emu(x.t()), pass: pass_of(x.t()), path, cause: Cause::E, why: if o.is_some() { "open: not a finished MP4".into() } else { "open: vanished".into() }, detail: String::new(), since_ns: 0 }),
                    "failed" if after_stop(x.t()) => stops_in_fail += 1,
                    "failed" => failed_other += 1,
                    _ => {}
                }
            }
            _ => {}
        }
    }

    // passes
    let ends: Vec<&Rlog> = logs.iter().filter(|x| x.kind == "pass-end").collect();
    let mut stop_counts: BTreeMap<String, usize> = BTreeMap::new();
    for e in &ends {
        let k = match (e.get("aborted"), e.get("outcome")) { ("-", "yielded") => "yielded: the 60 s budget".to_string(), ("-", o) => format!("ran to the end ({o})"), (a, _) => format!("stopped: {a}") };
        *stop_counts.entry(k).or_default() += 1;
    }

    // uploads, against the car's files
    let uploaded: BTreeMap<String, u64> = logs.iter().filter(|x| x.kind == "clip" && x.get("outcome") == "uploaded").fold(BTreeMap::new(), |mut m, x| { m.entry(x.get("path").to_string()).or_insert(emu(x.t())); m });
    let torn = data.share.iter().filter(|(p, same)| !**same && (p.ends_with(".mp4"))).count();
    for (p, _) in data.share.iter().filter(|(p, same)| !**same && p.ends_with(".mp4")) { eprintln!("[refusals] TORN or not the car's: {p}"); }
    let event_clips = data.truth.iter().filter(|(p, t)| p.starts_with("TeslaCam/SentryClips/") && p.ends_with(".mp4") && t.complete).count();

    // report: the classes
    let total = judged.len();
    let mut by: BTreeMap<Cause, usize> = BTreeMap::new();
    for j in &judged { *by.entry(j.cause).or_default() += 1; }
    let discards = judged.iter().filter(|j| j.since_ns > 0).count();
    eprintln!("[refusals] ---- refusals and discards by cause: {total} ({discards} discards after a read, {} at listing or open) ----", total - discards);
    for c in [Cause::A, Cause::B, Cause::C, Cause::D, Cause::E, Cause::G] {
        let n = by.get(&c).copied().unwrap_or(0);
        let clips: BTreeSet<&str> = judged.iter().filter(|j| j.cause == c).map(|j| j.path.as_str()).collect();
        eprintln!("[refusals] {:<46} {:>5} ({:>6}), {} clips", c.label(), n, pct(n, total), clips.len());
        let mut whys: BTreeMap<&str, usize> = BTreeMap::new();
        for j in judged.iter().filter(|j| j.cause == c) { *whys.entry(j.why.as_str()).or_default() += 1; }
        for (w, k) in whys.iter().take(12) { eprintln!("[refusals]     {k:>5}  {w}"); }
    }
    let abc = judged.iter().filter(|j| matches!(j.cause, Cause::A | Cause::B | Cause::C)).count();
    let removable = judged.iter().filter(|j| j.cause.removable()).count();
    let removable_discards = judged.iter().filter(|j| j.cause.removable() && j.since_ns > 0).count();
    eprintln!("[refusals] (a)+(b)+(c): {abc} of {total} ({}); with (g): {removable} ({}); of the {discards} discards after a read: {removable_discards} ({}); with hits in more than one false class: {multi}; where the clip may not have been whole at the read's start: {removable_torn}",
              pct(abc, total), pct(removable, total), pct(removable_discards, discards));
    eprintln!("[refusals] (d) by kind: {d_kinds:?}; removable discards by the hits they had: {combos:?}");
    eprintln!("[refusals] (f) passes: {} ended; {stop_counts:?}; uploads failed after a stop: {stops_in_fail}, failed otherwise: {failed_other}", ends.len());
    watch_ms.sort_unstable();
    eprintln!("[refusals] watches (open to re-check): {} re-checks, median {} ms, p90 {} ms, max {} ms", watch_ms.len(), q(&watch_ms, 0.5), q(&watch_ms, 0.9), q(&watch_ms, 1.0));
    let tight = coverage.iter().map(|&(c, w)| c as f64 / w.max(1) as f64).fold(f64::INFINITY, f64::min);
    let cov: Vec<u64> = { let mut v: Vec<u64> = coverage.iter().map(|c| c.0).collect(); v.sort_unstable(); v };
    eprintln!("[refusals] the dirty log at re-checks after it wrapped: {} re-checks, its reach back median {} ms, min {} ms; reach over the watch, least {:.1}x", coverage.len(), q(&cov, 0.5), q(&cov, 0.0), tight);
    let seqs: Vec<(u64, u64)> = logs.iter().filter(|x| x.kind == "reval" && x.f.contains_key("seq")).map(|x| (x.t(), x.num("seq"))).collect();
    if let (Some(a), Some(b)) = (seqs.first(), seqs.last()) { if b.0 > a.0 { eprintln!("[refusals] the car's writes as the tap counts them: {:.1} a second ({} from {:.1} s to {:.1} s), {} in all", (b.1 - a.1) as f64 * 1000.0 / (b.0 - a.0) as f64, b.1 - a.1, a.0 as f64 / 1e3, b.0 as f64 / 1e3, b.1); } }

    // accepted, archived, time to archive, and the counterfactual
    let accepted_clips: BTreeSet<&str> = accepted.iter().map(|a| a.1.as_str()).collect();
    let mut tta: Vec<u64> = Vec::new();
    for (p, &t) in &uploaded { if let Some(c) = data.truth.get(&format!("TeslaCam/{p}")).and_then(|f| f.closed_ns) { tta.push(t.saturating_sub(c)); } }
    tta.sort_unstable();
    eprintln!("[refusals] Sentry clips the car closed: {event_clips}; accepted by a re-check: {} re-checks of {} clips; archived: {} ({torn} on the share not equal to the car's file); from the car closing a clip to its upload: median {}, p90 {}, max {}",
              accepted.len(), accepted_clips.len(), uploaded.len(), secs(q(&tta, 0.5)), secs(q(&tta, 0.9)), secs(q(&tta, 1.0)));
    let mut first_removable: BTreeMap<&str, u64> = BTreeMap::new();
    for j in judged.iter().filter(|j| j.cause.removable() && j.since_ns > 0) { first_removable.entry(j.path.as_str()).or_insert(j.at_ns); }
    let (mut sooner, mut more) = (Vec::new(), 0usize);
    for (p, &t) in &first_removable { match uploaded.get(*p) { Some(&u) if u > t => sooner.push(u - t), Some(_) => {} None => more += 1 } }
    sooner.sort_unstable();
    eprintln!("[refusals] counterfactual: {} clips had a discard the reduced design removes; {} of them archived later in the run, median {} (p90 {}, max {}) after that discard; {more} not archived by the end of the run",
              first_removable.len(), sooner.len(), secs(q(&sooner, 0.5)), secs(q(&sooner, 0.9)), secs(q(&sooner, 1.0)));

    // per pass
    eprintln!("[refusals] ---- per pass ----");
    for (i, e) in ends.iter().enumerate() {
        let p = i + 1;
        let mut c: BTreeMap<Cause, usize> = BTreeMap::new();
        for j in judged.iter().filter(|j| j.pass == p) { *c.entry(j.cause).or_default() += 1; }
        let acc = accepted.iter().filter(|a| pass_of(((a.0 as i64 - off).max(0) as u64) / MS) == p).count();
        eprintln!("[refusals] pass {p:>2} {} to {}: {} {} uploaded={} accepted={acc} read={} refusals {:?}", secs(emu(starts.get(i).copied().unwrap_or(0))), secs(emu(e.t())), e.get("outcome"), e.get("aborted"), e.get("uploaded"), e.get("read"), c);
    }
    // worked examples: the first of each cause found after a read
    for c in [Cause::A, Cause::B, Cause::C, Cause::G, Cause::D] {
        for j in judged.iter().filter(|j| j.cause == c && j.since_ns > 0).take(2) { eprintln!("[refusals] example {}: {} at {}, pass {}: {}; {}", c.label(), j.path, secs(j.at_ns), j.pass, j.why, j.detail); }
    }
    if let Some(dir) = out {
        std::fs::create_dir_all(dir).unwrap();
        let rows: String = judged.iter().map(|j| format!("{}\t{}\t{:?}\t{}\t{}\t{}\t{}\n", j.at_ns, j.pass, j.cause, j.path, j.why, j.since_ns, j.detail)).collect();
        std::fs::write(dir.join("refusals.tsv"), format!("at_ns\tpass\tcause\tpath\twhy\tsince_ns\tdetail\n{rows}")).unwrap();
        let rows: String = uploaded.iter().map(|(p, t)| format!("{p}\t{t}\t{}\n", data.truth.get(&format!("TeslaCam/{p}")).and_then(|f| f.closed_ns).unwrap_or(0))).collect();
        std::fs::write(dir.join("uploads.tsv"), format!("path\tuploaded_ns\tclosed_ns\n{rows}")).unwrap();
        eprintln!("[refusals] tables in {}", dir.display());
    }
    let deleted = data.truth.values().filter(|t| t.deleted).count();
    eprintln!("[refusals] files the car deleted by the end: {deleted}");
    Summary { passes: ends.len(), own, windows, torn, total, classified: by.values().sum(), unmatched }
}
