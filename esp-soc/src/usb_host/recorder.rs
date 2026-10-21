//! The car's recorders, on either filesystem (`car::CarFs`): the Sentry-like recorder of
//! `cli/tests/oxidrive_refusals.rs` and the Tesla-like one of `cli/tests/oxidrive_hazards.rs`,
//! with the same steps and defaults, and the full card both record on. Those tests keep their own
//! FAT32 copies; these are for running the same workload on exFAT and FAT32 side by side.
use super::car::{Car, CarFs, Drive, Recorded};
use super::exfat::ExFat;
use super::fat32::{mp4_bytes, BlockDevice, Fat32};
use std::collections::{BTreeMap, VecDeque};

const S: u64 = 1_000_000_000;

/// Tesla's names: stamped to the minute of the recorder's clock.
pub fn stamp(minute: u32) -> String { format!("2026-01-01_{:02}-{:02}-00", minute / 60, minute % 60) }
pub const CAMS: [&str; 4] = ["front", "back", "left_repeater", "right_repeater"];

/// What a recorder wrote together: the paths of every `record_set` that closed, in order, and,
/// when `track_clusters` is set, each file's clusters read back right after its set closed.
#[derive(Clone, Debug, Default)]
pub struct Written {
    pub sets: Vec<(u64, Vec<String>)>,
    pub track_clusters: bool,
    pub clusters: BTreeMap<String, Vec<u32>>,
}

impl Written {
    fn record<D: Drive, F: CarFs>(&mut self, car: &mut Car<D, F>, paths: Vec<String>, r: &Recorded) {
        if !matches!(r, Recorded::Closed { .. }) { return; }
        if self.track_clusters {
            if let Some(fs) = car.fs.as_mut() {
                for p in &paths { if let Ok(c) = fs.clusters_of(&mut car.disk, p) { self.clusters.insert(p.clone(), c); } }
            }
        }
        let t = car.now();
        self.sets.push((t, paths));
    }
}

/// A Sentry-like recorder. Every step records a set of four camera clips into RecentClips, chunk i
/// of every clip due at t0 + i * gap. Every `event_every` sets a Sentry event: its folder, a small
/// `event.json` and `thumb.png`, and copies of the last `preroll` sets and of the trigger set,
/// written `copies` clips a step alongside the live set (the folder is a live write target for
/// several steps, at twice the camera rate). RecentClips keeps `ring` sets and SentryClips
/// `events_kept` events, the oldest deleted, so on a nearly full card the next clips take the
/// freed clusters.
#[derive(Clone, Debug)]
pub struct Sentry { pub set_ns: u64, pub clip: usize, pub ring: usize, pub event_every: u32, pub preroll: usize, pub copies: usize, pub events_kept: usize, pub minute: u32, pub sets: u32, pub events: u32, recent: VecDeque<Vec<String>>, kept: VecDeque<(String, Vec<String>)>, pending: VecDeque<(String, String)>, trigger_dir: Option<String>, pub written: Written }

impl Default for Sentry {
    /// `oxidrive_refusals.rs`'s defaults (`REFUSAL_*`).
    fn default() -> Self {
        Sentry { set_ns: 20 * S, clip: 8 << 20, ring: 6, event_every: 4, preroll: 2, copies: 4, events_kept: 6, minute: 0, sets: 0, events: 0, recent: VecDeque::new(), kept: VecDeque::new(), pending: VecDeque::new(), trigger_dir: None, written: Written::default() }
    }
}

impl Sentry {
    /// Free space the recorder needs at most, with a margin: the ring and one set more, the kept
    /// events and one more.
    pub fn needs(&self) -> u64 { let set = (4 * self.clip) as u64; (self.ring as u64 + 2) * set + (self.events_kept as u64 + 1) * ((self.preroll as u64 + 1) * set + (64 << 10)) + (64 << 20) }

    pub fn step<D: Drive, F: CarFs>(&mut self, car: &mut Car<D, F>) -> Result<Recorded, String> {
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
            self.written.record(car, small.iter().map(|s| s.0.clone()).collect(), &r);
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
        self.written.record(car, clips.iter().map(|c| c.0.clone()).collect(), &r);
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

/// A Tesla-like recorder, scaled down: a set of four camera clips every `set_ns` in RecentClips,
/// a ring of `ring` sets with the oldest deleted (on a full card the next set takes its
/// clusters), and every `event_every` sets an event that copies the last `event_sets` sets into
/// SentryClips or SavedClips at the bus's speed; past `events_kept` events the oldest event's
/// clips are deleted, as the car makes room.
#[derive(Clone, Debug)]
pub struct Tesla { pub set_ns: u64, pub clip: usize, pub ring: usize, pub event_every: u32, pub event_sets: usize, pub events_kept: usize, pub minute: u32, pub sets: u32, pub events: u32, recent: VecDeque<Vec<String>>, kept: VecDeque<Vec<String>>, pub written: Written }

impl Tesla {
    /// `oxidrive_hazards.rs`'s recorder from `minute`.
    pub fn new(minute: u32) -> Self { Tesla { set_ns: 6 * S, clip: 384 << 10, ring: 12, event_every: 5, event_sets: 2, events_kept: 8, minute, sets: 0, events: 0, recent: Default::default(), kept: Default::default(), written: Written::default() } }

    /// Take up the ring a previous boot left in RecentClips, oldest first, as the car's rotation
    /// does when it lists the folder.
    pub fn resume<D: Drive, F: CarFs>(&mut self, car: &mut Car<D, F>) {
        let mut fs = car.fs.clone().unwrap();
        let mut sets: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for e in fs.list(&mut car.disk, "TeslaCam/RecentClips").unwrap_or_default() {
            sets.entry(e.name.chars().take(19).collect()).or_default().push(format!("TeslaCam/RecentClips/{}", e.name));
        }
        self.recent = sets.into_values().collect();
        let mut events = Vec::new();
        for folder in ["SentryClips", "SavedClips"] {
            for ev in fs.list(&mut car.disk, &format!("TeslaCam/{folder}")).unwrap_or_default() {
                let dir = format!("TeslaCam/{folder}/{}", ev.name);
                let files: Vec<String> = fs.list(&mut car.disk, &dir).unwrap_or_default().into_iter().map(|e| format!("{dir}/{}", e.name)).collect();
                if !files.is_empty() { events.push((ev.name.clone(), files)); }
            }
        }
        events.sort();
        self.kept = events.into_iter().map(|e| e.1).collect();
    }

    /// One set; an event after every `event_every`. Stops at the first set the medium cut.
    pub fn step<D: Drive, F: CarFs>(&mut self, car: &mut Car<D, F>) -> Result<Recorded, String> {
        let m = self.minute;
        self.minute += 1;
        self.sets += 1;
        let paths: Vec<String> = CAMS.iter().map(|c| format!("TeslaCam/RecentClips/{}-{c}.mp4", stamp(m))).collect();
        let datas: Vec<Vec<u8>> = paths.iter().map(|p| mp4_bytes(p, self.clip)).collect();
        let clips: Vec<(&str, &[u8])> = paths.iter().map(|p| p.as_str()).zip(datas.iter().map(|d| d.as_slice())).collect();
        let chunks = self.clip.div_ceil(car.cfg.chunk) as u64;
        let r = car.record_set(&clips, self.set_ns / chunks.max(1))?;
        self.written.record(car, paths.clone(), &r);
        self.recent.push_back(paths);
        if !matches!(r, Recorded::Closed { .. }) { return Ok(r); }
        while self.recent.len() > self.ring { for p in self.recent.pop_front().unwrap() { car.delete(&p)?; } }
        if self.sets.is_multiple_of(self.event_every) {
            self.events += 1;
            let folder = if self.events % 2 == 1 { "SentryClips" } else { "SavedClips" };
            let dir = format!("TeslaCam/{folder}/{}", stamp(m));
            let src: Vec<String> = self.recent.iter().rev().take(self.event_sets).flatten().cloned().collect();
            let copies: Vec<(String, Vec<u8>)> = src.iter().map(|p| (format!("{dir}/{}", p.rsplit('/').next().unwrap()), car.files[p].data.clone())).collect();
            let clips: Vec<(&str, &[u8])> = copies.iter().map(|(p, d)| (p.as_str(), d.as_slice())).collect();
            let r = car.record_set(&clips, 0)?;
            self.written.record(car, copies.iter().map(|c| c.0.clone()).collect(), &r);
            self.kept.push_back(copies.into_iter().map(|c| c.0).collect());
            while self.kept.len() > self.events_kept { for p in self.kept.pop_front().unwrap() { car.delete(&p)?; } }
            if !matches!(r, Recorded::Closed { .. }) { return Ok(r); }
        }
        Ok(r)
    }
}

/// A filesystem a card can be filled on, as a car keeps a card full.
pub trait Fill: CarFs {
    /// Fill all of the volume but `leave` bytes with one file, FILLER.BIN, in the root; its data
    /// left as the medium holds it.
    fn fill(dev: &mut dyn BlockDevice, leave: u64) -> Result<(), String>;
}

impl Fill for Fat32 {
    /// Its chain written straight into both FATs, the entry and FSInfo after, as
    /// `oxidrive_refusals.rs`'s `fill_volume` does.
    fn fill(f: &mut dyn BlockDevice, leave: u64) -> Result<(), String> {
        let mut fs = Fat32::mount(f)?;
        let cb = fs.cluster_bytes() as u64;
        let n = fs.clusters as u64 - 1 - leave / cb;
        let fat = fs.volume_lba + fs.reserved as u64;
        let last = 3 + n - 1;
        for sec in (3 / 128)..=(last / 128) {
            let mut s = [0u8; 512];
            f.read_blocks(fat + sec, &mut s)?;
            for c in (sec * 128).max(3)..((sec + 1) * 128).min(last + 1) {
                let v: u32 = if c == last { 0x0fff_ffff } else { c as u32 + 1 };
                s[(c % 128) as usize * 4..(c % 128) as usize * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
            for copy in 0..fs.fats as u64 { f.write_blocks(fat + copy * fs.fat_sectors as u64 + sec, &s)?; }
        }
        drop(fs.create(f, "FILLER.BIN")?);
        let root = fs.data_lba;
        let mut s = [0u8; 512];
        f.read_blocks(root, &mut s)?;
        let slot = s.chunks(32).position(|e| &e[..11] == b"FILLER  BIN").ok_or("no FILLER.BIN entry")?;
        let e = &mut s[slot * 32..slot * 32 + 32];
        e[20..22].copy_from_slice(&0u16.to_le_bytes());
        e[26..28].copy_from_slice(&3u16.to_le_bytes());
        e[28..32].copy_from_slice(&((n * cb).min(u32::MAX as u64) as u32).to_le_bytes());
        f.write_blocks(root, &s)?;
        let mut info = [0u8; 512];
        f.read_blocks(fs.volume_lba + fs.fsinfo as u64, &mut info)?;
        let free = fs.clusters - 1 - n as u32;
        info[488..492].copy_from_slice(&free.to_le_bytes());
        info[492..496].copy_from_slice(&(last as u32).to_le_bytes());
        f.write_blocks(fs.volume_lba + fs.fsinfo as u64, &info)
    }
}

impl Fill for ExFat {
    /// NoFatChain over the free clusters, its bits and entry written straight (`ExFat::fill`).
    fn fill(f: &mut dyn BlockDevice, leave: u64) -> Result<(), String> { ExFat::mount(f)?.fill(f, "FILLER.BIN", leave).map(|_| ()) }
}
