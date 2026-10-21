//! The car's block layer between its FAT32 driver (`fat32::Fat32`) and the drive: what Linux's
//! buffer and page caches do there. Two behaviours, each optional:
//!
//! - **Keeping metadata** (`keep_meta`): the boot sector, FSInfo, the FAT and the directory sectors
//!   the car has read or written are answered from memory until `invalidate`, as the buffer cache
//!   answers them until the medium is seen to change. A device that changes them underneath (an
//!   archive window the car never saw) is not noticed.
//! - **Writeback** (`writeback`): writes stay in the cache and reach the drive later, the data of
//!   one write in one batch, each metadata page on its own schedule and usually held back, with the
//!   parameters of the firmware's writeback model (`WritebackCfg`). Reads see the car's own
//!   unwritten pages, as a page cache does. A page written again before it lands keeps its
//!   schedule and carries the newest bytes.
//!
//! Plain, it passes everything through, so a car without either reads and writes as before.
use super::fat32::BlockDevice;
use std::collections::{BTreeMap, HashMap};

pub const SECTOR: usize = 512;
/// Linux reclaims dirty pages of 4 KiB: sectors in one page land together.
pub const PAGE_SECTORS: u64 = 8;

/// When writeback lands: `WritebackCfg` of the firmware's writeback simulator (teslausb-p4
/// testkit/src/sim.rs), the same fields and defaults in nanoseconds. That file gives their
/// provenance: the flusher interval measured on a Linux host (lag follows
/// dirty_writeback_centisecs, 5 s at its default of 500), the data-first share measured on the car
/// (0.96), the metadata lag one dirty_expire_centisecs (30 s, not measured).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WritebackCfg {
    /// earliest a dirty page reaches the drive
    pub min_delay_ns: u64,
    /// latest, without the metadata lag: one flusher wake
    pub max_delay_ns: u64,
    /// pages may land out of the order they were written; without it they land in order
    pub reorder: bool,
    pub seed: u64,
    /// the share of metadata pages held back past the data they describe
    pub data_first_bias: f64,
    /// how far a held-back metadata page trails
    pub metadata_lag_ns: u64,
}

impl Default for WritebackCfg {
    fn default() -> Self {
        WritebackCfg { min_delay_ns: 50_000_000, max_delay_ns: 5_000_000_000, reorder: true, seed: 0x2545_f491_4f6c_dd1d, data_first_bias: 0.96, metadata_lag_ns: 30_000_000_000 }
    }
}

impl WritebackCfg {
    /// A host whose flusher wakes every `interval_ns`.
    pub fn for_flusher_interval(interval_ns: u64) -> Self { WritebackCfg { max_delay_ns: interval_ns.max(1), ..Self::default() } }
    /// Everything lands at the next flush, in the order written.
    pub fn immediate() -> Self { WritebackCfg { min_delay_ns: 0, max_delay_ns: 0, reorder: false, seed: 1, data_first_bias: 0.0, metadata_lag_ns: 0 } }
}

/// A write as it reached the drive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Landing {
    pub at_ns: u64,
    pub lba: u64,
    pub count: u32,
    pub meta: bool,
    /// when the oldest page in it was dirtied
    pub dirtied_ns: u64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CacheStats {
    pub meta_hits: u64,
    pub meta_misses: u64,
    pub pages_dirtied: u64,
    /// metadata pages given the lag
    pub held_back: u64,
    pub pages_landed: u64,
    /// dirty pages dropped (a power cut, a medium gone)
    pub pages_lost: u64,
    pub writes: u64,
    pub write_errors: u64,
}

struct Page { data: Box<[u8; SECTOR * PAGE_SECTORS as usize]>, mask: u8, due: u64, seq: u64, meta: bool, dirtied: u64 }

pub struct PageCache<D> {
    pub dev: D,
    pub keep_meta: bool,
    pub writeback: Option<WritebackCfg>,
    meta: HashMap<u64, Box<[u8; SECTOR]>>,
    dirty: BTreeMap<u64, Page>,
    seq: u64,
    rng: u64,
    pub stats: CacheStats,
    /// every write that reached the drive, when set, and the bytes of the metadata ones
    pub record: bool,
    pub landings: Vec<Landing>,
    pub meta_landed: Vec<(u64, u64, Vec<u8>)>,
}

impl<D: BlockDevice> PageCache<D> {
    /// A cache that passes everything through.
    pub fn new(dev: D) -> Self {
        PageCache { dev, keep_meta: false, writeback: None, meta: HashMap::new(), dirty: BTreeMap::new(), seq: 0, rng: 1, stats: CacheStats::default(), record: false, landings: Vec::new(), meta_landed: Vec::new() }
    }
    /// Metadata kept until `invalidate`, writes through.
    pub fn keeping_meta(dev: D) -> Self { PageCache { keep_meta: true, ..Self::new(dev) } }
    /// Metadata kept, writes landing as `cfg` says.
    pub fn writeback(dev: D, cfg: WritebackCfg) -> Self { PageCache { keep_meta: true, writeback: Some(cfg), rng: cfg.seed.max(1), ..Self::new(dev) } }

    fn caching(&self) -> bool { self.keep_meta || self.writeback.is_some() }

    fn next_rand(&mut self) -> u64 {
        let mut x = self.rng;
        x ^= x << 13; x ^= x >> 7; x ^= x << 17;
        self.rng = x;
        x
    }

    /// The delay a new dirty page gets: a flusher wake drawn from [min, max], and the metadata lag
    /// on top for a held-back metadata page (testkit/src/sim.rs `Sim::car`).
    fn delay(&mut self, meta: bool) -> u64 {
        let Some(c) = self.writeback else { return 0 };
        let mut d = if c.max_delay_ns > c.min_delay_ns { c.min_delay_ns + self.next_rand() % (c.max_delay_ns - c.min_delay_ns + 1) } else { c.min_delay_ns };
        if !c.reorder { d = c.min_delay_ns; }
        if meta && (self.next_rand() % 1_000_000) < (c.data_first_bias * 1e6) as u64 {
            d += c.metadata_lag_ns;
            self.stats.held_back += 1;
        }
        d
    }

    /// Dirty the sectors of `data` at `lba`. Data written in one call shares one delay; each
    /// metadata page draws its own.
    fn dirty_range(&mut self, lba: u64, data: &[u8], meta: bool) {
        let now = self.dev.now_ns();
        let batch = if meta { 0 } else { self.delay(false) };
        for (i, s) in data.chunks(SECTOR).enumerate() {
            let at = lba + i as u64;
            let (page, slot) = (at / PAGE_SECTORS, (at % PAGE_SECTORS) as usize);
            if !self.dirty.contains_key(&page) {
                let due = now + if meta { self.delay(true) } else { batch };
                self.seq += 1;
                self.stats.pages_dirtied += 1;
                self.dirty.insert(page, Page { data: Box::new([0; SECTOR * PAGE_SECTORS as usize]), mask: 0, due, seq: self.seq, meta, dirtied: now });
            }
            let p = self.dirty.get_mut(&page).unwrap();
            p.data[slot * SECTOR..slot * SECTOR + s.len()].copy_from_slice(s);
            p.mask |= 1 << slot;
            p.meta |= meta;
        }
    }

    fn dirty_sector(&self, lba: u64) -> Option<&[u8]> {
        let p = self.dirty.get(&(lba / PAGE_SECTORS))?;
        let slot = (lba % PAGE_SECTORS) as usize;
        (p.mask & 1 << slot != 0).then(|| &p.data[slot * SECTOR..slot * SECTOR + SECTOR])
    }

    /// When the next dirty page is due.
    pub fn next_due(&self) -> Option<u64> { self.dirty.values().map(|p| p.due).min() }
    pub fn dirty_pages(&self) -> usize { self.dirty.len() }

    /// Land `pages`, oldest due first, contiguous sectors in one write.
    fn land(&mut self, mut pages: Vec<u64>) -> Result<usize, String> {
        pages.sort_by_key(|p| { let x = &self.dirty[p]; (x.due, x.seq) });
        let mut runs: Vec<(u64, Vec<u8>, bool, u64)> = Vec::new();
        for page in &pages {
            let p = self.dirty.remove(page).unwrap();
            for slot in 0..PAGE_SECTORS as usize {
                if p.mask & 1 << slot == 0 { continue; }
                let lba = page * PAGE_SECTORS + slot as u64;
                let s = &p.data[slot * SECTOR..slot * SECTOR + SECTOR];
                match runs.last_mut() {
                    Some(r) if r.0 + (r.1.len() / SECTOR) as u64 == lba && r.2 == p.meta => { r.1.extend_from_slice(s); r.3 = r.3.min(p.dirtied); }
                    _ => runs.push((lba, s.to_vec(), p.meta, p.dirtied)),
                }
            }
            self.stats.pages_landed += 1;
        }
        let mut first_err = None;
        for (lba, data, meta, dirtied) in runs {
            self.stats.writes += 1;
            match self.dev.write_blocks(lba, &data) {
                Ok(()) => if self.record {
                    let at = self.dev.now_ns();
                    self.landings.push(Landing { at_ns: at, lba, count: (data.len() / SECTOR) as u32, meta, dirtied_ns: dirtied });
                    if meta { self.meta_landed.push((at, lba, data)); }
                },
                Err(e) => { self.stats.write_errors += 1; first_err.get_or_insert(e); }
            }
        }
        match first_err { Some(e) => Err(e), None => Ok(pages.len()) }
    }

    /// Land the pages that are due now.
    pub fn flush_due(&mut self) -> Result<usize, String> {
        if self.dirty.is_empty() { return Ok(0); }
        let now = self.dev.now_ns();
        let due: Vec<u64> = self.dirty.iter().filter(|(_, p)| p.due <= now).map(|(&k, _)| k).collect();
        if due.is_empty() { return Ok(0); }
        self.land(due)
    }

    /// Land every dirty page now, as sync(2) or an unmount does.
    pub fn sync(&mut self) -> Result<usize, String> {
        let all: Vec<u64> = self.dirty.keys().copied().collect();
        self.land(all)
    }

    /// Drop every dirty page: power gone with the car, or writes that can no longer reach the
    /// medium. Returns how many were lost.
    pub fn drop_dirty(&mut self) -> usize {
        let n = self.dirty.len();
        self.stats.pages_lost += n as u64;
        self.dirty.clear();
        n
    }

    /// Forget the metadata kept: the next reads go to the drive.
    pub fn invalidate(&mut self) { self.meta.clear(); }

    fn write_through(&mut self, lba: u64, data: &[u8], meta: bool) -> Result<(), String> {
        let t0 = self.dev.now_ns();
        let r = if meta { self.dev.write_meta(lba, data) } else { self.dev.write_blocks(lba, data) };
        self.stats.writes += 1;
        match &r {
            Ok(()) if self.record => {
                let at = self.dev.now_ns();
                self.landings.push(Landing { at_ns: at, lba, count: (data.len() / SECTOR) as u32, meta, dirtied_ns: t0 });
                if meta { self.meta_landed.push((at, lba, data.to_vec())); }
            }
            Ok(()) => {}
            Err(_) => self.stats.write_errors += 1,
        }
        r
    }
}

impl<D: BlockDevice> BlockDevice for PageCache<D> {
    fn block_size(&self) -> u32 { self.dev.block_size() }
    fn blocks(&self) -> u64 { self.dev.blocks() }

    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String> {
        self.flush_due()?;
        if self.dirty.is_empty() { return self.dev.read_blocks(lba, buf); }
        let n = buf.len() / SECTOR;
        let mut i = 0;
        while i < n {
            if let Some(s) = self.dirty_sector(lba + i as u64) { buf[i * SECTOR..(i + 1) * SECTOR].copy_from_slice(s); i += 1; continue; }
            let mut j = i + 1;
            while j < n && self.dirty_sector(lba + j as u64).is_none() { j += 1; }
            self.dev.read_blocks(lba + i as u64, &mut buf[i * SECTOR..j * SECTOR])?;
            i = j;
        }
        Ok(())
    }

    fn write_blocks(&mut self, lba: u64, data: &[u8]) -> Result<(), String> {
        self.flush_due()?;
        if self.caching() {
            for (i, s) in data.chunks(SECTOR).enumerate() {
                if let Some(m) = self.meta.get_mut(&(lba + i as u64)) { m.copy_from_slice(s); }
            }
        }
        if self.writeback.is_some() { self.dirty_range(lba, data, false); Ok(()) } else { self.write_through(lba, data, false) }
    }

    fn read_meta(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String> {
        self.flush_due()?;
        if !self.caching() { return self.dev.read_meta(lba, buf); }
        let n = buf.len() / SECTOR;
        let mut i = 0;
        while i < n {
            let at = lba + i as u64;
            if let Some(s) = self.dirty_sector(at) { buf[i * SECTOR..(i + 1) * SECTOR].copy_from_slice(s); i += 1; continue; }
            if let Some(s) = self.meta.get(&at) { buf[i * SECTOR..(i + 1) * SECTOR].copy_from_slice(&s[..]); self.stats.meta_hits += 1; i += 1; continue; }
            let mut j = i + 1;
            while j < n && self.dirty_sector(lba + j as u64).is_none() && !self.meta.contains_key(&(lba + j as u64)) { j += 1; }
            self.dev.read_meta(at, &mut buf[i * SECTOR..j * SECTOR])?;
            self.stats.meta_misses += (j - i) as u64;
            for k in i..j { self.meta.insert(lba + k as u64, Box::new(buf[k * SECTOR..(k + 1) * SECTOR].try_into().unwrap())); }
            i = j;
        }
        Ok(())
    }

    fn write_meta(&mut self, lba: u64, data: &[u8]) -> Result<(), String> {
        self.flush_due()?;
        if self.caching() {
            for (i, s) in data.chunks(SECTOR).enumerate() { self.meta.insert(lba + i as u64, Box::new(s.try_into().map_err(|_| "a partial sector")?)); }
        }
        if self.writeback.is_some() { self.dirty_range(lba, data, true); Ok(()) } else { self.write_through(lba, data, true) }
    }

    /// Time passes, the flusher landing pages as they fall due.
    fn pause_ns(&mut self, ns: u64) -> Result<(), String> {
        if self.writeback.is_none() { return self.dev.pause_ns(ns); }
        let end = self.dev.now_ns() + ns;
        loop {
            let now = self.dev.now_ns();
            if now >= end { return Ok(()); }
            let t = self.next_due().map_or(end, |d| d.clamp(now, end));
            if t > now { self.dev.pause_ns(t - now)?; }
            self.flush_due()?;
        }
    }

    fn now_ns(&mut self) -> u64 { self.dev.now_ns() }
}

/// Data before its FAT entry, the drive trace's statistic (the shared-reading plan's 98.2%): for
/// each data cluster, its first data landing against the first landing of the FAT sector holding
/// its entry (the first copy) made after the data was written. Counts of clusters whose data
/// landed first, and whose entry did.
pub fn data_first(landings: &[Landing], fs: &super::fat32::Fat32) -> (u64, u64) {
    let mut first: HashMap<u32, (u64, u64)> = HashMap::new();
    let mut fat: HashMap<u64, Vec<u64>> = HashMap::new();
    for l in landings {
        if !l.meta {
            for lba in l.lba..l.lba + l.count as u64 {
                if let Some(c) = fs.cluster_of_lba(lba) { first.entry(c).or_insert((l.dirtied_ns, l.at_ns)); }
            }
        } else {
            for lba in l.lba..l.lba + l.count as u64 {
                if let Some(sector) = fs.fat_sector_of_lba(lba) { fat.entry(sector).or_default().push(l.at_ns); }
            }
        }
    }
    let (mut data, mut entry) = (0, 0);
    for (c, (dirtied, at)) in first {
        let Some(t) = fat.get(&(c as u64 / 128)).and_then(|v| v.iter().copied().filter(|&t| t >= dirtied).min()) else { continue };
        if at <= t { data += 1 } else { entry += 1 }
    }
    (data, entry)
}
