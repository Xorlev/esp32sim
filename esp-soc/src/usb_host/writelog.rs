//! The car's writes as a write trace records them, for checking a trace analyzer against: each
//! write the car landed (`pagecache::Landing`), split into the regions of the volume in the
//! vocabulary of oxidrive's `oxi trace --writes` (its repository's docs/trace-capture.md), the
//! boot sector's VolumeFlags with what kind of change each write made, and each file's facts
//! from the car's own record (`exfat::Truth`). Also the counts that set FAT32 and exFAT side by
//! side for one workload: FAT and bitmap sectors by the change they carry, FAT sectors that files
//! written together share, boot-sector writes.
use super::exfat::{FileTruth, Truth, VOL_DIRTY};
use super::fat32::Fat32;
use super::pagecache::Landing;
use std::collections::{BTreeMap, BTreeSet, HashMap};

const SECTOR: usize = 512;

/// What a sector holds, named as `oxi trace` names it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Region { Mbr, PreVolume, BootSector, BootRegion, BackupBoot, FsInfo, Reserved, Fat, Fat2, Bitmap, UpCase, RootDir, Dir, Data, Unknown, Outside }

impl Region {
    pub fn name(self) -> &'static str {
        match self {
            Region::Mbr => "mbr", Region::PreVolume => "pre-volume", Region::BootSector => "boot-sector", Region::BootRegion => "boot-region",
            Region::BackupBoot => "backup-boot", Region::FsInfo => "fsinfo", Region::Reserved => "reserved", Region::Fat => "fat", Region::Fat2 => "fat2",
            Region::Bitmap => "bitmap", Region::UpCase => "upcase", Region::RootDir => "root-dir", Region::Dir => "dir", Region::Data => "data",
            Region::Unknown => "unknown", Region::Outside => "outside",
        }
    }
    pub fn parse(s: &str) -> Option<Region> {
        const ALL: [Region; 16] = [Region::Mbr, Region::PreVolume, Region::BootSector, Region::BootRegion, Region::BackupBoot, Region::FsInfo, Region::Reserved, Region::Fat,
                                   Region::Fat2, Region::Bitmap, Region::UpCase, Region::RootDir, Region::Dir, Region::Data, Region::Unknown, Region::Outside];
        ALL.into_iter().find(|r| r.name() == s)
    }
}

/// A FAT32 volume's regions; a directory cluster is one the car wrote as metadata (`meta`).
pub fn fat32_region(fs: &Fat32, lba: u64, meta: bool) -> Region {
    let v = fs.volume_lba;
    if lba == 0 && v > 0 { return Region::Mbr; }
    if lba < v { return Region::PreVolume; }
    let rel = lba - v;
    if rel == 0 { return Region::BootSector; }
    if rel == fs.fsinfo as u64 { return Region::FsInfo; }
    if (6..9).contains(&rel) { return Region::BackupBoot; }
    if rel < fs.reserved as u64 { return Region::Reserved; }
    if fs.fat_sector_of_lba(lba).is_some() { return Region::Fat; }
    if lba < fs.data_lba { return Region::Fat2; }
    match fs.cluster_of_lba(lba) {
        Some(c) if c == fs.root_cluster => Region::RootDir,
        Some(_) if meta => Region::Dir,
        Some(_) => Region::Data,
        None => Region::Outside,
    }
}

/// What a write that covers the boot sector changed in it: only VolumeFlags and PercentInUse,
/// which the boot checksum leaves out, or more.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootClass { State, Structural }

impl BootClass { pub fn name(self) -> &'static str { match self { BootClass::State => "state", BootClass::Structural => "structural" } } }

/// One write the car landed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WriteTruth {
    pub at_ns: u64,
    pub lba: u64,
    pub count: u32,
    /// sectors per region, in the order the write meets them
    pub regions: Vec<(Region, u32)>,
    /// a write covering the boot sector: VolumeFlags as it left them, the kind of change, and
    /// whether it was written at once (`sync_meta`) rather than by writeback
    pub boot: Option<(u16, BootClass, bool)>,
}

impl WriteTruth {
    /// `fat:1,data:63`, as `oxi trace --writes` prints a write's regions.
    pub fn regions_text(&self) -> String { self.regions.iter().map(|(r, n)| format!("{}:{n}", r.name())).collect::<Vec<_>>().join(",") }
}

fn split(lba: u64, count: u32, region: impl Fn(u64) -> Region) -> Vec<(Region, u32)> {
    let mut out: Vec<(Region, u32)> = Vec::new();
    for s in lba..lba + count as u64 {
        let r = region(s);
        match out.iter_mut().find(|x| x.0 == r) { Some(x) => x.1 += 1, None => out.push((r, 1)) }
    }
    out
}

/// The writes of an exFAT car (`Car::truth`, `PageCache::landings` and `meta_landed` with
/// `record` set), classified. `boot0` is the boot sector as the car's mount found it.
pub fn exfat_writes(truth: &Truth, landings: &[Landing], meta: &[(u64, u64, Vec<u8>)], boot0: &[u8; SECTOR]) -> Vec<WriteTruth> {
    let Some(l) = truth.layout else { return Vec::new() };
    let syncs: BTreeSet<u64> = truth.flags.iter().filter(|f| f.2).map(|f| f.0).collect();
    let mut prev = *boot0;
    let mut mi = 0;
    let mut out = Vec::with_capacity(landings.len());
    for w in landings {
        let bytes = if w.meta { let m = &meta[mi]; mi += 1; debug_assert_eq!((m.0, m.1), (w.at_ns, w.lba)); Some(&m.2) } else { None };
        let regions = split(w.lba, w.count, |s| l.region(s, &truth.root_clusters, &truth.dir_clusters));
        let boot = (w.lba..w.lba + w.count as u64).contains(&l.volume_lba).then(|| {
            let at = (l.volume_lba - w.lba) as usize * SECTOR;
            let b: [u8; SECTOR] = bytes.map_or([0u8; SECTOR], |d| d[at..at + SECTOR].try_into().unwrap());
            let state = (0..SECTOR).all(|i| matches!(i, 106 | 107 | 112) || b[i] == prev[i]);
            prev = b;
            (u16::from_le_bytes([b[106], b[107]]), if state { BootClass::State } else { BootClass::Structural }, syncs.contains(&w.dirtied_ns) && w.dirtied_ns == w.due_ns)
        });
        out.push(WriteTruth { at_ns: w.at_ns, lba: w.lba, count: w.count, regions, boot });
    }
    out
}

/// One file as the car made it and as its writes landed.
#[derive(Clone, Debug)]
pub struct FileFacts {
    pub path: String,
    pub truth: FileTruth,
    /// the first data landing on any of its clusters after it took them, and when every one of
    /// its clusters had had one
    pub data_first_ns: Option<u64>,
    pub data_all_ns: Option<u64>,
    /// the first landing of its entry set, and the first that held its final cluster, flags and
    /// size, the set still live
    pub set_first_ns: Option<u64>,
    pub set_final_ns: Option<u64>,
}

/// Every file of an exFAT car's record, with when its data and its entry set landed.
pub fn exfat_files(truth: &Truth, landings: &[Landing], meta: &[(u64, u64, Vec<u8>)]) -> Vec<FileFacts> {
    let Some(l) = truth.layout else { return Vec::new() };

    // data landings by cluster: (landed, dirtied)
    let mut data: HashMap<u32, Vec<(u64, u64)>> = HashMap::new();
    for w in landings.iter().filter(|w| !w.meta) {
        let (first, last) = (l.cluster_of_lba(w.lba), l.cluster_of_lba(w.lba + w.count as u64 - 1));
        if let (Some(a), Some(b)) = (first, last) { for c in a..=b { data.entry(c).or_default().push((w.at_ns, w.dirtied_ns)); } }
    }
    // metadata landings by sector: (landed, the sector's bytes)
    let mut sectors: HashMap<u64, Vec<(u64, &[u8])>> = HashMap::new();
    let wanted: BTreeSet<u64> = truth.files.values().chain(truth.past.iter().map(|p| &p.1)).flat_map(|f| f.set_lbas.iter().copied()).collect();
    for (at, lba, d) in meta {
        for (i, s) in d.chunks(SECTOR).enumerate() {
            let x = lba + i as u64;
            if wanted.contains(&x) { sectors.entry(x).or_default().push((*at, s)); }
        }
    }
    let mut out = Vec::new();
    for (path, f) in truth.files.iter().chain(truth.past.iter().map(|(p, f)| (p, f))) {
        let end = f.deleted_ns.unwrap_or(u64::MAX);
        let mut first = None;
        let mut all = Some(0u64);
        for &(c, taken) in &f.clusters {
            let t = data.get(&c).and_then(|v| v.iter().filter(|x| x.1 >= taken && x.0 <= end).map(|x| x.0).min());
            first = match (first, t) { (Some(a), Some(b)) => Some(std::cmp::min(a, b)), (a, b) => a.or(b) };
            all = match (all, t) { (Some(a), Some(b)) => Some(a.max(b)), _ => None };
        }
        if f.clusters.is_empty() || f.dir { all = None; }
        // the set's sectors as they landed, in landing order
        let mut events: Vec<(u64, usize, &[u8])> = f.set_lbas.iter().enumerate().flat_map(|(k, lba)| sectors.get(lba).into_iter().flatten().filter(|x| x.0 >= f.created_ns && x.0 <= end).map(move |x| (x.0, k, x.1))).collect();
        events.sort_by_key(|e| e.0);
        let set_first = events.first().map(|e| e.0);
        let mut now: Vec<Option<&[u8]>> = vec![None; f.set_lbas.len()];
        let mut set_final = None;
        let want_first = f.clusters.first().map_or(0, |c| c.0);
        let want_flags = if f.chained.is_some() { 0x01 } else { 0x03 };
        for (at, k, s) in events {
            now[k] = Some(s);
            if f.closed_ns.is_none() && !f.dir { continue; }
            if now.iter().any(|x| x.is_none()) { continue; }
            let set: Vec<u8> = now.iter().flat_map(|x| x.unwrap().iter().copied()).collect();
            let (file, st) = (&set[f.set_offset..f.set_offset + 32], &set[f.set_offset + 32..f.set_offset + 64]);
            let size = u64::from_le_bytes(st[24..32].try_into().unwrap());
            let first_c = u32::from_le_bytes(st[20..24].try_into().unwrap());
            let size_ok = f.dir || size == f.size;
            if file[0] == 0x85 && st[0] == 0xc0 && first_c == want_first && st[1] == want_flags && size_ok { set_final = Some(at); break; }
        }
        out.push(FileFacts { path: path.clone(), truth: f.clone(), data_first_ns: first, data_all_ns: all, set_first_ns: set_first, set_final_ns: set_final });
    }
    out
}

/// Sector writes to one region, by what they changed against the medium before them: FAT
/// entries linked or ended (`chain`) or zeroed (`free`); bitmap bits set (`chain`) or cleared
/// (`free`); both; or nothing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Changes { pub sectors: u64, pub chain: u64, pub free: u64, pub both: u64, pub unchanged: u64 }

/// `Changes` for the FAT (both copies) and the bitmap, over the metadata the car landed, against
/// `base`, the medium's sectors before the first of them.
pub fn meta_changes(meta: &[(u64, u64, Vec<u8>)], region: impl Fn(u64) -> Region, mut base: impl FnMut(u64) -> [u8; SECTOR]) -> BTreeMap<Region, Changes> {
    let mut prev: HashMap<u64, [u8; SECTOR]> = HashMap::new();
    let mut out: BTreeMap<Region, Changes> = BTreeMap::new();
    for (_, lba, d) in meta {
        for (i, s) in d.chunks(SECTOR).enumerate() {
            let x = lba + i as u64;
            let r = region(x);
            if !matches!(r, Region::Fat | Region::Fat2 | Region::Bitmap) { continue; }
            let old = *prev.entry(x).or_insert_with(|| base(x));
            let (mut chain, mut free) = (false, false);
            if r == Region::Bitmap {
                for (a, b) in old.iter().zip(s) { chain |= !a & b != 0; free |= a & !b != 0; }
            } else {
                for (a, b) in old.chunks(4).zip(s.chunks(4)) {
                    if a != b { if b.iter().all(|&v| v == 0) { free = true } else { chain = true } }
                }
            }
            let c = out.entry(r).or_default();
            c.sectors += 1;
            match (chain, free) { (true, true) => c.both += 1, (true, false) => c.chain += 1, (false, true) => c.free += 1, _ => c.unchanged += 1 }
            prev.insert(x, s.try_into().unwrap());
        }
    }
    out
}

/// FAT sectors that files written together hold live chain entries in: over each set of files a
/// recorder wrote at once, (FAT sectors holding entries of the set's files, those holding entries
/// of two files or more). `live` gives a file's clusters with a live FAT entry: all of them on
/// FAT32, none for a NoFatChain file on exFAT.
pub fn shared_fat_sectors(sets: &[(u64, Vec<String>)], live: impl Fn(&str) -> Vec<u32>) -> (u64, u64) {
    let (mut touched, mut shared) = (0, 0);
    for (_, paths) in sets {
        let mut by: HashMap<u32, BTreeSet<usize>> = HashMap::new();
        for (k, p) in paths.iter().enumerate() { for c in live(p) { by.entry(c / 128).or_default().insert(k); } }
        touched += by.len() as u64;
        shared += by.values().filter(|f| f.len() > 1).count() as u64;
    }
    (touched, shared)
}

/// The VolumeFlags of exFAT boot-sector writes: (written dirty, written clean, of them written at
/// once).
pub fn boot_writes(w: &[WriteTruth]) -> (u64, u64, u64) {
    let b: Vec<_> = w.iter().filter_map(|x| x.boot).collect();
    (b.iter().filter(|x| x.0 & VOL_DIRTY != 0).count() as u64, b.iter().filter(|x| x.0 & VOL_DIRTY == 0).count() as u64, b.iter().filter(|x| x.2).count() as u64)
}
