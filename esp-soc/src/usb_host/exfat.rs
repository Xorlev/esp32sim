//! exFAT on the host side of a block device, written as the car's Linux writes it: exfat-linux
//! 5.8-1arter97, the backport of Linux v5.8's fs/exfat that Tesla's buildroot builds as a module
//! (teslamotors/buildroot buildroot-2021.02, package/exfat-linux). Line numbers below are that
//! tag's; v5.8 differs only where noted. On-disk layout: Microsoft's exFAT File System
//! Specification.
//!
//! What the driver does that vfat does not:
//!
//! - **Allocation at write() time.** `exfat_write_begin` maps each page through `exfat_get_block`
//!   (`cont_write_begin`, no delayed allocation), which allocates one cluster as the file's pages
//!   cross into it (inode.c `exfat_map_cluster`, 104-262). The hint is the file's last cluster + 1
//!   (inode.c:171-172), or the volume's search pointer for an empty file (fatent.c:323-332); the
//!   search pointer becomes the last hint used (403). A file stays NoFatChain (contiguous, no FAT
//!   entries) while each new cluster is its last + 1. The first that is not converts the whole
//!   file to a FAT chain: its clusters so far are chained in the FAT (inode.c:203-210
//!   `exfat_chain_cont_cluster`), the new one linked after them, and its Stream Extension rewritten
//!   (227-241). Files written at the same time take turns and are FAT-chained.
//! - **Free clears allocation-bitmap bits only.** The FAT entries stay as they were
//!   (fatent.c:153-198 `exfat_free_cluster`), so a NoFatChain file can sit on clusters whose FAT
//!   entries still chain a deleted file.
//! - **VolumeFlags** (boot sector bytes 106-107) go dirty at the start of every change and clean
//!   at the end of create, unlink, mkdir and truncate (namei.c:563/566, 835/844, 874/877;
//!   file.c:107/218); allocation (inode.c:169) and inode writeback (inode.c:40) set them dirty.
//!   Nothing is written at mount. `exfat_set_vol_flags` (super.c:100-131) writes only on a change:
//!   dirty with `sync_dirty_buffer` (REQ_SYNC: an immediate WRITE(10) of the boot sector, without
//!   FUA) when the boot sector's buffer was clean, otherwise it marks the buffer dirty for writeback.
//!   The unmount sets them clean (super.c:42-55) and the device's last sync lands it. The boot
//!   checksum leaves VolumeFlags and PercentInUse out (misc.c:150-162); 5.8 never writes
//!   PercentInUse.
//! - **Directory entry sets**: a File entry, a Stream Extension and one File Name entry per 15
//!   characters (dir.c:327-337), so a 29-character Tesla name is four entries, 128 bytes, which can
//!   straddle a sector. The SetChecksum is recomputed whenever the set is written
//!   (dir.c:587-602). ValidDataLength is written equal to DataLength everywhere (inode.c:74-75,
//!   236-238; file.c:173-177). A new file's set is written with a FAT-chain flag, no cluster and
//!   size 0 (dir.c:434-480); its first allocation writes NoFatChain, the cluster and the size it has
//!   then (inode.c:227-238), which is 0 for the first page. A name entry is written up to and
//!   including its terminating 0, the rest left as the slot held it (dir.c:418-432; v5.8 zeroes
//!   it). A deleted entry keeps its bytes with the type's top bit cleared (dir.c:565-585).
//!
//! Linux's page cache applies as for vfat (`pagecache::PageCache`): data and metadata go through
//! `BlockDevice::write_blocks` and `write_meta`; the boot sector's synchronous writes through
//! `BlockDevice::sync_meta`. The allocation bitmap is read whole at the mount and kept, as the
//! driver pins its buffers (balloc.c:56-102); FAT and directory sectors are read as needed. Every
//! file's allocation, conversion, close and deletion goes to the ground truth (`Truth`), with the
//! VolumeFlags writes, for checking a trace of the car's writes against.
use super::fat32::{BlockDevice, DirEntry, FsckReport, FsckWrite};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, Mutex};

const SECTOR: usize = 512;
/// End of a chain, and the FAT entry of a cluster that ends one.
pub const EOF: u32 = 0xffff_ffff;
pub const VOL_CLEAN: u16 = 0x0000;
pub const VOL_DIRTY: u16 = 0x0002;
/// GeneralSecondaryFlags: AllocationPossible, and NoFatChain with it (exfat_raw.h:29-30).
pub const ALLOC_FAT_CHAIN: u8 = 0x01;
pub const ALLOC_NO_FAT_CHAIN: u8 = 0x03;
const T_BITMAP: u8 = 0x81;
const T_UPCASE: u8 = 0x82;
const T_VOLUME: u8 = 0x83;
const T_FILE: u8 = 0x85;
const T_STREAM: u8 = 0xc0;
const T_NAME: u8 = 0xc1;
pub const ATTR_SUBDIR: u16 = 0x10;
pub const ATTR_ARCHIVE: u16 = 0x20;
const NAME_UNITS: usize = 15;
/// A fixed time stamp, 2024-01-01 00:00 UTC, so the image is the same on every run: date, time,
/// centiseconds, and the UTC offset byte Linux writes (OffsetValid, offset 0; misc.c
/// `exfat_set_entry_time`).
const DATE: u16 = (2024 - 1980) << 9 | 1 << 5 | 1;
const TZ_UTC: u8 = 0x80;

fn get16(b: &[u8], at: usize) -> u16 { u16::from_le_bytes([b[at], b[at + 1]]) }
fn get32(b: &[u8], at: usize) -> u32 { u32::from_le_bytes(b[at..at + 4].try_into().unwrap()) }
fn get64(b: &[u8], at: usize) -> u64 { u64::from_le_bytes(b[at..at + 8].try_into().unwrap()) }
fn put16(b: &mut [u8], at: usize, v: u16) { b[at..at + 2].copy_from_slice(&v.to_le_bytes()); }
fn put32(b: &mut [u8], at: usize, v: u32) { b[at..at + 4].copy_from_slice(&v.to_le_bytes()); }
fn put64(b: &mut [u8], at: usize, v: u64) { b[at..at + 8].copy_from_slice(&v.to_le_bytes()); }

/// The 16-bit rotate-right-and-add checksum: an entry set's SetChecksum, which skips the
/// checksum's own bytes (2 and 3 of the first entry), and a name's NameHash (misc.c:136-148).
pub fn chksum16(data: &[u8], mut sum: u16, skip_own: bool) -> u16 {
    for (i, &b) in data.iter().enumerate() {
        if skip_own && (i == 2 || i == 3) { continue; }
        sum = sum.rotate_right(1).wrapping_add(b as u16);
    }
    sum
}

/// The 32-bit one: the boot region's checksum, which skips VolumeFlags and PercentInUse of the
/// boot sector (bytes 106, 107 and 112), and the up-case table's (misc.c:150-162).
pub fn chksum32(data: &[u8], mut sum: u32, boot: bool) -> u32 {
    for (i, &b) in data.iter().enumerate() {
        if boot && (i == 106 || i == 107 || i == 112) { continue; }
        sum = sum.rotate_right(1).wrapping_add(b as u32);
    }
    sum
}

/// The checksum of the entry set `entries` (32 bytes each, the File entry first).
pub fn set_checksum(entries: &[u8]) -> u16 {
    let mut sum = chksum16(&entries[..32], 0, true);
    for e in entries[32..].chunks(32) { sum = chksum16(e, sum, false); }
    sum
}

/// What the car's exFAT did, for checking a trace of its writes against: where each structure
/// is, every file's clusters in the order it took them and when, when it became a FAT chain, its
/// entry set's sectors, when it was closed and deleted, and every VolumeFlags write. A `Car` keeps
/// one across its mounts (`Car::truth`).
#[derive(Clone, Debug, Default)]
pub struct Truth {
    pub layout: Option<Layout>,
    /// by path; a path made again after its deletion moves the old record to `past`
    pub files: BTreeMap<String, FileTruth>,
    pub past: Vec<(String, FileTruth)>,
    /// clusters that hold a directory, and since when
    pub dir_clusters: BTreeMap<u32, u64>,
    /// VolumeFlags changes: (time, the new value, written at once (`sync_meta`) or left dirty)
    pub flags: Vec<(u64, u16, bool)>,
}

/// One file or directory as the car made it.
#[derive(Clone, Debug, Default)]
pub struct FileTruth {
    pub dir: bool,
    pub created_ns: u64,
    /// the sectors its entry set occupies, and how many entries it has
    pub set_lbas: Vec<u64>,
    pub entries: u32,
    /// its clusters in the order it took them, with the time
    pub clusters: Vec<(u32, u64)>,
    /// when it was converted to a FAT chain, and how many clusters it had then
    pub chained: Option<(u64, u32)>,
    pub closed_ns: Option<u64>,
    pub size: u64,
    pub deleted_ns: Option<u64>,
}

impl FileTruth {
    /// Runs of consecutive clusters.
    pub fn fragments(&self) -> usize { self.clusters.windows(2).filter(|w| w[1].0 != w[0].0 + 1).count() + usize::from(!self.clusters.is_empty()) }
    pub fn is_chained(&self) -> bool { self.chained.is_some() }
}

/// Shared between a car and the volume it has mounted, so it outlives remounts.
pub type TruthLog = Arc<Mutex<Truth>>;

/// Where the volume's structures are, in absolute sectors and clusters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Layout {
    pub volume_lba: u64,
    pub fat_lba: u64,
    pub fat_sectors: u32,
    pub heap_lba: u64,
    pub sectors_per_cluster: u32,
    /// ClusterCount: cluster numbers run from 2 to clusters + 1
    pub clusters: u32,
    pub root_cluster: u32,
    pub bitmap_cluster: u32,
    pub bitmap_bytes: u64,
    pub upcase_cluster: u32,
    pub upcase_bytes: u64,
}

/// What a sector of the volume holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Region { Mbr, Boot, Fat, Bitmap, UpCase, Dirent, Data, Other }

impl Layout {
    pub fn cluster_lba(&self, c: u32) -> u64 { self.heap_lba + (c as u64 - 2) * self.sectors_per_cluster as u64 }
    pub fn cluster_of_lba(&self, lba: u64) -> Option<u32> {
        (lba >= self.heap_lba).then(|| ((lba - self.heap_lba) / self.sectors_per_cluster as u64) as u32 + 2).filter(|&c| c < self.clusters + 2)
    }
    pub fn fat_sector_of(&self, c: u32) -> u64 { self.fat_lba + c as u64 / 128 }
    pub fn bitmap_sector_of(&self, c: u32) -> u64 { self.cluster_lba(self.bitmap_cluster) + (c as u64 - 2) / 4096 }
    /// The region of `lba`, with the clusters that hold directories.
    pub fn region(&self, lba: u64, dirs: &BTreeMap<u32, u64>) -> Region {
        if lba < self.volume_lba { return Region::Mbr; }
        if lba < self.volume_lba + 24 { return Region::Boot; }
        if (self.fat_lba..self.fat_lba + self.fat_sectors as u64).contains(&lba) { return Region::Fat; }
        let Some(c) = self.cluster_of_lba(lba) else { return Region::Other };
        let in_run = |first: u32, bytes: u64| c >= first && ((c - first) as u64) < bytes.div_ceil(self.sectors_per_cluster as u64 * SECTOR as u64);
        if in_run(self.bitmap_cluster, self.bitmap_bytes) { return Region::Bitmap; }
        if in_run(self.upcase_cluster, self.upcase_bytes) { return Region::UpCase; }
        if dirs.contains_key(&c) { Region::Dirent } else { Region::Data }
    }
}

/// A chain of clusters: its first, NoFatChain or not, and its length.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Chain { pub first: u32, pub flags: u8, pub clusters: u32 }

/// A directory: its chain, and where its own entry set is (its parent's first cluster and the
/// set's entry index); the root has none.
#[derive(Clone, Copy, Debug)]
struct Dir { chain: Chain, at: Option<(u32, u32)> }

/// A file or directory as a directory lists it.
#[derive(Clone, Debug)]
pub struct Found {
    /// the File entry's index in its directory, and the set's length
    pub index: u32,
    pub entries: u32,
    pub name: String,
    pub attr: u16,
    pub chain: Chain,
    pub size: u64,
    pub valid: u64,
}

/// Sectors changed by one operation, written once each at its end in the order first touched:
/// Linux marks a buffer dirty however often it changes it, and a car writing through sends each
/// sector once.
#[derive(Default)]
struct Pending { order: Vec<u64>, data: BTreeMap<u64, Box<[u8; SECTOR]>> }

impl Pending {
    fn get(&mut self, dev: &mut dyn BlockDevice, lba: u64) -> Result<&mut [u8; SECTOR], String> {
        if !self.data.contains_key(&lba) {
            let mut s = Box::new([0u8; SECTOR]);
            dev.read_meta(lba, &mut s[..])?;
            self.data.insert(lba, s);
        }
        Ok(self.data.get_mut(&lba).unwrap())
    }
    /// The sector, to be written at the flush.
    fn modify(&mut self, dev: &mut dyn BlockDevice, lba: u64) -> Result<&mut [u8; SECTOR], String> {
        if !self.order.contains(&lba) { self.order.push(lba); }
        self.get(dev, lba)
    }
    /// A sector given whole (a directory cluster zeroed, a bitmap sector from memory).
    fn put(&mut self, lba: u64, s: [u8; SECTOR]) {
        if !self.order.contains(&lba) { self.order.push(lba); }
        self.data.insert(lba, Box::new(s));
    }
    fn flush(&mut self, dev: &mut dyn BlockDevice) -> Result<(), String> {
        let order = std::mem::take(&mut self.order);
        // contiguous sectors in one write, as a cluster zeroed whole goes
        let mut i = 0;
        while i < order.len() {
            let mut j = i + 1;
            while j < order.len() && order[j] == order[j - 1] + 1 { j += 1; }
            let mut buf = Vec::with_capacity((j - i) * SECTOR);
            for lba in &order[i..j] { buf.extend_from_slice(&self.data[lba][..]); }
            dev.write_meta(order[i], &buf)?;
            i = j;
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ExFat {
    pub layout: Layout,
    pub label: String,
    /// the main boot sector as the driver holds it (`sbi->boot_bh`), and VolumeFlags in memory
    boot: Box<[u8; SECTOR]>,
    vol_flag: u16,
    /// keep VolumeFlags as the driver does; off, the car never writes them (the vfat-like car's
    /// `dirty_flag` off)
    pub track_flags: bool,
    /// the allocation bitmap, kept from the mount (`sbi->vol_amap`)
    bitmap: Vec<u8>,
    /// where the next empty file's search starts (`sbi->clu_srch_ptr`, 2 at the mount)
    clu_srch_ptr: u32,
    used: u32,
    upcase: Arc<Vec<u16>>,
    /// directories met so far, by first cluster
    dirs: BTreeMap<u32, Dir>,
    pub truth: Option<TruthLog>,
}

fn components(path: &str) -> Vec<&str> { path.split('/').filter(|c| !c.is_empty()).collect() }

impl ExFat {
    /// Read the volume as `exfat_fill_super` does: the boot sector, the boot region and its
    /// checksum, the up-case table, the allocation bitmap whole, the root's chain. An MBR's first
    /// exFAT partition (type 0x07), or a volume from LBA 0.
    pub fn mount(dev: &mut dyn BlockDevice) -> Result<Self, String> {
        if dev.block_size() != 512 { return Err(format!("{}-byte blocks: only 512 is supported", dev.block_size())); }
        let mut s = [0u8; SECTOR];
        dev.read_meta(0, &mut s)?;
        let mut volume = 0u64;
        if &s[3..11] != b"EXFAT   " {
            if s[510..512] != [0x55, 0xaa] { return Err("no exFAT boot sector and no MBR".into()); }
            let p = (0..4).map(|i| &s[446 + 16 * i..462 + 16 * i]).find(|e| e[4] == 0x07).ok_or("no exFAT partition (type 0x07) in the MBR")?;
            volume = get32(p, 8) as u64;
        }
        let mut region = vec![0u8; 12 * SECTOR];
        dev.read_meta(volume, &mut region)?;
        let b = &region[..SECTOR];
        if &b[3..11] != b"EXFAT   " || b[510..512] != [0x55, 0xaa] { return Err("no exFAT boot sector".into()); }
        if b[108] != 9 { return Err(format!("{}-byte sectors: only 512 is supported", 1u32 << b[108])); }
        let sum = chksum32(&region[SECTOR..11 * SECTOR], chksum32(&region[..SECTOR], 0, true), false);
        if get32(&region, 11 * SECTOR) != sum { return Err(format!("boot region checksum {:08x}, sector 11 says {:08x}", sum, get32(&region, 11 * SECTOR))); }
        let spc = 1u32 << b[109];
        let mut layout = Layout {
            volume_lba: volume, fat_lba: volume + get32(b, 80) as u64, fat_sectors: get32(b, 84), heap_lba: volume + get32(b, 88) as u64,
            sectors_per_cluster: spc, clusters: get32(b, 92), root_cluster: get32(b, 96), bitmap_cluster: 0, bitmap_bytes: 0, upcase_cluster: 0, upcase_bytes: 0,
        };
        let boot: Box<[u8; SECTOR]> = Box::new(region[..SECTOR].try_into().unwrap());
        let vol_flag = get16(&boot[..], 106);
        let mut fs = ExFat { layout, label: String::new(), boot, vol_flag, track_flags: false, bitmap: Vec::new(), clu_srch_ptr: 2, used: 0, upcase: Arc::new(Vec::new()), dirs: BTreeMap::new(), truth: None };
        let root_chain = fs.fat_chain(dev, layout.root_cluster)?;
        let root = Chain { first: layout.root_cluster, flags: ALLOC_FAT_CHAIN, clusters: root_chain.len() as u32 };
        fs.dirs.insert(layout.root_cluster, Dir { chain: root, at: None });
        // the root's critical entries: the label, the bitmap, the up-case table
        for i in 0..root.clusters * fs.entries_per_cluster() {
            let e = fs.entry(dev, root, i)?;
            match e[0] {
                0 => break,
                T_VOLUME => { let n = (e[1] as usize).min(11); fs.label = String::from_utf16_lossy(&(0..n).map(|k| get16(&e, 2 + 2 * k)).collect::<Vec<_>>()); }
                T_BITMAP if e[1] & 1 == 0 => { layout.bitmap_cluster = get32(&e, 20); layout.bitmap_bytes = get64(&e, 24); }
                T_UPCASE => { layout.upcase_cluster = get32(&e, 20); layout.upcase_bytes = get64(&e, 24); }
                _ => {}
            }
        }
        if layout.bitmap_cluster == 0 || layout.upcase_cluster == 0 { return Err("no allocation bitmap or up-case table in the root".into()); }
        if layout.bitmap_bytes < (layout.clusters as u64).div_ceil(8) { return Err(format!("a bitmap of {} bytes for {} clusters", layout.bitmap_bytes, layout.clusters)); }
        fs.layout = layout;
        let mut raw = vec![0u8; layout.upcase_bytes.next_multiple_of(SECTOR as u64) as usize];
        dev.read_meta(layout.cluster_lba(layout.upcase_cluster), &mut raw)?;
        fs.upcase = Arc::new(decompress_upcase(&raw[..layout.upcase_bytes as usize]));
        let mut bitmap = vec![0u8; layout.bitmap_bytes.next_multiple_of(SECTOR as u64) as usize];
        dev.read_meta(layout.cluster_lba(layout.bitmap_cluster), &mut bitmap)?;
        fs.used = (0..layout.clusters).filter(|&i| bitmap[i as usize / 8] & 1 << (i % 8) != 0).count() as u32;
        fs.bitmap = bitmap;
        Ok(fs)
    }

    pub fn cluster_bytes(&self) -> usize { self.layout.sectors_per_cluster as usize * SECTOR }
    fn entries_per_cluster(&self) -> u32 { self.cluster_bytes() as u32 / 32 }
    pub fn free_clusters(&self) -> u32 { self.layout.clusters - self.used }
    /// VolumeFlags as the driver holds them.
    pub fn vol_flags(&self) -> u16 { self.vol_flag }

    /// Share `log`: the layout goes in at once, then everything this volume does.
    pub fn set_truth(&mut self, log: TruthLog) {
        {
            let mut t = log.lock().unwrap();
            t.layout = Some(self.layout);
            let now = t.dir_clusters.get(&self.layout.root_cluster).copied().unwrap_or(0);
            t.dir_clusters.entry(self.layout.root_cluster).or_insert(now);
        }
        self.truth = Some(log);
    }
    fn note(&self, f: impl FnOnce(&mut Truth)) { if let Some(t) = &self.truth { f(&mut t.lock().unwrap()); } }

    // ---- the FAT and the bitmap ----

    fn fat_get(&self, dev: &mut dyn BlockDevice, p: &mut Pending, c: u32) -> Result<u32, String> {
        let s = p.get(dev, self.layout.fat_sector_of(c))?;
        Ok(get32(&s[..], (c % 128) as usize * 4))
    }
    fn fat_set(&self, dev: &mut dyn BlockDevice, p: &mut Pending, c: u32, v: u32) -> Result<(), String> {
        let s = p.modify(dev, self.layout.fat_sector_of(c))?;
        put32(&mut s[..], (c % 128) as usize * 4, v);
        Ok(())
    }
    /// The chain from `first` through the FAT.
    pub fn fat_chain(&self, dev: &mut dyn BlockDevice, first: u32) -> Result<Vec<u32>, String> {
        let mut p = Pending::default();
        let mut out = Vec::new();
        let mut c = first;
        while (2..self.layout.clusters + 2).contains(&c) {
            if out.len() > self.layout.clusters as usize { return Err("a cluster chain loops".into()); }
            out.push(c);
            c = self.fat_get(dev, &mut p, c)?;
        }
        Ok(out)
    }
    /// The clusters of `chain`: contiguous for NoFatChain, through the FAT otherwise.
    pub fn clusters(&self, dev: &mut dyn BlockDevice, chain: Chain) -> Result<Vec<u32>, String> {
        if chain.first < 2 || chain.clusters == 0 { return Ok(Vec::new()); }
        if chain.flags == ALLOC_NO_FAT_CHAIN { return Ok((chain.first..chain.first + chain.clusters).collect()); }
        let mut c = self.fat_chain(dev, chain.first)?;
        c.truncate(chain.clusters as usize);
        Ok(c)
    }
    fn nth_cluster(&self, dev: &mut dyn BlockDevice, chain: Chain, k: u32) -> Result<u32, String> {
        if chain.flags == ALLOC_NO_FAT_CHAIN { return Ok(chain.first + k); }
        let mut p = Pending::default();
        let mut c = chain.first;
        for _ in 0..k { c = self.fat_get(dev, &mut p, c)?; if !(2..self.layout.clusters + 2).contains(&c) { return Err(format!("chain from {} ends before cluster {k}", chain.first)); } }
        Ok(c)
    }

    fn bit(&self, c: u32) -> bool { let i = (c - 2) as usize; self.bitmap[i / 8] & 1 << (i % 8) != 0 }
    fn set_bit(&mut self, p: &mut Pending, c: u32, on: bool) {
        let i = (c - 2) as usize;
        if on { self.bitmap[i / 8] |= 1 << (i % 8); } else { self.bitmap[i / 8] &= !(1 << (i % 8)); }
        let sec = i / (SECTOR * 8);
        p.put(self.layout.bitmap_sector_of(c), self.bitmap[sec * SECTOR..sec * SECTOR + SECTOR].try_into().unwrap());
    }
    /// The first free cluster from `from` on, round the end (balloc.c `exfat_find_free_bitmap`).
    fn find_free(&self, from: u32) -> Option<u32> {
        let n = self.layout.clusters;
        let start = if (2..n + 2).contains(&from) { from } else { 2 };
        (start..n + 2).chain(2..start).find(|&c| !self.bit(c))
    }

    /// One cluster for a chain that ends at `last` (`EOF` for an empty one) with `flags`, as
    /// `exfat_alloc_cluster` takes one (fatent.c:312-429): its bit set, its FAT entry when the
    /// chain is a FAT chain. The cluster, and the flags the chain must have now: a cluster other
    /// than the hint ends NoFatChain.
    fn alloc_one(&mut self, dev: &mut dyn BlockDevice, p: &mut Pending, last: u32, mut flags: u8) -> Result<(u32, u8), String> {
        if self.used >= self.layout.clusters { return Err("the volume is full".into()); }
        let mut hint = if last == EOF { self.find_free(self.clu_srch_ptr).ok_or("the volume is full")? } else { last + 1 };
        if !(2..self.layout.clusters + 2).contains(&hint) { hint = 2; if flags == ALLOC_NO_FAT_CHAIN { flags = ALLOC_FAT_CHAIN; } }
        let new = self.find_free(hint).ok_or("the volume is full")?;
        if new != hint && flags == ALLOC_NO_FAT_CHAIN { flags = ALLOC_FAT_CHAIN; }
        self.set_bit(p, new, true);
        if flags == ALLOC_FAT_CHAIN { self.fat_set(dev, p, new, EOF)?; }
        self.clu_srch_ptr = hint;
        self.used += 1;
        Ok((new, flags))
    }

    /// FAT entries for a NoFatChain run becoming a FAT chain (fatent.c:135-151).
    fn chain_cont(&self, dev: &mut dyn BlockDevice, p: &mut Pending, first: u32, n: u32) -> Result<(), String> {
        for k in 0..n { self.fat_set(dev, p, first + k, if k + 1 == n { EOF } else { first + k + 1 })?; }
        Ok(())
    }

    // ---- VolumeFlags ----

    /// `exfat_set_vol_flags` (super.c:100-131): nothing when they are already `flag`; dirty
    /// written at once when the boot sector is not waiting to be written, otherwise left dirty for
    /// writeback.
    pub fn set_vol_flags(&mut self, dev: &mut dyn BlockDevice, flag: u16) -> Result<(), String> {
        if !self.track_flags || self.vol_flag == flag { return Ok(()); }
        self.vol_flag = flag;
        put16(&mut self.boot[..], 106, flag);
        let lba = self.layout.volume_lba;
        let sync = flag == VOL_DIRTY && !dev.meta_dirty(lba);
        let at = dev.now_ns();
        self.note(|t| t.flags.push((at, flag, sync)));
        if sync { dev.sync_meta(lba, &self.boot[..]) } else { dev.write_meta(lba, &self.boot[..]) }
    }

    /// The volume's dirty flag as the medium (or the car's cache of it) has it.
    pub fn dirty(&self, dev: &mut dyn BlockDevice) -> Result<bool, String> {
        let mut s = [0u8; SECTOR];
        dev.read_meta(self.layout.volume_lba, &mut s)?;
        Ok(get16(&s, 106) & VOL_DIRTY != 0)
    }

    // ---- directories ----

    fn entry_lba(&self, dev: &mut dyn BlockDevice, chain: Chain, index: u32) -> Result<(u64, usize), String> {
        let per = self.entries_per_cluster();
        let c = self.nth_cluster(dev, chain, index / per)?;
        let byte = (index % per) as usize * 32;
        Ok((self.layout.cluster_lba(c) + (byte / SECTOR) as u64, byte % SECTOR))
    }
    fn entry(&self, dev: &mut dyn BlockDevice, chain: Chain, index: u32) -> Result<[u8; 32], String> {
        let (lba, off) = self.entry_lba(dev, chain, index)?;
        let mut s = [0u8; SECTOR];
        dev.read_meta(lba, &mut s)?;
        Ok(s[off..off + 32].try_into().unwrap())
    }

    /// Every sector of a directory, in order.
    fn dir_sectors(&self, dev: &mut dyn BlockDevice, chain: Chain) -> Result<Vec<u64>, String> {
        let spc = self.layout.sectors_per_cluster as u64;
        Ok(self.clusters(dev, chain)?.into_iter().flat_map(|c| { let l = self.layout.cluster_lba(c); (0..spc).map(move |i| l + i) }).collect())
    }

    /// The file and directory entry sets of a directory, as `exfat_find_dir_entry` walks it, up
    /// to the first unused entry.
    pub fn dir_entries(&self, dev: &mut dyn BlockDevice, chain: Chain) -> Result<Vec<Found>, String> {
        let mut raw = Vec::new();
        for lba in self.dir_sectors(dev, chain)? {
            let mut s = [0u8; SECTOR];
            dev.read_meta(lba, &mut s)?;
            raw.extend_from_slice(&s);
            if s.chunks(32).any(|e| e[0] == 0) { break; }
        }
        let mut out = Vec::new();
        let n = raw.len() / 32;
        let mut i = 0;
        while i < n {
            let e = &raw[i * 32..i * 32 + 32];
            if e[0] == 0 { break; }
            if e[0] != T_FILE { i += 1; continue; }
            let k = e[1] as usize + 1;
            if k < 3 || i + k > n || raw[(i + 1) * 32] != T_STREAM { i += 1; continue; }
            let st = &raw[(i + 1) * 32..(i + 2) * 32];
            let len = st[3] as usize;
            let units: Vec<u16> = (0..k - 2).flat_map(|j| { let ne = &raw[(i + 2 + j) * 32..(i + 3 + j) * 32]; (0..NAME_UNITS).map(move |u| get16(ne, 2 + 2 * u)) }).take(len).collect();
            let size = get64(st, 24);
            let cb = self.cluster_bytes() as u64;
            out.push(Found { index: i as u32, entries: k as u32, name: String::from_utf16_lossy(&units), attr: get16(e, 4),
                             chain: Chain { first: get32(st, 20), flags: st[1] & ALLOC_NO_FAT_CHAIN, clusters: size.div_ceil(cb) as u32 }, size, valid: get64(st, 8) });
            i += k;
        }
        Ok(out)
    }

    fn upper(&self, s: &str) -> Vec<u16> { s.encode_utf16().map(|u| self.upcase.get(u as usize).copied().filter(|&v| v != 0).unwrap_or(u)).collect() }

    fn find(&self, dev: &mut dyn BlockDevice, dir: Chain, name: &str) -> Result<Option<Found>, String> {
        let want = self.upper(name);
        Ok(self.dir_entries(dev, dir)?.into_iter().find(|f| self.upper(&f.name) == want))
    }

    fn dir(&self, first: u32) -> Dir { self.dirs[&first] }

    /// The directory the components name, from the root.
    fn dir_of(&mut self, dev: &mut dyn BlockDevice, comps: &[&str]) -> Result<Dir, String> {
        let mut d = self.dir(self.layout.root_cluster);
        for (i, c) in comps.iter().enumerate() {
            let f = self.find(dev, d.chain, c)?.ok_or_else(|| format!("{}: no such directory", comps[..=i].join("/")))?;
            if f.attr & ATTR_SUBDIR == 0 { return Err(format!("{}: not a directory", comps[..=i].join("/"))); }
            d = self.adopt_dir(&f, d.chain.first);
        }
        Ok(d)
    }

    /// The directory a listing found, kept from then on: a directory grown since keeps the chain
    /// it has in memory, as the driver's inode does.
    fn adopt_dir(&mut self, f: &Found, parent: u32) -> Dir {
        let d = *self.dirs.entry(f.chain.first).or_insert(Dir { chain: f.chain, at: Some((parent, f.index)) });
        if self.truth.is_some() { let c = self.clusters_known(d.chain); self.note(|t| for x in c { t.dir_clusters.entry(x).or_insert(0); }); }
        d
    }
    fn clusters_known(&self, chain: Chain) -> Vec<u32> {
        if chain.flags == ALLOC_NO_FAT_CHAIN { (chain.first..chain.first + chain.clusters).collect() } else { vec![chain.first] }
    }

    /// The entries of the directory at `path` ("" is the root), as `DirEntry`s.
    pub fn list(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<Vec<DirEntry>, String> {
        let d = self.dir_of(dev, &components(path))?;
        Ok(self.dir_entries(dev, d.chain)?.into_iter().map(|f| DirEntry { name: f.name, size: f.size.min(u32::MAX as u64) as u32, cluster: f.chain.first, attr: f.attr as u8 }).collect())
    }

    /// The entry set at `path`, if there is one.
    pub fn stat(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<Option<Found>, String> {
        let comps = components(path);
        let Some((name, parents)) = comps.split_last() else { return Err("the root has no entry".into()) };
        let d = self.dir_of(dev, parents)?;
        self.find(dev, d.chain, name)
    }

    /// The clusters of the file at `path`.
    pub fn clusters_of(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<Vec<u32>, String> {
        let f = self.stat(dev, path)?.ok_or_else(|| format!("{path}: no such file"))?;
        self.clusters(dev, f.chain)
    }

    pub fn read_path(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<Vec<u8>, String> {
        let f = self.stat(dev, path)?.filter(|f| f.attr & ATTR_SUBDIR == 0).ok_or_else(|| format!("{path}: no such file"))?;
        let chain = self.clusters(dev, f.chain)?;
        let cb = self.cluster_bytes();
        let mut out = Vec::with_capacity(f.size as usize);
        let mut i = 0;
        while i < chain.len() && (out.len() as u64) < f.size {
            let mut j = i + 1;
            while j < chain.len() && chain[j] == chain[j - 1] + 1 && (j - i + 1) * cb <= 128 << 10 { j += 1; }
            let mut buf = vec![0u8; (j - i) * cb];
            dev.read_blocks(self.layout.cluster_lba(chain[i]), &mut buf)?;
            out.extend_from_slice(&buf);
            i = j;
        }
        if (out.len() as u64) < f.size { return Err(format!("{path}: the chain holds {} of {} bytes", out.len(), f.size)); }
        out.truncate(f.size as usize);
        Ok(out)
    }

    /// The first run of `n` unused or deleted entries, growing the directory by a cluster when it
    /// has none (namei.c:199-296 `exfat_search_empty_slot`, 302-398 `exfat_find_empty_entry`).
    fn find_empty(&mut self, dev: &mut dyn BlockDevice, p: &mut Pending, first: u32, n: u32) -> Result<u32, String> {
        loop {
            let d = self.dir(first);
            let per = self.entries_per_cluster();
            let total = d.chain.clusters * per;
            let mut run = 0;
            for i in 0..total {
                let (lba, off) = self.entry_lba(dev, d.chain, i)?;
                let t = p.get(dev, lba)?[off];
                if t == 0 || t < 0x80 { run += 1; if run == n { return Ok(i + 1 - n); } } else { run = 0; }
            }
            self.grow_dir(dev, p, first)?;
        }
    }

    /// A cluster more for the directory at `first`: hinted after its last, zeroed, linked, and
    /// its own entry's size and flags rewritten (namei.c:330-395).
    fn grow_dir(&mut self, dev: &mut dyn BlockDevice, p: &mut Pending, first: u32) -> Result<(), String> {
        let mut d = self.dir(first);
        let last = *self.clusters(dev, d.chain)?.last().ok_or("a directory without clusters")?;
        let (new, flags) = self.alloc_one(dev, p, last, d.chain.flags)?;
        self.zero_cluster(p, new);
        if flags != d.chain.flags { self.chain_cont(dev, p, d.chain.first, d.chain.clusters)?; d.chain.flags = ALLOC_FAT_CHAIN; }
        if flags == ALLOC_FAT_CHAIN { self.fat_set(dev, p, last, new)?; }
        d.chain.clusters += 1;
        self.dirs.insert(first, d);
        let now = dev.now_ns();
        self.note(|t| { t.dir_clusters.insert(new, now); });
        if let Some((parent, index)) = d.at {
            let bytes = d.chain.clusters as u64 * self.cluster_bytes() as u64;
            self.edit_set(dev, p, parent, index, |set| { let st = &mut set[32..64]; put64(st, 8, bytes); put64(st, 24, bytes); st[1] = d.chain.flags; })?;
        }
        // written now, so the walks that follow see the new cluster through the FAT
        p.flush(dev)
    }

    fn zero_cluster(&mut self, p: &mut Pending, c: u32) {
        let l = self.layout.cluster_lba(c);
        for i in 0..self.layout.sectors_per_cluster as u64 { p.put(l + i, [0u8; SECTOR]); }
    }

    /// The sectors an entry set occupies.
    fn set_lbas(&self, dev: &mut dyn BlockDevice, chain: Chain, index: u32, n: u32) -> Result<Vec<u64>, String> {
        let mut out: Vec<u64> = Vec::new();
        for k in 0..n { let (lba, _) = self.entry_lba(dev, chain, index + k)?; if out.last() != Some(&lba) { out.push(lba); } }
        Ok(out)
    }

    /// Change the entry set at `index` of the directory at `dir` with `f` and write it back with its
    /// checksum, as `exfat_get_dentry_set`, `exfat_update_dir_chksum_with_entry_set` and
    /// `exfat_free_dentry_set` do (dir.c:587-614, 812-...): every sector of the set is written.
    fn edit_set(&self, dev: &mut dyn BlockDevice, p: &mut Pending, dir: u32, index: u32, f: impl FnOnce(&mut [u8])) -> Result<(), String> {
        let chain = self.dir(dir).chain;
        let (lba0, off0) = self.entry_lba(dev, chain, index)?;
        let n = p.get(dev, lba0)?[off0 + 1] as u32 + 1;
        let mut slots = Vec::with_capacity(n as usize);
        let mut set = Vec::with_capacity(n as usize * 32);
        for k in 0..n {
            let (lba, off) = self.entry_lba(dev, chain, index + k)?;
            set.extend_from_slice(&p.get(dev, lba)?[off..off + 32]);
            slots.push((lba, off));
        }
        f(&mut set);
        let sum = set_checksum(&set);
        put16(&mut set, 2, sum);
        for (k, (lba, off)) in slots.into_iter().enumerate() { p.modify(dev, lba)?[off..off + 32].copy_from_slice(&set[k * 32..k * 32 + 32]); }
        Ok(())
    }

    /// Write a new entry set for `name` at `index` of the directory at `dir`, as
    /// `exfat_init_dir_entry` and `exfat_init_ext_entry` do (dir.c:434-563): the File entry's type,
    /// attributes and times, the Stream Extension's flags, cluster and sizes, the count of
    /// secondary entries, the name's length and hash, the name entries, then the checksum.
    fn init_set(&self, dev: &mut dyn BlockDevice, p: &mut Pending, dir: u32, index: u32, name: &str, is_dir: bool, start: u32, size: u64) -> Result<u32, String> {
        let units: Vec<u16> = name.encode_utf16().collect();
        if units.is_empty() || units.len() > 255 { return Err(format!("{name}: a name holds 1 to 255 characters")); }
        let n = (units.len() - 1) / NAME_UNITS as usize + 3;
        let chain = self.dir(dir).chain;
        let mut slot = |p: &mut Pending, k: usize, f: &mut dyn FnMut(&mut [u8])| -> Result<(), String> {
            let (lba, off) = self.entry_lba(dev, chain, index + k as u32)?;
            f(&mut p.modify(dev, lba)?[off..off + 32]);
            Ok(())
        };
        slot(p, 0, &mut |e| {
            e[0] = T_FILE;
            put16(e, 4, if is_dir { ATTR_SUBDIR } else { ATTR_ARCHIVE });
            for (t, d) in [(8, 10), (12, 14), (16, 18)] { put16(e, t, 0); put16(e, d, DATE); }
            e[20] = 0; e[21] = 0;
            e[22] = TZ_UTC; e[23] = TZ_UTC; e[24] = TZ_UTC;
        })?;
        slot(p, 1, &mut |e| {
            e[0] = T_STREAM;
            e[1] = if is_dir { ALLOC_NO_FAT_CHAIN } else { ALLOC_FAT_CHAIN };
            put32(e, 20, start);
            put64(e, 8, size);
            put64(e, 24, size);
        })?;
        slot(p, 0, &mut |e| e[1] = (n - 1) as u8)?;
        let upper = self.upper(name);
        let hash = chksum16(&upper.iter().flat_map(|u| u.to_le_bytes()).collect::<Vec<u8>>(), 0, false);
        slot(p, 1, &mut |e| { e[3] = units.len() as u8; put16(e, 4, hash); })?;
        for k in 2..n {
            let part = &units[(k - 2) * NAME_UNITS..];
            slot(p, k, &mut |e| {
                e[0] = T_NAME;
                e[1] = 0;
                // up to and including the terminating 0, the rest as the slot held it
                for u in 0..NAME_UNITS {
                    let v = part.get(u).copied().unwrap_or(0);
                    put16(e, 2 + 2 * u, v);
                    if v == 0 { break; }
                }
            })?;
        }
        self.edit_set(dev, p, dir, index, |_| {})?;
        Ok(n as u32)
    }

    /// `__exfat_write_inode` (inode.c:18-79) for the entry set at `index` of `dir`: VolumeFlags
    /// dirty, the attributes and times, ValidDataLength and DataLength, the checksum.
    fn write_inode(&mut self, dev: &mut dyn BlockDevice, p: &mut Pending, dir: u32, index: u32, attr: u16, size: u64) -> Result<(), String> {
        self.set_vol_flags(dev, VOL_DIRTY)?;
        self.edit_set(dev, p, dir, index, |set| {
            put16(set, 4, attr);
            let st = &mut set[32..64];
            put64(st, 8, size);
            put64(st, 24, size);
        })
    }

    /// A directory's inode written back, as it is after an entry in it was made or removed
    /// (`mark_inode_dirty(dir)`); the root has none (inode.c:28-29).
    fn write_dir_inode(&mut self, dev: &mut dyn BlockDevice, p: &mut Pending, dir: u32) -> Result<(), String> {
        let d = self.dir(dir);
        let Some((parent, index)) = d.at else { return Ok(()) };
        let size = d.chain.clusters as u64 * self.cluster_bytes() as u64;
        self.write_inode(dev, p, parent, index, ATTR_SUBDIR, size)
    }

    /// Make the directories of `path` that are missing, as `mkdir -p` does with `exfat_mkdir`
    /// (namei.c:864-...): VolumeFlags dirty; the entry's slots found; a NoFatChain cluster from the
    /// search pointer, its bit set, zeroed whole (fatent.c:247-310); the entry set; VolumeFlags
    /// clean; the parent's inode.
    pub fn mkdir_all(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<u32, String> {
        let mut d = self.dir(self.layout.root_cluster);
        let comps = components(path);
        for (i, &c) in comps.iter().enumerate() {
            if let Some(f) = self.find(dev, d.chain, c)? {
                if f.attr & ATTR_SUBDIR == 0 { return Err(format!("{c}: exists and is not a directory")); }
                d = self.adopt_dir(&f, d.chain.first);
                continue;
            }
            let mut p = Pending::default();
            self.set_vol_flags(dev, VOL_DIRTY)?;
            let units = c.encode_utf16().count() as u32;
            let n = units.saturating_sub(1) / NAME_UNITS as u32 + 3;
            let index = self.find_empty(dev, &mut p, d.chain.first, n)?;
            let (new, _) = self.alloc_one(dev, &mut p, EOF, ALLOC_NO_FAT_CHAIN)?;
            self.zero_cluster(&mut p, new);
            let cb = self.cluster_bytes() as u64;
            self.init_set(dev, &mut p, d.chain.first, index, c, true, new, cb)?;
            let set_lbas = self.set_lbas(dev, d.chain, index, n)?;
            p.flush(dev)?;
            self.set_vol_flags(dev, VOL_CLEAN)?;
            let child = Dir { chain: Chain { first: new, flags: ALLOC_NO_FAT_CHAIN, clusters: 1 }, at: Some((d.chain.first, index)) };
            self.dirs.insert(new, child);
            let now = dev.now_ns();
            let full = comps[..=i].join("/");
            self.note(|t| {
                t.dir_clusters.insert(new, now);
                let ft = FileTruth { dir: true, created_ns: now, set_lbas, entries: n, clusters: vec![(new, now)], ..Default::default() };
                if let Some(old) = t.files.insert(full.clone(), ft) { t.past.push((full, old)); }
            });
            let mut p = Pending::default();
            self.write_dir_inode(dev, &mut p, d.chain.first)?;
            p.flush(dev)?;
            d = child;
        }
        Ok(d.chain.first)
    }

    /// Start a file at `path` in a directory that exists, as `exfat_create` does (namei.c:552-591):
    /// VolumeFlags dirty, the entry set (a FAT-chain flag, no cluster, size 0), VolumeFlags clean,
    /// then the directory's inode.
    pub fn create_path(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<ExFile, String> {
        let comps = components(path);
        let Some((name, parents)) = comps.split_last() else { return Err("a file needs a name".into()) };
        let d = self.dir_of(dev, parents)?;
        if self.find(dev, d.chain, name)?.is_some() { return Err(format!("{path}: exists")); }
        let mut p = Pending::default();
        self.set_vol_flags(dev, VOL_DIRTY)?;
        let n = (name.encode_utf16().count() as u32).saturating_sub(1) / NAME_UNITS as u32 + 3;
        let index = self.find_empty(dev, &mut p, d.chain.first, n)?;
        let d = self.dir(d.chain.first);
        self.init_set(dev, &mut p, d.chain.first, index, name, false, 0, 0)?;
        let set_lbas = self.set_lbas(dev, d.chain, index, n)?;
        p.flush(dev)?;
        self.set_vol_flags(dev, VOL_CLEAN)?;
        let now = dev.now_ns();
        let full = path.to_string();
        self.note(|t| {
            let ft = FileTruth { created_ns: now, set_lbas, entries: n, ..Default::default() };
            if let Some(old) = t.files.insert(full.clone(), ft) { t.past.push((full, old)); }
        });
        let mut p = Pending::default();
        self.write_dir_inode(dev, &mut p, d.chain.first)?;
        p.flush(dev)?;
        Ok(ExFile { path: path.to_string(), dir: d.chain.first, index, first: EOF, last: EOF, clusters: 0, flags: ALLOC_NO_FAT_CHAIN, size: 0, linux: false })
    }

    /// Delete the file at `path` as Linux does: `exfat_unlink` (namei.c:801-861: VolumeFlags
    /// dirty, every entry of the set marked deleted, VolumeFlags clean, the directory's inode),
    /// then the eviction's `__exfat_truncate` to 0 (file.c:93-220: VolumeFlags dirty, each cluster's
    /// bit cleared, walking the FAT for a FAT chain, VolumeFlags clean). The FAT is not written.
    /// Returns the clusters freed.
    pub fn delete_path(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<Vec<u32>, String> {
        let comps = components(path);
        let Some((name, parents)) = comps.split_last() else { return Err("a file needs a name".into()) };
        let d = self.dir_of(dev, parents)?;
        let f = self.find(dev, d.chain, name)?.ok_or_else(|| format!("{path}: no such file"))?;
        if f.attr & ATTR_SUBDIR != 0 { return Err(format!("{path}: not a file")); }
        let mut p = Pending::default();
        self.set_vol_flags(dev, VOL_DIRTY)?;
        for k in 0..f.entries {
            let (lba, off) = self.entry_lba(dev, d.chain, f.index + k)?;
            p.modify(dev, lba)?[off] &= 0x7f;
        }
        p.flush(dev)?;
        self.set_vol_flags(dev, VOL_CLEAN)?;
        let mut p = Pending::default();
        self.write_dir_inode(dev, &mut p, d.chain.first)?;
        p.flush(dev)?;
        // the eviction
        let mut p = Pending::default();
        self.set_vol_flags(dev, VOL_DIRTY)?;
        let freed = if f.chain.first >= 2 && f.size > 0 { self.clusters(dev, f.chain)? } else { Vec::new() };
        for &c in &freed { self.set_bit(&mut p, c, false); }
        self.used -= freed.len() as u32;
        p.flush(dev)?;
        self.set_vol_flags(dev, VOL_CLEAN)?;
        let now = dev.now_ns();
        let path = path.to_string();
        self.note(|t| if let Some(ft) = t.files.get_mut(&path) { ft.deleted_ns = Some(now); });
        Ok(freed)
    }

    /// Check and repair the volume as exfatprogs 1.0.4's `fsck.exfat -y` does (fsck/fsck.c, the
    /// version Tesla's buildroot carries): VolumeFlags dirty, written at once; the boot region read
    /// and its checksum checked; every directory read; each file's chain walked (the FAT read for
    /// a FAT chain), a chain into a free or already owned cluster, or longer or shorter than the
    /// size, cut, a ValidDataLength past DataLength or a wrong checksum fixed; when anything was cut,
    /// the FAT read whole and the entries of clusters no file holds zeroed, and the bitmap's
    /// differing sectors written (`reclaim_free_clusters`, 1283-1294); VolumeFlags clean. `Bulk`
    /// writes the FAT and the bitmap whole instead (an assumption about the car's tool, as for
    /// FAT32). Clusters marked in the bitmap that no file holds are freed only when something
    /// was cut, as 1.0.4 does.
    pub fn fsck(&mut self, dev: &mut dyn BlockDevice, write: FsckWrite) -> Result<FsckReport, String> {
        let mut r = FsckReport::default();
        let l = self.layout;
        let was = self.dirty(dev)?;
        let mut boot = [0u8; SECTOR];
        dev.read_meta(l.volume_lba, &mut boot)?;
        let f = get16(&boot, 106) | VOL_DIRTY;
        put16(&mut boot, 106, f);
        dev.sync_meta(l.volume_lba, &boot)?;
        let mut region = vec![0u8; 12 * SECTOR];
        dev.read_meta(l.volume_lba, &mut region)?;
        let mut disk_bitmap = vec![0u8; l.bitmap_bytes.next_multiple_of(SECTOR as u64) as usize];
        dev.read_meta(l.cluster_lba(l.bitmap_cluster), &mut disk_bitmap)?;
        r.bitmap_bytes_read = disk_bitmap.len() as u64;
        let n = l.clusters;
        let mut owned = vec![0u8; disk_bitmap.len()];
        let own = |owned: &mut Vec<u8>, c: u32| { let i = (c - 2) as usize; owned[i / 8] |= 1 << (i % 8); };
        let is = |m: &[u8], c: u32| { let i = (c - 2) as usize; m[i / 8] & 1 << (i % 8) != 0 };
        let root_chain = self.fat_chain(dev, l.root_cluster)?;
        for &c in &root_chain { own(&mut owned, c); }
        let run = |first: u32, bytes: u64| first..first + bytes.div_ceil(self.cluster_bytes() as u64) as u32;
        for c in run(l.bitmap_cluster, l.bitmap_bytes).chain(run(l.upcase_cluster, l.upcase_bytes)) { own(&mut owned, c); }
        let mut fat_reads = BTreeSet::new();
        let mut cut = false;
        let mut dirs = vec![Chain { first: l.root_cluster, flags: ALLOC_FAT_CHAIN, clusters: root_chain.len() as u32 }];
        let cb = self.cluster_bytes() as u64;
        while let Some(dir) = dirs.pop() {
            let mut p = Pending::default();
            for f in self.dir_entries(dev, dir)? {
                let is_dir = f.attr & ATTR_SUBDIR != 0;
                if !is_dir { r.files += 1; }
                let contiguous = f.chain.flags == ALLOC_NO_FAT_CHAIN;
                let heap = |c: u32| (2..n + 2).contains(&c);
                // check_clus_chain (fsck.c:419-545): a chain that leaves the heap, meets a cluster
                // already held or one the bitmap marks free, runs past the size or ends before it
                // is cut where it went wrong, the size set to the clusters kept
                let want = f.size.div_ceil(cb);
                let (mut count, mut prev, mut c) = (0u64, 0u32, f.chain.first);
                let mut why: Option<&mut Vec<String>> = None;
                if (f.size == 0 && f.chain.first != 0) || (f.size > 0 && !heap(f.chain.first)) { why = Some(&mut r.cross_or_broken); }
                while why.is_none() && f.size > 0 && c != EOF {
                    if count >= want { if contiguous { break; } why = Some(&mut r.truncated); break; }
                    if !heap(c) || is(&owned, c) || !is(&disk_bitmap, c) { why = Some(&mut r.cross_or_broken); break; }
                    let next = if contiguous { c + 1 } else { fat_reads.insert(l.fat_sector_of(c)); let mut q = Pending::default(); self.fat_get(dev, &mut q, c)? };
                    if !contiguous && !heap(next) && next != EOF { why = Some(&mut r.cross_or_broken); break; }
                    count += 1;
                    own(&mut owned, c);
                    prev = c;
                    c = next;
                }
                if why.is_none() && count < want { why = Some(&mut r.resized); }
                let truncate = why.is_some();
                if let Some(list) = why { list.push(f.name.clone()); cut = true; }
                let new_size = count * cb;
                let mut set = Vec::new();
                for j in 0..f.entries { let (lba, off) = self.entry_lba(dev, dir, f.index + j)?; set.extend_from_slice(&p.get(dev, lba)?[off..off + 32]); }
                let sum_ok = get16(&set, 2) == set_checksum(&set);
                let mut changed = false;
                {
                    let st = &mut set[32..64];
                    if get64(st, 8) > get64(st, 24) { let s = get64(st, 24); put64(st, 8, s); changed = true; }
                    if truncate {
                        if get64(st, 8) > new_size { put64(st, 8, new_size); }
                        if prev == 0 { put32(st, 20, 0); }
                        put64(st, 24, new_size);
                        changed = true;
                    }
                    if get64(st, 24) == 0 && st[1] == ALLOC_NO_FAT_CHAIN { st[1] = ALLOC_FAT_CHAIN; changed = true; }
                }
                if truncate && !contiguous && prev != 0 { let mut q = Pending::default(); self.fat_set(dev, &mut q, prev, EOF)?; q.flush(dev)?; }
                if changed || !sum_ok {
                    let sum = set_checksum(&set);
                    put16(&mut set, 2, sum);
                    for j in 0..f.entries { let (lba, off) = self.entry_lba(dev, dir, f.index + j)?; p.modify(dev, lba)?[off..off + 32].copy_from_slice(&set[j as usize * 32..j as usize * 32 + 32]); }
                }
                let size = if truncate { new_size } else { f.size };
                if is_dir && size > 0 { dirs.push(Chain { first: f.chain.first, flags: f.chain.flags, clusters: (size / cb) as u32 }); }
            }
            r.dir_sectors_written += p.order.len() as u64;
            p.flush(dev)?;
        }
        r.fat_bytes_read = fat_reads.len() as u64 * SECTOR as u64;
        let lost: Vec<u32> = (2..n + 2).filter(|&c| is(&disk_bitmap, c) && !is(&owned, c)).collect();
        let fat_bytes = l.fat_sectors as usize * SECTOR;
        if cut || write == FsckWrite::Bulk {
            // the FAT whole, a free cluster's stale entry zeroed
            let mut fat = vec![0u8; fat_bytes];
            for (i, part) in fat.chunks_mut(2048 * SECTOR).enumerate() { dev.read_meta(l.fat_lba + i as u64 * 2048, part)?; }
            r.fat_bytes_read += fat_bytes as u64;
            let old = fat.clone();
            if cut { for c in 2..n + 2 { if !is(&owned, c) && get32(&fat, c as usize * 4) != 0 { put32(&mut fat, c as usize * 4, 0); } } }
            let changed: Vec<usize> = (0..l.fat_sectors as usize).filter(|&i| fat[i * SECTOR..(i + 1) * SECTOR] != old[i * SECTOR..(i + 1) * SECTOR]).collect();
            r.fat_sectors_changed = changed.len() as u64;
            if write == FsckWrite::Bulk {
                for (i, part) in fat.chunks(2048 * SECTOR).enumerate() { dev.write_meta(l.fat_lba + i as u64 * 2048, part)?; }
                r.fat_bytes_written = fat_bytes as u64;
            } else {
                for &i in &changed { dev.write_meta(l.fat_lba + i as u64, &fat[i * SECTOR..(i + 1) * SECTOR])?; }
                r.fat_bytes_written = changed.len() as u64 * SECTOR as u64;
            }
            let new_bitmap = if cut { owned.clone() } else { disk_bitmap.clone() };
            let base = l.cluster_lba(l.bitmap_cluster);
            if write == FsckWrite::Bulk {
                dev.write_meta(base, &new_bitmap)?;
                r.bitmap_bytes_written = new_bitmap.len() as u64;
            } else {
                for i in 0..new_bitmap.len() / SECTOR {
                    if new_bitmap[i * SECTOR..(i + 1) * SECTOR] != disk_bitmap[i * SECTOR..(i + 1) * SECTOR] { dev.write_meta(base + i as u64, &new_bitmap[i * SECTOR..(i + 1) * SECTOR])?; r.bitmap_bytes_written += SECTOR as u64; }
                }
            }
            if cut { r.lost_clusters = lost.len() as u32; }
        }
        dev.read_meta(l.volume_lba, &mut boot)?;
        let f = get16(&boot, 106) & !VOL_DIRTY;
        put16(&mut boot, 106, f);
        dev.sync_meta(l.volume_lba, &boot)?;
        r.dirty_cleared = was;
        r.unreclaimed = if cut { 0 } else { lost.len() as u32 };
        // what the driver held is stale now
        *self = ExFat { truth: self.truth.take(), track_flags: self.track_flags, ..ExFat::mount(dev)? };
        Ok(r)
    }
}

/// The up-case table from its stored form: 0xFFFF and a count skip that many characters that
/// map to themselves (nls.c `exfat_load_upcase_table`); 0 marks a character as itself.
fn decompress_upcase(raw: &[u8]) -> Vec<u16> {
    let mut t = vec![0u16; 0x1_0000];
    let (mut index, mut skip) = (0usize, false);
    for u in raw.chunks(2).map(|c| u16::from_le_bytes([c[0], c.get(1).copied().unwrap_or(0)])) {
        if index > 0xffff { break; }
        if skip { index += u as usize; skip = false; }
        else if u as usize == index { index += 1; }
        else if u == 0xffff { skip = true; }
        else { t[index] = u; index += 1; }
    }
    t
}

/// A file being written.
pub struct ExFile {
    pub path: String,
    /// its directory's first cluster, and its entry set's index there
    dir: u32,
    index: u32,
    first: u32,
    last: u32,
    clusters: u32,
    flags: u8,
    size: u64,
    /// Linux's inode writeback after every chunk: the entry set's sizes rewritten as the file
    /// grows (inode.c `__exfat_write_inode`), which a writeback cache lands late
    pub linux: bool,
}

impl ExFile {
    pub fn size(&self) -> u64 { self.size }
    pub fn first_cluster(&self) -> u32 { if self.first == EOF { 0 } else { self.first } }
    pub fn is_chained(&self) -> bool { self.flags == ALLOC_FAT_CHAIN }

    /// Append `data` (whole clusters, but for the file's last chunk). Each cluster as the pages
    /// reach it, as `exfat_map_cluster` takes it (inode.c:104-262): VolumeFlags dirty; a cluster
    /// hinted after the file's last; a FAT-chain conversion when it is not that one; the entry set
    /// rewritten for the first cluster and at a conversion, with the size the file has then. The
    /// data in one write per contiguous run; with `linux`, the inode written back.
    pub fn write(&mut self, fs: &mut ExFat, dev: &mut dyn BlockDevice, data: &[u8]) -> Result<(), String> {
        if data.is_empty() { return Ok(()); }
        let cb = fs.cluster_bytes() as u64;
        if self.size % cb != 0 { return Err("a chunk after a partial cluster".into()); }
        let mut p = Pending::default();
        let mut taken = Vec::new();
        for k in 0..(data.len() as u64).div_ceil(cb) {
            fs.set_vol_flags(dev, VOL_DIRTY)?;
            let (new, flags) = fs.alloc_one(dev, &mut p, self.last, self.flags)?;
            let i_size = self.size + k * cb;
            let mut modified = false;
            let now = dev.now_ns();
            if self.last == EOF {
                if flags == ALLOC_FAT_CHAIN { self.flags = ALLOC_FAT_CHAIN; }
                self.first = new;
                modified = true;
            } else {
                if flags != self.flags {
                    fs.chain_cont(dev, &mut p, self.first, self.clusters)?;
                    self.flags = ALLOC_FAT_CHAIN;
                    modified = true;
                    let (path, n) = (self.path.clone(), self.clusters);
                    fs.note(|t| if let Some(f) = t.files.get_mut(&path) { f.chained = Some((now, n)); });
                }
                if flags == ALLOC_FAT_CHAIN { fs.fat_set(dev, &mut p, self.last, new)?; }
            }
            self.last = new;
            self.clusters += 1;
            taken.push(new);
            let path = self.path.clone();
            fs.note(|t| if let Some(f) = t.files.get_mut(&path) { f.clusters.push((new, now)); });
            if modified {
                let (first, fl) = (self.first, self.flags);
                fs.edit_set(dev, &mut p, self.dir, self.index, |set| {
                    let st = &mut set[32..64];
                    st[1] = fl;
                    put32(st, 20, first);
                    put64(st, 8, i_size);
                    put64(st, 24, i_size);
                })?;
            }
        }
        p.flush(dev)?;
        let mut i = 0;
        while i < taken.len() {
            let mut j = i + 1;
            while j < taken.len() && taken[j] == taken[j - 1] + 1 { j += 1; }
            let (from, to) = (i * cb as usize, (j * cb as usize).min(data.len()));
            let mut buf = data[from..to].to_vec();
            buf.resize((j - i) * cb as usize, 0);
            dev.write_blocks(fs.layout.cluster_lba(taken[i]), &buf)?;
            i = j;
        }
        self.size += data.len() as u64;
        if self.linux { self.write_inode(fs, dev)?; }
        Ok(())
    }

    fn write_inode(&self, fs: &mut ExFat, dev: &mut dyn BlockDevice) -> Result<(), String> {
        let mut p = Pending::default();
        let size = if self.first == EOF { 0 } else { self.size };
        fs.write_inode(dev, &mut p, self.dir, self.index, ATTR_ARCHIVE, size)?;
        p.flush(dev)
    }

    /// The inode written back with the final size, as writeback does after the close.
    pub fn close(self, fs: &mut ExFat, dev: &mut dyn BlockDevice) -> Result<u64, String> {
        self.write_inode(fs, dev)?;
        let now = dev.now_ns();
        let (path, size) = (self.path.clone(), self.size);
        fs.note(|t| if let Some(f) = t.files.get_mut(&path) { f.closed_ns = Some(now); f.size = size; });
        Ok(self.size)
    }
}
