//! FAT32 on the host side of a block device, for the files a car writes onto the drive it
//! records to: mount (an MBR's first FAT32 partition, or a volume from LBA 0), list, read, and
//! write files in a vfat-like pattern: the directory entry first, the data in chunks as it comes,
//! the FAT sectors covering each chunk's clusters (both copies), and at the end the entry's size
//! and first cluster and FSInfo. A file cut off mid-write here keeps size 0 and no cluster, its
//! data orphaned. `FileWriter::linux` rewrites the entry and FSInfo after every chunk, and the
//! metadata goes through `BlockDevice::read_meta` and `write_meta`, so a `pagecache::PageCache`
//! underneath gives Linux's writeback order (data in batches, metadata held back, intermediate
//! entry sizes). Files are deleted as Linux unlinks them, the dirty flag kept, and fsck run as
//! dosfstools does. Paths reach subdirectories (`mkdir_all`), and a name that is not an upper-case
//! 8.3 name gets long-name entries before its short one, as Windows writes them. Field offsets:
//! Microsoft FAT32 File System Specification 1.03 (fatgen103), "FAT Directory Structure" and "FAT
//! Long Directory Entries".

/// Blocks on the other end, by number.
pub trait BlockDevice {
    fn block_size(&self) -> u32;
    fn blocks(&self) -> u64;
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String>;
    fn write_blocks(&mut self, lba: u64, data: &[u8]) -> Result<(), String>;
    /// Filesystem metadata (boot sector, FSInfo, FAT, directories), which a cache may keep
    /// (`pagecache::PageCache`); a plain device reads and writes it as blocks.
    fn read_meta(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String> { self.read_blocks(lba, buf) }
    fn write_meta(&mut self, lba: u64, data: &[u8]) -> Result<(), String> { self.write_blocks(lba, data) }
    /// Let time pass with nothing to do (a camera filling the next chunk).
    fn pause_ns(&mut self, _ns: u64) -> Result<(), String> { Ok(()) }
    /// Emulated time, for pacing; a device without a clock stays at 0.
    fn now_ns(&mut self) -> u64 { 0 }
}

/// A disk in memory.
pub struct MemDisk(pub Vec<u8>);
impl BlockDevice for MemDisk {
    fn block_size(&self) -> u32 { 512 }
    fn blocks(&self) -> u64 { self.0.len() as u64 / 512 }
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String> {
        let o = lba as usize * 512;
        buf.copy_from_slice(self.0.get(o..o + buf.len()).ok_or("read past the end")?);
        Ok(())
    }
    fn write_blocks(&mut self, lba: u64, data: &[u8]) -> Result<(), String> {
        let o = lba as usize * 512;
        self.0.get_mut(o..o + data.len()).ok_or("write past the end")?.copy_from_slice(data);
        Ok(())
    }
}

/// What a clip written to a camera's schedule saw.
#[derive(Clone, Debug, Default)]
pub struct ClipStats {
    pub bytes: u64,
    pub chunks: usize,
    pub elapsed_ns: u64,
    /// how far the latest chunk started behind its due time, and which chunk that was
    pub worst_late_ns: u64,
    pub worst_chunk: usize,
    /// chunks that started a whole gap or more behind: the car needed more than one chunk of buffer
    pub behind: usize,
    /// bytes over time from the start of the second half's first chunk to the close
    pub second_half_mb_s: f64,
}

impl ClipStats {
    pub fn mb_s(&self) -> f64 { self.bytes as f64 * 1e3 / self.elapsed_ns.max(1) as f64 }
    /// Never a whole chunk behind the camera.
    pub fn kept_up(&self, gap_ns: u64) -> bool { gap_ns == 0 || self.behind == 0 }
    pub fn summary(&self, gap_ns: u64) -> String {
        let pace = if gap_ns == 0 { "unpaced".to_string() } else {
            format!("camera {:.2} MB/s, worst {:.1} ms late (chunk {}), {}", self.bytes as f64 * 1e3 / (self.chunks as u64 * gap_ns).max(1) as f64,
                    self.worst_late_ns as f64 / 1e6, self.worst_chunk, if self.kept_up(gap_ns) { "kept up".to_string() } else { format!("fell behind on {} chunks", self.behind) })
        };
        format!("{} bytes in {} chunks, {:.3} s, {:.2} MB/s, second half {:.2} MB/s; {pace}", self.bytes, self.chunks, self.elapsed_ns as f64 / 1e9, self.mb_s(), self.second_half_mb_s)
    }
}

/// Record `data` as a camera feeds it: chunk i is ready at t0 + i·gap, t0 taken before the file
/// is created. The car writes a chunk as soon as it is ready and the previous one is done, and
/// waits only for one not ready yet; a chunk that starts after its due time counts as late.
pub fn write_clip(fs: &mut Fat32, dev: &mut dyn BlockDevice, name: &str, data: &[u8], chunk: usize, gap_ns: u64, fat_every: u32) -> Result<ClipStats, String> {
    let chunk = chunk.max(fs.cluster_bytes()) / fs.cluster_bytes() * fs.cluster_bytes();
    let t0 = dev.now_ns();
    let mut st = ClipStats { chunks: data.len().div_ceil(chunk), ..Default::default() };
    let (mut half_t, mut half_bytes) = (t0, 0u64);
    let mut f = fs.create_path(dev, name)?;
    f.fat_every = fat_every;
    for (i, part) in data.chunks(chunk).enumerate() {
        let due = t0 + i as u64 * gap_ns;
        let now = dev.now_ns();
        if due > now { dev.pause_ns(due - now)?; }
        let late = now.saturating_sub(due);
        if late > st.worst_late_ns { st.worst_late_ns = late; st.worst_chunk = i; }
        if gap_ns > 0 && late >= gap_ns { st.behind += 1; }
        if i == st.chunks / 2 { half_t = dev.now_ns(); half_bytes = (data.len() - i * chunk) as u64; }
        f.write(fs, dev, part)?;
    }
    st.bytes = f.close(fs, dev)?;
    let end = dev.now_ns();
    st.elapsed_ns = (end - t0).max(1);
    st.second_half_mb_s = half_bytes as f64 * 1e3 / (end - half_t).max(1) as f64;
    Ok(st)
}

const EOC: u32 = 0x0fff_ffff;
const FREE: u32 = 0;

fn get16(b: &[u8], at: usize) -> u16 { u16::from_le_bytes([b[at], b[at + 1]]) }
fn get32(b: &[u8], at: usize) -> u32 { u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]) }
fn put32(b: &mut [u8], at: usize, v: u32) { b[at..at + 4].copy_from_slice(&v.to_le_bytes()); }

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DirEntry { pub name: String, pub size: u32, pub cluster: u32, pub attr: u8 }

#[derive(Clone, Debug)]
pub struct Fat32 {
    pub volume_lba: u64,
    pub sectors_per_cluster: u32,
    pub reserved: u32,
    pub fats: u32,
    pub fat_sectors: u32,
    pub root_cluster: u32,
    pub fsinfo: u32,
    pub clusters: u32,
    pub data_lba: u64,
    pub label: String,
    /// clusters allocated since FSInfo was last written, and where allocation looks first: the
    /// last cluster allocated, or FSInfo's hint (fatgen103 calls it the next free cluster, Linux
    /// and FatFs keep the last allocated one there; a search that starts at it covers both)
    allocated: u32,
    hint: u32,
    /// clusters freed since FSInfo was last written
    freed: u32,
}

/// `name.ext` as the 11 bytes of a short directory entry, upper case.
pub fn short_name(name: &str) -> Result<[u8; 11], String> {
    let (base, ext) = name.rsplit_once('.').unwrap_or((name, ""));
    let ok = |s: &str, n: usize| !s.is_empty() && s.len() <= n && s.bytes().all(|c| c.is_ascii_alphanumeric() || b"_-~!#$%&'()@^{}".contains(&c));
    if !ok(base, 8) || !(ext.is_empty() || ok(ext, 3)) { return Err(format!("{name}: not an 8.3 name")); }
    let mut out = [b' '; 11];
    for (i, c) in base.bytes().enumerate() { out[i] = c.to_ascii_uppercase(); }
    for (i, c) in ext.bytes().enumerate() { out[8 + i] = c.to_ascii_uppercase(); }
    Ok(out)
}

fn display_name(raw: &[u8]) -> String {
    let base = String::from_utf8_lossy(&raw[..8]).trim_end().to_string();
    let ext = String::from_utf8_lossy(&raw[8..11]).trim_end().to_string();
    if ext.is_empty() { base } else { format!("{base}.{ext}") }
}

const ATTR_VOLUME_ID: u8 = 0x08;
const ATTR_DIRECTORY: u8 = 0x10;
const ATTR_ARCHIVE: u8 = 0x20;
const ATTR_LONG_NAME: u8 = 0x0f;
/// Where a long-name entry keeps its 13 UCS-2 characters.
const LFN_SLOTS: [usize; 13] = [1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30];

/// The checksum of a short name that its long-name entries carry.
pub fn lfn_checksum(short: &[u8; 11]) -> u8 { short.iter().fold(0u8, |s, &b| s.rotate_right(1).wrapping_add(b)) }

/// The name a short entry reads as, with Windows NT's lower-case flags (byte 12: 0x08 the base,
/// 0x10 the extension), which Linux and FatFs honour.
fn short_display(e: &[u8]) -> String {
    let (base, ext) = (String::from_utf8_lossy(&e[..8]).trim_end().to_string(), String::from_utf8_lossy(&e[8..11]).trim_end().to_string());
    let base = if e[12] & 0x08 != 0 { base.to_ascii_lowercase() } else { base };
    let ext = if e[12] & 0x10 != 0 { ext.to_ascii_lowercase() } else { ext };
    if ext.is_empty() { base } else { format!("{base}.{ext}") }
}

/// The short name a long one is stored under: upper case, characters a short name cannot hold
/// as `_`, and where anything was lost the base cut to fit `~N` with the first N not `taken`
/// (fatgen103, "Basis-Name Generation Algorithm").
fn basis_name(name: &str, taken: impl Fn(&[u8; 11]) -> bool) -> Result<[u8; 11], String> {
    let valid = |c: u8| c.is_ascii_alphanumeric() || b"$%'-_@~`!(){}^#&".contains(&c);
    let upper = name.to_ascii_uppercase();
    let trimmed = upper.trim_start_matches('.');
    let mut lossy = trimmed.len() != upper.len();
    let (base, ext) = trimmed.rsplit_once('.').unwrap_or((trimmed, ""));
    let mut conv = |s: &str, n: usize| -> Vec<u8> {
        let mut out = Vec::new();
        for c in s.bytes() {
            if c == b' ' || c == b'.' { lossy = true; continue; }
            let c = if valid(c) { c } else { lossy = true; b'_' };
            if out.len() < n { out.push(c) } else { lossy = true }
        }
        out
    };
    let (b, e) = (conv(base, 8), conv(ext, 3));
    if b.is_empty() { return Err(format!("{name}: no short name to make of it")); }
    let mut raw = [b' '; 11];
    raw[8..8 + e.len()].copy_from_slice(&e);
    if !lossy {
        raw[..b.len()].copy_from_slice(&b);
        if !taken(&raw) { return Ok(raw); }
    }
    for n in 1..1_000_000u32 {
        let tail = format!("~{n}");
        let keep = b.len().min(8 - tail.len());
        raw[..8].fill(b' ');
        raw[..keep].copy_from_slice(&b[..keep]);
        raw[keep..keep + tail.len()].copy_from_slice(tail.as_bytes());
        if !taken(&raw) { return Ok(raw); }
    }
    Err(format!("{name}: every short name is taken"))
}

/// A short entry of `attr` for `raw` at `cluster`, size 0, with the fixed time stamp.
fn short_entry(raw: &[u8; 11], attr: u8, cluster: u32) -> [u8; 32] {
    let mut e = [0u8; 32];
    e[..11].copy_from_slice(raw);
    e[11] = attr;
    // a fixed time stamp, 2024-01-01 00:00, so the image is the same on every run
    let date = ((2024 - 1980) << 9 | 1 << 5 | 1) as u16;
    for at in [16, 18, 24] { e[at..at + 2].copy_from_slice(&date.to_le_bytes()); }
    e[20..22].copy_from_slice(&((cluster >> 16) as u16).to_le_bytes());
    e[26..28].copy_from_slice(&(cluster as u16).to_le_bytes());
    e
}

/// The long-name entries for `name` stored under `short`, in the order they go in the directory:
/// the last part first with 0x40 in its ordinal, the name ended by 0x0000 and padded with 0xFFFF.
fn long_entries(name: &str, short: &[u8; 11]) -> Result<Vec<[u8; 32]>, String> {
    let units: Vec<u16> = name.encode_utf16().collect();
    if units.is_empty() || units.len() > 255 { return Err(format!("{name}: a long name holds 1 to 255 characters")); }
    let parts = units.len().div_ceil(13);
    let sum = lfn_checksum(short);
    Ok((1..=parts).rev().map(|ord| {
        let mut e = [0u8; 32];
        e[0] = ord as u8 | if ord == parts { 0x40 } else { 0 };
        e[11] = ATTR_LONG_NAME;
        e[13] = sum;
        for (i, &at) in LFN_SLOTS.iter().enumerate() {
            let k = (ord - 1) * 13 + i;
            let u = match k.cmp(&units.len()) { std::cmp::Ordering::Less => units[k], std::cmp::Ordering::Equal => 0, _ => 0xffff };
            e[at..at + 2].copy_from_slice(&u.to_le_bytes());
        }
        e
    }).collect())
}

/// Where a directory entry sits: its sector and its slot of 16 in it.
type Slot = (u64, usize);

/// A directory entry as `Fat32::dir_entries` finds it, with its short name.
#[derive(Clone, Debug)]
struct Found { entry: DirEntry, short: [u8; 11] }

fn components(path: &str) -> Vec<&str> { path.split('/').filter(|c| !c.is_empty()).collect() }

impl Fat32 {
    pub fn mount(dev: &mut dyn BlockDevice) -> Result<Self, String> {
        if dev.block_size() != 512 { return Err(format!("{}-byte blocks: only 512 is supported", dev.block_size())); }
        let mut s = [0u8; 512];
        dev.read_meta(0, &mut s)?;
        // An MBR's partition entries at 446; a FAT32 volume boot record starts with a jump.
        let mut volume = 0u64;
        if s[510..512] == [0x55, 0xaa] && !(s[0] == 0xeb || s[0] == 0xe9) {
            let p = (0..4).map(|i| &s[446 + 16 * i..462 + 16 * i]).find(|e| matches!(e[4], 0x0b | 0x0c)).ok_or("no FAT32 partition in the MBR")?;
            volume = get32(p, 8) as u64;
            dev.read_meta(volume, &mut s)?;
        }
        if get16(&s, 11) != 512 || s[510..512] != [0x55, 0xaa] { return Err("no FAT32 boot sector".into()); }
        let (spc, reserved, fats) = (s[13] as u32, get16(&s, 14) as u32, s[16] as u32);
        let fat_sectors = get32(&s, 36);
        if get16(&s, 22) != 0 || fat_sectors == 0 || spc == 0 { return Err("not FAT32 (BPB_FATSz16 set)".into()); }
        let total = if get16(&s, 19) != 0 { get16(&s, 19) as u32 } else { get32(&s, 32) };
        let data_lba = volume + reserved as u64 + (fats * fat_sectors) as u64;
        let clusters = (total - reserved - fats * fat_sectors) / spc;
        let mut fs = Fat32 { volume_lba: volume, sectors_per_cluster: spc, reserved, fats, fat_sectors, root_cluster: get32(&s, 44), fsinfo: get16(&s, 48) as u32,
                             clusters, data_lba, label: String::from_utf8_lossy(&s[71..82]).trim_end().to_string(), allocated: 0, hint: 2, freed: 0 };
        let mut info = [0u8; 512];
        dev.read_meta(volume + fs.fsinfo as u64, &mut info)?;
        if get32(&info, 0) == 0x4161_5252 && get32(&info, 484) == 0x6141_7272 {
            let hint = get32(&info, 492);
            if (2..clusters + 2).contains(&hint) { fs.hint = hint; }
        }
        Ok(fs)
    }

    pub fn cluster_bytes(&self) -> usize { self.sectors_per_cluster as usize * 512 }
    /// The cluster a data-area sector belongs to.
    pub fn cluster_of_lba(&self, lba: u64) -> Option<u32> { (lba >= self.data_lba).then(|| ((lba - self.data_lba) / self.sectors_per_cluster as u64) as u32 + 2).filter(|&c| c < self.clusters + 2) }
    /// The sector of the first FAT a sector is, by its number in that FAT.
    pub fn fat_sector_of_lba(&self, lba: u64) -> Option<u64> { let fat = self.volume_lba + self.reserved as u64; (fat..fat + self.fat_sectors as u64).contains(&lba).then(|| lba - fat) }
    fn cluster_lba(&self, c: u32) -> u64 { self.data_lba + (c as u64 - 2) * self.sectors_per_cluster as u64 }
    fn fat_lba(&self, copy: u32, c: u32) -> u64 { self.volume_lba + self.reserved as u64 + (copy * self.fat_sectors) as u64 + (c / 128) as u64 }

    /// The cluster chain from `first`, each FAT sector read once while the chain stays in it.
    pub fn chain(&self, dev: &mut dyn BlockDevice, first: u32) -> Result<Vec<u32>, String> {
        let mut out = Vec::new();
        let (mut s, mut loaded) = ([0u8; 512], u32::MAX);
        let mut c = first;
        while (2..0x0fff_fff8).contains(&c) {
            if out.len() > self.clusters as usize { return Err("a cluster chain loops".into()); }
            out.push(c);
            if c / 128 != loaded { dev.read_meta(self.fat_lba(0, c), &mut s)?; loaded = c / 128; }
            c = get32(&s, (c % 128) as usize * 4) & 0x0fff_ffff;
        }
        Ok(out)
    }

    /// Set FAT entries, both copies, one read-modify-write per FAT sector.
    fn write_fat(&self, dev: &mut dyn BlockDevice, entries: &[(u32, u32)]) -> Result<(), String> {
        let mut sectors: Vec<u32> = entries.iter().map(|&(c, _)| c / 128).collect();
        sectors.sort_unstable();
        sectors.dedup();
        for sec in sectors {
            let mut s = [0u8; 512];
            dev.read_meta(self.fat_lba(0, sec * 128), &mut s)?;
            for &(c, v) in entries.iter().filter(|(c, _)| c / 128 == sec) {
                let at = (c % 128) as usize * 4;
                let old = get32(&s, at);
                put32(&mut s, at, (old & 0xf000_0000) | (v & 0x0fff_ffff));
            }
            for copy in 0..self.fats { dev.write_meta(self.fat_lba(copy, sec * 128), &s)?; }
        }
        Ok(())
    }

    /// `n` free clusters from the hint on, read a FAT sector at a time.
    fn allocate(&mut self, dev: &mut dyn BlockDevice, n: usize) -> Result<Vec<u32>, String> {
        let mut out = Vec::with_capacity(n);
        let mut c = self.hint;
        let mut seen = 0u32;
        let mut s = [0u8; 512];
        let mut loaded = u32::MAX;
        while out.len() < n {
            if seen >= self.clusters { return Err("the volume is full".into()); }
            if c >= self.clusters + 2 { c = 2; }
            if c / 128 != loaded { dev.read_meta(self.fat_lba(0, c), &mut s)?; loaded = c / 128; }
            if get32(&s, (c % 128) as usize * 4) & 0x0fff_ffff == FREE { out.push(c); }
            c += 1;
            seen += 1;
        }
        if let Some(&l) = out.last() { self.hint = l; }
        self.allocated += n as u32;
        Ok(out)
    }

    fn root_sectors(&self, dev: &mut dyn BlockDevice) -> Result<Vec<u64>, String> {
        let spc = self.sectors_per_cluster as u64;
        Ok(self.chain(dev, self.root_cluster)?.into_iter().flat_map(|c| { let l = self.cluster_lba(c); (0..spc).map(move |i| l + i) }).collect())
    }

    pub fn list_root(&self, dev: &mut dyn BlockDevice) -> Result<Vec<DirEntry>, String> {
        let mut out = Vec::new();
        for lba in self.root_sectors(dev)? {
            let mut s = [0u8; 512];
            dev.read_meta(lba, &mut s)?;
            for e in s.as_chunks::<32>().0 {
                if e[0] == 0 { return Ok(out); }
                if e[0] == 0xe5 || e[11] & 0x0f == 0x0f { continue; }
                out.push(DirEntry { name: display_name(&e[..11]), size: get32(e, 28), cluster: (get16(e, 20) as u32) << 16 | get16(e, 26) as u32, attr: e[11] });
            }
        }
        Ok(out)
    }

    pub fn read_file(&self, dev: &mut dyn BlockDevice, name: &str) -> Result<Vec<u8>, String> {
        let want = display_name(&short_name(name)?);
        let e = self.list_root(dev)?.into_iter().find(|e| e.name == want && e.attr & 0x18 == 0).ok_or_else(|| format!("{name}: no such file"))?;
        let chain = self.chain(dev, e.cluster)?;
        let mut out = Vec::with_capacity(e.size as usize);
        // contiguous clusters in one read
        let mut i = 0;
        while i < chain.len() && out.len() < e.size as usize {
            let mut j = i + 1;
            while j < chain.len() && chain[j] == chain[j - 1] + 1 && (j - i + 1) * self.cluster_bytes() <= 128 << 10 { j += 1; }
            let mut buf = vec![0u8; (j - i) * self.cluster_bytes()];
            dev.read_blocks(self.cluster_lba(chain[i]), &mut buf)?;
            out.extend_from_slice(&buf);
            i = j;
        }
        if out.len() < e.size as usize { return Err(format!("{name}: the chain holds {} of {} bytes", out.len(), e.size)); }
        out.truncate(e.size as usize);
        Ok(out)
    }

    /// Start a file: its directory entry (size 0, no cluster) in the first free slot of the root
    /// directory, written at once, as `open(O_CREAT)` leaves it.
    pub fn create(&mut self, dev: &mut dyn BlockDevice, name: &str) -> Result<FileWriter, String> {
        let raw = short_name(name)?;
        if self.list_root(dev)?.iter().any(|e| e.name == display_name(&raw)) { return Err(format!("{name}: exists")); }
        for lba in self.root_sectors(dev)? {
            let mut s = [0u8; 512];
            dev.read_meta(lba, &mut s)?;
            if let Some(slot) = s.as_chunks::<32>().0.iter().position(|e| e[0] == 0 || e[0] == 0xe5) {
                let e = &mut s[slot * 32..slot * 32 + 32];
                e.fill(0);
                e[..11].copy_from_slice(&raw);
                e[11] = 0x20;                                    // ATTR_ARCHIVE
                // a fixed time stamp, 2024-01-01 00:00, so the image is the same on every run
                let date = ((2024 - 1980) << 9 | 1 << 5 | 1) as u16;
                for at in [16, 18, 24] { e[at..at + 2].copy_from_slice(&date.to_le_bytes()); }
                dev.write_meta(lba, &s)?;
                return Ok(FileWriter::new(lba, slot));
            }
        }
        Err("the root directory is full (it is not extended)".into())
    }

    /// FSInfo as it stands on the volume, less what this host allocated since it last wrote it
    /// (an unknown count stays unknown), and the last cluster allocated as the hint: the device
    /// may have changed the volume since the mount.
    pub fn write_fsinfo(&mut self, dev: &mut dyn BlockDevice) -> Result<(), String> {
        let lba = self.volume_lba + self.fsinfo as u64;
        let mut s = [0u8; 512];
        dev.read_meta(lba, &mut s)?;
        let free = get32(&s, 488);
        if free != u32::MAX { put32(&mut s, 488, free.saturating_sub(self.allocated).saturating_add(self.freed).min(self.clusters)); }
        put32(&mut s, 492, self.hint);
        self.allocated = 0;
        self.freed = 0;
        dev.write_meta(lba, &s)
    }

    fn dir_sectors(&self, dev: &mut dyn BlockDevice, cluster: u32) -> Result<Vec<u64>, String> {
        let spc = self.sectors_per_cluster as u64;
        Ok(self.chain(dev, cluster)?.into_iter().flat_map(|c| { let l = self.cluster_lba(c); (0..spc).map(move |i| l + i) }).collect())
    }

    /// The entries of the directory at `cluster` but `.`, `..` and the volume label, each under
    /// its long name where a long name with the short entry's checksum comes right before it.
    fn dir_entries(&self, dev: &mut dyn BlockDevice, cluster: u32) -> Result<Vec<Found>, String> {
        struct Long { units: Vec<u16>, sum: u8, next: usize }
        let mut out = Vec::new();
        let mut long: Option<Long> = None;
        for lba in self.dir_sectors(dev, cluster)? {
            let mut s = [0u8; 512];
            dev.read_meta(lba, &mut s)?;
            for e in s.as_chunks::<32>().0 {
                if e[0] == 0 { return Ok(out); }
                if e[0] == 0xe5 { long = None; continue; }
                if e[11] & 0x3f == ATTR_LONG_NAME {
                    let ord = (e[0] & 0x3f) as usize;
                    if e[0] & 0x40 != 0 { long = Some(Long { units: vec![0xffff; ord * 13], sum: e[13], next: ord }); }
                    long = long.take().filter(|l| ord >= 1 && ord == l.next && e[13] == l.sum).map(|mut l| {
                        for (i, &at) in LFN_SLOTS.iter().enumerate() { l.units[(ord - 1) * 13 + i] = get16(e, at); }
                        l.next = ord - 1;
                        l
                    });
                    continue;
                }
                let short: [u8; 11] = e[..11].try_into().unwrap();
                let name = match long.take() {
                    Some(l) if l.next == 0 && l.sum == lfn_checksum(&short) => {
                        let end = l.units.iter().position(|&u| u == 0 || u == 0xffff).unwrap_or(l.units.len());
                        String::from_utf16_lossy(&l.units[..end])
                    }
                    _ => short_display(e),
                };
                if e[11] & ATTR_VOLUME_ID != 0 || short[0] == b'.' { continue; }
                let entry = DirEntry { name, size: get32(e, 28), cluster: (get16(e, 20) as u32) << 16 | get16(e, 26) as u32, attr: e[11] };
                out.push(Found { entry, short });
            }
        }
        Ok(out)
    }

    fn find(&self, dev: &mut dyn BlockDevice, dir: u32, name: &str) -> Result<Option<Found>, String> {
        Ok(self.dir_entries(dev, dir)?.into_iter().find(|f| f.entry.name.eq_ignore_ascii_case(name)))
    }

    /// The first cluster of the directory the components name, from the root.
    fn dir_of(&self, dev: &mut dyn BlockDevice, comps: &[&str]) -> Result<u32, String> {
        let mut dir = self.root_cluster;
        for (i, c) in comps.iter().enumerate() {
            let f = self.find(dev, dir, c)?.ok_or_else(|| format!("{}: no such directory", comps[..=i].join("/")))?;
            if f.entry.attr & ATTR_DIRECTORY == 0 { return Err(format!("{}: not a directory", comps[..=i].join("/"))); }
            dir = if f.entry.cluster == 0 { self.root_cluster } else { f.entry.cluster };
        }
        Ok(dir)
    }

    /// The entries of the directory at `path` (`/`-separated from the root; "" is the root).
    pub fn list(&self, dev: &mut dyn BlockDevice, path: &str) -> Result<Vec<DirEntry>, String> {
        let dir = self.dir_of(dev, &components(path))?;
        Ok(self.dir_entries(dev, dir)?.into_iter().map(|f| f.entry).collect())
    }

    /// The entry at `path`, if there is one.
    pub fn stat(&self, dev: &mut dyn BlockDevice, path: &str) -> Result<Option<DirEntry>, String> {
        let comps = components(path);
        let Some((name, parents)) = comps.split_last() else { return Err("the root has no entry".into()) };
        let dir = self.dir_of(dev, parents)?;
        Ok(self.find(dev, dir, name)?.map(|f| f.entry))
    }

    /// The contents of the file at `path`.
    pub fn read_path(&self, dev: &mut dyn BlockDevice, path: &str) -> Result<Vec<u8>, String> {
        let e = self.stat(dev, path)?.filter(|e| e.attr & (ATTR_DIRECTORY | ATTR_VOLUME_ID) == 0).ok_or_else(|| format!("{path}: no such file"))?;
        let chain = if e.size == 0 { Vec::new() } else { self.chain(dev, e.cluster)? };
        let mut out = Vec::with_capacity(e.size as usize);
        let mut i = 0;
        while i < chain.len() && out.len() < e.size as usize {
            let mut j = i + 1;
            while j < chain.len() && chain[j] == chain[j - 1] + 1 && (j - i + 1) * self.cluster_bytes() <= 128 << 10 { j += 1; }
            let mut buf = vec![0u8; (j - i) * self.cluster_bytes()];
            dev.read_blocks(self.cluster_lba(chain[i]), &mut buf)?;
            out.extend_from_slice(&buf);
            i = j;
        }
        if out.len() < e.size as usize { return Err(format!("{path}: the chain holds {} of {} bytes", out.len(), e.size)); }
        out.truncate(e.size as usize);
        Ok(out)
    }

    /// The entries that store `name` in the directory at `dir`: long-name entries unless it is an
    /// upper-case 8.3 name, then the short entry.
    fn name_entries(&self, dev: &mut dyn BlockDevice, dir: u32, name: &str, attr: u8, cluster: u32) -> Result<Vec<[u8; 32]>, String> {
        if let Some(raw) = short_name(name).ok().filter(|_| name == name.to_ascii_uppercase()) { return Ok(vec![short_entry(&raw, attr, cluster)]); }
        let shorts: Vec<[u8; 11]> = self.dir_entries(dev, dir)?.into_iter().map(|f| f.short).collect();
        let raw = basis_name(name, |r| shorts.contains(r))?;
        let mut entries = long_entries(name, &raw)?;
        entries.push(short_entry(&raw, attr, cluster));
        Ok(entries)
    }

    /// Write `entries` into the first run of free slots of the directory at `dir` that holds them
    /// all, a sector at a time; where the last one, the short entry, went.
    fn put_entries(&self, dev: &mut dyn BlockDevice, dir: u32, entries: &[[u8; 32]]) -> Result<(u64, usize), String> {
        let sectors = self.dir_sectors(dev, dir)?;
        let mut cache: Vec<(u64, [u8; 512])> = Vec::new();
        let (mut start, mut run, mut ended) = (0, 0, false);
        'scan: for (i, &lba) in sectors.iter().enumerate() {
            let mut s = [0u8; 512];
            dev.read_meta(lba, &mut s)?;
            cache.push((lba, s));
            for slot in 0..16 {
                let first = s[slot * 32];
                ended |= first == 0;
                if ended || first == 0xe5 {
                    if run == 0 { start = i * 16 + slot; }
                    run += 1;
                    if run == entries.len() { break 'scan; }
                } else { run = 0; }
            }
        }
        if run < entries.len() { return Err("the directory is full (it is not extended)".into()); }
        for (k, e) in entries.iter().enumerate() {
            let at = start + k;
            cache[at / 16].1[at % 16 * 32..at % 16 * 32 + 32].copy_from_slice(e);
        }
        let (first, last) = (start / 16, (start + entries.len() - 1) / 16);
        for (lba, s) in &cache[first..=last] { dev.write_meta(*lba, s)?; }
        let end = start + entries.len() - 1;
        Ok((sectors[end / 16], end % 16))
    }

    /// Make the directories of `path` that are missing, as `mkdir -p` does; the first cluster of
    /// the last one. A new directory is one cluster: its FAT entry first, then the cluster with
    /// `.` and `..`, then its entry in the parent, then FSInfo.
    pub fn mkdir_all(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<u32, String> {
        let mut dir = self.root_cluster;
        for c in components(path) {
            if let Some(f) = self.find(dev, dir, c)? {
                if f.entry.attr & ATTR_DIRECTORY == 0 { return Err(format!("{c}: exists and is not a directory")); }
                dir = f.entry.cluster;
                continue;
            }
            let new = self.allocate(dev, 1)?[0];
            self.write_fat(dev, &[(new, EOC)])?;
            let mut buf = vec![0u8; self.cluster_bytes()];
            buf[..32].copy_from_slice(&short_entry(b".          ", ATTR_DIRECTORY, new));
            buf[32..64].copy_from_slice(&short_entry(b"..         ", ATTR_DIRECTORY, if dir == self.root_cluster { 0 } else { dir }));
            dev.write_meta(self.cluster_lba(new), &buf)?;
            let entries = self.name_entries(dev, dir, c, ATTR_DIRECTORY, new)?;
            self.put_entries(dev, dir, &entries)?;
            self.write_fsinfo(dev)?;
            dir = new;
        }
        Ok(dir)
    }

    /// Start a file at `path` in a directory that exists: its entries written at once, size 0 and
    /// no cluster, as `open(O_CREAT)` leaves them. An upper-case 8.3 name in the root is `create`.
    pub fn create_path(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<FileWriter, String> {
        let comps = components(path);
        let Some((name, parents)) = comps.split_last() else { return Err("a file needs a name".into()) };
        if parents.is_empty() && short_name(name).is_ok() && *name == name.to_ascii_uppercase() { return self.create(dev, name); }
        let dir = self.dir_of(dev, parents)?;
        if self.find(dev, dir, name)?.is_some() { return Err(format!("{path}: exists")); }
        let entries = self.name_entries(dev, dir, name, ATTR_ARCHIVE, 0)?;
        let (dir_lba, dir_slot) = self.put_entries(dev, dir, &entries)?;
        Ok(FileWriter::new(dir_lba, dir_slot))
    }

    /// The entry named `name` in the directory at `dir`, and where its long-name entries and its
    /// short entry sit, short entry last.
    fn slots_of(&self, dev: &mut dyn BlockDevice, dir: u32, name: &str) -> Result<Option<(Found, Vec<Slot>)>, String> {
        let Some(f) = self.find(dev, dir, name)? else { return Ok(None) };
        let sum = lfn_checksum(&f.short);
        let mut run: Vec<(u64, usize)> = Vec::new();
        for lba in self.dir_sectors(dev, dir)? {
            let mut s = [0u8; 512];
            dev.read_meta(lba, &mut s)?;
            for (slot, e) in s.as_chunks::<32>().0.iter().enumerate() {
                if e[0] == 0 { return Ok(None); }
                let live = e[0] != 0xe5;
                if live && e[11] & 0x3f == ATTR_LONG_NAME && e[13] == sum { run.push((lba, slot)); continue; }
                if live && e[..11] == f.short { run.push((lba, slot)); return Ok(Some((f, run))); }
                run.clear();
            }
        }
        Ok(None)
    }

    /// Delete the file at `path` as Linux's vfat unlinks one: its entries marked deleted (0xE5),
    /// then its chain freed in every FAT when the inode is evicted, then FSInfo (v6.12
    /// fs/fat/namei_vfat.c `vfat_unlink`, fs/fat/inode.c `fat_evict_inode`, fs/fat/fatent.c
    /// `fat_free_clusters`). Returns the clusters freed, which a later allocation reuses once it
    /// comes round to them.
    pub fn delete_path(&mut self, dev: &mut dyn BlockDevice, path: &str) -> Result<Vec<u32>, String> {
        let comps = components(path);
        let Some((name, parents)) = comps.split_last() else { return Err("a file needs a name".into()) };
        let dir = self.dir_of(dev, parents)?;
        let (f, slots) = self.slots_of(dev, dir, name)?.ok_or_else(|| format!("{path}: no such file"))?;
        if f.entry.attr & (ATTR_DIRECTORY | ATTR_VOLUME_ID) != 0 { return Err(format!("{path}: not a file")); }
        let mut sectors: Vec<u64> = slots.iter().map(|s| s.0).collect();
        sectors.dedup();
        for lba in sectors {
            let mut s = [0u8; 512];
            dev.read_meta(lba, &mut s)?;
            for &(_, slot) in slots.iter().filter(|x| x.0 == lba) { s[slot * 32] = 0xe5; }
            dev.write_meta(lba, &s)?;
        }
        let chain = if f.entry.cluster == 0 { Vec::new() } else { self.chain(dev, f.entry.cluster)? };
        let entries: Vec<(u32, u32)> = chain.iter().map(|&c| (c, FREE)).collect();
        self.write_fat(dev, &entries)?;
        self.freed += chain.len() as u32;
        self.write_fsinfo(dev)?;
        Ok(chain)
    }

    /// The volume's dirty flag: bit 0 of the FAT32 boot sector's byte 65 (BS_Reserved1), which
    /// Linux keeps as `state` (v6.12 include/uapi/linux/msdos_fs.h FAT_STATE_DIRTY). Linux sets it
    /// at a read-write mount and clears it at a clean unmount (fs/fat/inode.c `fat_set_state`,
    /// `fat_put_super`); a volume cut off without one is dirty at the next mount.
    pub fn dirty(&self, dev: &mut dyn BlockDevice) -> Result<bool, String> {
        let mut s = [0u8; 512];
        dev.read_meta(self.volume_lba, &mut s)?;
        Ok(s[65] & 1 != 0)
    }

    pub fn set_dirty(&self, dev: &mut dyn BlockDevice, dirty: bool) -> Result<(), String> {
        let mut s = [0u8; 512];
        dev.read_meta(self.volume_lba, &mut s)?;
        if (s[65] & 1 != 0) == dirty { return Ok(()); }
        s[65] = if dirty { s[65] | 1 } else { s[65] & !1 };
        dev.write_meta(self.volume_lba, &s)
    }

    /// Check and repair the volume the way dosfstools 4.2's `fsck.fat -a` does it (src/fat.c,
    /// src/check.c): both FATs read whole; every file's chain walked from its entry, a chain longer
    /// than the size cut at the size (check.c:528-537), a size past the chain cut to the chain
    /// (629-640), a chain into a free, bad or already owned cluster cut before it; clusters no
    /// entry reaches freed (`reclaim_free`, fat.c:420; without -f, which keeps them as FSCKnnnn.REC
    /// files); FSInfo's free count; the dirty flag cleared (check.c:1065-1080). How the FATs are
    /// written is `write`.
    pub fn fsck(&mut self, dev: &mut dyn BlockDevice, write: FsckWrite) -> Result<FsckReport, String> {
        let mut r = FsckReport::default();
        let fat_lba = self.volume_lba + self.reserved as u64;
        let bytes = self.fat_sectors as usize * 512;
        let read_fat = |dev: &mut dyn BlockDevice, copy: u32| -> Result<Vec<u8>, String> {
            let mut raw = vec![0u8; bytes];
            for (i, part) in raw.chunks_mut(2048 * 512).enumerate() { dev.read_meta(fat_lba + (copy * self.fat_sectors) as u64 + i as u64 * 2048, part)?; }
            Ok(raw)
        };
        let first = read_fat(dev, 0)?;
        let second = if self.fats > 1 { Some(read_fat(dev, 1)?) } else { None };
        r.fat_bytes_read = bytes as u64 * self.fats as u64;
        r.fats_differ = second.as_ref().is_some_and(|s| *s != first);
        let n = self.clusters as usize + 2;
        let mut fat: Vec<u32> = (0..n).map(|c| get32(&first, c * 4) & 0x0fff_ffff).collect();
        let mut owned = vec![false; n];
        let cb = self.cluster_bytes() as u64;
        // directories to visit: their first cluster
        let mut dirs = vec![self.root_cluster];
        while let Some(dir) = dirs.pop() {
            // the directory's own chain, cut where it leaves the volume or meets a cluster owned
            let mut chain = Vec::new();
            let mut c = dir;
            while (2..n as u32).contains(&c) && !owned[c as usize] { owned[c as usize] = true; chain.push(c); c = fat[c as usize]; if c >= 0x0fff_fff8 { break; } }
            for lba in chain.iter().flat_map(|&c| { let l = self.cluster_lba(c); (0..self.sectors_per_cluster as u64).map(move |i| l + i) }) {
                let mut s = [0u8; 512];
                dev.read_meta(lba, &mut s)?;
                let mut changed = false;
                for slot in 0..16 {
                    let e = &mut s[slot * 32..slot * 32 + 32];
                    if e[0] == 0 || e[0] == 0xe5 || e[11] & 0x3f == ATTR_LONG_NAME || e[11] & ATTR_VOLUME_ID != 0 || e[0] == b'.' { continue; }
                    let start = (get16(e, 20) as u32) << 16 | get16(e, 26) as u32;
                    let size = get32(e, 28) as u64;
                    let is_dir = e[11] & ATTR_DIRECTORY != 0;
                    let name = display_name(&e[..11]);
                    if is_dir { if (2..n as u32).contains(&start) { dirs.push(start); } continue; }
                    r.files += 1;
                    let want = size.div_ceil(cb);
                    let (mut prev, mut c, mut k) = (0u32, start, 0u64);
                    while c != 0 && c < 0x0fff_fff8 {
                        let bad = !(2..n as u32).contains(&c) || fat[c as usize] == FREE || owned[c as usize];
                        if bad || k >= want {
                            // cut before `c`: the chain ends at `prev`, or the file holds none
                            if prev == 0 { e[20..22].fill(0); e[26..28].fill(0); changed = true; } else { fat[prev as usize] = EOC; }
                            if bad { r.cross_or_broken.push(name.clone()); } else { r.truncated.push(name.clone()); }
                            break;
                        }
                        owned[c as usize] = true;
                        k += 1;
                        prev = c;
                        c = fat[c as usize];
                    }
                    if size > k * cb {
                        put32(e, 28, (k * cb) as u32);
                        r.resized.push(name.clone());
                        changed = true;
                    }
                }
                if changed { dev.write_meta(lba, &s)?; r.dir_sectors_written += 1; }
            }
        }
        for c in 2..n {
            if !owned[c] && fat[c] != FREE && fat[c] != 0x0fff_fff7 { fat[c] = FREE; r.lost_clusters += 1; }
        }
        let mut out = first.clone();
        for (c, &v) in fat.iter().enumerate().skip(2) { let old = get32(&out, c * 4); put32(&mut out, c * 4, (old & 0xf000_0000) | v); }
        let changed: Vec<usize> = (0..self.fat_sectors as usize).filter(|&i| out[i * 512..i * 512 + 512] != first[i * 512..i * 512 + 512]).collect();
        let write_copy = |dev: &mut dyn BlockDevice, copy: u32, data: &[u8]| -> Result<(), String> {
            for (i, part) in data.chunks(2048 * 512).enumerate() { dev.write_meta(fat_lba + (copy * self.fat_sectors) as u64 + i as u64 * 2048, part)?; }
            Ok(())
        };
        match write {
            FsckWrite::Bulk => {
                for copy in 0..self.fats { write_copy(dev, copy, &out)?; }
                r.fat_bytes_written = bytes as u64 * self.fats as u64;
            }
            FsckWrite::Changed => {
                for &i in &changed {
                    for copy in 0..self.fats { dev.write_meta(fat_lba + (copy * self.fat_sectors) as u64 + i as u64, &out[i * 512..i * 512 + 512])?; }
                }
                r.fat_bytes_written = changed.len() as u64 * 512 * self.fats as u64;
                // copies that differ: the second rewritten whole from the first (fat.c:160-186)
                if r.fats_differ {
                    for copy in 1..self.fats { write_copy(dev, copy, &out)?; }
                    r.fat_bytes_written += bytes as u64 * (self.fats as u64 - 1);
                }
            }
        }
        r.fat_sectors_changed = changed.len() as u64;
        let free = fat.iter().skip(2).filter(|&&v| v == FREE).count() as u32;
        let lba = self.volume_lba + self.fsinfo as u64;
        let mut s = [0u8; 512];
        dev.read_meta(lba, &mut s)?;
        if get32(&s, 0) == 0x4161_5252 && get32(&s, 484) == 0x6141_7272 && get32(&s, 488) != free { put32(&mut s, 488, free); dev.write_meta(lba, &s)?; }
        self.allocated = 0;
        self.freed = 0;
        r.dirty_cleared = self.dirty(dev)?;
        self.set_dirty(dev, false)?;
        Ok(r)
    }
}

/// How `Fat32::fsck` writes the FATs back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FsckWrite {
    /// Each FAT sector holding a changed entry, in every copy, and the second copy whole when the
    /// copies differed, as dosfstools 4.2 does (src/fat.c `set_fat`, 160-186).
    Changed,
    /// Every FAT whole, the first copy first: a bulk rewrite of FAT1 (the shared-reading plan's
    /// assumption for the car's fsck, not dosfstools' own pattern).
    Bulk,
}

/// What `Fat32::fsck` found and wrote.
#[derive(Clone, Debug, Default)]
pub struct FsckReport {
    pub files: usize,
    /// a chain longer than the size, cut at the size
    pub truncated: Vec<String>,
    /// a size past the chain, cut to the chain
    pub resized: Vec<String>,
    /// a chain into a free, bad or already owned cluster, cut before it
    pub cross_or_broken: Vec<String>,
    pub lost_clusters: u32,
    pub fats_differ: bool,
    pub fat_sectors_changed: u64,
    pub fat_bytes_read: u64,
    pub fat_bytes_written: u64,
    pub dir_sectors_written: u64,
    /// the volume was marked dirty
    pub dirty_cleared: bool,
}

impl FsckReport {
    pub fn summary(&self) -> String {
        format!("{} files: {} truncated to their size, {} sizes cut to their chain, {} chains broken or cross-linked, {} lost clusters freed; FATs {}; {} FAT sectors changed, {} KiB of FAT read and {} KiB written, {} directory sectors written; {}",
                self.files, self.truncated.len(), self.resized.len(), self.cross_or_broken.len(), self.lost_clusters, if self.fats_differ { "differed" } else { "agreed" },
                self.fat_sectors_changed, self.fat_bytes_read >> 10, self.fat_bytes_written >> 10, self.dir_sectors_written, if self.dirty_cleared { "dirty flag cleared" } else { "the volume was clean" })
    }
}

/// A file being written.
pub struct FileWriter {
    dir_lba: u64,
    dir_slot: usize,
    first: u32,
    last: u32,
    size: u64,
    /// FAT entries not written yet
    pending: Vec<(u32, u32)>,
    /// write the FAT after this many chunks
    pub fat_every: u32,
    chunks: u32,
    /// Linux's order: the entry's first cluster and size, and FSInfo, written again after every
    /// chunk, as inode writeback finds the inode and FSInfo dirty (v6.12 fs/fat/inode.c
    /// `__fat_write_inode`, fatent.c `mark_fsinfo_dirty`); a writeback cache keeps the latest
    /// and lands it late, so the medium sees intermediate sizes
    pub linux: bool,
}

impl FileWriter {
    fn new(dir_lba: u64, dir_slot: usize) -> Self {
        FileWriter { dir_lba, dir_slot, first: 0, last: 0, size: 0, pending: Vec::new(), fat_every: 1, chunks: 0, linux: false }
    }

    /// Bytes written so far.
    pub fn size(&self) -> u64 { self.size }
    /// The first cluster, 0 before the first chunk.
    pub fn first_cluster(&self) -> u32 { self.first }

    /// Append `data` (a whole number of clusters, except for the file's last chunk): clusters
    /// from the volume's free ones, the data in one write per contiguous run, then the FAT
    /// sectors for them every `fat_every` chunks.
    pub fn write(&mut self, fs: &mut Fat32, dev: &mut dyn BlockDevice, data: &[u8]) -> Result<(), String> {
        let cb = fs.cluster_bytes();
        if data.is_empty() { return Ok(()); }
        if !self.size.is_multiple_of(cb as u64) { return Err("a chunk after a partial cluster".into()); }
        let clusters = fs.allocate(dev, data.len().div_ceil(cb))?;
        for &c in &clusters {
            if self.first == 0 { self.first = c; } else { self.pending.push((self.last, c)); }
            self.last = c;
        }
        self.pending.push((self.last, EOC));
        let mut i = 0;
        while i < clusters.len() {
            let mut j = i + 1;
            while j < clusters.len() && clusters[j] == clusters[j - 1] + 1 { j += 1; }
            let (from, to) = (i * cb, (j * cb).min(data.len()));
            let mut buf = data[from..to].to_vec();
            buf.resize((j - i) * cb, 0);
            dev.write_blocks(fs.cluster_lba(clusters[i]), &buf)?;
            i = j;
        }
        self.size += data.len() as u64;
        self.chunks += 1;
        if self.chunks.is_multiple_of(self.fat_every.max(1)) { self.flush_fat(fs, dev)?; }
        if self.linux {
            self.write_entry(dev)?;
            fs.write_fsinfo(dev)?;
        }
        Ok(())
    }

    fn flush_fat(&mut self, fs: &Fat32, dev: &mut dyn BlockDevice) -> Result<(), String> {
        // the last entry for a cluster wins: an end-of-chain mark is replaced by its successor
        let mut entries: Vec<(u32, u32)> = Vec::new();
        for &(c, v) in &self.pending { if let Some(e) = entries.iter_mut().find(|e| e.0 == c) { e.1 = v; } else { entries.push((c, v)); } }
        fs.write_fat(dev, &entries)?;
        self.pending.clear();
        Ok(())
    }

    /// The directory entry's first cluster and size as they stand.
    fn write_entry(&self, dev: &mut dyn BlockDevice) -> Result<(), String> {
        let mut s = [0u8; 512];
        dev.read_meta(self.dir_lba, &mut s)?;
        let e = &mut s[self.dir_slot * 32..self.dir_slot * 32 + 32];
        e[20..22].copy_from_slice(&((self.first >> 16) as u16).to_le_bytes());
        e[26..28].copy_from_slice(&(self.first as u16).to_le_bytes());
        put32(e, 28, self.size as u32);
        dev.write_meta(self.dir_lba, &s)
    }

    /// The FAT left, the directory entry's first cluster and size, FSInfo.
    pub fn close(mut self, fs: &mut Fat32, dev: &mut dyn BlockDevice) -> Result<u64, String> {
        self.flush_fat(fs, dev)?;
        self.write_entry(dev)?;
        fs.write_fsinfo(dev)?;
        Ok(self.size)
    }
}

/// The bytes of a test clip: a xorshift stream seeded from the name, so a writer and a checker
/// make the same file without keeping it.
pub fn clip_bytes(name: &str, len: usize) -> Vec<u8> {
    let mut x: u64 = name.bytes().fold(0xcbf2_9ce4_8422_2325, |h, b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3)) | 1;
    let mut out = Vec::with_capacity(len + 8);
    while out.len() < len {
        x ^= x << 13; x ^= x >> 7; x ^= x << 17;
        out.extend_from_slice(&x.to_le_bytes());
    }
    out.truncate(len);
    out
}

/// A clip of `len` bytes as a recorder that writes its index last leaves it: an MP4 file whose
/// top-level boxes are `ftyp`, an `mdat` holding `clip_bytes(name, ..)`, and a `moov` (ISO/IEC
/// 14496-12, "Object Structure"), which is what a reader checks for a finished clip. `len` is at
/// least 48.
pub fn mp4_bytes(name: &str, len: usize) -> Vec<u8> {
    const FTYP: [u8; 24] = *b"\0\0\0\x18ftypisom\0\0\x02\0isommp41";
    const MOOV: [u8; 16] = *b"\0\0\0\x10moov\0\0\0\x08mvhd";
    let mdat = len.max(48) - FTYP.len() - MOOV.len();
    let mut out = Vec::with_capacity(len.max(48));
    out.extend_from_slice(&FTYP);
    out.extend_from_slice(&(mdat as u32).to_be_bytes());
    out.extend_from_slice(b"mdat");
    out.extend_from_slice(&clip_bytes(name, mdat - 8));
    out.extend_from_slice(&MOOV);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_names_for_long_ones_follow_the_basis_name_rules() {
        let none = |_: &[u8; 11]| false;
        assert_eq!(&basis_name("event.mp4", none).unwrap(), b"EVENT   MP4", "fits, so no tail");
        assert_eq!(&basis_name("2026-01-01_00-00-00", none).unwrap(), b"2026-0~1   ");
        assert_eq!(&basis_name("TeslaCam", none).unwrap(), b"TESLACAM   ");
        assert_eq!(&basis_name("a+b.long-ext", none).unwrap(), b"A_B~1   LON", "an invalid character and a long extension");
        assert_eq!(&basis_name(".hidden", none).unwrap(), b"HIDDEN~1   ", "a leading period is dropped, which is a loss");
        let taken = [*b"2026-0~1   ", *b"2026-0~2   "];
        assert_eq!(&basis_name("2026-01-01_00-02-00", |r| taken.contains(r)).unwrap(), b"2026-0~3   ");
        assert_eq!(&basis_name("event.mp4", |r| r == b"EVENT   MP4").unwrap(), b"EVENT~1 MP4");
    }

    /// The checksum is fatgen103's rotate-right-and-add over the 11 bytes; a long name of 19
    /// characters takes two entries, the second-numbered first with 0x40, ended by 0x0000 and
    /// padded with 0xFFFF.
    #[test]
    fn long_entries_carry_the_name_in_ucs2_and_the_short_name_checksum() {
        assert_eq!(lfn_checksum(b"           "), (0..11).fold(0u8, |s, _| s.rotate_right(1).wrapping_add(b' ')));
        assert_ne!(lfn_checksum(b"2026-0~1   "), lfn_checksum(b"2026-0~2   "));
        let short = *b"2026-0~1   ";
        let e = long_entries("2026-01-01_00-00-00", &short).unwrap();
        assert_eq!(e.len(), 2);
        assert_eq!((e[0][0], e[1][0]), (0x42, 0x01));
        for x in &e { assert_eq!((x[11], x[12], x[13], x[26], x[27]), (ATTR_LONG_NAME, 0, lfn_checksum(&short), 0, 0)); }
        let units = |x: &[u8; 32]| LFN_SLOTS.iter().map(|&at| get16(x, at)).collect::<Vec<u16>>();
        let name: Vec<u16> = "2026-01-01_00-00-00".encode_utf16().collect();
        assert_eq!(units(&e[1]), name[..13]);
        let mut tail = name[13..].to_vec();
        tail.push(0);
        tail.resize(13, 0xffff);
        assert_eq!(units(&e[0]), tail);
        assert_eq!(long_entries("x".repeat(13).as_str(), &short).unwrap().len(), 1, "exactly 13: no terminator needed");
        assert!(long_entries("", &short).is_err());
    }
}
