//! FAT32 SD card images for the tests, built by hand so that they are the same bytes on every
//! host: an MBR with one FAT32 (LBA) partition from LBA 2048, the way card formatters lay a card
//! out, an empty root directory, and a reader for the files the firmware leaves in it (8.3
//! names, as IDF's FatFs writes them without long file names). Field offsets: Microsoft's FAT32
//! File System Specification 1.03 (fatgen103), "Boot Sector and BPB", "FAT32 FSInfo Sector
//! Structure", "FAT Directory Structure".
#![allow(dead_code)]
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::Path;

const SECTOR: u64 = 512;
pub const PARTITION_LBA: u64 = 2048;
const RESERVED: u32 = 32;

fn put16(b: &mut [u8], at: usize, v: u16) { b[at..at + 2].copy_from_slice(&v.to_le_bytes()); }
fn put32(b: &mut [u8], at: usize, v: u32) { b[at..at + 4].copy_from_slice(&v.to_le_bytes()); }
fn get16(b: &[u8], at: usize) -> u16 { u16::from_le_bytes(b[at..at + 2].try_into().unwrap()) }
fn get32(b: &[u8], at: usize) -> u32 { u32::from_le_bytes(b[at..at + 4].try_into().unwrap()) }

/// Write a `size`-byte image (sparse where it is zero) with a FAT32 volume of `cluster`-byte
/// clusters. FAT32 needs at least 65525 clusters, so `size` must allow that.
pub fn fat32(path: &Path, size: u64, cluster: u32, label: &str) -> std::io::Result<()> {
    let mut f = std::fs::File::create(path)?;
    f.set_len(size)?;
    let total = (size / SECTOR - PARTITION_LBA) as u32;
    let spc = cluster / SECTOR as u32;
    // FAT size: fatgen103, "FAT Type Determination" (TmpVal1 / TmpVal2, rounded up).
    let fat = (total - RESERVED).div_ceil((256 * spc + 2) / 2);
    let clusters = (total - RESERVED - 2 * fat) / spc;
    assert!(clusters >= 65525, "{clusters} clusters is not FAT32");
    let mut mbr = [0u8; 512];
    put32(&mut mbr, 440, 0x5d1c_0b17);                          // disk signature
    mbr[446..462].copy_from_slice(&[0, 0xfe, 0xff, 0xff, 0x0c, 0xfe, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0]);
    put32(&mut mbr, 454, PARTITION_LBA as u32);
    put32(&mut mbr, 458, total);
    mbr[510] = 0x55; mbr[511] = 0xaa;
    let mut vbr = [0u8; 512];
    vbr[..3].copy_from_slice(&[0xeb, 0x58, 0x90]);
    vbr[3..11].copy_from_slice(b"ESP32SIM");
    put16(&mut vbr, 11, 512);
    vbr[13] = spc as u8;
    put16(&mut vbr, 14, RESERVED as u16);
    vbr[16] = 2;                                                  // FATs
    vbr[21] = 0xf8;                                               // fixed media
    put16(&mut vbr, 24, 63); put16(&mut vbr, 26, 255);
    put32(&mut vbr, 28, PARTITION_LBA as u32);                   // hidden sectors
    put32(&mut vbr, 32, total);
    put32(&mut vbr, 36, fat);
    put32(&mut vbr, 44, 2);                                       // root directory cluster
    put16(&mut vbr, 48, 1); put16(&mut vbr, 50, 6);               // FSInfo, backup boot sector
    vbr[64] = 0x80; vbr[66] = 0x29;
    put32(&mut vbr, 67, 0x5d1c_0b17);                             // volume ID
    let mut l = [b' '; 11];
    l[..label.len().min(11)].copy_from_slice(&label.as_bytes()[..label.len().min(11)]);
    vbr[71..82].copy_from_slice(&l);
    vbr[82..90].copy_from_slice(b"FAT32   ");
    vbr[510] = 0x55; vbr[511] = 0xaa;
    let mut info = [0u8; 512];
    put32(&mut info, 0, 0x4161_5252); put32(&mut info, 484, 0x6141_7272);
    put32(&mut info, 488, clusters - 1);                          // free: all but the root's
    put32(&mut info, 492, 3);                                     // next free
    put32(&mut info, 508, 0xaa55_0000);
    let mut fat0 = [0u8; 512];
    put32(&mut fat0, 0, 0x0fff_fff8); put32(&mut fat0, 4, 0x0fff_ffff); put32(&mut fat0, 8, 0x0fff_ffff);
    let at = |s: u64| (PARTITION_LBA + s) * SECTOR;
    let mut put = |off: u64, b: &[u8]| -> std::io::Result<()> { f.seek(SeekFrom::Start(off))?; f.write_all(b) };
    put(0, &mbr)?;
    for base in [0, 6] { put(at(base), &vbr)?; put(at(base + 1), &info)?; }
    for n in 0..2 { put(at(RESERVED as u64 + n * fat as u64), &fat0)?; }
    if !label.is_empty() {
        let mut entry = [0u8; 32];
        entry[..11].copy_from_slice(&l);
        entry[11] = 0x08;                                         // volume label
        put(at((RESERVED + 2 * fat) as u64), &entry)?;
    }
    Ok(())
}

/// The volume of an image `fat32` wrote, as far as reading files from its root needs.
pub struct Volume { file: std::fs::File, spc: u64, fat: u64, data: u64 }
impl Volume {
    pub fn open(path: &Path) -> std::io::Result<Volume> {
        let mut file = std::fs::File::open(path)?;
        let mut mbr = [0u8; 512];
        file.read_exact(&mut mbr)?;
        let part = get32(&mbr, 454) as u64;
        let mut vbr = [0u8; 512];
        file.seek(SeekFrom::Start(part * SECTOR))?;
        file.read_exact(&mut vbr)?;
        let (spc, reserved, fats, fat_size) = (vbr[13] as u64, get16(&vbr, 14) as u64, vbr[16] as u64, get32(&vbr, 36) as u64);
        Ok(Volume { file, spc, fat: part + reserved, data: part + reserved + fats * fat_size })
    }
    fn cluster(&mut self, n: u32) -> Vec<u8> {
        let mut b = vec![0u8; (self.spc * SECTOR) as usize];
        self.file.seek(SeekFrom::Start((self.data + (n as u64 - 2) * self.spc) * SECTOR)).unwrap();
        self.file.read_exact(&mut b).unwrap();
        b
    }
    fn next(&mut self, n: u32) -> u32 {
        let mut e = [0u8; 4];
        self.file.seek(SeekFrom::Start(self.fat * SECTOR + 4 * n as u64)).unwrap();
        self.file.read_exact(&mut e).unwrap();
        u32::from_le_bytes(e) & 0x0fff_ffff
    }
    fn chain(&mut self, mut n: u32) -> Vec<u8> {
        let mut out = Vec::new();
        while (2..0x0fff_fff8).contains(&n) { out.extend(self.cluster(n)); n = self.next(n); }
        out
    }
    /// The files in the root directory: (8.3 name as "NAME.EXT", contents).
    pub fn root_files(&mut self) -> Vec<(String, Vec<u8>)> {
        let dir = self.chain(2);
        let mut files = Vec::new();
        for e in dir.chunks(32) {
            if e[0] == 0 { break; }
            if e[0] == 0xe5 || e[11] & 0x08 != 0 || e[11] == 0x0f { continue; }
            let name = String::from_utf8_lossy(&e[..8]).trim_end().to_string();
            let ext = String::from_utf8_lossy(&e[8..11]).trim_end().to_string();
            let first = (get16(e, 20) as u32) << 16 | get16(e, 26) as u32;
            let mut data = self.chain(first);
            data.truncate(get32(e, 28) as usize);
            files.push((if ext.is_empty() { name } else { format!("{name}.{ext}") }, data));
        }
        files
    }
}

/// Check the volume of a card image with the system's checker for its filesystem, without
/// changing it: exFAT (the partition's boot sector names it) goes to `fsck_exfat`, anything else to
/// the FAT checker, the partition from LBA 2048 copied out (sparse) and `FSCK_FAT` (a command),
/// else macOS's `/sbin/fsck_msdos -n` or Linux's `fsck.fat -n`, run on the copy. Its output, or
/// why it failed.
pub fn fsck(image: &Path) -> Result<String, String> {
    if is_exfat(image) { return fsck_exfat(image); }
    let tool = std::env::var("FSCK_FAT").ok()
        .or_else(|| ["/sbin/fsck_msdos", "/usr/sbin/fsck.fat", "/sbin/fsck.fat", "/usr/sbin/fsck.vfat"].iter().find(|p| Path::new(p).exists()).map(|p| p.to_string()))
        .ok_or("no FAT checker: set FSCK_FAT or install fsck_msdos (macOS) or dosfstools (Linux)")?;
    check_volume_copy(image, &tool)
}

/// `tool -n` on the partition from LBA 2048, copied out of `image` (sparse).
fn check_volume_copy(image: &Path, tool: &str) -> Result<String, String> {
    let mut src = std::fs::File::open(image).map_err(|e| format!("{}: {e}", image.display()))?;
    let vol = image.with_extension("volume");
    let mut dst = std::fs::File::create(&vol).map_err(|e| format!("{}: {e}", vol.display()))?;
    let len = src.metadata().unwrap().len() - PARTITION_LBA * SECTOR;
    dst.set_len(len).unwrap();
    src.seek(SeekFrom::Start(PARTITION_LBA * SECTOR)).unwrap();
    let mut buf = vec![0u8; 1 << 20];
    let mut at = 0u64;
    while at < len {
        let n = src.read(&mut buf).unwrap();
        if n == 0 { break; }
        if buf[..n].iter().any(|&b| b != 0) { dst.seek(SeekFrom::Start(at)).unwrap(); dst.write_all(&buf[..n]).unwrap(); }
        at += n as u64;
    }
    drop(dst);
    let out = std::process::Command::new(tool).arg("-n").arg(&vol).output().map_err(|e| format!("{tool}: {e}"))?;
    let _ = std::fs::remove_file(&vol);
    let text = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    if out.status.success() { Ok(text) } else { Err(format!("{tool} -n: {}\n{text}", out.status)) }
}

// exFAT. Field offsets and checksums: Microsoft's exFAT File System Specification ("Main Boot
// Sector", "Main and Backup Boot Checksum Sub-regions", "Allocation Bitmap", "Up-case Table",
// "Volume Label Directory Entry"); the layout is exfatprogs' mkfs (1.2.5, mkfs/mkfs.c
// `exfat_build_mkfs_info`): the FAT at the first 1 MiB boundary after the 24 boot-region sectors,
// the cluster heap at the next 1 MiB boundary after the FAT, the allocation bitmap, the up-case
// table and the root directory in the heap's first clusters, each a FAT chain.

/// Whether the partition from LBA 2048 of `image` holds an exFAT boot sector.
pub fn is_exfat(image: &Path) -> bool {
    let mut b = [0u8; 512];
    std::fs::File::open(image).and_then(|mut f| { f.seek(SeekFrom::Start(PARTITION_LBA * SECTOR))?; f.read_exact(&mut b) }).is_ok() && &b[3..11] == b"EXFAT   "
}

/// The up-case table the builder writes, compressed (0xFFFF then a count of characters that map
/// to themselves): every character to itself but `a`-`z`, which map to `A`-`Z`. The specification
/// requires those 128 first mappings ("Mandatory First 128 Up-case Table Entries") and lets a
/// volume carry a smaller table than its recommended one; a fixed table keeps the image's bytes
/// the same on every host, where one made from the host's Unicode tables would not be.
pub const EXFAT_UPCASE: [u16; 30] = {
    let mut t = [0u16; 30];
    t[0] = 0xffff;
    t[1] = 0x61;
    let mut i = 0;
    while i < 26 { t[2 + i] = 0x41 + i as u16; i += 1; }
    t[28] = 0xffff;
    t[29] = (0x1_0000 - 0x7b) as u16;
    t
};

/// The 32-bit checksum of the boot region and of the up-case table: rotate right, add each byte.
/// `boot` skips VolumeFlags (bytes 106-107) and PercentInUse (112) of the first sector.
pub fn exfat_checksum32(data: &[u8], boot: bool, mut sum: u32) -> u32 {
    for (i, &b) in data.iter().enumerate() {
        if boot && (i == 106 || i == 107 || i == 112) { continue; }
        sum = sum.rotate_right(1).wrapping_add(b as u32);
    }
    sum
}

/// Where `exfat` put each structure, in sectors from the partition's start and in clusters.
#[derive(Clone, Copy, Debug)]
pub struct ExfatLayout { pub volume_sectors: u64, pub fat_offset: u32, pub fat_length: u32, pub heap_offset: u32, pub clusters: u32, pub spc_bits: u8, pub bitmap_bytes: u32, pub upcase_cluster: u32, pub root_cluster: u32 }

pub fn exfat_layout(size: u64, cluster: u32) -> ExfatLayout {
    assert!(cluster.is_power_of_two() && (512..=32 << 20).contains(&cluster), "{cluster}-byte clusters");
    const ALIGN: u64 = 1 << 20;
    let part = PARTITION_LBA * SECTOR;
    let vol = size - part;
    let fat_off = (part + 24 * SECTOR).next_multiple_of(ALIGN) - part;
    let max_clusters = (vol - fat_off - 8 - 1) / (cluster as u64 + 4) + 1;
    let fat_len = ((max_clusters + 2) * 4).next_multiple_of(SECTOR);
    let heap_off = (part + fat_off + fat_len).next_multiple_of(ALIGN) - part;
    let clusters = ((vol - heap_off) / cluster as u64) as u32;
    let bitmap_bytes = clusters.div_ceil(8);
    let bitmap_clusters = bitmap_bytes.div_ceil(cluster);
    let upcase_cluster = 2 + bitmap_clusters;
    let root_cluster = upcase_cluster + (EXFAT_UPCASE.len() as u32 * 2).div_ceil(cluster);
    ExfatLayout { volume_sectors: vol / SECTOR, fat_offset: (fat_off / SECTOR) as u32, fat_length: (fat_len / SECTOR) as u32, heap_offset: (heap_off / SECTOR) as u32,
                  clusters, spc_bits: (cluster / SECTOR as u32).trailing_zeros() as u8, bitmap_bytes, upcase_cluster, root_cluster }
}

/// Write a `size`-byte image (sparse where it is zero) with an MBR and one exFAT partition (type
/// 0x07) from LBA 2048 of `cluster`-byte clusters, laid out as exfatprogs' mkfs lays one out: the
/// main and backup boot regions with their checksums, one FAT, the allocation bitmap, the up-case
/// table (`EXFAT_UPCASE`) and a root directory holding the volume label, the bitmap's and the
/// up-case table's entries, VolumeFlags clean. exfatprogs picks 4 KiB clusters up to 256 MiB,
/// 32 KiB up to 32 GiB and 128 KiB above (lib/libexfat.c `exfat_set_default_cluster_size`); a
/// car's own format may differ, so the size is the caller's.
pub fn exfat(path: &Path, size: u64, cluster: u32, label: &str) -> std::io::Result<()> {
    let l = exfat_layout(size, cluster);
    let mut f = std::fs::File::create(path)?;
    f.set_len(size)?;
    let mut mbr = [0u8; 512];
    put32(&mut mbr, 440, 0x5d1c_0b17);
    mbr[446..462].copy_from_slice(&[0, 0xfe, 0xff, 0xff, 0x07, 0xfe, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0]);
    put32(&mut mbr, 454, PARTITION_LBA as u32);
    put32(&mut mbr, 458, l.volume_sectors.min(u32::MAX as u64) as u32);
    mbr[510] = 0x55; mbr[511] = 0xaa;
    // the boot region: the boot sector, eight extended boot sectors, the OEM parameters (0xFF as
    // exfatprogs leaves them), a reserved sector, the checksum sector
    let mut region = vec![0u8; 12 * 512];
    let b = &mut region[..512];
    b[..3].copy_from_slice(&[0xeb, 0x76, 0x90]);
    b[3..11].copy_from_slice(b"EXFAT   ");
    b[64..72].copy_from_slice(&PARTITION_LBA.to_le_bytes());
    b[72..80].copy_from_slice(&l.volume_sectors.to_le_bytes());
    put32(b, 80, l.fat_offset);
    put32(b, 84, l.fat_length);
    put32(b, 88, l.heap_offset);
    put32(b, 92, l.clusters);
    put32(b, 96, l.root_cluster);
    put32(b, 100, 0x5d1c_0b17);                                  // serial
    b[104] = 0x00; b[105] = 0x01;                                  // revision 1.00
    b[108] = 9;                                                    // 512-byte sectors
    b[109] = l.spc_bits;
    b[110] = 1;                                                    // one FAT
    b[111] = 0x80;                                                 // drive select
    b[510] = 0x55; b[511] = 0xaa;
    for s in 1..9 { region[s * 512 + 510] = 0x55; region[s * 512 + 511] = 0xaa; }
    region[9 * 512..10 * 512].fill(0xff);
    let sum = exfat_checksum32(&region[512..11 * 512], false, exfat_checksum32(&region[..512], true, 0));
    for i in 0..128 { put32(&mut region[11 * 512..], i * 4, sum); }
    let cb = cluster as u64;
    let heap = (PARTITION_LBA + l.heap_offset as u64) * SECTOR;
    let at_cluster = |c: u32| heap + (c as u64 - 2) * cb;
    let bitmap_clusters = l.upcase_cluster - 2;
    let upcase: Vec<u8> = EXFAT_UPCASE.iter().flat_map(|u| u.to_le_bytes()).collect();
    let upcase_clusters = l.root_cluster - l.upcase_cluster;
    // the FAT: the media descriptor, then a chain for each of the three
    let mut fat = vec![0u8; ((l.root_cluster as usize + 1) * 4).next_multiple_of(512)];
    put32(&mut fat, 0, 0xffff_fff8); put32(&mut fat, 4, 0xffff_ffff);
    for (first, n) in [(2, bitmap_clusters), (l.upcase_cluster, upcase_clusters), (l.root_cluster, 1)] {
        for c in first..first + n { put32(&mut fat, c as usize * 4, if c + 1 == first + n { 0xffff_ffff } else { c + 1 }); }
    }
    let mut bitmap = vec![0u8; l.bitmap_bytes as usize];
    for c in 2..=l.root_cluster { let i = (c - 2) as usize; bitmap[i / 8] |= 1 << (i % 8); }
    let mut root = [0u8; 96];
    root[0] = 0x83;
    let units: Vec<u16> = label.encode_utf16().take(11).collect();
    root[1] = units.len() as u8;
    for (i, u) in units.iter().enumerate() { put16(&mut root, 2 + 2 * i, *u); }
    root[32] = 0x81;
    put32(&mut root, 32 + 20, 2);
    root[32 + 24..32 + 32].copy_from_slice(&(l.bitmap_bytes as u64).to_le_bytes());
    root[64] = 0x82;
    put32(&mut root, 64 + 4, exfat_checksum32(&upcase, false, 0));
    put32(&mut root, 64 + 20, l.upcase_cluster);
    root[64 + 24..64 + 32].copy_from_slice(&(upcase.len() as u64).to_le_bytes());
    let mut put = |off: u64, b: &[u8]| -> std::io::Result<()> { f.seek(SeekFrom::Start(off))?; f.write_all(b) };
    put(0, &mbr)?;
    put(PARTITION_LBA * SECTOR, &region)?;
    put((PARTITION_LBA + 12) * SECTOR, &region)?;
    put((PARTITION_LBA + l.fat_offset as u64) * SECTOR, &fat)?;
    put(at_cluster(2), &bitmap)?;
    put(at_cluster(l.upcase_cluster), &upcase)?;
    put(at_cluster(l.root_cluster), &root)?;
    Ok(())
}

/// Check an exFAT card image without changing it: `FSCK_EXFAT` (a command taking `-n` and a
/// volume file) on the partition copied out; else, on macOS, `/sbin/fsck_exfat -n` on the image
/// attached read-only and unmounted with `hdiutil` (it checks devices, not files); else Linux's
/// exfatprogs `fsck.exfat -n` on the copy. Its output, or why it failed.
pub fn fsck_exfat(image: &Path) -> Result<String, String> {
    if let Ok(tool) = std::env::var("FSCK_EXFAT") { return check_volume_copy(image, &tool); }
    if Path::new("/sbin/fsck_exfat").exists() && Path::new("/usr/bin/hdiutil").exists() { return fsck_exfat_macos(image); }
    match ["/usr/sbin/fsck.exfat", "/sbin/fsck.exfat", "/usr/bin/fsck.exfat"].iter().find(|p| Path::new(p).exists()) {
        Some(tool) => check_volume_copy(image, tool),
        None => Err("no exFAT checker: set FSCK_EXFAT, or have fsck_exfat (macOS) or exfatprogs (Linux)".into()),
    }
}

fn fsck_exfat_macos(image: &Path) -> Result<String, String> {
    let run = |cmd: &str, args: &[&std::ffi::OsStr]| std::process::Command::new(cmd).args(args).output().map_err(|e| format!("{cmd}: {e}"));
    let s = |x: &str| std::ffi::OsStr::new(x).to_owned();
    let attach = run("/usr/bin/hdiutil", &[&s("attach"), &s("-readonly"), &s("-nomount"), &s("-noverify"), &s("-imagekey"), &s("diskimage-class=CRawDiskImage"), image.as_os_str()])?;
    let listing = String::from_utf8_lossy(&attach.stdout).to_string();
    if !attach.status.success() { return Err(format!("hdiutil attach: {}\n{listing}{}", attach.status, String::from_utf8_lossy(&attach.stderr))); }
    let devs: Vec<&str> = listing.lines().filter_map(|l| l.split_whitespace().next()).filter(|d| d.starts_with("/dev/disk")).collect();
    let whole = devs.iter().copied().min_by_key(|d| d.len()).ok_or_else(|| format!("hdiutil attach listed no disk:\n{listing}"))?.to_string();
    let slice = devs.iter().find(|d| d.ends_with("s1")).map(|d| d.replace("/dev/disk", "/dev/rdisk"));
    let r = match &slice {
        Some(dev) => run("/sbin/fsck_exfat", &[&s("-n"), &s(dev)]),
        None => Err(format!("no partition in hdiutil's listing:\n{listing}")),
    };
    let detach = run("/usr/bin/hdiutil", &[&s("detach"), &s(&whole)]);
    let out = r?;
    if let Ok(d) = detach { if !d.status.success() { return Err(format!("hdiutil detach {whole}: {}", String::from_utf8_lossy(&d.stderr))); } }
    let text = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    if out.status.success() { Ok(text) } else { Err(format!("fsck_exfat -n: {}\n{text}", out.status)) }
}
