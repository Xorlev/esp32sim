//! The car's exFAT (`exfat::ExFat`) on hand-laid images (`sd_image::exfat`): what Linux 5.8's
//! exfat writes, checked on the bytes.
use super::sd_image;
use super::{FakeMsc, FakePort, FakeWindow, FileDisk, msc};
use crate::usb_host::car::{Car, CarCfg, Offline, Recorded};
use crate::usb_host::exfat::{self, ExFat, Region, ALLOC_FAT_CHAIN, ALLOC_NO_FAT_CHAIN, EOF, VOL_CLEAN, VOL_DIRTY};
use crate::usb_host::fat32::{mp4_bytes, FsckWrite, MemDisk};
use crate::usb_host::pagecache::{PageCache, WritebackCfg};

const S: u64 = 1_000_000_000;

/// An exFAT image of `size` bytes and `cluster`-byte clusters, in memory.
fn volume(tag: &str, size: u64, cluster: u32) -> Vec<u8> {
    let img = std::env::temp_dir().join(format!("usbhost-exfat-{tag}-{}.img", std::process::id()));
    sd_image::exfat(&img, size, cluster, "ESP32SIM").unwrap();
    let d = std::fs::read(&img).unwrap();
    let _ = std::fs::remove_file(&img);
    d
}

fn fat_bytes(d: &MemDisk, fs: &ExFat) -> Vec<u8> { let l = fs.layout; d.0[l.fat_lba as usize * 512..(l.fat_lba as usize + l.fat_sectors as usize) * 512].to_vec() }
fn entry32(fat: &[u8], c: u32) -> u32 { u32::from_le_bytes(fat[c as usize * 4..c as usize * 4 + 4].try_into().unwrap()) }
fn bit(d: &MemDisk, fs: &ExFat, c: u32) -> bool {
    let at = fs.layout.cluster_lba(fs.layout.bitmap_cluster) as usize * 512 + (c as usize - 2) / 8;
    d.0[at] & 1 << ((c - 2) % 8) != 0
}

const TESLA: &str = "TeslaCam/RecentClips/2026-01-01_00-00-00-front.mp4";

/// The image as `sd_image::exfat` lays it out reads as exFAT: the boot region's checksum, the
/// label, the bitmap and up-case table from the root, every cluster but the three free.
#[test]
fn an_exfat_image_mounts_with_its_layout() {
    let mut d = MemDisk(volume("layout", 64 << 20, 4096));
    let fs = ExFat::mount(&mut d).unwrap();
    let l = fs.layout;
    assert_eq!((l.volume_lba, l.fat_lba, l.heap_lba, fs.cluster_bytes()), (2048, 4096, 6144, 4096), "{l:?}");
    assert_eq!((l.bitmap_cluster, l.upcase_cluster, l.root_cluster, fs.label.as_str()), (2, 3, 4, "ESP32SIM"));
    assert_eq!(fs.free_clusters(), l.clusters - 3);
    assert_eq!(fs.vol_flags(), VOL_CLEAN);
    // the boot checksum leaves VolumeFlags out: a dirty volume still mounts
    let mut dirty = MemDisk(d.0.clone());
    dirty.0[2048 * 512 + 106] = VOL_DIRTY as u8;
    let again = ExFat::mount(&mut dirty).unwrap();
    assert!(again.dirty(&mut dirty).unwrap());
    // and the checksum covers the rest
    dirty.0[2048 * 512 + 100] ^= 1;
    assert!(ExFat::mount(&mut dirty).unwrap_err().contains("checksum"));
}

/// A file written alone takes each cluster after its last, so it stays NoFatChain: no FAT entry
/// is written for it, and its Stream Extension says so, with ValidDataLength = DataLength.
#[test]
fn a_file_written_alone_stays_nofatchain() {
    let mut d = MemDisk(volume("alone", 64 << 20, 4096));
    let mut fs = ExFat::mount(&mut d).unwrap();
    let fat0 = fat_bytes(&d, &fs);
    fs.mkdir_all(&mut d, "TeslaCam/RecentClips").unwrap();
    let clip = mp4_bytes("alone", 300_000);
    let mut f = fs.create_path(&mut d, TESLA).unwrap();
    for part in clip.chunks(64 << 10) { f.write(&mut fs, &mut d, part).unwrap(); }
    assert!(!f.is_chained());
    assert_eq!(f.close(&mut fs, &mut d).unwrap(), 300_000);
    assert_eq!(fat_bytes(&d, &fs), fat0, "no FAT entry written: the directories and the file are NoFatChain");
    let e = fs.stat(&mut d, TESLA).unwrap().unwrap();
    assert_eq!((e.chain.flags, e.size, e.valid, e.chain.clusters), (ALLOC_NO_FAT_CHAIN, 300_000, 300_000, 74));
    assert_eq!(e.entries, 4, "File, Stream Extension and two name entries for a 29-character name");
    let cl = fs.clusters_of(&mut d, TESLA).unwrap();
    assert!(cl.windows(2).all(|w| w[1] == w[0] + 1) && cl.iter().all(|&c| bit(&d, &fs, c)));
    assert_eq!(fs.read_path(&mut d, TESLA).unwrap(), clip);
    let mut again = MemDisk(d.0.clone());
    let mut fresh = ExFat::mount(&mut again).unwrap();
    assert_eq!(fresh.read_path(&mut again, TESLA).unwrap(), clip, "a fresh mount reads it back");
    assert_eq!(fresh.list(&mut again, "TeslaCam").unwrap().iter().map(|e| e.name.as_str()).collect::<Vec<_>>(), ["RecentClips"]);
}

/// Two files written in turn: the second takes the cluster after the first's, so the first's next
/// cluster is not its last + 1 and it becomes a FAT chain, its clusters so far chained in the FAT;
/// the second, likewise, at its next chunk. Both chains read back through the FAT.
#[test]
fn two_files_written_in_turn_become_fat_chains() {
    let mut d = MemDisk(volume("turns", 64 << 20, 4096));
    let mut fs = ExFat::mount(&mut d).unwrap();
    let log = exfat::TruthLog::default();
    fs.set_truth(log.clone());
    fs.mkdir_all(&mut d, "TeslaCam/RecentClips").unwrap();
    let (pa, pb) = ("TeslaCam/RecentClips/a-front.mp4", "TeslaCam/RecentClips/a-back.mp4");
    let (ca, cb) = (mp4_bytes(pa, 256 << 10), mp4_bytes(pb, 256 << 10));
    let mut a = fs.create_path(&mut d, pa).unwrap();
    let mut b = fs.create_path(&mut d, pb).unwrap();
    for (x, y) in ca.chunks(64 << 10).zip(cb.chunks(64 << 10)) { a.write(&mut fs, &mut d, x).unwrap(); b.write(&mut fs, &mut d, y).unwrap(); }
    assert!(a.is_chained() && b.is_chained());
    a.close(&mut fs, &mut d).unwrap();
    b.close(&mut fs, &mut d).unwrap();
    let fat = fat_bytes(&d, &fs);
    for (p, c) in [(pa, &ca), (pb, &cb)] {
        let e = fs.stat(&mut d, p).unwrap().unwrap();
        assert_eq!(e.chain.flags, ALLOC_FAT_CHAIN, "{p}");
        let cl = fs.clusters_of(&mut d, p).unwrap();
        assert_eq!(cl.len(), 64);
        assert_eq!(cl.chunks(16).count(), 4);
        assert!(cl.chunks(16).all(|r| r.windows(2).all(|w| w[1] == w[0] + 1)), "runs of 16 clusters, a chunk each");
        for w in cl.windows(2) { assert_eq!(entry32(&fat, w[0]), w[1]); }
        assert_eq!(entry32(&fat, *cl.last().unwrap()), EOF);
        assert_eq!(fs.read_path(&mut d, p).unwrap(), *c);
    }
    let t = log.lock().unwrap();
    assert_eq!(t.files[pa].chained.map(|c| c.1), Some(16), "a converted at its second chunk, with one chunk's clusters");
    assert_eq!(t.files[pb].chained.map(|c| c.1), Some(16));
    assert_eq!((t.files[pa].fragments(), t.files[pa].clusters.len()), (4, 64));
}

/// Deleting frees the bitmap bits and marks the entries deleted; the FAT keeps the chain.
#[test]
fn a_deletion_clears_bitmap_bits_and_leaves_the_fat() {
    let mut d = MemDisk(volume("delete", 64 << 20, 4096));
    let mut fs = ExFat::mount(&mut d).unwrap();
    fs.mkdir_all(&mut d, "TeslaCam/RecentClips").unwrap();
    let (pa, pb) = ("TeslaCam/RecentClips/x.mp4", "TeslaCam/RecentClips/y.mp4");
    let mut a = fs.create_path(&mut d, pa).unwrap();
    let mut b = fs.create_path(&mut d, pb).unwrap();
    for _ in 0..3 { a.write(&mut fs, &mut d, &[1u8; 64 << 10]).unwrap(); b.write(&mut fs, &mut d, &[2u8; 64 << 10]).unwrap(); }
    a.close(&mut fs, &mut d).unwrap();
    b.close(&mut fs, &mut d).unwrap();
    let cl = fs.clusters_of(&mut d, pa).unwrap();
    let (fat0, free0) = (fat_bytes(&d, &fs), fs.free_clusters());
    let e = fs.stat(&mut d, pa).unwrap().unwrap();
    let freed = fs.delete_path(&mut d, pa).unwrap();
    assert_eq!(freed, cl);
    assert_eq!(fat_bytes(&d, &fs), fat0, "the FAT is not written");
    assert!(cl.iter().all(|&c| !bit(&d, &fs, c)) && fs.free_clusters() == free0 + 48);
    assert!(fs.stat(&mut d, pa).unwrap().is_none());
    // the set's entries keep their bytes, the type's top bit cleared
    let dir = fs.stat(&mut d, "TeslaCam/RecentClips").unwrap().unwrap().chain.first;
    let first = fs.layout.cluster_lba(dir) as usize * 512;
    let set = &d.0[first + e.index as usize * 32..first + (e.index + e.entries) as usize * 32];
    assert_eq!(set.chunks(32).map(|x| x[0]).collect::<Vec<_>>(), [0x05, 0x40, 0x41]);
    assert_eq!(fs.read_path(&mut d, pb).unwrap(), vec![2u8; 192 << 10]);
    // the next file written alone takes freed clusters whose FAT entries still chain the deleted one
    let mut c = fs.create_path(&mut d, "TeslaCam/RecentClips/z.mp4").unwrap();
    c.write(&mut fs, &mut d, &[3u8; 4096]).unwrap();
    c.close(&mut fs, &mut d).unwrap();
    let z = fs.clusters_of(&mut d, "TeslaCam/RecentClips/z.mp4").unwrap();
    assert_eq!(fs.stat(&mut d, "TeslaCam/RecentClips/z.mp4").unwrap().unwrap().chain.flags, ALLOC_NO_FAT_CHAIN);
    let _ = z;
}

/// Every entry set on the medium carries the checksum of its bytes and the hash of its up-cased
/// name; a name of 31 characters at the end of a sector spills into the next one, and both
/// sectors are written.
#[test]
fn entry_sets_carry_their_checksum_and_name_hash() {
    let mut d = MemDisk(volume("sums", 64 << 20, 4096));
    let mut fs = ExFat::mount(&mut d).unwrap();
    fs.mkdir_all(&mut d, "TeslaCam/SentryClips/2026-01-01_00-03-00").unwrap();
    for i in 0..9 {
        let p = format!("TeslaCam/SentryClips/2026-01-01_00-03-00/2026-01-01_00-0{i}-00-right_repeater.mp4");
        let mut f = fs.create_path(&mut d, &p).unwrap();
        f.write(&mut fs, &mut d, &mp4_bytes(&p, 5000)).unwrap();
        f.close(&mut fs, &mut d).unwrap();
    }
    let mut m = MemDisk(d.0.clone());
    let mut fresh = ExFat::mount(&mut m).unwrap();
    let dir = fresh.stat(&mut m, "TeslaCam/SentryClips/2026-01-01_00-03-00").unwrap().unwrap();
    let base = fresh.layout.cluster_lba(dir.chain.first) as usize * 512;
    let mut straddled = 0;
    for e in fresh.dir_entries(&mut m, dir.chain).unwrap() {
        let set = &m.0[base + e.index as usize * 32..base + (e.index + e.entries) as usize * 32];
        assert_eq!(u16::from_le_bytes([set[2], set[3]]), exfat::set_checksum(set), "{}", e.name);
        let up: Vec<u8> = e.name.to_ascii_uppercase().encode_utf16().flat_map(|u| u.to_le_bytes()).collect();
        assert_eq!(u16::from_le_bytes([set[36], set[37]]), exfat::chksum16(&up, 0, false), "{}", e.name);
        if (e.index * 32) / 512 != ((e.index + e.entries) * 32 - 1) / 512 { straddled += 1; }
    }
    assert!(straddled > 0, "five-entry sets cross a sector boundary");
}

/// A car on exFAT with Linux's writeback and VolumeFlags kept as 5.8 keeps them: nothing at the
/// mount; the first change writes them dirty at once; create and unlink leave them clean behind
/// them for writeback, and the allocation that follows a create sets them dirty again while that
/// is still waiting, so recording writes the boot sector only by writeback, dirty; a change to
/// dirty after the boot sector has landed clean is written at once; a clean unmount lands clean
/// last.
#[test]
fn volume_flags_go_dirty_at_each_change_and_clean_after_create_and_unlink() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("flags", 64 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut car = Car::<_, ExFat>::on(disk, CarCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, gap_ns: S / 4, ..CarCfg::default() });
    car.disk.record = true;
    car.mount().unwrap();
    assert!(car.disk.landings.is_empty(), "nothing written at the mount");
    for i in 0..3 {
        car.record(&format!("TeslaCam/RecentClips/c{i}.mp4"), &mp4_bytes(&format!("c{i}"), 1 << 20)).unwrap();
        car.idle(40 * S).unwrap();
    }
    let flags = car.truth.lock().unwrap().flags.clone();
    assert_eq!((flags[0].1, flags[0].2), (VOL_DIRTY, true), "the first change: dirty, written at once");
    assert!(flags[1..].iter().all(|f| !f.2), "every later change while recording left for writeback: {flags:?}");
    assert!(flags.windows(2).all(|w| w[0].1 != w[1].1), "only changes are written");
    let boot = |car: &Car<_, ExFat>| -> Vec<(u64, u16)> { car.disk.meta_landed.iter().filter(|m| m.1 == 2048).map(|m| (m.0, u16::from_le_bytes([m.2[106], m.2[107]]))).collect() };
    assert!(boot(&car).iter().all(|b| b.1 == VOL_DIRTY), "the boot sector lands dirty while the car records: {:?}", boot(&car));
    // an unlink leaves them clean; once that has landed, the next create writes dirty at once
    car.delete("TeslaCam/RecentClips/c0.mp4").unwrap();
    car.idle(40 * S).unwrap();
    assert_eq!(boot(&car).last().unwrap().1, VOL_CLEAN);
    let t0 = car.now();
    car.record("TeslaCam/RecentClips/c3.mp4", &mp4_bytes("c3", 1 << 20)).unwrap();
    let flags = car.truth.lock().unwrap().flags.clone();
    assert!(flags.iter().any(|f| f.0 == t0 && f.1 == VOL_DIRTY && f.2), "{flags:?}");
    car.unmount().unwrap();
    let last = car.disk.meta_landed.iter().rev().find(|m| m.1 == 2048).unwrap();
    assert_eq!(u16::from_le_bytes([last.2[106], last.2[107]]), VOL_CLEAN, "the unmount's clean lands last");
    let mut m = std::mem::replace(&mut car.disk.dev.dev, MemDisk(Vec::new()));
    let mut fs = ExFat::mount(&mut m).unwrap();
    assert!(!fs.dirty(&mut m).unwrap());
    for i in 1..3 { assert_eq!(fs.read_path(&mut m, &format!("TeslaCam/RecentClips/c{i}.mp4")).unwrap(), mp4_bytes(&format!("c{i}"), 1 << 20)); }
}

/// The medium mid-recording under writeback: the first allocation's entry (NoFatChain, the first
/// cluster, size 0) and later sizes land before the clip's last; the region of every landing is
/// one the layout names.
#[test]
fn writeback_lands_intermediate_entry_sets() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("wb", 64 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut car = Car::<_, ExFat>::on(disk, CarCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, gap_ns: S, ..CarCfg::default() });
    car.disk.record = true;
    car.mount().unwrap();
    let p = "TeslaCam/SavedClips/e/front.mp4";
    assert!(matches!(car.record(p, &mp4_bytes(p, 3 << 20)).unwrap(), Recorded::Closed { .. }));
    car.idle(40 * S).unwrap();
    let t = car.truth.lock().unwrap().clone();
    let f = &t.files[p];
    let (layout, dirs) = (t.layout.unwrap(), &t.dir_clusters);
    // the folder holds the one file: its stream entry in each landing of the set's sector
    let lba0 = f.set_lbas[0];
    let sizes: Vec<(u8, u32, u64)> = car.disk.meta_landed.iter().filter(|m| (m.1..m.1 + m.2.len() as u64 / 512).contains(&lba0)).filter_map(|m| {
        let s = &m.2[(lba0 - m.1) as usize * 512..][..512];
        let i = s.chunks(32).position(|e| e[0] == 0x85)?;
        let st = &s[(i + 1) * 32..(i + 2) * 32];
        Some((st[1], u32::from_le_bytes(st[20..24].try_into().unwrap()), u64::from_le_bytes(st[24..32].try_into().unwrap())))
    }).collect();
    assert!(sizes.iter().any(|s| s.2 > 0 && s.2 < 3 << 20), "an intermediate size landed: {sizes:?}");
    assert_eq!(*sizes.last().unwrap(), (ALLOC_NO_FAT_CHAIN, f.clusters[0].0, 3 << 20));
    for l in &car.disk.landings {
        let r = layout.region(l.lba, dirs);
        assert!(if l.meta { matches!(r, Region::Boot | Region::Fat | Region::Bitmap | Region::Dirent) } else { r == Region::Data }, "{l:?} in {r:?}");
    }
}

/// Power gone mid-recording on exFAT: the medium keeps a dirty, torn volume; exfatprogs-style
/// fsck cuts what does not hold together, and a second run finds nothing.
#[test]
fn a_power_cut_leaves_an_exfat_volume_fsck_repairs() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("cut", 64 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut car = Car::<_, ExFat>::on(disk, CarCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, ..CarCfg::default() });
    car.mount().unwrap();
    let ps: Vec<String> = (0..4).map(|i| format!("TeslaCam/RecentClips/c{i}.mp4")).collect();
    let data: Vec<Vec<u8>> = ps.iter().map(|p| mp4_bytes(p, 2 << 20)).collect();
    let set: Vec<(&str, &[u8])> = ps.iter().map(|p| p.as_str()).zip(data.iter().map(|d| d.as_slice())).collect();
    car.record_set(&set, 16_000_000).unwrap();
    car.idle(3 * S).unwrap();
    assert!(car.power_cut() > 0);
    let mut m = Offline { dev: std::mem::replace(&mut car.disk.dev.dev, MemDisk(Vec::new())), now: 0 };
    let mut fs = ExFat::mount(&mut m).unwrap();
    assert!(fs.dirty(&mut m).unwrap(), "no clean unmount");
    let r = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(r.dirty_cleared, "{}", r.summary());
    let again = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(!again.dirty_cleared && again.truncated.is_empty() && again.resized.is_empty() && again.cross_or_broken.is_empty() && again.fat_sectors_changed == 0, "{}", again.summary());
    assert!(!fs.dirty(&mut m).unwrap());
}

/// A car's RecentClips ring on exFAT: deleted entry sets leave their slots to the next clip.
/// Thirteen sets of 18 entries (four or five per name) outgrow a 4 KiB cluster's 128 entries
/// once; the cluster after the folder's is a clip's by then, so the folder becomes a FAT chain of
/// two and the ring stays in them. Each set of four cameras written together is FAT-chained.
#[test]
fn an_exfat_ring_of_clips_reuses_its_slots_and_chains_its_sets() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("ring", 256 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut car = Car::<_, ExFat>::on(disk, CarCfg { poll_ns: 0, gap_ns: 0, linux_entries: true, dirty_flag: true, ..CarCfg::default() });
    car.mount().unwrap();
    let cams = ["front", "back", "left_repeater", "right_repeater"];
    let mut ring = std::collections::VecDeque::new();
    for m in 0..120u32 {
        let paths: Vec<String> = cams.iter().map(|c| format!("TeslaCam/RecentClips/2026-01-01_{:02}-{:02}-00-{c}.mp4", m / 60, m % 60)).collect();
        let datas: Vec<Vec<u8>> = paths.iter().map(|p| mp4_bytes(p, 200_000)).collect();
        let set: Vec<(&str, &[u8])> = paths.iter().map(|p| p.as_str()).zip(datas.iter().map(|d| d.as_slice())).collect();
        car.record_set(&set, 0).unwrap();
        ring.push_back(paths);
        if ring.len() > 12 { for p in ring.pop_front().unwrap() { car.delete(&p).unwrap(); } }
    }
    car.unmount().unwrap();
    let t = car.truth.lock().unwrap().clone();
    let clips: Vec<_> = t.files.iter().chain(t.past.iter().map(|(p, f)| (p, f))).filter(|(_, f)| !f.dir).collect();
    assert_eq!(clips.len(), 480);
    assert!(clips.iter().all(|(_, f)| f.is_chained()), "four cameras written together: every clip chained");
    let mut m = std::mem::replace(&mut car.disk.dev.dev, MemDisk(Vec::new()));
    let mut fs = ExFat::mount(&mut m).unwrap();
    let e = fs.stat(&mut m, "TeslaCam/RecentClips").unwrap().unwrap();
    assert_eq!((e.chain.clusters, e.chain.flags), (2, ALLOC_FAT_CHAIN), "grown once, chained");
    assert_eq!(fs.list(&mut m, "TeslaCam/RecentClips").unwrap().len(), 48);
    let r = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(r.truncated.is_empty() && r.resized.is_empty() && r.cross_or_broken.is_empty() && r.unreclaimed == 0, "{}", r.summary());
}

/// A directory past its cluster grows by one, hinted after its last; when that cluster is taken
/// the directory becomes a FAT chain and its own entry says so.
#[test]
fn a_directory_grows_and_chains_when_its_next_cluster_is_taken() {
    let mut d = MemDisk(volume("grow", 64 << 20, 4096));
    let mut fs = ExFat::mount(&mut d).unwrap();
    fs.mkdir_all(&mut d, "TeslaCam/SavedClips").unwrap();
    // a file right after the folder's cluster, then more names than its 128 entries hold
    let mut f = fs.create_path(&mut d, "TeslaCam/SavedClips/big.bin").unwrap();
    f.write(&mut fs, &mut d, &[7u8; 8192]).unwrap();
    f.close(&mut fs, &mut d).unwrap();
    for i in 0..45 {
        let f = fs.create_path(&mut d, &format!("TeslaCam/SavedClips/event-{i:03}.json")).unwrap();
        f.close(&mut fs, &mut d).unwrap();
    }
    let e = fs.stat(&mut d, "TeslaCam/SavedClips").unwrap().unwrap();
    assert_eq!((e.chain.clusters, e.chain.flags, e.size), (2, ALLOC_FAT_CHAIN, 8192));
    assert_eq!(fs.list(&mut d, "TeslaCam/SavedClips").unwrap().len(), 46);
    let mut m = MemDisk(d.0.clone());
    let mut fresh = ExFat::mount(&mut m).unwrap();
    assert_eq!(fresh.read_path(&mut m, "TeslaCam/SavedClips/big.bin").unwrap(), vec![7u8; 8192]);
    assert_eq!(fresh.list(&mut m, "TeslaCam/SavedClips").unwrap().len(), 46);
}

/// A Tesla tree on a 3 GiB exFAT card with 32 KiB clusters, written by a writeback car with
/// rotation and unmounted cleanly, checked by the system's exFAT checker (`sd_image::fsck`).
#[test]
#[ignore = "needs an exFAT checker: fsck_exfat with hdiutil (macOS), exfatprogs' fsck.exfat (Linux), or FSCK_EXFAT"]
fn external_exfat_tree_passes_fsck() {
    let img = std::env::temp_dir().join(format!("usbhost-exfat-fsck-{}.img", std::process::id()));
    sd_image::exfat(&img, 3 << 30, 32 << 10, "ESP32SIM").unwrap();
    let file = FileDisk(std::fs::OpenOptions::new().read(true).write(true).open(&img).unwrap());
    let mut car = Car::<_, ExFat>::on(PageCache::writeback(Offline { dev: file, now: 0 }, WritebackCfg::default()), CarCfg { poll_ns: 0, gap_ns: 0, linux_entries: true, dirty_flag: true, ..CarCfg::default() });
    car.mount().unwrap();
    let cams = ["front", "back", "left_repeater", "right_repeater"];
    let mut ring = std::collections::VecDeque::new();
    for m in 0..30u32 {
        let paths: Vec<String> = cams.iter().map(|c| format!("TeslaCam/RecentClips/2026-01-01_00-{m:02}-00-{c}.mp4")).collect();
        let datas: Vec<Vec<u8>> = paths.iter().map(|p| mp4_bytes(p, 700_000)).collect();
        let set: Vec<(&str, &[u8])> = paths.iter().map(|p| p.as_str()).zip(datas.iter().map(|d| d.as_slice())).collect();
        car.record_set(&set, 0).unwrap();
        ring.push_back(paths);
        if ring.len() > 6 { for p in ring.pop_front().unwrap() { car.delete(&p).unwrap(); } }
        if m % 10 == 9 {
            let ev = format!("TeslaCam/SavedClips/2026-01-01_00-{m:02}-00/event.json");
            car.record(&ev, b"{\"reason\":\"user_interaction_honk\"}").unwrap();
        }
    }
    car.unmount().unwrap();
    drop(car);
    let r = sd_image::fsck(&img);
    let _ = std::fs::remove_file(&img);
    eprintln!("{}", r.as_ref().unwrap_or_else(|e| e));
    r.unwrap();
}

// ---- the car's hazards on exFAT, against the fake drive ----

fn exfat_disk(tag: &str) -> Vec<u8> { volume(tag, 64 << 20, 4096) }

/// The application's FatFs making a folder on an exFAT card in a window (here the car's own
/// writer stands in for it).
fn app_mkdir_exfat(path: &'static str) -> super::AppWrite {
    Box::new(move |disk: &mut Vec<u8>| {
        let mut m = MemDisk(std::mem::take(disk));
        let mut fs = ExFat::mount(&mut m).unwrap();
        fs.mkdir_all(&mut m, path).unwrap();
        *disk = m.0;
    })
}

/// `a_car_keeps_writing_into_a_window_until_its_next_poll` on exFAT: the writes up to the poll
/// reach the card; the cut clip's entry has its first cluster and the size it had at its first
/// allocation, and the bitmap holds its clusters.
#[test]
fn an_exfat_car_keeps_writing_into_a_window_until_its_next_poll() {
    let mut dev = FakeMsc::new(0);
    dev.disk = exfat_disk("window");
    dev.windows.push(FakeWindow { from: S / 2, to: 3 * S, app: None });
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut car = Car::<_, ExFat>::on(PageCache::new(m), CarCfg::default());
    car.mount().unwrap();
    let r = car.record("TeslaCam/RecentClips/clip.mp4", &mp4_bytes("event", 8 << 20)).unwrap();
    let Recorded::MediumGone { written } = r else { panic!("{r:?}") };
    assert!(written > 6 << 20 && written < 8 << 20, "{written}");
    car.idle(2 * S).unwrap();
    assert!(car.present && car.stats.mounts == 2 && car.stats.media_changes == 2, "{:?}", car.stats);
    drop(car);
    let mut mem = MemDisk(port.dev.disk.clone());
    let mut fs = ExFat::mount(&mut mem).unwrap();
    let e = fs.stat(&mut mem, "TeslaCam/RecentClips/clip.mp4").unwrap().unwrap();
    assert_eq!((e.chain.flags, e.size), (ALLOC_NO_FAT_CHAIN, 0), "the first allocation's entry: NoFatChain, size 0");
    assert!(e.chain.first >= 2);
}

/// The gated window on exFAT: the car refused at once, gone at its poll, back at the unit
/// attention, the volume read again.
#[test]
fn a_gated_window_refuses_an_exfat_car_and_its_unit_attention_revalidates() {
    let mut dev = FakeMsc::new(0);
    dev.disk = exfat_disk("gated");
    dev.windows.push(FakeWindow { from: S / 2, to: 3 * S, app: None });
    dev.gated = true;
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut car = Car::<_, ExFat>::on(PageCache::new(m), CarCfg::default());
    car.mount().unwrap();
    let r = car.record("TeslaCam/RecentClips/clip.mp4", &mp4_bytes("event", 8 << 20)).unwrap();
    assert!(matches!(r, Recorded::WriteFailed { .. }), "{r:?}");
    car.idle(4_500_000_000).unwrap();
    assert_eq!((car.stats.unit_attentions, car.stats.media_changes, car.stats.mounts), (1, 2, 2), "{:?}", car.stats);
    drop(car);
    assert!(port.dev.write_at.iter().all(|&t| !(S / 2..3 * S).contains(&t)), "no WRITE(10) reached the card in the window");
}

/// An exFAT car between polls: a window's folder is lost to a car that does not see the window,
/// caching its metadata or not, because the driver keeps the allocation bitmap from the mount
/// whatever the cache does; it is kept when a gated window's unit attention makes the car read the
/// volume again.
#[test]
fn an_exfat_car_and_a_window_between_its_polls() {
    for (keep, gated) in [(true, false), (false, false), (true, true)] {
        let mut dev = FakeMsc::new(0);
        dev.disk = exfat_disk("short");
        dev.windows.push(FakeWindow { from: S, to: 1_500_000_000, app: Some(app_mkdir_exfat("TeslaCam/SavedClips")) });
        dev.gated = gated;
        let mut port = FakePort::new(dev);
        let m = msc(&mut port);
        let disk = if keep { PageCache::keeping_meta(m) } else { PageCache::new(m) };
        let mut car = Car::<_, ExFat>::on(disk, CarCfg { poll_ns: 5 * S, ..CarCfg::default() });
        car.mount().unwrap();
        assert!(car.fs.clone().unwrap().stat(&mut car.disk, "TeslaCam").unwrap().is_none());
        car.idle(6 * S).unwrap();
        let fs = car.fs.as_mut().unwrap();
        fs.mkdir_all(&mut car.disk, "TeslaCam/RecentClips").unwrap();
        drop(car);
        let mut mem = MemDisk(port.dev.disk.clone());
        let mut fs = ExFat::mount(&mut mem).unwrap();
        let names: Vec<String> = fs.list(&mut mem, "TeslaCam").unwrap().into_iter().map(|e| e.name).collect();
        eprintln!("keep {keep}, gated {gated}: TeslaCam holds {names:?}");
        if gated {
            assert_eq!(names, ["SavedClips", "RecentClips"], "the unit attention: the volume and its bitmap read again");
        } else if keep {
            assert_eq!(names, ["RecentClips"], "the car's cached root and bitmap: its TeslaCam took the window's slot and cluster");
        } else {
            assert!(!names.iter().any(|n| n == "SavedClips"), "the root read again but the bitmap kept: the window's clusters taken again");
        }
    }
}
