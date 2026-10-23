//! The recorder's exFAT (`exfat::ExFat`) on hand-laid images (`sd_image::exfat`): what Linux 5.8's
//! exfat writes, checked on the bytes.
use super::sd_image;
use super::{FakeMsc, FakePort, FakeWindow, msc};
use crate::usb_host::recorder::{Recorder, RecorderCfg, Fill, FsFile, Offline, Recorded};
use crate::usb_host::exfat::{self, ExFat, Region, ALLOC_FAT_CHAIN, ALLOC_NO_FAT_CHAIN, EOF, VOL_CLEAN, VOL_DIRTY};
use crate::usb_host::writelog;
use crate::usb_host::fat32::{bmff_bytes, Fat32, FileDisk, FsckWrite, MemDisk};
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

const LONG_NAME: &str = "HostData/ThirdGroup/2026-01-01_00-00-00-first.bin";

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
    fs.mkdir_all(&mut d, "HostData/ThirdGroup").unwrap();
    let payload = bmff_bytes("alone", 300_000);
    let mut f = fs.create_path(&mut d, LONG_NAME).unwrap();
    for part in payload.chunks(64 << 10) { f.write(&mut fs, &mut d, part).unwrap(); }
    assert!(!f.is_chained());
    assert_eq!(f.close(&mut fs, &mut d).unwrap(), 300_000);
    assert_eq!(fat_bytes(&d, &fs), fat0, "no FAT entry written: the directories and the file are NoFatChain");
    let e = fs.stat(&mut d, LONG_NAME).unwrap().unwrap();
    assert_eq!((e.chain.flags, e.size, e.valid, e.chain.clusters), (ALLOC_NO_FAT_CHAIN, 300_000, 300_000, 74));
    assert_eq!(e.entries, 4, "File, Stream Extension and two name entries for a 29-character name");
    let cl = fs.clusters_of(&mut d, LONG_NAME).unwrap();
    assert!(cl.windows(2).all(|w| w[1] == w[0] + 1) && cl.iter().all(|&c| bit(&d, &fs, c)));
    assert_eq!(fs.read_path(&mut d, LONG_NAME).unwrap(), payload);
    let mut again = MemDisk(d.0.clone());
    let mut fresh = ExFat::mount(&mut again).unwrap();
    assert_eq!(fresh.read_path(&mut again, LONG_NAME).unwrap(), payload, "a fresh mount reads it back");
    assert_eq!(fresh.list(&mut again, "HostData").unwrap().iter().map(|e| e.name.as_str()).collect::<Vec<_>>(), ["ThirdGroup"]);
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
    fs.mkdir_all(&mut d, "HostData/ThirdGroup").unwrap();
    let (pa, pb) = ("HostData/ThirdGroup/a-first.bin", "HostData/ThirdGroup/a-aux1.bin");
    let (ca, cb) = (bmff_bytes(pa, 256 << 10), bmff_bytes(pb, 256 << 10));
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
    fs.mkdir_all(&mut d, "HostData/ThirdGroup").unwrap();
    let (pa, pb) = ("HostData/ThirdGroup/x.bin", "HostData/ThirdGroup/y.bin");
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
    let dir = fs.stat(&mut d, "HostData/ThirdGroup").unwrap().unwrap().chain.first;
    let first = fs.layout.cluster_lba(dir) as usize * 512;
    let set = &d.0[first + e.index as usize * 32..first + (e.index + e.entries) as usize * 32];
    assert_eq!(set.chunks(32).map(|x| x[0]).collect::<Vec<_>>(), [0x05, 0x40, 0x41]);
    assert_eq!(fs.read_path(&mut d, pb).unwrap(), vec![2u8; 192 << 10]);
    // the next file written alone takes freed clusters whose FAT entries still chain the deleted
    // one
    let mut c = fs.create_path(&mut d, "HostData/ThirdGroup/z.bin").unwrap();
    c.write(&mut fs, &mut d, &[3u8; 4096]).unwrap();
    c.close(&mut fs, &mut d).unwrap();
    let z = fs.clusters_of(&mut d, "HostData/ThirdGroup/z.bin").unwrap();
    assert_eq!(fs.stat(&mut d, "HostData/ThirdGroup/z.bin").unwrap().unwrap().chain.flags, ALLOC_NO_FAT_CHAIN);
    let _ = z;
}

/// Every entry set on the medium carries the checksum of its bytes and the hash of its up-cased
/// name; a name of 31 characters at the end of a sector spills into the next one, and both
/// sectors are written.
#[test]
fn entry_sets_carry_their_checksum_and_name_hash() {
    let mut d = MemDisk(volume("sums", 64 << 20, 4096));
    let mut fs = ExFat::mount(&mut d).unwrap();
    fs.mkdir_all(&mut d, "HostData/SecondGroup/2026-01-01_00-03-00").unwrap();
    for i in 0..9 {
        let p = format!("HostData/SecondGroup/2026-01-01_00-03-00/2026-01-01_00-0{i}-00-fourth_channel.bin");
        let mut f = fs.create_path(&mut d, &p).unwrap();
        f.write(&mut fs, &mut d, &bmff_bytes(&p, 5000)).unwrap();
        f.close(&mut fs, &mut d).unwrap();
    }
    let mut m = MemDisk(d.0.clone());
    let mut fresh = ExFat::mount(&mut m).unwrap();
    let dir = fresh.stat(&mut m, "HostData/SecondGroup/2026-01-01_00-03-00").unwrap().unwrap();
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

/// A recorder on exFAT with Linux's writeback and VolumeFlags kept as 5.8 keeps them: nothing at
/// the mount; the first change writes them dirty at once; create and unlink leave them clean behind
/// them for writeback, and the allocation that follows a create sets them dirty again while that is
/// still waiting, so recording writes the boot sector only by writeback, dirty; a change to dirty
/// after the boot sector has landed clean is written at once; a clean unmount lands clean last.
#[test]
fn volume_flags_go_dirty_at_each_change_and_clean_after_create_and_unlink() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("flags", 64 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut rec = Recorder::<_, ExFat>::on(disk, RecorderCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, gap_ns: S / 4, ..RecorderCfg::default() });
    rec.disk.record = true;
    rec.mount().unwrap();
    assert!(rec.disk.landings.is_empty(), "nothing written at the mount");
    for i in 0..3 {
        rec.record(&format!("HostData/ThirdGroup/c{i}.bin"), &bmff_bytes(&format!("c{i}"), 1 << 20)).unwrap();
        rec.idle(40 * S).unwrap();
    }
    let flags = rec.truth.lock().unwrap().flags.clone();
    assert_eq!((flags[0].1, flags[0].2), (VOL_DIRTY, true), "the first change: dirty, written at once");
    assert!(flags[1..].iter().all(|f| !f.2), "every later change while recording left for writeback: {flags:?}");
    assert!(flags.windows(2).all(|w| w[0].1 != w[1].1), "only changes are written");
    let boot = |rec: &Recorder<_, ExFat>| -> Vec<(u64, u16)> { rec.disk.meta_landed.iter().filter(|m| m.1 == 2048).map(|m| (m.0, u16::from_le_bytes([m.2[106], m.2[107]]))).collect() };
    assert!(boot(&rec).iter().all(|b| b.1 == VOL_DIRTY), "the boot sector lands dirty while the recorder records: {:?}", boot(&rec));
    // an unlink leaves them clean; once that has landed, the next create writes dirty at once
    rec.delete("HostData/ThirdGroup/c0.bin").unwrap();
    rec.idle(40 * S).unwrap();
    assert_eq!(boot(&rec).last().unwrap().1, VOL_CLEAN);
    let t0 = rec.now();
    rec.record("HostData/ThirdGroup/c3.bin", &bmff_bytes("c3", 1 << 20)).unwrap();
    let flags = rec.truth.lock().unwrap().flags.clone();
    assert!(flags.iter().any(|f| f.0 == t0 && f.1 == VOL_DIRTY && f.2), "{flags:?}");
    rec.unmount().unwrap();
    let last = rec.disk.meta_landed.iter().rev().find(|m| m.1 == 2048).unwrap();
    assert_eq!(u16::from_le_bytes([last.2[106], last.2[107]]), VOL_CLEAN, "the unmount's clean lands last");
    let mut m = std::mem::replace(&mut rec.disk.dev.dev, MemDisk(Vec::new()));
    let mut fs = ExFat::mount(&mut m).unwrap();
    assert!(!fs.dirty(&mut m).unwrap());
    for i in 1..3 { assert_eq!(fs.read_path(&mut m, &format!("HostData/ThirdGroup/c{i}.bin")).unwrap(), bmff_bytes(&format!("c{i}"), 1 << 20)); }
}

/// The medium mid-recording under writeback: the first allocation's entry (NoFatChain, the first
/// cluster, size 0) and later sizes land before the file's last; the region of every landing is
/// one the layout names.
#[test]
fn writeback_lands_intermediate_entry_sets() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("wb", 64 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut rec = Recorder::<_, ExFat>::on(disk, RecorderCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, gap_ns: S, ..RecorderCfg::default() });
    rec.disk.record = true;
    rec.mount().unwrap();
    let p = "HostData/FirstGroup/e/first.bin";
    assert!(matches!(rec.record(p, &bmff_bytes(p, 3 << 20)).unwrap(), Recorded::Closed { .. }));
    rec.idle(40 * S).unwrap();
    let t = rec.truth.lock().unwrap().clone();
    let f = &t.files[p];
    let (layout, dirs) = (t.layout.unwrap(), &t.dir_clusters);
    // the folder holds the one file: its stream entry in each landing of the set's sector
    let lba0 = f.set_lbas[0];
    let sizes: Vec<(u8, u32, u64)> = rec.disk.meta_landed.iter().filter(|m| (m.1..m.1 + m.2.len() as u64 / 512).contains(&lba0)).filter_map(|m| {
        let s = &m.2[(lba0 - m.1) as usize * 512..][..512];
        let i = s.chunks(32).position(|e| e[0] == 0x85)?;
        let st = &s[(i + 1) * 32..(i + 2) * 32];
        Some((st[1], u32::from_le_bytes(st[20..24].try_into().unwrap()), u64::from_le_bytes(st[24..32].try_into().unwrap())))
    }).collect();
    assert!(sizes.iter().any(|s| s.2 > 0 && s.2 < 3 << 20), "an intermediate size landed: {sizes:?}");
    assert_eq!(*sizes.last().unwrap(), (ALLOC_NO_FAT_CHAIN, f.clusters[0].0, 3 << 20));
    for l in &rec.disk.landings {
        let r = layout.region(l.lba, &t.root_clusters, dirs);
        assert!(if l.meta { matches!(r, Region::BootSector | Region::Fat | Region::Bitmap | Region::RootDir | Region::Dir) } else { r == Region::Data }, "{l:?} in {r:?}");
    }
    // the ground truth: every write's regions, the file's facts
    let mut boot0 = [0u8; 512];
    boot0.copy_from_slice(&volume("wb", 64 << 20, 4096)[2048 * 512..2049 * 512]);
    let w = writelog::exfat_writes(&t, &rec.disk.landings, &rec.disk.meta_landed, &boot0);
    assert_eq!(w.len(), rec.disk.landings.len());
    assert!(w.iter().all(|x| x.regions.iter().map(|r| r.1).sum::<u32>() == x.count));
    let boots: Vec<_> = w.iter().filter_map(|x| x.boot).collect();
    assert!(!boots.is_empty() && boots.iter().all(|b| b.1 == writelog::BootClass::State), "VolumeFlags only: {boots:?}");
    assert!(boots[0].2 && boots[0].0 == VOL_DIRTY, "the first, dirty and at once: {boots:?}");
    let facts = writelog::exfat_files(&t, &rec.disk.landings, &rec.disk.meta_landed);
    let ff = facts.iter().find(|x| x.path == p).unwrap();
    assert!(!ff.truth.is_chained() && ff.truth.fragments() == 1);
    let (d0, dall, s0, sfin) = (ff.data_first_ns.unwrap(), ff.data_all_ns.unwrap(), ff.set_first_ns.unwrap(), ff.set_final_ns.unwrap());
    assert!(d0 <= dall && s0 <= sfin && dall <= sfin, "data first {d0}, all {dall}; set first {s0}, final {sfin}");
}

/// Power gone mid-recording on exFAT: the medium keeps a dirty, torn volume; exfatprogs-style
/// fsck cuts what does not hold together, and a second run finds nothing.
#[test]
fn a_power_cut_leaves_an_exfat_volume_fsck_repairs() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("cut", 64 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut rec = Recorder::<_, ExFat>::on(disk, RecorderCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, ..RecorderCfg::default() });
    rec.mount().unwrap();
    let ps: Vec<String> = (0..4).map(|i| format!("HostData/ThirdGroup/c{i}.bin")).collect();
    let data: Vec<Vec<u8>> = ps.iter().map(|p| bmff_bytes(p, 2 << 20)).collect();
    let set: Vec<(&str, &[u8])> = ps.iter().map(|p| p.as_str()).zip(data.iter().map(|d| d.as_slice())).collect();
    rec.record_set(&set, 16_000_000).unwrap();
    rec.idle(3 * S).unwrap();
    assert!(rec.power_cut() > 0);
    let mut m = Offline { dev: std::mem::replace(&mut rec.disk.dev.dev, MemDisk(Vec::new())), now: 0 };
    let mut fs = ExFat::mount(&mut m).unwrap();
    assert!(fs.dirty(&mut m).unwrap(), "no clean unmount");
    let r = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(r.dirty_cleared, "{}", r.summary());
    let again = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(!again.dirty_cleared && again.truncated.is_empty() && again.resized.is_empty() && again.cross_or_broken.is_empty() && again.fat_sectors_changed == 0, "{}", again.summary());
    assert!(!fs.dirty(&mut m).unwrap());
}

/// A recorder's ring of files on exFAT: deleted entry sets leave their slots to the next file.
/// Thirteen sets of 18 entries (four or five per name) outgrow a 4 KiB cluster's 128 entries
/// once; the cluster after the folder's is a file's by then, so the folder becomes a FAT chain of
/// two and the ring stays in them. Each set of four sources written together is FAT-chained.
#[test]
fn an_exfat_ring_of_files_reuses_its_slots_and_chains_its_sets() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("ring", 256 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut rec = Recorder::<_, ExFat>::on(disk, RecorderCfg { poll_ns: 0, gap_ns: 0, linux_entries: true, dirty_flag: true, ..RecorderCfg::default() });
    rec.mount().unwrap();
    let streams = ["first", "aux1", "third_channel", "fourth_channel"];
    let mut ring = std::collections::VecDeque::new();
    for m in 0..120u32 {
        let paths: Vec<String> = streams.iter().map(|c| format!("HostData/ThirdGroup/2026-01-01_{:02}-{:02}-00-{c}.bin", m / 60, m % 60)).collect();
        let datas: Vec<Vec<u8>> = paths.iter().map(|p| bmff_bytes(p, 200_000)).collect();
        let set: Vec<(&str, &[u8])> = paths.iter().map(|p| p.as_str()).zip(datas.iter().map(|d| d.as_slice())).collect();
        rec.record_set(&set, 0).unwrap();
        ring.push_back(paths);
        if ring.len() > 12 { for p in ring.pop_front().unwrap() { rec.delete(&p).unwrap(); } }
    }
    rec.unmount().unwrap();
    let t = rec.truth.lock().unwrap().clone();
    let files: Vec<_> = t.files.iter().chain(t.past.iter().map(|(p, f)| (p, f))).filter(|(_, f)| !f.dir).collect();
    assert_eq!(files.len(), 480);
    assert!(files.iter().all(|(_, f)| f.is_chained()), "four files written together: every file chained");
    let mut m = std::mem::replace(&mut rec.disk.dev.dev, MemDisk(Vec::new()));
    let mut fs = ExFat::mount(&mut m).unwrap();
    let e = fs.stat(&mut m, "HostData/ThirdGroup").unwrap().unwrap();
    assert_eq!((e.chain.clusters, e.chain.flags), (2, ALLOC_FAT_CHAIN), "grown once, chained");
    assert_eq!(fs.list(&mut m, "HostData/ThirdGroup").unwrap().len(), 48);
    let r = fs.fsck(&mut m, FsckWrite::Changed).unwrap();
    assert!(r.truncated.is_empty() && r.resized.is_empty() && r.cross_or_broken.is_empty() && r.unreclaimed == 0, "{}", r.summary());
}

/// A directory past its cluster grows by one, hinted after its last; when that cluster is taken
/// the directory becomes a FAT chain and its own entry says so.
#[test]
fn a_directory_grows_and_chains_when_its_next_cluster_is_taken() {
    let mut d = MemDisk(volume("grow", 64 << 20, 4096));
    let mut fs = ExFat::mount(&mut d).unwrap();
    fs.mkdir_all(&mut d, "HostData/FirstGroup").unwrap();
    // a file right after the folder's cluster, then more names than its 128 entries hold
    let mut f = fs.create_path(&mut d, "HostData/FirstGroup/big.bin").unwrap();
    f.write(&mut fs, &mut d, &[7u8; 8192]).unwrap();
    f.close(&mut fs, &mut d).unwrap();
    for i in 0..45 {
        let f = fs.create_path(&mut d, &format!("HostData/FirstGroup/event-{i:03}.json")).unwrap();
        f.close(&mut fs, &mut d).unwrap();
    }
    let e = fs.stat(&mut d, "HostData/FirstGroup").unwrap().unwrap();
    assert_eq!((e.chain.clusters, e.chain.flags, e.size), (2, ALLOC_FAT_CHAIN, 8192));
    assert_eq!(fs.list(&mut d, "HostData/FirstGroup").unwrap().len(), 46);
    let mut m = MemDisk(d.0.clone());
    let mut fresh = ExFat::mount(&mut m).unwrap();
    assert_eq!(fresh.read_path(&mut m, "HostData/FirstGroup/big.bin").unwrap(), vec![7u8; 8192]);
    assert_eq!(fresh.list(&mut m, "HostData/FirstGroup").unwrap().len(), 46);
}

/// A recorder's tree on a 3 GiB exFAT card with 32 KiB clusters, written by a writeback recorder
/// with rotation and unmounted cleanly, checked by the system's exFAT checker (`sd_image::fsck`).
#[test]
#[ignore = "needs an exFAT checker: fsck_exfat with hdiutil (macOS), exfatprogs' fsck.exfat (Linux), or FSCK_EXFAT"]
fn external_exfat_tree_passes_fsck() {
    let img = std::env::temp_dir().join(format!("usbhost-exfat-fsck-{}.img", std::process::id()));
    sd_image::exfat(&img, 3 << 30, 32 << 10, "ESP32SIM").unwrap();
    let file = FileDisk(std::fs::OpenOptions::new().read(true).write(true).open(&img).unwrap());
    let mut rec = Recorder::<_, ExFat>::on(PageCache::writeback(Offline { dev: file, now: 0 }, WritebackCfg::default()), RecorderCfg { poll_ns: 0, gap_ns: 0, linux_entries: true, dirty_flag: true, ..RecorderCfg::default() });
    rec.mount().unwrap();
    let streams = ["first", "aux1", "third_channel", "fourth_channel"];
    let mut ring = std::collections::VecDeque::new();
    for m in 0..30u32 {
        let paths: Vec<String> = streams.iter().map(|c| format!("HostData/ThirdGroup/2026-01-01_00-{m:02}-00-{c}.bin")).collect();
        let datas: Vec<Vec<u8>> = paths.iter().map(|p| bmff_bytes(p, 700_000)).collect();
        let set: Vec<(&str, &[u8])> = paths.iter().map(|p| p.as_str()).zip(datas.iter().map(|d| d.as_slice())).collect();
        rec.record_set(&set, 0).unwrap();
        ring.push_back(paths);
        if ring.len() > 6 { for p in ring.pop_front().unwrap() { rec.delete(&p).unwrap(); } }
        if m % 10 == 9 {
            let ev = format!("HostData/FirstGroup/2026-01-01_00-{m:02}-00/entry.json");
            rec.record(&ev, b"{\"reason\":\"a_test_request\"}").unwrap();
        }
    }
    rec.unmount().unwrap();
    drop(rec);
    let r = sd_image::fsck(&img);
    let _ = std::fs::remove_file(&img);
    eprintln!("{}", r.as_ref().unwrap_or_else(|e| e));
    r.unwrap();
}

// ---- the recorder's hazards on exFAT, against the fake drive ----

fn exfat_disk(tag: &str) -> Vec<u8> { volume(tag, 64 << 20, 4096) }

/// The application's FatFs making a folder on an exFAT card in a window (here the recorder's own
/// writer stands in for it).
fn app_mkdir_exfat(path: &'static str) -> super::AppWrite {
    Box::new(move |disk: &mut Vec<u8>| {
        let mut m = MemDisk(std::mem::take(disk));
        let mut fs = ExFat::mount(&mut m).unwrap();
        fs.mkdir_all(&mut m, path).unwrap();
        *disk = m.0;
    })
}

/// `a_recorder_keeps_writing_into_a_window_until_its_next_poll` on exFAT: the writes up to the poll
/// reach the card; the cut file's entry has its first cluster and the size it had at its first
/// allocation, and the bitmap holds its clusters.
#[test]
fn an_exfat_recorder_keeps_writing_into_a_window_until_its_next_poll() {
    let mut dev = FakeMsc::new(0);
    dev.disk = exfat_disk("window");
    dev.windows.push(FakeWindow { from: S / 2, to: 3 * S, app: None });
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut rec = Recorder::<_, ExFat>::on(PageCache::new(m), RecorderCfg::default());
    rec.mount().unwrap();
    let r = rec.record("HostData/ThirdGroup/file.bin", &bmff_bytes("event", 8 << 20)).unwrap();
    let Recorded::MediumGone { written } = r else { panic!("{r:?}") };
    assert!(written > 6 << 20 && written < 8 << 20, "{written}");
    rec.idle(2 * S).unwrap();
    assert!(rec.present && rec.stats.mounts == 2 && rec.stats.media_changes == 2, "{:?}", rec.stats);
    drop(rec);
    let mut mem = MemDisk(port.dev.disk.clone());
    let mut fs = ExFat::mount(&mut mem).unwrap();
    let e = fs.stat(&mut mem, "HostData/ThirdGroup/file.bin").unwrap().unwrap();
    assert_eq!((e.chain.flags, e.size), (ALLOC_NO_FAT_CHAIN, 0), "the first allocation's entry: NoFatChain, size 0");
    assert!(e.chain.first >= 2);
}

/// The gated window on exFAT: the recorder refused at once, gone at its poll, back at the unit
/// attention, the volume read again.
#[test]
fn a_gated_window_refuses_an_exfat_recorder_and_its_unit_attention_revalidates() {
    let mut dev = FakeMsc::new(0);
    dev.disk = exfat_disk("gated");
    dev.windows.push(FakeWindow { from: S / 2, to: 3 * S, app: None });
    dev.gated = true;
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut rec = Recorder::<_, ExFat>::on(PageCache::new(m), RecorderCfg::default());
    rec.mount().unwrap();
    let r = rec.record("HostData/ThirdGroup/file.bin", &bmff_bytes("event", 8 << 20)).unwrap();
    assert!(matches!(r, Recorded::WriteFailed { .. }), "{r:?}");
    rec.idle(4_500_000_000).unwrap();
    assert_eq!((rec.stats.unit_attentions, rec.stats.media_changes, rec.stats.mounts), (1, 2, 2), "{:?}", rec.stats);
    drop(rec);
    assert!(port.dev.write_at.iter().all(|&t| !(S / 2..3 * S).contains(&t)), "no WRITE(10) reached the card in the window");
}

/// An exFAT recorder between polls: a window's folder is lost to a recorder that does not see the
/// window, caching its metadata or not, because the driver keeps the allocation bitmap from the
/// mount whatever the cache does; it is kept when a gated window's unit attention makes the
/// recorder read the volume again.
#[test]
fn an_exfat_recorder_and_a_window_between_its_polls() {
    for (keep, gated) in [(true, false), (false, false), (true, true)] {
        let mut dev = FakeMsc::new(0);
        dev.disk = exfat_disk("short");
        dev.windows.push(FakeWindow { from: S, to: 1_500_000_000, app: Some(app_mkdir_exfat("HostData/FirstGroup")) });
        dev.gated = gated;
        let mut port = FakePort::new(dev);
        let m = msc(&mut port);
        let disk = if keep { PageCache::keeping_meta(m) } else { PageCache::new(m) };
        let mut rec = Recorder::<_, ExFat>::on(disk, RecorderCfg { poll_ns: 5 * S, ..RecorderCfg::default() });
        rec.mount().unwrap();
        assert!(rec.fs.clone().unwrap().stat(&mut rec.disk, "HostData").unwrap().is_none());
        rec.idle(6 * S).unwrap();
        let fs = rec.fs.as_mut().unwrap();
        fs.mkdir_all(&mut rec.disk, "HostData/ThirdGroup").unwrap();
        drop(rec);
        let mut mem = MemDisk(port.dev.disk.clone());
        let mut fs = ExFat::mount(&mut mem).unwrap();
        let names: Vec<String> = fs.list(&mut mem, "HostData").unwrap().into_iter().map(|e| e.name).collect();
        eprintln!("keep {keep}, gated {gated}: HostData holds {names:?}");
        if gated {
            assert_eq!(names, ["FirstGroup", "ThirdGroup"], "the unit attention: the volume and its bitmap read again");
        } else if keep {
            assert_eq!(names, ["ThirdGroup"], "the recorder's cached root and bitmap: its HostData took the window's slot and cluster");
        } else {
            assert!(!names.iter().any(|n| n == "FirstGroup"), "the root read again but the bitmap kept: the window's clusters taken again");
        }
    }
}

/// The fsck burst at a mount on exFAT: a recorder that lost power left the volume dirty, so the
/// next recorder's mount runs fsck (exfatprogs 1.0.4's way) before it reads the volume, and the
/// volume it mounts is clean.
#[test]
fn an_exfat_recorder_runs_fsck_at_the_mount_of_a_dirty_volume() {
    let disk = PageCache::writeback(Offline { dev: MemDisk(volume("fsck-mount", 64 << 20, 4096)), now: 0 }, WritebackCfg::default());
    let mut rec = Recorder::<_, ExFat>::on(disk, RecorderCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, ..RecorderCfg::default() });
    rec.mount().unwrap();
    for i in 0..2 { rec.record(&format!("HostData/ThirdGroup/c{i}.bin"), &bmff_bytes(&format!("c{i}"), 2 << 20)).unwrap(); }
    rec.idle(2 * S).unwrap();
    rec.power_cut();
    let dev = std::mem::replace(&mut rec.disk.dev.dev, MemDisk(Vec::new()));
    let mut next = Recorder::<_, ExFat>::on(PageCache::writeback(Offline { dev, now: 0 }, WritebackCfg::default()), RecorderCfg { poll_ns: 0, linux_entries: true, dirty_flag: true, ..RecorderCfg::default() });
    next.fsck_at_mount = Some(FsckWrite::Changed);
    next.mount().unwrap();
    let (_, _, r) = next.last_fsck.clone().expect("fsck ran at the mount");
    assert!(r.dirty_cleared && r.bitmap_bytes_read > 0, "{}", r.summary());
    assert!(!next.fs.clone().unwrap().dirty(&mut next.disk).unwrap());
    assert!(matches!(next.record("HostData/ThirdGroup/c2.bin", &bmff_bytes("c2", 1 << 20)).unwrap(), Recorded::Closed { .. }));
}

/// A USB reset, then the recorder's fsck with the FAT and bitmap written whole (`FsckWrite::Bulk`,
/// the assumption the FAT32 test makes), on exFAT: every FAT sector and every bitmap sector
/// written, and the volume read again.
#[test]
fn a_reset_then_a_bulk_fsck_rewrites_the_exfat_fat_and_bitmap_whole() {
    let mut dev = FakeMsc::new(0);
    dev.disk = exfat_disk("bulk-fsck");
    let mut port = FakePort::new(dev);
    let m = msc(&mut port);
    let mut rec = Recorder::<_, ExFat>::on(PageCache::new(m), RecorderCfg { poll_ns: 0, gap_ns: 0, ..RecorderCfg::default() });
    rec.mount().unwrap();
    rec.record("HostData/ThirdGroup/a.bin", &bmff_bytes("a", 300_000)).unwrap();
    rec.disk.dev.host.reset_device().unwrap();
    let l = rec.fs.clone().unwrap().layout;
    let r = rec.fsck_now(FsckWrite::Bulk).unwrap();
    assert_eq!((r.files, r.fat_bytes_written, r.bitmap_bytes_written), (1, l.fat_sectors as u64 * 512, l.bitmap_bytes.next_multiple_of(512)), "{}", r.summary());
    assert_eq!(rec.read("HostData/ThirdGroup/a.bin").unwrap(), bmff_bytes("a", 300_000));
    drop(rec);
    let blocks = |lo: u64, n: u64| port.dev.writes.iter().filter(|x| (lo..lo + n).contains(&(x.0 as u64))).map(|x| x.1 as u64).sum::<u64>();
    assert!(blocks(l.fat_lba, l.fat_sectors as u64) >= l.fat_sectors as u64);
}

/// A card filled but for `leave` bytes, as a recorder keeps one, on either filesystem: one file in
/// the root over every other free cluster, the free count what was left, and the next file taking
/// from what was left.
#[test]
fn a_fresh_volume_fills_but_for_what_is_left_on_either_filesystem() {
    fn check<F: Fill>(mut d: MemDisk, leave: u64) {
        F::fill(&mut d, leave).unwrap();
        let mut fs = F::mount(&mut d).unwrap();
        let cb = fs.cluster_bytes() as u64;
        let filler = fs.list(&mut d, "").unwrap().into_iter().find(|e| e.name == "FILLER.BIN").unwrap_or_else(|| panic!("{}: no FILLER.BIN", F::NAME));
        let clusters = fs.clusters_of(&mut d, "FILLER.BIN").unwrap();
        assert_eq!(filler.size as u64, clusters.len() as u64 * cb, "{}", F::NAME);
        assert!(clusters.windows(2).all(|w| w[1] == w[0] + 1), "{}: one run", F::NAME);
        let mut f = fs.create_path(&mut d, "next.bin").unwrap();
        f.write(&mut fs, &mut d, &vec![7u8; leave as usize]).unwrap();
        f.close(&mut fs, &mut d).unwrap();
        let mut f = fs.create_path(&mut d, "more.bin").unwrap();
        assert!(f.write(&mut fs, &mut d, &vec![7u8; cb as usize]).is_err(), "{}: nothing left past `leave`", F::NAME);
    }
    let fat = || { let img = std::env::temp_dir().join(format!("usbhost-fill-fat32-{}.img", std::process::id())); sd_image::fat32(&img, 64 << 20, 512, "").unwrap(); let d = std::fs::read(&img).unwrap(); let _ = std::fs::remove_file(&img); MemDisk(d) };
    check::<Fat32>(fat(), 64 << 10);
    check::<ExFat>(MemDisk(volume("fill", 64 << 20, 4096)), 64 << 10);
}
