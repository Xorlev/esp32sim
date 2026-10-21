//! A car on the drive: it records clips as its cameras feed them and polls the drive between
//! writes the way Linux's sd driver does, and it keeps what it believes each file holds, so what
//! reached the card, or an archive, can be compared with it.
//!
//! The poll follows `sd_check_events` (Linux v6.12 drivers/scsi/sd.c:1710-1776): a TEST UNIT READY
//! that answers NOT READY or UNIT ATTENTION with MEDIUM NOT PRESENT (ASC 3Ah) marks the medium gone
//! (`media_not_present`, 1682-1699), and a change either way, or any UNIT ATTENTION, is a medium
//! change. A medium gone has capacity 0 (`set_media_not_present`, 1671-1680), so the writes the car
//! still holds cannot reach it: they are dropped. When the medium is back the car reads the volume
//! again. Between polls nothing tells the car the medium went, so its writes keep going to the
//! drive, which may accept them (esp_tinyusb's WRITE(10) callback does not look at who holds the
//! card).
//!
//! What the car keeps of the volume between polls, and when its writes land, is its
//! `pagecache::PageCache`.
use super::fat32::{BlockDevice, Fat32, FsckReport, FsckWrite};
use super::msc::{sense, Msc, MscError, Sense};
use super::pagecache::PageCache;
use std::collections::BTreeMap;

/// What a TEST UNIT READY said.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ready { Ready, Check(Sense), Failed(String) }

/// A drive the car can poll.
pub trait Drive: BlockDevice {
    fn test_unit_ready(&mut self) -> Ready;
    /// READ CAPACITY, before a volume is read.
    fn capacity(&mut self) -> Result<(), String> { Ok(()) }
}

impl Drive for Msc<'_> {
    fn test_unit_ready(&mut self) -> Ready {
        match Msc::test_unit_ready(self) { Ok(()) => Ready::Ready, Err(MscError::Check(s)) => Ready::Check(s), Err(e) => Ready::Failed(e.to_string()) }
    }
    fn capacity(&mut self) -> Result<(), String> { self.read_capacity().map(|_| ()).map_err(|e| e.to_string()) }
}

/// A block device with a clock of its own and a medium always there, for a car run without
/// USB (a card image prepared before a boot, unit tests).
pub struct Offline<D> { pub dev: D, pub now: u64 }
impl<D: BlockDevice> BlockDevice for Offline<D> {
    fn block_size(&self) -> u32 { self.dev.block_size() }
    fn blocks(&self) -> u64 { self.dev.blocks() }
    fn read_blocks(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), String> { self.dev.read_blocks(lba, buf) }
    fn write_blocks(&mut self, lba: u64, data: &[u8]) -> Result<(), String> { self.dev.write_blocks(lba, data) }
    fn pause_ns(&mut self, ns: u64) -> Result<(), String> { self.now += ns; Ok(()) }
    fn now_ns(&mut self) -> u64 { self.now }
}
impl<D: BlockDevice> Drive for Offline<D> { fn test_unit_ready(&mut self) -> Ready { Ready::Ready } }

#[derive(Clone, Copy, Debug)]
pub struct CarCfg {
    /// TEST UNIT READY this often while recording and idling (udev polls a removable disk every
    /// 2 s: systemd v256 rules.d/60-block.rules)
    pub poll_ns: u64,
    /// bytes per write, and the camera's time between them (64 KiB every 16 ms is 4.1 MB/s)
    pub chunk: usize,
    pub gap_ns: u64,
    /// Linux's order for a file being written (`FileWriter::linux`)
    pub linux_entries: bool,
    /// the volume's dirty flag set at a mount and cleared at a clean unmount, as Linux does it
    pub dirty_flag: bool,
}
impl Default for CarCfg {
    fn default() -> Self { CarCfg { poll_ns: 2_000_000_000, chunk: 64 << 10, gap_ns: 16_000_000, linux_entries: false, dirty_flag: false } }
}

/// What the car believes a file holds.
#[derive(Clone, Debug)]
pub struct CarFile {
    /// the clip the camera made
    pub data: Vec<u8>,
    /// bytes the car wrote before it stopped
    pub written: u64,
    /// closed: every chunk written and the entry final, as far as the car knows
    pub complete: bool,
    pub deleted: bool,
    pub started_ns: u64,
    pub closed_ns: Option<u64>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CarStats {
    pub polls: u64,
    pub not_ready: u64,
    pub media_changes: u64,
    pub unit_attentions: u64,
    pub mounts: u64,
    pub chunks: u64,
    pub bytes: u64,
    /// clips the car stopped writing because the medium went or a write failed
    pub cut: u64,
    pub deleted: u64,
}

/// How a recording ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Recorded {
    Closed { bytes: u64, worst_late_ns: u64 },
    /// a poll said the medium went: the chunks after it were never written
    MediumGone { written: u64 },
    WriteFailed { written: u64, error: String },
}

pub struct Car<D: Drive> {
    pub disk: PageCache<D>,
    pub fs: Option<Fat32>,
    pub cfg: CarCfg,
    pub present: bool,
    pub next_poll_ns: u64,
    pub files: BTreeMap<String, CarFile>,
    pub stats: CarStats,
    /// what happened, with the time
    pub log: Vec<(u64, String)>,
    /// print the log as it grows (`[car] t=...`)
    pub echo: bool,
    /// at a mount of a volume marked dirty: run fsck first, writing the FATs this way
    pub fsck_at_mount: Option<FsckWrite>,
    pub last_fsck: Option<(u64, u64, FsckReport)>,
}

impl<D: Drive> Car<D> {
    /// A car on `disk`; nothing read yet.
    pub fn new(disk: PageCache<D>, cfg: CarCfg) -> Self {
        Car { disk, fs: None, cfg, present: true, next_poll_ns: 0, files: BTreeMap::new(), stats: CarStats::default(), log: Vec::new(), echo: false, fsck_at_mount: None, last_fsck: None }
    }

    pub fn now(&mut self) -> u64 { self.disk.now_ns() }

    pub fn say(&mut self, what: String) {
        let t = self.now();
        if self.echo { eprintln!("[car] t={:.3} {what}", t as f64 / 1e9); }
        self.log.push((t, what));
    }

    /// Read the volume, after an fsck when it is marked dirty and the car runs one; with the
    /// dirty flag kept, mark it in use.
    pub fn mount(&mut self) -> Result<(), String> {
        self.disk.drop_dirty();
        self.disk.invalidate();
        self.disk.dev.capacity()?;
        let mut fs = Fat32::mount(&mut self.disk)?;
        if let Some(mode) = self.fsck_at_mount {
            if fs.dirty(&mut self.disk)? {
                let t0 = self.now();
                let r = fs.fsck(&mut self.disk.dev, mode)?;
                let t1 = self.now();
                self.disk.invalidate();
                fs = Fat32::mount(&mut self.disk)?;
                self.say(format!("fsck in {:.3} s: {}", (t1 - t0) as f64 / 1e9, r.summary()));
                self.last_fsck = Some((t0, t1, r));
            }
        }
        if self.cfg.dirty_flag { fs.set_dirty(&mut self.disk.dev, true)?; self.disk.invalidate(); }
        self.stats.mounts += 1;
        self.fs = Some(fs);
        let t = self.now();
        self.next_poll_ns = t + self.cfg.poll_ns;
        Ok(())
    }

    /// Land everything held and clear the dirty flag: a clean unmount.
    pub fn unmount(&mut self) -> Result<(), String> {
        self.disk.sync()?;
        if let Some(fs) = self.fs.take() { if self.cfg.dirty_flag { fs.set_dirty(&mut self.disk.dev, false)?; } }
        self.disk.invalidate();
        Ok(())
    }

    /// Power gone with the car: what it held is lost, the medium keeps what had landed.
    pub fn power_cut(&mut self) -> usize {
        let lost = self.disk.drop_dirty();
        self.disk.invalidate();
        self.fs = None;
        self.say(format!("power cut: {lost} dirty pages lost"));
        lost
    }

    /// One TEST UNIT READY, and what the car makes of it.
    pub fn poll(&mut self) -> Result<Ready, String> {
        self.stats.polls += 1;
        let r = self.disk.dev.test_unit_ready();
        let t = self.now();
        self.next_poll_ns = t + self.cfg.poll_ns;
        match &r {
            Ready::Ready if !self.present => {
                self.present = true;
                self.stats.media_changes += 1;
                self.say("TEST UNIT READY: ready again, a medium change: the volume read again".into());
                self.mount()?;
            }
            Ready::Ready => {}
            Ready::Check(s) if s.asc == sense::MEDIUM_NOT_PRESENT && matches!(s.key, sense::NOT_READY | sense::UNIT_ATTENTION) => {
                self.stats.not_ready += 1;
                if self.present {
                    self.present = false;
                    self.stats.media_changes += 1;
                    let lost = self.disk.drop_dirty();
                    self.disk.invalidate();
                    self.fs = None;
                    self.say(format!("TEST UNIT READY: {s}: medium gone, {lost} dirty pages dropped"));
                }
            }
            Ready::Check(s) if s.key == sense::UNIT_ATTENTION => {
                self.stats.unit_attentions += 1;
                self.stats.media_changes += 1;
                self.say(format!("TEST UNIT READY: {s}: a medium change: the volume read again"));
                self.mount()?;
            }
            Ready::Check(s) => self.say(format!("TEST UNIT READY: {s}")),
            Ready::Failed(e) => self.say(format!("TEST UNIT READY failed: {e}")),
        }
        Ok(r)
    }

    /// Let time pass to `t_ns`, polling as due; the writeback lands meanwhile.
    pub fn wait_until(&mut self, t_ns: u64) -> Result<(), String> {
        loop {
            let now = self.now();
            if self.cfg.poll_ns > 0 && now >= self.next_poll_ns { self.poll()?; continue; }
            if now >= t_ns { return Ok(()); }
            let next = if self.cfg.poll_ns > 0 { t_ns.min(self.next_poll_ns) } else { t_ns };
            self.disk.pause_ns(next - now)?;
        }
    }

    pub fn idle(&mut self, ns: u64) -> Result<(), String> { let t = self.now() + ns; self.wait_until(t) }

    /// Record `data` at `path` (its directories made as needed) as a camera feeds it, a chunk
    /// due every `gap_ns` from the start, polling between chunks. A poll that finds the medium
    /// gone ends the clip there; so does a write that fails.
    pub fn record(&mut self, path: &str, data: &[u8]) -> Result<Recorded, String> { self.record_set(&[(path, data)], self.cfg.gap_ns) }

    /// Record several clips at once, as a car's cameras do: the files made first, then chunk i of
    /// every clip, clip after clip, due at t0 + i * `gap_ns`, the files closed at the end. A medium
    /// gone or a failed write ends them all there.
    pub fn record_set(&mut self, clips: &[(&str, &[u8])], gap_ns: u64) -> Result<Recorded, String> {
        if !self.present || self.fs.is_none() { return Err("no volume".into()); }
        let t0 = self.now();
        let mut ws = Vec::new();
        for (path, data) in clips {
            let fs = self.fs.as_mut().unwrap();
            if let Some((dir, _)) = path.rsplit_once('/') { fs.mkdir_all(&mut self.disk, dir).map_err(|e| format!("{dir}: {e}"))?; }
            let mut w = fs.create_path(&mut self.disk, path).map_err(|e| format!("{path}: {e}"))?;
            w.linux = self.cfg.linux_entries;
            ws.push(w);
            self.files.insert(path.to_string(), CarFile { data: data.to_vec(), written: 0, complete: false, deleted: false, started_ns: t0, closed_ns: None });
        }
        let cb = self.fs.as_ref().unwrap().cluster_bytes();
        let chunk = self.cfg.chunk.max(cb) / cb * cb;
        let n = clips.iter().map(|c| c.1.len().div_ceil(chunk)).max().unwrap_or(0);
        let mut worst = 0;
        let mut cut = None;
        'chunks: for i in 0..n {
            let due = t0 + i as u64 * gap_ns;
            self.wait_until(due)?;
            if !self.present || self.fs.is_none() { cut = Some(Recorded::MediumGone { written: ws.iter().map(|w| w.size()).sum() }); break; }
            let now = self.now();
            worst = worst.max(now.saturating_sub(due));
            for (k, (path, data)) in clips.iter().enumerate() {
                let Some(part) = data.chunks(chunk).nth(i) else { continue };
                let fs = self.fs.as_mut().unwrap();
                if let Err(e) = ws[k].write(fs, &mut self.disk, part) { cut = Some(Recorded::WriteFailed { written: ws.iter().map(|w| w.size()).sum(), error: e }); break 'chunks; }
                self.stats.chunks += 1;
                self.stats.bytes += part.len() as u64;
                self.files.get_mut(*path).unwrap().written = ws[k].size();
            }
        }
        if let Some(r) = cut {
            self.stats.cut += clips.len() as u64;
            let first = clips.first().map_or("", |c| c.0).to_string();
            self.say(format!("{first} and {} more: cut, {r:?}", clips.len() - 1));
            return Ok(r);
        }
        let mut bytes = 0;
        for (w, (path, _)) in ws.into_iter().zip(clips) {
            let fs = self.fs.as_mut().unwrap();
            bytes += w.close(fs, &mut self.disk)?;
            let t = self.now();
            let f = self.files.get_mut(*path).unwrap();
            f.complete = true;
            f.closed_ns = Some(t);
        }
        Ok(Recorded::Closed { bytes, worst_late_ns: worst })
    }

    /// fsck now, whatever the dirty flag says, then the volume read again, the pages held written
    /// first: what the car is assumed to do after a USB reset in the shared-reading plan's
    /// experiment. The FATs are written as `mode` says.
    pub fn fsck_now(&mut self, mode: FsckWrite) -> Result<FsckReport, String> {
        self.disk.sync()?;
        self.disk.invalidate();
        let mut fs = Fat32::mount(&mut self.disk.dev)?;
        let t0 = self.now();
        let r = fs.fsck(&mut self.disk.dev, mode)?;
        let t1 = self.now();
        self.say(format!("fsck in {:.3} s: {}", (t1 - t0) as f64 / 1e9, r.summary()));
        self.last_fsck = Some((t0, t1, r.clone()));
        self.mount()?;
        Ok(r)
    }

    /// Delete `path`, as the car's rotation does with its oldest clip.
    pub fn delete(&mut self, path: &str) -> Result<usize, String> {
        let fs = self.fs.as_mut().ok_or("no volume")?;
        let freed = fs.delete_path(&mut self.disk, path)?.len();
        if let Some(f) = self.files.get_mut(path) { f.deleted = true; }
        self.stats.deleted += 1;
        Ok(freed)
    }

    /// The car's view of the file at `path`, read through its own cache.
    pub fn read(&mut self, path: &str) -> Result<Vec<u8>, String> {
        let fs = self.fs.clone().ok_or("no volume")?;
        fs.read_path(&mut self.disk, path)
    }
}
