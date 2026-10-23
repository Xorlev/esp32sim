//! oxidrive's per-boot car report (its `car_report` module): figures about the car kept in NVS
//! through a drive with no WiFi and the power cut that ends it, published to MQTT when the device
//! is next home. The firmware is `OXIDRIVE_CAR_REPORT_DIR` (the `oxidrive-car-report` fixture by
//! default); `OXIDRIVE_BASELINE_DIR` (`oxidrive-bc40b82`) is a build without the report, for the
//! cost comparison.
//!
//! The P4's flash takes time to program and erase here (`SocBus::flash_timing`, GD25Q-class
//! typical figures: an assumption, not a measurement), so an NVS write holds the flash as long as
//! it would on the board, and IDF's own code stalls the other core meanwhile. The emulator does
//! not model the cache, so a task running from flash during a flash operation is not caught; the
//! stall IDF imposes in software is. The SD bus is timed and the card programs for
//! `OXIDRIVE_SD_PRG_US` (1500 us) after each write, every 97th write for 30 ms and every 401st for
//! 150 ms in the first test. A power cut is a P4 reset with the power-on cause: RAM survives it
//! here, which the firmware never relies on.
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
use esp_periph::spi_mem::{FlashOp, FlashTiming};
use esp_soc::json::{parse_json, Json};
use esp_soc::usb_host::car::{Car, CarCfg, Recorded};
use esp_soc::usb_host::fat32::mp4_bytes;
use esp_soc::usb_host::msc::Msc;
use esp_soc::usb_host::pagecache::PageCache;
use esp_soc::usb_host::timing::CmdTime;
use esp_soc::usb_host::UsbHost;
use oxidrive_rig::*;
use std::time::Duration;

const S: u64 = 1_000_000_000;
const MS: u64 = 1_000_000;

fn env_u64(name: &str, default: u64) -> u64 { std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default) }
fn secs(ns: u64) -> String { format!("{:.3} s", ns as f64 / 1e9) }
fn fixture(var: &str, name: &str) -> String {
    std::env::var(var).unwrap_or_else(|_| format!("{}/Code/dlp/sim-fixtures/esp32p4/{name}", std::env::var("HOME").unwrap_or_default()))
}
fn report_dir() -> String { fixture("OXIDRIVE_CAR_REPORT_DIR", "oxidrive-car-report") }
fn baseline_dir() -> String { fixture("OXIDRIVE_BASELINE_DIR", "oxidrive-bc40b82") }

/// GD25Q-class typical times, `OXIDRIVE_FLASH_ERASE_MS` and `OXIDRIVE_FLASH_BYTE_NS` to vary them.
fn flash_timing() -> FlashTiming {
    let t = FlashTiming::GD25Q_TYPICAL;
    FlashTiming { sector_ns: env_u64("OXIDRIVE_FLASH_ERASE_MS", t.sector_ns / MS) * MS, program_byte_ns: env_u64("OXIDRIVE_FLASH_BYTE_NS", t.program_byte_ns), ..t }
}

/// The firmware's buckets (oxidrive-core `car_report::WRITE_US` and `TUR_MS`), in ms.
const WRITE_MS: [u64; 6] = [10, 25, 50, 100, 250, 500];
const TUR_MS: [u64; 6] = [500, 1_000, 2_000, 5_000, 10_000, 30_000];
fn bucket(edges: &[u64; 6], v: u64) -> usize { edges.iter().position(|&e| v <= e).unwrap_or(6) }

fn rig_on(dir: String, card: &str, slow: [(u64, u64); 2]) -> Rig {
    let prg = env_u64("OXIDRIVE_SD_PRG_US", 1500);
    eprintln!("[report] {dir}: card programming {prg} us, slow {slow:?}, flash {:?}", flash_timing());
    rig_with(RigOpts { card_name: card.into(), card_bytes: env_u64("OXIDRIVE_CARD_MIB", 4096) << 20, car: true, prg_us: Some(prg), timed: Some(true), slow, sd_writes: true, flash_timing: Some(flash_timing()), ..RigOpts::new(dir) })
}

/// A car on the port with every SCSI command it sends timed from before its first.
fn car_timed<'p>(r: &'p mut Rig, cfg: CarCfg) -> Car<Msc<'p>> {
    let mut host = UsbHost::new(r);
    host.log = std::env::var("OXIDRIVE_USB_LOG").is_ok_and(|v| v == "1");
    host.enumerate(10 * S).unwrap_or_else(|e| panic!("enumerate: {e}"));
    let mut msc = Msc::open(host).unwrap_or_else(|e| panic!("no mass-storage interface: {e}"));
    msc.times = Some(Vec::new());
    msc.wait_ready(100 * MS, 10 * S).unwrap_or_else(|e| panic!("never ready: {e}"));
    let mut car = Car::new(PageCache::keeping_meta(msc), cfg);
    car.echo = std::env::var("OXIDRIVE_CAR_ECHO").is_ok_and(|v| v == "1");
    car.mount().unwrap_or_else(|e| panic!("mount: {e}"));
    car
}

fn idle_until(car: &mut Car<Msc<'_>>, shared: &std::rc::Rc<std::cell::RefCell<Shared>>, what: &str, deadline_ns: u64) -> Option<u64> {
    while car.now() < deadline_ns {
        if let Some(t) = shared.borrow().all(what).last().copied() { return Some(t); }
        car.idle(100 * MS).unwrap_or_else(|e| panic!("idle: {e}"));
    }
    None
}

fn record(car: &mut Car<Msc<'_>>, path: &str, mib: usize) {
    let st = car.record(path, &mp4_bytes(path, mib << 20)).unwrap_or_else(|e| panic!("{path}: {e}"));
    assert!(matches!(st, Recorded::Closed { .. }), "{path}: {st:?}");
}

/// Run without a car until the console holds `n` lines with `what`.
fn run_until_lines(r: &mut Rig, what: &str, n: usize, deadline_ns: u64) -> bool {
    while r.cs.now_ns < deadline_ns {
        if r.p4_lines().iter().filter(|l| l.contains(what)).count() >= n { return true; }
        let t = r.cs.now_ns + 500 * MS;
        r.run_to(t);
    }
    false
}

/// The P4 loses power and comes back: a reset with the power-on cause.
fn power_cut(r: &mut Rig) {
    esp_soc::SocBus::request_reset(&mut r.cs.p4.bus, esp_periph::RST_POWERON);
    let t = r.cs.now_ns + 10 * MS;
    r.run_to(t);
}

/// The retained reports, parsed; each document is printed whole for `bin/car-report file`.
fn reports(r: &Rig) -> Vec<(String, Json)> {
    let topic = format!("oxidrive/{}/car_report/+", r.id);
    r.mqtt.client.retained(&[&topic], Duration::from_secs(1)).unwrap().into_iter()
        .map(|(t, p)| {
            eprintln!("[report] {t} {p}");
            let j = parse_json(&p).unwrap_or_else(|e| panic!("{t}: {e}: {p}"));
            (t, j)
        }).collect()
}

fn num(j: &Json, path: &str) -> i64 {
    let mut at = j;
    for k in path.split('.') { at = at.get(k).unwrap_or_else(|| panic!("no {path} in the report")); }
    at.as_i64().unwrap_or_else(|| panic!("{path} is not a number"))
}
fn hist(j: &Json, path: &str) -> Vec<i64> {
    let mut at = j;
    for k in path.split('.') { at = at.get(k).unwrap_or_else(|| panic!("no {path} in the report")); }
    at.as_arr().unwrap_or_else(|| panic!("{path} is not an array")).iter().map(|v| v.as_i64().unwrap()).collect()
}

/// Flash operations close together, as one NVS write makes them: (start, end, bytes programmed,
/// erases).
fn bursts(ops: &[FlashOp], from_ns: u64, to_ns: u64) -> Vec<(u64, u64, u64, u32)> {
    let mut out: Vec<(u64, u64, u64, u32)> = Vec::new();
    for op in ops.iter().filter(|o| o.start_ns >= from_ns && o.start_ns < to_ns) {
        let (bytes, erases) = if op.erase { (0, 1) } else { (op.len as u64, 0) };
        match out.last_mut() {
            Some(b) if op.start_ns <= b.1 + 5 * MS => { b.1 = b.1.max(op.end_ns); b.2 += bytes; b.3 += erases; }
            _ => out.push((op.start_ns, op.end_ns, bytes, erases)),
        }
    }
    out
}

/// Car commands in flight while the flash was busy, and how much longer than the median of their
/// kind each took.
fn held_by_flash(times: &[CmdTime], ops: &[FlashOp]) -> Vec<(CmdTime, u64)> {
    let mut medians = std::collections::HashMap::new();
    for op in [0x00u8, 0x03, 0x28, 0x2a, 0x25, 0x12, 0x1a] {
        let mut v: Vec<u64> = times.iter().filter(|t| t.op == op).map(|t| t.latency_ns()).collect();
        v.sort_unstable();
        if !v.is_empty() { medians.insert(op, v[v.len() / 2]); }
    }
    times.iter().filter(|t| ops.iter().any(|o| o.start_ns < t.done_ns && o.end_ns > t.submit_ns))
        .map(|t| (*t, t.latency_ns().saturating_sub(*medians.get(&t.op).unwrap_or(&0)))).collect()
}

/// What a boot's car did, from the emulator's side, up to `until_ns`.
struct Truth { tur: usize, sense: usize, capacity: usize, inquiry: usize, tur_gaps: [i64; 7], tur_min_ms: u64, near_edge: usize,
               not_present: usize, attention: usize, no_sense: usize, other_sense: usize, writes: usize, sectors: u64, write_hist: [i64; 7], write_max_ns: u64 }

fn truth(times: &[CmdTime], sd: &[esp_periph::sdmmc::SdWrite], until_ns: u64) -> Truth {
    let t: Vec<&CmdTime> = times.iter().filter(|c| c.submit_ns < until_ns).collect();
    let count = |ops: &[u8]| t.iter().filter(|c| ops.contains(&c.op)).count();
    let turs: Vec<u64> = t.iter().filter(|c| c.op == 0x00).map(|c| c.submit_ns).collect();
    let (mut tur_gaps, mut near_edge) = ([0i64; 7], 0);
    for w in turs.windows(2) {
        let ms = (w[1] - w[0]) / MS;
        tur_gaps[bucket(&TUR_MS, ms)] += 1;
        if TUR_MS.iter().any(|&e| e.abs_diff(ms) <= 2) { near_edge += 1; }
    }
    let senses: Vec<(u8, u8, u8)> = t.iter().filter_map(|c| c.sense).collect();
    let writes: Vec<_> = sd.iter().filter(|w| w.start_ns < until_ns).collect();
    let mut write_hist = [0i64; 7];
    for w in &writes { write_hist[bucket(&WRITE_MS, (w.end_ns - w.start_ns) / MS)] += 1; }
    Truth {
        tur: turs.len(), sense: count(&[0x03]), capacity: count(&[0x25, 0x23]), inquiry: count(&[0x12]),
        tur_gaps, tur_min_ms: turs.windows(2).map(|w| (w[1] - w[0]) / MS).min().unwrap_or(0), near_edge,
        not_present: senses.iter().filter(|s| **s == (2, 0x3a, 0)).count(), attention: senses.iter().filter(|s| **s == (6, 0x28, 0)).count(),
        no_sense: senses.iter().filter(|s| **s == (0, 0, 0)).count(),
        other_sense: senses.iter().filter(|s| ![(2, 0x3a, 0), (6, 0x28, 0), (0, 0, 0)].contains(*s)).count(),
        writes: writes.len(), sectors: writes.iter().map(|w| w.blocks as u64).sum(), write_hist,
        write_max_ns: writes.iter().map(|w| w.end_ns - w.start_ns).max().unwrap_or(0),
    }
}

/// The report survives a power cut and says what the car did. A car records four saved events on
/// a card whose writes stall now and then, polls TEST UNIT READY every 1.5 s, meets a forced
/// archive window and records again after it, then idles past the minute after which the report
/// is saved. The P4 then loses power before it publishes anything about this boot; it boots again,
/// with no car, and publishes the first boot's report. Asserted against the emulator: the SCSI
/// command counts, the sense data returned, the TUR poll gaps bucketed, the car's writes and their
/// service times bucketed, the gaps after the window. A third boot publishes the second boot's
/// report and not the first one's again.
#[test]
#[ignore = "needs the oxidrive car-report images and nvs.csv under OXIDRIVE_CAR_REPORT_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_car_report_survives_a_power_cut() {
    let mut r = rig_on(report_dir(), "car-report.img", [(97, 30 * MS), (401, 150 * MS)]);
    let shared = r.shared.clone();
    let (cmd, mqtt) = (format!("oxidrive/{}/cmd", r.id), r.mqtt.client.clone());
    let wall = std::time::Instant::now();
    let poll_ns = 1_500 * MS;
    let (times, returned, last_write) = {
        let mut car = car_timed(&mut r, CarCfg { poll_ns, ..CarCfg::default() });
        for (i, mib) in [(0u32, 4usize), (1, 4), (2, 4), (3, 4)] {
            record(&mut car, &format!("TeslaCam/SavedClips/2026-01-01_00-0{i}-00/event.mp4"), mib);
            car.idle(2 * S).unwrap();
        }
        idle_until(&mut car, &shared, "session ready", 90 * S).expect("no MQTT session within 90 s");
        let deadline = car.now() + 120 * S;
        while shared.borrow().when("supervisor: card taken for archiving").is_none() && car.now() < deadline {
            mqtt.publish(&cmd, "force_open").unwrap();
            car.idle(2 * S).unwrap();
        }
        let by = car.now() + 200 * S;
        let returned = idle_until(&mut car, &shared, "supervisor: card returned to the car", by).expect("the window never ended");
        while car.stats.media_changes < 2 && car.now() < returned + 10 * S { car.idle(100 * MS).unwrap(); }
        car.idle(3 * S).unwrap();
        record(&mut car, "TeslaCam/SavedClips/2026-01-01_00-09-00/event.mp4", 2);
        let last_write = car.now();
        // Past the minute after which the car counts as stopped, and the save after it.
        car.idle(75 * S).unwrap();
        eprintln!("[report] car: {:?}", car.stats);
        (car.disk.dev.times.take().unwrap(), returned, last_write)
    };
    let cut_at = r.cs.now_ns;
    let sd = r.sd_writes.as_ref().unwrap().borrow().clone();
    let ops_boot1 = r.cs.p4.bus.flash_ops.clone();
    eprintln!("[report] boot 1: {} car commands, {} card writes, {} flash operations; power cut at {}", times.len(), sd.len(), ops_boot1.len(), secs(cut_at));
    power_cut(&mut r);
    assert!(run_until_lines(&mut r, "car_report: car 1/", 1, cut_at + 120 * S), "boot 2 never published boot 1's report");
    let got = reports(&r);
    let one = got.iter().find(|(t, _)| t.ends_with("/car_report/1")).map(|(_, j)| j.clone()).expect("no retained car_report/1");
    let last = got.iter().find(|(t, _)| t.ends_with("/car_report/last")).map(|(_, j)| j.clone()).expect("no retained car_report/last");
    let summary = r.p4_lines().into_iter().find(|l| l.contains("car_report: car 1/")).unwrap().to_string();
    eprintln!("[report] {summary}");
    for (t, j) in &got { eprintln!("[report] retained {t}: seq {}", num(j, "seq")); }

    let saved_ns = num(&one, "up_ms") as u64 * MS;
    let truth = truth(&times, &sd, saved_ns);
    eprintln!("[report] boot 1 saved last at {} (its uptime), {} after the last write", secs(saved_ns), secs(saved_ns.saturating_sub(last_write)));
    let pair = |name: &str, dev: i64, emu: i64| { eprintln!("[report] {name:<28} device {dev:>8}  emulator {emu:>8}"); (dev, emu) };
    let checks = [
        pair("TEST UNIT READY", num(&one, "scsi.tur"), truth.tur as i64),
        pair("REQUEST SENSE", num(&one, "scsi.sense"), truth.sense as i64),
        pair("READ CAPACITY", num(&one, "scsi.capacity"), truth.capacity as i64),
        pair("INQUIRY", num(&one, "scsi.inquiry"), truth.inquiry as i64),
        pair("sense 2/3A/00", num(&one, "scsi.not_present"), truth.not_present as i64),
        pair("sense 6/28/00", num(&one, "scsi.attention"), truth.attention as i64),
        pair("sense none", num(&one, "scsi.no_sense"), truth.no_sense as i64),
        pair("sense other", num(&one, "scsi.other_sense"), truth.other_sense as i64),
        pair("car writes", num(&one, "writes.n"), truth.writes as i64),
        pair("car sectors", num(&one, "writes.sectors"), truth.sectors as i64),
        pair("windows", num(&one, "windows.n"), 1),
        pair("windows not written after", num(&one, "windows.no_write"), 0),
    ];
    let dev_tur = hist(&one, "scsi.tur_gaps");
    let dev_write = hist(&one, "writes.hist");
    eprintln!("[report] TUR gaps     device {dev_tur:?}  emulator {:?} ({} within 2 ms of an edge)", truth.tur_gaps, truth.near_edge);
    eprintln!("[report] write times  device {dev_write:?}  emulator {:?}", truth.write_hist);
    eprintln!("[report] longest write device {} us, emulator {} us; slow (>=100 ms) {}", num(&one, "writes.max_us"), truth.write_max_ns / 1000, num(&one, "writes.slow"));

    // The gaps after the window: from the card's return to the car's first command, the one the
    // unit attention met, and to its first write. The return is the console line's slice end, so
    // up to 100 ms late.
    let ua = times.iter().position(|c| c.submit_ns >= returned.saturating_sub(S) && c.sense == Some((6, 0x28, 0))).expect("no unit attention after the window");
    let first_cmd = times[..ua].iter().rev().find(|c| !c.ok).map(|c| c.submit_ns).expect("the command the unit attention failed");
    let first_write = times[ua..].iter().find(|c| c.op == 0x2a && c.ok).map(|c| c.submit_ns).expect("no write after the window");
    let (dev_cmd, dev_write_gap) = (num(&one, "windows.first_cmd_ms") as u64, num(&one, "windows.first_write_ms") as u64);
    eprintln!("[report] after the window: first command device {dev_cmd} ms, emulator {} ms; first write device {dev_write_gap} ms, emulator {} ms",
              first_cmd.saturating_sub(returned) / MS, first_write.saturating_sub(returned) / MS);
    let p = |k: &str| num(&one, &format!("persist.{k}"));
    eprintln!("[report] persists {} longest {} us, {} ms in all, held commands {}+{}; histogram {:?}", p("n"), p("max_us"), p("total_ms"), p("during"), p("after"), hist(&one, "persist.hist"));
    for b in bursts(&ops_boot1, 0, cut_at) { eprintln!("[report]   flash busy {} to {} ({:.1} ms): {} bytes programmed, {} erases", secs(b.0), secs(b.1), (b.1 - b.0) as f64 / 1e6, b.2, b.3); }
    let held = held_by_flash(&times, &ops_boot1);
    for (c, extra) in &held { eprintln!("[report]   car command {:#04x} at {} took {} us, {} us over its median, the flash busy", c.op, secs(c.submit_ns), c.latency_ns() / 1000, extra / 1000); }

    // Boot 2 saves the mark at its next quiet moment; boot 3 must not publish boot 1 again.
    let t2 = r.cs.now_ns + 10 * S;
    r.run_to(t2);
    power_cut(&mut r);
    let by = r.cs.now_ns + 120 * S;
    let published_two = run_until_lines(&mut r, "car_report: car 2/", 1, by);
    let after = reports(&r);
    let firsts = r.p4_lines().iter().filter(|l| l.contains("car_report: car 1/")).count();
    r.report(20);
    eprintln!("[report] {:.0} s wall for {} emulated", wall.elapsed().as_secs_f64(), secs(r.cs.now_ns));

    assert_eq!(r.cs.summary.p4_resets, 2, "two power cuts");
    assert_eq!(num(&one, "seq"), 1);
    assert_eq!(one.str_or("reset", "?"), "power on");
    assert_eq!(one.str_or("ended", "?"), "power on", "boot 1 ended by a power cut");
    assert_eq!(num(&last, "seq"), 1, "car_report/last after boot 2's publish");
    for (dev, emu) in checks { assert_eq!(dev, emu, "a device figure differs from the emulator's"); }
    assert_eq!(truth.near_edge, 0, "a TUR gap within 2 ms of a bucket edge; move poll_ns");
    assert_eq!(dev_tur, truth.tur_gaps.to_vec(), "TUR gaps");
    assert!((num(&one, "scsi.tur_gap_min_ms") as u64).abs_diff(truth.tur_min_ms) <= 5, "shortest TUR gap");
    assert_eq!(dev_write, truth.write_hist.to_vec(), "car write service times");
    assert!(truth.write_hist[2] > 0 && truth.write_hist[4] > 0, "the slow writes the card was set to make");
    assert!(dev_cmd.abs_diff(first_cmd.saturating_sub(returned) / MS) <= 150, "first command after the window");
    assert!(dev_write_gap.abs_diff(first_write.saturating_sub(returned) / MS) <= 150, "first write after the window");
    assert!(p("n") >= 2, "saved at boot and when the car went quiet");
    assert!(saved_ns >= last_write + 60 * S, "the last save came after the car stopped");
    assert!(published_two, "boot 3 never published boot 2's report");
    assert!(after.iter().any(|(t, j)| t.ends_with("/car_report/2") && num(j, "seq") == 2));
    assert_eq!(firsts, 1, "boot 1's report published again after it was marked");
}

/// One boot of a car that records the way a Linux flusher writes back: a second of writes
/// every five seconds, TEST UNIT READY every 2 s, for `secs`. Returns the car's command times and
/// the flash operations.
fn bursty_run(dir: String, card: &str, secs_total: u64) -> (Rig, Vec<CmdTime>, Vec<FlashOp>, u64, u64) {
    let mut r = rig_on(dir, card, [(0, 0); 2]);
    let shared = r.shared.clone();
    let (times, from, to, failed) = {
        let mut car = car_timed(&mut r, CarCfg::default());
        let from = car.now();
        let end = from + secs_total * S;
        let mut n = 0;
        let mut failed = None;
        while car.now() < end {
            let start = car.now();
            let path = format!("TeslaCam/RecentClips/2026-01-01_{:02}-{:02}-00-front.mp4", n / 60, n % 60);
            let st = car.record(&path, &mp4_bytes(&path, 4 << 20));
            if !matches!(st, Ok(Recorded::Closed { .. })) {
                failed = Some(format!("{path} at {}: {st:?}", secs(car.now())));
                break;
            }
            n += 1;
            car.wait_until(start + 5 * S).unwrap();
        }
        let to = car.now();
        (car.disk.dev.times.take().unwrap(), from, to, failed)
    };
    if let Some(why) = failed {
        let sh = shared.borrow();
        let lines: Vec<_> = sh.lines.iter().filter(|l| !l.1.contains("esp_core_dump")).collect();
        for (t, l) in &lines[lines.len().saturating_sub(150)..] { eprintln!("[cost] {} {l}", secs(*t)); }
        // The core dump partition (0x1a000, 64 KiB in oxidrive's table), for espcoredump.py.
        let dump = common::tmp("car-report-coredump.bin");
        std::fs::write(&dump, &r.cs.p4.bus.flash[0x1a000..0x2a000]).unwrap();
        panic!("{why}; the core dump partition is in {}", dump.display());
    }
    let ops = r.cs.p4.bus.flash_ops.clone();
    (r, times, ops, from, to)
}

fn latency_hist(times: &[CmdTime]) -> [usize; 7] {
    let mut h = [0usize; 7];
    for t in times { h[bucket(&WRITE_MS, t.latency_ns() / MS)] += 1; }
    h
}

/// What the report costs the car: the same bursty recording on a build without the report
/// (`OXIDRIVE_BASELINE_DIR`) and on one with it, for `OXIDRIVE_COST_SECS` (330: past the five
/// minutes after which a writing car's report is saved again). Printed: every car command's
/// latency bucketed as the report buckets writes, the flash's busy spells, and the car commands
/// in flight during one. Asserted: the report's own saves came in gaps (no car command in flight
/// during one), the report build's latency tail is no worse than the baseline's, and the report
/// the next boot publishes counts its saves as the emulator saw them.
#[test]
#[ignore = "needs the oxidrive car-report and bc40b82 images and nvs.csv (OXIDRIVE_CAR_REPORT_DIR, OXIDRIVE_BASELINE_DIR), ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_car_report_costs_the_car_little() {
    let secs_total = env_u64("OXIDRIVE_COST_SECS", 330);
    let wall = std::time::Instant::now();
    let (base_r, base_times, base_ops, base_from, base_to) = bursty_run(baseline_dir(), "cost-base.img", secs_total);
    drop(base_r);
    let (mut r, times, ops, from, to) = bursty_run(report_dir(), "cost-report.img", secs_total);
    let in_run = |t: &[CmdTime], a: u64, b: u64| t.iter().filter(|c| c.submit_ns >= a && c.submit_ns < b).copied().collect::<Vec<_>>();
    let (base_times, times) = (in_run(&base_times, base_from, base_to), in_run(&times, from, to));
    let (base_h, h) = (latency_hist(&base_times), latency_hist(&times));
    let base_held = held_by_flash(&base_times, &base_ops);
    let held = held_by_flash(&times, &ops);
    let base_bursts = bursts(&base_ops, base_from, base_to);
    let report_bursts = bursts(&ops, from, to);
    eprintln!("[cost] {} s of recording; car commands by latency (<=10, 25, 50, 100, 250, 500, >500 ms):", secs_total);
    eprintln!("[cost]   without the report: {base_h:?} of {}, max {} us", base_times.len(), base_times.iter().map(|t| t.latency_ns()).max().unwrap_or(0) / 1000);
    eprintln!("[cost]   with the report:    {h:?} of {}, max {} us", times.len(), times.iter().map(|t| t.latency_ns()).max().unwrap_or(0) / 1000);
    eprintln!("[cost]   flash busy spells: {} without, {} with", base_bursts.len(), report_bursts.len());
    for (name, bs) in [("without", &base_bursts), ("with", &report_bursts)] {
        for b in bs { eprintln!("[cost]     {name}: {} for {:.2} ms, {} bytes programmed, {} erases", secs(b.0), (b.1 - b.0) as f64 / 1e6, b.2, b.3); }
    }
    for (name, hs) in [("without", &base_held), ("with", &held)] {
        for (c, extra) in hs.iter() { eprintln!("[cost]     {name}: car command {:#04x} at {} took {} us ({} us over its median) with the flash busy", c.op, secs(c.submit_ns), c.latency_ns() / 1000, extra / 1000); }
    }
    // The report's saves since boot: spells that programmed a record's worth (465 bytes and its
    // entry headers).
    let saves: Vec<_> = bursts(&ops, 0, to).into_iter().filter(|b| b.2 >= 400).collect();
    let in_save = |c: &CmdTime| saves.iter().any(|b| b.0 < c.done_ns && b.1 > c.submit_ns);
    let held_by_saves: Vec<_> = times.iter().filter(|c| in_save(c)).collect();
    eprintln!("[cost] the report's saves: {} spells, {} car commands in flight during one", saves.len(), held_by_saves.len());

    let cut = r.cs.now_ns;
    power_cut(&mut r);
    assert!(run_until_lines(&mut r, "car_report: car 1/", 1, cut + 120 * S), "the next boot never published the report");
    let one = reports(&r).into_iter().find(|(t, _)| t.ends_with("/car_report/1")).map(|(_, j)| j).expect("no retained car_report/1");
    let p = |k: &str| num(&one, &format!("persist.{k}"));
    eprintln!("[cost] the report: {} saves, longest {} us, {} ms in all, histogram {:?}; commands during {} and just after {}", p("n"), p("max_us"), p("total_ms"), hist(&one, "persist.hist"), p("during"), p("after"));
    eprintln!("[cost] its writes: {:?}, slow {}", hist(&one, "writes.hist"), num(&one, "writes.slow"));
    eprintln!("[cost] {:.0} s wall", wall.elapsed().as_secs_f64());

    assert!(saves.len() >= 2, "a save at boot and one five minutes in");
    assert!(held_by_saves.is_empty(), "a car command was in flight during a save");
    // The record counts the saves before its own.
    assert!(p("n") as usize + 1 >= saves.len(), "the report counts every save the emulator saw");
    assert!(h[4] + h[5] + h[6] <= base_h[4] + base_h[5] + base_h[6], "more commands of 100 ms or more with the report");
    assert_eq!(num(&one, "writes.slow"), 0, "a slow car write");
}
