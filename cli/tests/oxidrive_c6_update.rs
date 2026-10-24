//! oxidrive's update of its C6 co-processor (the owner's firmware, not in this repository): the
//! slave image staged on the share with a signed manifest, `c6_update` over MQTT, the image
//! downloaded, verified, pushed through ESP-Hosted's slave OTA to the C6's other slot, the C6 told
//! to boot it, and the P4 restarting to meet it, on the rig of `oxidrive_services`
//! (`oxidrive_rig`). The C6 starts on ESP-Hosted 2.12.13's committed slave.
//!
//! Inputs, none of them in this repository: `OXIDRIVE_DIR`, a build with `c6_update` whose OTA
//! key is a throwaway one; `OXIDRIVE_C6_IMAGE` and `OXIDRIVE_C6_MANIFEST`, a 2.13.1 slave and its
//! manifest signed with that key (`bin/oxi-update c6-stage --dry-run` writes both);
//! `OXIDRIVE_C6_BROKEN_IMAGE` and `OXIDRIVE_C6_BROKEN_MANIFEST`, a slave that never answers the
//! P4 (a `-test` build, which `c6-stage` refuses; `oxi ota sign-c6` signs it); and
//! `OXIDRIVE_SLAVE_KNOWN`, the slave builds' directories, so the C6's PHY stub follows the image
//! it boots (`hosted_slave`). esp32p4.md, "The C6's own update", has the method and results.
#[path = "../../tests/common.rs"]
mod common;
#[path = "../../tests/sd_image.rs"]
mod sd_image;
#[path = "../../tests/oxidrive_board.rs"]
mod oxidrive_board;
#[path = "../../tests/services.rs"]
mod services;
#[path = "../../tests/hosted_slave.rs"]
mod hosted_slave;
#[path = "../../tests/oxidrive_rig.rs"]
mod oxidrive_rig;
use oxidrive_rig::*;

/// An image and its manifest, from the environment.
struct Staged { name: String, image: Vec<u8>, manifest: String }

fn staged(image_var: &str, manifest_var: &str) -> Staged {
    let path = std::env::var(image_var).unwrap_or_else(|_| panic!("set {image_var} to a slave image"));
    let manifest = std::fs::read_to_string(std::env::var(manifest_var).unwrap_or_else(|_| panic!("set {manifest_var} to its manifest"))).unwrap();
    let name = manifest.lines().find_map(|l| l.strip_prefix("file=")).expect("a file= line").to_string();
    Staged { name, image: std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}")), manifest }
}

impl Staged {
    fn version(&self) -> String { self.manifest.lines().find_map(|l| l.strip_prefix("version=")).expect("a version= line").to_string() }
    /// The image's `app_elf_sha256`, its first four bytes in hex, as the state document has it.
    fn elf8(&self) -> String { hosted_slave::elf_sha(&self.image).unwrap()[..4].iter().map(|b| format!("{b:02x}")).collect() }
    /// Put it in `oxidrive-ota/c6/` on the share, the manifest last.
    fn stage(&self, r: &Rig) {
        let dir = r.smb.path().join("oxidrive-ota/c6");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(&self.name), &self.image).unwrap();
        std::fs::write(dir.join("manifest"), &self.manifest).unwrap();
    }
}

fn since(r: &Rig, from: usize) -> Vec<&str> { r.lines[from..].iter().filter(|l| !l.1).map(|l| l.2.as_str()).collect() }
fn c6_since(r: &Rig, from: usize) -> Vec<&str> { r.lines[from..].iter().filter(|l| l.1).map(|l| l.2.as_str()).collect() }
fn when_since(r: &Rig, from: usize, what: &str) -> Option<u64> { r.lines[from..].iter().find(|l| !l.1 && l.2.contains(what)).map(|l| l.0) }

/// Run a second at a time until a P4 line from `from` on holds `what`, or until `end_ns`.
fn run_until(r: &mut Rig, from: usize, what: &str, end_ns: u64) -> bool {
    run_until_any(r, from, &[what], end_ns).is_some()
}

/// Run a second at a time until a P4 line from `from` on holds one of `whats`, or until
/// `end_ns`; which one did.
fn run_until_any(r: &mut Rig, from: usize, whats: &[&str], end_ns: u64) -> Option<usize> {
    loop {
        if let Some(i) = whats.iter().position(|w| when_since(r, from, w).is_some()) { return Some(i); }
        if r.cs.now_ns >= end_ns { return None; }
        let t = r.cs.now_ns + 1_000_000_000;
        r.run_to(t);
    }
}

/// Run until the retained state document's `c6` says `phase`, or until `end_ns`.
fn settle(r: &mut Rig, phase: &str, end_ns: u64) -> bool {
    let want = format!("\"phase\":\"{phase}\"");
    while r.cs.now_ns < end_ns && !c6_state(r).contains(&want) { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    c6_state(r).contains(&want)
}

/// The C6's guard lines (`c6-slave/oxidrive_guard.c` logs under the tag `oxidrive`) from `from`.
fn guard_since(r: &Rig, from: usize) -> Vec<String> {
    r.lines[from..].iter().filter(|l| l.1 && l.2.contains("oxidrive:")).map(|l| format!("{:.1} s {}", l.0 as f64 / 1e9, l.2.trim())).collect()
}

/// A rig on the committed 2.12.13 slave, run until its MQTT session is up and the C6 has said
/// what it runs.
fn c6_rig(card: &str, end_ns: u64) -> Rig {
    let mut r = rig(card, 4 << 30, false);
    assert!(run_until(&mut r, 0, "session ready", end_ns), "no MQTT session");
    assert!(run_until(&mut r, 0, "c6: runs ", end_ns), "the C6 never said what it runs");
    r
}

fn send(r: &Rig, cmd: &str) {
    eprintln!("[oxidrive] t={:.3} s: {cmd}", r.cs.now_ns as f64 / 1e9);
    r.mqtt.client.publish(&format!("oxidrive/{}/cmd", r.id), cmd).unwrap();
}

/// The retained state document's `c6` object.
fn c6_state(r: &Rig) -> String {
    let state = format!("oxidrive/{}/state", r.id);
    let doc = r.mqtt.client.retained(&[&state], std::time::Duration::from_secs(1)).unwrap().into_iter().next().map(|(_, p)| p).unwrap_or_default();
    doc.split("\"c6\":").nth(1).and_then(|c| c.split('}').next()).unwrap_or("").to_string()
}

fn show(r: &Rig, from: usize) {
    for l in since(r, from).iter().filter(|l| l.contains("c6") || l.contains("supervisor") || l.contains("ota:") || l.contains("Rebooting") || l.contains("transport")) { eprintln!("[oxidrive]   {l}"); }
    for l in c6_since(r, from).iter().filter(|l| l.contains("oxidrive") || l.contains("OTA") || l.contains("ota") || l.contains("boot:") || l.contains("Loaded app")) { eprintln!("[oxidrive]   [c6] {l}"); }
}

/// (c): the whole update. The C6 on 2.12.13 is updated to the staged 2.13.1 slave: downloaded,
/// verified and pushed with the card taken, the C6 told to boot it, the P4 restarted with the card
/// still taken, the new image's ELF hash read back from the C6, and the image confirmed to the
/// slave's guard once WiFi and the NAS are up, then reported `confirmed`.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXIDRIVE_C6_MANIFEST, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_updates_its_c6() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 300) * 1_000_000_000;
    let mut r = c6_rig("oxidrive-c6.img", end);
    let s = staged("OXIDRIVE_C6_IMAGE", "OXIDRIVE_C6_MANIFEST");
    let v = s.version();
    s.stage(&r);
    let mark = r.lines.len();
    let t0 = r.cs.now_ns;
    let wall = std::time::Instant::now();
    send(&r, &format!("c6_update {v}"));
    let confirmed = run_until(&mut r, mark, &format!("confirmed {v}"), end);
    // The retained document is published once the session is back after the restart.
    while r.cs.now_ns < end && !c6_state(&r).contains("\"phase\":\"confirmed\"") { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    show(&r, mark);
    let at = |what: &str| when_since(&r, mark, what).map_or(f64::NAN, |n| (n - t0) as f64 / 1e9);
    eprintln!("[oxidrive] from the command: card taken {:.1} s, pushed {:.1} s, P4 back {:.1} s, the C6 read {:.1} s, the slave kept its confirmation {:.1} s, confirmed {:.1} s ({:.1} s wall)",
              at("card taken, fetching"), at(&format!("pushed {v}")), at("oxidrive bring-up"), at(&format!("c6: runs {v}")), at("kept its confirmation"), at(&format!("confirmed {v}")), wall.elapsed().as_secs_f64());
    // The card stays taken from the push to the restart: the car loses it once.
    let returned = when_since(&r, mark, "card returned to the car");
    let back_up = when_since(&r, mark, "oxidrive bring-up");
    eprintln!("[oxidrive] card returned to the car before the restart: {}", matches!((returned, back_up), (Some(a), Some(b)) if a < b));
    for l in guard_since(&r, mark) { eprintln!("[oxidrive]   [c6] {l}"); }
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    assert!(confirmed, "no confirmation of {v}");
    assert!(when_since(&r, mark, "supervisor: card taken for a firmware update").is_some(), "the card was not taken");
    assert!(!matches!((returned, back_up), (Some(a), Some(b)) if a < b), "the card went back to the car a moment before the restart took it again");
    assert!(guard_since(&r, mark).iter().any(|l| l.contains("confirmed by the host")), "the slave's guard never heard the P4's confirmation");
    assert_eq!(r.cs.summary.p4_resets, 1, "the P4 restarts once, to meet the new image");
    let state = c6_state(&r);
    assert!(state.contains(&format!("\"fw\":\"{v}\"")) && state.contains(&format!("\"elf\":\"{}\"", s.elf8())) && state.contains("\"phase\":\"confirmed\""), "{state}");
    assert!(!since(&r, mark).iter().any(|l| l.contains("mismatch")), "a version mismatch after the update");
}

/// (c), refusals: an image whose bytes are not the manifest's, a manifest whose signature no key
/// made, and an unsigned one. Each is refused by name, the C6 keeps 2.12.13, and nothing restarts.
/// The forged and the unsigned manifests are refused before the card is taken; the changed byte
/// only after the download, and the card comes back. Then `c6_update` naming another minor of
/// ESP-Hosted than the P4's is refused on the version alone.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXIDRIVE_C6_MANIFEST, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_refuses_a_c6_image_it_cannot_verify() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 300) * 1_000_000_000;
    let mut r = c6_rig("oxidrive-c6-refuse.img", end);
    let good = staged("OXIDRIVE_C6_IMAGE", "OXIDRIVE_C6_MANIFEST");
    let v = good.version();
    let mut flipped = Staged { name: good.name.clone(), image: good.image.clone(), manifest: good.manifest.clone() };
    flipped.image[0x1000] ^= 0x40;
    let sig = good.manifest.lines().find(|l| l.starts_with("sig=")).unwrap().to_string();
    let forged = Staged { name: good.name.clone(), image: good.image.clone(), manifest: good.manifest.replace(&sig, &format!("sig={}", "1".repeat(128))) };
    let unsigned = Staged { name: good.name.clone(), image: good.image.clone(), manifest: good.manifest.replace(&format!("{sig}\n"), "") };
    let mut outcomes = Vec::new();
    for (what, s, want, takes) in [("flipped byte", &flipped, "crc32", true), ("forged signature", &forged, "signature did not verify", false), ("unsigned", &unsigned, "manifest is not signed", false)] {
        s.stage(&r);
        let mark = r.lines.len();
        send(&r, &format!("c6_update {v}"));
        let seen = run_until(&mut r, mark, want, end);
        let t = r.cs.now_ns + 3_000_000_000;
        r.run_to(t);
        show(&r, mark);
        let line = since(&r, mark).iter().find(|l| l.contains(want)).map(|l| l.to_string()).unwrap_or_default();
        outcomes.push((what, seen, line, takes, when_since(&r, mark, "card taken").is_some(), when_since(&r, mark, "card returned").is_some()));
    }
    // ESP-Hosted 2.12 under a 2.13 host: refused before the share is read.
    let mark = r.lines.len();
    send(&r, "c6_update 2.12.14-oxi1");
    let policy = run_until_any(&mut r, mark, &["the slave must match", "refused update to 2.12.14-oxi1", "update to 2.12.14-oxi1 failed"], end);
    let t = r.cs.now_ns + 3_000_000_000;
    r.run_to(t);
    let policy_line = since(&r, mark).iter().find(|l| l.contains("update to 2.12.14-oxi1")).map(|l| l.to_string()).unwrap_or_default();
    let policy_taken = when_since(&r, mark, "card taken").is_some();
    for (what, seen, line, _, taken, back) in &outcomes { eprintln!("[oxidrive] {what}: refused={seen} card taken={taken} returned={back}: {line}"); }
    eprintln!("[oxidrive] another minor: card taken={policy_taken}: {policy_line}");
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    for (what, seen, _, takes, taken, back) in &outcomes {
        assert!(seen, "the {what} image was not refused by name");
        assert_eq!(taken, takes, "the {what} image: card taken {taken}, expected {takes}");
        assert!(!taken || *back, "the {what} image kept the card");
    }
    assert_eq!(policy, Some(0), "another minor was not refused on the version: {policy_line}");
    assert!(!policy_taken, "another minor took the card");
    assert_eq!(r.cs.summary.p4_resets, 0, "nothing restarts");
    assert!(c6_state(&r).contains("\"fw\":\"2.12.13\""), "the C6 no longer runs 2.12.13");
}

/// (c), cancel: `c6_update_cancel` sent once the download has started stops the update before
/// the C6 is told to boot anything. The card comes back and the C6 keeps 2.12.13.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXIDRIVE_C6_MANIFEST, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_cancels_a_c6_update_before_the_push_ends() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 300) * 1_000_000_000;
    let mut r = c6_rig("oxidrive-c6-cancel.img", end);
    let s = staged("OXIDRIVE_C6_IMAGE", "OXIDRIVE_C6_MANIFEST");
    let v = s.version();
    s.stage(&r);
    let mark = r.lines.len();
    send(&r, &format!("c6_update {v}"));
    assert!(run_until(&mut r, mark, "card taken, fetching", end), "the update never took the card");
    send(&r, &format!("c6_update_cancel {v}"));
    let cancelled = run_until(&mut r, mark, &format!("update to {v} cancelled"), end);
    let t = r.cs.now_ns + 5_000_000_000;
    r.run_to(t);
    show(&r, mark);
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    assert!(cancelled, "no cancel recorded");
    assert!(when_since(&r, mark, "card returned").is_some(), "the card did not come back");
    assert_eq!(r.cs.summary.p4_resets, 0, "nothing restarts");
    assert!(!since(&r, mark).iter().any(|l| l.contains("the C6 boots it now")), "the C6 was told to boot it");
    assert!(c6_state(&r).contains("\"fw\":\"2.12.13\"") && c6_state(&r).contains("\"phase\":\"cancelled\""), "{}", c6_state(&r));
}

/// (c), a C6 that does not come back: a slave that never answers the P4 is installed. The image
/// guard in the slave (`c6-slave/oxidrive_guard.c`) marks it invalid and the bootloader returns
/// to 2.12.13; the P4 meets 2.12.13 and records the update failed.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_BROKEN_IMAGE, OXIDRIVE_C6_BROKEN_MANIFEST, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_c6_that_never_answers_reverts() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 600) * 1_000_000_000;
    let mut r = c6_rig("oxidrive-c6-broken.img", end);
    let s = staged("OXIDRIVE_C6_BROKEN_IMAGE", "OXIDRIVE_C6_BROKEN_MANIFEST");
    let v = s.version();
    s.stage(&r);
    let mark = r.lines.len();
    let t0 = r.cs.now_ns;
    send(&r, &format!("c6_update {v}"));
    let failed = run_until(&mut r, mark, &format!("update to {v} failed: the C6 came back on 2.12.13"), end);
    while r.cs.now_ns < end && !c6_state(&r).contains("\"phase\":\"failed\"") { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    show(&r, mark);
    let c6 = c6_since(&r, mark);
    let guard: Vec<_> = c6.iter().filter(|l| l.contains("oxidrive:")).collect();
    eprintln!("[oxidrive] the guard said {} lines: {guard:?}", guard.len());
    let at = |what: &str| when_since(&r, mark, what).map_or(f64::NAN, |n| (n - t0) as f64 / 1e9);
    eprintln!("[oxidrive] from the command: pushed {:.1} s, recorded failed {:.1} s; P4 resets {}", at(&format!("pushed {v}")), at("failed: the C6 came back"), r.cs.summary.p4_resets);
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    assert!(failed, "the P4 did not meet 2.12.13 again");
    assert!(c6.iter().any(|l| l.contains("marking this image invalid")), "the guard did not revert the image");
    let state = c6_state(&r);
    assert!(state.contains("\"fw\":\"2.12.13\"") && state.contains("\"phase\":\"failed\""), "{state}");
}

/// Run a second at a time until a line from `from` on, the P4's or the C6's, holds one of
/// `whats`; which one did.
fn run_until_line(r: &mut Rig, from: usize, whats: &[&str], end_ns: u64) -> Option<usize> {
    loop {
        if let Some(i) = whats.iter().position(|w| r.lines[from..].iter().any(|l| l.2.contains(w))) { return Some(i); }
        if r.cs.now_ns >= end_ns { return None; }
        let t = r.cs.now_ns + 1_000_000_000;
        r.run_to(t);
    }
}

/// Push the slave that never answers and run until the P4 records that the C6 came back on
/// 2.12.13 and the state document says `failed`. Whether it did, and the guard's lines.
fn push_broken(r: &mut Rig, s: &Staged, end: u64) -> (bool, Vec<String>) {
    let v = s.version();
    s.stage(r);
    let mark = r.lines.len();
    send(r, &format!("c6_update {v}"));
    let failed = run_until(r, mark, &format!("update to {v} failed: the C6 came back on 2.12.13"), end) && settle(r, "failed", end);
    (failed, guard_since(r, mark))
}

/// A good slave after `broken` pushes of one that never answers, each reverted by its guard.
/// The good one counts its own boots from 1, whatever the reverted ones left, and is confirmed.
fn good_after_broken(card: &str, broken: usize) {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 300 + 300 * broken as u64) * 1_000_000_000;
    let mut r = c6_rig(card, end);
    let bad = staged("OXIDRIVE_C6_BROKEN_IMAGE", "OXIDRIVE_C6_BROKEN_MANIFEST");
    let good = staged("OXIDRIVE_C6_IMAGE", "OXIDRIVE_C6_MANIFEST");
    let t0 = r.cs.now_ns;
    let mut reverted = Vec::new();
    for i in 1..=broken {
        let (failed, guard) = push_broken(&mut r, &bad, end);
        eprintln!("[oxidrive] broken push {i}: recorded failed {failed} at {:.1} s; the guard:", (r.cs.now_ns - t0) as f64 / 1e9);
        for l in &guard { eprintln!("[oxidrive]   [c6] {l}"); }
        reverted.push(failed);
    }
    let v = good.version();
    good.stage(&r);
    let mark = r.lines.len();
    send(&r, &format!("c6_update {v}"));
    let outcome = run_until_any(&mut r, mark, &[&format!("confirmed {v}"), &format!("update to {v} failed")], end);
    if outcome == Some(0) { settle(&mut r, "confirmed", end); }
    show(&r, mark);
    let guard = guard_since(&r, mark);
    eprintln!("[oxidrive] the good push: {}; the guard:", match outcome { Some(0) => "confirmed", Some(_) => "failed", None => "no outcome" });
    for l in &guard { eprintln!("[oxidrive]   [c6] {l}"); }
    eprintln!("[oxidrive] the state's c6: {}; P4 resets {}", c6_state(&r), r.cs.summary.p4_resets);
    r.report(10);
    assert!(reverted.iter().all(|f| *f), "a broken image was not reverted: {reverted:?}");
    assert_eq!(outcome, Some(0), "the good image after {broken} reverted one(s) was not confirmed");
    assert!(guard.iter().any(|l| l.contains("boot 1 of")), "the good image's first boot was not counted as its first");
    let state = c6_state(&r);
    assert!(state.contains(&format!("\"fw\":\"{v}\"")) && state.contains(&format!("\"elf\":\"{}\"", good.elf8())) && state.contains("\"phase\":\"confirmed\""), "{state}");
}

/// A revert does not lock out the next image: the slave that never answers is pushed, reverts,
/// and then the good slave is pushed and confirmed.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXIDRIVE_C6_MANIFEST, OXIDRIVE_C6_BROKEN_IMAGE, OXIDRIVE_C6_BROKEN_MANIFEST, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_c6_takes_a_good_image_after_one_that_reverted() {
    good_after_broken("oxidrive-c6-after-one.img", 1);
}

/// As above, with two reverted pushes before the good one.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXIDRIVE_C6_MANIFEST, OXIDRIVE_C6_BROKEN_IMAGE, OXIDRIVE_C6_BROKEN_MANIFEST, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_c6_takes_a_good_image_after_two_that_reverted() {
    good_after_broken("oxidrive-c6-after-two.img", 2);
}

/// The slave that never answers, on a C6 whose otadata holds only the entry the push wrote: once
/// the image is pushed, the sector with the older entry is erased, as on a C6 whose bootloader
/// never wrote one for the image it first booted. ESP-IDF then refuses the rollback; the guard
/// sets the other slot to boot itself, and the P4 meets 2.12.13 again.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_BROKEN_IMAGE, OXIDRIVE_C6_BROKEN_MANIFEST, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_c6_reverts_with_the_other_otadata_entry_blank() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 600) * 1_000_000_000;
    let mut r = c6_rig("oxidrive-c6-blank-otadata.img", end);
    let s = staged("OXIDRIVE_C6_BROKEN_IMAGE", "OXIDRIVE_C6_BROKEN_MANIFEST");
    let v = s.version();
    s.stage(&r);
    let mark = r.lines.len();
    let t0 = r.cs.now_ns;
    send(&r, &format!("c6_update {v}"));
    assert!(run_until(&mut r, mark, &format!("pushed {v}"), end), "the image was never pushed");
    let pushed = hosted_slave::otadata(&r.cs.c6.borrow().m.bus.flash);
    let blanked = hosted_slave::blank_older_otadata(&mut r.cs.c6.borrow_mut().m);
    eprintln!("[oxidrive] otadata after the push (seq, state): {pushed:?}; erased the sector at {blanked:#x?}");
    let failed = run_until(&mut r, mark, &format!("update to {v} failed: the C6 came back on 2.12.13"), end);
    if failed { settle(&mut r, "failed", end); }
    let guard = guard_since(&r, mark);
    for l in &guard { eprintln!("[oxidrive]   [c6] {l}"); }
    eprintln!("[oxidrive] otadata at the end (seq, state): {:?}", hosted_slave::otadata(&r.cs.c6.borrow().m.bus.flash));
    eprintln!("[oxidrive] recorded failed {:.1} s after the command; P4 resets {}", when_since(&r, mark, "failed: the C6 came back").map_or(f64::NAN, |n| (n - t0) as f64 / 1e9), r.cs.summary.p4_resets);
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    assert!(blanked.is_some(), "the push left no older otadata entry to erase");
    assert!(guard.iter().any(|l| l.contains("booting the other slot")), "the guard did not boot the other slot itself");
    assert!(failed, "the P4 did not meet 2.12.13 again");
    assert!(c6_state(&r).contains("\"fw\":\"2.12.13\""), "{}", c6_state(&r));
}

/// How the network is kept from the C6 after the push.
#[derive(Clone, Copy)]
enum Away { Renamed, Rekeyed }

/// A slave that answers the P4 but cannot join the network: the good slave is pushed, and from
/// the push on the network is kept from the C6 (`Away`). The P4 never has WiFi and the NAS, so it
/// never confirms the image and the guard reverts it; the network comes back once it has. Which
/// of the guard's limits acts depends on whether the P4 resets the C6 meanwhile: it does after
/// four failed bring-ups unless they timed out and a scan still answers (out of range).
fn cannot_associate(card: &str, away: Away) {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 900) * 1_000_000_000;
    let mut r = c6_rig(card, end);
    let s = staged("OXIDRIVE_C6_IMAGE", "OXIDRIVE_C6_MANIFEST");
    let v = s.version();
    s.stage(&r);
    let mark = r.lines.len();
    let t0 = r.cs.now_ns;
    send(&r, &format!("c6_update {v}"));
    assert!(run_until(&mut r, mark, &format!("pushed {v}"), end), "the image was never pushed");
    let key = match away { Away::Renamed => { r.set_ap_ssid("esp32sim-elsewhere"); None } Away::Rekeyed => Some(r.set_ap_pmk([0x5a; 32])) };
    let gone = r.cs.now_ns;
    let first = run_until_line(&mut r, mark, &[&format!("confirmed {v}"), "marking this image invalid"], end);
    let back = r.cs.now_ns;
    match key { None => r.set_ap_ssid("esp32sim"), Some(k) => { r.set_ap_pmk(k); } }
    let failed = first == Some(1) && run_until(&mut r, mark, &format!("update to {v} failed: the C6 came back on 2.12.13"), end);
    if failed { settle(&mut r, "failed", end); }
    show(&r, mark);
    let guard = guard_since(&r, mark);
    for l in &guard { eprintln!("[oxidrive]   [c6] {l}"); }
    for l in since(&r, mark).iter().filter(|l| l.contains("bring-up failed") || l.contains("out of range") || l.contains("re-initialising")) { eprintln!("[oxidrive]   {l}"); }
    let at = |n: u64| (n - t0) as f64 / 1e9;
    let why = guard.iter().find(|l| l.contains("marking this image invalid")).cloned().unwrap_or_default();
    eprintln!("[oxidrive] from the command: network kept away {:.1} s, {} {:.1} s, recorded failed {:.1} s; P4 resets {}; {why}",
              at(gone), match first { Some(0) => "confirmed", Some(_) => "the guard reverted", None => "nothing" }, at(back),
              when_since(&r, mark, "failed: the C6 came back").map_or(f64::NAN, at), r.cs.summary.p4_resets);
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    assert_ne!(first, Some(0), "the image was confirmed while the C6 could not join the network");
    assert!(!guard.iter().any(|l| l.contains("confirmed by the host")), "the guard kept the image");
    assert!(failed, "the P4 did not meet 2.12.13 again");
    assert!(c6_state(&r).contains("\"fw\":\"2.12.13\""), "{}", c6_state(&r));
}

/// The access point goes by another name: the C6 finds no network. Its bring-ups fail fast
/// rather than time out, so the P4 re-initialises its radio after every fourth, resetting the C6
/// each time, and the guard's boot limit reverts the image.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXIDRIVE_C6_MANIFEST, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_c6_that_cannot_associate_reverts() {
    cannot_associate("oxidrive-c6-no-ap.img", Away::Renamed);
}

/// The access point takes another key: the C6 finds it and is refused at the 4-way handshake.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXIDRIVE_C6_MANIFEST, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_c6_refused_by_the_access_point_reverts() {
    cannot_associate("oxidrive-c6-rekeyed.img", Away::Rekeyed);
}

/// What the firmware's own log ring holds about the hosted link, and its heap, a while after
/// boot: the console does not carry the C components' lines past the start of `main`, the ring
/// (`diag log`) does. With the committed 2.12.13 slave under a 2.13.1 host the ring holds the
/// version mismatch; with a matching slave it does not. `OXIDRIVE_SLAVE_DIR` picks the slave.
#[test]
#[ignore = "needs OXIDRIVE_DIR, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxidrive_c6_link_report() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 120) * 1_000_000_000;
    let mut r = c6_rig("oxidrive-c6-link.img", end);
    let base = format!("oxidrive/{}", r.id);
    let heard = r.mqtt.client.listen(&[&format!("{base}/diag/rsp")]).unwrap();
    let t = r.cs.now_ns + 20_000_000_000;
    r.run_to(t);
    for (id, req) in [(1, r#"{"id":1,"cmd":"log","n":96}"#), (2, r#"{"id":2,"cmd":"heap"}"#)] {
        r.mqtt.client.publish(&format!("{base}/diag/req"), req).unwrap();
        let t = r.cs.now_ns + 3_000_000_000;
        r.run_to(t);
        for (_, p) in heard.messages().iter().filter(|(t, p)| t.ends_with("/diag/rsp") && p.contains(&format!(r#""id":{id},"#))) {
            if id == 2 { eprintln!("[oxidrive] heap: {p}"); continue; }
            for l in p.split("\",\"").filter(|l| ["H_SDIO", "transport", "mismatch", "rpc", "Co-proc", "slave", "RPC"].iter().any(|k| l.contains(k))) { eprintln!("[oxidrive] ring: {l}"); }
        }
    }
    let p4: Vec<&str> = r.p4_lines().into_iter().filter(|l| l.contains("c6: runs")).collect();
    eprintln!("[oxidrive] {p4:?}");
}

/// (c), through the owner's tooling: `bin/oxi-update c6-deploy` (`OXI_UPDATE` names it, its
/// `OXI_OTA_KEY_DIR` the throwaway key) pointed at this rig's share and broker stages the slave,
/// sends `c6_update`, follows the state document across the P4's restart, and exits 0 once the C6
/// reports the pushed ELF hash. The emulator runs while the script waits.
#[test]
#[ignore = "needs OXIDRIVE_DIR (a build with c6_update), OXIDRIVE_C6_IMAGE, OXI_UPDATE, OXI_OTA_KEY_DIR, OXIDRIVE_SLAVE_KNOWN, ESP-IDF's nvs_partition_gen.py, samba and mosquitto"]
fn external_oxi_update_c6_deploy_drives_it() {
    let end = Rig::env_secs("OXIDRIVE_SECONDS", 400) * 1_000_000_000;
    let mut r = c6_rig("oxidrive-c6-deploy.img", end);
    let script = std::env::var("OXI_UPDATE").expect("set OXI_UPDATE to oxidrive's bin/oxi-update");
    let image = std::env::var("OXIDRIVE_C6_IMAGE").expect("set OXIDRIVE_C6_IMAGE");
    let out = std::env::temp_dir().join(format!("oxi-update-c6-{}.log", std::process::id()));
    let mut child = std::process::Command::new(&script)
        .args(["c6-deploy", &image, "--timeout", "900"])
        .env("OXI_OTA_URL", format!("smb://127.0.0.1:{}/{}", r.smb.svc.port, r.smb.share))
        .env("OXI_OTA_USER", SHARE_USER).env("OXI_SMB_PASS", SHARE_PASS)
        .env("OXI_MQTT_HOST", "127.0.0.1").env("OXI_MQTT_PORT", r.mqtt.svc.port.to_string())
        .env("OXI_MQTT_USER", BROKER_USER).env("OXI_MQTT_PASS", BROKER_PASS).env("OXI_DEVICE_ID", &r.id)
        .stdin(std::process::Stdio::null()).stdout(std::fs::File::create(&out).unwrap()).stderr(std::process::Stdio::from(std::fs::File::create(out.with_extension("err")).unwrap()))
        .spawn().unwrap_or_else(|e| panic!("{script}: {e}"));
    let mark = r.lines.len();
    let status = loop {
        if let Some(s) = child.try_wait().unwrap() { break Some(s); }
        if r.cs.now_ns >= end { let _ = child.kill(); break None; }
        let t = r.cs.now_ns + 500_000_000;
        r.run_to(t);
    };
    let said = format!("{}{}", std::fs::read_to_string(&out).unwrap_or_default(), std::fs::read_to_string(out.with_extension("err")).unwrap_or_default());
    for l in said.lines().filter(|l| !l.starts_with("warning") && !l.trim_start().starts_with('|') && !l.trim().is_empty() && !l.contains("-->") && !l.trim_start().starts_with('=')) { eprintln!("[oxi-update] {}", l.split("sig=").next().unwrap()); }
    show(&r, mark);
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(5);
    assert_eq!(status.and_then(|s| s.code()), Some(0), "c6-deploy did not succeed");
    assert!(said.contains("read back after the restart"), "c6-deploy did not report the C6 confirmed");
}
