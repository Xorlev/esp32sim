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
//! P4; and `OXIDRIVE_SLAVE_KNOWN`, the slave builds' directories, so the C6's PHY stub follows the
//! image it boots (`hosted_slave`). esp32p4.md, "The C6's own update", has the method and results.
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
    while r.cs.now_ns < end_ns && when_since(r, from, what).is_none() { let t = r.cs.now_ns + 1_000_000_000; r.run_to(t); }
    when_since(r, from, what).is_some()
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
/// verified and pushed with the card taken, the C6 told to boot it, the P4 restarted, and the
/// new image's ELF hash read back from the C6 and reported `confirmed`.
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
    eprintln!("[oxidrive] from the command: card taken {:.1} s, pushed {:.1} s, P4 back {:.1} s, the C6 read {:.1} s, confirmed {:.1} s ({:.1} s wall)",
              at("card taken, fetching"), at(&format!("pushed {v}")), at("oxidrive bring-up"), at(&format!("c6: runs {v}")), at(&format!("confirmed {v}")), wall.elapsed().as_secs_f64());
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    assert!(confirmed, "no confirmation of {v}");
    assert!(when_since(&r, mark, "supervisor: card taken for a firmware update").is_some(), "the card was not taken");
    assert_eq!(r.cs.summary.p4_resets, 1, "the P4 restarts once, to meet the new image");
    let state = c6_state(&r);
    assert!(state.contains(&format!("\"fw\":\"{v}\"")) && state.contains(&format!("\"elf\":\"{}\"", s.elf8())) && state.contains("\"phase\":\"confirmed\""), "{state}");
    assert!(!since(&r, mark).iter().any(|l| l.contains("mismatch")), "a version mismatch after the update");
}

/// (c), refusals: an image whose bytes are not the manifest's, a manifest whose signature no key
/// made, and an unsigned one. Each is refused by name, the C6 keeps 2.12.13, the card comes back,
/// and nothing restarts.
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
    for (what, s, want) in [("flipped byte", &flipped, "crc32"), ("forged signature", &forged, "signature did not verify"), ("unsigned", &unsigned, "manifest is not signed")] {
        s.stage(&r);
        let mark = r.lines.len();
        send(&r, &format!("c6_update {v}"));
        let seen = run_until(&mut r, mark, want, end);
        let t = r.cs.now_ns + 3_000_000_000;
        r.run_to(t);
        show(&r, mark);
        let line = since(&r, mark).iter().find(|l| l.contains(want)).map(|l| l.to_string()).unwrap_or_default();
        outcomes.push((what, seen, line, when_since(&r, mark, "card taken").is_some(), when_since(&r, mark, "card returned").is_some()));
    }
    for (what, seen, line, taken, back) in &outcomes { eprintln!("[oxidrive] {what}: refused={seen} card taken={taken} returned={back}: {line}"); }
    eprintln!("[oxidrive] the state's c6: {}", c6_state(&r));
    r.report(10);
    for (what, seen, _, taken, back) in &outcomes {
        assert!(seen, "the {what} image was not refused by name");
        assert!(!taken || *back, "the {what} image kept the card");
    }
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
