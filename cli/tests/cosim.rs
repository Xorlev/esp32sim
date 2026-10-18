//! The P4 and its C6 co-processor in lock-step (`esp32sim::cosim`). The first tests need no
//! firmware: both cores spin in a loop while the test drives the P4's SDMMC host and GPIO through
//! its bus, as ESP-Hosted's host does, and watches where the C6 stands. The last ones run the
//! committed ESP-Hosted pair from the mask ROMs.
#[path = "../../tests/common.rs"]
mod common;
#[path = "../../tests/sd_image.rs"]
mod sd_image;
#[path = "../../tests/oxidrive_board.rs"]
mod oxidrive_board;
use common::*;
use emu_core::{Bus, Core};
use esp32sim::cosim::{c6_cycle_of_ns, p4_cycle_of_ns, CoSim, Config, CoprocSpec};
use esp_soc::SocBus;

const SDMMC: u32 = 0x5008_3000;
const GPIO: u32 = 0x500e_0000;
const J_SELF: [u8; 4] = [0x6f, 0, 0, 0];

/// Both chips spinning: the P4 in L2MEM, the C6 in SRAM, its ROM's reset vector a loop too.
fn spinning(cfg: Config) -> CoSim {
    let mut p4 = esp32p4::machine([0x30, 0xed, 0xa0, 0, 0, 1], 1 << 20);
    p4.bus.load_bytes(0x4ff0_0000, &J_SELF).unwrap();
    p4.cores[0].pc = 0x4ff0_0000;
    let mut c6 = esp32c6::machine([0x40, 0x4c, 0xca, 0, 0, 1], 1 << 20);
    c6.bus.load_bytes(0x4080_0000, &J_SELF).unwrap();
    c6.bus.load_bytes(0x4000_0000, &J_SELF).unwrap();
    c6.cores[0].pc = 0x4080_0000;
    CoSim::new(p4, c6, cfg)
}

/// Slot 1's clock on pin 18 through the GPIO matrix (IO MUX function 1, SD_CARD_CCLK_2 out),
/// four lines, as `sdmmc_host_init_slot` leaves it.
fn route_slot1(cs: &mut CoSim) {
    cs.p4.bus.write32(0x500e_1004 + 4 * 18, 1 << 12).unwrap();
    cs.p4.bus.write32(GPIO + 0x558 + 4 * 18, 0).unwrap();
    cs.p4.bus.write32(SDMMC + 0x18, 1 << 1).unwrap();
}

/// A command on slot 1 with a short response; returns RESP0, or None on a response timeout.
fn command(cs: &mut CoSim, index: u32, arg: u32) -> Option<u32> {
    cs.p4.bus.write32(SDMMC + 0x44, u32::MAX).unwrap();
    cs.p4.bus.write32(SDMMC + 0x28, arg).unwrap();
    cs.p4.bus.write32(SDMMC + 0x2c, 1 << 31 | 1 << 16 | 1 << 6 | index).unwrap();
    let rint = cs.p4.bus.read32(SDMMC + 0x44).unwrap();
    assert_ne!(rint & 1 << 2, 0, "CMD_DONE");
    (rint & 1 << 8 == 0).then(|| cs.p4.bus.read32(SDMMC + 0x30).unwrap())
}

/// Run the P4 alone to `ns`, as it runs inside a slice before the C6 catches up.
fn p4_alone(cs: &mut CoSim, ns: u64) {
    cs.p4.max_cycles = p4_cycle_of_ns(ns);
    cs.p4.run(u64::MAX);
    cs.p4.max_cycles = u64::MAX;
}

#[test]
fn slices_end_with_both_chips_at_the_same_instant() {
    let mut cs = spinning(Config::default());
    cs.run_until(1_000_000).unwrap();
    assert_eq!(cs.p4.bus.cycles(), p4_cycle_of_ns(1_000_000));
    assert_eq!(cs.c6.borrow().now_cycles(), c6_cycle_of_ns(1_000_000));
    assert_eq!((cs.p4.bus.cycles(), cs.c6.borrow().m.bus.cycles()), (360_000, 160_000), "1 ms at 360 and 160 MHz");
    assert_eq!(cs.summary.slices, 100, "10 µs slices");
}

/// A command is where the two meet: the C6 runs to the command's time before it answers, and is
/// never ahead of the P4.
#[test]
fn the_c6_meets_each_command_at_its_time() {
    let mut cs = spinning(Config::default());
    cs.run_until(100_000).unwrap();
    route_slot1(&mut cs);
    p4_alone(&mut cs, 157_000);
    assert_eq!(cs.c6.borrow().now_cycles(), c6_cycle_of_ns(100_000), "the C6 waits behind the P4");
    let r4 = command(&mut cs, 5, 0).expect("the slave answers CMD5");
    assert_eq!(r4, 0xa0ff_8000, "R4: ready, two functions, 2.7-3.6 V");
    let p4_ns = cs.p4.bus.periph.now_ns();
    assert_eq!(cs.c6.borrow().now_cycles(), c6_cycle_of_ns(p4_ns), "the C6 at the command's nanosecond");
    assert!(p4_ns >= 157_000);
    // the rest of IDF's sequence: CMD5 with the OCR, CMD3, CMD7, then CMD52 (sdmmc_card_init)
    command(&mut cs, 5, 0x30_0000).unwrap();
    let rca = command(&mut cs, 3, 0).unwrap() & 0xffff_0000;
    assert_eq!(rca, 0x1_0000);
    command(&mut cs, 7, rca).unwrap();
    assert_eq!(command(&mut cs, 52, 0x08 << 9), Some(0x1000 | esp32c6::sdio::CARD_CAP as u32), "CMD52 reads CCCR 0x08");
    assert_eq!(command(&mut cs, 8, 0x1aa), None, "an I/O-only card does not answer CMD8");
}

/// The slave's interrupt reaches the P4 on DAT1 by the end of the slice: D1's input level goes low
/// and the slot's RINTSTS SDIO bit is set on that edge; the host clearing it releases D1.
#[test]
fn dat1_carries_the_slave_interrupt_to_the_p4() {
    let mut cs = spinning(Config::default());
    route_slot1(&mut cs);
    for (cmd, arg) in [(5, 0), (5, 0x30_0000), (3, 0)] { command(&mut cs, cmd, arg); }
    command(&mut cs, 7, 0x1_0000).unwrap();
    command(&mut cs, 52, 1 << 31 | 0x04 << 9 | 0x03).unwrap();          // IENM and IEN1
    assert!(cs.p4.bus.periph.gpio.g.level(15), "D1 high: nothing pending");
    {
        let mut c = cs.c6.borrow_mut();
        c.m.bus.write32(0x6001_80dc, 0xff).unwrap();                  // SLCHOST FUNC1_INT_ENA
        c.m.bus.write32(0x6001_705c, 1 << 3).unwrap();                // SLC INTVEC_TOHOST
    }
    cs.run_until(10_000).unwrap();
    assert!(!cs.p4.bus.periph.gpio.g.level(15), "D1 low at the slice end");
    assert_ne!(cs.p4.bus.read32(SDMMC + 0x44).unwrap() & 1 << 17, 0, "RINTSTS SDIO interrupt, slot 1");
    // the host's interrupt clear: CMD52 to INT_CLR, and DAT1 is sampled after the command
    command(&mut cs, 52, 1 << 31 | 1 << 28 | 0xd4 << 9 | 1 << 3).unwrap();
    assert!(cs.p4.bus.periph.gpio.g.level(15), "D1 released after the clearing command");
}

/// EN on the P4's GPIO 54: driven low it holds the C6 where it is; driven high again it powers the
/// C6 up from its ROM, which then runs on the shared clock from that instant.
#[test]
fn en_holds_the_c6_and_powers_it_up_again() {
    let mut cs = spinning(Config::default());
    cs.run_until(50_000).unwrap();
    cs.p4.bus.write32(GPIO + 0x18, 1 << 22).unwrap();                 // OUT1_W1TC: GPIO 54 low
    cs.p4.bus.write32(GPIO + 0x30, 1 << 22).unwrap();                 // ENABLE1_W1TS
    cs.run_until(60_000).unwrap();
    let held = cs.c6.borrow().m.bus.cycles();
    assert!(cs.c6.borrow().held);
    assert_eq!(held, c6_cycle_of_ns(60_000), "EN seen at the slice end");
    cs.run_until(200_000).unwrap();
    assert_eq!(cs.c6.borrow().m.bus.cycles(), held, "held in reset");
    route_slot1(&mut cs);
    assert_eq!(command(&mut cs, 5, 0), None, "a chip in reset does not answer");
    cs.p4.bus.write32(GPIO + 0x14, 1 << 22).unwrap();                 // OUT1_W1TS: high
    cs.run_until(210_000).unwrap();
    {
        let c = cs.c6.borrow();
        assert!(!c.held && c.power_ups == 1);
        assert_eq!(c.m.cores[0].pc(), 0x4000_0000, "from the ROM's reset vector");
        assert_eq!(c.m.bus.reset_cause(), esp_periph::RST_POWERON);
        assert_eq!(c.now_cycles(), c6_cycle_of_ns(210_000));
    }
    cs.p4.bus.write32(GPIO + 0x34, 1 << 22).unwrap();                 // ENABLE1_W1TC: the pull-up
    cs.run_until(300_000).unwrap();
    assert_eq!(cs.summary.en_changes, 2, "an undriven EN stays high");
    let inverted = spinning(Config { en_inverted: true, ..Config::default() });
    assert!(!inverted.c6.borrow().held, "undriven: not in reset either way");
}

#[test]
fn the_coproc_spec_parses_its_keys() {
    let s = CoprocSpec::parse("c6:bootloader=b.bin,ptable=p.bin,app=a.bin,otadata=o.bin@0xe000,stub=bb_init=0,stub=0x4200=1,en=!53,slice-us=5,flash-mb=8,mac=02:00:00:00:00:01").unwrap();
    assert_eq!((s.bootloader.as_deref(), s.ptable.as_deref(), s.app.as_deref()), (Some("b.bin"), Some("p.bin"), Some("a.bin")));
    assert_eq!(s.otadata, Some(("o.bin".into(), 0xe000)));
    assert_eq!(s.stubs, ["bb_init=0", "0x4200=1"]);
    assert_eq!((s.en_pin, s.en_inverted, s.slice_ns, s.flash_mb, s.mac), (Some(53), true, Some(5000), Some(8), Some([2, 0, 0, 0, 0, 1])));
    assert_eq!(CoprocSpec::parse("c6:app=a.bin,otadata=o.bin").unwrap().otadata, Some(("o.bin".into(), 0xd000)));
    for bad in ["s3:app=a", "c6:bootloader=b", "c6:app=a,colour=red", "c6:app=a,en=64", "c6:app=a,slice-us=0", "c6:app"] {
        assert!(CoprocSpec::parse(bad).is_err(), "{bad}");
    }
}

// ------------------------------------------------------------------ with firmware
const FW: &str = "web/wasm/fw/public";

/// The committed ESP-Hosted slave on a C6 booted from its mask ROM, the virtual access point on
/// its radio and no NAT behind it.
fn hosted_slave() -> esp32c6::Machine {
    let read = |name: &str| std::fs::read(root().join(FW).join(name)).unwrap_or_else(|e| panic!("{FW}/{name}: {e}"));
    let mut c6 = esp32c6::machine([0x40, 0x4c, 0xca, 0x00, 0x11, 0x22], 4 << 20);
    c6.bus.set_flash_size(4 << 20);
    c6.load_rom(&std::fs::read(rom("esp32c6_rev0")).unwrap()).unwrap();
    for (off, f) in [(0, "c6-hosted-bootloader.bin"), (0x8000, "c6-hosted-ptable.bin"), (0xd000, "c6-hosted-otadata.bin"), (0x10000, "c6-hosted-slave.bin")] { c6.write_flash(off, &read(f)).unwrap(); }
    // bb_init, the PHY's baseband calibration (riscv32-esp-elf-nm on network_adapter.elf)
    c6.stubs.insert(0x420b_ac5c, 0);
    c6.bus.attach_wifi(esp_soc::wifi::ApConfig::parse("ssid=esp32sim,psk=esp32sim-pass").unwrap(), None).unwrap();
    c6.boot_rom();
    c6
}

/// The committed ESP-Hosted pair, booted from the mask ROMs: IDF's station example on the P4,
/// ESP-Hosted's slave on the C6.
fn hosted_pair(cfg: Config) -> CoSim {
    let read = |name: &str| std::fs::read(root().join(FW).join(name)).unwrap_or_else(|e| panic!("{FW}/{name}: {e}"));
    let mut p4 = esp32p4::machine([0x30, 0xed, 0xa0, 0x00, 0x11, 0x22], 2 << 20);
    p4.bus.set_flash_size(2 << 20);
    p4.load_rom(&std::fs::read(rom("esp32p4_rev0")).unwrap()).unwrap();
    for (off, f) in [(0x2000, "p4-hosted-bootloader.bin"), (0x8000, "p4-hosted-ptable.bin"), (0x10000, "p4-hosted-station.bin")] { p4.write_flash(off, &read(f)).unwrap(); }
    p4.boot_rom();
    CoSim::new(p4, hosted_slave(), cfg)
}

fn transcript(cs: &mut CoSim) -> String {
    cs.lines.drain(..).map(|(c6, l)| format!("{}{l}\n", if c6 { "[c6] " } else { "" })).collect()
}

/// Determinism of the lock-step: the same inputs give the same run to the instruction, however
/// the run is split into calls, and every console line in the same order.
#[test]
#[ignore = "needs the ESP32-P4 and ESP32-C6 mask ROM ELFs"]
fn the_pair_runs_the_same_whichever_way_it_is_driven() {
    let mut whole = hosted_pair(Config::default());
    whole.run_until(2_000_000_000).unwrap();
    let mut pieces = hosted_pair(Config::default());
    let mut t = 0;
    while t < 2_000_000_000 { t += 7_919_000; pieces.run_until(t.min(2_000_000_000)).unwrap(); }
    let (a, b) = (transcript(&mut whole), transcript(&mut pieces));
    assert!(a.contains("H_SDIO_DRV: Card init success") && a.contains("transport: Identified slave [esp32c6]"), "the transport came up:\n{a}");
    assert_eq!(a, b);
    assert_eq!((whole.p4.insns(), whole.c6.borrow().m.insns()), (pieces.p4.insns(), pieces.c6.borrow().m.insns()));
    assert_eq!(whole.p4.interrupts, pieces.p4.interrupts);
}

/// The owner's oxidrive firmware (not in this repository) on the P4, with the committed
/// ESP-Hosted 2.12.13 slave on the C6 as on its board, and the firmware's NVS partition made at
/// test time (`oxidrive_board::nvs_image`) from `nvs.csv` beside the images: the virtual access
/// point's credentials and placeholders that reach nothing (`--net none`: every name resolves to
/// 10.0.2.3, every connection is refused). Neither the CSV nor the image is in this repository.
/// Reports how far it gets in `OXIDRIVE_SECONDS` (20) of emulated time, both consoles' last lines,
/// and how the P4's console compares with the board's capture line by line.
#[test]
#[ignore = "needs the oxidrive images and nvs.csv under ~/Code/dlp/sim-fixtures and ESP-IDF's nvs_partition_gen.py"]
fn external_oxidrive_brings_up_wifi_through_the_c6() {
    let (mut p4, board_log, _card) = oxidrive_board::boot("oxidrive-cosim.img", false);
    let dir = oxidrive_board::dir();
    let csv = std::path::Path::new(&dir).join("nvs.csv");
    assert!(csv.exists(), "{}: missing; write the NVS generator CSV (key,type,encoding,value; the namespace row, then the WiFi credentials esp32sim / esp32sim-pass and placeholders for the rest) there", csv.display());
    let table = std::fs::read(format!("{dir}/partition-table.bin")).unwrap();
    let (off, size) = oxidrive_board::nvs_partition(&table).expect("an NVS partition in partition-table.bin");
    p4.write_flash(off, &oxidrive_board::nvs_image(&csv, size)).unwrap();
    let secs = std::env::var("OXIDRIVE_SECONDS").ok().and_then(|v| v.parse::<u64>().ok()).unwrap_or(20);
    let mut cs = CoSim::new(p4, hosted_slave(), Config::default());
    let t0 = std::time::Instant::now();
    let r = cs.run_until(secs * 1_000_000_000);
    let (mut p4_lines, mut c6_lines) = (Vec::new(), Vec::new());
    for (c6, l) in cs.lines.drain(..) { if c6 { c6_lines.push(l) } else { p4_lines.push(l) } }
    let emu: Vec<String> = p4_lines.iter().map(|l| oxidrive_board::strip_timestamp(l)).collect();
    let board = oxidrive_board::board_lines(&board_log);
    let cmp = oxidrive_board::compare(&emu, &board);
    let c = cs.c6.borrow();
    let st = &c.m.bus.periph.optional.sdio.stats;
    eprintln!("[oxidrive] {} s in {:.1} s wall ({r:?}); P4 {} instructions, C6 {}; SDIO {} commands, {} bytes to the P4, {} from it; {} EN changes",
              secs, t0.elapsed().as_secs_f64(), cs.p4.insns(), c.m.insns(), st.commands, st.to_host, st.from_host, cs.summary.en_changes);
    eprintln!("[oxidrive] {}", esp_soc::SocBus::report(&c.m.bus).replace('\n', "\n[oxidrive] "));
    eprintln!("[oxidrive] {} of {} board lines match, {} blocks differ, the comparison reached board line {}", cmp.same, board.len(), cmp.blocks.len(), cmp.reached + 1);
    eprintln!("[oxidrive] the P4's last lines:");
    for l in p4_lines.iter().rev().take(40).rev() { eprintln!("[oxidrive]   {l}"); }
    eprintln!("[oxidrive] the C6's last lines:");
    for l in c6_lines.iter().rev().take(25).rev() { eprintln!("[oxidrive]   [c6] {l}"); }
    assert!(st.to_host > 0 && st.from_host > 0, "no packet crossed the SDIO link");
    assert!(c6_lines.iter().any(|l| l.contains("Sta mode connected")), "the C6 did not join the access point");
    // the virtual network's DHCP lease, which the firmware logs once WiFi is up
    assert!(p4_lines.iter().any(|l| l.contains("10.0.2.15")), "no lease through the C6");
}
