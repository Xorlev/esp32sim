//! The committed ESP32-P4 hello_world (`web/wasm/fw/public/p4-hello-*`, built from
//! `examples/hello_world-p4`) through the bootloader into the application, as far as the core
//! model allows. The plain run stops in the bootloader (`hello-p4` in cli/tests/goldens.rs); this
//! one goes on with a stand-in for the core's missing `cycle`/`cycleh` CSRs.
#[path = "../../tests/common.rs"]
mod common;
use common::*;
use emu_core::Core;

const FW: &str = "web/wasm/fw/public";

/// Stands in for the core's `cycle`/`cycleh` (0xC00/0xC80) aliases of `mcycle` until riscv-rv32
/// provides them: before every 64-instruction slice the CSRs the core stores but does not
/// implement are set to the cycle count. Delete with the shim when the core has them.
fn run_with_cycle_csr_shim(m: &mut esp32p4::Machine) -> esp_soc::Stop {
    loop {
        let c = &mut m.cores[0];
        let cycles = c.read_csr(0xb00) as u64 | (c.read_csr(0xb80) as u64) << 32;
        c.csr_other.insert(0xc00, cycles as u32);
        c.csr_other.insert(0xc80, (cycles >> 32) as u32);
        match m.run(64) { esp_soc::Stop::MaxInsns => continue, stop => return stop }
    }
}

/// With the cycle CSR in place the app reaches FreeRTOS: the heap regions and the clock are
/// the board's, and the run ends where the first interrupts would be taken. The systimer tick
/// (CLIC ID 18) and the scheduler's first yield (ID 17) are pending, and the core has no CLIC
/// mode to take them, so `vTaskStartScheduler` returns and `start_cpu0` spins on its final `j .`.
#[test]
#[ignore = "needs the ESP32-P4 mask ROM ELF"]
fn hello_world_p4_with_cycle_csr_shim() {
    let mut m = esp32p4::machine([0x30, 0xed, 0xa0, 0x00, 0x11, 0x22], 2 << 20);
    esp_soc::SocBus::set_flash_size(&mut m.bus, 2 << 20);
    m.load_rom(&std::fs::read(rom("esp32p4_rev0")).unwrap()).unwrap();
    for (off, file) in [(0x2000, "p4-hello-bootloader.bin"), (0x8000, "p4-hello-ptable.bin"), (0x10000, "p4-hello_world.bin")] {
        m.write_flash(off, &std::fs::read(root().join(FW).join(file)).unwrap()).unwrap();
    }
    m.boot_rom();
    m.console.capture = true;
    m.max_cycles = esp32p4::periph::CPU_HZ / 10;
    let stop = run_with_cycle_csr_shim(&mut m);
    assert!(matches!(stop, esp_soc::Stop::Halted), "{stop:?}");
    let console = String::from_utf8_lossy(&m.console.uart0).to_string();
    assert!(console.contains("cpu_start: cpu freq: 360000000 Hz"), "{console}");
    assert!(console.contains("heap_init: At 4FF40000 len 00060000 (384 KiB): RAM"), "{console}");
    // start_cpu0's `j .` after esp_startup_start_app returns (hello_world.elf of this build)
    assert_eq!(m.cores[0].pc(), 0x4000_0682);
    assert_eq!(m.bus.periph.clic.core[0].candidate().map(|(id, _)| id), Some(17));
    assert_eq!((m.exceptions, m.interrupts), (0, 0));
    expect_text("hello-p4-cycle-shim.console.txt", &console);
    expect_u64("hello-p4-cycle-shim.insns", m.insns());
}
