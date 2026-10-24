//! The ESP-Hosted slave a test puts on the C6, and the stub each slave image needs: the committed
//! 2.12.13 slave by default, or a slave the owner built (`OXIDRIVE_SLAVE_DIR`), which is not in
//! this repository. Included with `#[path]` next to `common`.
//!
//! A slave's PHY calibration, `bb_init`, is stubbed at its address in that image, so a C6 that
//! installs another image over the air needs another stub. `track` points the stub at whichever
//! image the bootloader is about to load, after every reset.
#![allow(dead_code)]
use crate::common::*;

const FW: &str = "web/wasm/fw/public";
/// The committed slave's `bb_init` (riscv32-esp-elf-nm on its network_adapter.elf).
const COMMITTED_BB_INIT: u32 = 0x420b_ac5c;

/// One slave build: what the C6's flash holds before a test, and its `bb_init`.
#[derive(Clone)]
pub struct Slave {
    pub dir: String,
    pub bootloader: Vec<u8>,
    pub ptable: Vec<u8>,
    pub otadata: Vec<u8>,
    pub app: Vec<u8>,
    pub bb_init: u32,
}

/// The app descriptor's `app_elf_sha256`, at 0xb0 in an app image: what tells two builds of one
/// version apart.
pub fn elf_sha(app: &[u8]) -> Option<[u8; 32]> { app.get(0xb0..0xd0)?.try_into().ok() }
/// The app descriptor's version string.
pub fn version(app: &[u8]) -> String {
    app.get(0x30..0x50).map(|v| String::from_utf8_lossy(v.split(|&b| b == 0).next().unwrap_or(&[])).to_string()).unwrap_or_default()
}

impl Slave {
    /// ESP-Hosted 2.12.13's unmodified slave, committed as `c6-hosted-*.bin`.
    pub fn committed() -> Slave {
        let read = |name: &str| std::fs::read(root().join(FW).join(name)).unwrap_or_else(|e| panic!("{FW}/{name}: {e}"));
        Slave { dir: FW.into(), bootloader: read("c6-hosted-bootloader.bin"), ptable: read("c6-hosted-ptable.bin"),
                otadata: read("c6-hosted-otadata.bin"), app: read("c6-hosted-slave.bin"), bb_init: COMMITTED_BB_INIT }
    }

    /// A slave build in `dir`: an ESP-IDF project after `idf.py build` (its `build/`), or a flat
    /// directory of `bootloader.bin`, `partition-table.bin`, `ota_data_initial.bin`,
    /// `network_adapter.bin` and `network_adapter.elf`. `bb_init` is read from the ELF.
    pub fn from_dir(dir: &str) -> Slave {
        let base = if std::path::Path::new(dir).join("build/network_adapter.bin").exists() { format!("{dir}/build") } else { dir.to_string() };
        let read = |name: &str| std::fs::read(format!("{base}/{name}")).unwrap_or_else(|e| panic!("{base}/{name}: {e}"));
        let flat = |a: &str, b: &str| if std::path::Path::new(&format!("{base}/{a}")).exists() { read(a) } else { read(b) };
        let elf = esp_soc::elf::parse(&read("network_adapter.elf")).unwrap_or_else(|e| panic!("{base}/network_adapter.elf: {e}"));
        let bb_init = *elf.by_name.get("bb_init").unwrap_or_else(|| panic!("{base}/network_adapter.elf has no bb_init"));
        Slave { dir: dir.into(), bootloader: flat("bootloader.bin", "bootloader/bootloader.bin"), ptable: flat("partition-table.bin", "partition_table/partition-table.bin"),
                otadata: read("ota_data_initial.bin"), app: read("network_adapter.bin"), bb_init }
    }

    /// `OXIDRIVE_SLAVE_DIR`, else the committed slave.
    pub fn from_env() -> Slave {
        match std::env::var("OXIDRIVE_SLAVE_DIR") { Ok(d) if !d.is_empty() => Slave::from_dir(&d), _ => Slave::committed() }
    }

    /// The images at the offsets the slave's own build flashes them to.
    pub fn flash(&self, c6: &mut esp32c6::Machine) {
        for (off, data) in [(0, &self.bootloader), (0x8000, &self.ptable), (0xd000, &self.otadata), (0x10000, &self.app)] { c6.write_flash(off, data).unwrap(); }
    }
}

/// Every slave build a test may meet on the C6, by ELF hash: the one it starts with, the
/// committed one, and `OXIDRIVE_SLAVE_KNOWN` (directories, `:`-separated) for images a test
/// installs over the air.
pub fn known(first: &Slave) -> Vec<([u8; 32], u32, String)> {
    let mut all = vec![first.clone(), Slave::committed()];
    if let Ok(dirs) = std::env::var("OXIDRIVE_SLAVE_KNOWN") {
        all.extend(dirs.split(':').filter(|d| !d.is_empty()).map(Slave::from_dir));
    }
    all.iter().filter_map(|s| Some((elf_sha(&s.app)?, s.bb_init, version(&s.app)))).collect()
}

/// The app partitions' offsets, `ota_0` then `ota_1`, from the partition table at 0x8000.
fn ota_slots(flash: &[u8]) -> [usize; 2] {
    let mut slots = [0x10000, 0x1f0000];
    for e in flash.get(0x8000..0x8c00).unwrap_or(&[]).as_chunks::<32>().0.iter().take_while(|e| e[..2] == [0xaa, 0x50]) {
        if e[2] == 0x00 && (0x10..=0x11).contains(&e[3]) { slots[(e[3] - 0x10) as usize] = u32::from_le_bytes(e[4..8].try_into().unwrap()) as usize; }
    }
    slots
}

/// The slot the bootloader boots, as ESP-IDF v5.5's bootloader_utility picks it: the otadata
/// entry with the highest sequence number among those not blank, invalid (3) or aborted (4), its
/// slot `(seq - 1) % 2`; `ota_0` when there is none. CRCs are not checked: only ESP-IDF writes
/// these sectors here.
pub fn booted_slot(flash: &[u8]) -> usize {
    let entry = |off: usize| -> Option<u32> {
        let e = flash.get(off..off + 32)?;
        let seq = u32::from_le_bytes(e[0..4].try_into().ok()?);
        let state = u32::from_le_bytes(e[24..28].try_into().ok()?);
        (seq != u32::MAX && state != 3 && state != 4).then_some(seq)
    };
    match [entry(0xd000), entry(0xe000)].into_iter().flatten().max() { Some(seq) => ((seq - 1) % 2) as usize, None => 0 }
}

/// Point the C6's stubs at the image the bootloader will load now, and say which it is. A slot
/// holding an image the test does not know gets no stub and a warning.
pub fn stub_booted(c6: &mut esp32c6::Machine, known: &[([u8; 32], u32, String)]) {
    let flash = &c6.bus.flash;
    let slot = booted_slot(flash);
    let off = ota_slots(flash)[slot];
    let app = flash.get(off..off + 0x100).unwrap_or(&[]);
    let found = elf_sha(app).and_then(|sha| known.iter().find(|k| k.0 == sha));
    for (_, addr, _) in known { c6.stubs.remove(addr); }
    match found {
        Some((sha, addr, v)) => {
            c6.stubs.insert(*addr, 0);
            eprintln!("[c6] boots ota_{slot}: slave {v} ({:02x}{:02x}{:02x}{:02x}), bb_init stubbed at {addr:#x}", sha[0], sha[1], sha[2], sha[3]);
        }
        None => eprintln!("[c6] boots ota_{slot}: an image this test has no bb_init for ({:?}); its PHY calibration runs unstubbed", version(app)),
    }
}
