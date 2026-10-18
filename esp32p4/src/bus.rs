//! ESP32-P4 memory map (IDF v5.5.3 components/soc/esp32p4/include/soc/soc.h and ext_mem_defs.h).
//!
//! L2MEM (768 KB at 0x4FF00000) holds code and data; the 128 KB mask ROM sits at 0x4FC00000, the
//! 8 KB TCM at 0x30100000, the LP ROM and LP RAM at 0x50100000 / 0x50108000. Flash
//! (0x40000000) and PSRAM (0x48000000) are 64 MB windows, each behind its own 1024-entry MMU of
//! 64 KB pages, programmed through the item index / content registers of the flash controller
//! (SPI_MEM_C) and of the PSRAM controller (SPI_MEM_S). Each of these has a non-cacheable alias
//! at +0x4000_0000 (SOC_NON_CACHEABLE_OFFSET); caches are not modelled, so both views are one.

use crate::periph::Peripherals;
use riscv_rv32::bus::{Bus, Fault};

pub const ROM_LOW: u32 = 0x4FC0_0000;
pub const ROM_HIGH: u32 = 0x4FC2_0000;
pub const L2MEM_LOW: u32 = 0x4FF0_0000;
pub const L2MEM_HIGH: u32 = 0x4FFC_0000;
pub const TCM_LOW: u32 = 0x3010_0000;
pub const TCM_HIGH: u32 = 0x3010_2000;
pub const LP_ROM_LOW: u32 = 0x5010_0000;
pub const LP_ROM_HIGH: u32 = 0x5010_4000;
pub const LP_RAM_LOW: u32 = 0x5010_8000;
pub const LP_RAM_HIGH: u32 = 0x5011_0000;
pub const FLASH_LOW: u32 = 0x4000_0000;
pub const FLASH_HIGH: u32 = 0x4400_0000;
pub const PSRAM_LOW: u32 = 0x4800_0000;
pub const PSRAM_HIGH: u32 = 0x4C00_0000;
pub const UNCACHED_OFFSET: u32 = 0x4000_0000;
pub const MMU_ENTRIES: usize = 1024;
pub const MMU_PAGE_SHIFT: u32 = 16;
/// Flash MMU entry: page in bits 10:0, VALID bit 12 (SENSITIVE bit 13 marks encrypted pages).
pub const MMU_FLASH_VALID: u32 = 1 << 12;
pub const MMU_FLASH_PAGE_MASK: u32 = 0x7ff;
/// PSRAM MMU entry: page in bits 9:0, ACCESS_PSRAM bit 10, VALID bit 11.
pub const MMU_PSRAM_VALID: u32 = 1 << 11;
pub const MMU_PSRAM_PAGE_MASK: u32 = 0x3ff;
/// The controllers the MMUs are reached through, and their item registers
/// (register/hw_ver1/soc/spi_mem_c_reg.h:2627-2654, spi_mem_s_reg.h).
pub const FLASH_SPI0: u32 = 0x5008_C000;
pub const PSRAM_MSPI0: u32 = 0x5008_E000;
pub const SPI_MMU_ITEM_CONTENT: u32 = 0x37c;
pub const SPI_MMU_ITEM_INDEX: u32 = 0x380;
/// CPU-subsystem window (CLIC, debug), CPU peripherals (cache, ASSIST_DEBUG), HP and LP peripherals.
const PERIPH_RANGES: [(u32, u32); 4] = [(0x2000_0000, 0x3000_0000), (0x3FF0_0000, 0x3FF2_0000), (0x5000_0000, 0x5010_0000), (0x5011_0000, 0x5013_0000)];

/// Which MMU an item register belongs to.
#[derive(Clone, Copy)]
enum Mmu { Flash, Psram }

pub struct SocBus {
    pub rom: Vec<u8>,
    pub l2mem: Vec<u8>,
    pub tcm: Vec<u8>,
    pub lp_rom: Vec<u8>,
    pub lp_ram: Vec<u8>,
    pub flash: Vec<u8>,
    pub psram: Vec<u8>,
    pub mmu_flash: Box<[u32; MMU_ENTRIES]>,
    pub mmu_psram: Box<[u32; MMU_ENTRIES]>,
    pub mmu_flash_index: u32,
    pub mmu_psram_index: u32,
    pub periph: Peripherals,
    /// wafer version the efuses report, kept across a reset
    pub wafer: (u32, u32),
    pub board: esp_soc::Board,
    pub cycles: u64,
    pub last_fault: Option<(u32, bool)>,
    /// a peripheral write may have moved an interrupt line: re-derive before the next instruction
    pub irq_dirty: bool,
    pub gpio_events: Option<Vec<(u64, u8, bool)>>,
    pub debug: esp_soc::DebugFlags,
    /// CLIC IDs already reported as pending but not deliverable, per core
    pub(crate) undelivered: [u64; 2],
    /// core 1's release from reset has been reported
    pub(crate) core1_reported: std::cell::Cell<bool>,
}

impl SocBus {
    pub fn new(flash_size: usize, mac: [u8; 6], wafer: (u32, u32)) -> Self {
        SocBus {
            rom: vec![0; (ROM_HIGH - ROM_LOW) as usize], l2mem: vec![0; (L2MEM_HIGH - L2MEM_LOW) as usize], tcm: vec![0; (TCM_HIGH - TCM_LOW) as usize],
            lp_rom: vec![0; (LP_ROM_HIGH - LP_ROM_LOW) as usize], lp_ram: vec![0; (LP_RAM_HIGH - LP_RAM_LOW) as usize],
            flash: vec![0xff; flash_size], psram: Vec::new(),
            mmu_flash: Box::new([0; MMU_ENTRIES]), mmu_psram: Box::new([0; MMU_ENTRIES]), mmu_flash_index: 0, mmu_psram_index: 0,
            periph: Peripherals::new(mac, wafer), wafer, board: Box::new(esp_soc::NoBoard),
            cycles: 0, last_fault: None, irq_dirty: true, gpio_events: None, debug: Default::default(), undelivered: [0; 2], core1_reported: Default::default(),
        }
    }

    /// Resolve to (buffer, offset, writable). The flash and PSRAM windows go through their MMUs.
    fn resolve(&mut self, addr: u32) -> Option<(&mut Vec<u8>, usize, bool)> {
        let a = if (0x8000_0000..0x9000_0000).contains(&addr) { addr - UNCACHED_OFFSET } else { addr };
        match a {
            L2MEM_LOW..=0x4FFB_FFFF => Some((&mut self.l2mem, (a - L2MEM_LOW) as usize, true)),
            FLASH_LOW..=0x43FF_FFFF => {
                let entry = self.mmu_flash[((a - FLASH_LOW) >> MMU_PAGE_SHIFT) as usize];
                if entry & MMU_FLASH_VALID == 0 { return None; }
                let off = (((entry & MMU_FLASH_PAGE_MASK) as usize) << MMU_PAGE_SHIFT) | (a & 0xffff) as usize;
                if off < self.flash.len() { Some((&mut self.flash, off, false)) } else { None }
            }
            PSRAM_LOW..=0x4BFF_FFFF => {
                let entry = self.mmu_psram[((a - PSRAM_LOW) >> MMU_PAGE_SHIFT) as usize];
                if entry & MMU_PSRAM_VALID == 0 { return None; }
                let off = (((entry & MMU_PSRAM_PAGE_MASK) as usize) << MMU_PAGE_SHIFT) | (a & 0xffff) as usize;
                if off < self.psram.len() { Some((&mut self.psram, off, true)) } else { None }
            }
            ROM_LOW..=0x4FC1_FFFF => Some((&mut self.rom, (a - ROM_LOW) as usize, false)),
            TCM_LOW..=0x3010_1FFF => Some((&mut self.tcm, (a - TCM_LOW) as usize, true)),
            LP_RAM_LOW..=0x5010_FFFF => Some((&mut self.lp_ram, (a - LP_RAM_LOW) as usize, true)),
            LP_ROM_LOW..=0x5010_3FFF => Some((&mut self.lp_rom, (a - LP_ROM_LOW) as usize, false)),
            _ => None,
        }
    }

    #[inline]
    fn is_periph(addr: u32) -> bool { PERIPH_RANGES.iter().any(|&(lo, hi)| (lo..hi).contains(&addr)) }

    fn mmu_reg(a: u32) -> Option<(Mmu, u32)> {
        let o = a & 0xfff;
        if o != SPI_MMU_ITEM_CONTENT && o != SPI_MMU_ITEM_INDEX { return None; }
        match a & !0xfff { FLASH_SPI0 => Some((Mmu::Flash, o)), PSRAM_MSPI0 => Some((Mmu::Psram, o)), _ => None }
    }

    fn mmu_read(&self, m: Mmu, o: u32) -> u32 {
        match (m, o) {
            (Mmu::Flash, SPI_MMU_ITEM_INDEX) => self.mmu_flash_index,
            (Mmu::Flash, _) => self.mmu_flash[self.mmu_flash_index as usize % MMU_ENTRIES],
            (Mmu::Psram, SPI_MMU_ITEM_INDEX) => self.mmu_psram_index,
            (Mmu::Psram, _) => self.mmu_psram[self.mmu_psram_index as usize % MMU_ENTRIES],
        }
    }

    fn mmu_write(&mut self, m: Mmu, o: u32, v: u32) {
        match (m, o) {
            (Mmu::Flash, SPI_MMU_ITEM_INDEX) => self.mmu_flash_index = v & 0x3ff,
            (Mmu::Flash, _) => self.mmu_flash[self.mmu_flash_index as usize % MMU_ENTRIES] = v & 0x3fff,
            (Mmu::Psram, SPI_MMU_ITEM_INDEX) => self.mmu_psram_index = v & 0x3ff,
            (Mmu::Psram, _) => self.mmu_psram[self.mmu_psram_index as usize % MMU_ENTRIES] = v & 0x1fff,
        }
    }

    fn periph_read(&mut self, addr: u32, size: u32) -> u32 {
        let a = addr & !3;
        let w = match Self::mmu_reg(a) { Some((m, o)) => self.mmu_read(m, o), None => self.periph.read32(a) };
        match size { 1 => (w >> ((addr & 3) * 8)) & 0xff, 2 => (w >> ((addr & 2) * 8)) & 0xffff, _ => w }
    }

    fn periph_write(&mut self, addr: u32, v: u32, size: u32) {
        let a = addr & !3;
        let merge = |old: u32| match size {
            4 => v,
            1 => { let sh = (addr & 3) * 8; (old & !(0xff << sh)) | ((v & 0xff) << sh) }
            _ => { let sh = (addr & 2) * 8; (old & !(0xffff << sh)) | ((v & 0xffff) << sh) }
        };
        if let Some((m, o)) = Self::mmu_reg(a) { let old = self.mmu_read(m, o); self.mmu_write(m, o, merge(old)); return; }
        let v = if size == 4 { v } else { merge(self.periph.read32(a)) };
        self.periph.write32(a, v);
        if self.periph.spi_exec { self.run_spi(); }
        if self.periph.psram_exec { self.run_psram(); }
        self.irq_dirty = true;
    }

    /// Execute a pending SPI1 command against the flash image.
    fn run_spi(&mut self) {
        self.periph.spi_exec = false;
        self.periph.spi1.execute(&mut self.flash, &mut []);
        self.periph.spi1.dirty.clear();
    }

    /// Execute a pending PSRAM controller command against the PSRAM array.
    fn run_psram(&mut self) {
        self.periph.psram_exec = false;
        self.periph.psram1.execute(&mut [], &mut self.psram);
        self.periph.psram1.dirty.clear();
    }

    /// Write straight into flash (image loaders, not the guest).
    pub fn write_flash(&mut self, offset: usize, data: &[u8]) -> Result<(), String> {
        let target = self.flash.get_mut(offset..).and_then(|tail| tail.get_mut(..data.len())).ok_or("flash image too large")?;
        target.copy_from_slice(data);
        Ok(())
    }

    pub fn load_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), String> {
        // The mask ROM ELF links its .eh_frame at the flash window; nothing ever reads it.
        if (FLASH_LOW..FLASH_HIGH).contains(&addr) { return Ok(()); }
        for (i, b) in data.iter().enumerate() {
            let a = addr.wrapping_add(i as u32);
            match self.resolve(a) {
                Some((buf, off, _)) if off < buf.len() => buf[off] = *b,
                _ => return Err(format!("load: address {:#010x} not mapped", a)),
            }
        }
        Ok(())
    }

    /// Re-derive each core's CLIC input. Nothing is presented to the cores, so a candidate that
    /// would be taken is reported once per ID instead.
    pub(crate) fn refresh_lines(&mut self) -> bool {
        let changed = self.periph.refresh_lines();
        for core in 0..2 {
            let Some((id, level)) = self.periph.clic.core[core].candidate() else { continue };
            if self.undelivered[core] & (1 << id) != 0 { continue; }
            self.undelivered[core] |= 1 << id;
            let srcs: Vec<String> = self.periph.sources_of(core, id as u32).iter().map(|&s| format!("{} ({})", s, crate::periph::src::name(s))).collect();
            let c = &self.periph.clic.core[core];
            eprintln!("[esp32p4] core {} CLIC: interrupt id {} (CPU line {}, source {}) pending at level {:#x} over threshold {:#x}, {}; not delivered: the core has no CLIC mode (mtvt vectoring, mintstatus, mcause.mpil)",
                core, id, id as i32 - crate::periph::CLIC_EXT_OFFSET as i32, if srcs.is_empty() { "none".into() } else { srcs.join(", ") }, level, c.thresh >> 24, if c.vectored(id) { "hardware-vectored" } else { "not vectored" });
        }
        changed
    }

    fn devices(&mut self, cycles: u32) {
        if self.periph.spi_exec { self.run_spi(); }
        self.periph.tick(cycles as u64);
    }
}

macro_rules! rd {
    ($self:ident, $addr:expr, $n:expr, $conv:expr) => {{
        let addr = $addr;
        match $self.resolve(addr) {
            Some((b, o, _)) if b.len().saturating_sub(o) >= $n => Ok($conv(&b[o..o + $n])),
            _ => { $self.last_fault = Some((addr, false)); Err(Fault::Unmapped) }
        }
    }};
}

impl Bus for SocBus {
    fn note_code_page(&mut self, _vidx: u32) {} // This bus has no decode cache.
    fn read8(&mut self, addr: u32) -> Result<u8, Fault> {
        if Self::is_periph(addr) { return Ok(self.periph_read(addr, 1) as u8); }
        rd!(self, addr, 1, |b: &[u8]| b[0])
    }
    fn read16(&mut self, addr: u32) -> Result<u16, Fault> {
        if Self::is_periph(addr) { return Ok(self.periph_read(addr, 2) as u16); }
        rd!(self, addr, 2, |b: &[u8]| u16::from_le_bytes(b.try_into().unwrap()))
    }
    fn read32(&mut self, addr: u32) -> Result<u32, Fault> {
        if Self::is_periph(addr) { return Ok(self.periph_read(addr, 4)); }
        rd!(self, addr, 4, |b: &[u8]| u32::from_le_bytes(b.try_into().unwrap()))
    }
    fn write8(&mut self, addr: u32, v: u8) -> Result<(), Fault> {
        if Self::is_periph(addr) { self.periph_write(addr, v as u32, 1); return Ok(()); }
        match self.resolve(addr) {
            Some((b, o, true)) if o < b.len() => { b[o] = v; Ok(()) }
            _ => { self.last_fault = Some((addr, true)); Err(Fault::Prohibited) }
        }
    }
    fn write16(&mut self, addr: u32, v: u16) -> Result<(), Fault> {
        if Self::is_periph(addr) { self.periph_write(addr, v as u32, 2); return Ok(()); }
        match self.resolve(addr) {
            Some((b, o, true)) if o + 2 <= b.len() => { b[o..o + 2].copy_from_slice(&v.to_le_bytes()); Ok(()) }
            _ => { self.last_fault = Some((addr, true)); Err(Fault::Prohibited) }
        }
    }
    fn write32(&mut self, addr: u32, v: u32) -> Result<(), Fault> {
        if Self::is_periph(addr) { self.periph_write(addr, v, 4); return Ok(()); }
        match self.resolve(addr) {
            Some((b, o, true)) if o + 4 <= b.len() => { b[o..o + 4].copy_from_slice(&v.to_le_bytes()); Ok(()) }
            _ => { self.last_fault = Some((addr, true)); Err(Fault::Prohibited) }
        }
    }
    fn fetch(&mut self, pc: u32) -> Result<[u8; 4], Fault> {
        match self.resolve(pc) {
            Some((b, o, _)) if o < b.len() => {
                let mut r = [0u8; 4];
                for (i, x) in r.iter_mut().enumerate() { if let Some(&v) = b.get(o + i) { *x = v; } }
                Ok(r)
            }
            _ => { self.last_fault = Some((pc, false)); Err(Fault::Unmapped) }
        }
    }
    fn tick(&mut self, cycles: u32) -> u32 {
        self.cycles += cycles as u64;
        self.devices(cycles);
        1
    }
    #[inline(always)]
    fn note_pc(&mut self, pc: u32) { self.periph.misc.cur_pc = pc; }
    #[inline(always)]
    fn block_break(&self) -> bool { self.irq_dirty }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flash_window_and_its_uncached_alias_go_through_the_mmu() {
        let mut b = SocBus::new(0x40000, [0; 6], (1, 0));
        b.flash[0x2_0010..0x2_0014].copy_from_slice(&[1, 2, 3, 4]);
        assert!(b.read32(0x4001_0010).is_err());
        b.write32(FLASH_SPI0 + SPI_MMU_ITEM_INDEX, 1).unwrap();
        b.write32(FLASH_SPI0 + SPI_MMU_ITEM_CONTENT, MMU_FLASH_VALID | 2).unwrap();
        assert_eq!(b.read32(FLASH_SPI0 + SPI_MMU_ITEM_CONTENT).unwrap(), MMU_FLASH_VALID | 2);
        assert_eq!(b.read32(0x4001_0010).unwrap(), 0x0403_0201);
        assert_eq!(b.read32(0x8001_0010).unwrap(), 0x0403_0201);
        assert!(b.write32(0x4001_0010, 0).is_err());
    }

    #[test]
    fn psram_window_is_writable_through_its_own_mmu() {
        let mut b = SocBus::new(0x1000, [0; 6], (1, 0));
        b.psram = vec![0; 0x20000];
        b.write32(PSRAM_MSPI0 + SPI_MMU_ITEM_INDEX, 3).unwrap();
        b.write32(PSRAM_MSPI0 + SPI_MMU_ITEM_CONTENT, MMU_PSRAM_VALID | 1 << 10 | 1).unwrap();
        b.write32(0x4803_0004, 0xdead_beef).unwrap();
        assert_eq!(&b.psram[0x1_0004..0x1_0008], &0xdead_beefu32.to_le_bytes());
        assert_eq!(b.read32(0x8803_0004).unwrap(), 0xdead_beef);
        assert_eq!(b.read32(FLASH_SPI0 + SPI_MMU_ITEM_CONTENT).unwrap(), 0);
    }

    #[test]
    fn l2mem_tcm_and_lp_ram_are_one_store_per_alias() {
        let mut b = SocBus::new(0x1000, [0; 6], (1, 0));
        b.write32(0x4FF4_0000, 0x1234_5678).unwrap();
        assert_eq!(b.read32(0x8FF4_0000).unwrap(), 0x1234_5678);
        b.write8(0x3010_1FFF, 0xab).unwrap();
        assert_eq!(b.read8(0x3010_1FFF).unwrap(), 0xab);
        b.write16(0x5010_FFFE, 0xcdef).unwrap();
        assert_eq!(b.read16(0x5010_FFFE).unwrap(), 0xcdef);
        assert!(b.write32(ROM_LOW, 0).is_err());
        assert!(b.read32(0x4FFC_0000).is_err());
    }

    /// A user command on the PSRAM controller's command side, as psram_ctrlr_ll_common_transaction
    /// issues it: 16-bit command, 32-bit address, CS1 (IDF v5.5.3 hal/esp32p4/include/hal/psram_ctrlr_ll.h:754-798).
    fn psram_cmd(b: &mut SocBus, cmd: u32, addr: u32, mosi: Option<u32>, miso_bits: u32) -> u32 {
        const BASE: u32 = 0x5008_F000;
        let user = 1 << 31 | 1 << 30 | if mosi.is_some() { 1 << 27 } else { 0 } | if miso_bits > 0 { 1 << 28 } else { 0 };
        for (o, v) in [(0x34, 1), (0x18, user), (0x1c, 31 << 26), (0x20, 15 << 28 | cmd), (0x04, addr), (0x24, 31), (0x28, miso_bits.saturating_sub(1))] { b.write32(BASE + o, v).unwrap(); }
        if let Some(v) = mosi { b.write32(BASE + 0x58, v).unwrap(); }
        b.write32(BASE, 1 << 18).unwrap();
        assert_eq!(b.read32(BASE).unwrap(), 0, "USR clears when the command is done");
        b.read32(BASE + 0x58).unwrap()
    }

    #[test]
    fn hex_psram_answers_its_mode_registers_and_shares_the_window() {
        let mut b = SocBus::new(0x1000, [0; 6], (1, 0));
        b.psram = vec![0; 0x20000];
        assert_eq!(psram_cmd(&mut b, 0x4040, 0, None, 16) & 0xffff, 0x0d30);   // MR0, MR1 vendor AP
        assert_eq!(psram_cmd(&mut b, 0x4040, 2, None, 16) & 0xffff, 0x20df);   // MR2 256 Mbit, MR3 slow refresh
        psram_cmd(&mut b, 0x8080, 0x1_0004, Some(0x5a6b_7c8d), 0);
        assert_eq!(psram_cmd(&mut b, 0x0000, 0x1_0004, None, 32), 0x5a6b_7c8d);
        b.write32(PSRAM_MSPI0 + SPI_MMU_ITEM_INDEX, 0).unwrap();
        b.write32(PSRAM_MSPI0 + SPI_MMU_ITEM_CONTENT, MMU_PSRAM_VALID | 1 << 10 | 1).unwrap();
        assert_eq!(b.read32(PSRAM_LOW + 4).unwrap(), 0x5a6b_7c8d);
    }

    #[test]
    fn byte_writes_merge_into_clic_control_words() {
        let mut b = SocBus::new(0x1000, [0; 6], (1, 0));
        let ctrl = 0x2080_1000 + 4 * 20;
        b.write8(ctrl + 1, 1).unwrap();                               // IE, as esprv_intc_int_enable writes it
        b.write8(ctrl + 3, 2 << 5).unwrap();                          // priority 2
        assert_eq!(b.read32(ctrl).unwrap(), 0x40c0_0100);
        assert_eq!(b.read8(ctrl + 3).unwrap(), 0x40);
    }
}
