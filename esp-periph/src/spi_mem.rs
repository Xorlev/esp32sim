use crate::device::{Device, WriteEffect};
use crate::regram::RegRam;

/// Which array a SPI command wrote (the SoC maps this to its own buffer ids).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DirtyMem { Flash, Psram }

/// How long the flash array takes to program and erase, for a model whose writes take time: the
/// status register's WIP bit stays set for this long after the command. `None` on `SpiMem` keeps
/// every program and erase instant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FlashTiming {
    /// a page program: the first byte, and each byte after it
    pub program_first_ns: u64,
    pub program_byte_ns: u64,
    /// 4 KiB sector, 32 and 64 KiB block, and chip erase
    pub sector_ns: u64,
    pub block32_ns: u64,
    pub block64_ns: u64,
    pub chip_ns: u64,
}

impl FlashTiming {
    /// GigaDevice GD25Q-class typical figures (tBP1 30 us, tBP2 2.5 us, tSE 45 ms, tBE 150 and
    /// 250 ms, tCE 80 s): an assumption for a 32 MB part, not a measurement.
    pub const GD25Q_TYPICAL: FlashTiming = FlashTiming { program_first_ns: 30_000, program_byte_ns: 2_500, sector_ns: 45_000_000, block32_ns: 150_000_000, block64_ns: 250_000_000, chip_ns: 80_000_000_000 };

    pub fn program_ns(&self, bytes: usize) -> u64 { self.program_first_ns + self.program_byte_ns * bytes.saturating_sub(1) as u64 }
}

/// One timed program or erase: what was busy, and when.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FlashOp { pub start_ns: u64, pub end_ns: u64, pub addr: u32, pub len: u32, pub erase: bool }

// ------------------------------------------------------------------ SPI flash controller (SPI_MEM: SPI1 = command engine, SPI0 = cache path)
pub struct SpiMem {
    /// Whether a PSRAM device sits on CS1. True for the S3; the C3 has no PSRAM, and without
    /// this a flash command issued with CS0 disabled is misrouted to the PSRAM branch and
    /// answered with zeros (found on real C3 silicon: `E memspi: no response`).
    pub has_psram: bool,
    /// Classic ESP32 SPI_ADDR_REG is shifted out MSB first in user mode: for an address phase shorter
    /// than 32 bits the HAL writes `(addr << (32 - bitlen)) | ones` (IDF v5.5.4
    /// hal/esp32/include/hal/spi_flash_ll.h:365-377), so a 24-bit address sits in bits 31:8 and the
    /// ROM's SPIRead writes `addr << 8` for its 32-bit 0xBB/0xEB phases; only a full 32-bit address
    /// (4-byte-address commands, :368-370, not decoded here) stays right-aligned. The SPI_MEM
    /// controllers (S3/C3/C6) take the register right-aligned
    /// (esp32s3/include/hal/spimem_flash_ll.h:590-594). Dedicated flash commands (FLASH_READ/PP/SE/BE)
    /// keep the address in bits 23:0 and a length in 31:24 on both (hal/spi_flash_hal_iram.c:75-82;
    /// esp_rom/patches/esp_rom_spiflash.c:151-214).
    pub addr_msb_aligned: bool,
    pub regs: RegRam,
    /// (memory, offset, len) ranges the last command wrote, for the bus's page versions
    pub dirty: Vec<(DirtyMem, usize, usize)>,
    pub w: [u32; 16],
    pub pending_cmd: u32,
    pub status: u32,
    pub jedec: [u8; 3],
    pub is_spi1: bool,
    pub log: bool,
    /// octal PSRAM (APS6408-like) mode registers MR0..MR8, device on CS1 (SPI1 only)
    pub psram_mr: [u8; 9],
    /// program and erase times; `None` for instant ones
    pub timing: Option<FlashTiming>,
    /// emulated time, set by the bus before each command
    pub now_ns: u64,
    busy_until_ns: u64,
    /// the timed programs and erases since the bus last took them
    pub ops: Vec<FlashOp>,
}
impl SpiMem {
    pub fn new(is_spi1: bool) -> Self { SpiMem { has_psram: true, addr_msb_aligned: false, regs: RegRam::new(), dirty: Vec::new(), w: [0; 16], pending_cmd: 0, status: 0x200, jedec: [0x20, 0x40, 0x17], is_spi1, log: false , psram_mr: [0x09, 0x0d, 0x8b, 0x00, 0x20, 0, 0, 0, 0x03], timing: None, now_ns: 0, busy_until_ns: 0, ops: Vec::new() } }
    /// WIP while a timed program or erase is in progress.
    fn wip(&self) -> u32 { u32::from(self.now_ns < self.busy_until_ns) }
    /// A program or erase begins: busy for as long as `timing` says.
    fn busy(&mut self, addr: usize, len: usize, erase: bool) {
        let Some(t) = self.timing else { return };
        let ns = match (erase, len) {
            (false, n) => t.program_ns(n),
            (true, 0x1000) => t.sector_ns,
            (true, 0x8000) => t.block32_ns,
            (true, 0x10000) => t.block64_ns,
            (true, _) => t.chip_ns,
        };
        let start = self.now_ns.max(self.busy_until_ns);
        self.busy_until_ns = start + ns;
        self.ops.push(FlashOp { start_ns: start, end_ns: self.busy_until_ns, addr: addr as u32, len: len as u32, erase });
    }
    pub fn read(&self, off: u32) -> u32 {
        match off {
            0x0 => 0,                                   // CMD: always idle after execution
            0x2c => self.status | self.wip(),           // RD_STATUS
            0x54 => 0,                                  // FSM idle
            0x58..=0x94 => self.w[((off - 0x58) / 4) as usize],
            0xa4 => 0,                                  // SUS_STATUS
            0x3fc => 0x2101040,
            _ => self.regs.read(off),
        }
    }
    /// Returns true if a command must be executed against the flash array.
    pub fn write(&mut self, off: u32, v: u32) -> bool {
        match off {
            0x0 => { self.pending_cmd = v; return v & 0xffff_0003 != 0; }
            0x2c => {}   // RD_STATUS is written by hardware only
            0x58..=0x94 => self.w[((off - 0x58) / 4) as usize] = v,
            _ => self.regs.write(off, v),
        }
        false
    }
    fn w_bytes(&self, n: usize) -> Vec<u8> { let mut o = Vec::with_capacity(n); for i in 0..n { o.push((self.w[(i / 4) & 15] >> ((i % 4) * 8)) as u8); } o }
    fn set_w_bytes(&mut self, data: &[u8]) { for w in self.w.iter_mut() { *w = 0; } for (i, b) in data.iter().enumerate().take(64) { self.w[i / 4] |= (*b as u32) << ((i % 4) * 8); } }

    pub fn execute(&mut self, flash: &mut [u8], psram: &mut [u8]) {
        let cmd = self.pending_cmd; self.pending_cmd = 0;
        let user = self.regs.read(0x18); let user1 = self.regs.read(0x1c); let user2 = self.regs.read(0x20);
        let addr_reg = self.regs.read(0x4);
        let miso_bytes = ((self.regs.read(0x28) & 0x3ff) as usize + 8) / 8;
        let mosi_bytes = ((self.regs.read(0x24) & 0x3ff) as usize + 8) / 8;
        let addr_bits = ((user1 >> 26) & 0x3f) + 1;
        // User-command address: right-aligned on SPI_MEM; MSB-aligned on the classic ESP32, where the
        // flash sees the first 24 of the `addr_bits` shifted bits as the address: the HAL's DIO/QIO
        // phase is 28 bits (the continuous-mode bits follow, spi_flash_hal_common.inc:104-138), the
        // ROM's is 32, and the classic PSRAM driver sends an opcode as an 8-bit "address"
        // (esp_psram_impl_quad.c:495-512).
        let addr = if self.addr_msb_aligned {
            if addr_bits >= 24 { addr_reg >> 8 } else { addr_reg >> (32 - addr_bits) }
        } else if addr_bits > 24 { addr_reg } else { addr_reg & 0xff_ffff };
        // Dedicated flash commands (bits 31..22) take the address from bits 23:0 on the classic.
        let fixed = if self.addr_msb_aligned { addr_reg & 0xff_ffff } else { addr };
        let fsize = flash.len();
        let rd = |a: u32, n: usize| -> Vec<u8> { (0..n).map(|i| { let x = a as usize + i; if x < fsize { flash[x] } else { 0xff } }).collect() };
        // IDF v5.5.5 components/soc/esp32s3/register/soc/spi_mem_reg.h:
        // USR bit 18 (110-113), MISO/MOSI bits 28/27 (383-391), CS disable (560-570).
        let misc = self.regs.read(0x34);
        if self.has_psram && cmd & (1 << 18) != 0 && misc & 1 != 0 && misc & 2 == 0 {   // USR command with CS0 disabled, CS1 enabled: PSRAM
            let c16 = user2 & 0xffff;
            let has_miso = user & (1 << 28) != 0; let has_mosi = user & (1 << 27) != 0;
            if self.log { eprintln!("[spi1] psram cmd {:#06x} addr {:#x} miso {} mosi {}", c16, addr, if has_miso { miso_bytes } else { 0 }, if has_mosi { mosi_bytes } else { 0 }); }
            let psize = psram.len();
            if psize == 0 {
                if has_miso { self.set_w_bytes(&vec![0xff; miso_bytes]); }
                return;
            }
            // IDF v5.5.5 components/esp_psram/device/esp_quad_psram_defs_ap.h:21-60;
            // v4.4.8 components/esp_hw_support/port/esp32s3/spiram_psram.c:46-83
            // use the same commands, KGD and 2/4/8 MiB density encoding.
            // Revision bits and absent-device 0xff are inferred model choices.
            match c16 {
                0x9f => {
                    let density = match psize {
                        0x200000 => 0x02, 0x400000 => 0x22, 0x800000 => 0x42,
                        _ => { self.set_w_bytes(&vec![0xff; miso_bytes]); return; }
                    };
                    self.set_w_bytes(&[0x0d, 0x5d, density][..miso_bytes.min(3)]);
                }
                0x4040 => { let i = (addr & 0xf) as usize; let d: Vec<u8> = (0..miso_bytes).map(|k| *self.psram_mr.get(i + k).unwrap_or(&0)).collect(); self.set_w_bytes(&d); }   // mode register read
                0xC0C0 => { let d = self.w_bytes(mosi_bytes); let i = (addr & 0xf) as usize; for (k, b) in d.iter().enumerate() { if i + k == 0 || i + k == 8 { self.psram_mr[i + k] = *b; } } }   // mode register write (MR0/MR8 writable)
                0x02 | 0x38 | 0x8080 => { let d = self.w_bytes(mosi_bytes); self.dirty.push((DirtyMem::Psram, addr as usize, d.len())); for (k, b) in d.iter().enumerate() { let x = addr as usize + k; if x < psize { psram[x] = *b; } } }   // sync write
                0x03 | 0x0b | 0xeb | 0x0000 => { let d: Vec<u8> = (0..miso_bytes).map(|k| { let x = addr as usize + k; if x < psize { psram[x] } else { 0 } }).collect(); self.set_w_bytes(&d); }   // sync read
                _ => { if has_miso { self.set_w_bytes(&vec![0u8; miso_bytes]); } }
            }
            return;
        }
        if cmd & (1 << 18) != 0 {   // USR: command from USER2
            let c = if user & (1 << 31) != 0 { (user2 & 0xff) as u8 } else { 0 };
            let has_addr = user & (1 << 30) != 0;
            let has_miso = user & (1 << 28) != 0;
            let has_mosi = user & (1 << 27) != 0;
            if self.log { eprintln!("[spi1] usr cmd {:#04x} addr {:#x}{} miso {} mosi {}", c, addr, if has_addr { "" } else { " (no addr)" }, if has_miso { miso_bytes } else { 0 }, if has_mosi { mosi_bytes } else { 0 }); }
            match c {
                0x03 | 0x0b | 0x3b | 0x6b | 0xbb | 0xeb => { let d = rd(addr, miso_bytes); self.set_w_bytes(&d); }
                0x9f => { let j = self.jedec; self.set_w_bytes(&j); }
                0x05 => { let s = self.status | self.wip(); self.set_w_bytes(&[s as u8]); }
                0x35 => { let s = self.status; self.set_w_bytes(&[(s >> 8) as u8]); }
                0x06 => self.status |= 0x02,                       // WREN: set WEL
                0x04 => self.status &= !0x02,                      // WRDI
                0x01 | 0x31 | 0x11 => self.status &= !0x02,        // WRSR*: latch consumed (keep QE set)
                0x15 => self.set_w_bytes(&[0x00]),
                0x02 | 0x32 | 0x38 => { let d = self.w_bytes(mosi_bytes); for (i, b) in d.iter().enumerate() { let x = addr as usize + i; if x < fsize { flash[x] &= *b; } } self.dirty.push((DirtyMem::Flash, addr as usize, d.len())); self.status &= !0x02; self.busy(addr as usize, d.len(), false); }
                0x20 => { let a = (addr as usize) & !0xfff; for b in flash.iter_mut().take((a + 0x1000).min(fsize)).skip(a) { *b = 0xff; } self.dirty.push((DirtyMem::Flash, a, 0x1000)); self.status &= !0x02; self.busy(a, 0x1000, true); }
                0x52 => { let a = (addr as usize) & !0x7fff; for b in flash.iter_mut().take((a + 0x8000).min(fsize)).skip(a) { *b = 0xff; } self.dirty.push((DirtyMem::Flash, a, 0x8000)); self.status &= !0x02; self.busy(a, 0x8000, true); }
                0xd8 => { let a = (addr as usize) & !0xffff; for b in flash.iter_mut().take((a + 0x10000).min(fsize)).skip(a) { *b = 0xff; } self.dirty.push((DirtyMem::Flash, a, 0x10000)); self.status &= !0x02; self.busy(a, 0x10000, true); }
                0xc7 | 0x60 => { for b in flash.iter_mut() { *b = 0xff; } self.dirty.push((DirtyMem::Flash, 0, fsize)); self.status &= !0x02; self.busy(0, fsize, true); }
                _ => { if has_miso { self.set_w_bytes(&vec![0u8; miso_bytes]); } }
            }
            return;
        }
        if cmd & (1 << 31) != 0 { let d = rd(fixed, miso_bytes); self.set_w_bytes(&d); }                       // FLASH_READ
        if cmd & (1 << 28) != 0 { let j = self.jedec; self.set_w_bytes(&j); }                                   // RDID
        if cmd & (1 << 30) != 0 { self.status |= 0x02; }                                                         // WREN
        if cmd & (1 << 29) != 0 { self.status &= !0x02; }                                                        // WRDI
        if cmd & (1 << 26) != 0 { self.status &= !0x02; }                                                        // WRSR
        // RDSR (bit 27): RD_STATUS already reflects the live status register
        if cmd & (1 << 25) != 0 {                                                                                // PP
            let n = if addr_reg >> 24 != 0 { (addr_reg >> 24) as usize } else { mosi_bytes };
            let d = self.w_bytes(n);
            for (i, b) in d.iter().enumerate() { let x = (fixed & 0xff_ffff) as usize + i; if x < fsize { flash[x] &= *b; } }
            self.dirty.push((DirtyMem::Flash, (fixed & 0xff_ffff) as usize, d.len()));
            self.status &= !0x02;
            self.busy((fixed & 0xff_ffff) as usize, d.len(), false);
        }
        if cmd & (1 << 24) != 0 { let a = (fixed as usize) & !0xfff; for b in flash.iter_mut().take((a + 0x1000).min(fsize)).skip(a) { *b = 0xff; } self.dirty.push((DirtyMem::Flash, a, 0x1000)); self.status &= !0x02; self.busy(a, 0x1000, true); }      // SE
        if cmd & (1 << 23) != 0 { let a = (fixed as usize) & !0xffff; for b in flash.iter_mut().take((a + 0x10000).min(fsize)).skip(a) { *b = 0xff; } self.dirty.push((DirtyMem::Flash, a, 0x10000)); self.busy(a, 0x10000, true); }    // BE
        if cmd & (1 << 22) != 0 { for b in flash.iter_mut() { *b = 0xff; } self.dirty.push((DirtyMem::Flash, 0, fsize)); self.busy(0, fsize, true); }                          // CE
    }
}

impl Device for SpiMem {
    fn read(&mut self, off: u32) -> u32 { SpiMem::read(self, off) }
    /// Only SPI1 is the command engine; SPI0 is the cache path and its command bit is ignored.
    fn write(&mut self, off: u32, v: u32) -> WriteEffect { if SpiMem::write(self, off, v) && self.is_spi1 { WriteEffect::SPI_EXEC } else { WriteEffect::NONE } }
    fn debug(&mut self, on: bool) { self.log = on; }
}

#[cfg(test)]
mod tests {
    use super::{FlashOp, FlashTiming, SpiMem};

    /// RDSR through a user command, at `now`: the status register's low byte.
    fn rdsr(spi: &mut SpiMem, flash: &mut [u8], now: u64) -> u32 {
        spi.now_ns = now;
        spi.write(0x34, 2);
        spi.write(0x18, (1 << 31) | (1 << 28));
        spi.write(0x20, 0x05);
        spi.write(0x28, 7);
        spi.write(0, 1 << 18);
        spi.execute(flash, &mut []);
        spi.w[0] & 0xff
    }

    /// A timed program or erase holds WIP for its time, through RDSR and RD_STATUS alike, and is
    /// logged; untimed, nothing is ever busy.
    #[test]
    fn a_timed_program_and_erase_hold_wip_for_their_time() {
        let mut flash = vec![0xffu8; 0x20000];
        let mut spi = SpiMem::new(true);
        let t = FlashTiming { program_first_ns: 30_000, program_byte_ns: 2_500, sector_ns: 45_000_000, block32_ns: 1, block64_ns: 1, chip_ns: 1 };
        spi.timing = Some(t);
        spi.now_ns = 1_000;
        spi.write(0x04, (4 << 24) | 0x100);
        spi.write(0x58, 0);
        spi.write(0, 1 << 25);
        spi.execute(&mut flash, &mut []);
        assert_eq!(t.program_ns(4), 37_500);
        assert_eq!(rdsr(&mut spi, &mut flash, 38_499) & 1, 1, "programming");
        assert_eq!(spi.read(0x2c) & 1, 1, "RD_STATUS agrees");
        assert_eq!(rdsr(&mut spi, &mut flash, 38_500) & 1, 0, "programmed");
        spi.now_ns = 100_000;
        spi.write(0x04, 0x1000);
        spi.write(0, 1 << 24);
        spi.execute(&mut flash, &mut []);
        assert_eq!(rdsr(&mut spi, &mut flash, 45_099_999) & 1, 1, "erasing");
        assert_eq!(rdsr(&mut spi, &mut flash, 45_100_000) & 1, 0, "erased");
        assert_eq!(spi.ops, vec![
            FlashOp { start_ns: 1_000, end_ns: 38_500, addr: 0x100, len: 4, erase: false },
            FlashOp { start_ns: 100_000, end_ns: 45_100_000, addr: 0x1000, len: 0x1000, erase: true },
        ]);
        let mut untimed = SpiMem::new(true);
        untimed.write(0x04, 0x1000);
        untimed.write(0, 1 << 24);
        untimed.execute(&mut flash, &mut []);
        assert_eq!(rdsr(&mut untimed, &mut flash, 0) & 1, 0);
        assert!(untimed.ops.is_empty());
    }

    /// IDF v5.5.4 hal/esp32/include/hal/spi_flash_ll.h:365-378: the classic ESP32 shifts SPI_ADDR_REG out MSB first,
    /// so a 24-bit user address is written as `(addr << 8) | 0xff`, and the DIO read 0xBB uses a 28-bit address
    /// phase for the continuous-mode bits (spi_flash_hal_common.inc:104-138). Dedicated commands keep the address
    /// in bits 23:0 with the length in 31:24 (spi_flash_hal_iram.c:75-82). The SPI_MEM controllers take the user
    /// address right-aligned (esp32s3/include/hal/spimem_flash_ll.h:590-594).
    #[test]
    fn classic_user_address_is_msb_aligned_and_dedicated_commands_keep_bits_23_0() {
        let mut flash = vec![0xffu8; 0x400000];
        flash[0x200000..0x200004].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        let user_read = |spi: &mut SpiMem, flash: &mut [u8], addr_bits: u32, cmd: u32| {
            spi.write(0x34, 2);                                   // CS0: the flash
            spi.write(0x18, (1 << 31) | (1 << 30) | (1 << 28));   // USER: command, address, miso phases
            spi.write(0x1c, (addr_bits - 1) << 26);               // USER1: address bit length
            spi.write(0x20, cmd);                                 // USER2: command
            spi.write(0x04, (0x200000 << 8) | 0xff);             // ADDR as the classic HAL writes it
            spi.write(0x28, 31);                                  // MISO_DLEN: 32 bits
            spi.write(0x58, 0);
            spi.write(0, 1 << 18);
            spi.execute(flash, &mut []);
            spi.w[0]
        };
        let mut spi = SpiMem::new(true);
        spi.addr_msb_aligned = true;
        for (addr_bits, cmd) in [(24, 0x03), (28, 0xbb)] {
            assert_eq!(user_read(&mut spi, &mut flash, addr_bits, cmd), 0x44332211, "user read {cmd:#x}, {addr_bits}-bit address phase");
        }
        spi.write(0x04, (4 << 24) | 0x200100);                   // PP: length 4 at 0x200100
        spi.write(0x58, 0xa5a5a5a5);
        spi.write(0, 1 << 25);
        spi.execute(&mut flash, &mut []);
        assert_eq!(&flash[0x200100..0x200104], &[0xa5; 4]);
        spi.write(0x04, 0x200100);                               // SE: the sector holding both
        spi.write(0, 1 << 24);
        spi.execute(&mut flash, &mut []);
        assert_eq!(&flash[0x200000..0x200004], &[0xff; 4]);
        // The ROM's SPIRead: a 32-bit 0xEB phase with the address in bits 31:8 and a zero low byte.
        flash[0x200000..0x200004].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        spi.write(0x1c, 31 << 26);
        spi.write(0x20, 0xeb);
        spi.write(0x04, 0x200000 << 8);
        spi.write(0x58, 0);
        spi.write(0, 1 << 18);
        spi.execute(&mut flash, &mut []);
        assert_eq!(spi.w[0], 0x44332211, "ROM-style 32-bit phase");
        // A phase shorter than 24 bits takes its leading bits (the PSRAM driver's 8-bit opcode phase).
        flash[0x12..0x16].copy_from_slice(&[0xa1, 0xb2, 0xc3, 0xd4]);
        spi.write(0x1c, 7 << 26);
        spi.write(0x20, 0x03);
        spi.write(0x04, 0x12 << 24);
        spi.write(0x58, 0);
        spi.write(0, 1 << 18);
        spi.execute(&mut flash, &mut []);
        assert_eq!(spi.w[0], 0xd4c3b2a1, "8-bit phase");
        // Right-aligned decoding reads 0x200000ff, past a 4 MiB array.
        let mut spi = SpiMem::new(true);
        assert_eq!(user_read(&mut spi, &mut flash, 24, 0x03), 0xffffffff);
    }

    #[test]
    fn cs1_quad_id_tracks_capacity_and_keeps_octal_and_flash_ids() {
        for (size, id) in [(0, 0xffffff), (0x100000, 0xffffff), (0x200000, 0x025d0d), (0x400000, 0x225d0d), (0x800000, 0x425d0d)] {
            let mut spi = SpiMem::new(true);
            let mut psram = vec![0; size];
            spi.write(0x18, (1 << 31) | (1 << 28));
            spi.write(0x20, 0x9f);
            spi.write(0x28, 47);
            spi.write(0x34, 1);
            spi.write(0, 1 << 18);
            spi.execute(&mut [], &mut psram);
            assert_eq!(spi.w[0] & 0xffffff, id);
            if size == 0 {
                spi.write(0x20, 0x4040);
                spi.write(0, 1 << 18);
                spi.execute(&mut [], &mut psram);
                assert_eq!(spi.w[0], u32::MAX);
            } else {
                spi.write(0x20, 0x4040);
                spi.write(0, 1 << 18);
                spi.execute(&mut [], &mut psram);
                assert_eq!(spi.w[0] & 0xffffff, 0x8b0d09);
                for write in [0x02, 0x38, 0x8080] {
                    spi.write(0x24, 31);
                    let value = 0x12340000 | write;
                    spi.write(0x58, value);
                    spi.write(0x20, write);
                    spi.write(0, 1 << 18);
                    spi.execute(&mut [], &mut psram);
                    for read in [0x03, 0x0b, 0xeb, 0x0000] {
                        spi.write(0x58, 0);
                        spi.write(0x20, read);
                        spi.write(0, 1 << 18);
                        spi.execute(&mut [], &mut psram);
                        assert_eq!(spi.w[0], value);
                    }
                }
            }
            spi.write(0x34, 2);
            spi.write(0x20, 0x9f);
            spi.write(0, 1 << 18);
            spi.execute(&mut [], &mut psram);
            assert_eq!(spi.w[0] & 0xffffff, 0x174020);
        }
    }
}
