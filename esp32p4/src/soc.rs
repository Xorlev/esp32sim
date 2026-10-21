//! The ESP32-P4 as a `Soc`: two RISC-V HP cores on `bus::SocBus`, core 1 held in reset until
//! HP_SYS_CLKRST releases it.
use crate::bus::{SocBus, FLASH_HIGH, FLASH_LOW, MMU_ENTRIES, MMU_FLASH_VALID, MMU_PAGE_SHIFT};
use crate::periph::{self, CLIC_EXT_OFFSET};
use esp_periph::Misc;
use esp_soc::{BoardModel, CoreState, Soc};
use riscv_rv32::exec::{CLIC_IRQ_LEVEL_SHIFT, CLIC_IRQ_SHV};
use riscv_rv32::Cpu;
use std::marker::PhantomData;

/// What differs between ESP32-P4 silicon revisions. rev >= 3 (ECO5) moves the interrupt threshold
/// into the `mintthresh` CSR, the L2 cache to the bottom of L2MEM and has its own ROM; it becomes
/// a second implementation of this trait.
pub trait Revision: 'static {
    /// The mask ROM ELF's name in espressif/esp-rom-elfs.
    const ROM_ELF: &'static str;
    /// (major, minor) wafer version the efuses report.
    const WAFER: (u32, u32);
}

/// rev < 3 (v0.x, v1.x), as on the Waveshare ESP32-P4-WIFI6 (v1.0). The ECO0 ROM is the only one
/// published for it; its jump table matches IDF v5.5.3's esp32p4.rom.ld, so images built for
/// v1.x link against it, but its first two console lines are ECO0's.
pub struct RevLt3;
impl Revision for RevLt3 {
    const ROM_ELF: &'static str = "esp32p4_rev0_rom.elf";
    const WAFER: (u32, u32) = (1, 0);
}

pub struct Esp32p4<R: Revision>(PhantomData<R>);
pub type P4 = Esp32p4<RevLt3>;
pub type Machine = esp_soc::Machine<P4>;

/// The mask ROM's reset vector (IDF v5.5.3 include/soc/soc.h SOC_IROM_MASK_LOW).
pub const RESET_VECTOR: u32 = 0x4FC0_0000;

pub fn machine(mac: [u8; 6], flash_size: usize) -> Machine { machine_rev::<RevLt3>(mac, flash_size) }

pub fn machine_rev<R: Revision>(mac: [u8; 6], flash_size: usize) -> esp_soc::Machine<Esp32p4<R>> {
    let mut m = esp_soc::Machine::new(mac, SocBus::new(flash_size, mac, R::WAFER));
    m.set_debug(&esp_soc::DebugFlags::from_env());
    m
}

impl<R: Revision> Soc for Esp32p4<R> {
    type Core = Cpu;
    type Bus = SocBus;
    const NAME: &'static str = "esp32p4";
    /// IDF v5.5.3 components/bootloader/Kconfig.projbuild: BOOTLOADER_OFFSET_IN_FLASH 0x2000.
    const BOOTLOADER_OFFSET: usize = 0x2000;
    const ROM_ELF: &'static str = R::ROM_ELF;
    const CPU_HZ: u64 = periph::CPU_HZ;
    const CORES: usize = 2;
    const IDLE_CHUNK: u64 = 64;
    const ROM_BOOT_DEFAULT: bool = true;
    /// 12-byte (dst_start, dst_end, rom_src) entries, as on the C6.
    const ROM_DATA_TABLE: &'static [&'static str] = &["_data_table_start"];
    const ROM_DATA_TABLE_END: &'static [&'static str] = &["_data_table_end"];
    const ROM_DATA_TABLE_STRIDE: u32 = 12;
    /// RV32IMAFC with the P4's EXT_ILL CSR and CLIC mode; Xesppie and the hardware loops decode
    /// as illegal. `mhartid` is the core's index.
    fn new_core(i: usize) -> Cpu { let mut c = Cpu::new_esp32p4(); c.hartid = i as u32; c.pc = RESET_VECTOR; c }
    /// Both cores reset into the mask ROM, which sends core 1 to the boot address in LP_SYSTEM.
    fn reset_core(c: &mut Cpu, _i: usize) { Cpu::reset(c); c.pc = RESET_VECTOR; }
    /// A reset core has no interrupt input until the lines are presented again.
    fn reset_core_with_bus(c: &mut Cpu, bus: &mut SocBus, i: usize) { Self::reset_core(c, i); bus.irq_dirty = true; }
    fn boot_core(c: &mut Cpu, entry: u32) {
        Self::reset_core(c, 0);
        c.pc = entry;
        c.x[2] = 0x4FF3_CFC0;                 // SOC_ROM_STACK_START, the stack the ROM leaves the bootloader
    }
    /// Each core's CLIC candidate, as the core's CLIC mode takes it (`riscv_rv32::exec::CLIC_IRQ_*`).
    /// IDs 0-15 are the core's own and IDF routes nothing to them.
    fn irqs(bus: &SocBus, out: &mut [Option<u32>]) {
        for (core, o) in out.iter_mut().enumerate() {
            let c = &bus.periph.clic.core[core];
            *o = c.candidate().filter(|&(id, _)| id >= CLIC_EXT_OFFSET as usize).map(|(id, level)| {
                (id as u32 - CLIC_EXT_OFFSET) | level << CLIC_IRQ_LEVEL_SHIFT | if c.vectored(id) { CLIC_IRQ_SHV } else { 0 }
            });
        }
    }
    /// A hardware-vectored edge interrupt is no longer pending once taken.
    fn interrupt_taken(bus: &mut SocBus, core: usize, line: u32) {
        bus.periph.clic.core[core & 1].taken((line + CLIC_EXT_OFFSET) as usize);
        bus.irq_dirty = true;
    }
    fn select_core(bus: &mut SocBus, core: usize) { bus.cur_core = core; }
    /// Core 1 runs once HP_SYS_CLKRST gives it its clock and releases its reset.
    fn core_state(bus: &SocBus, core: usize) -> CoreState {
        if core == 0 || bus.periph.hp_clkrst.core1_released() { CoreState::Running } else { CoreState::Reset }
    }
}

impl esp_soc::SocBus for SocBus {
    fn cycles(&self) -> u64 { self.cycles }
    fn next_deadline(&self) -> Option<u64> {
        let c = self.periph.cycles_until_timer();
        if self.periph.sdmmc.timed { return self.timed_sd_deadline(c); }
        match c { u32::MAX => None, c => Some(c.max(1) as u64) }
    }
    fn irq_dirty(&mut self) -> &mut bool { &mut self.irq_dirty }
    fn refresh_irq(&mut self) -> bool { self.periph.refresh_lines(); true }
    fn misc(&mut self) -> &mut Misc { &mut self.periph.misc }
    fn load_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), String> { SocBus::load_bytes(self, addr, data) }
    fn write_flash(&mut self, offset: usize, data: &[u8]) -> Result<(), String> { SocBus::write_flash(self, offset, data) }
    /// Copy the RAM segments and map the flash-resident ones through the flash MMU, as the
    /// 2nd-stage bootloader would. The clocks and caches it would have set up are not.
    fn boot_app(&mut self, app_off: usize) -> Result<u32, String> {
        let image = self.flash.get(app_off..).ok_or("app offset beyond flash")?;
        let img = esp_soc::image::parse(image)?;
        for s in &img.segments {
            let start = app_off + s.file_off as usize;
            let end = start + s.len as usize;
            if end > self.flash.len() { return Err("segment beyond flash".into()); }
            if (FLASH_LOW..FLASH_HIGH).contains(&s.load_addr) {
                let mask = (1u32 << MMU_PAGE_SHIFT) - 1;
                if (s.load_addr & mask) != (start as u32 & mask) { return Err(format!("segment {:#x} not page-aligned with flash offset {:#x}", s.load_addr, start)); }
                if !s.load_addr.checked_add(s.len).is_some_and(|e| e <= FLASH_HIGH) { return Err("segment beyond the flash window".into()); }
                let first_page = (start as u32) >> MMU_PAGE_SHIFT;
                let npages = ((s.load_addr & mask) + s.len + mask) >> MMU_PAGE_SHIFT;
                for i in 0..npages {
                    let idx = (((s.load_addr - FLASH_LOW) >> MMU_PAGE_SHIFT) + i) as usize;
                    if idx < MMU_ENTRIES { self.mmu_flash[idx] = MMU_FLASH_VALID | (first_page + i); }
                }
            } else {
                let data = self.flash[start..end].to_vec();
                SocBus::load_bytes(self, s.load_addr, &data)?;
            }
        }
        Ok(img.entry)
    }
    /// Digital peripherals re-created; L2MEM, TCM, the LP domain and the efuses kept.
    fn reboot(&mut self, mac: [u8; 6]) -> u32 {
        let cause = self.periph.lp.reset_cause;
        let old = std::mem::replace(&mut self.periph, periph::Peripherals::new(mac, self.wafer));
        let p = &mut self.periph;
        p.cycle_base = self.cycles;
        p.efuse = old.efuse;
        p.misc.log_unknown = old.misc.log_unknown;
        p.usb.connected = old.usb.connected;
        p.lp = old.lp;
        p.lp.sw_reset = false;
        p.lp.reset_cause = cause;
        p.assist_debug.saved_pc = old.misc.cur_pc;
        p.spi0.0.jedec = old.spi0.0.jedec;
        p.spi1.jedec = old.spi1.jedec;
        p.psram1.psram_mr = old.psram1.psram_mr;      // the PSRAM keeps its mode registers
        p.gpio.g.strap = old.gpio.g.strap;
        p.tsens.celsius = old.tsens.celsius;
        // The card in the socket keeps its power and its state; IDF's init resets it with CMD0.
        p.sdmmc.devices = old.sdmmc.devices;
        p.sdmmc.stats = old.sdmmc.stats;
        p.sdmmc.timed = old.sdmmc.timed;
        p.sd_pins = old.sd_pins;
        p.has_sdio = old.has_sdio;
        if p.has_sdio { p.sync_sdio(); }
        // The USB cable stays plugged in; the host sees the reset core's pull-up go.
        if let Some(link) = old.usb_otg.link { p.usb_otg.stats = old.usb_otg.stats; p.usb_otg.reattach(old.usb_otg.now, link); }
        esp_periph::Dispatch::refresh_optional(p, periph::USB_OTG_BLOCK);
        *self.mmu_flash = [0; MMU_ENTRIES];
        *self.mmu_psram = [0; MMU_ENTRIES];
        self.mmu_flash_index = 0;
        self.mmu_psram_index = 0;
        self.reservation = [None; 2];
        for area in self.debug.iter() { esp_periph::Dispatch::debug(&mut self.periph, area, true); }
        self.periph.misc.log_all = self.debug.has("mmio");
        cause
    }
    fn sw_reset(&self) -> bool { self.periph.lp.sw_reset }
    fn request_reset(&mut self, cause: u32) { self.periph.lp.sw_reset = true; self.periph.lp.reset_cause = cause; }
    fn reset_cause(&self) -> u32 { self.periph.lp.reset_cause }
    fn last_fault(&self) -> Option<(u32, bool)> { self.last_fault }
    fn console_take(&mut self) -> [Vec<u8>; 4] { [std::mem::take(&mut self.periph.usb.tx_out), std::mem::take(&mut self.periph.uart[0].tx_out), std::mem::take(&mut self.periph.uart[1].tx_out), Vec::new()] }
    fn serial_input(&mut self, data: &[u8]) {
        let before = self.periph.usb.irq();
        self.periph.usb.host_input(data);
        self.irq_dirty |= before != self.periph.usb.irq();
    }
    fn uart_input(&mut self, n: usize, data: &[u8]) {
        let Some(u) = self.periph.uart.get_mut(n) else { return };
        let before = u.irq();
        u.host_input(data);
        self.irq_dirty |= before != u.irq();
    }
    fn gpio_set_input(&mut self, pin: u8, level: bool) {
        self.irq_dirty |= self.periph.gpio.g.set_input(pin, level);
        if let Some(ev) = &mut self.gpio_events { ev.push((self.cycles, pin, level)); }
    }
    fn set_flash_size(&mut self, bytes: usize) {
        self.flash = vec![0xff; bytes];
        let cap = bytes.trailing_zeros() as u8; self.periph.spi1.jedec[2] = cap; self.periph.spi0.0.jedec[2] = cap;
    }
    fn set_psram_size(&mut self, bytes: usize) -> Result<(), String> { self.psram = vec![0; bytes]; Ok(()) }
    fn set_strap(&mut self, v: u32) { self.periph.gpio.g.strap = v; }
    fn set_reset_cause(&mut self, c: u32) { self.periph.lp.reset_cause = c; }
    fn set_debug(&mut self, f: &esp_soc::DebugFlags) {
        self.debug = f.clone();
        for area in f.iter() { esp_periph::Dispatch::debug(&mut self.periph, area, true); }
        self.periph.misc.log_all = f.has("mmio");
    }
    fn observe_gpio(&mut self, on: bool) { self.gpio_events = if on { Some(Vec::new()) } else { None }; }
    fn take_gpio_events(&mut self) -> Vec<(u64, u8, bool)> { self.gpio_events.as_mut().map(std::mem::take).unwrap_or_default() }
    fn gpio_input(&self) -> u64 { self.periph.gpio.g.input }
    fn board(&mut self) -> &mut dyn BoardModel { &mut *self.board }
    fn board_ref(&self) -> &dyn BoardModel { &*self.board }
    fn audio(&self) -> (&[i16], u32) { (&[], 44100) }
    /// `line` is the CPU interrupt line, CLIC ID - 16.
    fn irq_sources_of(&self, core: usize, line: u32) -> Vec<usize> { self.periph.sources_of(core, line + CLIC_EXT_OFFSET) }
    /// The card goes in the reference board's socket, on slot 0's IO MUX pins; firmware reaches
    /// it through slot 0 or, routing those pins through the GPIO matrix, slot 1.
    fn attach_sd_card(&mut self, image: Box<dyn esp_periph::sdmmc::SdStore>, config: esp_periph::sdmmc::SdCardConfig) -> Result<String, String> {
        let timed = config.timed_bus;
        let card = esp_periph::sdmmc::SdCard::with_config(image, config)?;
        let what = format!("{}, {} blocks ({}), serial {:08x}, programming {} us, at CLK {} CMD {} D0-D3 {:?}", card.describe(), card.blocks(), if card.sdxc() { "SDXC" } else { "SDHC" },
                           card.serial(), card.config.prg_ns / 1000, periph::SdPins::SOCKET.clk, periph::SdPins::SOCKET.cmd, periph::SdPins::SOCKET.data);
        self.periph.sdmmc.timed = timed;
        self.periph.attach_sd(periph::SdPins::SOCKET, Box::new(card));
        Ok(format!("{what}{}", if timed { ", bus timed" } else { "" }))
    }
    fn report(&self) -> String {
        let s = &self.periph.sdmmc.stats;
        let mut out = Vec::new();
        if !self.periph.sdmmc.devices.is_empty() || s.commands > 0 {
            out.push(format!("[sdmmc] {} commands ({} unanswered), {} bytes read, {} written", s.commands, s.timeouts, s.bytes_read, s.bytes_written));
        }
        if let Some(l) = &self.periph.usb_otg.link {
            let (d, k) = (&self.periph.usb_otg.stats, &l.stats);
            out.push(format!("[usb] {} SETUPs, {} IN packets ({} bytes), {} OUT packets ({} bytes), {} NAKs, {} STALLs; host: {} transfers, {} bus resets, {} data PID errors; core: {} DMA faults, {} FIFO size errors",
                             d.setups, d.packets_in, d.bytes_in, d.packets_out, d.bytes_out, d.naks, d.stalls, k.transfers, k.resets, k.toggle_errors, d.dma_faults, d.fifo_errors));
        }
        out.join("\n")
    }
    /// The host on the other end of the high-speed USB-OTG port (rhport 1, `DR_REG_USB2_BASE`).
    fn attach_usb_host(&mut self, timing: esp_periph::usb_otg::UsbTiming) -> Result<(), String> {
        let now = self.cycles;
        let mut link = esp_periph::usb_otg::UsbLink::new(timing, periph::CPU_HZ, now);
        link.log = self.periph.usb_otg.logging();
        self.periph.usb_otg.attach(now, link);
        esp_periph::Dispatch::refresh_optional(&mut self.periph, periph::USB_OTG_BLOCK);
        self.irq_dirty = true;
        Ok(())
    }
    fn usb_link(&mut self) -> Option<&mut esp_periph::usb_otg::UsbLink> { self.periph.usb_otg.link.as_deref_mut() }
}
