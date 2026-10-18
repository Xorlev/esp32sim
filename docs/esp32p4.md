# ESP32-P4 (RISC-V) — draft

The first chip here with two RISC-V cores: silicon **rev < 3** (v0.x, v1.x), as on the Waveshare
ESP32-P4-WIFI6 (v1.0).
Unmodified ESP-IDF firmware built for rev < 3 boots from the real mask ROM, through the real
2nd-stage bootloader, into FreeRTOS on one or both cores; an image built for rev ≥ 3 is refused
("Revisions" below). This page describes the chip and its peripherals.

```sh
cargo build --release
FW=web/wasm/fw/public
./target/release/esp32sim-p4 --flash-mb 2 --bootloader $FW/p4-hello-bootloader.bin \
    --ptable $FW/p4-hello-ptable.bin --app $FW/p4-hello_world.bin --max-seconds 27
```

prints the ROM banner, the bootloader's partition table, the IDF startup log, `Hello world!`, the
countdown, and reboots through the ROM on `esp_restart()` (goldens `hello-p4`, `hello-p4-reboot`):

```
ESP-ROM:esp32p4-20230811
Build:Aug 11 2023
rst:0x1 (POWERON),boot:0x30f (SPI_FAST_FLASH_BOOT)
I (6) boot: ESP-IDF v5.5.3 2nd stage bootloader
I (7) boot: chip revision: v1.0
I (7) boot: efuse block revision: v0.3
I (35) cpu_start: cpu freq: 360000000 Hz
I (36) main_task: Calling app_main()
Hello world!
This is esp32p4 chip with 2 CPU core(s), , silicon revision v1.0, 2MB external flash
Restarting now.
rst:0xc (SW_CPU_RESET),boot:0x30f (SPI_FAST_FLASH_BOOT)
Core0 Saved PC:0x4ff0232a
```

The images are IDF v5.5.3's hello_world for rev < 3 (`examples/hello_world-p4`): `p4-hello-*`
single-core, which the browser build runs too (`web/wasm/fw/p4-hello.json`), and `p4-hello-smp*`
both cores (golden `hello-p4-smp`, where both cores take their own systimer tick and cross-core
interrupts), which `tools/fetch-p4-firmware.sh` pins by SHA-256. `esp32sim --chip p4` is the same
binary; the boot mode defaults to `rom` and the console to UART0. The mask ROM ELF is
`esp32p4_rev0_rom.elf` (ECO0, from ESP-IDF's esp-rom-elfs or `tools/fetch-rom-elfs.sh`). No ROM is
published for the ECO2 silicon on v1.x boards. ECO0's jump table matches every entry of IDF
v5.5.3's rev < 3 `esp32p4.rom.ld`, so images built for v1.x link against it; the first two
console lines are ECO0's, not the board's, and so is the length of the heap region below the
ROM's own data, which IDF takes from the ROM's `ets_rom_layout` (`_dram0_rtos_reserved_start` is
0x4FF3FBA4 in the ECO0 ELF).

## The core

`Cpu::new_esp32p4()` is RV32IMAFC with the P4's EXT_ILL CSR, the Zicntr counters and CLIC mode;
the P4 builds core *i* with `hartid` *i*. Nothing of it is enabled on the C3 and C6 cores.

- **Counters.** `cycle`, `instret` and their high halves (0xC00/0xC02/0xC80/0xC82) read the
  `mcycle`/`minstret` counters. IDF on the P4 reads the cycle count with `rdcycle`
  (riscv/include/riscv/rv_utils.h); the bootloader's random-number loop waits on it.
- **CLIC mode** (RISC-V CLIC specification, "CLIC Interrupt Operation"; IDF v5.5.3
  riscv/vectors_clic.S, vectors.S). With `mtvec` in mode 3 the core takes the candidate the SoC
  presents (CPU line, 8-bit level, SHV) when its level is strictly above `mintstatus.mil` and
  `mstatus.MIE` is set. The SoC has applied the rev < 3 threshold, which is the memory-mapped
  `CLIC_INT_THRESH[31:24]`. On a take, `mcause` is 0x8000_0000 | MPIL (the previous level) << 16 |
  ID, with MPP and MPIE read through from `mstatus`; the level becomes the interrupt's. A
  hardware-vectored interrupt jumps to the word at `mtvt + 4*id` (CSR 0x307), a bus read made
  with MINHV set. Non-vectored interrupts and every exception enter at the 64-byte-aligned
  `mtvec` base, an exception recording the current level in MPIL. `mret` restores the level
  from MPIL. `mintstatus` is CSR 0x346 on rev < 3. Any presented candidate ends a WFI, takeable
  or not (a WFI may resume for any reason).
- **CLINT mode.** In `mtvec` modes 0 and 1 the core takes no CLIC candidate and does not count one
  as pending, so it idles: interrupts there would be `mie`/`mip`'s, nothing raises `mip` on this
  chip, and the ROM runs in mode 1 with `mie` = 0. `mcause` reads as written, without the
  aliases, and the ROM takes its exceptions at its base.
- **Not modelled:** a fault on the vector-table read is an instruction access fault at the
  table entry with MINHV left set, but `mret` does not resume the fetch as the specification has
  it, and MPIE/MPIL then describe the half-entered handler. IDF's table is in mapped RAM.
- **Harts.** `mhartid` reads the core's index and survives a reset. An LR reservation is checked
  with the bus at the SC (`Bus::reserve`, `reservation_held`): the P4 bus keeps one per hart, and
  any store by the other hart to the reserved word (word, half, byte, AMO, SC) breaks it. A trap
  breaks the core's own reservation, so an SC after a handler cannot overwrite the handler's
  store (IDF's vectors issue no SC of their own). The machine tells the bus which core it is
  about to run (`Soc::select_core`), which is how each core sees its own CLIC block at
  0x20800000 and the other's at 0x20810000.

The CLIC and the interrupt matrices are registers (`periph::Clic`, `periph::IntMatrix`): sources
reach each core's CLIC through its matrix; IE, the attributes and the levels read back, the control
words take byte writes as the four byte registers they are, and an edge-triggered ID clears when it
is taken hardware-vectored (`Soc::interrupt_taken`) or when software writes 1 to IP and nothing
else: IDF acknowledges with a 32-bit `REG_SET_BIT` of IP, and its SHV/TRIG updates are
read-modify-writes of the same word that write a pending IP back as 1, which must not consume the
edge.

The core's P4 paths cost the other chips nothing: each is behind a flag only
`Cpu::new_esp32p4` sets (`misa.F`, `has_ext_ill`, `user_counters`, `clic`), the reservation hooks
are `Bus` defaults that a single-hart bus inlines away, and the machine's `select_core` and
`interrupt_taken` are empty `Soc` defaults. The hello demos keep their instruction counts.

## Revisions

The model is rev < 3, with the efuses reporting v1.0. ESP-IDF v5.5.x builds for rev ≥ 3 unless
`CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` (it defaults to n, IDF v5.5.3
`components/esp_hw_support/port/esp32p4/Kconfig.hw_support`), and the two families differ in
ISA, interrupt controller and ROM (the table at the end of this section). So a stock
`idf.py set-target esp32p4 build` does not run here. Its bootloader and app record
`min_chip_rev_full` 300 or more in their image header (`esp_image_header_t`, IDF v5.5.3
`components/bootloader_support/include/esp_app_format.h`), and the emulator stops before the first
instruction:

```
[emu] build/hello_world.bin was built for ESP32-P4 rev ≥ v3.0 (min_chip_rev_full=300); this model is rev < 3 (v1.0). Rebuild with CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y, or wait for rev 3 support.
```

The examples set `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` and `CONFIG_ESP32P4_REV_MIN_100=y`, which
records 100 to 199. The check (`Soc::check_image`) runs on every bootloader and app the CLI loads
(`--bootloader`, `--app`, `--flash-at`, and those inside `--flash-image`) and on the browser
build's loads, and only on headers whose chip ID is the P4's. Rev 3 becomes a second
`esp32p4::Revision`: the ROM `esp32p4_rev300_rom.elf` is in the esp-rom-elfs release
`tools/fetch-rom-elfs.sh` already pins, and its `refuse` accepts these images and refuses rev < 3
builds instead.

| | rev < 3 (v0.x, v1.x) | rev ≥ 3 |
| --- | --- | --- |
| `-march` | `rv32imafc_zicsr_zifencei_xesppie` | `rv32imafc_zicsr_zifencei_xesploop_xespv` |
| CLIC threshold | memory-mapped, `0x20800008` bits 31:24 | `mintthresh` CSR (0x347) |
| `mintstatus` | CSR 0x346 | CSR 0xFB1 |
| L2 cache | top of L2MEM | bottom of L2MEM |
| ROM published in esp-rom-elfs | `esp32p4_rev0_rom.elf` (ECO0) | `esp32p4_rev300` |

## Compared with silicon

Nothing on this page was compared with P4 silicon directly; the ECO0 ROM and IDF's own sources
are what it is checked against.

## What the bring-up found

- **The ROM waits for the flash controller's AXI FIFOs to report empty**, in SPI_MEM_C's
  SMEM_AXI_ADDR_CTRL (0x178), not at 0x170 as on the C6; IDF waits on the same bits in the PSRAM
  controller.
- **The ROM checks the bootloader's appended SHA-256** with the SHA accelerator before it jumps;
  without one it prints `SHA-256 comparison failed` and boots anyway.
- **The cache controller's size registers must hold their defaults**: IDF derives cache sizes
  from them, so the block is seeded from `cache_reg.h` and every operation reports done at once.
- **`rdcycle` is the P4's cycle count.** The C3 and C6 read it from MPCCR; without `cycle` the
  bootloader's random-number loop never ends.
- **A CLIC control word is four registers.** The ROM and IDF write IE, the attributes and the
  level one byte at a time; a read-modify-write of the whole word would write IP back.

## What is modelled

| | |
| --- | --- |
| CPU | two RV32IMAFC cores (`Cpu::new_esp32p4`, `misa` 0x40001125): F with the lazy FPU traps, EXT_ILL, Zicntr, CLIC mode, `mhartid`; core 1 held in reset until HP_SYS_CLKRST releases it. Xesppie and the hardware loops decode as illegal |
| Memory | 128 KB mask ROM at 0x4FC00000 (reset vector), 768 KB L2MEM at 0x4FF00000 and its uncached alias at 0x8FF00000, 8 KB TCM at 0x30100000, LP ROM (empty) and 32 KB LP RAM; flash (0x40000000) and PSRAM (0x48000000) windows behind two 1024-entry MMUs of 64 KB pages, reached through SPI_MEM_C / SPI_MEM_S item registers, with their uncached aliases |
| Interrupts | both interrupt matrices (128 sources, rev < 3) and both CLIC register blocks, delivered through the cores' CLIC mode |
| Peripherals | UART0/1, USB-Serial/JTAG, systimer (16 MHz from XTAL), TIMG0/1, SHA, efuse (v1.0, block v0.3, security efuses zero), the flash controller (SPI0 cache side, SPI1 commands, a GigaDevice part answering RDID), the PSRAM controller with an AP Memory x16 PSRAM, the cache controller, HP_SYSTEM (cross-core interrupts, stall status), HP_SYS_CLKRST (divider update, PLL calibration done, core 1 clock and reset), LP_SYSTEM / LP_CLKRST / LP_TIMER (reset cause, software reset, core 1 boot address, RTC counter), the analog I2C master (regi2c read-back), ASSIST_DEBUG's saved PC, the RNG; everything else is register RAM, listed by `--log-periph` |
| Time base | 360 MHz, the rev < 3 application clock; one instruction is one cycle |

The revision is a type parameter (`esp32p4::Revision`, `RevLt3`): rev ≥ 3 (its own ROM, the
`mintthresh` CSR, the L2 cache at the bottom of L2MEM) is a second implementation, not a change
to this one, and an image built for it is refused until then ("Revisions"). The browser build
boots the same images (`web/wasm/fw/p4-hello.json`, run by `tools/wasm-test.mjs p4-hello`); its
firmware dialog offers the board `esp32p4`.

## Not there yet

- **SDMMC, USB-OTG, ESP-Hosted**: the SD card, the USB port and the C6 co-processor.
- **AXI-GDMA, the LP core, MIPI-DSI/CSI, DMA2D/PPA, rev ≥ 3**, and Xesppie's save and restore,
  which FreeRTOS runs only for a task that uses PIE.
- **GPIO** uses the shared model at the C6's register offsets; the P4's FUNCn_IN_SEL and
  FUNCn_OUT_SEL arrays sit 4 bytes higher and it has 55 pins. Configuration reads back, so the
  console is unaffected; pin-level boards need the layout parameter.
- **`--boot app`** maps the image and jumps to it without the clocks and caches the bootloader
  would have set.
- **`Machine::run_until_cycle`** still schedules core 0 alone; nothing that drives the P4 needs it.
- **Unmodelled blocks** hello_world touches: PMU, PVT, APB_SARADC, LP_WDT, LP_ANA_PERI, LP_ADC,
  KEYMNG, UART2-4, TRACE0/1, SDMMC, LP_UART, LP_PERI, LP_GPIO and DSI, all configuration the
  firmware writes and reads back (`--log-periph` lists them). The watchdogs never fire.
