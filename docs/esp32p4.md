# ESP32-P4 (RISC-V) — draft

The fourth chip, and the first with two RISC-V cores: silicon **rev < 3** (v0.x, v1.x), as on the
Waveshare ESP32-P4-WIFI6 (v1.0) the plan targets ([esp32p4-plan.md](esp32p4-plan.md), milestone 1).
The real mask ROM boots the real 2nd-stage bootloader, which reads the partition table; how far
the application gets is set by what the RISC-V core model provides, not by the chip model.

```sh
cargo build --release
FW=web/wasm/fw/public
./target/release/esp32sim-p4 --flash-mb 2 --bootloader $FW/p4-hello-bootloader.bin \
    --ptable $FW/p4-hello-ptable.bin --app $FW/p4-hello_world.bin --max-seconds 0.05
```

prints the ROM banner and the bootloader's log up to the first application segment, then stops
with a report on stderr (golden `hello-p4`):

```
ESP-ROM:esp32p4-20230811
Build:Aug 11 2023
rst:0x1 (POWERON),boot:0x30f (SPI_FAST_FLASH_BOOT)
I (0) boot: ESP-IDF v5.5.3 2nd stage bootloader
I (0) boot: chip revision: v1.0
I (0) boot: efuse block revision: v0.3
I (0) boot: End of partition table
I (0) esp_image: segment 0: paddr=00010020 vaddr=40020020 size=07114h ( 28948) map
[esp32p4] RNG_DATA read 65536 times with nothing else in between at pc 0x4ff2dd24: bootloader_fill_random waits for `rdcycle` (CSR 0xC00) to advance, and the core model returns 0 for it
```

The images are IDF v5.5.3's hello_world for rev < 3, single-core (`examples/hello_world-p4`).
`esp32sim --chip p4` is the same binary; the boot mode defaults to `rom` and the console to UART0.
The mask ROM ELF is `esp32p4_rev0_rom.elf` (ECO0, from ESP-IDF's esp-rom-elfs or
`tools/fetch-rom-elfs.sh`). No ROM is published for the ECO2 silicon on v1.x boards. ECO0's jump
table matches every entry of IDF v5.5.3's rev < 3 `esp32p4.rom.ld`, so images built for v1.x link
against it; the first two console lines are ECO0's, not the board's.

## Where the core stops it

| | stops at | what the core must provide |
| --- | --- | --- |
| hello_world, plain run | the bootloader's `bootloader_fill_random` (`process_segments`, before the first segment is loaded), pc 0x4ff2dd20 / 0x4ff2dd2c | `cycle` / `cycleh` (CSR 0xC00 / 0xC80, also `instret` 0xC02) reading the `mcycle` / `minstret` counters; IDF on the P4 reads the cycle count with `rdcycle` and the loop waits for 1440 cycles between two reads |
| hello_world, with a stand-in for that CSR (`esp32p4/tests/hello.rs`, golden `hello-p4-cycle-shim`) | the scheduler start: `vTaskStartScheduler` returns and `start_cpu0` spins on its final `j .` (0x40000682) | CLIC mode: with `mtvec` = 0x4ff00003 (mode 3) and `mtvt` set, the first interrupts are CLIC ID 18 (SYSTIMER_TARGET0, CPU line 2) and ID 17 (FROM_CPU_INTR0, line 1, the first yield), both hardware-vectored, level 0x3f over threshold 0x1f. The core must take the highest level above `max(mintstatus.mil, CLIC_INT_THRESH[31:24])` with `mstatus.MIE` set (the higher ID on equal levels, as the CLIC specification has it; not checked on silicon), fetch the handler from `mtvt + 4 * id`, write `mcause` = 0x8000_0000 \| id with the previous level in bits 23:16 (MPIL), and `mret` must restore the level from `mcause.MPIL`; `mintstatus` is CSR 0x346 on rev < 3 |
| dual-core hello_world and oxidrive, with the stand-in | `start_other_core`, waiting for core 1 (`ets_delay_us`) | a `mhartid` per hart (0 and 1, kept across a reset); HP_SYS_CLKRST releases core 1 at boot address 0x4ff003a6 (hello_world) and the chip holds it, because a second hart reading `mhartid` 0 would run the ROM as core 0 |

The cores are built with `Cpu::new_esp32p4()`: RV32IMAFC, `mstatus.FS` Off at reset with the lazy
FPU traps, and the EXT_ILL CSR. None of the paths above executes an F instruction (the run to the
scheduler takes no exception); the only ones in hello_world are FreeRTOS's lazy save in
`rtos_save_fpu_coproc`, reached from the illegal-instruction trap once tasks run.

The CLIC and the interrupt matrices are modelled as registers (`periph::Clic`, `periph::IntMatrix`):
sources reach the per-core CLIC input through the matrix, IE, attributes, levels and the rev < 3
threshold read back, and `ClicCore::candidate` says which ID a core would take. Nothing is
presented to the core (`Soc::irqs` is empty); the first time an interrupt would be taken the bus
reports it on stderr.

## Compared with a board

The owner's oxidrive firmware (not in this repository) was booted from the same bootloader,
partition table and app as a board whose console was captured, with the bootloader's flash mode
set to QIO as espflash wrote it, 32 MB of flash and 32 MB of PSRAM, and the cycle-CSR stand-in.
With timestamps stripped, the emulator's console matches the board's for 25 lines: the reset
line, `SPI mode:QIO, clock div:1`, the three segment loads, the twelve `hex_psram` mode-register
lines, the MSPI timing tuning, `Found 32MB PSRAM device`, `Speed: 200MHz` and
`cpu_start: Multicore app`. The banner (lines 1 and 2) differs by construction. The board's next
line, `cpu_start: GPIO 38 and 37 are used as console UART I/O pins`, needs core 1.
`external_oxidrive_console_matches_the_board` (`OXIDRIVE_DIR` names the images and the capture)
reports this comparison; it is not expected to pass yet.

## What the bring-up found

- **The ROM waits for the flash controller's AXI FIFOs to report empty**, in SPI_MEM_C's
  SMEM_AXI_ADDR_CTRL (0x178), not at 0x170 as on the C6; IDF waits on the same bits in the PSRAM
  controller.
- **The ROM checks the bootloader's appended SHA-256** with the SHA accelerator before it jumps;
  without one it prints `SHA-256 comparison failed` and boots anyway.
- **The cache controller's size registers must hold their defaults**: IDF derives cache sizes
  from them, so the block is seeded from `cache_reg.h` and every operation reports done at once.
- **The bootloader's random-number loop is a cycle-count wait**, which is how the missing
  `cycle` CSR shows: it reads WDEV_RND_REG until `rdcycle` has advanced.

## What is modelled

| | |
| --- | --- |
| CPU | two RV32IMAFC cores (`Cpu::new_esp32p4`, `misa` 0x40001125, the EXT_ILL CSR); core 1 held in reset (see above). Xesppie and the hardware loops decode as illegal |
| Memory | 128 KB mask ROM at 0x4FC00000 (reset vector), 768 KB L2MEM at 0x4FF00000 and its uncached alias at 0x8FF00000, 8 KB TCM at 0x30100000, LP ROM (empty) and 32 KB LP RAM; flash (0x40000000) and PSRAM (0x48000000) windows behind two 1024-entry MMUs of 64 KB pages, reached through SPI_MEM_C / SPI_MEM_S item registers, with their uncached aliases |
| Interrupts | both interrupt matrices (128 sources, rev < 3) and both CLIC register blocks; not delivered (above) |
| Peripherals | UART0/1, USB-Serial/JTAG, systimer (16 MHz from XTAL), TIMG0/1, SHA, efuse (v1.0, block v0.3, security efuses zero), the flash controller (SPI0 cache side, SPI1 commands, a GigaDevice part answering RDID), the PSRAM controller with an AP Memory x16 PSRAM, the cache controller, HP_SYSTEM (cross-core interrupts, stall status), HP_SYS_CLKRST (divider update, PLL calibration done, core 1 clock and reset), LP_SYSTEM / LP_CLKRST / LP_TIMER (reset cause, software reset, core 1 boot address, RTC counter), the analog I2C master (regi2c read-back), ASSIST_DEBUG's saved PC, the RNG; everything else is register RAM, listed by `--log-periph` |
| Time base | 360 MHz, the rev < 3 application clock; one instruction is one cycle |

The revision is a type parameter (`esp32p4::Revision`, `RevLt3`): rev >= 3 (its own ROM, the
`mintthresh` CSR, the L2 cache at the bottom of L2MEM) is a second implementation, not a change
to this one. The browser build boots the same images (`web/wasm/fw/p4-hello.json`, run by
`tools/wasm-test.mjs p4-hello`).

## Not there yet

- The core work above: `cycle`, the CLIC trap path, a second hart. Then Xesppie's save and
  restore, which FreeRTOS runs only for a task that uses PIE.
- **GPIO** uses the shared model at the C6's register offsets; the P4's FUNCn_IN_SEL and
  FUNCn_OUT_SEL arrays sit 4 bytes higher and it has 55 pins. Configuration reads back, so the
  console is unaffected; pin-level boards need the layout parameter.
- **`--boot app`** maps the image and jumps to it without the clocks and caches the bootloader
  would have set.
- **Unmodelled blocks** hello_world touches on its way to the scheduler: PMU, PVT, APB_SARADC,
  LP_WDT, LP_ANA_PERI, LP_ADC, KEYMNG, UART2-4, TRACE0/1, SDMMC, LP_UART, LP_PERI, LP_GPIO and
  DSI, all configuration the firmware writes and reads back (`--log-periph` lists them). The
  watchdogs never fire.
- SDMMC, USB-OTG, ESP-Hosted, AXI-DMA, the LP core: milestones 2 to 4 of the plan.
