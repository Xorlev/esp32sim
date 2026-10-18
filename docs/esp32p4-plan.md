# ESP32-P4 — plan

Status: milestones 1 and 2 done, milestone 3 (USB) next. hello_world boots to `app_main` on one
and on both cores and reboots through the ROM; the core has F with the lazy FPU traps, the Zicntr
counters, CLIC mode and two harts. The SDMMC host and an SD card carry IDF's sdmmc example through
a FAT32 mount ([esp32p4.md](esp32p4.md)). Tracks
[#215](https://github.com/mikroverk/esp32sim/issues/215).

The P4 is a dual-core RISC-V SoC with no radio. This plan brings it up the way the C6 came up:
the real mask ROM, the real 2nd-stage bootloader and an unmodified ESP-IDF image, checked
against a real board's console. It is staged so each milestone ends in a test that passes on
its own. The later milestones follow what a board built around the P4's peripherals asks for: an
application that serves an SD card to a car over USB and reaches the network through the C6
co-processor needs SDMMC, USB-OTG and ESP-Hosted, none of which any chip here modelled before.

Reference board: Waveshare ESP32-P4-WIFI6, silicon **v1.0** (rev < 3), 32 MB flash, 32 MB PSRAM,
an ESP32-C6 on SDIO.

## Revisions

The two silicon families differ in ISA, interrupt controller and ROM, so the revision is a
property of the chip, not a detail.

| | rev < 3 (v0.x, v1.x) | rev ≥ 3 |
| --- | --- | --- |
| `-march` | `rv32imafc_zicsr_zifencei_xesppie` | `rv32imafc_zicsr_zifencei_xesploop_xespv` |
| CLIC threshold | memory-mapped, `0x20800008` bits 31:24 | `mintthresh` CSR (0x347) |
| `mintstatus` | CSR 0x346 | CSR 0xFB1 |
| L2 cache | top of L2MEM | bottom of L2MEM |
| ROM published in esp-rom-elfs | `esp32p4_rev0_rom.elf` (ECO0) | `esp32p4_rev300` |

rev < 3 comes first, because that is the silicon on the reference board. The board prints
`ESP-ROM:esp32p4-eco2-20240710`; no ECO2 ROM is published. The ECO0 ELF's jump table at
`0x4FC00018–0x4FC00A4C` matches every address in ESP-IDF's rev < 3 `esp32p4.rom.ld` (459 of
459), so IDF images link correctly against it; the code behind the stubs is ECO0's. The first
two console lines therefore differ from the board until a ROM dumped from it is used locally
(`--rom`, never committed). rev ≥ 3 is a second chip variant once rev < 3 boots.

## Milestones

### 1. Boot

ROM → bootloader → FreeRTOS on both HP cores → `app_main`.

The CPU work is the bulk of it:

- **F extension**, including the compressed FP loads and stores, which the ROM itself uses
  (`setjmp`, `__mulsc3`). Gated on `misa`, so the C3 and C6 decode exactly as today. Done:
  a soft-float on integers (`riscv-rv32/src/softfloat.rs`), checked against the host and a
  round-to-odd reference in every rounding mode.
- **Lazy FPU traps.** IDF switches FP context lazily: an FP instruction with `mstatus.FS` = Off
  must raise an illegal instruction with `mtval` = the instruction and the reason in CSR 0x7F0.
  Executing it instead does not crash; FreeRTOS silently stops saving FP registers. Done, with
  0x7F0 bit 0 set for the computational instructions only: the P4 does not flag FLW/FSW
  (`SOC_CPU_HAS_FPU_EXT_ILL_BUG`, ESP-IDF v5.5.3 `soc_caps.h`), and IDF decodes `mtval` for
  those and for the FP CSR accesses.
- **CLIC**, per core at `0x20800000` (the other core's at `+0x10000`): 48 IDs, IDF's CPU lines
  0–31 at IDs 16–47, 3 priority bits, hardware vectoring through `mtvt` (needs a bus read on
  trap entry), `mcause` carrying the previous level, `mret` restoring it. The ROM uses the
  older `mtvec` mode 1; both must work.
- **Two harts**: `mhartid` that survives `reset()`, cross-hart LR/SC, core 1 held in reset
  until the ROM releases it. A multi-core `run_until_cycle` is left until something that drives
  the P4 needs it.
- Xesppie, decoded and executed only as far as FreeRTOS's PIE save and restore needs (32
  instructions), is not done yet: FreeRTOS runs that code only for a task that uses PIE. The rest
  of the SIMD set is for esp-dsp and is out of scope here.

The chip crate `esp32p4/` follows `esp32c6/`: the memory map (ROM, TCM, L2MEM with its uncached
alias, flash and PSRAM through two MMUs), the interrupt matrices, HP_SYS/LP_SYS/PMU, the cache
controller, MSPI, efuse, and the shared UART, USB-Serial/JTAG, systimer and TIMG models at the
P4's addresses. The registers the boot polls (cache sync, PLL calibration, clock-divider
update, analog I2C, RTC calibration, the core 1 stall and boot-address words) cite their
headers where they are modelled.

Done when:

- the ESP-IDF `hello_world` for esp32p4 rev < 3 reaches `app_main` with both cores scheduled, as
  a golden test;
- an application built for the reference board boots to its first SD card access with its
  console matching a v1.0 board's capture of the same binaries line for line once timestamps
  are stripped, but for the lines the ECO0 ROM changes ([esp32p4.md](esp32p4.md), "Compared
  with a board"; the application and the capture are not committed);
- every existing golden and instruction count is unchanged.

### 2. SD card

The SDMMC host (the DesignWare IP the ESP32 and S3 also have, plus Espressif's wrapper registers
from `0x800`) and an SD card model behind it, backed by an image file: CMD0/8/55/41/2/3/9/7/6
initialisation, 4-bit at 40 MHz, single- and multi-block reads and writes through the IDMAC.
Slot 1 of the same host carries ESP-Hosted in milestone 4, so the host is written for two slots.

Done when IDF's sdmmc example mounts a FAT32 image at 4 bits, writes, renames and reads back a
file, and long transfers refill the driver's DMA descriptors. Done: the host in
`esp-periph/src/sdmmc.rs`, the card in `sdmmc/card.rs`, `--sd`/`--sd-rw`, goldens `sdcard-p4`
and `sdcard-sectors-p4` (esp32p4.md, "SD card").

### 3. USB device and the car

The HS USB-OTG controller (DWC2 with a UTMI PHY, at `0x50000000`) as a device, and a host on
the other end of the cable written in Rust: enumeration, then SCSI over bulk-only transport
(INQUIRY, TEST UNIT READY, READ CAPACITY, READ/WRITE(10)). The host is scripted, so a test can
record like a car (clip-sized writes, FAT updates, idle gaps), hold the drive, and see what the
device reports while it has the card away (MEDIUM NOT PRESENT, which the tusb_msc example shows
after an eject).

Done when IDF's own tusb_msc example enumerates to the host, which reads the capacity, reads and
writes blocks, mounts FAT32 and writes and reads a file, as a golden with the card's contents
checked afterwards.

### 4. The co-processor

A C6 machine running unmodified ESP-Hosted slave firmware, joined to the P4 machine over SDIO:
an SDIO slave model on the C6, slot 1 of the P4's SDMMC host on the other side, and a harness
that steps both machines in lock-step the way `--cooja` steps 802.15.4 nodes. The C6 already
joins the virtual access point with the unmodified WiFi blob, so the P4 reaches the host's
network through the existing NAT.

Done when IDF's WiFi station example on the P4 takes a lease through ESP-Hosted's unmodified
slave on the C6, as a golden.

BLE is not modelled (see the roadmap); firmware that uses it must tolerate a controller that
never answers.

## Out of scope

The LP core, MIPI-DSI/CSI, DMA2D and PPA, the rest of Xesppie and the hardware loops, rev ≥ 3
until rev < 3 boots, and cycle accuracy.
