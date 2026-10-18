# ESP32-P4 (RISC-V) — draft

The first chip here with two RISC-V cores: silicon **rev < 3** (v0.x, v1.x), as on the Waveshare
ESP32-P4-WIFI6 (v1.0).
Unmodified ESP-IDF firmware built for rev < 3 boots from the real mask ROM, through the real
2nd-stage bootloader, into FreeRTOS on one or both cores, and reaches an SD card in the board's
socket through the SDMMC host ("SD card" below); an image built for rev ≥ 3 is
refused ("Revisions" below). This page describes the chip and its peripherals.

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

## SD card

`--sd IMAGE` puts a card holding the image in the reference board's SD socket; `--sd-rw IMAGE`
does the same and writes through to the file, where `--sd` keeps what the firmware writes in
memory and never modifies the file. `--sd-prg-us N` gives the card a programming time, and
`--sd-serial HEX` its CID serial number. A card put in the socket again replaces the one there,
which is how the browser build's load kind 8 swaps images. ESP-IDF v5.5.3's own sdmmc example
(`examples/sd_card-p4`, golden `sdcard-p4`) mounts a FAT32 card through it, writes `hello.txt`,
renames it, reads it back and unmounts:

```sh
./target/release/esp32sim-p4 --flash-mb 2 --bootloader $FW/p4-sdcard-bootloader.bin \
    --ptable $FW/p4-hello-ptable.bin --app $FW/p4-sdcard.bin --sd-rw card.img --max-seconds 1
```

```
I (87) example: Filesystem mounted
Name: SIMSD
Type: SDHC
Speed: 20.00 MHz (limit: 20.00 MHz)
Size: 64MB
CSD: ver=2, sector_size=512, capacity=131072 read_bl_len=9
SSR: bus_width=4
I (87) example: Read from file: 'Hello SIMSD!'
I (87) example: Card unmounted
```

The golden's card is a 64 MiB image the test lays out by hand (`tests/sd_image.rs`: an MBR,
FAT32 from LBA 2048), so its bytes are the same on every host; afterwards the test reads
`FOO.TXT` and `NIHAO.TXT` back from it. A blank image ends in `Failed to mount filesystem`
(the example does not format) and an empty socket in `ESP_ERR_TIMEOUT`, the card's commands
timing out (`sd_card_p4_unformatted_and_absent`).

The second golden, `sdcard-sectors-p4` (`examples/sd_card-p4-sectors`), adds raw transfers of 64
and 128 sectors from DMA-capable buffers: CMD25 and CMD18 with the auto-stop, longer than the
16 KiB the driver's four descriptors hold, so its interrupt handler refills them; a misaligned
buffer the driver splits into CMD17s; a read past the end the driver refuses (0x104); and a card
that programs for 2 ms, which the driver waits out polling CMD13 (`write 0x0 in 2039 us`).

### The host

The SDMMC block (0x50083000) is Synopsys' DesignWare Mobile Storage Host with Espressif's
wrapper from 0x800. It is modelled once, in `esp-periph/src/sdmmc.rs`, for the chips that carry
the core: the ESP32 and S3 have the same registers below 0x200 and a CLOCK register at 0x800
(`SdmmcLayout`); only the P4 is wired. Sources are IDF v5.5.3
`soc/esp32p4/register/hw_ver1/soc/sdmmc_struct.h` and `sdmmc_reg.h`, and what the driver does
with them, `esp_driver_sdmmc/src/sdmmc_host.c` and `sdmmc_transaction.c`,
`hal/esp32p4/include/hal/sdmmc_ll.h`.

A command runs to completion in the register write that starts it (the bus calls
`Sdmmc::run` after any write that left one pending): the response, CMD_DONE, the whole data
phase through the IDMAC, DATA_OVER and the auto-stop. Nothing on the bus takes emulated time;
the devices see it (`Sdmmc::now_ns`, set before every SDMMC access from the device clock) for
their own busy periods. A write's DATA_OVER comes when its last block is on the card, before the
card has programmed it, as on the DesignWare core.

| register | modelled |
| --- | --- |
| CTRL 0x00 | controller, FIFO and DMA reset (bits 0-2) clear at once; the controller and DMA resets abort a transfer; INT_ENABLE (4) gates the controller's interrupt; DMA_ENABLE, USE_INTERNAL_DMA (25) read back |
| CLKDIV, CLKSRC, CLKENA, TMOUT, FIFOTH, DEBNCE, USRID, UHS, RST_N, EMMCDDR, ENSHIFT | read back from their reset values; the card clock is not checked |
| CTYPE 0x18 | 1, 4 or 8 lines per slot; a data phase on other lines than the card drives fails every block's CRC (DCRC) |
| BLKSIZ, BYTCNT | block and transfer size, 512 after reset |
| INTMASK, MINTSTS, RINTSTS | MINTSTS is RINTSTS & INTMASK; RINTSTS is write-1-to-clear |
| CMDARG, CMD 0x2C | START (31) stays set until the command is taken; a clock-update command (UPDATE_CLK_REG, 21) is taken at once without CMD_DONE; CHECK_RESPONSE_CRC (8) asks for RCRC; STOP_ABORT ends a transfer, a suspended one too; SEND_AUTO_STOP sends CMD12 after the data |
| RESP0-3 | a short response in RESP0, an R2 in RESP0-3 (bit 0 first), the auto-stop's in RESP1 |
| STATUS 0x48 | FIFO empty, below the transmit watermark, command FSM idle, DAT3 high with a device on the slot, DATA_BUSY from the card at the current time, the index of the last response; reading it changes neither the card nor its time |
| CDETECT, WRTPRT 0x50, 0x54 | the GPIO matrix's card-detect and write-protect inputs of each slot (below) |
| TCBCNT, TBBCNT | bytes moved to or from the card, and through the FIFO |
| VERID, HCON | 0x5432270A and HCON's field reset values |
| BMOD, PLDMND, DBADDR, IDSTS, IDINTEN, DSCADDR, BUFADDR | the IDMAC (below) |
| CARDTHRCTL 0x100 | BUSY_CLR_INT_EN (1): BCI (RINTSTS 13) at the first host access after the card that took a write releases DAT0 (the host keeps no clock to raise it at that moment); a STATUS read that raises it re-derives the interrupt line |
| CLK_EDGE_SEL, RAW_INTS, DLL_CLK_CONF, DLL_CONF 0x800-0x80C | CLK_EDGE_SEL from its reset value; RAW_INTS reads RINTSTS (a guess: the header says only "raw ints"); DLL_CONF reports DLL_CAL_END |

A device answers a command with a response or an `SdCmdError`: no response (RTO with CMD_DONE,
also for a slot with nothing on it), a response with a bad CRC (RCRC when the command asks for
the check) or a malformed one (RESP_ERR); after either error the data phase does not start. A
data error is the device's `SdDataError`: a read that gets no data is DRTO, a block with a bad
CRC either way DCRC (IDF: ESP_ERR_INVALID_CRC), a write that gets no CRC status EBE; DATA_OVER
follows it, as the core reports data-over after a data error, with the IDMAC's CES. The
auto-stop goes out after a data error as well. That is an assumption that only a board can
settle, and IDF depends on it to get the card out of its data state. The interrupt is source
23 (`ETS_SDIO_HOST_INTR_SOURCE`, include/soc/interrupts.h): `CTRL.INT_ENABLE && MINTSTS`, or
TI/RI in IDSTS & IDINTEN with IDINTEN.NI, or FBE/DU/CES with IDINTEN.AI. IDF enables NI, RI and
TI only, so a DU interrupts it through the RI before it.

The IDMAC walks descriptors in memory (`sdmmc_desc_t`, sdmmc_struct.h): chained (CH, DES3) or in
a ring (the next 16 bytes plus BMOD's skip length, back to DBADDR after ER), one or two buffers
each. A finished descriptor goes back to the CPU with OWN cleared and its other bits kept, and
raises RI (reception) or TI (transmission) with NIS unless DIC is set. One the CPU still owns
suspends the transfer with DU and AIS; a PLDMND write fetches it again. An address it cannot
reach is a fatal bus error (FBE, AIS, FBE_CODE). It reaches L2MEM and PSRAM through its MMU, at
cached or uncached addresses (`SOC_SDMMC_PSRAM_DMA_CAPABLE`); caches are not modelled. IDF's
driver keeps four descriptors of up to 4 KiB in a ring and refills them from its interrupt
handler, so a transfer over 16 KiB suspends on DU once per refill, which silicon only does when
the CPU is slower than the card.

**Slots and the socket.** A device hangs on pins, and the routing decides which slot reaches it,
looked up when a command starts (`Peripherals::sd_routes`). Slot 0 is wired to its IO MUX pins,
CLK 43, CMD 44, D0-D3 39-42 (`soc/esp32p4/include/soc/sdmmc_pins.h`): it reaches the socket
while the clock pad selects function 0 (`FUNC_GPIO43_SD1_CCLK_PAD`, io_mux_reg.h). Slot 1 goes
through the GPIO matrix: it reaches a device whose clock pad is a GPIO (function 1) driving
`SD_CARD_CCLK_2_PAD_OUT_IDX` (gpio_sig_map.h). The `--sd` card sits on slot 0's pins, as on the
reference board; IDF's example reaches it through slot 1 and the matrix (`SDMMC_HOST_DEFAULT()`
is slot 1 on the P4), an application configured for slot 0 through the IO MUX. ESP-Hosted's C6 will be an
`SdDevice` attached at CLK 18, CMD 19, D0-D3 14-17 (`Peripherals::attach_sd`, which replaces a
device already on those pins), which slot 1 reaches the same way; the trait already carries the
SDIO interrupt (DAT1), unused so far.
CDETECT and WRTPRT read `SD_CARD_DETECT_N_1/2` and `SD_CARD_WRITE_PRT_1/2` (signals 126, 127,
130, 131) through FUNCn_IN_SEL: a pin's level or a constant, inverted if asked. The driver
routes constant low to card detect and inverted constant high to write protect when the board
has neither line, so a card counts as present on both slots.

**Clocks, reset, pins, power.** HP_SYS_CLKRST's SOC_CLK_CTRL1.REG_SDMMC_SYS_CLK_EN (bit 14) and
PERI_CLK_CTRL01/02 (the SDIO_LS clock source, enable and divider edges) read back, which is what
`sdmmc_host_get_real_freq` computes the bus frequency from (160 MHz PLL / 8 = 20 MHz for the
example, / 4 = 40 MHz for `SDMMC_FREQ_HIGHSPEED`); the clock gate is not checked. LP_CLKRST
HP_SDMMC_EMAC_RST_CTRL.RST_EN_SDMMC (0x5011104C bit 28) resets the host's registers and leaves
what is attached (lp_clkrst_reg.h). IO_MUX is register RAM; the driver only writes it. The card's
on-chip LDO (channel 4, `sd_pwr_ctrl_by_on_chip_ldo.c` through `hal/esp32p4/include/hal/ldo_ll.h`)
is PMU configuration the driver writes and never polls, register RAM here; its `ldo: The
voltage value 0 is out of the recommended range` warning is IDF's own.

**GPIO layout.** The P4 has 57 PIN registers from 0x74, which puts FUNCn_IN_SEL_CFG at 0x158 and
FUNCn_OUT_SEL_CFG at 0x558, four bytes above the C6's (register/hw_ver1/soc/gpio_reg.h).
`periph::GpioP4` moves both arrays down four bytes into the shared model, keeps PIN49-56 and
OUT_SEL49-56 as register RAM, and resets FUNCn_IN_SEL to the header's values (0x3F, constant
high, or 0x3E, constant low). Pin levels stay the shared model's 49.

### The card

`esp-periph/src/sdmmc/card.rs`, after the SD Physical Layer Simplified Specification v9.10:

- **States** (§4.2, §4.3, the transitions of §4.8): idle, ready, ident, stby, tran, data, rcv,
  prg, dis and ina. An R1 shows CURRENT_STATE as the command found the card, READY_FOR_DATA
  cleared while programming (§4.10.1). A command that is not legal in the state, including the
  SDIO commands 5, 52 and 53, is not answered and sets ILLEGAL_COMMAND for the next response;
  the error bits clear with the response that reports them. A card deselected while programming
  goes to dis and from there to stby when programming ends; selected again it returns to prg
  only while it still programs.
- **Identification** (§4.2): CMD0; CMD8 echoes its check pattern for 2.7-3.6 V and stays silent
  otherwise; CMD55 and ACMD41, busy for one poll, then ready with CCS. Without HCS, or without a
  CMD8 since the reset, the card stays busy, as a high capacity card must (§4.2.3). CMD2 and CMD10
  give the CID, CMD3 an RCA (R6), CMD9 the CSD, CMD7 selects and deselects (R1b), CMD4 and CMD15.
- **Registers** (chapter 5): CID from `SdCardConfig`, by default MID 0, OID "ES", product
  "SIMSD", revision 1.0, a serial number derived from the image size and 2024-01; CSD version 2.0
  with C_SIZE from the image,
  TAAC 1 ms, CCC 0x535 (classes 0, 2, 4, 5, 8, 10), TRAN_SPEED 25 MHz or 50 MHz once high speed is
  switched on; both with their CRC7 (§4.5). SCR: SD 3.0x, security 2.00 (3.00 above 32 GiB),
  1 and 4 lines, CMD23. SD status (ACMD13, §4.10.2): bus width, speed class 10, AU 4 MiB. OCR
  2.7-3.6 V, no 1.8 V.
- **Transfer** (§4.3): CMD6 check and switch (§4.3.10; group 1 offers default and high speed),
  ACMD6 bus width, CMD13, CMD16 (blocks stay 512 bytes), CMD17 and CMD18 reads, CMD24 and CMD25
  writes, CMD12, CMD23 block count (for the next CMD18 or CMD25; any other read or write clears
  it), ACMD22 written blocks, ACMD23 and ACMD42 accepted, CMD32, CMD33 and CMD38 erase (erased
  blocks read 0). A start address past the end answers OUT_OF_RANGE and moves no data; a
  multi-block transfer that runs past it stops there.
- **Time.** The card answers at once. ACMD41 reports busy `SdCardConfig::init_polls` times (1).
  After the last block of a write, after the CMD12 that ends a multi-block write, and after an
  erase the card programs for `SdCardConfig::prg_ns` of emulated time (`--sd-prg-us`; 0 by
  default): DAT0 busy, prg, not ready for data, and a read or write command meanwhile is illegal.
  The driver's own writes wait for it with CMD13, but its write returns only after that, so a
  read from another task can reach the card while it programs: no response (IDF:
  `sdmmc_send_cmd returned 0x107`), then a CMD13 status of 0x400e00, ILLEGAL_COMMAND in prg.
- **Image.** A card of the image's size rounded down to 512 KiB (C_SIZE's unit), from 512 KiB to
  2 TiB, held by an `SdStore`: `FileStore` (copy-on-write or write-through) or `MemStore`. It is
  SDHC up to 32 GiB and SDXC above (SCR security 3.00); a card under 2 GB would be SDSC on
  silicon, and this one is SDHC at every size (IDF prints `SDHC` for both). A copy-on-write
  erase is kept as a range, not a block per address. The card stays in the socket through a chip
  reset, with its state; IDF's init resets it with CMD0.
- **Not modelled**: SDSC (byte addresses, CSD 1.0), UHS-I and CMD11, DDR, eMMC, lock and unlock,
  write protection, the PIO data FIFO at 0x200, the card-detect interrupt, the time a command or
  a transfer takes on the bus.

`--debug sdmmc` prints every command, its response and the card's state. The run's report adds
the commands, the unanswered ones and the bytes moved.

The SD card costs the other chips nothing: it is new code the P4 alone calls, plus a default
method on `SocBus` and a load kind (8) for the browser build. The P4 pays one flag test per
peripheral write, an address compare per peripheral read, and a few compares per GPIO and SDMMC
access.

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
| Peripherals | UART0/1, USB-Serial/JTAG, systimer (16 MHz from XTAL), TIMG0/1, SHA, efuse (v1.0, block v0.3, security efuses zero), the flash controller (SPI0 cache side, SPI1 commands, a GigaDevice part answering RDID), the PSRAM controller with an AP Memory x16 PSRAM, the cache controller, HP_SYSTEM (cross-core interrupts, stall status), HP_SYS_CLKRST (divider update, PLL calibration done, core 1 clock and reset), LP_SYSTEM / LP_CLKRST / LP_TIMER (reset cause, software reset, core 1 boot address, RTC counter, the SDMMC reset), the analog I2C master (regi2c read-back), ASSIST_DEBUG's saved PC, the RNG, GPIO with the P4's register layout, the SDMMC host with its IDMAC and an SD card in the board's socket (`--sd`); everything else is register RAM, listed by `--log-periph` |
| Time base | 360 MHz, the rev < 3 application clock; one instruction is one cycle |

The revision is a type parameter (`esp32p4::Revision`, `RevLt3`): rev ≥ 3 (its own ROM, the
`mintthresh` CSR, the L2 cache at the bottom of L2MEM) is a second implementation, not a change
to this one, and an image built for it is refused until then ("Revisions"). The browser build
boots the same images (`web/wasm/fw/p4-hello.json`, run by `tools/wasm-test.mjs p4-hello`); its
firmware dialog offers the board `esp32p4`.

## Not there yet

- **USB-OTG, ESP-Hosted**: the USB port and the C6 co-processor.
- **AXI-GDMA, the LP core, MIPI-DSI/CSI, DMA2D/PPA, rev ≥ 3**, and Xesppie's save and restore,
  which FreeRTOS runs only for a task that uses PIE.
- **GPIO** has the P4's register layout (`periph::GpioP4`), but pin levels, interrupts and edges
  are the shared model's 49 pins; the P4 has 55.
- **`--boot app`** maps the image and jumps to it without the clocks and caches the bootloader
  would have set.
- **`Machine::run_until_cycle`** still schedules core 0 alone; nothing that drives the P4 needs it.
- **Unmodelled blocks** hello_world touches: PMU, PVT, APB_SARADC, LP_WDT, LP_ANA_PERI, LP_ADC,
  KEYMNG, UART2-4, TRACE0/1, LP_UART, LP_PERI, LP_GPIO and DSI, all configuration the firmware
  writes and reads back (`--log-periph` lists them). The watchdogs never fire.
