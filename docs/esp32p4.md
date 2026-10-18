# ESP32-P4 (RISC-V) — draft

The fourth chip, and the first with two RISC-V cores: silicon **rev < 3** (v0.x, v1.x), as on the
Waveshare ESP32-P4-WIFI6 (v1.0) the plan targets ([esp32p4-plan.md](esp32p4-plan.md), milestone 1).
Unmodified ESP-IDF firmware boots from the real mask ROM, through the real 2nd-stage bootloader,
into FreeRTOS on one or both cores, reaches an SD card in the board's socket through the SDMMC
host (milestone 2, "SD card" below), and presents it as a USB mass-storage device to a host on
the high-speed USB-OTG port, a car written in Rust (milestone 3, "USB" below).

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
single-core, `p4-hello-smp*` both cores (golden `hello-p4-smp`, where both cores take their own
systimer tick and cross-core interrupts). `esp32sim --chip p4` is the same binary; the boot mode
defaults to `rom` and the console to UART0. The mask ROM ELF is `esp32p4_rev0_rom.elf` (ECO0,
from ESP-IDF's esp-rom-elfs or `tools/fetch-rom-elfs.sh`). No ROM is published for the ECO2
silicon on v1.x boards. ECO0's jump table matches every entry of IDF v5.5.3's rev < 3
`esp32p4.rom.ld`, so images built for v1.x link against it; the first two console lines are
ECO0's, not the board's.

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

The core's P4 paths cost the other chips nothing, measured against cdbbe33d (the F extension
alone) with `cargo build --release --bins`, alternating runs, median `user` time, identical
instruction counts (the SD card's receipt is in "SD card" below):

| demo | runs | cdbbe33d | this branch | instructions |
| --- | --- | --- | --- | --- |
| S3 `hello` (30 s) | 7 | 0.38 s | 0.38 s | 11,990,488 + 6,798,360 |
| `c3-hello` (30 s) | 7 | 3.86 s | 3.81 s | 4,800,000,000 |
| `c6-hello` (30 s) | 7 | 5.59 s | 5.63 s | 4,800,000,000 |
| pocket-tank (8 s) | 5 | 13.92 s | 13.98 s | 1,113,634,993 + 1,490,529,616 |

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

With `--sd-timing` (`SdCardConfig::timed_bus`) the bus takes its time. The data still moves
through the IDMAC in the write that starts the command, but CMD_DONE comes when the command and
its response are through (48 bits, NCR 2, 48 or 136 bits, NCC 8 card clocks; an unanswered one
after the 64-clock response timeout), DATA_OVER when every block is (a start bit, the block's
bits over the slot's width, CRC16 and an end bit on each line, a write's CRC status token, two
clocks between blocks: SD Physical Layer Simplified Specification v9.10 §3.6.1, §4.12), ACD after
the auto-stop's own command, and a write's card programs from the end of its data on, BCI
following when it is done. The card clock is the one IDF computes in `sdmmc_host_get_real_freq`:
HP_SYS_CLKRST PERI_CLK_CTRL01.REG_SDIO_LS_CLK_SRC_SEL (PLL_F160M or the 200 MHz SDIO PLL), divided
by PERI_CLK_CTRL02's SDIO_LS EDGE_L + 1, then by the slot's CLKDIV divider (CLKSRC picks it; n
divides by 2n). The deferred interrupts are raised from the device ticks at their time
(`Sdmmc::next_event_ns`, `advance`), which bound the bus's deadline. The card's read access time
and the IDMAC's pace through the descriptors are not timed: a long transfer's descriptors come
back at once, its DATA_OVER at the end.

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
is slot 1 on the P4), oxidrive through slot 0. ESP-Hosted's C6 (milestone 4) will be an
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
example, / 4 = 40 MHz for oxidrive); the clock gate is not checked. LP_CLKRST
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
  `sdmmc_send_cmd returned 0x107`), then a CMD13 status of 0x400e00, ILLEGAL_COMMAND in prg,
  which is what the reference board reported for reads made while its card took writes over USB.
- **Image.** A card of the image's size rounded down to 512 KiB (C_SIZE's unit), from 512 KiB to
  2 TiB, held by an `SdStore`: `FileStore` (copy-on-write or write-through) or `MemStore`. It is
  SDHC up to 32 GiB and SDXC above (SCR security 3.00); a card under 2 GB would be SDSC on
  silicon, and this one is SDHC at every size (IDF prints `SDHC` for both). A copy-on-write
  erase is kept as a range, not a block per address. The card stays in the socket through a chip
  reset, with its state; IDF's init resets it with CMD0.
- **Not modelled**: SDSC (byte addresses, CSD 1.0), UHS-I and CMD11, DDR, eMMC, lock and unlock,
  write protection, the PIO data FIFO at 0x200, the card-detect interrupt, and (without
  `--sd-timing`) the time a command or a transfer takes on the bus.

`--debug sdmmc` prints every command, its response and the card's state. The run's report adds
the commands, the unanswered ones and the bytes moved.

The SD card costs the other chips nothing: it is new code the P4 alone calls, plus a default
method on `SocBus` and a load kind (8) for the browser build. Measured on the final tree against b31a21de with
`cargo build --release --bins`, alternating runs, median `user` time, identical instruction
counts:

| demo | runs | b31a21de | this branch | instructions |
| --- | --- | --- | --- | --- |
| S3 `hello` (30 s) | 7 | 0.37 s | 0.37 s | 11,990,488 + 6,798,360 |
| `c3-hello` (30 s) | 7 | 3.80 s | 3.79 s | 4,800,000,000 |
| `c6-hello` (30 s) | 7 | 5.58 s | 5.58 s | 4,800,000,000 |
| `p4-hello` (27 s) | 7 | 17.51 s | 17.38 s | 9,720,000,000 + 0 |
| `p4-hello-smp` (27 s) | 7 | 18.42 s | 18.38 s | 9,720,000,000 + 9,713,148,105 |
| pocket-tank (8 s) | 5 | 13.96 s | 13.93 s | 1,113,634,993 + 1,490,529,616 |

The P4 runs pay one flag test per peripheral write, an address compare per peripheral read, and
a few compares per GPIO and SDMMC access; nothing in the change makes them faster, so the
0.2-0.7% they gained is most likely code layout. An earlier round, before the card's
programming time and the read-side check, measured the same within 1%.

## USB

`--usb-host FILE` plugs a host into the high-speed USB-OTG port and runs FILE's commands
([cli.md](cli.md), "USB host"). ESP-IDF v5.5.3's own tusb_msc example (`examples/usb_msc-p4`,
golden `usbmsc-p4`), built for SD card storage, presents the card in the socket over USB mass
storage through TinyUSB 0.21.0~2 and esp_tinyusb 2.4.0; the car in `car.txt` enumerates it,
records a clip on its FAT32 volume and reads it back, ejects it and polls it while the example's
application owns the card:

```sh
./target/release/esp32sim-p4 --flash-mb 4 --bootloader $FW/p4-usbmsc-bootloader.bin \
    --ptable $FW/p4-usbmsc-ptable.bin --app $FW/p4-usbmsc.bin --sd-rw card.img \
    --script examples/usb_msc-p4/console.txt --usb-host examples/usb_msc-p4/car.txt --max-seconds 2
```

```
[usb-host] t=0.190625 port reset: High speed
[usb-host] t=0.203250 device 303a:4002, USB 2.00, class ef/02/01, EP0 64 bytes, 1 configuration(s)
[usb-host] t=0.203750 interface 0: class 08/06/50, endpoints 01 (512 bytes) 81 (512 bytes)
[usb-host] t=0.206375 READ CAPACITY: 131072 blocks of 512 bytes (64 MiB)
[usb-host] t=0.566250 fat-write CLIP0001.MP4: 1048576 bytes in 16 chunks of 64 KiB, 0.355 s, 2.95 MB/s
[usb-host] t=0.609500 fat-read CLIP0001.MP4: 1048576 bytes, hash 208645dd266d37e6, 24.24 MB/s, matches what was written
[usb-host] t=0.611125 ejected
[usb-host] t=0.611750 TEST UNIT READY: sense 2/3a/00 (MEDIUM NOT PRESENT)
[usb-host] t=1.517000 TEST UNIT READY passed after 4 failed polls
[usb-host] t=1.518875 root: CLIP0001.MP4 (1048576), README.MD (379)
```

When the host configures the device, esp_tinyusb takes the card from the application and
exposes it; the eject (START STOP UNIT with LoEj) gives it back, the example's console writes
README.MD on it and `expose` hands it to USB again. The golden pins the console, the host's log
with its emulated times and the instruction count, then reads the clip, README.MD and the raw
blocks the car wrote back from the image.

### The core

The DWC2 core (Synopsys DesignWare Hi-Speed USB 2.0 OTG) is modelled once, in
`esp-periph/src/usb_otg.rs`, as a device with buffer DMA; what differs between instances is
`Dwc2Config` (the S3's full-speed core would be a second configuration, not wired). Sources: IDF
v5.5.3 `soc/esp32p4/include/soc/usb_dwc_struct.h` (the v4.30a HS instance),
`soc/esp32p4/register/hw_ver1/soc/usb_dwc_cfg.h`, `hal/esp32p4/include/hal/usb_dwc_ll.h`; the bit
names and what the driver expects of them: TinyUSB 0.21.0~2 `src/portable/synopsys/dwc2/`
`dwc2_type.h`, `dwc2_common.c`, `dcd_dwc2.c`. It is the P4's block 0x50000 as an optional
device-table entry, so it is ticked and asked for deadlines only while a host is plugged in. The
interrupt is source 93 (`ETS_USB_OTG_INTR_SOURCE`, include/soc/interrupts.h): GAHBCFG.GINT and
an unmasked GINTSTS bit. LP_CLKRST HP_USB_CLKRST_CTRL1.RST_EN_USB_OTG20 (0x50111048 bit 2,
lp_clkrst_reg.h) resets its registers, which `usb_new_phy` does through
`_usb_utmi_ll_reset_register`; a chip reset keeps the cable. Its DMA reaches L2MEM and PSRAM like
the SDMMC IDMAC (`SOC_PSRAM_DMA_CAPABLE`).

| register | modelled |
| --- | --- |
| GOTGCTL 0x000 | the VBUS, A- and B-valid overrides and the HNP bits read back; CIDSTS 1 (a B-device); BSESVLD from VBUS or the B-valid override, ASVLD from the A-valid override |
| GOTGINT 0x004 | write 1 to clear; SEDET when VBUS goes |
| GAHBCFG, GUSBCFG 0x008, 0x00C | read back; GINT gates the interrupt; GUSBCFG.FHMOD keeps D+ released |
| GRSTCTL 0x010 | AHBIDL reads 1; CSRST is a core soft reset: from 4.20a CSRST stays set until software clears it and CSRSTDONE is write 1 to clear (`usb_dwc_ll_grstctl_core_soft_reset`), before it CSRST clears itself; RXFFLSH, TXFFLSH, FCRST and HSRST clear at once, TXFNUM reads back |
| GINTSTS, GINTMSK 0x014, 0x018 | the latched bits are write 1 to clear; OTGINT follows GOTGINT, GINAKEFF and BOUTNAKEFF the global NAKs, IEPINT and OEPINT DAINT & DAINTMSK, SOF the (micro)frames since the last clear; CURMOD 0 |
| GRXSTSR, GRXSTSP 0x01C, 0x020 | 0: slave mode is not modelled |
| GRXFSIZ, GNPTXFSIZ, DIEPTXF1-15, GDFIFOCFG | read back, from the configuration's FIFO depths; an IN endpoint enabled with a TX FIFO smaller than its packets NAKs and counts a FIFO size error |
| GNPTXSTS 0x02C | four free queue entries, the FIFO empty |
| GSNPSID, GHWCFG1-4 0x040-0x050 | 0x4F54430A and 0, 0x215FFFD0, 0x03805EB5, 0xDFF1A030 (below) |
| DCFG 0x800 | read back; DSPD 0 makes the next reset end at high speed |
| DCTL 0x804 | SDIS releases D+ (the host sees a disconnect); SGINAK/CGINAK and SGONAK/CGONAK set and clear GINSTS/GONSTS at once; RWUSIG signals remote wakeup |
| DSTS 0x808 | SUSPSTS, ENUMSPD (0 high, 1 full speed), SOFFN counting microframes at high speed |
| DIEPMSK, DOEPMSK, DAINT, DAINTMSK, DIEPEMPMSK | a DAINT bit is set while the endpoint's interrupt register has a bit its mask lets through; DAINTMSK does not gate it |
| DIEPCTLn, DOEPCTLn | MPS, USBAEP, DPID, NAKSTS, EPTYP, STALL, TXFNUM; EPENA is set by software and cleared by the core at the end of a transfer and at SETUP done; EPDIS disables an enabled endpoint (EPDISD); SNAK (and INEPNE), CNAK, SD0PID, SD1PID; control endpoint 0 takes DIEPCTL0's packet size both ways, a SETUP clears its STALL and sets NAK on both, control OUT 0 cannot be disabled |
| DIEPINTn, DOEPINTn | write 1 to clear; TXFE reads 1 |
| DIEPTSIZn, DOEPTSIZn | XferSize and PktCnt count down by packet (17 and 8 bits, OTG20_TRANS_COUNT_WIDTH and OTG20_PACKET_COUNT_WIDTH; 7 and 2 or 1 on endpoint 0), STUPCNT on control OUT 0 |
| DIEPDMAn, DOEPDMAn | the DMA address, advancing with the data |
| DTXFSTSn | the TX FIFO's depth: it is empty between transactions |
| PCGCCTL 0xE00 | read back |
| the rest | register RAM, as are the UTMI PHY (0x5009C000), USB_WRAP and the clock gates the driver sets (HP_SYS_CLKRST SOC_CLK_CTRL1.REG_USB_OTG20_SYS_CLK_EN, LP_CLKRST HP_USB_CLKRST_CTRL1's PHY reference clock, HP_SYSTEM USBOTG20_CTRL); the clock gate is not checked |

A transaction is what the host asks of the core and is answered at once from the endpoint
registers:

- **SETUP** (always acknowledged, USB 2.0 §8.5.3): written by DMA at DOEPDMA0, which advances 8
  bytes, STUPCNT counting down from the 3 TinyUSB arms, Setup Packet Received set; past three
  back-to-back SETUPs the last slot is written again with B2BSTUP. The token after them ends the
  setup phase: SETUP (DOEPINT0 bit 3) and control OUT 0 handed back (EPENA cleared), so the driver
  finds the last SETUP at DOEPDMA0 - 8 (`handle_epout_dma`). A SETUP that comes while control OUT
  0 is not enabled waits, as in the RxFIFO, until it is.
- **IN**: NAK unless the endpoint is enabled, not NAKing and no global IN NAK holds; STALL if set;
  otherwise min(XferSize, MPS) bytes from DIEPDMA with the endpoint's data PID, and the transfer
  ends (XFRC, EPENA cleared) on its last packet. A zero XferSize sends a zero-length packet.
- **OUT**: NAK and STALL the same way; a packet with the wrong data PID is acknowledged and dropped
  (§8.6.4); otherwise written by DMA at DOEPDMA, XferSize counts down by its length, and a short
  packet or the last of PktCnt ends the transfer (XFRC, EPENA cleared, NAK set). More than XferSize
  is cut at XferSize with BBLEERR.
- A DMA access the memory map refuses is AHBERR and a NAK.

Bus events come from the link: VBUS on sets SRQINT, VBUS off SEDET (TinyUSB reports the unplug);
a bus reset sets USBRST and clears USBActEP but on endpoint 0, its end ENUMDNE at the speed the
chirp allowed; SOFs count from enumeration; early suspend (ESUSP) and suspend (USBSUSP, SUSPSTS),
then resume (WKUINT). `--debug usb_otg` prints every transaction and bus event.

**Identity.** GSNPSID is 4.30a: `usb_dwc_struct.h` describes "the High Speed v4.30a instance"
and `hal/usb_dwc_hal.c` accepts 4.00a and 4.30a. GHWCFG1-4 are the values TinyUSB 0.21.0~2
recorded from silicon (`dwc2_info.py`, `ESP32-P4`): internal DMA, a UTMI+/ULPI HS PHY, 15 device
endpoints besides endpoint 0, 16 host channels, dynamic FIFOs, per-endpoint interrupts, FIFO
queue depths of 4, 8 IN endpoints, one control endpoint, dedicated TX FIFOs, descriptor DMA, all
as `usb_dwc_cfg.h` configures the instance; TinyUSB reads the PHY type, the endpoint count, the
DMA architecture and the PHY width from them. The open points are listed below.

### The host

The host is two layers, the host controller on the cable and the software that drives it.

`esp_periph::usb_otg::UsbLink` (`usb_otg/link.rs`) is the host controller, owned by the core it
is plugged into and running in device time: one request at a time from host software (power,
reset, suspend, resume, a control or a bulk transfer), carried out as the transactions and bus
states a controller puts on the wire, each at its own time, against the device side
(`UsbFunction`, which the core implements). Its timing model, in CPU cycles:

- A transaction takes its payload plus a fixed protocol overhead at the line rate: 55 bytes at
  480 Mbit/s at high speed, 13 at 12 Mbit/s at full speed (USB 2.0 §5.8.4, the bulk protocol
  overhead). The transactions of a transfer follow back to back, and none crosses a (micro)frame
  boundary, so a high-speed bulk transfer moves at most 13 packets of 512 bytes in a 125 µs
  microframe (53 MB/s). `UsbTiming::frame_budget` (`rate` in a script) caps the payload per
  microframe below that.
- A NAKed transaction is tried again at the start of the next microframe (the next frame at full
  speed): what an EHCI controller does once a queue head's NAK counter runs out, and how a
  high-speed bulk OUT is paced by PING (§8.5.1). A NAK costs a retry a microframe, not a stream of
  them, so a waiting endpoint is cheap to emulate.
- SOFs every 125 µs from the end of the bus reset while the port is enabled and not suspended
  (§8.4.3); the core counts them from when they start rather than receiving each one, and asks
  for a deadline only while its SOF interrupt is unmasked.
- A bus reset holds SE0 for 50 ms (TDRSTR from a root port, §7.1.7.5) and ends in the speed the
  device's chirp allows (high speed while DCFG.DSPD is 0); resume drives K for 20 ms (TDRSMDN,
  §7.1.7.7). With SOFs stopped the core sees early suspend after 3 ms and suspend 125 µs later
  (§7.1.7.6).
- One device hangs on the cable: tokens reach it whatever its address.

Its next event is the core's device deadline, so idle cores sleep through a transfer to the
packet that wakes them, and the bus runs what is due after the device ticks of a round.

`esp_soc::usb_host` is the host software, written against a `UsbPort` (the link, and a way to let
emulated time run to a given nanosecond, which `Machine` implements by running to that exact
cycle). `UsbHost` learns that a request finished at the next microframe boundary after it did
(the one-microframe interrupt threshold Linux's EHCI driver programs) and submits the next one
there, so the firmware runs between the transfers of a command:

- `enumerate`: VBUS, waiting for the pull-up, the 100 ms connect debounce (TATTDB, §7.1.7.3), the
  port reset, 10 ms of reset recovery (§9.2.6.2), the device descriptor's first 64 bytes at address
  0, SET_ADDRESS and its 2 ms recovery (§9.2.6.3), the device and configuration descriptors, the
  strings, SET_CONFIGURATION (§9.1.2, §9.4); control transfers time out after 5 s.
- `msc::Msc`: SCSI over bulk-only transport (BOT 1.0): CBW and CSW with tags, the CSW checked
  for validity and meaning (§6.3), a STALLed data stage cleared and the CSW read, a STALLed CSW
  cleared and read once more, phase errors and untrustworthy CSWs ending in reset recovery
  (Bulk-Only Mass Storage Reset and CLEAR_FEATURE(ENDPOINT_HALT) on both endpoints, §5.3.4), REQUEST
  SENSE after CHECK CONDITION; INQUIRY, TEST UNIT READY, READ CAPACITY(10), MODE SENSE(6), PREVENT
  ALLOW MEDIUM REMOVAL, START STOP UNIT, READ(10) and WRITE(10) of any length in commands of 240
  blocks at most (Linux usb-storage's default), SYNCHRONIZE CACHE(10); `wait_ready` polls TEST UNIT
  READY as a car polls its drive. The LUN is a `fat32::BlockDevice`.
- `fat32::Fat32` mounts a volume (an MBR's FAT32 partition, or from LBA 0), lists, reads and
  writes 8.3 files in the root directory as Linux's vfat writes them: the directory entry at once,
  the data a chunk at a time as it comes (one WRITE(10) per contiguous run of clusters), the FAT
  sectors of each chunk's clusters in both FATs, then the entry's size and first cluster and
  FSInfo. `FileWriter::fat_every` writes the FAT less often; `BlockDevice::pause_ns` is the idle
  gap between chunks.
- `script::Runner` runs a command file (`--usb-host`), each command logging what it did.

A test drives the same API between runs of the machine:

```rust
esp_soc::SocBus::attach_usb_host(&mut m.bus, Default::default())?;
let mut host = UsbHost::new(&mut m);              // the machine is the host's port
host.enumerate(10_000_000_000)?;                  // connect timeout, ns
let mut msc = Msc::open(host)?;
msc.wait_ready(100_000_000, 10_000_000_000)?;     // TEST UNIT READY every 100 ms
msc.read_capacity()?;
let mut fs = Fat32::mount(&mut msc)?;
let mut f = fs.create(&mut msc, "CLIP0001.MP4")?;
f.write(&mut fs, &mut msc, &chunk)?;
f.close(&mut fs, &mut msc)?;
```

`esp-soc/src/usb_host/tests.rs` runs the host against a mass-storage device written in Rust at
the transaction level, with knobs for what BOT's host side must survive (MEDIUM NOT PRESENT, short
data stages, STALLs, phase errors, wrong tags, NAKs); `esp-periph/src/usb_otg/tests.rs` drives
the core with TinyUSB's register sequences against the link.

**Throughput.** A car writing 4 MiB in 64 KiB chunks without gaps, then reading it back (FAT32,
the card instant, programming time 0):

| firmware | SD bus | write | read |
| --- | --- | --- | --- |
| tusb_msc example (8 KiB MSC buffer) | at once | 9.1 MB/s | 24.8 MB/s |
| tusb_msc example | `--sd-timing`, 4 bits at 20 MHz | 4.4 MB/s | 6.8 MB/s |
| oxidrive (32 KiB buffer) | at once | 19.7 MB/s | |
| oxidrive | `--sd-timing`, 4 bits at 40 MHz | 11.0 MB/s | |

oxidrive's board measured about 10.9 MB/s writing over USB. With 16 ms between chunks (a camera
at 4 MB/s) oxidrive takes the clip at 3.5 MB/s, 3.0 MB/s with the bus timed.

The USB port and the timed SD bus cost the other chips nothing and the P4 nothing while unused:
the core is an optional device-table entry, the bus tests one flag per round for the link and
one for a timed SD bus, whose work and deadline sit in cold functions (inlined, they cost the P4
hello demos 0.9-1.0%). Measured against 6bc9b9cf with `cargo build --release --bins`,
alternating runs, median `user` time, identical instruction counts; the P4 rows on the final
tree, the others on the tree before the timed SD bus, which only the P4 runs:

| demo | runs | 6bc9b9cf | this branch | instructions |
| --- | --- | --- | --- | --- |
| S3 `hello` (30 s) | 7 | 0.39 s | 0.39 s | 11,990,488 + 6,798,360 |
| `c3-hello` (30 s) | 7 | 3.96 s | 4.00 s | 4,800,000,000 |
| `c6-hello` (30 s) | 7 | 5.82 s | 5.75 s | 4,800,000,000 |
| `p4-hello` (27 s) | 7 | 18.16 s | 18.09 s | 9,720,000,000 + 0 |
| `p4-hello-smp` (27 s) | 7 | 19.18 s | 19.18 s | 9,720,000,000 + 9,713,148,105 |
| pocket-tank (8 s) | 5 | 14.47 s | 14.42 s | 1,113,634,993 + 1,490,529,616 |

The C3 and C6 run no code this branch changed; their ±1% is layout.

**Open points**, each a guess where the sources disagree or say nothing:

- GSNPSID: TinyUSB's `dwc2_info.py` lists 0x4F54400A (4.00a) for the P4, against IDF's 4.30a.
  TinyUSB's own reset takes either path, and 4.30a's is the one modelled.
- GHWCFG2 and GHWCFG3 differ from `usb_dwc_cfg.h` in single point (0 against
  OTG20_SINGLE_POINT 1), the FS PHY type (3 against 2), the counter widths (16 and 7 bits against 17
  and 8), synchronous reset and battery charger support; the registers keep TinyUSB's values, the
  counters `usb_dwc_cfg.h`'s widths.
- Reset values not in any header: DCTL.SDIS 1 (no pull-up until a driver connects), GUSBCFG's
  USBTrdTim 5, the FIFO sizes from the configured depths; which registers a core soft reset keeps
  (the PHY selection in GUSBCFG, DCFG.DSPD, DCTL.SDIS, PCGCCTL).
- No XFRC with a SETUP, the fourth back-to-back SETUP rewriting the last slot, OUT babble cut at
  XferSize, no NYET handshake, USB reset leaving DCFG.DevAddr to the driver.
- The NAK retry at the next microframe is one controller's policy, not the specification's; a
  controller that retries within the microframe would see a NAKing device sooner.

## Compared with a board

The owner's oxidrive firmware (not in this repository) was booted from the same bootloader,
partition table and app as a board whose console was captured, with the bootloader's flash mode
set to QIO as espflash wrote it, 32 MB of flash and 32 MB of PSRAM, both cores, and a FAT32 card
in the socket: a sparse image of the board card's size, an SDXC card, laid out as a card
formatter lays one out (MBR, FAT32 from LBA 2048, 32 KiB clusters), with the board card's serial
number. `external_oxidrive_console_matches_the_board` (`OXIDRIVE_DIR` names the images and the
capture; the board's MAC, the card's size and its serial number come from there, not from this
repository) compares the two with timestamps stripped, line by line, picking the comparison up
again after lines that differ. 76 of the board's 105 lines match, among them the twelve `hex_psram`
mode-register lines, the MSPI timing tuning, `Found 32MB PSRAM device`, `cpu_start: Multicore
app`, the application information, the heap regions, `spi_flash: flash io: qio`, the ESP-Hosted
start-up, the firmware's own start-up lines, its card probe (size, serial, 4 data lines at
40000 kHz) and its USB mass-storage presentation: the device stack up on the high-speed port, the
card presented, the DMA buffer in internal RAM, the USB link waiting for a host. Three differ by
construction, because the ECO0 ROM stands in for the board's ECO2: the banner pair, and the heap
region below the ROM's own data, whose length IDF takes from the ROM's `ets_rom_layout`
(`_dram0_rtos_reserved_start` is 0x4FF3FBA4 in the ECO0 ELF, and 12 bytes later on the board).
Eight blocks differ:

- the board's flash held a core dump, the emulator's coredump partition is blank: one line
  where the board prints three;
- the firmware's free heap at start, and its free internal RAM later, are 108 bytes lower than
  the board's (open; ESP-Hosted's SDIO link, which has no slave on slot 1, is the first place to
  look);
- the card's FAT geometry, label and root directory: the image's, not the board card's;
- the firmware's journal and configuration, which the board's NVS holds from earlier boots and
  the emulator's blank flash does not: it reports a first boot, seeds a default and runs
  unconfigured (four blocks);
- the die temperature: the temperature sensor is register RAM here.

The board's lines from 93 on come through the C6 (WiFi, the clock, MQTT): milestone 4.

`external_oxidrive_presents_the_card_to_a_car` boots the same images with VBUS from power-up and
the card image written through. The car enumerates oxidrive at 0.27 s at high speed, the drive is
ready at the first TEST UNIT READY, its FAT32 volume (32 KiB clusters) mounts, and a 4 MiB clip
written in 64 KiB chunks 16 ms apart takes 1.2 s (3.5 MB/s; 19.7 MB/s without the gaps), reads
back over USB unchanged and is on the image byte for byte afterwards. The firmware logs its USB
link connected and mounted. It prints nothing about the car's writes in the 14 s of emulated time
looked at: what it counts feeds the archive windows, which need WiFi and the NAS. Its WiFi
bring-up then fails in ESP-Hosted's `esp_hosted_reconfigure` (`transport_drv_reconfigure()`
returns ESP_FAIL, no slave answering on slot 1) and retries every 30 s, serving the card to the
car meanwhile.

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
| Peripherals | UART0/1, USB-Serial/JTAG, systimer (16 MHz from XTAL), TIMG0/1, SHA, efuse (v1.0, block v0.3, security efuses zero), the flash controller (SPI0 cache side, SPI1 commands, a GigaDevice part answering RDID), the PSRAM controller with an AP Memory x16 PSRAM, the cache controller, HP_SYSTEM (cross-core interrupts, stall status), HP_SYS_CLKRST (divider update, PLL calibration done, core 1 clock and reset), LP_SYSTEM / LP_CLKRST / LP_TIMER (reset cause, software reset, core 1 boot address, RTC counter, the SDMMC reset), the analog I2C master (regi2c read-back), ASSIST_DEBUG's saved PC, the RNG, GPIO with the P4's register layout, the SDMMC host with its IDMAC and an SD card in the board's socket (`--sd`), the high-speed USB-OTG core as a device with buffer DMA and a host on its port (`--usb-host`); everything else is register RAM, listed by `--log-periph` |
| Time base | 360 MHz, the rev < 3 application clock; one instruction is one cycle |

The revision is a type parameter (`esp32p4::Revision`, `RevLt3`): rev >= 3 (its own ROM, the
`mintthresh` CSR, the L2 cache at the bottom of L2MEM) is a second implementation, not a change
to this one. The browser build boots the same images (`web/wasm/fw/p4-hello.json`, run by
`tools/wasm-test.mjs p4-hello`).

## Not there yet

- **ESP-Hosted, AXI-DMA, the LP core**: ESP-Hosted is milestone 4 of the plan, the others are
  out of scope. Xesppie's save and restore, which FreeRTOS runs only for a task that uses PIE.
- **USB**: the full-speed OTG core (0x50040000) and USB_WRAP, host mode, the core's slave mode
  (the data FIFOs), descriptor DMA, isochronous and interrupt endpoints, LPM, the
  per-endpoint interrupt source (94), and the time a transaction takes inside the core: it
  answers at once. The UTMI PHY is register RAM.
- **SD timing** is off by default: a command and its data are done in the register write that
  starts them. `--sd-timing` times the bus but not the card's read access time.
- **GPIO** has the P4's register layout (`periph::GpioP4`), but pin levels, interrupts and edges
  are the shared model's 49 pins; the P4 has 55.
- **`--boot app`** maps the image and jumps to it without the clocks and caches the bootloader
  would have set.
- **`Machine::run_until_cycle`** still schedules core 0 alone; nothing that drives the P4 needs it.
- **Unmodelled blocks** hello_world touches: PMU, PVT, APB_SARADC, LP_WDT, LP_ANA_PERI, LP_ADC,
  KEYMNG, UART2-4, TRACE0/1, LP_UART, LP_PERI, LP_GPIO and DSI, all configuration the firmware
  writes and reads back (`--log-periph` lists them). The watchdogs never fire.

## Handoff: where milestone 4 starts

**Status.** Milestone 3 is done: the high-speed DWC2 core as a device with buffer DMA, a host on
its port written in Rust (enumeration, SCSI over bulk-only transport, FAT32 written as Linux
writes it, a command file for the CLI), IDF's tusb_msc example presenting the card to it (golden
`usbmsc-p4`), and oxidrive enumerating to a car that records a clip which lands intact on the
image ("Compared with a board"). Archive windows, in which oxidrive takes the card from the car
and the car sees MEDIUM NOT PRESENT, need WiFi and the NAS and move to milestone 4; the car's side
of them is what the golden's eject already shows.

**Where oxidrive stops.** With the car attached (`external_oxidrive_presents_the_card_to_a_car`,
`OXIDRIVE_DIR=<images>`), the firmware brings WiFi up through ESP-Hosted, whose host side talks
SDIO to the C6 on slot 1 of the SDMMC host (CLK 18, CMD 19, D0-D3 14-17; `CONFIG_ESP_HOSTED_SDIO_SLOT=1`,
4 bits, 40 MHz). Its card initialisation on slot 1 goes CMD52 (an I/O reset of CCCR 0x06),
CMD0, CMD8, CMD5, then ACMD41 and CMD1, fifteen rounds of it in the first 14 s, every command
unanswered, and `esp_hosted_reconfigure` fails (`transport_drv_reconfigure()`: ESP_FAIL); the
firmware retries every 30 s and serves the card to the car meanwhile. `ESP_EMU_DEBUG=sdmmc` on
the test prints each command.

**What milestone 4 needs** ([esp32p4-plan.md](esp32p4-plan.md), "4. The co-processor"):

1. An SDIO function on slot 1: an `SdDevice` attached at slot 1's pins
   (`Peripherals::attach_sd`, "Slots and the socket") that answers CMD5 with an R4 (OCR, one I/O
   function), CMD3, CMD7, CMD52 on the CCCR and FBR (SD Specifications Part E1, SDIO Simplified
   Specification) and CMD53 byte and block transfers, and raises the SDIO interrupt on DAT1 (the
   trait carries it; the host's SDIO interrupt path in `sdmmc.rs` is not written yet). What sits
   behind it is the C6's SDIO slave controller as ESP-Hosted's slave firmware programs it.
2. The C6 co-processor: a second machine, `esp32c6`, running unmodified ESP-Hosted slave firmware
   built for it (ESP-Hosted 2.12.13 is what oxidrive links on the host side), with an SDIO slave
   model of the C6's HINF and SLC blocks (IDF v5.5.3 `soc/esp32c6/register/soc/sdio_hinf_reg.h`,
   `sdio_slc_reg.h`, `sdio_slc_host_reg.h`), the slave's
   reset on the host's GPIO 54, and the C6's WiFi joining the virtual access point as it already
   does.
3. Lock-step co-simulation: both machines stepped in emulated time the way `--cooja` steps
   802.15.4 nodes (`cli/src/cooja.rs`), SDIO transactions handed across at the cycle they start.
   The P4 is multi-core, so `Machine::run_until_cycle` (core 0 only) is not the tool; a run to an
   exact cycle through `max_cycles`, as `usb_host::UsbPort` does for the USB host, works on both
   cores.
4. Then the archive window: the NAS as an SMB server on the host behind the NAT, oxidrive taking
   the card from the car (TEST UNIT READY: MEDIUM NOT PRESENT), uploading a clip, giving it back,
   and the car's next writes landing intact.

**Timed SD transfers** are there (`--sd-timing`): with them oxidrive takes the car's writes at
11.0 MB/s where its board measured about 10.9 MB/s ("USB", "Throughput"). The card's read access
time and its programming time per block (`--sd-prg-us` is per write) are what is left; an archive
window's reads of the card on slot 0 and ESP-Hosted's traffic on slot 1 will share the one host,
so timing both slots matters there.
