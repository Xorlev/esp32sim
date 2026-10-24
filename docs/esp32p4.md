# ESP32-P4 (RISC-V) — draft

The fourth chip, and the first with two RISC-V cores: silicon **rev < 3** (v0.x, v1.x), as on the
Waveshare ESP32-P4-WIFI6 (v1.0) the plan targets ([esp32p4-plan.md](esp32p4-plan.md), milestone 1).
Unmodified ESP-IDF firmware boots from the real mask ROM, through the real 2nd-stage bootloader,
into FreeRTOS on one or both cores, reaches an SD card in the board's socket through the SDMMC
host (milestone 2, "SD card" below), presents it as a USB mass-storage device to a host on
the high-speed USB-OTG port, a car written in Rust (milestone 3, "USB" below), and reaches WiFi
through an emulated ESP32-C6 on SDIO running unmodified ESP-Hosted slave firmware, the two chips
in lock-step in one process (milestone 4a, "The co-processor" below). Through the C6 an
application reaches real servers on the host: oxidrive archives a car's clips to a samba share
and reports to a mosquitto broker in an `external_` test (milestone 4b, "Services" below).

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
[usb-host] t=0.459375 fat-write CLIP0001.MP4: 1048576 bytes in 16 chunks, 0.248 s, 4.22 MB/s, second half 4.35 MB/s; camera 4.10 MB/s, worst 1.9 ms late (chunk 0), kept up
[usb-host] t=0.502625 fat-read CLIP0001.MP4: 1048576 bytes, hash 208645dd266d37e6, 24.24 MB/s, matches what was written
[usb-host] t=0.504250 ejected
[usb-host] t=0.504875 TEST UNIT READY: sense 2/3a/00 (MEDIUM NOT PRESENT)
[usb-host] t=1.510750 TEST UNIT READY passed after 5 failed polls
[usb-host] t=1.511125 READ CAPACITY: 131072 blocks of 512 bytes (64 MiB)
[usb-host] t=1.513625 root: CLIP0001.MP4 (1048576), README.MD (379)
```

When the host configures the device, esp_tinyusb takes the card from the application and
exposes it; the eject (START STOP UNIT with LoEj) gives it back, the example's console writes
README.MD on it and `expose` hands it to USB again. The golden pins the console, the host's log
with its emulated times and the instruction count, then reads the clip, README.MD and the raw
blocks the car wrote back from the image. A second golden, `usbmsc-p4-faults`
(`examples/usb_msc-p4/faults.txt`), has the same example meet the faults a car meets
("Faults").

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
lp_clkrst_reg.h) holds the core in reset while it is set: the registers go to their reset values
when it is asserted and ignore writes until it is released. `usb_new_phy` pulses it through
`_usb_utmi_ll_reset_register`; a chip reset keeps the cable. Its DMA reaches L2MEM and PSRAM like
the SDMMC IDMAC (`SOC_PSRAM_DMA_CAPABLE`; IDF's own host driver puts DWC DMA memory in PSRAM on
the P4, components/usb/hcd_dwc.c:50).

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
| DIEPCTLn, DOEPCTLn | MPS, USBAEP, DPID, NAKSTS, EPTYP, STALL, TXFNUM; EPENA is set by software and cleared by the core at the end of a transfer and at SETUP done; EPDIS disables an enabled endpoint once it NAKs, an IN endpoint with NAK set (SNAK, INEPNE), an OUT endpoint under the global OUT NAK (SGONAK, BOUTNAKEFF), the order dcd_dwc2's `edpt_disable` follows: until then EPDIS reads 1 and the endpoint stays enabled, then EPENA and EPDIS clear with EPDISD; SNAK (and INEPNE), CNAK, SD0PID, SD1PID; control endpoint 0 takes DIEPCTL0's packet size both ways, a SETUP clears its STALL and sets NAK on both, control OUT 0 cannot be disabled |
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
- `UsbLink::inject(at_ns, LinkFault::VbusOff)` drops VBUS at a set time whatever the host is
  doing then: a transfer in flight ends `Disconnected` and the core sees a session end (SEDET),
  which TinyUSB reports as an unplug.

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
- `reset_device`: Linux's `usb_reset_device` (v6.12 drivers/usb/core/hub.c,
  `usb_reset_and_verify_device` at 6090 and `hub_port_init`'s new scheme at 4985-5034): reset,
  GET_DESCRIPTOR(64) at address 0, reset again, SET_ADDRESS to the old address, the device and
  configuration descriptors read and compared with the old ones, SET_CONFIGURATION with the old
  configuration; 50 ms after each reset (TRSTRCY "plus some extra", hub.c:3112-3113), 10 ms
  after SET_ADDRESS.
- `msc::Msc`: SCSI over bulk-only transport (BOT 1.0): CBW and CSW with tags, the CSW checked
  for validity and meaning (§6.3), a STALLed data stage cleared and the CSW read, a STALLed CSW
  cleared and read once more, REQUEST SENSE after CHECK CONDITION; INQUIRY, TEST UNIT READY, READ
  CAPACITY(10), MODE SENSE(6), PREVENT ALLOW MEDIUM REMOVAL, START STOP UNIT, READ(10) and
  WRITE(10) of any length in commands of 240 blocks at most (usb-storage's `max_sectors`,
  scsiglue.c:670), SYNCHRONIZE CACHE(10); `wait_ready` polls TEST UNIT READY as a car polls its drive.
  A CHECK CONDITION whose sense is UNIT ATTENTION marks the device changed (`Msc::changed`, Linux's
  `sdev->changed` for a removable device: v6.12 drivers/scsi/scsi_lib.c:803-811 for a read or
  write, 2391-2399 for TEST UNIT READY); while it is set the host refuses READ(10) and WRITE(10)
  itself (`MscError::Changed`, sd's "device offline or changed", sd.c:1340-1343) until the next
  media poll takes the change (`take_changed`, sd.c:1773-1774). `wait_ready` clears it: its caller
  reads the drive afresh.
  A command may take 30 s (sd's SD_TIMEOUT, drivers/scsi/sd.h:15, set at sd.c:3970-3972) before it
  is aborted. The LUN is a `fat32::BlockDevice`. Recovery (`Msc::recovery`) is Linux's by default,
  from v6.12 drivers/usb/storage/transport.c:
  - a transport error (a CBW that fails, a phase error, a CSW with the wrong tag or a status past
    2, a CSW still bad after the second read) or a command aborted at its timeout goes to
    `Handle_Errors` (transport.c:903-926): a port reset first (`usb_stor_port_reset`,
    `reset_device` above), the class reset only if that fails (`usb_stor_Bulk_reset`,
    `usb_stor_reset_common` at 1347: the request with a 5 s timeout, 6 s for the device to
    recover, then the halt cleared on bulk IN and bulk OUT);
  - a data stage that runs long (babble) with a good status gives a faked INVALID FIELD IN CDB
    (`fake_sense`, 1186 and 1305-1310), and the CSW is still read;
  - a zero-length packet where the CSW should be is read past once (1229-1233);
  - a residue past the transfer length is clamped to it (1298).

  `Recovery::Bot` (`recovery bot`) is BOT 1.0's reset recovery for all of these (Bulk-Only Mass
  Storage Reset and CLEAR_FEATURE(ENDPOINT_HALT) on both endpoints, §5.3.4). What Linux's SCSI
  layer adds on top is not modelled: the retries of a failed command (sd's `max_retries`,
  SD_MAX_RETRIES 5) and the error handler's escalation past the abort. The review that asked for
  this described Linux's timeout handling as the class reset first, then a port reset; the
  v6.12 source does the port reset first, and the class reset only when the port reset fails.
- `fat32::Fat32` mounts a volume (an MBR's FAT32 partition, or from LBA 0), lists, reads and
  writes 8.3 files in the root directory in a vfat-like order: the directory entry at once, the
  data a chunk at a time as it comes (one WRITE(10) per contiguous run of clusters), the FAT
  sectors of each chunk's clusters in both FATs, then the entry's size and first cluster and
  FSInfo. A clip cut off mid-write in this order keeps size 0 and no cluster, its clusters
  orphaned in the FAT. Allocation starts at FSInfo's hint and writes the last cluster allocated
  back as the hint; the free count written is FSInfo's as it stands on the volume less what this
  host allocated since and plus what it freed. `FileWriter::fat_every` writes the FAT less often,
  and `FileWriter::linux` rewrites the entry and FSInfo after every chunk, as Linux's inode
  writeback finds them dirty. The metadata (boot sector, FSInfo, FAT, directories) goes through
  `BlockDevice::read_meta` and `write_meta`, which a plain device answers as blocks, so a cache
  underneath can tell it from data. `delete_path` unlinks as vfat does (the entries marked
  deleted, then the chain freed in both FATs, then FSInfo), `dirty`/`set_dirty` keep the volume's
  dirty flag, and `fsck` checks and repairs the volume after dosfstools 4.2 ("The car's hazards").
- `pagecache::PageCache` is the car's buffer and page cache between its `Fat32` and the drive:
  metadata kept until the medium is seen to change, or writeback ("The car's hazards", "The
  writeback model"). Plain, it passes everything through. A page the drive refuses to take is
  lost and counted (`write_errors`), not returned as the error of whatever woke the flusher, as
  Linux keeps a writeback error for fsync (include/linux/pagemap.h:238-248 `mapping_set_error`).
- `car::Car` records clips (and sets of camera clips) as cameras feed them, polls TEST UNIT READY
  between writes as Linux's sd driver does, mounts, unmounts, loses power, runs fsck, and keeps
  what it believes each file holds. A medium change (NOT READY then ready, a UNIT ATTENTION at a
  poll, or one on a read or write since the last poll) revalidates as `sd_revalidate_disk` does
  (sd.c:3698-3763): READ CAPACITY, MODE SENSE (a failure taken as write enabled, as
  `sd_read_write_protect_flag` takes it), then the volume read again.
- `fat32::write_clip` records a clip as a camera feeds it: chunk i is due at t0 + i x gap (t0
  taken before the file is created), the car waits only for a chunk that is not due yet, and the
  result (`ClipStats`) has the rate overall and over the second half, the worst lateness against
  the schedule and whether the car kept up (never a whole chunk behind).
- `script::Runner` runs a command file (`--usb-host`), each command logging what it did. A
  NOT READY or UNIT ATTENTION seen by `ready` or `poll` drops the mounted volume, and a failure
  that leaves no device on the port drops the interface and the volume, so the next `mount` reads
  the volume again. The CLI runs it under the run's `--max-insns` and reboot policy
  (`usb_host::run_machine_until`).

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
data stages, STALLs, phase errors, wrong tags, NAKs, a zero-length packet before the CSW, an
endpoint that stops answering) under both recovery policies, and for the camera's pacing;
`esp-periph/src/usb_otg/tests.rs` drives the core with TinyUSB's register sequences against the
link, including control writes of one and two packets (the status stage is DATA1, §8.5.3), an
endpoint disable waiting for its NAK, the reset line held, and an injected VBUS loss.

**Throughput.** A car writing in 64 KiB chunks as fast as the drive takes them, then reading the
clip back (FAT32, the card's programming time 0). The second half is the rate from the middle
chunk to the close. The large clips are thousands of times either firmware's buffer, and their
second half runs at the rate of the whole and of the 4 MiB clip, so these are steady-state rates:

| firmware | SD bus | clip | write | second half | read |
| --- | --- | --- | --- | --- | --- |
| tusb_msc example (8 KiB MSC buffer) | at once | 4 MiB | 9.1 MB/s | | 24.8 MB/s |
| tusb_msc example | at once | 32 MiB | 9.2 MB/s | 9.2 MB/s | |
| tusb_msc example | `--sd-timing`, 4 bits at 20 MHz | 4 MiB | 4.4 MB/s | | 6.8 MB/s |
| tusb_msc example | `--sd-timing` | 32 MiB | 4.4 MB/s | 4.4 MB/s | |
| oxidrive (32 KiB buffer) | at once | 4 MiB | 19.7 MB/s | 19.6 MB/s | |
| oxidrive | at once | 64 MiB | 20.0 MB/s | 20.0 MB/s | |
| oxidrive | `--sd-timing`, 4 bits at 40 MHz | 4 MiB | 11.0 MB/s | 11.0 MB/s | |
| oxidrive | `--sd-timing` | 64 MiB | 11.1 MB/s | 11.1 MB/s | |

Against a camera, 64 KiB due every 16 ms (4.1 MB/s, `fat32::write_clip`), both keep up with the
SD bus timed or not: oxidrive over 4 MiB and 64 MiB clips (the 64 MiB one takes 16.37 s, the
camera's schedule plus the last chunk), the tusb_msc example over 1 MiB (the golden) and 16 MiB.
The latest chunk starts 1.9 ms after its due time, and it is the first, behind the directory
entry. With the bus timed the example keeps up with a 7% margin (4.4 against 4.1 MB/s), oxidrive
with 2.7 times. Earlier versions of this page gave 3.5 MB/s for oxidrive against "a camera at
4 MB/s": the car then slept a whole gap after each chunk instead of waiting for the next chunk's
due time, so that figure measured the pacing, not the drive.

oxidrive's board measured about 10.9 MB/s writing over USB. That the timed bus gives 11.0 MB/s is
a coincidence, not a calibration. The 4 MiB clip takes 0.213 s with the card instant: USB and the
CPU, at one instruction a cycle. Its SD wire time alone is about 0.216 s (8192 blocks at 4 bits
and 40 MHz, 1051 clocks a block: start bit, 1024 data clocks, CRC16, end bit, the CRC status
token, two clocks between blocks; plus the commands). Timed, the clip takes 0.381 s, so the two overlap by only about 0.048 s: the
card write runs after each buffer arrives, in the TinyUSB task (esp_tinyusb defers it there,
tinyusb_msc.c:351-388), and only the next buffer's DMA proceeds alongside it. Two things the
model leaves out would each slow it: the card's programming time is 0 (`--sd-prg-us`,
`SdCardConfig::prg_ns`), and cache misses and PSRAM stalls are not modelled. Before the rate is
compared with a board again, `--sd-prg-us` wants a value measured on a card: the busy time after
a multi-block write.

The USB port and the timed SD bus cost the other chips nothing and the P4 nothing while unused:
the core is an optional device-table entry, the bus tests one flag per round for the link and
one for a timed SD bus, whose work and deadline sit in cold functions (inlined, they cost the P4
hello demos 0.9-1.0%). Measured against 6bc9b9cf with `cargo build --release --bins`,
alternating runs, median `user` time, identical instruction counts; the P4 rows on the tree at
4b57750e, the others on the tree before the timed SD bus, which only the P4 runs:

| demo | runs | 6bc9b9cf | this branch | instructions |
| --- | --- | --- | --- | --- |
| S3 `hello` (30 s) | 7 | 0.39 s | 0.39 s | 11,990,488 + 6,798,360 |
| `c3-hello` (30 s) | 7 | 3.96 s | 4.00 s | 4,800,000,000 |
| `c6-hello` (30 s) | 7 | 5.82 s | 5.75 s | 4,800,000,000 |
| `p4-hello` (27 s) | 7 | 18.16 s | 18.09 s | 9,720,000,000 + 0 |
| `p4-hello-smp` (27 s) | 7 | 19.18 s | 19.18 s | 9,720,000,000 + 9,713,148,105 |
| pocket-tank (8 s) | 5 | 14.47 s | 14.42 s | 1,113,634,993 + 1,490,529,616 |

The C3 and C6 run no code this branch changed; their ±1% is layout. The review fixes after it
(the endpoint disable, the reset line, recovery, pacing, faults) change no per-round work: the
core's paths run only with a host attached, which no demo has, and the reset line is a write to
one LP_CLKRST register. A spot check on the final tree, five alternating runs each: `p4-hello`
18.16 s against 18.04 s, `p4-hello-smp` 19.18 s against 19.23 s, instruction counts identical.

**Faults.** `examples/usb_msc-p4/faults.txt` (golden `usbmsc-p4-faults`) has the tusb_msc
example meet three faults; the example logs each hand-over of the card ("Storage mounted to
application: No" when the car has it, "Yes" when the application has it), and the golden pins the
sequence:

| fault | what the car does | what the example does |
| --- | --- | --- |
| a WRITE(10) of 240 blocks with a 5 ms command timeout | aborts it mid data stage, resets the port (Linux's recovery), writes the blocks again and verifies them | nothing visible but one line: `proc_write10_cmd 910: ASSERT FAILED` (below); the next commands work |
| a port reset after an eject | `port-reset`: two resets, the descriptors compared, SET_CONFIGURATION | takes the card back from the application: "No" |
| VBUS lost 100 ms into a 1 MiB clip | the write fails `Disconnected`; the interface and the volume are dropped; `attach` again | gives the card to the application on the unplug ("Yes"), takes it again on the new SET_CONFIGURATION ("No") |

The clip cut by the VBUS loss is on the card as an entry of size 0 and no cluster, its clusters
allocated in the FAT (the golden checks both): the vfat-like writer's failure mode, not Linux's.

A third golden, `usbmsc-p4-window` (`examples/usb_msc-p4/window.txt`, the SD bus timed, the card
programming for 1.5 ms), has the car keep writing after it ejected: the application holds the card
and writes README.MD, TEST UNIT READY answers MEDIUM NOT PRESENT, and four WRITE(10)s of 16 blocks
are answered GOOD all the same, read back by READ(10) and on the card afterwards. esp_tinyusb's
READ(10) and WRITE(10) callbacks serve the card whoever holds it (2.4.0 here; oxidrive's 2.2.1
copy had the same callbacks until its window gate, teslausb-p4 5633fc2), which is what "The car's
hazards" meets with oxidrive 728b71f.

What the sources say about the hand-overs (TinyUSB 0.21.0~2, esp_tinyusb 2.4.0):

- SET_CONFIGURATION with a non-zero value calls `tud_mount_cb` (usbd.c:1006-1012). A bus reset
  alone does not unmount (usbd.c:717-721); an unplug calls `tud_umount_cb` (723-727).
- esp_tinyusb's `tud_mount_cb` runs `msc_storage_mount_to_usb` (tinyusb.c:62-65), which unmounts
  the card from the application unless `auto_mount_off` is set (tinyusb_msc.c:776-790);
  `tud_umount_cb` runs `msc_storage_mount_to_app` (tinyusb.c:86-89, tinyusb_msc.c:760-774).

So a port reset, which Linux's usb-storage does to recover from any transport error or timeout,
takes the card from an application that is using it, unless the firmware sets `auto_mount_off`.

The assertion after the aborted write reads, from the sources, as a race in the stack rather than
in the model: dcd_dwc2's bus reset handler closes the endpoints in the interrupt (it clears
`xfer_status`, dcd_dwc2.c:775) while the TinyUSB task still holds the write's last transfer
completion; asking for the next chunk then finds bulk OUT closed (`dcd_edpt_xfer`,
dcd_dwc2.c:687-688) and msc_device.c:910 asserts. The bus reset event that follows resets the
class (`mscd_reset`). A 5 ms timeout cuts a live transfer, which is what opens the window; Linux's
30 s timeout aborts a device that has stopped. Seen only in emulation; not checked on silicon.

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

## The co-processor

The P4 has no radio. On the reference board an ESP32-C6 sits on SDIO slot 1 and runs ESP-Hosted's
slave firmware; the P4's application calls `esp_wifi_*` as usual, and esp_wifi_remote and
ESP-Hosted's host driver carry the calls and the packets over SDIO. `--coproc` puts an emulated C6
there, running the unmodified slave, and runs both chips in one process ([cli.md](cli.md),
"Co-processor (P4)"). IDF v5.5.3's WiFi station example with esp_hosted 2.12.13 and ESP-Hosted
2.12.13's slave (`examples/esp-hosted-p4`, golden `hosted-p4`) join the virtual access point
through the C6's radio:

```sh
./target/release/esp32sim-p4 --flash-mb 2 --bootloader $FW/p4-hosted-bootloader.bin \
    --ptable $FW/p4-hosted-ptable.bin --app $FW/p4-hosted-station.bin \
    --coproc c6:bootloader=$FW/c6-hosted-bootloader.bin,ptable=$FW/c6-hosted-ptable.bin,app=$FW/c6-hosted-slave.bin,otadata=$FW/c6-hosted-otadata.bin,stub=0x420bac5c=0 \
    --wifi ssid=esp32sim,psk=esp32sim-pass --net none --max-seconds 4
```

```
W (81) H_SDIO_DRV: Reset slave using GPIO[54]
[c6] ESP-ROM:esp32c6-20220919
[c6] I (115) co-pro-main:                 ESP-Hosted-MCU Slave FW version :: 2.12.13
[c6] I (115) SDIO_SLAVE: sdio_init: sending mode: SDIO_SLAVE_SEND_STREAM
I (1601) sdio_wrapper: SDIO master: Slot 1, Data-Lines: 4-bit Freq(KHz)[40000 KHz]
I (1621) sdio_wrapper: Function 1 Blocksize: 512
I (1721) H_SDIO_DRV: Card init success, TRANSPORT_RX_ACTIVE
I (1771) transport: Identified slave [esp32c6]
I (1771) transport: capabilities: 0xd
[c6] W (2728) slave_wifi_std: Sta mode connected
I (2771) RPC_WRAP: ESP Event: Station mode: Connected
I (3771) wifi station: got ip:10.0.2.15
```

The P4 holds the C6's EN low for 10 ms on GPIO 54 and waits 1.5 s
(CONFIG_ESP_HOSTED_SDIO_RESET_DELAY_MS) while the C6 boots again; the card comes up (CMD52 I/O
reset, CMD0, CMD8, CMD5 twice, CMD3, CMD7, then CMD52 for the function, its interrupt enables,
four lines, high speed and both block sizes), the host opens the data path with a slave
interrupt, and the slave answers with its INIT event: the chip, the capabilities and its firmware
version. The host compares that with its own and prints `Version mismatch` only when the
major.minor differ (transport_drv.c, `compare_fw_version`); it prints nothing here, 2.12 against
2.12. The station's RPCs follow, the C6 joins the access point, and the P4's lwIP takes a lease
through the link 3.77 s after power-up. The golden pins both consoles in the order the slices
produced them, the P4's instruction count, the C6's run and its SDIO, WiFi and network counts.
19 frames go by before the C6's driver has given its MAC a receive ring (beacons before the
radio is on; the report counts them apart from frames that find no free descriptor, of which
there are none). Co-simulating costs about a second of CPU per emulated second while the
station idles: the golden's 4 s take 3.6 s, 8 s take 7.7 s (both chips, one host thread).

### The slave

`esp32c6/src/sdio.rs` models the three blocks IDF v5.5.3's sdio_slave driver programs
(`esp_driver_sdio/src/sdio_slave.c`, `hal/sdio_slave_hal.c`, `hal/esp32c6/include/hal/sdio_slave_ll.h`;
registers: `soc/esp32c6/register/soc/sdio_hinf_reg.h`, `sdio_slc_reg.h`, `sdio_slc_host_reg.h`),
HINF at 0x60016000, SLC at 0x60017000 and SLCHOST at 0x60018000 (`reg_base.h:35-37`), as one
optional device: the SLC entry raises source 64, `ETS_SLC0_INTR_SOURCE` (`interrupts.h:86`), and
HINF and SLCHOST are aliases of it. SLC's "rx" link carries slave-to-host data, its "tx" link
host-to-slave data, the driver's sending and receiving. A C6 run without a host costs the table
entries and one address compare after a peripheral write.

| block, register | modelled |
| --- | --- |
| HINF CFG_DATA0 0x00 | DEVICE_ID_FN1 0x6666, USER_ID_FN1 0x0092 at reset; the CIS's MANFID |
| HINF CFG_DATA1 0x04 | SDIO_ENABLE (the card answers nothing without it), IOREADY1/2 (CCCR I/O Ready), HIGHSPEED_ENABLE (CCCR SHS), SDIO_INT_MASK, SDIO_VER 0x232 at reset; the read-only HIGHSPEED_MODE, IOENABLE1/2 and CD_DISABLE show the host's CCCR |
| SLC SLCCONF0 0x00 | TX_RST and RX_RST reset a link; RX_AUTO_WRBACK writes a sent descriptor back with the owner cleared |
| SLC SLC0INT_RAW/ST/ENA/CLR 0x04-0x10 | FRHOST_BIT0-7 from the host, TX_OVF, TX_DONE, TX_SUC_EOF, RX_DONE, RX_EOF; ST is RAW & ENA, the SLC0 line |
| SLC SLC0RX_LINK, SLC0RX_LINK_ADDR 0x3C, 0x40 | START reads the descriptor at once and raises RX_DONE when the hardware owns it (the driver polls RX_DONE right after); STOP |
| SLC SLC0TX_LINK, SLC0TX_LINK_ADDR 0x44, 0x48 | START, STOP; RESTART follows the next pointer of the last buffer closed |
| SLC SLCINTVEC_TOHOST 0x5C | bits 7:0 raise SLCHOST's TOHOST_BIT0-7 |
| SLC SLC0TOKEN0/1 0x60, 0x64 | WR, INC, INC_MORE; the count at 27:16 and at SLCHOST TOKEN_RDATA |
| SLC SLC0_LEN_CONF, SLC0_LENGTH 0xF4, 0xF8 | LEN_WR, LEN_INC, LEN_INC_MORE set the cumulative length SLCHOST PKT_LEN shows (20 bits, wrapping); the first length write after a START on a packet raises the host's RX_NEW_PACKET |
| SLCHOST TOKEN_RDATA 0x44 | TOKEN1 at 27:16, TOKEN0 at 11:0 (TOKEN1 can be held for a fault test) |
| SLCHOST SLC0HOST_INT_RAW/ST 0x50, 0x58 | TOHOST_BIT0-7, RX_UDF, TX_OVF, RX_NEW_PACKET; ST is RAW & FUNC1_INT_ENA |
| SLCHOST PKT_LEN 0x60 | the length in 19:0; LEN_CHECK (31:20) reads 0 |
| SLCHOST CONF_W0-W15 0x6C-0xB8 | shared scratch, both sides; CONF_W7's low byte is the host's interrupt to the slave (FRHOST_BIT0-7) and reads 0 |
| SLCHOST SLC0HOST_INT_CLR, FUNC1_INT_ENA 0xD4, 0xDC | write 1 to clear; the interrupt enable |
| the rest | register RAM (timing, check sums, the windows, SLC1, function 2) |

The card the host sees follows the SDIO Simplified Specification v3.00 (SD Specifications Part
E1, 2011):

| command | the card |
| --- | --- |
| CMD5 | R4 (§3.3): C set, two I/O functions, no memory, I/O OCR 2.7-3.6 V; answered only before an RCA is assigned, and with argument 0 it does not start initialization (§3.1.2); nothing else is answered before a CMD5 since power-up or the I/O reset |
| CMD3 | R6 (§4.3): RCA 0x0001, the SDIO-only status bits zero |
| CMD7 | with the RCA: selected, R1b; with another: deselected, no answer (§6.14) |
| CMD52 | R5 (§5.2.1): IO_CURRENT_STATE CMD, the data; read after write (RAW); FUNCTION_NUMBER for a function past two |
| CMD53 | R5 with IO_CURRENT_STATE TRN (§5.3): bytes (count 0 is 512) or blocks of the function's block size, incrementing or fixed address; infinite block counts are not modelled |
| CMD0, CMD8, ACMD41, CMD1 and the rest | no answer: an I/O-only card ignores memory commands (§4.4: CMD0 does not reset the I/O part) |

Function 0 is the CCCR (§6.9 Table 6-1: revisions from SDIO_VER, I/O Enable, I/O Ready, Int
Enable, Int Pending, I/O Abort with RES and the abort select, Bus Interface Control, Card
Capability 0x17, the common CIS pointer, FN0 block size, Bus Speed Select with SHS and EHS), the
FBR's block sizes (§6.10) and the common CIS at 0x1000: DEVICE, MANFID 0092/6666, FUNCID, FUNCE,
CONFIG, CFTABLE_ENTRY and END, the tuples IDF's `sdmmc_io_print_cis_info` printed for an ESP32-C6
slave (esp_hosted 2.12.13 `docs/esp32_p4_function_ev_board.md`). The I/O reset (§4.4) clears the
function enables, interrupt enables, bus width, high speed and block sizes and takes the card
back to needing CMD5; the SLC side is the C6's and stays as its driver left it. Function 1 is
Espressif's protocol (IDF v5.5.3 `docs/en/api-reference/protocols/esp_sdio_slave_protocol.rst`):
addresses below 0x400 reach SLCHOST's registers a byte at a time (ESP-Hosted and
esp_serial_slave_link mask register addresses with 0x3FF), and above them is the FIFO, where a
transfer to address A carries a packet of 0x1F800 - A bytes:

- **Slave to host.** The driver's send link is a chain of descriptors with EOF on the last
  (stream mode collects every queued packet). A host read takes the packet's bytes from the
  descriptors in SRAM, zero-fills past the packet (ESP-Hosted reads whole 512-byte blocks), hands
  each descriptor back with RX_DONE and the last with RX_EOF, which is when the driver returns
  the buffers and starts the next chain. Reading more than the link holds is an underflow
  (RX_UDF, zeros, and the CMD53 succeeds). Starting the link on a descriptor the hardware owns
  raises the slave's RX_DONE at once: the driver starts a one-byte descriptor and waits for that
  RX_DONE (`send_isr_invoker_enable`). The host's RX_NEW_PACKET waits for the length the driver
  writes right after the START (`send_new_packet`): raised at the START, ESP-Hosted's combined
  register read (INT_RAW to PKT_LEN in one CMD53) could land between the two, see a packet with
  the old length, clear the interrupt and wait for ever. `SdioStats::stale_new_packets` counts host
  reads of INT_RAW that show RX_NEW_PACKET with PKT_LEN where the host last read it (an upper
  bound: a host PKT_LEN poll in between also counts); the report names it when it is not zero.
  A CMD52 into the FIFO is a one-byte transfer to address A: a read takes the packet's next
  byte, a write is a byte of a packet of 0x1F800 - A bytes and closes it at the last address.
- **Host to slave.** The driver loads receive buffers (START for the first, the tail's next pointer
  and RESTART for the rest, TOKEN1 counting them; the host subtracts the buffers it has used). A
  host write fills the current buffer, spills into the next, and closes the last with its length
  and EOF when the packet's 0x1F800 - A bytes are in; the block padding is dropped. A packet may
  span two CMD53s (a block, then the remainder at the incremented address). With no buffer the
  bytes are lost and TX_OVF goes to both sides.
- **When the C6 sees it.** The bytes cross the bus in the host's register write that runs the
  transfer, but what the C6 sees of it, each descriptor handed back (owner, length, EOF) and its
  RX_DONE, RX_EOF, TX_DONE and TX_SUC_EOF, waits for the data phase's end on the bus: with
  `--sd-timing` its wire time after the command, and when the P4's IDMAC resumes a suspended
  transfer (a read longer than IDF's four descriptors) the moment of the resume, which the C6,
  behind the P4 then, has not reached. The slave queues the hand-back and the co-simulation stops
  the C6 at that cycle to apply it. Until then the buffers are the hardware's, so moving the bytes
  early is invisible to the driver. Untimed, at a command the C6 already stands at the command's
  time, so nothing waits.
- **Interrupts.** The slave raises the host's TOHOST bits through SLCINTVEC_TOHOST; DAT1 goes low
  while SLC0HOST_INT_ST is not zero, IENM and IEN1 are set in the CCCR (§6.3) and HINF does not
  mask it. The host's byte in CONF_W7 raises FRHOST_BIT0-7 on the slave, as ESP-Hosted's "open
  data path" interrupt does.

`--debug sdio` prints the slave's commands, the descriptors handed over and the I/O reset. The
I/O reset (§4.4) leaves SLCHOST's counters (TOKEN1, PKT_LEN) to the C6's driver: Espressif's
driver resets them itself and says the host has to ask it to (`sdio_slave.c`, header comment),
which ESP-Hosted's slave does on the host's reset interrupt (`sdio_reset`, `sdio_slave_reset`).
Guesses, each from the sources' silence:

- the CCCR revision bytes as SDIO_VER's split (0x32, 0x02), the card capability (0x17), two
  functions in R4, CMD5's C bit set at once, and the FN block sizes at 512 after a reset (the
  protocol document's default; the specification says 0);
- PKT_LEN's LEN_CHECK as 0, INT_ST under FUNC1_INT_ENA rather than SLC0HOST_INT_ENA, the 0x400
  split between the register window and the FIFO;
- RX_DONE at a link START on an owned descriptor, and the host's RX_NEW_PACKET at the length write
  that follows it;
- a read past the send link: zeros, RX_UDF, and a CMD53 that succeeds; silicon may stall the bus
  and retry instead (SLCHOST's SLC0_HOST_RD_RETRY_INT, SLC's SLC0_RD_RETRY_THRESHOLD);
- a write with no receive buffer: the bytes dropped, TX_OVF to both sides, and a CMD53 that
  succeeds (a host is expected to wait for TOKEN1 and never get there);
- the I/O reset: what it does to SLCHOST's INT_RAW and CONF_W registers is not documented; they are
  left as they are, like the counters;
- the card answers from power-up whatever the C6's IO MUX, its PCR clock gate or its ROM is doing:
  a host that sends CMD5 while the C6's ROM boots is answered here, which silicon may not do.

Not modelled: the card's busy and read wait, its
CRC (it never answers with a CRC error or a malformed response), suspend and resume, function 2,
the second SLC (SLC1), the host's token registers (TOKEN_WDATA, TOKEN_CON), the PCR reset of the
block (`PCR_SDIO_SLAVE_CONF.RST_EN`, which the driver pulses before it configures a block already
at its reset values) and the pins (the card answers however the C6's IO MUX is set).

### The host side

An `SdDevice` says whether it is an SDIO function (`io`). After each command the P4 samples the
function's DAT1: the slot's D1 pin (15) reads low while the function interrupts, which is what
IDF's `sdmmc_host_io_int_wait` looks at first, and RINTSTS's SDIO bit for the slot (16 + slot)
is set when DAT1 goes low, edge triggered as that function's comment describes it
(`esp_driver_sdmmc/src/sdmmc_host.c`). The ISR masks it and wakes the waiting task. The `card_int`
and card-detect inputs IDF ties high are not consulted. Nothing of this runs while only a memory
card is on the bus. The C6's EN follows GPIO 54 (`GpioP4::drive`; ESP-Hosted's
`ESP_HOSTED_SDIO_GPIO_RESET_SLAVE`, 54 on the P4, active high: low resets), read from OUT1 and
ENABLE1, which the W1TS and W1TC registers reach for every pin to 56; the IO MUX function of pins
49-56 is not checked.

### Lock-step

`cli/src/cosim.rs` runs a P4 machine and a C6 machine on one clock, nanoseconds from power-up
(the P4 at 9 cycles per 25 ns, the C6 at 4). Slices of 10 µs (`slice-us`) end on a fixed grid:

1. the P4 runs to the slice's end (`max_cycles`, both cores);
2. the C6 runs to the same instant (`run_until_cycle`);
3. the lines between the chips are exchanged: the slave's DAT1 into the P4 (D1 and RINTSTS), the
   P4's EN line into the C6.

The SDIO bus is host-driven, so the transactions are where the chips meet exactly: when the P4's
host starts a command, the C6 first runs to that nanosecond, then the command and its data are
carried out against the slave as it stands, the descriptors read and written in the C6's SRAM;
the C6 goes on from there in its next run. The C6 is never ahead of the P4. What the P4 senses
without a command, the slave's interrupt, is sampled after every command and at every slice end,
so it reaches the P4 at most one slice late, as a frame reaches csim's medium at the end of the
slice it started in (`--cooja`). EN is read at slice ends only, so it takes effect up to a slice
late: driven low, the C6 runs to that instant and stops; released after at least the C6's t_RST
(50 µs, ESP32-C6 Series Datasheet, "Chip Power-up and Reset"), the C6 powers up from its ROM (a
chip reset with cause POWERON; its LP domain and SRAM are kept, which a real power-up would not
keep), and its clock goes on from that instant. A shorter pulse is ridden out: the C6 runs on
from where it stopped. The restarted P4 makes GPIO 54 an output a moment before it writes the
level, and a slice end can catch that as a one-slice pulse; the report counts such pulses. At
power-up EN is the pull-up's: the C6 boots with the P4, as on the board, until ESP-Hosted resets
it. A chip that resets itself comes back through its ROM; a P4 reset keeps the SD bus's time
(the device clock starts again at a chip reset, `Peripherals::cycle_base` carries the cycles
before it), so the C6 still meets every command at its nanosecond, and the report counts
commands that ever find the C6 ahead (none should). The co-simulation reads no host clock and no
host randomness and the order of every step is fixed, so a run is the same to the instruction
every time, and the same however `run_until` is called
(`the_pair_runs_the_same_whichever_way_it_is_driven` runs the pair for 2 s whole and in uneven
pieces). The virtual network's SNTP server answers its own clock plus emulated time; `--net
none,epoch=S` fixes that clock, and without it the host's clock at the first answer sets it
([networking-howto.md](networking-howto.md)), so a firmware that asks for the time, as oxidrive
does, is reproducible only with an epoch. The goldens and tests pass one. Both consoles are
collected a line at a time and printed in the order the slices produced them. The co-processor
is native only: the browser build has none.

**Cost.** Without `--coproc` nothing of this runs: the C6's slave is an optional device-table
entry and one address compare after a peripheral write, the P4's SDIO path one flag tested where
the SDMMC host runs (`has_sdio`), the fault hooks and the deferred hand-back live in the
co-simulation, and the rest is in cold functions. Measured against 59c1d1cd with `cargo build
--release --bins`, alternating runs, median `user` time, identical instruction counts:

| demo | runs | 59c1d1cd | this branch | instructions |
| --- | --- | --- | --- | --- |
| S3 `hello` (30 s) | 7 | 0.39 s | 0.39 s | 11,990,488 + 6,798,360 |
| `c3-hello` (30 s) | 7 | 3.94 s | 3.96 s | 4,800,000,000 |
| `c6-hello` (30 s) | 7 | 5.72 s | 5.74 s | 4,800,000,000 |
| `c6-wifi-station` (14 s, the golden's run) | 5 | 3.10 s | 3.11 s | 2,240,000,000 |
| `p4-hello` (27 s) | 7 | 18.20 s | 18.15 s | 9,720,000,000 + 0 |
| `p4-hello-smp` (27 s) | 7 | 18.71 s | 18.60 s | 9,720,000,000 + 9,713,148,105 |
| pocket-tank (8 s) | 13 | 14.15 s | 14.29 s | 1,113,634,993 + 1,490,529,616 |

The S3 and C3 run no code this branch changed, so their differences are layout: pocket-tank's
13 runs are two sets, base first (14.14 s against 14.45 s, 5 runs) and this branch first (14.18
s against 14.28 s, 8 runs). The review changes after it (the SD bus's time base, the deferred
hand-back, the fault hooks, the WiFi drop counts, the network clock) touch no per-instruction
path; a spot check on the final tree, five alternating runs each: `c3-hello` 3.85 s against
3.82 s, `c6-wifi-station` 3.00 s against 3.02 s, `p4-hello` 17.28 s against 17.40 s (+0.7%, the
runs within 0.2% of each other), `p4-hello-smp` 18.43 s against 18.38 s, instruction counts
identical.

### Throughput

ESP-Hosted's own transport test (RawTP) moves 1460-byte packets as fast as the link takes them
and reports the rate every second on both consoles. Its images are committed beside the pair's
(`p4-hosted-rawtp-rx.bin`, `p4-hosted-rawtp-tx.bin`, `c6-hosted-rawtp-slave.bin`; the Kconfig
recipe is in `examples/esp-hosted-p4/README.md`), and the goldens `hosted_rawtp_rx` and
`hosted_rawtp_tx` pin 10 s of each with the bus timed. Both sides wait 5 s after the data path
opens, so traffic starts near 7.7 s. The steady per-second reports, 12 s runs:

| direction | SDIO bus | emulator | Espressif, P4 + C6, 4 bits at 40 MHz |
| --- | --- | --- | --- |
| C6 to P4 (host receive) | at once | 427-474 Mbit/s | 80 Mbit/s |
| C6 to P4 | `--sd-timing`, 4 bits at 40 MHz | 139.5 Mbit/s | |
| P4 to C6 (host transmit) | at once | 428-430 Mbit/s | 72 Mbit/s |
| P4 to C6 | `--sd-timing` | 106.3 Mbit/s | |

The rates did not move when the slave's hand-back began to wait for the end of the data phase on
the bus ("When the C6 sees it"): the C6 is busy elsewhere for longer than a block's wire time. The
board numbers are Espressif's (esp_hosted 2.12.13 `docs/performance_optimization.md`, "ESP32-C6
as Co-Processor", over the air; iperf there gives 30-32 Mbit/s TCP and 49-50 UDP); a P4-WIFI6
measured about 36 Mbit/s receiving over WiFi (SDIO at 40 MHz, HT20), and oxidrive uploads over
SMB at about 2 MB/s end to end. Without timing a transfer takes no time on the bus, so the rate
is what the two CPUs' drivers sustain at one instruction a cycle: the P4's ESP-Hosted tasks and
memcpy, the C6's ISR and the SLC hand-overs. `--sd-timing` gives the link the card's wire-time
model ("The host"): a 512-byte block in 4 bits at 40 MHz is 1044 card clocks, 157 Mbit/s on the
data lines, and each command adds 106 clocks and each packet its register reads (the interrupt
status, the length), which leaves the 139 Mbit/s measured; transmit pays the TOKEN1 reads and
the padding of each packet to whole blocks. What is still missing against the board's 72-80: the
slave's busy and FIFO latency (it answers at once), the CPUs' caches and PSRAM stalls, and the
time the data takes through the C6's DMA.

The WiFi side is not part of these numbers, and through the air it is the first ceiling. The
virtual access point's receive pacing (`esp-soc/src/wifi.rs`, `StationLink::next_rx`) delivers
one frame per 400 µs of airtime (`RX_GAP_US`) and none while the last descriptor it filled is
still unrecycled, for up to 50 ms (`RX_STALL_US`), as an 802.11b/g link without A-MPDU would:
about 30 Mbit/s of 1500-byte frames at best, below the board's 36. oxidrive's SMB upload meets
neither this pacing nor the SDIO link first: at about 0.55 MB/s it is most likely held by the
NAT's 5840-byte TCP window ("Services", "Upload throughput").

### Faults

`--coproc c6:...,fault=KIND@WHEN` (repeatable; [cli.md](cli.md)) injects faults at points the run
itself fixes, the Nth command or transfer of a kind or a shared time, so a faulted run is as
reproducible as a clean one, and nothing of it runs without `--coproc`. `cli/tests/cosim.rs` runs
each against the hosted pair and pins ESP-Hosted 2.12.13's reaction:

| fault | what happens on the link | ESP-Hosted's reaction |
| --- | --- | --- |
| `read-crc@N[:K]`, `read-timeout@N` | the Nth FIFO read fails (0x109, 0x107) after the slave gave its data, or at the block holding byte K | `Failed to read data`; the packet is lost; the lost one was an RPC response, which times out after 5 s, and the station example aborts and restarts the P4, which resets the C6 and starts again |
| `reg-read-crc@N` | the Nth register-window read fails | `failed to read registers`, a transport failure and an immediate host restart (`H_TRANSPORT_RESTART_ON_FAILURE`); the pair takes its lease again |
| `write-crc@N`, `write-nostatus@N` | the Nth FIFO write: the card drops the block (CRC status error) or takes it without a status | `Failed to send data`, retried (`MAX_SDIO_WRITE_RETRY`); the lease on time |
| `cmd-timeout@N`, `resp-crc@N` | the Nth command never arrives, or is carried out with a bad response CRC | one error; the link goes on |
| `wedge@T` | the C6's CPU stops, its SDIO hardware answers on, TOKEN and PKT_LEN frozen | silence: no error, no restart, no lease |
| `dead-bus@T` | nothing answers | silence |
| `drop-irq@T` | one DAT1 edge goes unlatched (`SdDevice::edge_lost`), D1 stays low | silence: the receive task already waits on the edge, and checks D1's level only when it starts to wait |
| `spurious-irq@T+D` | D1 low with nothing pending | the host reads the registers, finds nothing; harmless |
| `token-stall@T+D` | TOKEN1 shows no free receive buffer | the host polls TOKEN1 at growing intervals (`sdio_is_write_buffer_available`) and its traffic waits: the lease 0.5 s late |

A C6 reset in the middle of the traffic (`a_c6_reset_mid_traffic_ends_in_a_host_restart`) loses
the RPC in flight and ends in a host restart too. The three silent stalls are the finding: in
this configuration ESP-Hosted's receive task waits for the slave's interrupt with no timeout and
the station example waits for its connect event the same way, so a wedged co-processor, a dead
bus or a single lost DAT1 edge stop the WiFi for good without a log line. Nothing here models
the board's own failure; "Handoff" names the stub that would.

### oxidrive

`external_oxidrive_brings_up_wifi_through_the_c6` (cli/tests/cosim.rs) boots oxidrive as the
USB test does, with ESP-Hosted 2.12.13's slave on the C6 as on its board, and the firmware's NVS
partition made at test time with ESP-IDF's `nvs_partition_gen.py` from `nvs.csv` beside the
images (`OXIDRIVE_DIR`): the virtual access point's SSID and password, and placeholders that
reach nothing for the rest (`--net none`: every name resolves to 10.0.2.3 and every connection
is refused), the network's clock fixed so its SNTP answer is the same every run. Neither the CSV
nor the image is in this repository. In 20 s of emulated time (17 s on the host) the firmware
gets past `esp_hosted_reconfigure`, where it stopped before: its WiFi is initialised at 3.25 s and
up with the lease 10.0.2.15 at 6.81 s, its DNS lookup of the NTP pool is answered, SNTP starts,
it reports the NAS unreachable, and its MQTT client starts and finds its session refused. That
is the next point: the services. Against the board's capture, 80 of the board's 105 lines now
match (76 before), the WiFi initialisation and the SNTP lines among them; the blocks that differ
are the eight of "Compared with a board" (the core dump, the heap 108 bytes lower, the card's
geometry, the journal and the configuration, the die temperature), the configuration's effects
(no car paired, no syslog, the placeholder addresses), and the network: the SSID and address
in the WiFi line, the NAS reachable on the board, MQTT's session up there and refused here.
With the services of milestone 4b behind forwards the NAS is reachable and the session comes up
here too ("Services").

## Services

Milestone 4b puts real servers behind the virtual network: mosquitto as the MQTT broker and
samba's smbd as the NAS, run unprivileged on 127.0.0.1 from a directory of their own per test,
and reached from the P4 through ESP-Hosted, the C6's radio, the virtual access point and two TCP
forwards. `tests/services.rs` starts them (`Mosquitto::start`, `Samba::start`), waits until
each accepts a connection, and stops each with its process group when dropped (SIGTERM, then
SIGKILL after 5 s: smbd's parent stops its children on SIGTERM, a SIGKILL to it alone would leave
them). `external_samba_and_mosquitto_serve_a_test_client` checks both with their own clients.

**Installing them.** macOS: `brew install samba mosquitto`. Homebrew installs smbd as
`/opt/homebrew/opt/samba/sbin/samba-dot-org-smbd`; `/usr/sbin/smbd` is the system's own SMB
server and is never used. Linux (Debian, Ubuntu): `apt install samba mosquitto
mosquitto-clients`, `/usr/sbin/smbd`; the configurations below should serve unchanged, but have
only been run on macOS. `SMBD`, `PDBEDIT`, `SMBCLIENT`, `MOSQUITTO`, `MOSQUITTO_PASSWD`,
`MOSQUITTO_SUB` and `MOSQUITTO_PUB` override the paths.

**samba** (`services::samba_conf`), samba 4.25:

```ini
[global]
  smb ports = PORT                      # unprivileged; the forward maps 445 to it
  bind interfaces only = yes
  interfaces = 127.0.0.1
  server role = standalone server
  server min protocol = SMB2_10
  server max protocol = SMB3_11         # oxidrive's libsmb2 negotiates 3.1.1, signed
  server signing = mandatory
  map to guest = never
  passdb backend = tdbsam:DIR/private/passdb.tdb
  username map = DIR/users.map          # UNIX_USER = TEST_USER
  private dir = DIR/private             # and lock, state, cache, pid, ncalrpc, log under DIR
  vfs mkdir use tmp name = no
  disable spoolss = yes
  load printers = no
[SHARE]
  path = DIR/share
  read only = no
  valid users = UNIX_USER
```

samba keeps a password only for an account of the host, so the test user is mapped to the
account running the test (`username map`), whose password entry `pdbedit -a -t` makes in the
test's own passdb. `vfs mkdir use tmp name = no` is needed on macOS: samba 4.25 makes a
directory under a temporary name with mode 0 and renames it into place, macOS refuses that rename
(EACCES), and the client gets ACCESS_DENIED with a mode-0 directory left behind; oxidrive's
`mkdir_p` then fails every clip. smbd runs with `--foreground --no-process-group`; its "tdb
mutexes not available" warnings are harmless on macOS.

**mosquitto** (`services::mosquitto_conf`), 2.1.2: `listener PORT 127.0.0.1`,
`allow_anonymous false`, `password_file` made with `mosquitto_passwd -c -b`, `persistence false`.

**The forward.** oxidrive's NAS probe connects to port 445 of its share's host whatever the URL
says, so the share stays at port 445 for the firmware and a forward takes it to smbd's port
(`--net none,fwd=...`, [networking-howto.md](networking-howto.md), "Forwarding a guest address to a
host service"). With `none` everything else stays inside the virtual network: names resolve to
10.0.2.3, SNTP answers from the fixed epoch, every other connection is refused, so a firmware
cannot reach a real server it has built in.

**oxidrive's provisioning.** `cli/tests/oxidrive_services.rs` makes the NVS partition from
`OXIDRIVE_DIR/nvs.csv` as before; for these tests its SMB URL is `smb://10.0.2.100/SHARE` and its
MQTT URL `mqtt://10.0.2.100:1883`, with the test accounts of the file (`esp32sim` / `esp32sim-smb`,
`esp32sim` / `esp32sim-mqtt`). The test reads the two URLs from the CSV, starts the services, and
forwards the guest addresses to them (a host name in a URL stands for 10.0.2.3). Neither the CSV
nor anything the firmware publishes is in this repository; the device id in the topics comes from
the board's MAC at test time. oxidrive vetoes every window and every shared pass when it cannot
read its die temperature; the P4's temperature sensor reads a fixed 25 °C ("The temperature
sensor" below), which it reports as `temp_c=Some(25)`. Milestone 4b's tests seeded the register
instead, which a P4 restart lost.

**The broker and the NAS** (`external_oxidrive_meets_its_broker_and_its_nas`). The station takes
its lease at 6.81 s, and at 7.30 s the firmware finds the NAS (`senses: nas_reachable=true ...
temp_c=Some(25)`), starts its MQTT client, and logs `session up` 3 ms later and `session ready,
commands on oxidrive/<id>/cmd` at 8.33 s. A subscriber started by the test then receives the
retained `oxidrive/<id>/avail` (`online`) and `oxidrive/<id>/state`, the firmware's JSON state
document (`"nas":{"reachable":true}`, `"window":{"phase":"serving"}`, `"temp_c":25`, ...). 9 s of
emulated time take 9.3 s on the host. The build seeds Home Assistant discovery on, so it also
announces its sensors and buttons.

**An archive window** (`external_oxidrive_archives_a_window`). The car of "USB" enumerates at
0.27 s and records two saved events, `TeslaCam/SavedClips/<stamp>/event.mp4`, 2 MiB each as a
4.1 MB/s camera writes them: directories three deep under long names and a clip whose top-level
MP4 boxes are `ftyp`, `mdat` and `moov` (`fat32::mkdir_all`, `create_path`, `mp4_bytes`), which
oxidrive's walkers need. It then idles, polling TEST UNIT READY every 2 s as udev polls a
removable disk. oxidrive's own policy does the rest. `OXIDRIVE_FORCE_OPEN=1` instead publishes
`force_open` on `oxidrive/<id>/cmd` every 2 s from 15 s after the car's last write: the first one
forces a shared pass, a later one the window, taken at 28.8 s with the same uploads, which saves
40 s of emulated time. Without it:

| t (s) | what happens |
| --- | --- |
| 60.3 | the first probe; a shared pass (oxidrive reading alongside the car, which keeps the card) uploads the older event and holds the newest back for one pass, as its archiver does |
| 69.8 | the shared pass done: `.part` written, read back, renamed, the manifest (`.oxidrive-index.tsv`) written |
| 70.8 | the card quiet for a minute: `supervisor: card taken for archiving` |
| 71.3 | the car's poll: `sense 2/3a/00 (MEDIUM NOT PRESENT)` |
| 70.8-79.7 | the window uploads the newer event (`1 uploaded, 1 already there`), `.part`, read-back, rename, manifest |
| 79.7 | `supervisor: card returned to the car` |
| 81.3 | the car's poll passes; it reads the volume again (a medium change) and records a third event |

Afterwards fsck (`sd_image::fsck`: macOS's `fsck_msdos -n` or Linux's `fsck.fat -n` on the
volume copied out of the image) passes, every clip reads back over USB and off the image as the
car wrote it, both uploads on the share equal the card byte for byte, no `.part` is left, and the
manifest lists both with their sizes and CRC-32s. The third event, written after the window, is
not uploaded within the run. 86 s of emulated time take 179 s on the host.

**Upload throughput.** The window test times each upload from the firmware's SMB log
(`begin_part` to the `.part` size check after the last write, then the read-back and rename) and
samples the station's frames and the NAT's bytes every 100 ms; `OXIDRIVE_WINDOW_CLIP_MIB` sizes
the clip the window takes (the shared pass's stays at 2 MiB, and oxidrive paces that pass itself):

| SD bus | window's clip | write | read back |
| --- | --- | --- | --- |
| at once | 8 MiB | 0.51-0.53 MB/s (3 runs) | 0.55 MB/s |
| at once | 16 MiB | 0.54 MB/s | 0.55 MB/s |
| timed (`OXIDRIVE_SD_TIMING=1`: the card and the SDIO link, as `--sd-timing`) | 8 MiB | 0.35 MB/s | 0.55 MB/s |
| timed | 16 MiB | 0.35 MB/s | 0.55 MB/s |

On a first boot oxidrive reads back every upload before it renames it, so its own figure
(`window: 16777216 bytes in 61.6s = 0.27 MB/s end to end`) counts both directions. Its board
uploads at about 2 MB/s on the same archiver path. What sets the emulator's
rate, by experiment (one constant changed and the 8 MiB run repeated; none of these is
committed):

| change | write | read back | station frames a second while writing, out / in |
| --- | --- | --- | --- |
| none | 0.53 MB/s | 0.55 MB/s | 1003 / 2023 |
| an MSS option (1400) in the NAT's SYN-ACK | 0.54 MB/s | 0.55 MB/s | 391 / 797 |
| the access point's receive pacing at 100 µs instead of 400 µs | 0.55 MB/s | 0.62 MB/s | 1043 / 2098 |
| the NAT's window at 16384 or 65535 instead of 5840 bytes | none: the first 1.5 MB write never completes | | |

- **The NAT's window is the likeliest bound.** The NAT advertises at most 5840 bytes and scales
  nothing, so each direction has at most 5840 bytes in flight whatever the firmware's lwIP
  offers (65535 in this build, as on its board). Both directions run at about that window over a
  10.7 ms round trip (5840 bytes at 0.54 MB/s), and the rate does not follow the segment size:
  536-byte segments take 1003 frames a second, 1400-byte ones 391, for the same rate, so it is
  not a per-packet cost. No CPU is saturated: while writing, P4 core 0 retires an instruction on
  77% of its cycles, core 1 on 44%, the C6 on 28%; while reading back 40%, 66% and 8%. A
  per-byte cost on core 0 is not ruled out. The direct test, a larger window, could not be made:
  with 16384 or 65535 bytes the first write stalls (last row). Where the 10.7 ms go (the P4's and
  the C6's ESP-Hosted queues, the C6's WiFi driver, the NAT's 500 µs host-socket poll) is not
  measured yet. The servers answer in host time, so repeated runs differ by a few percent.
- **The NAT's SYN-ACK carries no MSS option**, so lwIP keeps its initial send MSS of 536 bytes
  (lwIP `tcp.c`, `INITIAL_MSS`, raised only by the option); a NAS sends one. Here it costs
  frames, not rate.
- **The ceilings "Throughput" names do not bind here.** The receive pacing, a frame per 400 µs
  (2500 a second), is about 81% used while writing, by the two ACKs the NAT returns for each
  segment, yet a quarter of the gap moves the write by 4% and the read-back by 12%. No frame was
  dropped for want of a descriptor, and 802.11b/g without A-MPDU, about 30 Mbit/s of 1500-byte
  frames, is far above 0.55 MB/s.
- **The timed SD bus slows only the write** (0.53 to 0.35 MB/s): the upload reads the card on
  slot 0 while its packets cross to the C6 on slot 1 of the same SDMMC host, so each card read's
  wire time holds the SDIO link; the read-back touches no card and does not change. That is the
  sharing "Timed SD transfers" below anticipated.
- **Against the board's ~2 MB/s** the emulator's rate is not comparable yet: the NAT would need
  the window the firmware asks for (with window scaling) and an MSS option first, and then the
  round trip taken apart; the emulator also has no cache or PSRAM stalls. Raising the window
  alone (last row) stalled the first write: libsmb2 waited 80 s for a reply, the firmware
  recorded the radio dead (`err 0xffffffff x4 over 50s`) and its fail-safe restarted the P4. That
  wants understanding before the window changes: a burst the 5840-byte window never let through
  meets something on the way (the C6's buffers, the access point's queue, the NAT).

**An update** (`external_oxidrive_refuses_an_unsigned_update`, goal 4, not passing). oxidrive
fetches an update from the share and checks a signature only the owner's key makes, so the test
can only exercise the refusal: a manifest with the image's true size, CRC-32 and SHA-256 and a
signature no key made. The firmware checks the manifest, takes the card for the update, and then
stops: it hashes the download with the SHA accelerator in DMA mode, which on the P4 runs through
the AXI GDMA (IDF v5.5.3 `mbedtls/port/crypto_shared_gdma`, `SOC_AXI_GDMA_SUPPORTED`), not
modelled, so SHA_BUSY (0x50091018) never clears and core 0 spins on it. The card stays with the
application. mbedTLS's AES in DMA mode goes the same way on the P4.

**Commands sent while it is away.** From teslausb-p4 cfdcace oxidrive keeps a persistent MQTT
session (clean session off, its device id as the client id), so the broker queues QoS 1
commands for it while it is gone; publishers set an MQTT 5 message expiry, which the broker
enforces by its own clock. Five tests put a `Relay` (`tests/services.rs`) between the firmware
and mosquitto (`RigOpts::mqtt_relay`): the relay can cut every connection and refuse new ones,
or go deaf one way, dropping what the device sends while the broker's side still flows. "Away"
is the relay cut until the broker has published the device's will (`avail` = `offline`), then
`CoSim::p4` reset as a chip reset (`Machine::reboot`: flash and card kept), with the relay
restored before the emulator runs again. The tests that send commands while the device is away
wait for `session up (resumed)`, the firmware's log of CONNACK's session-present flag. Results
on teslausb-p4 ded3c80 (and before it 6ca4b86), all passing, 19-28 s on the host each:

| Test | Published while away (expiry) | What the firmware did |
| --- | --- | --- |
| `external_oxidrive_runs_a_command_queued_across_a_reboot_and_drops_an_expired_one` | `ble_probe` (2 s), `update_status` (600 s), then 4 s of the host's time | session resumed about 7.3 s after the reset; `update_status` arrived within a millisecond of CONNACK, before the firmware subscribed again, and ran once; `ble_probe` never arrived |
| `external_oxidrive_acts_once_on_a_redelivered_command` | none: `update_status` sent while connected, the relay deaf, so its PUBACK was lost; then the connection cut | ran it once; 15 s later, on the resumed session, `command 1 delivered again within 120s; ignored` |
| `external_oxidrive_cancel_queued_behind_an_update_now_wins` | `update V now` and `update_cancel V` (300 s each), V staged as in the update test | both arrived 3 ms apart after CONNACK; `update to V cancelled before it started`, the card never taken, the retained state's `ota.phase` `cancelled`; then `update V now`, sent after the cancel, took the card |
| `external_oxidrive_never_sees_an_update_now_that_expired_while_it_was_away` | `update V now` (2 s), `update_status` (600 s), then 4 s | only `update_status` arrived; the card never taken |
| `external_oxidrive_cancel_gives_back_the_card_an_update_holds` | none: `update V now` sent while connected, `update_cancel V` once the update had taken the card | the card was back with the car 155 ms after the cancel arrived, `cancelling the update to V` |

The redelivery test is the only one that needs the deaf relay: a QoS 1 message is sent again
only on the next session, and only when its acknowledgement was lost. mosquitto 2.1.2 sent it
with its original packet id, which is what the firmware keys on. The last test stops an update
from the supervisor's side only: the download never ends here (SHA's DMA mode, "An update"),
so the cancel checks the firmware's SMB thread makes between reads and before it sets the boot
partition are not reached. `a_relay_passes_goes_deaf_one_way_and_cuts` checks the relay itself
against an echo server and runs with the CI suite.

**A cut SMB link** (`external_oxidrive_shared_pass_stops_once_when_its_smb_link_is_cut`). A second
`Relay` can stand in front of smbd (`RigOpts::smb_relay`); `Relay` is `Clone`, so a test holds
the relay while the car holds the rig. The car records four saved events of 1.5 MiB, then writes a
RecentClips clip 64 KiB every 500 ms, so the card is never quiet and no window opens. `force_open`
at 8.2 s starts a shared pass, which archives two of the events (the newest is held back, and one
is discarded because the car wrote a sector it shares). At 29.1 s the car records four newer
events. The firmware's own pass a minute after the forced one uploads those newest first and then
checks the older ones; the relay is cut at 78.8 s, while it writes its second clip, and restored
at 83.9 s. A second `force_open` is not used: one that meets the supervisor probing takes the card
from the recording car. Results on teslausb-p4 c4c538c and on f08bb82 (branch smb-session-lost):

| | c4c538c | f08bb82 |
| --- | --- | --- |
| the cut pass ends | 81.4 s | 81.1 s, at the first failed call |
| `archiver: connection lost` | none | once |
| failure lines before the next session | 10: a failed close, 5 failed clip opens, 2 failed manifest unlinks, 2 failed manifest saves | none |
| SMB calls logged before the next session | 16 | 3: the write in flight and two closes of its handle |
| next session | 130.5 s | 130.2 s |
| clips on the share, byte for byte | 4 of 8 at 240 s; the 4 others failed and wait out a 15-minute cooldown | 8 of 8 at 167.5 s |
| host time | 457 s for 240 s | 504 s for 169.5 s |

The loss reaches the firmware 2.3 s after the cut as `smb2_service: POLLERR`, the error the car
logged. c4c538c then fails every clip left in the pass, and because a failed stat reads as an
absent file it drops the manifest rows of the clips it had archived and retries the manifest save
after each clip. f08bb82 stops the pass at that first failure, saves nothing, and its next pass
resumes the clip in flight. Rerun on teslausb-p4 7f8d2a6 (branch smb-followups) and f08bb82 with
the relay fixed below, the cut test gives the same figures on both builds to the 100 ms slice:
the pass ends at 81.1 s, three SMB calls and no failure line before the next session at 130.2 s,
every clip on the share at 167.4 s.

**A half-open SMB link** (`external_oxidrive_shared_pass_ends_when_its_smb_link_goes_half_open`).
The same scenario (`smb_fault`), with `Relay::half_open` in place of `cut`: everything both ways
is read and dropped and nothing is closed, live connections and new ones alike, as when the NAS
vanishes without a reset; the relay passes again 100 s later. Nothing on the wire ends the pass,
so the firmware's own timeouts must. The test asserts the faulted pass ends within 90 s, no
main-loop iteration is longer, and a pass on the silent link gives up within 30 s.

| | f08bb82 | 7f8d2a6 |
| --- | --- | --- |
| the faulted pass ends | 319.1 s, 240.3 s after the fault: the SMB thread waited on the write for ever, the archiver gave up at `STALL_BUDGET` (120 s) and its disconnect waited another 120 s behind the stuck call | 142.2 s, 63.4 s after the fault: libsmb2's 60 s request timeout (`SMB2_STATUS_IO_TIMEOUT`) on the write in flight |
| longest main-loop iteration after the fault | 250.4 s | 73.5 s, of which 9 s came before the fault |
| a pass while the link is still silent | none: the next pass came at 560.3 s, after the link was back, and still found no session | 159.2 s, after 16.0 s: the 15 s connect timeout |
| next session | never in 900 s; every pass blocked the main loop about 241 s | 263.3 s, the pass a backed-off two minutes later |
| clips on the share, byte for byte | 3 of 8 at 900 s | 8 of 8 at 300.5 s |
| host time | 1214 s for 900 s | 656 s for 302.5 s |

f08bb82 has no request timeout, so the write in flight waits for replies that never come, and
the SMB thread with it, after the link is back too: the bytes dropped while it was silent leave
the server waiting for the rest of a request. Every later call queues behind it. The new build
times the write out, fails the calls queued behind it at once, and treats a call that outlives
`STALL_BUDGET` as a stuck thread to stop waiting on.

**The relay and a half-close.** The first half-open runs never reconnected after the link came
back: the firmware's connect timed out with "Timeout expired and no connection exists", and the
NAT logged no SYN for it. The NAT holds at most 64 TCP flows (`nat.rs`, `MAX_FLOWS`) and evicts
only one in TIME_WAIT or closed; a SYN that finds it full of others is dropped without a word.
The firmware's NAS probe connects and closes about once a second, and each probe left a flow
whose host side never closed: after the probe's FIN the relay shut its sockets down, which on
macOS sent nothing to a peer that had already half-closed, and `Relay::cut` kept a clone of every
socket, so the last handle never closed either. Instrumented, the table held 42 such flows at
80 s and 63 at 220 s. Once a connection's pumps end, its handles now leave `live`, so the last
one's close sends the FIN; a client that half-closes reads 0 from the relay at once (it waited
10 s for nothing before), as `a_relay_passes_goes_deaf_one_way_and_cuts` now checks. The cut
test never noticed, because `cut` closes everything held.

**A share that misbehaves** (`cli/tests/oxidrive_share_faults.rs`). The same car as `smb_fault`
(saved events, then a RecentClips clip written every 500 ms, so the firmware archives in shared
passes a minute apart, the first forced over MQTT), against a share that misbehaves in one way
per test. New knobs: `Relay::start_smb` reads what the client sends as SMB2 messages, and a
`Trap` closes the connection at the first matching request, before passing it on or, with
`after`, once it has been carried out, so its reply is lost (`smb2::renames_to`, `smb2::is_create`;
`a_relay_trap_ends_a_connection_at_one_smb2_request` checks it in CI). `nat::Shape`, set through
`RigOpts::smb_rate` and `smb_connect_delay_ms`, holds the share's port to a byte rate each way
and delays its handshakes, in emulated time, so the figures do not move with the host's load.
`OXIDRIVE_NTP_FROM_S` (`VirtualNet::ntp_from_us`) holds the network's NTP answers back, which
places the clock's first step. Each test asserts what a firmware that handles the fault does;
on an older build it fails, and its printout shows what that build did instead. Results on
teslausb-p4 eb81ae4 (branch smb-followups before the second review) and on 39fe424:

| test | eb81ae4 | 39fe424 |
| --- | --- | --- |
| `keeps_its_checksums_when_a_manifest_save_dies_at_the_rename`: after a clean pass, the first rename onto `.oxidrive-index.tsv` ends the connection unforwarded; a power cut; a reboot | the manifest after the reboot has no checksum for the 3 events archived before the cut | `index: no manifest; 4 entries from .oxidrive-index.tsv.part (4), .oxidrive-index.tsv.prev (3)`; all 3 checksums kept |
| `does_not_save_over_a_manifest_too_large_to_load`: 4,239,010 bytes of rows on the share | `index: 4239010 bytes is not a manifest, ignoring it`, then saved over: 174 bytes | not loaded, said so; the file unchanged; the events archived |
| `keeps_the_rows_of_a_manifest_with_a_bad_byte`: 500 rows and one with 0xff 0xfe in its path | `index: not text, continuing without it`; 0 of the 500 rows kept | 500 of 500 kept |
| `shared_pass_ends_when_its_smb_link_goes_half_open` (`oxidrive_services.rs`, now also: no pass after the restore finds no session) | f08bb82: the faulted pass ends 240.2 s after the fault and nothing reconnects; 3 of 8. eb81ae4: 63.0 s; no pass without a session after the restore; 5 of 8 by the 360 s deadline, its first pass having met the clock's first step and backed off | 62.9 s; reconnected 263.2 s; 8 of 8 at 301.3 s |
| `verifies_a_large_clip_over_a_slow_link`: 100 kB/s, a 20 MiB event and a small one | the resumed part's read-back is given up at 120 s (556.6 s); the next pass finds the SMB thread still digesting (`no SMB session`) and backs off; the one after re-verifies and is given up at 120 s again (798.6 s); not on the share by 900 s | read back in one call over 239 s; archived at 556.0 s with its checksum; no call given up, no late-reply warning |
| `backs_off_from_a_nas_that_drops_every_session`: every session ended at its first CREATE for 300 s | 5 passes lost, 60-61 s apart; 1 clip deferred on strikes; all archived at 402.7 s | 2 passes lost, 120 s apart; none deferred; all archived at 471.1 s |
| `waits_out_a_clip_whose_stat_is_refused`: one event directory on the share mode 000 | refused in 6 of 6 passes, never deferred; `backlog_retry` not published | refused in 3 passes, then deferred; state `backlog` 1, `backlog_retry` 0 |
| `replaces_an_archived_copy_that_does_not_match`: the event's rename passed on and its reply lost, a byte of the copy flipped | `does not match its part's checksum; recording none`; the damaged copy stays, with no checksum | removed at 147.7 s, sent again, on the share byte for byte at 198.3 s with its checksum |
| `connects_with_its_request_timeout_off`: `SMB_TIMEOUT=0` builds, 350 ms handshakes, 10 boots | 4 of 10 first connects fail, `Timeout expired and no connection exists` | 0 of 10 |
| the half-open test, `OXIDRIVE_NTP_FROM_S=20`: the first session opens before the clock is set | 9641a73 (the request timeout off until the session ends): the faulted pass ends 120.2 s after the fault, the SMB thread never comes back, 3 of 8 | the session takes its timeout at its first call after the clock is set (70.1 s); 62.9 s; 8 of 8 at 300.5 s |
| `oxidrive_latency.rs`, the shared-pass test, `OXIDRIVE_NTP_FROM_S=20` | the pass never ends, 1 of 2 events (fc6a26f: `SMB2_STATUS_IO_TIMEOUT` at 41.3 s, 1 of 2) | connected with `0s request timeout`, the timeout taken mid-pass; 2 of 2 |

The manifest test loses the session at the rename into place rather than having it refused:
samba refuses that rename only by refusing the unlink before it, which needs the directory's
write permission too, and the delete-pending state that refuses one without the other needs a
client holding the manifest open with FILE_SHARE_DELETE, which neither libsmb2 nor smbclient
opens with. The state on the share is the same. eb81ae4 passes the half-open test's new check
because its 60 s request timeout ends the stuck call before the 120 s budget, so the replies a
stuck worker left are never read there; the courier's host test is the proof of that fix. Each
test runs about 5 to 15 minutes of emulated time, the slow link longer.

**The latency test and the clock's first step.** On fb6b483 the shared-pass latency test failed
in every run, and on fc6a26f in most, with the first upload stalled 60 s; smbd logged `Bad SMB2
(sign_algo_id=1) signature for message` once in each of five failing runs checked and not in the
passing one. The SMB stream, captured at the NAT, held a 64 KiB WRITE whose last 234 bytes were
the next request's NetBIOS and SMB2 headers: the device sent a request short and the next one
after it. Another failing run timed a write out 6 s after connecting.
libsmb2 times requests by `time()`, and its sweep (`smb2_timeout_pdus`) takes an expired request
out of the send queue even when it is half written. The test forces a pass 9 s after boot, and
the clock's first SNTP step, which moves `time()` on by decades, lands within a few seconds of
it: every request queued before the step expires at once. Whether the step met a request half
written, one waiting for its reply, or the connect depended on the build's timing, which is why
eb81ae4 passed it. `OXIDRIVE_NTP_FROM_S=20` puts the step inside the pass on every build, as the
last row shows.

**libsmb2 on a monotonic clock.** 39fe424 worked around the step: a session opened before the
clock was set ran without a request timeout until it was. If NTP never answers, as after a boot
away from home WiFi, that is the whole boot. teslausb-p4 3f445d4 vendors libsmb2 and measures its
request timeouts and connect wait on `CLOCK_MONOTONIC` (esp_timer time since boot), so the
timeout is on from the first request, and drops the workarounds. Fixtures `oxidrive-smb-followups`
(39fe424) and `oxidrive-smb-monoclock` (3f445d4):

| | 39fe424 | 3f445d4 |
| --- | --- | --- |
| the half-open test with `OXIDRIVE_NTP_FROM_S=100000`: NTP never answers | connected with `0s request timeout`; the faulted pass ends 120.2 s after the fault, at `STALL_BUDGET`, and the SMB thread never comes back; 2 passes after the restore find no session; 3 of 8 | connected with `60s request timeout`; 63.6 s; reconnected 269.5 s; 8 of 8 at 309.8 s |
| the latency test with `OXIDRIVE_NTP_FROM_S=20`: the step mid-pass | passes, the session's timeout taken mid-pass | passes with `60s request timeout` from the connect: one session, no `Bad SMB2 signature` in smbd's log |

On 3f445d4 the share-fault tests, cut, half-open, the update tests in `oxidrive_services.rs`,
the latency test and the car report's power-cut and update tests pass as on 39fe424. The NTLMv2
timestamp is still `time()`; in the run where NTP never answered, smbd accepted every session
with the wall clock at 1970.

**An update in flight.** No test makes an SMB call during an update: `Ota::busy` keeps windows,
passes, crash uploads and trace uploads from starting, so the courier's new refusal during a
job is not reachable from the rig. The update tests in `oxidrive_services.rs` pass on 39fe424.

## The temperature sensor

LP_TSENS (0x5012F000, `esp32p4::periph::Tsens`) reads a fixed die temperature, 25 °C unless a
test sets `Tsens::celsius`, kept through a chip reset. CTRL.OUT (bits 7:0) and READY (8) read the
sensor while POWER_UP (22) is set, in the range the driver last wrote over regi2c: I2C_SAR_ADC
(block 0x69) register I2C_SARADC_TSENS_DAC (6), bits 3:0 (IDF v5.5.3
`soc/esp32p4/include/soc/regi2c_saradc.h`). OUT inverts IDF's conversion,
0.4386 × OUT − 27.88 × offset − 20.52 (`hal/esp32p4/include/hal/temperature_sensor_ll.h`;
`esp_hw_support/sar_periph_ctrl_common.c`, `temperature_sensor_get_raw_value`), with the offset
of the range whose register value matches (`soc/esp32p4/temperature_sensor_periph.c`: 5, 7, 15,
11, 10 for offsets −2 to 2), rounded to the nearest count: 25 °C is 104 in the −10..80 °C range
oxidrive asks for (DAC 15, offset 0), which reads back as 25.09. A DAC value outside the table, or
none written, reads 0 without READY, as a sensor not powered. CTRL resets to the header's value
(WAKEUP_MASK, INT_EN, CLK_DIV 6); the other registers are register RAM. The LP_PERI clock gate and
reset the driver toggles, and the efuse calibration (zero here), are not modelled.

A chip reset re-creates the digital peripherals but keeps the analog blocks behind regi2c (the
PLLs' dividers, the SAR range): IDF's `esp_restart` leaves the CPLL on, and a bootloader that
finds HP_ROOT on the CPLL after a reset reads the dividers back (`rtc_clk_cpu_freq_get_config`
aborts on 0 MHz). The map was re-created with the peripherals before, so a CPU reset that did not
go through `esp_restart` aborted oxidrive's bootloader. `external_oxidrive_reads_its_temperature_after_a_restart`
restarts the P4 with a CPU software reset and finds `temp_c=Some(25)` before and after.

The sensor is a `device_set!` entry. Measured against 109a1fa8 with `cargo build --release
--bins`, seven alternating runs, median `user` time, identical instruction counts:

| demo | 109a1fa8 | this branch | instructions |
| --- | --- | --- | --- |
| S3 `hello` (30 s) | 0.40 s | 0.40 s | 11,990,488 + 6,798,360 |
| `c3-hello` (30 s) | 3.88 s | 3.89 s | 4,800,000,000 |
| `c6-hello` (30 s) | 5.37 s | 5.36 s | 4,800,000,000 |
| `p4-hello` (27 s) | 18.24 s | 18.29 s | 9,720,000,000 + 0 |
| `p4-hello-smp` (27 s) | 19.34 s | 19.29 s | 9,720,000,000 + 9,713,148,105 |
| pocket-tank (8 s) | 14.68 s | 14.71 s | 1,113,634,993 + 1,490,529,616 |

The P4 pays one more block compare per peripheral access (`pre_access`); the others run no code
this change touches, so their differences are noise.

## The car's hazards (milestone 4c, part 1)

The car of "USB" now behaves like the Linux host in a Tesla where an archive window can hurt it:
it keeps writing until a poll tells it the medium went, it can keep its FAT and directories
between polls, and it can write back as a page cache does (`pagecache`, `car`, the additions to
`fat32` in "USB", "The host"). `cli/tests/oxidrive_hazards.rs` runs it against the current
oxidrive build (728b71f, `OXIDRIVE_CURRENT_DIR`; the 4b tests keep the 46a5472 build of
`OXIDRIVE_DIR`) on the rig of "Services", which moved to `tests/oxidrive_rig.rs` and logs every
block written to the card, so a test can tell the car's writes from oxidrive's own. Each test
asserts what the firmware does today, named so that it fails once the firmware changes it. The
tests of findings 1 to 3 now assert the fixed behaviour and need a build with oxidrive's window
gate (teslausb-p4 5633fc2, `OXIDRIVE_CURRENT_DIR`): "The window gate" below; what 728b71f did
stays in the sections before it.

Every run here times the SD bus (`--sd-timing`) and gives the card 1500 µs of programming after
each write (`OXIDRIVE_SD_PRG_US`): an assumption, not a value measured on a card. The emulator runs
one instruction a cycle with no cache or PSRAM stalls and leaves the card's read access time and
the IDMAC's pace untimed, so every time below is a lower bound. smbd and mosquitto answer in host
time, so the interleaving of a window with the car's writes differs between runs: the tests
assert the mechanisms, and the numbers below name the run they come from. Every number below was
taken with the SD model's auto-stop fix (a timed bus sends a multi-block command's auto-stop at
the end of its data phase, so the card programs after the data, not during it; `sdmmc: on a
timed bus the auto-stop reaches the card after the data phase`); runs before it are not counted.

### The car

- **Polls** follow `sd_check_events` (Linux v6.12 drivers/scsi/sd.c:1710-1776): TEST UNIT READY
  every `poll_ns` while it records and idles (2 s by default, udev's media poll: systemd v256
  rules.d/60-block.rules; the kernel's own default interval is 0, block/disk-events.c:38-39); NOT
  READY or UNIT ATTENTION with ASC 3Ah marks the medium gone (`media_not_present`, 1682-1699), a
  change either way or any UNIT ATTENTION is a medium change. A medium gone has capacity 0
  (`set_media_not_present`, 1671-1680), so what the car still holds cannot reach it and is
  dropped; a medium back is read again (`Car::revalidate`: READ CAPACITY, MODE SENSE, the volume),
  after an fsck when the volume is dirty and the car runs one. Between polls only a command that
  fails tells the car the medium went: a read or write refused with NOT READY fails (the car's
  clip is cut), and one that meets a UNIT ATTENTION also marks the device changed, so the car's
  reads and writes stop until its next poll, which takes it as a medium change (scsi_lib.c:803-811,
  sd.c:1340-1343).
- **Recording**: `record_set` writes clips as cameras feed them, chunk i of every clip due at
  t0 + i × gap, clip after clip, polling between chunks, until a poll finds the medium gone or a
  write fails; the files are made first and closed last. The car keeps every file's clip and how
  much of it it wrote, the truth an archive is compared with.
- **Keeping metadata** (`PageCache::keeping_meta`): the boot sector, FSInfo, FAT and directory
  sectors the car read or wrote are answered from memory until the medium is seen to change, as
  Linux's buffer cache answers them.
- **Writeback** (`PageCache::writeback`), below.
- **Rotation** deletes as vfat unlinks (v6.12 fs/fat/namei_vfat.c `vfat_unlink`, inode.c
  `fat_evict_inode`, fatent.c `fat_free_clusters`); allocation from FSInfo's hint, as Linux's
  `fat_alloc_clusters` starts after `prev_free` (fatent.c:488), so on a full card the next clip
  takes the clusters the last deletion freed.
- **The dirty flag** (FAT32 boot sector byte 65, which Linux keeps as `state`: v6.12
  include/uapi/linux/msdos_fs.h FAT_STATE_DIRTY) set at a mount and cleared at a clean unmount
  (inode.c `fat_set_state`, `fat_put_super`); a car that loses power leaves it set.
- **fsck** (`Fat32::fsck`) after dosfstools 4.2's `fsck.fat -a`: both FATs read whole; each
  file's chain walked from its entry, a chain longer than the size cut at the size (src/check.c:
  528-537), a size past the chain cut to the chain (629-640), a chain into a free, bad or owned
  cluster cut before it; clusters no entry reaches freed (`reclaim_free`, src/fat.c:420; with -f
  dosfstools keeps them as FSCKnnnn.REC files instead, `reclaim_file`, not modelled); FSInfo's
  free count; the dirty flag cleared (check.c:1065-1080). The FATs are written either as dosfstools
  writes them (`FsckWrite::Changed`: the sectors with a changed entry, in both copies, and the
  second copy whole when the two differed, fat.c:160-186) or whole, FAT1 first
  (`FsckWrite::Bulk`, the shared-reading plan's "fsck rewriting FAT1 in bulk", an assumption about
  the car).

`esp-soc/src/usb_host/tests.rs` pins each behaviour against the fake device, which can now serve
a window the way esp_tinyusb does (TEST UNIT READY answers MEDIUM NOT PRESENT, READ(10) and
WRITE(10) still reach the card) and change the card at the window's start: writes into a window
until the next poll; a car keeping metadata writing over a window between its polls, and one
that does not reading it; writeback's ordering; a power cut that fsck repairs; rotation; a ring
of long names in one directory; a USB reset followed by a bulk fsck. With `gated` it serves a
window as oxidrive's gated copy does ("The window gate"): a car refused at its first write in the
window and revalidating at the unit attention after it; a caching car reading again what a window
between its polls wrote; a writeback car whose pages the window and the unit attention refuse.

### The writeback model

`pagecache::WritebackCfg` is the firmware's own writeback model (`WritebackCfg` in teslausb-p4
testkit/src/sim.rs) with the same fields and defaults, in nanoseconds; that file records what
each value rests on:

| parameter | value | provenance (testkit/src/sim.rs) |
| --- | --- | --- |
| `min_delay_ns`, `max_delay_ns` | 50 ms, 5 s | lag is one flusher wake: measured on a Linux host, the bound followed `dirty_writeback_centisecs` (4.74 s at its default 500, 14.3-15.0 s at 1500); 500 is v6.12 mm/page-writeback.c:102. The car's own value has not been read |
| `data_first_bias` | 0.96 | the share of metadata pages held back past their data, measured on the car (0.005 on macOS) |
| `metadata_lag_ns` | 30 s | one `dirty_expire_centisecs` (mm/page-writeback.c:109); not measured |
| `reorder`, `seed` | true, 0x2545F4914F6CDD1D | landing order independent of write order; fixed so a run reproduces |
| page | 8 sectors | `PAGE_SECTORS`: Linux reclaims 4 KiB pages |

How the car uses them, and where it differs from `testkit::sim::Sim`:

- The data of one write lands in one batch, 50 ms to 5 s after it was written (testkit lands each
  page on its own draw); each metadata page draws its own delay, and 96 times in 100 the 30 s lag
  on top. A page written again before it lands keeps its schedule and carries the newest bytes
  (testkit queues a second copy, which can land after the first). Reads see the car's own pages.
- The writes come from the car's own FAT32 writer in Linux's order (`FileWriter::linux`): the
  entry's size and first cluster rewritten after every chunk, as inode writeback finds the inode
  dirty, so the medium sees intermediate sizes, about once a lag.
- FSInfo is a metadata page written whenever the free count changes, as Linux marks its FSInfo
  inode dirty on every allocation and free (v6.12 fs/fat/fatent.c `mark_fsinfo_dirty`, written by
  inode.c `fat_write_inode` through misc.c `fat_clusters_flush`): it lands with the metadata lag,
  not only at sync or unmount as the handoff guessed.
- Under contention pages land later than they fall due: the flusher's writes are USB commands
  like any other, behind the car's own and slowed by the card.
- A power cut drops what has not landed; the volume stays dirty.

On the fake drive the model puts a cluster's data on the medium before its FAT entry for 97.2%
of clusters (35,778 against 1,047; `writeback_lands_data_in_batches_and_holds_the_metadata_back`),
against the 98.2% the shared-reading plan measured on a drive (18,460 clusters over 178 FAT
sectors), counted the drive trace's way: a cluster's first data landing against the first landing
of its FAT1 sector after the data was written.

### Writes into a window

Against 728b71f, in what is now `external_oxidrive_refuses_car_writes_in_a_window` ("The window
gate" has the fixed build). The car (2 s polls, the vfat-like writer)
records two saved events (`event.mp4`, 2 MiB each), idles, and starts a 16 MiB Sentry clip (a
4.1 MB/s camera) the moment oxidrive's policy takes the card for a window, 72 s into the run.
The first shared pass took the older event; the window is for the newer.

esp_tinyusb answers TEST UNIT READY with MEDIUM NOT PRESENT once oxidrive holds the card
(oxidrive's vendored esp_tinyusb 2.2.1, components/esp_tinyusb/tinyusb_msc.c:1195-1210,
`tud_msc_test_unit_ready_cb`), but its WRITE(10) callback defers the write to the card whatever
the mount point (1267-1285, `tud_msc_write10_cb`, and `msc_storage_write_sector_deferred`,
346-385; READ(10) the same, 1253-1262). So every WRITE(10) the car sends before its next poll is
answered GOOD and written to the card under oxidrive's FatFs. The golden `usbmsc-p4-window` pins
the same callbacks in IDF's tusb_msc example, deterministically and without a network ("USB",
"Faults").

The window and the car's writes interleave differently in each run, because smbd and mosquitto
answer in host time. Eight runs:

| run | window | the car's WRITE(10)s while oxidrive held the card | the car | the window |
| --- | --- | --- | --- | --- |
| 1 | 6.0 s | 358, 7,544 KiB | its poll 1.8 s after the take saw MEDIUM NOT PRESENT; clip cut: 230 clusters orphaned | 1 failed, nothing archived |
| 2 (bus logged) | 11.1 s | 295, 6,179 KiB | saw it 1.5 s after the take; 188 orphaned | the newer event uploaded, equal to the car's file |
| 3 | 1.0 s | 205, 4,229 KiB | never saw it | 1 failed |
| 4 | 1.0 s | 205, 4,229 KiB | never saw it | one of the two events missing from its listing; finished "clean", nothing archived |
| 5 | 1.0 s | 203, 4,228 KiB | never saw it | as run 4 |
| 6 (bus logged) | 1.0 s | 202, 4,164 KiB | never saw it | a CMD17 the card did not answer while programming one of the car's writes (`list SavedClips failed: I/O error`) |
| 7 (bus logged) | 10.9 s | 298, 6,244 KiB | saw it 1.5 s after the take; 190 orphaned | the newer event uploaded |
| 8 (bus logged) | 1.0 s | 202, 4,164 KiB | never saw it | a CMD17 unanswered the same way while listing SavedClips; one event missing from the listing, finished "clean" |

In six of the eight the window did not archive the event it opened for: three reported a failure,
three ended "clean" with an event missing from the listing. oxidrive wrote no block itself in any
of them (it writes in a window only when it makes TeslaCam; reclaim is off in this build). When
the car's poll fell inside the window (runs 1, 2, 7) the clip it was writing was left on the card
with its entry at size 0 and no cluster and its clusters orphaned (fsck_msdos: 188 to 230 orphaned
clusters, FSInfo's free count off by as many); when it did not (the other five, whose windows
ended within 1.0 s) the car never knew, its clip closed, and fsck is clean.

Why oxidrive's reads fail, seen in runs 6 and 8, whose SD bus was logged (`ESP_EMU_DEBUG=sdmmc`):
FatFs's CMD17 came while the card was programming one of the car's writes (`prg`), and a card in
`prg` answers no read (SD Physical Layer Simplified Specification v9.10 §4.8; "The card" above;
the 0x107 and status 0x400E00 the reference board reported for reads during USB writes). IDF's
`sdmmc_read_sectors` sends CMD17 or CMD18 without waiting for the card to be ready. FatFs turns
the failed read into EIO (IDF v5.5.3 fatfs/vfs/vfs_fat.c:325-326). Where it hits a folder's
listing, oxidrive's window source skips the entry as unreadable and goes on (pipeline/src/
fs_source.rs:198-202, the same in `clips_in`, 103-106), so the event drops out of the window
without a failure being counted; the window source's `revalidate` takes a size match as enough
because "while the card is held exclusively nothing else writes to it" (fs_source.rs:228-243),
which the car's writes contradict. Runs 1 and 3 were not logged and their cause is the likely one,
not a proven one. Two more firmware facts from the sources: oxidrive's write tap records nothing
while it holds the card (app/src/storage/card.rs:603-675, `__wrap_sdmmc_write_sectors`, gates the
dirty log and the quiet timer on `car_has_card()`), so the car's writes in a window are in
neither once the card is back; and a window that fails or finds nothing ends within about a
second, shorter than a 2 s poll.

### A car that keeps its FAT

`external_oxidrive_caching_car_through_windows_it_sees`. The car keeps its metadata between polls
and polls every 2 s; it records two events, oxidrive's policy takes the card at 61.2 s (16.0 s,
two uploads), `force_open` again at 96.3 s (16.0 s, two uploads). The car saw both (four medium
changes) and read the volume again after each; between them its cache answered 369 metadata reads
and missed 22. oxidrive wrote no block in either window, every upload equals the car's file, every
clip is on the card as the car wrote it, and fsck is clean: with 728b71f a caching car is safe
across a window it sees, because the window writes nothing on a card that has TeslaCam.

### Windows shorter than the poll

Against 728b71f, in what is now
`external_oxidrive_window_between_polls_reaches_a_caching_car_as_a_unit_attention`. A fresh card
(FAT32 and nothing on it); the car keeps its metadata and looked for TeslaCam before the window,
so its cache holds the root without TeslaCam and the first FAT sector with every cluster free.
oxidrive's first
window (61.3 s) finds no TeslaCam on a card this empty and makes it (the TeslaCam self-heal:
pipeline/src/fs_source.rs `ensure_layout`): the FAT sector in both FATs, the four directory
clusters (64 blocks each, FatFs clears a new directory's cluster), FSInfo after each, then gives
the card back after 1.40 s with nothing to archive. Then the car makes TeslaCam/RecentClips and
records a clip:

| car's poll | saw the window | TeslaCam afterwards | fsck |
| --- | --- | --- | --- |
| 5 s | no | RecentClips only: the car's TESLACAM took the same root slot and cluster 3, its stale FAT sector freed oxidrive's other three clusters again | clean |
| 2 s | yes: a poll fell inside the 1.4 s window (a 2 s poll does with probability 0.7, by arithmetic) | RecentClips, SavedClips, SentryClips | clean |

The car that missed the window erased oxidrive's SavedClips and SentryClips and left nothing for
fsck to find. Stock esp_tinyusb never answers UNIT ATTENTION, so a car that misses the medium
going never learns it changed; Linux's own media poll is off by default and udev's is 2 s, and a
window that has nothing to do, or fails, lasts 1.0-1.4 s.

### The first epoch after a mount

`external_oxidrive_reads_through_the_cars_fsck_burst_after_a_reset`. A 64 GiB card two earlier
boots left dirty and torn (`previous_boots`, below), so its FATs are 8 MiB each. The car runs
fsck at the mount and writes the FATs whole (`FsckWrite::Bulk`), records sets of four camera clips
(`Tesla`, below), and forces a shared pass as soon as oxidrive's broker session is up; when the
pass begins to upload a clip, the car resets the port without dropping VBUS
(`UsbHost::reset_device`) and runs the same fsck again:

- The mount's burst ran from 0.29 s to 3.53 s (both FATs read and written whole, 16 MiB each way;
  the copies differed after the power cuts, 4 sizes cut). oxidrive's first pass began at 9.2 s,
  5.7 s after it, and only that early because it was forced as soon as its broker session was up:
  unforced, the first pass comes a minute after boot (`shared_interval_secs`). The first epoch
  waits out the mount's burst.
- The pass began uploading a Sentry clip at 9.27 s; the reset's burst ran from 9.58 s to 12.82 s.
  At 9.60 s the pass's walker failed (`list SavedClips failed: directory walk failed`) and the
  pass ended having archived nothing. Nothing it archived differs from the car's file, and fsck
  is clean afterwards. A run before the SD model's auto-stop fix (above) read through more of the
  burst: oxidrive's watch caught the FAT writes and discarded the two clips it was reading
  (`shared: the car touched ... while it was read`), then the walker failed the same way.

oxidrive 728b71f sets `auto_mount_off`, so the port reset did not take the card from it (as the
tusb_msc example's reset did, "Faults").

### Writeback, shared passes and windows

`external_oxidrive_archives_no_torn_clip_from_a_writeback_car`, on the writeback model above:

- **The card**: 4 GiB, filled with one file but for 160 MiB (`fill_volume`), so the car's
  rotation hands freed clusters straight to the next clip, as on a full card.
- **The recorder** (`Tesla`): a set of four 384 KiB camera clips every 6 s into RecentClips,
  written together, a ring of 12 sets with the oldest deleted; every fifth set an event copying the
  last two sets into SentryClips or SavedClips at the bus's speed; at most 8 events kept, the oldest
  deleted. The clips are `mp4_bytes`: `ftyp`, an `mdat` that states its size, `moov` last.
- **Earlier boots**, offline (`previous_boots`): four boots of 90 s on the card, each starting with
  an fsck when the volume is dirty (`FsckWrite::Changed`) and ending in a power cut 0-6 s into the
  next set, which lost 31, 78, 96 and 35 dirty pages. Their fscks found 4 to 12 files to cut, 8 to
  12 broken chains, 48 to 353 lost clusters, and once two FATs that differed.
- **This boot**: oxidrive boots on that card; the car's fsck at the mount cuts 8 files, frees 305
  lost clusters and rewrites FAT2 from FAT1 (the copies differed); then the car records 120 s,
  stays quiet until a window has come and gone, records 120 s more, and stays quiet with
  `force_open` until another. oxidrive passes every minute meanwhile. The medium is copied every
  second while the car records.

One run, 447 s emulated (1,215 s on the host):

| | |
| --- | --- |
| written | 1,584 chunks, 104 MB; 268 clips deleted by the rotation |
| landings | data 1,586: written to landed median 2.53 s, 99th percentile 4.94 s, max 5.00 s; metadata 183: median 32.4 s, max 35.0 s. Behind their schedule by at most 20 ms: the shared passes' contention never pushed a landing late |
| shared passes | 9; 72 clips uploaded; 2 stopped early on `card busy with the car` (a read the card did not answer while programming), 2 clips discarded by the watch because the car wrote them while they were read, 2 walks of a folder failed |
| windows | 208.3 s (1.3 s, nothing new), 414.3 s (22.2 s, 17 clips), 440.8 s (1.3 s, forced, nothing new); the car's 2 s poll fell inside all three, by phase: a 2 s poll meets a 1.3 s window with probability 0.65 (arithmetic) |
| uploads against the car's files | 72 by shared passes and 17 by windows, every one equal to the car's file: **no torn or prefix clip archived as finished** |

The 238 copies of the medium, judged as a reader without a dirty log would judge them (each Saved
and Sentry clip present: what each rule accepts, and how many of those differ from the clip the car
was recording into that file):

| rule | clips accepted | torn among them |
| --- | --- | --- |
| any file with a size | 15,503 | 959 |
| MP4 boxes complete (oxidrive's complete.rs) | 14,544 | 0 |
| and the entry agreeing with its chain (oxidrive's shared.rs `structural`) | 14,544 | 0 |
| and nothing written to its sectors for 120 s (the settle gate, from the card's write log) | 3,632 | 0 |

Every torn clip on the medium in this run was one being written: its entry's size, landed late
from inode writeback, falls short of the `mdat` size the clip states, so the box walk runs past
the end. The state that would pass the MP4 check, a final size and the `moov` on the medium with a
middle chunk not landed, needs the entry's page to be one of the 4% not held back and to land
before a data batch; it is reachable in the model and did not occur in these copies. Earlier
boots' torn files did not survive the car's fsck as complete-looking clips: fsck cuts a file to its
chain, and a cut clip's `mdat` runs past its end. So the counts say the MP4 check and the watch were
enough for this model and run; they do not show that the settle gate, which oxidrive ships off,
is unnecessary, and a recorder that writes `moov` first or patches `mdat`'s size at the end would
change the first rule's column. The settle gate's column uses the card's own write log, which also
records the car's writes during a window; oxidrive's dirty log does not ("Writes into a window").

`OXIDRIVE_SNAPSHOT_DIR` keeps the copies with `truth.tsv` (every file the car made, its size and
the CRC-32 of its clip), a table per copy of what the car knew of each file, and `writes.tsv` (every
run of blocks written to the card, with its time), for judging other completeness rules offline.
The run is slow: the emulated firmware polls CMD13 through every programming time the car's many
small writes cause, about 2.7 times real time with the pair running.

### The window gate

oxidrive's fix for findings 1 to 4 below (teslausb-p4 63024af and 5633fc2, reviewed through
51ae822; its README, "The window gate"): its esp_tinyusb copy answers TEST UNIT READY, READ
CAPACITY, READ(10) and WRITE(10) with NOT READY, MEDIUM NOT PRESENT while the application holds the
card; a take closes that gate and then waits for the reads and writes it had already admitted, a
WRITE(10) being acknowledged before its deferred write reaches the card, and a take that fails
gives the card back inside the component; giving the card back leaves UNIT ATTENTION 6/28/00 for
the car's first command, reported once, the REQUEST SENSE after a refused READ(10) or WRITE(10)
carrying it (`CONFIG_TINYUSB_MSC_UNIT_ATTENTION`, shipped on); and a readdir error, which ends that
directory's listing, counts as a failure. The car treats the unit attention as Linux does ("The
car"). The emulated build is 51ae822 (`OXIDRIVE_CURRENT_DIR`); the first runs, on 5633fc2, met
the same mechanisms.

| test | 728b71f | 5633fc2, 51ae822 |
| --- | --- | --- |
| `external_oxidrive_refuses_car_writes_in_a_window` | fails: 207 of the car's WRITE(10)s, 4,294 KiB, reached the card in a 1.03 s window, which finished with failures after a read error; the car never saw it | passes: the car refused at its first command after the take, none of its writes on the card, the newer event archived, the window clean |
| `external_oxidrive_window_between_polls_reaches_a_caching_car_as_a_unit_attention` | fails: the 5 s car met no unit attention, missed the 1.40 s window and erased SavedClips and SentryClips | passes: both cars met the unit attention and kept oxidrive's folders; fsck clean |
| `external_oxidrive_window_counts_a_listing_cut_short_as_a_failure` | fails: the error passed over, the window finished clean | passes: `SavedClips: listing cut short; read errors: 1`, the window finished with failures |

`external_oxidrive_caching_car_through_windows_it_sees` and
`external_oxidrive_reads_its_temperature_after_a_restart` pass on 5633fc2 as well (not re-run on
51ae822): the caching car
saw its four media changes, each window's NOT READY and then its unit attention. The writeback
and fsck-burst tests were not run on it.

**Writes into a window.** With 51ae822 the window held the card from 72.79 s to 83.80 s (11.0 s; on
5633fc2, 72.52 s to 83.43 s). The car's Sentry clip began at the take; its first command, the READ
that looks up SentryClips, was refused with 2/3A/00 within 1 ms, so nothing of the clip reached the
card, and no block was written in the window by the car or by oxidrive. The car saw NOT READY at its
poll at 74.30 s and UNIT ATTENTION 6/28/00 at its first poll after the window (84.30 s), read the
volume again and recorded the next event. The window uploaded the newer event (the first shared pass
had the older), both equal to the car's files, and finished clean with no read error; fsck is clean.
Two earlier runs of 5633fc2, stopped by the test's own bookkeeping, held windows of 11.0 s and 12.9
s with no car write on the card either. With 728b71f the window lasted 1.03 s, took 207 of the car's
WRITE(10)s (4,294 KiB), met a read error and archived nothing for its event, and the car never saw
it; the test fails at its first assertion, the Sentry clip not being refused.

**Windows shorter than the poll.** On the fresh card the self-heal window lasted 1.40 s under both
cars on 5633fc2 (1.50 s and 1.40 s on 51ae822, with the same outcome). The 5 s car had no poll
inside it (no NOT READY); its first command after the window, a poll, met the unit attention, and it
read the volume again: TeslaCam holds RecentClips, SavedClips and SentryClips, and fsck is clean.
The 2 s car saw NOT READY at one poll inside the window, the unit attention at the next, and kept
the same three folders. With 728b71f the 5 s car erased SavedClips and SentryClips ("Windows shorter
than the poll"). Linux v6.12 reaches the same media change from either: a UNIT ATTENTION at a TEST
UNIT READY sets `sdev->changed` (scsi_lib.c:2391-2399), and so does a medium that went and came back
(sd.c:1765-1775). So holding the card for a poll interval after a window that wrote it buys nothing
for a polling Linux host, and oxidrive does not.

**A listing that cannot be read.** With the gate the car no longer keeps the card programming under
FatFs, which was what failed the window's reads, so `break_dir_chain` makes the error instead: two
saved events in SavedClips, the rest of its first cluster filled with deleted entries and its FAT1
entry zeroed, so FatFs's readdir fails when it steps past the cluster (ff.c:1773-1774, FR_INT_ERR).
A readdir error ends that directory's listing (std's `ReadDir` stops after one; IDF's
`vfs_fat_readdir` returns no entry), so what follows it is missing from the pass. With 51ae822 the
window (72.8 s to 83.8 s) logged `SavedClips: listing cut short; read errors: 1` and finished with
failures, having uploaded the newer event (the first shared pass had the older); 5633fc2, which
called it `1 directory entries unreadable`, did the same. With 728b71f the same window uploaded the
same and finished clean.

How a Tesla takes UNIT ATTENTION 6/28/00 is not known: the car here takes it as a media change
because Linux's sd does, and the car's userspace decides the rest ("Handoff", what part 1 leaves
open). The risk oxidrive names for board day: sd fails the I/O that meets the unit attention and
refuses all I/O after it until the next media check (scsi_lib.c:803-811, sd.c:1340-1343), which
only a media poll (udev's 2 s; the kernel's own interval is 0) or a reopen runs, so a car that
never polls and keeps its filesystem mounted would get EIO on every write after the first window.
The car here polls.

### Findings for oxidrive

1. **A car's writes reach the card in a window and break the window.** esp_tinyusb's READ(10) and
   WRITE(10) callbacks serve the card whoever holds it (tinyusb_msc.c:1253-1285 in oxidrive's
   2.2.1 copy); only TEST UNIT READY looks. A car that starts writing within a poll interval of
   the take writes the card under FatFs. The card's programming makes FatFs's reads fail (IDF's
   reads do not wait for a ready card): in six of eight runs the window did not archive the event
   it opened for. When the car's poll falls inside the window, the clip it was writing is left
   orphaned on the card; when it does not, the window has ended in about a second and the car
   never knew. Possible fixes: answer READ(10) and WRITE(10) with NOT READY / MEDIUM NOT PRESENT
   while the app holds the card (the car then learns at its next command); make the app's reads
   wait for the card to be ready. **Fixed** by the first, in teslausb-p4 5633fc2 ("The window
   gate").
2. **A read error while listing drops an event without a failure.** The window source skips an
   unreadable directory entry and goes on (pipeline/src/fs_source.rs:198-202, 103-106); in three
   of the eight runs a window ended "clean" with an event missing from its listing. The event is
   listed again next time, so nothing is lost, but the window reports a clean drain it did not
   make; and the window's `revalidate` relies on nobody writing the card while it is held
   (fs_source.rs:228-243), which finding 1 contradicts. **Fixed** in teslausb-p4 63024af and
   ae63d65: a readdir error ends the directory's listing rather than skipping one entry, so it is
   counted as a listing cut short and the window finishes with failures; `revalidate`'s
   assumption holds once the gate refuses the car.
3. **A window shorter than the car's poll is invisible to the car, and what it writes can be
   overwritten.** Shown for the TeslaCam self-heal on a fresh card: a car polling every 5 s that
   had read the root erased SavedClips and SentryClips, silently. Windows that fail or find
   nothing last 1.0-1.4 s, under udev's 2 s. Possible fixes: answer UNIT ATTENTION (6/28/00, NOT
   READY TO READY CHANGE) at the car's first command after the card comes back, which Linux takes
   as a medium change (v6.12 drivers/scsi/scsi_lib.c:2390-2400 `scsi_test_unit_ready`,
   sd.c:1767-1775); or keep the card for several poll intervals whenever the window has written it.
   **Fixed** by the first, in teslausb-p4 5633fc2; the window is not held longer.
4. **The car's writes in a window are invisible to the dirty log and the quiet timer** (card.rs:
   612 and 645 test `car_has_card()`). With the settle gate off (`SHARED_SETTLE_MS` 0, this build)
   nothing here depends on it; with it on, a clip written during a window would count as settled.
   **Closed** by finding 1's fix: the car's writes no longer reach the card in a window.
5. **Torn clips from writeback were not archived** in the writeback run: 89 uploads, all equal to
   the car's files; the MP4 check rejected every torn clip on 238 copies of the medium, and the
   watch caught the car writing two clips while they were read. The state the MP4 check would pass
   is rare in the model and did not occur; the counts do not argue for or against the settle gate.
6. **An fsck burst during a shared pass costs the pass, not the archive**: the pass archived
   nothing during it and nothing torn, and its walker failed outright, which a ready check before
   each read (finding 1) would also address. The first pass after boot comes well after the car's
   fsck at the mount.

## Car write latency under a shared reader

Tesla reports a drive as too slow when its writes take too long; how long is not known. While the
car records, oxidrive's shared pass reads the card through the SDMMC host the car's writes go
through (slot 0), which ESP-Hosted's SDIO link shares (slot 1). The firmware's shared-reading plan
asks for the car's per-WRITE(10) latency with reader chunks of 16, 32, 64 and 128 KiB, and knows
that a read meeting the card while it programs the car's last write is not answered (`0x107`,
card status `0x400e00`). `cli/tests/oxidrive_latency.rs` measures both (milestone 4c, item 5):

- A reader transfer adds at most its own time on the bus to a car write that meets it: 0.8, 1.7,
  3.4 and 6.7 ms for 16, 32, 64 and 128 KiB at 4 bits and 40 MHz. The cost is linear in the
  chunk, with no knee, and the reader's rate does not depend on it (its 1 MB/s pacer and SMB set
  it).
- With the card programming for 1 or 2 ms after each write, 23 of 24 passes met that race and
  stopped, after at most 7.8 MB. When the race hits the boot-sector read a pass starts
  with, shared reading stays off until the next boot. No chunk size avoids it.
- The car's worst waits, 50 to 250 ms, were not bus time. The reader runs in the firmware's main
  task, pinned to core 0; it held the SD host's request mutex while libsmb2's software AES (SMB3
  signing) held core 0, and the car's USB task waited for the mutex with core 1 idle. More reader
  commands mean more chances: the firmware as it stands reads 512 bytes at a time, because its
  buffer is not cache-line aligned, and was hit hardest. With the SMB thread pinned to core 1
  there were none.
- oxidrive's `fix-shared-reader` build shares a lock with the car's writes, retries a refused
  read, reads 32 KiB at a time and holds the card at priority 23: at 0, 1 and 2 ms no read went
  unanswered, no car command took 100 ms, and every pass archived both events ("The fixed
  reader", asserted by `external_a_shared_pass_completes_without_racing_the_car`).

### What sets the size of a shared read

From oxidrive's source (not in this repository):

- `SharedCardSource::open` hands the archiver a `ChainReader`, which reads one cluster per call
  into a cluster-sized `Vec<u8>` (`app/src/storage/shared_source.rs`). The clusters are 32 KiB, as
  on the owner's card.
- Each read first pays a token bucket of 1 MB/s (`DEFAULT_READ_BYTES_PER_S`). Refill is capped at a
  second's worth, so after an SMB stall the reader can read up to 1 MB back to back.
- The archiver fills a 1.5 MiB transfer buffer from it (`size_buffer`: the SMB sink's
  `transfer_size()`, `MAX_WRITE` 64 KiB times `INFLIGHT` 24, in `pipeline/src/archiver.rs` and
  `oxidrive-smb/src/sink.rs`) and queues it for the SMB thread, three deep. That sets how much is
  read between hand-offs, not the size of a card read.
- `Vec<u8>` promises no alignment, and IDF v5.5.3's `sdmmc_read_sectors` reads a buffer that fails
  the cache-line check one block at a time through a bounce buffer (`sdmmc_cmd.c`). In the emulator
  every shared read of the current firmware is a one-block CMD17: its chunk is 512 bytes. The
  archiver's own buffer is aligned for that reason (`Aligned64`); the reader's is not.
- The FAT walker (listing, chain walks) reads single sectors.

The sweep's builds change only the reader: a `READ_CHUNK` constant and a 64-byte-aligned chunk
buffer in `ChainReader` (contiguous clusters in one read), one build per size. The 32 KiB build
reads what the current firmware reads, aligned. None of the builds is in this repository.

### Method

`external_car_write_latency_under_a_shared_reader` and the control
`external_car_write_latency_alone` run the rig of "Services" against `OXIDRIVE_DIR`'s build:

- **The card**: a 4 GiB FAT32 image with 32 KiB clusters, the bus timed on both slots (the card and
  the SDIO link, as `--sd-timing`), programming for `OXIDRIVE_SD_PRG_US` after each write. 1000
  and 2000 µs are assumptions until a card is measured; 0 is a diagnostic arm in which the race
  cannot happen.
- **The car**: enumerates, records three 4 MiB saved events (`TeslaCam/SavedClips/<stamp>/event.mp4`),
  then one long RecentClips clip at a 4.1 MB/s camera, 64 KiB due every 16 ms (`write_clip`). Each
  chunk is a WRITE(10) of 128 blocks and the vfat-like FAT update after it: a one-block READ(10)
  and two one-block WRITE(10)s.
- **The pass**: forced with `force_open` once the MQTT session is up, at 9.0 s. The firmware holds
  the newest event back, so the pass has two 4 MiB clips to read. The control forces nothing; the
  firmware's own first pass is due at 60 s, after the window.
- **The measurement**: `Msc::times` times every car command from its CBW to its checked CSW, over
  the 40 s from 0.5 s after the force. A tap on each SD device logs every command at its emulated
  time, so each WRITE(10) carries the bus time the firmware's own card reads (the reader) and the
  SDIO link took inside it.
- Each cell ran twice (32 KiB with no programming time three times): 39 runs of 2 to 4.5 minutes,
  four at a time.

What a WRITE(10)'s latency covers: esp_tinyusb takes it in 32 KiB halves and defers each half's
card write to the TinyUSB task (`tud_msc_write10_cb`, `tinyusb_msc.c`), so the CSW follows the
first half's CMD25 and its programming, and the second half is written after the CSW. Alone, a
64 KiB WRITE(10) takes 2.62 ms with no programming time, 3.62 ms with 1 ms, 4.62 ms with 2 ms. A
delay to the second half shows in the car's next command.

The timed bus sent the auto-stop's CMD12 to the card at the command's start, so a multi-block
write's programming was over before its data was in. It now goes at the end of the data phase
(`a_timed_multi_block_write_programs_from_its_auto_stop`); the runs with a programming time are
from after that fix.

### Results

No programming time, two runs each ("a-b" when the runs differ). "Met the reader": data WRITE(10)s
with reader bus time inside them, and the most of it in one.

| reader | transfers in the window | data WRITE(10), ms: median / p90 / p99 / max | met the reader | those, ms: median / p99 | car commands of 100 ms or more | worst car command | car behind the camera | reader over the pass |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| none (control) | | 2.63 / 2.63 / 2.63 / 2.63 | | | 0 | 2.63 ms | never | |
| current firmware, 512 B | 2880 CMD17 | 2.63 / 2.63-2.75 / 2.87-2.88 / 247 | 37-39 (1.5%), 0.57 ms | 2.88 / 247 | 7-8 | 247 ms | 178-196 chunks, up to 1.8 s | 1.8 MB at 0.29 MB/s, then stopped by its own guard |
| 16 KiB | 496 CMD18 | 2.62 / 2.63-2.75 / 3.25 / 3.6-252 | 106-117 (4.2-4.7%), 1.7-2.4 ms | 2.88 / 3.50-4.25 | 4 | 249-252 ms | 71-80, up to 0.30 s | 8.5 MB, 0.20 MB/s |
| 32 KiB | 250 CMD18 | 2.62 / 2.62-2.75 / 2.87-3.37 / 4.13-4.25 | 49 (2.0%), 1.67 ms | 2.62-3.37 / 4.13-4.25 | 2 | 171-242 ms | 39-51, up to 0.23 s | 8.5 MB, 0.20 MB/s |
| 64 KiB | 127 CMD18 | 2.62 / 2.62 / 2.75 / 5.88 | 25-27 (1.0%), 3.37 ms | 5.12 / 5.88 | 2 | 145 ms | 29, up to 0.14 s | 8.7 MB, 0.20 MB/s |
| 128 KiB | 66 CMD18 | 2.63 / 2.63-2.75 / 2.75 / 9.25 | 9-10 (0.4%), 6.74 ms | 9.25 / 9.25 | 0 | 15.4 ms | 0-1, 22 ms | 8.9 MB, 0.21 MB/s |
| 32 KiB, SMB thread on core 1 | 250 CMD18 | 2.62 / 2.62-2.63 / 2.75 / 4.12 | 40-41 (1.6%), 1.74 ms | 2.62 / 4.12 | 0 | 5.4 ms | never | 8.5 MB, 0.20 MB/s |

Every data WRITE(10)'s minimum is 2.62 ms. The longest run of consecutive data WRITE(10)s meeting
the reader was 23-25 for 512-byte reads, 13-21 for 16 KiB, 4 for 32 KiB and 1-3 above. The reader
read at its pacer's 1 MB/s in bursts (median gaps 16, 33, 65 and 131 ms between transfers) and
otherwise waited on SMB: 0.2 MB/s over the pass, both clips uploaded.

With a programming time, the race ends the pass first. "Ended" gives each run's first unanswered
firmware read and what was read before it:

| reader | programming | data WRITE(10), ms: median / p99 / max | car commands of 100 ms or more | the pass ended |
| --- | --- | --- | --- | --- |
| none (control) | 1 ms | 3.62 / 3.63 / 3.63-3.75 | 0 | its own pass at 60.2 s: the boot-sector read (CMD17 LBA 0) in both runs; shared reading off until reboot |
| none (control) | 2 ms | 4.63 / 4.75 / 4.75-4.87 | 0 | its own pass at 60.4 s: a FAT read while listing, in both runs |
| current firmware, 512 B | 1 ms | 3.62 / 3.63 / 3.63-3.75 | 0 | a FAT read while listing, 9.2 s, nothing read (both runs) |
| current firmware | 2 ms | 4.63 / 4.75 / 4.75 | 0 | a clip block at 9.35 s after 0.16 MB; the root directory at 9.22 s |
| 16 KiB | 1 ms | 3.62 / 3.63-3.75 / 4.25-240 | 0-1 | 12.1 s after 3.0 MB; 10.8 s after 1.6 MB |
| 16 KiB | 2 ms | 4.62 / 4.75-4.88 / 4.75-227 | 0-3 | 9.6 s after 0.4 MB; no race, stopped at 12.0 s by its own guard after 2.6 MB |
| 32 KiB | 1 ms | 3.62 / 3.63-3.75 / 3.75-5.12 | 0-1 | 11.6 s after 2.4 MB; a FAT read at 9.2 s, nothing read |
| 32 KiB | 2 ms | 4.62 / 4.75-4.87 / 6.25 | 0 | 35.7 s after 4.7 MB, one clip uploaded; 35.6 s after 4.3 MB, one clip |
| 64 KiB | 1 ms | 3.62 / 3.63-3.75 / 3.75-7.13 | 0-2 | 32.4 s after 4.3 MB, one clip; 10.9 s, nothing read |
| 64 KiB | 2 ms | 4.62 / 4.75-4.87 / 7.25 | 0 | 35.7 s after 4.6 MB, one clip; 38.1 s after 7.8 MB, one clip |
| 128 KiB | 1 ms | 3.63 / 3.63 / 9.25 | 0 | 10.0 s after 0.7 MB (both runs) |
| 128 KiB | 2 ms | 4.63 / 4.75 / 4.75 | 0 | a FAT read at 9.2 s; 9.8 s after 0.5 MB |

Every WRITE(10) in the windows ended with good status, and every clip read back off the image as
the car wrote it.

### The stalls: a priority inversion

During each long stall the card bus is idle: no command from the reader or from the car for 50 to
250 ms. Sampling both P4 cores' PCs (`LATENCY_PCS=1`) puts core 0 in libsmb2's AES-CMAC
(`smb3_aes_cmac_128`, `AES128_ECB_encrypt_reference`) and core 1 in `esp_cpu_wait_for_intr` for
the whole stall; interrupts stay enabled on both, the SDMMC interrupt on core 0 and USB-OTG's on
core 1. The stall ends when the SMB thread next runs on core 1. The sequence, from the bus and the
sources:

1. The reader, in the main task (priority 1, pinned to core 0), sends a command inside
   `sdmmc_host_do_transaction`, which holds the host's request mutex until the command's event is
   handled (IDF v5.5.3 `sdmmc_transaction.c`).
2. The command completes; its interrupt makes the main task ready, but the SMB thread (a pthread at
   priority 5, no affinity) is running on core 0, so the main task waits.
3. The car's next command reaches the TinyUSB task (priority 24, pinned to core 1), which blocks on
   the request mutex. FreeRTOS raises the main task's priority to 24 (`xTaskPriorityInherit`,
   `tasks.c`) but does not make core 0 reschedule, and core 0's tick switches only for a task the
   tick itself unblocked, for time slicing at the running task's priority, or for a pending yield.
4. Core 0 runs the SMB thread's signing until it blocks, and only then the main task, which
   releases the mutex.

More reader commands give step 2 more chances, which is why 512-byte reads stall most and 128 KiB
not in these runs. The pacer does not help: the window opens while the reader holds the mutex,
whatever the budget. Two ways out, the first measured: keep libsmb2 off core 0 (the SMB
thread pinned to core 1: no stall, the car never behind, the same 0.20 MB/s), or run the reader
at a priority above the SMB thread. The signing load exists because the test's samba requires
signing ("Services"); whether the owner's NAS does is not known here.

### The bus race

A reader's command that meets the card programming the car's last write is not answered: on the
bus, a CMD18 with no response, and the reader's own CMD13 after it answered `0x400e00`
(ILLEGAL_COMMAND in prg), the card's status the board reported. IDF logs it as
`sdmmc_read_sectors_dma: sdmmc_send_cmd returned 0x107, status 0x400e00` (`sdmmc_cmd.c`); that line
goes to oxidrive's syslog hook (`esp_log_set_vprintf`), not to the UART this test reads. What
oxidrive does next:

- In a pass, `CardBlocks` latches the failure: every later read in the pass is refused without
  touching the card, the clip being uploaded fails (its `.part` stays on the share to resume), and
  the pass ends. It logs `shared: ... stopped early: card busy with the car` only when it uploaded
  or refused something; otherwise only its `scan: shared` line marks the end. The next attempt is
  the next pass, 60 s later.
- `main.rs` reads the boot sector again (`Card::geometry`) before every pass. If that read meets
  the race, the firmware logs `shared: no FAT geometry; not reading alongside the car` and turns
  shared reading off until the next boot. Both control runs at 1 ms lost it that way at their
  first pass.
- The card recovers by itself: ILLEGAL_COMMAND clears with the response that reports it, and the
  car's writes, whose CSWs went out before their second halves reached the card, all landed.

The driver's writes return only after CMD13 reports the card ready, but each poll is its own
transaction, so the request mutex is free between polls and the reader's command lands in prg. With
the car writing four times per 16 ms (two halves of 32 KiB, two FAT sectors), the card programs
for about a quarter of the time at 1 ms, half at 2 ms, and no pass here got past 7.8 MB.

### What the data support

The chunk's own cost is the time one transfer holds the bus, paid by the car write that meets it:
+0.8, +1.7, +3.4, +6.7 ms. It grows linearly with the chunk; nothing bends. A bigger chunk buys
fewer commands, which matters here only through the inversion and the race, and does not raise
the reader's rate, which is the pacer's. 32 KiB, one cluster in an aligned buffer, keeps a colliding
write under 2 ms over its baseline in one command per cluster; 64 KiB doubles that; 128 KiB nearly
quadruples a 64 KiB write's time with no programming time (9.25 against 2.63 ms). Without Tesla's
threshold, 32 KiB is the size these runs support, 64 KiB if each command costs more on silicon.

Three things matter more than the chunk, in order: the race (the reader has to wait for the card,
or share a lock with the write tap that covers the write and its CMD13 polls, as the plan
proposes; and a refused read should not end the pass, or shared reading for the boot), the
inversion above, and the reader's alignment, without which a 32 KiB read is 64 commands.

### Limits

- **Lower bounds.** The CPUs run one instruction a cycle with no cache or PSRAM stalls, so the
  signing, the drivers and TinyUSB all take longer on silicon, and the stalls would last longer.
- **The card.** Its read access time and the IDMAC's pace are not timed: a read holds the bus for
  its wire time only. The programming time is an assumption, the same after every write; a real
  card's busy time varies and can run far longer (internal garbage collection). It wants a value
  measured on the owner's card.
- **The car.** 64 KiB WRITE(10)s every 16 ms with a vfat-like FAT update after each, not Linux's
  writeback (larger writes back to back, metadata held back), so how often a write meets the reader
  and how often the card programs are this car's.
- **The network.** The NAT's 5840-byte window holds SMB to about 0.2 MB/s ("Upload throughput"),
  so the reader waits on SMB most of a pass and met at most 5% of the car's data writes. At the
  board's 2 MB/s it would read at its 1 MB/s pace and meet about five times as many, at the same
  cost per write.
- **Repeats.** The servers answer in host time, so two runs of one cell differ in which commands
  collide (16 KiB's worst data write: 3.6 ms in one run, 252 ms in the other). Counts of stalls are
  small numbers.
- **Tesla's threshold** can only be found on the car, with the firmware plan's delay-injection probe.

### The fixed reader

oxidrive's branch `fix-shared-reader` (0518b69, not in this repository) changes the reader along
the lines above:

- **A lock with the car's writes.** One FreeRTOS mutex. The firmware's write tap (its linker wrap
  of `sdmmc_write_sectors`) holds it across each car write, which returns only once CMD13 reports
  the card back in TRAN, and the reader holds it for one transfer of at most 32 KiB, or one
  sector of an unaligned buffer. A car write waits at most 10 ms for it, then writes without it.
- **Refused reads retried.** A timeout, a busy lock or a garbled answer, after 2, 4, ... 256 ms;
  three reads that fail every try in a row, or eight in a pass, end the pass and nothing more,
  and each pass in a row ended that way doubles the wait for the next. The boot sector is no
  longer read before each pass: the geometry from the card probe is kept, and read again only
  after a car write to LBA 0 or the boot sector.
- **32 KiB reads**, aligned, contiguous clusters in one transfer (the 32 KiB build above), and a
  pacer that banks at most two of them while it waits on the network.
- **The reader at priority 23 while it holds the card**, above the SMB thread (5) and below the
  TinyUSB task (24), so made ready on core 0 it preempts libsmb2's signing whatever priority
  inheritance does. IDF's own error log for a failed SD command, which ran under the lock, is off
  while the car has the card.

`external_a_shared_pass_completes_without_racing_the_car` runs the scenario of "Method" against
the build in `OXIDRIVE_READER_DIR` (default the fixtures' `oxidrive-fix-reader`) and asserts:
every firmware read answered, no car command of 100 ms or more in the window, the pass's `scan:
shared` line with no `stopped early`, and both older events on the share byte for byte.
`the_fixed_reader_checks_fail_on_each_hazard` checks the checks on made-up runs. Two runs per
cell, three at a time:

| build | programming | data WRITE(10), ms: median / p99 / max | worst car command | car commands of 100 ms or more | unanswered firmware reads | the pass | MB/s over it | test |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 728b71f | 0 | 2.63 / 2.88 / 247 | 247 ms | 8 | 0 | stopped by its own guard at 15.4 s, 1.8 MB read, nothing archived | 0.30 | fails |
| 728b71f | 1 ms | 3.62 / 3.63 / 3.63-3.87 | 3.87 ms | 0 | 1 | ended at 9.2-9.6 s after 0-0.4 MB, nothing archived | | fails |
| 728b71f | 2 ms | 4.63 / 4.75 / 4.75 | 4.75 ms | 0 | 1 | ended at 9.2 s, nothing read | | fails |
| fix-shared-reader | 0 | 2.63 / 2.88 / 4.25 | 4.25 ms | 0 | 0 | both events archived, summary at 52.5-52.7 s | 0.20 | passes |
| fix-shared-reader | 1 ms | 3.62 / 3.75-3.87 / 5.12-5.50 | 5.12-5.50 ms | 0 | 0 | both events archived, 55.2-55.4 s | 0.19 | passes |
| fix-shared-reader | 2 ms | 4.63 / 4.87-4.88 / 5.75-6.00 | 7.25 ms | 0 | 0 | both events archived, 62.0 s | 0.16 | passes |

The 728b71f rows are one run each (0 ms twice, the same both times). In a fixed run 39 to 76 of
the 2500 data WRITE(10)s (1.6-3.0%) met a reader transfer, never two in a row and never more
than 1.75 ms of bus inside one; no read was retried, and the car kept up with its camera in
every run, at most 6.0 ms late. Every WRITE(10) ended with good status and the card held every
clip as the car wrote it.

The branch's earlier builds (6adb83d, 1e685d4) passed the same cells, but at 2 ms the car fell
behind. Its commands for a chunk (the 64 KiB WRITE(10), the FAT sector's READ(10) and two
one-block WRITE(10)s) take a median 11.6 ms of the 16 ms, leaving 4.4 ms. The pacer's bucket
banked up to a second's worth (1 MB) while the reader waited on SMB, then let it out at once:
35-39 transfers within 200 ms, 50-62 ms of bus, met by about one car command in nine. Each run
at 2 ms started 9 or 10 chunks a whole chunk or more late, at most 30 ms; at 0 and 1 ms they
kept up. 728b71f's passes ended before that could happen. Capped at two transfers (0518b69), a
steady 1 MB/s is one transfer every two chunks, at the same 0.16 MB/s over the pass.

The inversion fix was chosen by measuring the others, from scratch commits of the same branch
before the burst cap (`sim-fix-reader-*` in oxidrive's repository; the `fixed` checks applied to
each):

| build | programming | runs | car commands of 100 ms or more | worst car command | MB/s over the pass |
| --- | --- | --- | --- | --- | --- |
| the lock and retries only, the SMB thread floating | 0 / 1 / 2 ms | 2 / 1 / 1 | 2-3 / 4 / 3 | 203 / 223 / 195 ms | 0.20 / 0.19 / 0.16 |
| the same, with the SMB thread pinned to core 1 | 0 / 1 / 2 ms | 2 / 2 / 2 | 0 | 5.25 / 6.37 / 7.37 ms | 0.20 / 0.16 / 0.13 |
| the same, with the reader at priority 23 (the shipped fix) | 0 / 1 / 2 ms | 2 / 1 / 1 | 0 | 5.38 / 6.25 / 7.25 ms | 0.20 / 0.19 / 0.16 |

The lock alone leaves the inversion: the reader now holds the card lock as well as the SD host's
mutex while libsmb2 holds its core. Both fixes remove it. The pinned build uploads more slowly
whenever the card programs, because the TinyUSB task on core 1 polls CMD13 back to back through
each write's programming time and libsmb2 then shares that core: at 2 ms each phase of its pass
(two uploads, a read-back) took 1.25 to 1.6 times as long as the floating builds' while the car
wrote, and as long as at 0 ms once the car stopped. The scratch builds have the branch's
reader as of 6adb83d.

## Shared-pass refusals by cause

oxidrive's shared-reading plan proposes a reduced design for invalidation: a per-cluster
last-write map in place of the 16K-entry dirty-log ring, and a comparison of the chain's FAT
entries and of the clip's 32-byte entry and long name in place of whole-sector watches
(shared-reading-plan.md, "Snapshot overlay: reviewed, reduced design", on oxidrive's
snapshot-overlay-design branch). It can remove only the refusals the ring's wrap and metadata
false sharing cause. `cli/tests/oxidrive_refusals.rs` measures how many of oxidrive's shared-pass
refusals those are, by putting each refusal and discard into one of the causes below.

### Method

- **The firmware.** A scratch build of oxidrive 728b71f, 4bfea67 on oxidrive's sim-refusal-log
  branch (not merged, not in this repository; `OXIDRIVE_REFUSAL_DIR`). It logs `RLOG` lines at
  info level at these points: every listing refusal (the walk's facts and where the entry is),
  every watch it arms (its spans, the directory entry's place), every re-check (the dirty log's
  write count, entries lost, oldest and newest retained, its last four entries, and every
  retained write that hit the watch, up to 24), every stalled read, every clip's outcome, and
  the start and end of each pass. The rest of the diff is read-only accessors on `DirtyLog` and
  `Watch`, and a field that holds the entry's place for the log. Two lines were removed. Both
  were replaced by equivalent code: the re-check still makes one `with_log` call and matches on
  the same `watch.revalidate` answer. The emulator's UART takes bytes at once, so the lines cost
  CPU time only. The scan for hits runs only after a dirty answer. Info lines do not reach the
  firmware's syslog while the car writes.
- **The car.** The writeback car of "The writeback model" (`PageCache::writeback`,
  `WritebackCfg`'s defaults, Linux's entry order, the dirty flag). A Sentry-like recorder:
  - Four cameras record a set every 20 s, each camera's clip written in 64 KiB chunks.
  - RecentClips keeps 6 sets, and the oldest is deleted as vfat unlinks.
  - Every fourth set is an event. The car makes `SentryClips/<stamp>/`, writes `event.json` and
    `thumb.png`, and copies the last two sets and the trigger set into the folder. The copies are
    written four clips a step beside the live set, so the folder is a live write target for 60 s
    at twice the camera rate.
  - Six events are kept.
  - The card is 4 GiB, filled with one file except for what the recorder needs, so rotation
    reuses freed clusters.
  - The run records continuously, so no window opens; every run checks that none did.
- **The arms.** Three clip sizes per camera-set: 8, 4 and 2 MiB. While an event is copied the car
  writes 3.4, 1.7 or 0.84 MB/s, and half that otherwise. The 8 MiB arm's write tap counted
  91 writes a second, the rate oxidrive's dirty.rs assumes for a recording car (94).
- **The bus.** The SD bus is timed (`--sd-timing`) and the card has no programming time
  (`OXIDRIVE_SD_PRG_US=0`). With programming time, the bus race ("The bus race") ends nearly
  every pass before refusals accumulate. A run with programming time, after the reader waits for
  a ready card, is the follow-up.
- **The network.** The NAT reaps a TCP flow idle for 120 s without telling either end
  (`Nat::idle_close_us`). oxidrive keeps its SMB session between passes, and the first pilot lost
  it that way: every upload waited on a dead connection. The test raises the limit to an hour,
  as a NAS on the same LAN would keep the connection. samba runs with `server signing =
  mandatory`, as everywhere else in these tests, or with `auto` (`SAMBA_SERVER_SIGNING`): signing
  offered, not required. libsmb2 signs only when the server requires it (its `negotiate_cb`), so
  under `auto` the firmware does not sign. The firmware is the same in both.
- **Judging a discard.** The medium's history is the card image after the car's mount, plus
  every write the car landed, with its time, and the bytes of every metadata page it landed
  (`PageCache::meta_landed`; the car's FAT and directory writes all go through `write_meta`).
  From that history, at the watch's start (`t0`) and at the re-check (`t1`), the test takes:
  - the chain, walked through FAT copy 0 at `t0`;
  - whether the car landed any write on one of the chain's clusters in (`t0`, `t1`];
  - whether any of the chain's entries changed in either FAT copy;
  - whether the clip's 8.3 entry or its long-name entries changed. The slots are found from where
    the firmware listed the entry.

  Any of the four is a change, and the reduced design would discard the clip too. That is (d).
  Otherwise the discard is put down to the firmware's own answer (`Unknown` is (a)) and to where
  the car's writes inside the watch landed during the window: every landing, not only the hits
  the firmware logged, which stop at 24. The firmware's clock (esp_timer milliseconds) plus
  60.4 to 60.6 ms is the emulator's. That is the median distance from each write the firmware
  logged to the same blocks in the card's write log, over 298 to 1146 matches a run; the 10th
  percentile is within 0.6 ms of it, and the tail is other writes to the same blocks (`clock:`
  lines).
- **Whether a saved discard was safe.** For every discard the reduced design would not make, the
  test also checks two things against the car's own record. Had the car closed the file before
  the read began? Had every cluster of the chain had data landed on it after the file was begun?
  If either fails, accepting the clip could have published a clip that was not whole.

Each test asserts that oxidrive wrote no block, that no window opened, that every clip on the
share equals the car's file, and that every refusal falls in exactly one class.
`REFUSAL_OUT` keeps a run (console, card writes, landings, metadata bytes, the car's record, the
image after the mount). `external_oxidrive_refusals_judged_again` judges a kept run again from
`REFUSAL_IN`. The recorder's knobs are `REFUSAL_CLIP_KIB` (8192), `REFUSAL_SET_SECS` (20),
`REFUSAL_RING` (6), `REFUSAL_EVENT_EVERY` (4), `REFUSAL_PREROLL` (2), `REFUSAL_COPIES` (4),
`REFUSAL_EVENTS_KEPT` (6), `REFUSAL_RECORD_SECS` (720), `REFUSAL_CARD_MIB` (4096) and
`REFUSAL_FREE_MIB` (what the recorder needs); the network's are `REFUSAL_NAT_IDLE_S` (3600) and
`SAMBA_SERVER_SIGNING`. The firmware's `app.bin` had SHA-256 `d432a04888ce58b64227f142177dee307e526556fc987b1bc123f21e9b4a9d0f`. Three runs at
once took 4.4 to 5.2 times real time without signing and 1.8 times with it, when the passes read
little.

### The causes

Each refusal or discard is in exactly one class.

| class | what it is | does the reduced design remove it? |
| --- | --- | --- |
| (a) | the re-check answered `Unknown` (the ring had wrapped past the watch's start, or no answer), and nothing in (d) happened | yes |
| (b) | a hit on a watched FAT sector, and the chain's entries in it (both copies) unchanged; includes sectors the watch's FAT range covers that hold none of the chain | yes |
| (c) | a hit on the entry's sectors, and its 8.3 entry and long-name entries unchanged | yes |
| (d) | the clip changed during the read: a write landed on one of its chain's clusters, a chain entry changed, or its entry changed | no |
| (e) | refused at listing (chain against size: `ChainBroken`, `ChainAheadOfSize`, `ChainBehindSize`, `SizeUnset`; a walk that failed, `Unproven`), or not a finished MP4 when opened | no |
| (f) | not a refusal: the pass stopped (the slow-write guard, a read the card refused, the 60 s budget) | n/a, counted per pass |
| (g) | a hit inside the watch but outside the clip: a bounding run `Watch::add` made when the clip had more than 8 runs (data and the directory entry share one region), or a listing-time span the chain no longer holds; the clip unchanged | yes: the map is per cluster |

(g) is not in the plan's list. It is the same false sharing in the data region. Where a discard
had false hits of more than one kind, it is counted once, as (b) before (c) before (g); the
results give the overlap.

### Results

**With signing required** (samba `mandatory`), every pass that read a clip was stopped by
oxidrive's slow-write guard (`shared: the card went slow for the car (3 to 7 new writes over
100 ms); stopping`). This happened at every rate tried, after 2.1 to 4.9 MB had been read, and
37 of the 37 reading passes stopped this way. In 45 minutes of recording, 3 reads reached a
re-check. The first two passes of each run had nothing to read yet.

| arm | recorded | passes | stopped by the guard | re-checks: accepted / discarded | archived | refused at listing (e) | discards |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 8 MiB | 720 s | 12 | 10 | 0 / 0 | 0 | 36 | none |
| 4 MiB | 900 s | 14 | 12 | 0 / 0 | 0 | 60 | none |
| 2 MiB | 1080 s | 17 | 15 | 1 / 2 | 1 | 60 | (b) 1, (g) 1 |

The mechanism is the priority inversion of "The stalls: a priority inversion": the reader's
512-byte reads hold the SD host's mutex while libsmb2's signing holds core 0. That is what binds
here, not the ring.

**With signing offered but not required** (samba `auto`, so the firmware does not sign), passes
read until their 60 s budget, and refusals accumulate. 900 s of recording per arm:

| | 8 MiB | 4 MiB | 2 MiB |
| --- | --- | --- | --- |
| car's writes, as the write tap counts them | 91.1 a second | 46.1 a second | 23.4 a second |
| passes: nothing to read / ended by the 60 s budget / ran out of clips | 2 / 11 / 0 | 2 / 10 / 1 | 2 / 5 / 6 |
| re-checks: accepted / discarded | 26 / 19 | 40 / 47 | 113 / 34 |
| watch, open to re-check: median, max | 13.3 s, 14.3 s | 7.0 s, 8.0 s | 3.9 s, 4.9 s |
| read time spent on discarded reads | 258 of 611 s (42%) | 335 of 621 s (54%) | 133 of 576 s (23%) |
| (a) ring wrapped or no answer | 0 | 0 | 0 |
| (b) FAT sector false sharing | 17 (21.5%) | 30 (33.0%) | 22 (28.2%) |
| (c) directory sector false sharing | 0 | 0 | 0 |
| (d) the clip changed during the read | 0 | 0 | 0 |
| (e) refused at listing | 60 (75.9%): `ChainBroken` 52, `ChainAheadOfSize` 4, `ChainBehindSize` 4 | 44 (48.4%): `ChainBroken` 40, `ChainAheadOfSize` 4 | 44 (56.4%): `ChainBroken` 20, `ChainBehindSize` 16, `ChainAheadOfSize` 8 |
| (g) widened watch | 2 (2.5%) | 17 (18.7%) | 12 (15.4%) |
| all refusals and discards | 79 | 91 | 78 |
| **(a)+(b)+(c)** | **17 (21.5%)** | **30 (33.0%)** | **22 (28.2%)** |
| (a)+(b)+(c)+(g) | 19 (24.1%) | 47 (51.6%) | 34 (43.6%) |
| of the discards after a read | 19 of 19 | 47 of 47 | 34 of 34 |
| discards with false hits of more than one kind | 1 | 16 | 6 |
| removable discards of a clip that might not have been whole | 0 | 0 | 0 |
| Sentry clips closed by the car / archived | 132 / 25 | 132 / 40 | 132 / 113 |
| car's close to the upload: median, p90, max | 183 s, 212 s, 223 s | 162 s, 177 s, 185 s | 120 s, 166 s, 344 s |
| (f) passes stopped by the guard or a refused read | 0 | 0 | 0 |

Every clip on the share equals the car's file. oxidrive wrote no block, and no window opened.
(e) counts a clip again at every pass that refuses it. Its refusals are clips still in flight:
an event's copies whose FAT entries had not landed (the metadata lag), and the entry's size
against the chain. Both designs make these refusals.

- **(a) never happened, and cannot at these rates.** The pass budget bounds a watch: no watch
  here lasted longer than 14.3 s, and the budget holds one to about 60 s. At 91 writes a second the
  ring reached back 176.8 s or more at every re-check after it had wrapped (44 of 45), 12.4 times
  the longest watch. At 23 writes a second it reached back 692 s. A watch would wrap the ring
  only above about 270 writes a second.
- **(b) is the interleaved allocation.** The car writes an event's copies and the live set
  together, 64 KiB per file in turn, so one FAT sector holds entries of eight files. Rotation
  deletes the RecentClips set two minutes later and frees its entries in those sectors. Worked
  example (8 MiB, pass 3): `SentryClips/2026-01-01_00-03-00/2026-01-01_00-01-00-back.mp4`, a
  256-cluster chain, was read for 13.3 s. Its watch covered FAT sectors 2879+17 and 3903+17. The
  car wrote FAT1 sectors 2872-2879 and FAT2 sectors 3896-3903 during the read. Sector 2879
  holds 6 of the clip's entries; 99 of its 128 entries changed in the window, none of them the
  clip's. In the 2 MiB run's first discard, of the same file name, sector 3035 holds 6 of the
  clip's entries; 67 of its entries changed, none of the clip's. Both were checked by hand against
  the landed bytes.
- **(g) is the 8-run cap.** Interleaved and reused clusters give a clip 32 to 256 runs, so
  `Watch::add` collapses the data region into one bounding run. The directory entry is in the
  data region and joins the same run. Worked example (4 MiB, pass 8):
  `SentryClips/2026-01-01_00-19-00/2026-01-01_00-17-00-right_repeater.mp4`, a 128-cluster chain
  spread over clusters 114115 to 130994. Its watch was one run of 1,080,320 sectors (527 MiB) over
  those clusters, its directory sector inside it. 241 of the car's live writes hit it in 7 s, none on the clip's
  clusters, its entry or its FAT.
- **(c) did not happen.** Reads came minutes after their event's folder stopped changing, because
  of the backlog and because the archiver holds back the newest set. An event's directory sectors
  were quiet by then.
- **(d) did not happen**: no read here met a change to its clip. The detector runs on the same
  landings that the whole-clip check below finds for every cluster, so it is not blind.
- **Safety of what would be saved.** Every removable discard was of a clip that the car had
  closed before the read began, and every cluster of its chain had data landed after the file was
  begun. Accepting them would have published whole clips in this model.

**The counterfactual** (first order, the same reads and schedule): under the reduced design's
rules every re-check here would have accepted its clip, because every discard was removable. The
passes would have accepted 45 clips instead of 26 (8 MiB), 87 instead of 40 (4 MiB) and 147
instead of 113 (2 MiB). The backlog held more closed clips than that, so the re-reads of
discarded clips would have gone to others. Of the clips with a removable discard (18, 31, 18),
3, 1 and 11 were archived later in the run, by a median of 36 s, 37 s and 100 s (at most 214 s)
after the discard that refused them. The rest (15, 30, 7) were not archived by the end. A second
order (which clips a faster pass would have reached, and the car's rotation deleting unarchived
events) is not simulated.

### What the data support

- With the firmware as it stands and a NAS that requires signing, the reduced design changes
  nothing measurable. At every car rate tried the slow-write guard stops a pass before a read
  completes. The inversion and the 512-byte reads come first, then the bus race.
- Once passes run to their budget, all the discards are false: 100% of them in all three arms,
  and 21.5-33.0% of all refusals by the plan's taxonomy, (a)+(b)+(c). All of those are (b), FAT
  false sharing; (a) and (c) are zero. With (g), 24-52% of all refusals. These discards
  cost a quarter to half of the passes' read time.
- What would remove them: FAT-entry comparison removes (b). (g) needs only a watch that does not
  widen: a per-cluster map, or the directory entry in a region of its own and more runs. The
  ring's replacement buys nothing here, because a watch never outlives the ring.

### Limits

- **The CPU.** One instruction a cycle, with no cache or PSRAM stalls. The inversion's stalls
  would be longer on silicon.
- **No programming time.** The bus race cannot happen here. With programming time, the race ends
  the passes first ("The bus race"). The next step is a re-run with programming time, after the
  reader's fix.
- **The car is a model.** The writeback parameters are testkit's, and its allocation interleaves
  files 64 KiB at a time in lock-step. Linux's vfat on the car may interleave differently, which
  sets how many FAT sectors a clip shares.
- **The recorder is scaled down.** It uses 20 s sets, clips of 2 to 8 MiB, and two sets of
  pre-roll. A car's minute clips of 30-40 MB would make reads span passes (a resume re-reads from
  the start), and watches reach the 60 s budget.
- **SMB is slow here.** Uploads ran at about 0.5 MB/s against the board's 2 MB/s ("Upload
  throughput"), so each read is held open longer than on the board.
- **The NAS.** Whether the owner's NAS requires signing is not known. The NAT's idle reap was
  raised to an hour.
- **One run per arm.** The servers answer in host time. The counts are one sample each, and
  differences of a few discards between arms are not resolved by one run each.
- **The safety check** uses the car's own record, which the device does not have. It says the
  saved discards were whole clips here, not that a timing rule on the device would know.

## An exFAT car

A Tesla can format its card exFAT. Its Linux then writes the card with exfat-linux 5.8-1arter97,
a backport of Linux v5.8's fs/exfat that teslamotors/buildroot builds as a module
(buildroot-2021.02, package/exfat-linux; the tag's tarball matches the package's SHA-256). The car
of "The car's hazards" now records on either filesystem (`car::CarFs`): FAT32 as before, or exFAT
as that driver writes it (`usb_host::exfat`). `Car::new` still makes a FAT32 car;
`Car::<_, ExFat>::on` makes an exFAT one. Every golden and every FAT32 test is unchanged. Line
numbers below are the 5.8-1arter97 tag's; it differs from v5.8 in option parsing, in
compatibility code, and in the two places noted.

### What the driver writes

| | FAT32 (vfat, as modelled) | exFAT (5.8-1arter97) |
| --- | --- | --- |
| when a cluster is taken | after FSInfo's hint | at write() time, one cluster as the pages reach it (inode.c:104-262 `exfat_map_cluster`, through `cont_write_begin`) |
| which cluster | the next free after the hint | the file's last cluster + 1 (inode.c:171-172); for an empty file, the first free from the search pointer (fatent.c:335-345), which goes back to cluster 2 at every mount (super.c:512) |
| the chain | always in the FAT | none while each cluster is the last + 1 (NoFatChain); the first cluster that is not converts the file: its clusters so far chained in the FAT, the new one linked, the Stream Extension rewritten (fatent.c:366-372, inode.c:203-241) |
| a free | the chain zeroed in both FATs | allocation-bitmap bits cleared; the FAT is not written (fatent.c:153-198) |
| the dirty flag | set at a mount, cleared at a clean unmount | VolumeFlags dirty at the start of every change and clean at the end of each create, unlink, mkdir and truncate (namei.c:563/566, 835/844, 874/877; file.c:107/218); allocation (inode.c:169) and inode writeback (inode.c:40) set it dirty; nothing at a mount |
| how the flag is written | at once | a change to dirty with `sync_dirty_buffer` when the boot sector's buffer is clean (super.c:120-129): an immediate WRITE(10), REQ_SYNC, without FUA; every other change only marks the buffer dirty for writeback. The unmount's clean lands at the device's last sync (super.c:42-55) |
| entries | an 8.3 entry and long-name entries | a File entry, a Stream Extension and a name entry per 15 characters (dir.c:327-337): a 29-character Tesla name is four entries, 128 bytes, and can straddle a sector. SetChecksum rewritten with every write of the set (dir.c:587-602); ValidDataLength written equal to DataLength (inode.c:74-75, 236-238) |
| a new file's entry | size 0, no cluster | a FAT-chain flag, no cluster, size 0 (dir.c:434-480); the first allocation rewrites it NoFatChain with the cluster and the size then, 0 for the first page (inode.c:227-238) |

Five details that set the write pattern and are easy to read otherwise:

- **VolumeFlags is not written at a mount**, nor with FUA. It flips at every create, unlink and
  mkdir, and a write goes out at once only on a change to dirty while the boot sector is not
  waiting for writeback. `exfat_sync_fs` (sync(2)) lands what is dirty, then only marks the flag
  clean (super.c:57-71). Later kernels write it at once with REQ_FUA and REQ_PREFLUSH (v6.1
  fs/exfat/super.c:124); 5.8 and v5.10 do not.
- **PercentInUse is never written** by 5.8.
- **A name entry is written up to its terminating 0** (dir.c:418-432); v5.8 zeroes the rest of
  the entry, 5.8-1arter97 leaves what a reused slot held.
- **The allocation bitmap is pinned for the mount's life** (balloc.c:56-102): the driver
  allocates from the bitmap it read at the mount, whatever its buffer cache drops. A car that
  misses a window that allocated clusters takes them again
  (`an_exfat_car_and_a_window_between_its_polls`).
- **The search pointer starts at cluster 2 at every mount**, so the first file after a mount
  takes the lowest free cluster; FAT32 continues from FSInfo's hint.

### The model

- `exfat::ExFat` mounts (the boot region and its checksum, the up-case table, the bitmap whole),
  lists, reads, makes directories, writes and deletes files in the driver's order. Writes go
  through `PageCache` as for FAT32: data with `write_blocks`, the FAT, the bitmap, directory
  sectors and the boot sector with `write_meta`, a synchronous VolumeFlags write with the new
  `BlockDevice::sync_meta`, which `PageCache` writes at once and takes off its dirty list. A
  directory grows by a cluster hinted after its last, and is chained when that cluster is taken
  (namei.c:302-398).
- A chunk is any whole number of sectors; a cluster is taken when a chunk reaches one the file does
  not have (`CarFs::write_unit`). The FAT32 car still rounds a chunk up to a cluster.
- `CarCfg::linux_entries` rewrites the entry set after every chunk, as inode writeback finds the
  inode dirty, and `dirty_flag` keeps VolumeFlags as above. Inode writeback is modelled at the
  write, not 30 s later; the write lands with the metadata lag all the same, but VolumeFlags
  changes to dirty a little earlier than on the car. Allocation sets it dirty at the same time
  anyway while a file grows.
- `ExFat::fsck` follows exfatprogs 1.0.4's `fsck.exfat -y`, the version in Tesla's buildroot:
  VolumeFlags dirty at once; every chain walked; a chain into a free or owned cluster, or longer
  or shorter than its size, cut; checksums and ValidDataLength fixed; when a file was cut, the FAT
  read whole with the stale entries of free clusters zeroed, and the bitmap's differing sectors
  written; VolumeFlags clean. Clusters the bitmap marks that no file holds are freed only when a
  file was cut, as in 1.0.4. Whether the car runs it is not known.
- `tests/sd_image.rs` lays out an exFAT card as exfatprogs' mkfs does (1.2.5, mkfs/mkfs.c):
  partition from LBA 2048, the FAT and the cluster heap on 1 MiB boundaries, one FAT, the bitmap,
  a 60-byte up-case table (a-z to A-Z, the rest to themselves) and the root in the heap's first
  clusters. exfatprogs picks 4 KiB clusters up to 256 MiB, 32 KiB to 32 GiB, 128 KiB above
  (lib/libexfat.c:121-130); the car's own format is not known, so the cluster is a parameter.
  `sd_image::fsck` sends an exFAT image to macOS's `fsck_exfat -n` on the image attached
  read-only with `hdiutil` (it checks devices, not files), to exfatprogs' `fsck.exfat -n`, or to
  `FSCK_EXFAT`. macOS `fsck_exfat` passes the images at 4 KiB and 32 KiB clusters and a
  writeback car's 3 GiB tree (`external_exfat_tree_passes_fsck`), and fails an image with one
  bitmap bit flipped.

### Ground truth

`usb_host::writelog` turns what the car landed (`PageCache::landings` and `meta_landed`, with
`record` set) and its record (`Car::truth`, an `exfat::Truth`) into what a trace analyzer should
find:

- per write: its sectors by region, in `oxi trace --writes`' names (`mbr`, `pre-volume`,
  `boot-sector`, `boot-region`, `backup-boot`, `fat`, `fat2`, `bitmap`, `upcase`, `root-dir`,
  `dir`, `data`, `unknown`, `outside`; FAT32 adds `fsinfo` and `reserved`); for a write over the
  boot sector, VolumeFlags as it left them, whether only VolumeFlags and PercentInUse changed
  (`state`) or more (`structural`), and whether it went at once or by writeback;
- per file: chained or not, when and after how many clusters, its clusters and fragments, when it
  was made, closed and deleted, when its data first landed and when every cluster had, and when
  its entry set first landed and first held its final cluster, flags and size.

### FAT32 and exFAT on one workload

`one_sentry_workload_on_fat32_and_exfat` runs the Sentry-like recorder of "Shared-pass refusals
by cause" (`recorder::Sentry`, the same steps: four cameras, a set every 20 s, a ring of six
sets, an event every fourth set copied beside the live sets, six events kept) with the same
writeback car (`WritebackCfg`'s defaults, Linux's entry order, the dirty flag kept), on a 4 GiB
card filled but for what the recorder needs, once on FAT32 and once on exFAT, offline. It counts
from what landed, then 40 s for the writeback. The run is seeded and has no firmware, so it gives
the same numbers every time. 8 MiB clips, 900 s (`EXFAT_CMP_SECS=900 EXFAT_CMP_CLIP_KIB=8192`,
`EXFAT_CMP_EXFAT_CLUSTER_KIB` 32 and 128; the CI run is 240 s of 2 MiB clips); 15.9 minutes in
all, 46 sets, 11 events:

| | FAT32, 32 KiB clusters | exFAT, 32 KiB | exFAT, 128 KiB |
| --- | --- | --- | --- |
| camera clips FAT-chained | 316 of 316 (FAT32 always chains) | 316 of 316 | 316 of 316 |
| clips chained after | | 2 clusters (the first chunk) | 1 cluster |
| `event.json`, `thumb.png` (one cluster, written alone) chained | all | 0 of 22 | 0 of 22 |
| fragments a clip: median, max | 128, 256 | 128, 256 | 64, 64 |
| FAT sector writes | 1,699 in FAT1 + 1,689 in FAT2 | 905 | 242 |
| of them: entries linked / entries zeroed / both / no change | 844 / 791 / 64 / 0 (FAT1) | 891 / 0 / 0 / 14 | 238 / 0 / 0 / 4 |
| FAT sector writes a minute | 213 | 57 | 15 |
| bitmap sector writes: bits set / cleared / both | | 106: 51 / 49 / 6 | 52: 20 / 15 / 17 |
| bitmap sector writes a minute | | 7 | 3 |
| FAT sectors holding live entries of files written together: shared by two or more | 950 of 952 (99.8%) | 935 of 940 (99.5%) | 273 of 273 (100%) |
| boot-sector writes while recording | 0 (the flag at the mount and the unmount only) | 25: 24 landed dirty, 1 clean; 1 at once | 25: 24 dirty, 1 clean; 1 at once |
| boot-sector writes a minute | 0 | 1.57 | 1.57 |
| directory writes a minute | 11 | 13 | 34 |
| data writes a minute | 2,695 | 2,695 | 2,548 |

A write's count is a landing (`PageCache::landings`, one per contiguous run, as WRITE(10)s of up
to 240 blocks); a sector write is one sector of a landing. "Entries zeroed" and "bits cleared"
are the frees; "linked" and "set" the allocations, conversions included. On exFAT, "no change"
writes rewrite entries a ring reusing the same clusters in the same pattern gave the same values,
stale from the clip deleted before. 128 KiB clusters have twice the directory writes because a
new event folder zeroes a 128 KiB cluster, which lands page by page.

**The prediction** for a trace of the car writing exFAT, if the car writes as modelled:

1. **Clips written together are FAT-chained.** Every camera clip recorded beside another is
   converted as soon as it takes its second cluster run: after its first chunk at 32 KiB, after
   its first cluster at 128 KiB. Only a file written alone stays NoFatChain. A trace shows each
   conversion as a burst of FAT entries for the file's clusters so far, then its Stream Extension
   rewritten with flags 0x01.
2. **No FAT entry is ever zeroed.** FAT sectors are written only for chained allocation; the
   rotation's frees write bitmap sectors only. FAT32 writes about as many FAT sectors for frees as
   for allocation, in both copies.
3. **About a quarter of FAT32's FAT writes at 32 KiB, 7% at 128 KiB**; the bitmap adds 3 to 7
   sector writes a minute. One bitmap sector covers 4,096 clusters (128 MiB at 32 KiB), so files
   written minutes apart share it.
4. **FAT sectors are shared just as much**: every FAT sector a set writes holds entries of two
   clips or more, as on FAT32. exFAT does not remove the FAT false sharing of "Shared-pass
   refusals by cause" for chained clips; it removes the frees' share of it, and adds the bitmap's.
5. **The boot sector is written about every 38 s while the car records** (1.57 a minute), by
   writeback, nearly always holding VolumeFlags dirty: a create's clean is overtaken by the next
   allocation's dirty before it lands. A write at once happens only on a change to dirty with
   the buffer clean: the first change after a mount, and the first after an unlink's clean has
   landed. Nothing is written at the mount. Once the car stops, its deferred inode writebacks
   most likely leave the medium dirty ("Limits"). A trace showing a boot-sector write at the
   mount, or one per create, would mean the car's driver is not this one. A trace cannot show FUA:
   the write tap sees sectors, not the command's flags.
6. **An entry set lands before its file is whole**: NoFatChain, the first cluster and size 0
   from the first allocation, then intermediate sizes (`writeback_lands_intermediate_entry_sets`).
   fsck.exfat reads such an entry as "first cluster is wrong" (size 0 with a cluster).

### What else the tests pin

`esp-soc/src/usb_host/tests/exfat.rs`: the image's layout and checksum (VolumeFlags left out of
it); a file written alone stays NoFatChain with no FAT entry; two files written in turn are
chained after one chunk each and read back through the FAT; a deletion clears bits and leaves the
FAT, and keeps the deleted set's bytes with the type's top bit cleared; checksums and name hashes
of sets that straddle sectors; VolumeFlags under writeback; intermediate entry sets and the ground
truth; a power cut that fsck repairs, and fsck at the mount of a dirty volume; a ring of 120
sets, whose folder outgrows a 4 KiB cluster once and is chained; a directory that grows; on the
fake drive, a car writing into a window until its poll, a gated window and its unit attention,
and windows between polls (a caching or not caching car loses the window's folder, the bitmap
being pinned; a gated window's unit attention saves it); a port reset and a bulk fsck.

### The firmware on exFAT

`cli/tests/oxidrive_exfat.rs` runs the owner's exFAT builds of oxidrive (not in this repository)
on the rig of "Services", with the exFAT car on its port: Linux's writeback, VolumeFlags kept,
the entry set rewritten as a file grows. The card is a fresh 4 GiB exFAT image of 32 KiB clusters;
the SD bus is timed and the card programs for 1500 µs after each write (assumed, as in "The car's
hazards"). Builds: 037647d, with exFAT archive windows (`OXIDRIVE_EXFAT_DIR`), and 67e58e5, the
SD_TRACE build (`OXIDRIVE_EXFAT_TRACE_DIR`), which uploads every car write to `oxidrive-trace/` on
the share. One run each, five at a time:

| test | what happened | verdict |
| --- | --- | --- |
| `external_oxidrive_archives_an_exfat_card_through_windows` | The first window (61.2 s, 1.5 s long) found no TeslaCam and made it. The image as it left it passes `fsck_exfat`, and the car mounts it with TeslaCam's three folders, VolumeFlags clean. The car met MEDIUM NOT PRESENT inside it and UNIT ATTENTION after it, read the volume again and recorded two saved events. Two windows forced once its writeback had landed (108.7 s and 164.8 s, 11 s each) archived the older event, then the newer: both equal the car's files. The third event is held back. Every clip is on the card as written; the car unmounts clean; fsck is clean | passes; 220 s emulated, 350 s wall |
| `external_oxidrive_refuses_shared_passes_on_exfat_and_says_so_once` | The car wrote a clip every 20 s for 200 s, so no window opened. The refusal (`shared: the card is exFAT; shared passes read FAT32 only, ...`) came at 60.2 s and never again. MQTT's state carries `"fs":"exFAT"` and `shared.aborted` "card is exFAT: not read". Nothing reached the share, and oxidrive wrote no block | passes |
| `external_oxidrive_refuses_exfat_car_writes_in_a_window` | The window opened at 61.2 s. The car's first command after the take (a READ looking up its folders) was refused with 2/3A/00 within 1 ms. No block of the car's reached the card in the window (61.2-72.3 s). The window archived the older event and held back the newer; the car met UNIT ATTENTION and recorded; fsck is clean | passes |
| `external_oxidrive_archives_a_card_the_car_formats_exfat` | A FAT32 card. The car recorded an event, then formatted the card exFAT over USB at 0.54-0.87 s (exfatprogs' layout and order, `format_exfat`). At 1.15 s the firmware logged `fs: the car rewrote the volume's layout; now ExFat`. It refused the shared pass once (60.3 s). The window at 121.3 s archived the older exFAT event, read back as it does for the first clips of a boot. It did not upload the FAT32 event the format had erased, and held back the newer one. fsck is clean | passes |
| `external_oxidrive_forced_window_drops_a_writeback_cars_metadata` | The car recorded two events on the card the first window had prepared. `force_open` 15 s after its last write opened a window (83.7 s) while the car still held 20 metadata pages. The window found no event and closed in 1.0 s. At its poll inside it, the car dropped the 20 pages; neither event's folder ever reached the card, though their data did | passes (asserts today's behaviour) |
| `external_oxidrive_exfat_trace_matches_the_cars_ground_truth` | The car recorded the Sentry workload for 200 s, then small clips every 15 s until a flush came after its last write. 5 trace files: 4,463 writes over 298 s, 12 boot-sector contents, none lost. Every trace write lies inside a write the car landed (the emulator's clock is the firmware's plus 62.2 ms). The trace's data sectors equal the car's (279,220): esp_tinyusb writes each 64 KiB WRITE(10) as two 32 KiB card writes. `oxi trace --writes` gives every write the regions the car's layout gives it, with the card image and without (where it calls directory clusters `data`, root-dir excepted), and the boot class (`state`) of all 12 boot-sector writes | passes |

Findings for oxidrive:

1. **The window's own exFAT writes are sound** on a fresh card: TeslaCam and its three folders,
   their four clusters zeroed, one bitmap sector, one root-directory sector, TeslaCam's entries; no
   FAT write; VolumeFlags left clean. `fsck_exfat` passes the card and the car mounts it.
2. **A window holds back the newest event**, on exFAT as on FAT32, so a single window after a
   session archives every event but the last; the next window takes it.
3. **A forced window can lose a writeback car's last events.** A window that opens on its own waits
   for a minute of quiet, longer than the car's metadata lag (35 s at most). `force_open` does not:
   a window forced within the lag refuses the car's writeback, and the car drops what it held,
   the events' folders and entries with it. The data of the clips is on the card, unreachable. A
   force that waits for the card to have been quiet as long as a natural window would avoid it.
4. **The policy's cooldown is 1,200 s** after a window, so on exFAT, where shared passes are
   refused, a car's clips wait up to 20 minutes for the next window.

`OXI_BIN` is the `oxi` CLI built from oxidrive's repository; `OXIDRIVE_EXFAT_OUT` keeps the trace
and the card image of the trace test for `oxi trace` by hand (never commit either). The oxi used
here was built at 037647d, whose trace code matches b905a73's.

### Limits

- **The writer is Tesla's guess.** How many files the car writes at once, in what chunks, and
  whether it copies or renames event clips are the recorder's, not measured. Any interleaving of
  two growing files chains both; a car that writes each clip whole before the next would chain
  none.
- **The cluster size** of the car's own format is not known: 32 KiB and 128 KiB bracket
  exfatprogs' choices for cards above 256 MiB.
- **Writeback** is testkit's model ("The writeback model"), as for FAT32. Inode writeback is
  modelled at the write. Linux writes a dirty inode back about 30 s after it was dirtied, and
  `__exfat_write_inode` sets VolumeFlags dirty. While the car records, allocation keeps the flag
  dirty all the same. After its last unlink, though, Linux's deferred inode writebacks set it
  dirty again about 30 s later (written at once if the unlink's clean has landed by then). So a
  car at rest most likely leaves the medium dirty, where the model leaves it clean.
- **fsck**: whether the car runs fsck.exfat, and with which options, is not known.
- **Not modelled**: rename, rmdir, truncate other than to 0, TexFAT's second FAT, sectors other
  than 512 bytes, the up-case table's full contents (the image's is the mandatory part).

## A report per boot, through a power cut

oxidrive's car report (its `car_report` module, on oxidrive's car-report branch; not merged, not in
this repository) keeps per-boot figures about the car in NVS, a record per boot in a ring of eight,
and publishes each boot to MQTT and syslog once the device is home: the car's write service times
as a histogram, the card lock, the car's SCSI commands and the gaps between its TEST UNIT READY
polls, the sense data it was sent, the gaps after a window, the radio, the network, shared
reading, health, and what saving the record cost. `cli/tests/oxidrive_car_report.rs` checks the
figures against the emulator's own record and measures what the saves cost the car. Fixtures:
`OXIDRIVE_CAR_REPORT_DIR` (the branch's build, `oxidrive-car-report`) and `OXIDRIVE_BASELINE_DIR`
(`oxidrive-bc40b82`, a build without the report).

### What the model gained

- **Flash that takes time to program and erase** (`spi_mem::FlashTiming`, set on the P4 as
  `SocBus::flash_timing`, `RigOpts::flash_timing`). After a page program or an erase the status
  register's WIP bit stays set for the part's time, through RDSR and RD_STATUS alike, so IDF's
  wait-idle loop waits it out. IDF's own code holds the other core in its IPC stall and masks the
  non-IRAM interrupts meanwhile, as compiled, so the car's commands wait as long as on a board. The
  cache is still not modelled: code that runs from flash during the operation is not caught. Each
  timed operation is logged (`SocBus::flash_ops`: start, end, address, length, erase). The times
  are GigaDevice GD25Q-class typical figures (a page program 30 us plus 2.5 us a byte, a 4 KiB
  sector erase 45 ms): an assumption for the board's 32 MB part, not a measurement.
  `OXIDRIVE_FLASH_ERASE_MS` and `OXIDRIVE_FLASH_BYTE_NS` vary them. Untimed, as before, by
  default.
- **A card that stalls now and then** (`SdCardConfig::slow`: every `n`th programming period lasts
  longer) **and its writes as served** (`SdCardConfig::writes`: each write command from CMD24 or
  CMD25 to the end of its programming), the ground truth for a write tap's service time.
- **Sense data per REQUEST SENSE** in `CmdTime::sense`.
- **An NVS partition with more in it** (`RigOpts::nvs_extra`: CSV rows appended to the fixture's
  `nvs.csv` before ESP-IDF's generator lays it out).
- **A power cut**: a P4 reset with the power-on cause (`request_reset(RST_POWERON)`). RAM survives it
  here; the firmware re-initialises what it uses. The car is a new host after it: the old one is
  dropped and, where a test wants a car again, it enumerates afresh.

### Ground truth

`external_oxidrive_car_report_survives_a_power_cut`: a car records four 4 MiB saved events on a
card that programs for 1.5 ms after each write, 30 ms every 97th and 150 ms every 401st, polls TEST
UNIT READY every 1.5 s, meets a forced window and records a 2 MiB event after it, then idles 75 s.
The P4 then loses power before it has published anything about that boot; it boots again with no
car and publishes it; a second power cut, and the third boot publishes the second boot's report
and not the first's again. Every count is the device's against the emulator's, up to the device's
last save (its `up_ms`):

| Figure | Device | Emulator |
| --- | --- | --- |
| TEST UNIT READY / REQUEST SENSE / READ CAPACITY / INQUIRY | 109 / 14 / 2 / 0 | 109 / 14 / 2 / 0 |
| sense 2/3A/00, 6/28/00, none, other | 13, 1, 0, 0 | 13, 1, 0, 0 |
| TUR gaps, buckets <=0.5, 1, 2, 5, 10, 30 s, more | [0, 0, 108, 0, 0, 0, 0] | [0, 0, 108, 0, 0, 0, 0] |
| car writes (the tap's calls; card write commands) | 1211 | 1211 |
| sectors | 37940 | 37940 |
| service time, buckets <=10, 25, 50, 100, 250, 500 ms, more | [1196, 0, 12, 0, 3, 0, 0] | [1196, 0, 12, 0, 3, 0, 0] |
| longest write | 151.707 ms | 151.684 ms (card side) |
| from the card's return to the car's first command / first write | 216 / 3269 ms | 149 / 3201 ms |
| MQTT sessions lost (a `session down` after a `session up`) | 4 | 4 |

The device stamps the card's return where the component hands it back; the emulator times it from
the console line, whose slice ends up to 100 ms after the event, so the emulator's gaps are short
by the line's lag; the test allows 150 ms. The device's last save came 60.3 s after the car's last
write, as its schedule says. Three saves in the boot: at boot, 60 s after the car's first four
events while a shared pass uploaded them, and 60 s after its last write; the longest held the flash
for 3.6 ms, and no car command was in flight during any flash operation. The figures were the same
on every run, apart from a TEST UNIT READY poll or a REQUEST SENSE more or less (both sides) and
the gaps after the window, which move with where the car's poll falls. The device counts a lost
session in esp-mqtt's callback, only if the session was up: esp-mqtt reports every failed connect
as a disconnect too, so the test pairs each `session down` with a `session up` before it. The
record's thread had 3.3 KiB of its 6 KiB stack left. 681 s wall for 207 s emulated.

### What saving costs the car

`external_oxidrive_car_report_costs_the_car_little`: the same car on bc40b82 and on the report's
build, the flash timed in both. It records the way a Linux flusher writes back on a timer: a 4 MiB
clip as 64 KiB WRITE(10)s every 16 ms (about a second), then nothing but a TEST UNIT READY every
2 s until five seconds have passed, for 330 s. RecentClips only, so shared passes list and read
nothing, and the car never goes quiet for a window.

| | without the report | with it |
| --- | --- | --- |
| car commands, buckets <=10, 25, 50, 100, 250, 500 ms, more | [13260, 0, 0, 0, 0, 0, 0] | [13260, 0, 0, 0, 0, 0, 0] |
| longest car command | 5.63 ms | 5.63 ms |
| flash busy spells in the 330 s | 5 (1.0-1.1 ms) | 6 (5 of 1.1-1.2 ms, 1 of 2.69 ms) |
| car commands in flight during one | 2-3 a run, at most 620 us over their median | the same |
| ... during one of the report's saves | | 0 |

The five spells common to both are oxidrive's journal, written after every shared pass whether or
not the car is writing (`archiver.rs`, `run_shared_pass`): those are the car WRITE(10)s that met a
busy flash, in both builds. The report's own save five minutes in landed in a gap between bursts
and held the flash for 2.69 ms with no car command in flight; the report the next boot published
counted it (`persist`: one save before its own, 2.7 ms, histogram bucket <=5 ms, nothing during or
just after). No NVS sector erase came due in either run; the next test makes one. 1219 s wall.

### An NVS page reclaim

`external_oxidrive_car_report_nvs_reclaim_under_the_car`: the same car on the report's build, with
the NVS partition generated full: thirteen pages of filler strings, the page the firmware writes
next with 56 of its 126 entries free, and one empty page. 75 s with no car follow the 330 s, so
the save after the car stops records the saves before it; then the power cut and the next boot's
publish.

| | |
| --- | --- |
| the reclaim | the report's five-minute save at 301.406 s: the flash busy 53.33 ms, a 4 KiB erase of NVS page 0 (0x9000) and 1260 bytes programmed, the page's live entries among them |
| car commands in flight during it | 0 |
| car commands by latency, <=10 ms ... >500 ms | [13260, 0, 0, 0, 0, 0, 0], longest 5.63 ms: the cost test's figures |
| car commands that met a busy flash | 2, both the journal's 1.1 ms writes, at most 0 us over their median |
| the published report | 2 saves before its own, longest 54,283 us, histogram [0, 1, 0, 0, 0, 1, 0] (the boot's save and the reclaim), nothing during or within 1 ms after |

The save waited for a gap between bursts and the erase fitted in it. A car command that had come
during it would have waited up to the rest of the 53 ms; this car's commands did not. The only
other erase in the run is the bootloader's of `otadata` (0x2a000) 36 ms into the boot, before the
car, in every run. 688 s wall.

### An update with a boot's report unpublished

`external_oxidrive_car_report_from_an_older_build_survives_the_update`: the car-report layout
grew two counts after the first v1 layout (469 bytes), and a car that had not powered off since
its last drive still held a 469-byte record for that drive. The older build
(`OXIDRIVE_OLDER_REPORT_DIR`, `oxidrive-c4c538c`, which writes the first layout) boots, saves
its boot's report and is not let publish it; the newer build (`OXIDRIVE_CAR_REPORT_DIR`) is
written over its app at 0x30000, the rest of the flash and NVS kept, and the P4 loses power.

| | eb81ae4 | the follow-up branch's head |
| --- | --- | --- |
| the newer build's boot | `car report: r1 unreadable (Length(466)); it will be replaced`, `0 kept` | `boot 2, 1 kept, 1 not yet published` |
| `car_report/1` | never published | published 9.4 s into the new boot: `fw` 0.1.0-c4c538c, `seq` 1, the older counts as numbers, `connection_lost` and `no_session` null |

`bin/car-report` prints `n/a` for the two. 80 s wall.

### A crash the first runs found

The first runs of the cost test lost the P4 a minute or two in: FreeRTOS asserted in
`xTaskPriorityDisinherit` (`pxTCB == pxCurrentTCBs[...]`) on core 1 with the TinyUSB task in
oxidrive's write tap. The core dump (`espcoredump.py info_corefile` on the coredump partition, read
out of `SocBus::flash`) put the give inside `card_lock::take`: it built its guard with
`bool::then_some(Held(m))`, which evaluates its argument whether the take succeeded or not, so a
failed take dropped a guard and gave the mutex the reader held. On bc40b82 only a car write that
waits out its 10 ms, or a shared read its 50 ms, fails a take; the report's build tries the lock
once without waiting first, so every car write that met the reader did. Fixed on the car-report
branch with `then(|| Held(m))`, and on oxidrive's main since; the runs above are on that build.

### Limits

- **The flash's times are assumed**, and the cache is not modelled: the cost is IDF's software
  stall for the part's typical times, not a board's.
- **The car's write pattern is the emulator's.** Whether a Tesla leaves gaps between bursts, and
  how long, is not measured; the report's TUR gaps and save counts from a car will say.
- **The power cut keeps RAM**, and the boots after it run with no car.
- **The return of the card** is timed from a console line, to 100 ms.
- **One reclaim was measured**, forced by a pre-filled partition, at the part's assumed 45 ms erase.
  The datasheet's maximum is several times that; the run does not say how a car takes it.

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
  the board's (open; not the missing slave: with the C6 on slot 1 it is the same 108 bytes);
- the card's FAT geometry, label and root directory: the image's, not the board card's;
- the firmware's journal and configuration, which the board's NVS holds from earlier boots and
  the emulator's blank flash does not: it reports a first boot, seeds a default and runs
  unconfigured (four blocks);
- the die temperature: the emulator's sensor reads a fixed 25 °C ("The temperature sensor").

The board's lines from 93 on come through the C6 (WiFi, the clock, MQTT). With the C6 on slot 1
(`--coproc`, "The co-processor", "oxidrive") the WiFi and clock lines are reached and 80 lines
match.

`external_oxidrive_presents_the_card_to_a_car` boots the same images with VBUS from power-up and
the card image written through. The car enumerates oxidrive at 0.27 s at high speed, the drive is
ready at the first TEST UNIT READY, its FAT32 volume (32 KiB clusters) mounts, and a 4 MiB clip
written in 64 KiB chunks, one due every 16 ms (a 4.1 MB/s camera), keeps up with the camera: it
takes 1.01 s, no chunk starts more than 1.9 ms late (the first, behind the directory entry), and
the clip reads back over USB unchanged and is on the image byte for byte afterwards. A 64 MiB
clip (`OXIDRIVE_CLIP_MIB=64`) keeps up the same way, with the SD bus timed or not ("USB",
"Throughput"). The firmware logs its USB
link connected and mounted. It prints nothing about the car's writes in the 14 s of emulated time
looked at: what it counts feeds the archive windows, which need WiFi and the NAS. Its WiFi
bring-up then fails in ESP-Hosted's `esp_hosted_reconfigure` (`transport_drv_reconfigure()`
returns ESP_FAIL, no slave answering on slot 1) and retries every 30 s, serving the card to the
car meanwhile; with the co-processor it comes up ("The co-processor").

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
- **The SDIO slave's send link must answer at the register write that starts it.** The driver
  starts a one-byte descriptor and spins on RX_DONE before it does anything else, so the DMA reads
  the descriptor in the write's own bus call (`sdio_slc_write`), not later.
- **IDF reads the D1 pin, not only the host's interrupt bit.** `sdmmc_host_io_int_wait` returns at
  once if D1 is low, so DAT1 is a GPIO input level as well as RINTSTS's edge.
- **Slices must end on a fixed grid.** Cutting a slice where a caller happened to stop moved where
  EN was seen and where the P4's scheduling rounds fell, and the C6's `Saved PC` after its reset
  differed between two drivings of the same run; the lock-step test caught it.
- **A P4 reset restarted the SD bus's clock.** The device clock starts again with the new
  peripheral set, and the SDMMC host took its time from it, so after a P4 software reset every
  command reached the C6 with a time the C6 had passed and it answered up to a slice stale; a
  slot-0 card's busy period was measured on the wrong clock too. Found in review; the SD bus now
  counts from power-on.
- **The host's packet announcement must wait for the length.** Raising RX_NEW_PACKET at the rx
  link's START left a few instructions in which ESP-Hosted's combined register read could see a
  packet with its old length and wait for ever. Found in review, not seen in a run.
- **A chip reset must keep the analog registers.** The bootloader reads the CPLL's dividers back
  over regi2c when HP_ROOT is still on the CPLL, which every reset but `esp_restart`'s leaves it
  on; re-created with the digital peripherals, they read 0 and the bootloader aborted.
- **A timed auto-stop reached the card at the command's start.** `Sdmmc::finish` sent the CMD12
  with the time of the register write that started the command, so with `--sd-timing` a
  multi-block write began programming before its data had crossed the bus and a 32 KiB write hid
  up to 1.7 ms of programming time. Found by the car-latency runs; it now goes at the end of the
  data phase.
- **A restarted P4 glitches EN.** ESP-Hosted makes GPIO 54 an output (its output latch still 0
  after the reset) before it writes the level (`transport_gpio_reset`), so EN is low for a few
  instructions; a slice end can see that, so EN needs the C6's t_RST before it resets the chip.

## What is modelled

| | |
| --- | --- |
| CPU | two RV32IMAFC cores (`Cpu::new_esp32p4`, `misa` 0x40001125): F with the lazy FPU traps, EXT_ILL, Zicntr, CLIC mode, `mhartid`; core 1 held in reset until HP_SYS_CLKRST releases it. Xesppie and the hardware loops decode as illegal |
| Memory | 128 KB mask ROM at 0x4FC00000 (reset vector), 768 KB L2MEM at 0x4FF00000 and its uncached alias at 0x8FF00000, 8 KB TCM at 0x30100000, LP ROM (empty) and 32 KB LP RAM; flash (0x40000000) and PSRAM (0x48000000) windows behind two 1024-entry MMUs of 64 KB pages, reached through SPI_MEM_C / SPI_MEM_S item registers, with their uncached aliases |
| Interrupts | both interrupt matrices (128 sources, rev < 3) and both CLIC register blocks, delivered through the cores' CLIC mode |
| Peripherals | UART0/1, USB-Serial/JTAG, systimer (16 MHz from XTAL), TIMG0/1, SHA, efuse (v1.0, block v0.3, security efuses zero), the flash controller (SPI0 cache side, SPI1 commands, a GigaDevice part answering RDID, its programs and erases instant or, with `SocBus::flash_timing`, timed), the PSRAM controller with an AP Memory x16 PSRAM, the cache controller, HP_SYSTEM (cross-core interrupts, stall status), HP_SYS_CLKRST (divider update, PLL calibration done, core 1 clock and reset), LP_SYSTEM / LP_CLKRST / LP_TIMER (reset cause, software reset, core 1 boot address, RTC counter, the SDMMC reset), the analog I2C master (regi2c read-back, kept through a chip reset), the die temperature sensor at a fixed temperature, ASSIST_DEBUG's saved PC, the RNG, GPIO with the P4's register layout, the SDMMC host with its IDMAC and an SD card in the board's socket (`--sd`), the high-speed USB-OTG core as a device with buffer DMA and a host on its port (`--usb-host`), an SDIO function on slot 1 with its interrupt on DAT1 (`--coproc`: an ESP32-C6 running ESP-Hosted's slave, "The co-processor"); everything else is register RAM, listed by `--log-periph` |
| Time base | 360 MHz, the rev < 3 application clock; one instruction is one cycle |

The revision is a type parameter (`esp32p4::Revision`, `RevLt3`): rev >= 3 (its own ROM, the
`mintthresh` CSR, the L2 cache at the bottom of L2MEM) is a second implementation, not a change
to this one. The browser build boots the same images (`web/wasm/fw/p4-hello.json`, run by
`tools/wasm-test.mjs p4-hello`).

## Not there yet

- **AXI-DMA, the LP core**: out of scope. Xesppie's save and restore, which FreeRTOS runs only for
  a task that uses PIE.
- **The co-processor**: native only; the SDIO link without timing by default and the slave
  without busy or read wait, CRC or suspend ("The co-processor"); faults are injected on the link
  rather than arising from a model of the bus. The services oxidrive wants are real servers on
  the host, run by `external_` tests ("Services"); there is no in-process broker or SMB server, so
  no golden reaches them, and `--usb-host` does not run with `--coproc` yet (a test drives the car
  through `CoSim`'s `UsbPort`).
- **USB**: the full-speed OTG core (0x50040000) and USB_WRAP, host mode, the core's slave mode
  (the data FIFOs), descriptor DMA, isochronous and interrupt endpoints, LPM, the
  per-endpoint interrupt source (94), and the time a transaction takes inside the core: it
  answers at once. The UTMI PHY is register RAM.
- **SD timing** is off by default: a command and its data are done in the register write that
  starts them. `--sd-timing` times the bus but not the card's read access time.
- **No cache model, so DMA coherency bugs pass.** The data cache is not modelled: the CPU and
  every DMA engine see the same memory at once. Firmware that omits `esp_cache_msync` or whose
  call fails, a DMA buffer that is not cache-line aligned or shares a line with other data, or a
  buffer changed after the DMA was started all work here and can corrupt data on silicon. The
  USB core reads IN data at the IN token (`Dwc2::token_in`), where buffer DMA fetches it once the
  endpoint is enabled, so a buffer the driver changes after setting EPENA goes out changed here
  (dcd_dwc2 cleans the buffer and then enables the endpoint, `edpt_schedule_packets`). A cheap
  checker would keep two bits per 64-byte line of DMA-capable memory: written by the CPU since
  the line was last cleaned, and written by DMA since it was last invalidated. CPU stores set the
  first, DMA writes the second, and the cache controller's sync operations (CACHE_SYNC_CTRL with
  CACHE_SYNC_MAP, CACHE_SYNC_ADDR and CACHE_SYNC_SIZE, cache_reg.h, which `esp_cache_msync` ends
  in) clear them. A DMA read of a line with the first bit set, or a CPU read of a line with the
  second, is a coherency bug to report, with the address and the PC. It sits on every store, so it
  would be behind a flag, off by default.
- **GPIO** has the P4's register layout (`periph::GpioP4`), but pin levels, interrupts and edges
  are the shared model's 49 pins; the P4 has 55.
- **`--boot app`** maps the image and jumps to it without the clocks and caches the bootloader
  would have set.
- **`Machine::run_until_cycle`** still schedules core 0 alone; nothing that drives the P4 needs it.
- **Unmodelled blocks** hello_world touches: PMU, PVT, APB_SARADC, LP_WDT, LP_ANA_PERI, LP_ADC,
  KEYMNG, UART2-4, TRACE0/1, LP_UART, LP_PERI, LP_GPIO and DSI, all configuration the firmware
  writes and reads back (`--log-periph` lists them). The watchdogs never fire.

## Handoff: where milestone 4c goes on

**Status.** Milestone 4c, part 1 is done ("The car's hazards"): the car writes into a window
until its poll, keeps its FAT between polls, misses windows shorter than its poll, writes back as
Linux does with the firmware's own writeback parameters, rotates on a full card, keeps the dirty
flag, and runs fsck at a mount and after a USB reset; each runs against oxidrive 728b71f in
`cli/tests/oxidrive_hazards.rs`, the findings for the firmware listed in "Findings for
oxidrive". `usbmsc-p4-window` pins the window's write mechanism with IDF's own tusb_msc example,
deterministic and without a network. The die temperature sensor is a model ("The temperature
sensor"), and a chip reset keeps the analog registers. Milestones 1-4b are as described above;
the update refusal (goal 4) still stops on SHA's DMA mode ("An update").

**What is left of 4c**, numbered as the handoff numbered it:

5. **Car write latency under a shared reader**, done ("Car write latency under a shared reader"):
   `Msc::times`, `cli/tests/oxidrive_latency.rs`, reader chunks of 512 bytes (the firmware as it
   stands) and 16 to 128 KiB with no, 1 and 2 ms of programming. A transfer costs a colliding car
   write its wire time; every pass with a programming time met the 0x107 race and stopped; the
   long waits were a priority inversion with libsmb2's signing on core 0. What is left is the
   firmware's (a race-free reader, the inversion, the buffer's alignment) and a measured
   programming time.
7. **The network under the firmware**: the NAT's window and window scaling, an MSS option in its
   SYN-ACK, and the round trip taken apart (where the 10.7 ms go) before the rate is compared
   with the board's; first why a window of 16384 or 65535 bytes stalls the first 1.5 MB write
   ("Upload throughput").
8. **The AXI GDMA** for SHA's (and AES's) DMA mode, which the update path and TLS need, then
   `external_oxidrive_refuses_an_unsigned_update` should pass.
10. **The owner's radio wedge** (carried from 4b's plan): a one-shot stub that makes the P4's
   `hosted_malloc_align` (esp_hosted's `port_esp_hosted_host_os.c`) return NULL at its Nth call
   would reproduce the board's wedge chain end to end; the fault hooks show where such a chain
   ends ("Faults").

**What part 1 leaves open**:

- oxidrive now refuses the car's commands while it holds the card, reports UNIT ATTENTION when it
  gives the card back (teslausb-p4 5633fc2, switchable since 1f18779) and counts a listing a read
  error cut short as a failure (63024af, ae63d65); the tests of findings 1 to 3 are turned round
  ("The window gate"). It does not hold a window that wrote the card longer, and its reads still do
  not wait for a ready card. When oxidrive changes again, the hazard tests that assert today's
  behaviour fail by design: turn their assertions round and keep them.
- Shared reading stayed on in every run here; the item-5 work on another branch saw a boot-sector
  read in `Card::geometry()` meet the card programming and turn shared reading off until a reboot.
  A different interleaving, not a contradiction: a run that meets it has no shared passes.
- Where the sources stop, the car's reactions are assumptions: what Tesla's MCU does after a
  medium change (here it reads the volume again, after an fsck when the volume is dirty, which it
  is while mounted), after a USB reset (here an fsck rewriting both FATs whole), and whether it
  writes MP4 the way `mp4_bytes` lays a clip out. Linux itself reports the change as an event and
  marks the disk for a partition rescan when it is next opened (v6.12 block/disk-events.c:272-284
  `disk_check_media_change`, which sd calls from `sd_open`, drivers/scsi/sd.c:1484-1488); a volume
  mounted on it is left as it is, and the car's userspace decides what follows.
- The writeback model is the testkit's: the car's own `dirty_writeback_centisecs` has not been
  read, the metadata lag has not been measured, and the data-first share is the model's (97.2%),
  close to the drive's 98.2% by construction rather than by fit.
- Builds with the settle gate on (`SHARED_SETTLE_MS`) and with reclaim configured, which 728b71f
  does not ship; more boots and clips for the writeback counts. The writeback run is slow: the
  emulated firmware polls CMD13 through every programming time the car's many small writes
  cause, about 2.7 times real time with the pair running.
- `--usb-host` still does not run with `--coproc`; the oxidrive tests drive the car from Rust.
- A digital system reset (LP_SYSTEM's SYS_SW_RST) of oxidrive was not tried again after the
  regi2c fix; the CPU reset was.

**Timed SD transfers** are there (`--sd-timing`). oxidrive's 11.0 MB/s with them against its
board's 10.9 MB/s is a coincidence, not a calibration ("USB", "Throughput"): cache and PSRAM
stalls are not modelled, and the hazard runs' 1500 µs programming time is an assumption. The
card's read access time and its programming time per block (`--sd-prg-us` is per write; measure
it on a card first) are what is left. An archive window's reads of the card on slot 0 and
ESP-Hosted's traffic on slot 1 share the one host, and timing both slots takes the upload from
0.53 to 0.35 MB/s ("Upload throughput").

The tusb_msc example shows two more things a window will meet ("Faults"): a port reset, which is
how Linux recovers from any transport error or timeout, takes the card back from an application
that does not set `auto_mount_off` (oxidrive sets it); and a bus reset during a write's data stage
makes TinyUSB's MSC class assert in `proc_write10_cmd` (harmless there; seen only in emulation, not
checked on silicon).
