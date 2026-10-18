//! The SD/MMC host (`Sdmmc`) and the SD card (`SdCard`) from outside the crate: the host's
//! self-clearing resets and START bit, its interrupt registers, the IDMAC's descriptor walk
//! with the OWN hand-back, suspend and resume; the card's identification sequence, R1 status,
//! data transfers, stop and errors (SD Physical Layer Simplified Specification v9.10).
use emu_core::Fault;
use esp_periph::sdmmc::card::{crc7, status, CardState, FileStore, MemStore, SdCard, SdStore};
use esp_periph::sdmmc::{des0, idsts, int, reg, SdDataError, SdDevice, SdResponse, Sdmmc, SdmmcLayout, SdmmcMemory, BMOD_DE, CMD_AUTO_STOP, CMD_DATA, CMD_RESP, CMD_RESP_LONG, CMD_START, CMD_UPDATE_CLK, CMD_WRITE, CTRL_INT_ENABLE, CTRL_USE_INTERNAL_DMA};
use esp_periph::Device;

const RCA_ARG: u32 = 0xe624 << 16;

/// A 4 MiB image whose bytes count up, block by block.
fn image() -> Vec<u8> { (0..4 << 20).map(|i: u32| (i / 512 + i % 251) as u8).collect() }
fn card() -> SdCard { SdCard::new(Box::new(MemStore(image()))).unwrap() }
fn short(r: Option<SdResponse>) -> u32 { match r { Some(SdResponse::Short(v)) => v, other => panic!("expected a short response, got {other:?}") } }
fn state(r1: u32) -> u32 { r1 >> 9 & 0xf }

/// CMD0, CMD8, ACMD41 until ready, CMD2, CMD3, CMD7: the card in transfer state.
fn to_tran(c: &mut SdCard) {
    assert_eq!(c.command(0, 0), None);
    assert_eq!(short(c.command(8, 0x1aa)), 0x1aa);
    loop {
        short(c.command(55, 0));
        if short(c.command(41, 0x40ff_8000)) & 1 << 31 != 0 { break; }
    }
    assert!(matches!(c.command(2, 0), Some(SdResponse::Long(_))));
    short(c.command(3, 0));
    short(c.command(7, RCA_ARG));
}

// ------------------------------------------------------------------ card

#[test]
fn crc7_matches_the_specification_examples() {
    // SD spec §4.5: CMD0 with argument 0 ends 0x95, CMD8 with 0x1AA ends 0x87.
    assert_eq!(crc7(&[0x40, 0, 0, 0, 0]) << 1 | 1, 0x95);
    assert_eq!(crc7(&[0x48, 0, 0, 1, 0xaa]) << 1 | 1, 0x87);
}

#[test]
fn identification_takes_the_card_from_idle_to_transfer() {
    let mut c = card();
    assert_eq!(c.command(0, 0), None, "CMD0 has no response");
    assert_eq!(short(c.command(8, 0x1aa)), 0x1aa, "CMD8 echoes the voltage and the check pattern (R7)");
    let r1 = short(c.command(55, 0));
    assert_eq!((state(r1), r1 & status::APP_CMD), (0, status::APP_CMD), "CMD55 in idle expects an ACMD");
    let ocr = short(c.command(41, 0x40ff_8000));
    assert_eq!(ocr & 1 << 31, 0, "the first ACMD41 finds the card still powering up");
    short(c.command(55, 0));
    let ocr = short(c.command(41, 0x40ff_8000));
    assert_eq!(ocr, 0xc0ff_8000, "ready, high capacity (CCS), 2.7-3.6 V");
    assert_eq!(c.state(), CardState::Ready);
    let Some(SdResponse::Long(cid)) = c.command(2, 0) else { panic!("CMD2 answers R2") };
    let name: Vec<u8> = (0..5).map(|i| { let p = 96 - 8 * i; (cid[p / 32] >> (p % 32)) as u8 }).collect();
    assert_eq!(&name, b"SIMSD", "CID product name");
    assert_eq!(cid[0] & 0xff, (crc7(&c.cid().to_be_bytes()[..15]) << 1 | 1) as u32, "CID ends with its CRC7 and a 1");
    let r6 = short(c.command(3, 0));
    assert_eq!((r6 >> 16, r6 >> 9 & 0xf), (0xe624, CardState::Ident as u32), "R6: the new RCA, the state CMD3 found");
    let Some(SdResponse::Long(csd)) = c.command(9, RCA_ARG) else { panic!("CMD9 answers R2") };
    assert_eq!(csd[3] >> 30, 1, "CSD version 2.0");
    let c_size = (csd[1] >> 16) | (csd[2] & 0x3f) << 16;
    assert_eq!((c_size as u64 + 1) * 1024, c.blocks(), "capacity from C_SIZE");
    assert_eq!(c.blocks(), 8192, "4 MiB");
    assert_eq!(c.command(9, 0x1234 << 16), None, "a card that is not addressed stays silent");
    let r1 = short(c.command(7, RCA_ARG));
    assert_eq!(state(r1), CardState::Stby as u32, "CMD7's R1b shows standby, the state it found");
    let r1 = short(c.command(13, RCA_ARG));
    assert_eq!((state(r1), r1 & status::READY_FOR_DATA), (CardState::Tran as u32, status::READY_FOR_DATA));
}

#[test]
fn the_card_needs_a_compatible_host() {
    let mut c = card();
    c.command(0, 0);
    assert_eq!(c.command(8, 0x2aa), None, "CMD8 with a voltage the card cannot take is not answered");
    for _ in 0..3 { short(c.command(55, 0)); assert_eq!(short(c.command(41, 0x40ff_8000)) & 1 << 31, 0, "without CMD8 a high capacity card stays busy"); }
    c.command(0, 0);
    short(c.command(8, 0x1aa));
    for _ in 0..3 { short(c.command(55, 0)); assert_eq!(short(c.command(41, 0x00ff_8000)) & 1 << 31, 0, "nor without HCS"); }
    short(c.command(55, 0));
    assert_eq!(short(c.command(41, 0)), 0x00ff_8000, "an empty voltage window only asks for the OCR");
}

#[test]
fn an_illegal_command_is_unanswered_and_reported_once() {
    let mut c = card();
    to_tran(&mut c);
    assert_eq!(c.command(2, 0), None, "CMD2 is not legal in transfer state");
    assert_eq!(c.command(52, 0x8000_0c08), None, "nor are the SDIO commands on a memory card");
    let r1 = short(c.command(13, RCA_ARG));
    assert_eq!(r1 & status::ILLEGAL_COMMAND, status::ILLEGAL_COMMAND, "the next response reports it");
    assert_eq!(short(c.command(13, RCA_ARG)) & status::ILLEGAL_COMMAND, 0, "and that clears it");
    assert_eq!(c.state(), CardState::Tran, "an illegal command changes no state");
}

#[test]
fn single_and_multi_block_transfers_and_stop() {
    let mut c = card();
    to_tran(&mut c);
    let img = image();
    let r1 = short(c.command(17, 3));
    assert_eq!(state(r1), CardState::Tran as u32);
    assert_eq!(c.state(), CardState::Data);
    let mut b = [0u8; 512];
    c.read_data(&mut b).unwrap();
    assert_eq!(&b[..], &img[3 * 512..4 * 512]);
    assert_eq!(c.state(), CardState::Tran, "back to transfer after one block");
    assert_eq!(c.read_data(&mut b), Err(SdDataError::Timeout), "nothing more to send");

    let data: Vec<u8> = (0..3 * 512).map(|i| (i * 7) as u8).collect();
    short(c.command(25, 100));
    assert_eq!(c.state(), CardState::Rcv);
    c.write_data(&data[..700]).unwrap();
    c.write_data(&data[700..]).unwrap();
    let r1 = short(c.command(12, 0));
    assert_eq!(state(r1), CardState::Rcv as u32, "CMD12 found the card receiving");
    assert_eq!(c.state(), CardState::Tran, "programming is instant");
    assert_eq!(c.blocks_written, 3);
    short(c.command(55, RCA_ARG));
    short(c.command(22, 0));
    let mut n = [0u8; 4];
    c.read_data(&mut n).unwrap();
    assert_eq!(u32::from_be_bytes(n), 3, "ACMD22 counts the blocks written");

    short(c.command(18, 99));
    let mut out = vec![0u8; 5 * 512];
    c.read_data(&mut out[..1000]).unwrap();
    c.read_data(&mut out[1000..]).unwrap();
    assert_eq!(&out[..512], &img[99 * 512..100 * 512]);
    assert_eq!(&out[512..4 * 512], &data[..]);
    assert_eq!(&out[4 * 512..], &img[103 * 512..104 * 512]);
    assert_eq!(c.state(), CardState::Data, "a multi-block read runs until CMD12");
    assert_eq!(state(short(c.command(12, 0))), CardState::Data as u32);
    assert_eq!(c.state(), CardState::Tran);
    assert_eq!(c.command(12, 0), None, "CMD12 with nothing to stop is illegal");

    short(c.command(23, 2));
    short(c.command(18, 10));
    let mut two = vec![0u8; 1024];
    c.read_data(&mut two).unwrap();
    assert_eq!(c.state(), CardState::Tran, "CMD23 ends the read after its count");
}

#[test]
fn addresses_beyond_the_card_are_out_of_range() {
    let mut c = card();
    to_tran(&mut c);
    let end = c.blocks() as u32;
    let r1 = short(c.command(17, end));
    assert_eq!(r1 & status::OUT_OF_RANGE, status::OUT_OF_RANGE);
    assert_eq!(c.state(), CardState::Tran, "and no data phase starts");
    let mut b = [0u8; 512];
    assert_eq!(c.read_data(&mut b), Err(SdDataError::Timeout));
    assert_eq!(short(c.command(13, RCA_ARG)) & status::OUT_OF_RANGE, 0, "cleared by the response that reported it");
    short(c.command(18, end - 1));
    let mut two = [0u8; 1024];
    assert_eq!(c.read_data(&mut two), Err(SdDataError::Timeout), "a multi-block read stops at the end");
    short(c.command(12, 0));
    let r1 = short(c.command(24, end));
    assert_eq!(r1 & status::OUT_OF_RANGE, status::OUT_OF_RANGE);
    assert_eq!(c.write_data(&[0; 512]), Err(SdDataError::NoCrcStatus), "the card does not take the block");
}

#[test]
fn registers_bus_width_and_high_speed() {
    let mut c = card();
    to_tran(&mut c);
    short(c.command(55, RCA_ARG));
    let r1 = short(c.command(51, 0));
    assert_eq!(r1 & status::APP_CMD, status::APP_CMD, "an ACMD's R1 says it was taken as one");
    let mut scr = [0u8; 8];
    c.read_data(&mut scr).unwrap();
    assert_eq!(scr[..4], [0x02, 0x35, 0x80, 0x02], "SCR: SD 3.0x, security 2.00, 1- and 4-bit, CMD23");
    assert_eq!(c.bus_width(), Some(1));
    short(c.command(55, RCA_ARG));
    short(c.command(6, 2));
    assert_eq!(c.bus_width(), Some(4), "ACMD6 selects four lines");
    short(c.command(55, RCA_ARG));
    short(c.command(13, 0));
    let mut ssr = [0u8; 64];
    c.read_data(&mut ssr).unwrap();
    assert_eq!(ssr[0] >> 6, 2, "SD status: four lines");
    short(c.command(6, 0x00ff_fff1));
    let mut sw = [0u8; 64];
    c.read_data(&mut sw).unwrap();
    assert_eq!((sw[12], sw[13]), (0x80, 0x03), "group 1 offers default and high speed");
    assert_eq!(sw[16] & 0xf, 1, "a check of function 1 reports it would be selected");
    assert_eq!(sw[17], 1, "data structure version 1");
    assert_eq!(c.csd() >> 96 & 0xff, 0x32, "a check switches nothing");
    short(c.command(6, 0x80ff_fff1));
    c.read_data(&mut sw).unwrap();
    assert_eq!(c.csd() >> 96 & 0xff, 0x5a, "TRAN_SPEED 50 MHz after the switch");
    short(c.command(6, 0x80ff_ff2f));
    c.read_data(&mut sw).unwrap();
    assert_eq!(sw[16] >> 4, 0xf, "an unsupported function reads 0xF");
    assert_eq!(c.csd() >> 96 & 0xff, 0x5a, "and nothing switches");
}

#[test]
fn erase_and_the_programming_time_hook() {
    let mut c = card();
    to_tran(&mut c);
    short(c.command(32, 5));
    short(c.command(33, 6));
    short(c.command(38, 0));
    short(c.command(17, 6));
    let mut b = [0xffu8; 512];
    c.read_data(&mut b).unwrap();
    assert!(b.iter().all(|&x| x == 0), "erased blocks read 0");
    assert_eq!(short(c.command(38, 0)) & status::ERASE_SEQ_ERROR, status::ERASE_SEQ_ERROR, "CMD38 needs CMD32 and CMD33");

    c.prg_polls = 3;
    short(c.command(24, 7));
    c.write_data(&[9; 512]).unwrap();
    assert_eq!(c.state(), CardState::Prg);
    assert!(c.busy(), "DAT0 busy while programming, one poll");
    for _ in 0..2 {
        let r1 = short(c.command(13, RCA_ARG));
        assert_eq!((state(r1), r1 & status::READY_FOR_DATA), (CardState::Prg as u32, 0));
    }
    let r1 = short(c.command(13, RCA_ARG));
    assert_eq!((state(r1), r1 & status::READY_FOR_DATA), (CardState::Tran as u32, status::READY_FOR_DATA), "done after its polls");
    assert!(!c.busy());
}

#[test]
fn file_images_copy_on_write_or_write_through() {
    let path = std::env::temp_dir().join(format!("esp32sim-sd-{}.img", std::process::id()));
    std::fs::write(&path, vec![0x11u8; 1 << 20]).unwrap();
    let p = path.to_str().unwrap();
    let mut cow = FileStore::copy_on_write(p).unwrap();
    cow.write(3, &[0x22; 512]).unwrap();
    let mut b = [0u8; 512];
    cow.read(3, &mut b).unwrap();
    assert_eq!(b[0], 0x22, "the overlay answers reads");
    assert!(std::fs::read(&path).unwrap().iter().all(|&x| x == 0x11), "the file is untouched");
    let mut wt = FileStore::write_through(p).unwrap();
    wt.write(3, &[0x33; 512]).unwrap();
    assert_eq!(std::fs::read(&path).unwrap()[3 * 512], 0x33);
    assert!(SdCard::new(Box::new(MemStore(vec![0; 1000]))).is_err(), "less than one capacity unit");
    std::fs::remove_file(&path).unwrap();
}

// ------------------------------------------------------------------ host

/// Memory at 0x4FF00000 for the IDMAC.
struct Ram(Vec<u8>);
const RAM: u32 = 0x4ff0_0000;
impl Ram {
    fn range(&self, a: u32, n: usize) -> Result<std::ops::Range<usize>, Fault> {
        let s = a.checked_sub(RAM).ok_or(Fault::Unmapped)? as usize;
        if s + n > self.0.len() { return Err(Fault::Unmapped); }
        Ok(s..s + n)
    }
    fn desc(&mut self, at: u32, des0: u32, size: u32, buf: u32, next: u32) {
        for (i, w) in [des0, size, buf, next].iter().enumerate() { let r = self.range(at + 4 * i as u32, 4).unwrap(); self.0[r].copy_from_slice(&w.to_le_bytes()); }
    }
    fn word(&self, at: u32) -> u32 { u32::from_le_bytes(self.0[self.range(at, 4).unwrap()].try_into().unwrap()) }
}
impl SdmmcMemory for Ram {
    fn read(&mut self, a: u32, buf: &mut [u8]) -> Result<(), Fault> { let r = self.range(a, buf.len())?; buf.copy_from_slice(&self.0[r]); Ok(()) }
    fn write(&mut self, a: u32, data: &[u8]) -> Result<(), Fault> { let r = self.range(a, data.len())?; self.0[r].copy_from_slice(data); Ok(()) }
}

fn host() -> (Sdmmc, Ram) {
    let mut h = Sdmmc::new(SdmmcLayout::Esp32p4);
    let i = h.attach(Box::new(card()));
    h.route[1] = Some(i);
    (h, Ram(vec![0; 64 << 10]))
}

/// Start a command on slot 1 and let the host carry it out; returns RINTSTS, cleared.
fn send(h: &mut Sdmmc, m: &mut Ram, index: u32, arg: u32, flags: u32) -> u32 {
    h.write(reg::CMDARG, arg);
    h.write(reg::CMD, CMD_START | 1 << 16 | flags | index);
    h.run(m);
    let r = h.read(reg::RINTSTS);
    h.write(reg::RINTSTS, r);
    r
}

fn host_to_tran(h: &mut Sdmmc, m: &mut Ram) {
    send(h, m, 0, 0, 0);
    send(h, m, 8, 0x1aa, CMD_RESP);
    loop {
        send(h, m, 55, 0, CMD_RESP);
        send(h, m, 41, 0x40ff_8000, CMD_RESP);
        if h.read(reg::RESP0) & 1 << 31 != 0 { break; }
    }
    send(h, m, 2, 0, CMD_RESP | CMD_RESP_LONG);
    send(h, m, 3, 0, CMD_RESP);
    send(h, m, 7, RCA_ARG, CMD_RESP);
}

#[test]
fn resets_and_start_bits_clear_themselves() {
    let (mut h, mut m) = host();
    h.write(reg::CTRL, 7 | CTRL_INT_ENABLE);
    assert_eq!(h.read(reg::CTRL), CTRL_INT_ENABLE, "controller, FIFO and DMA reset done; the rest stays");
    assert_eq!((h.read(reg::BLKSIZ), h.read(reg::BYTCNT), h.read(reg::TMOUT)), (512, 512, 0xffff_ff40), "reset values");
    assert_eq!(h.read(reg::VERID), 0x5432_270a);
    h.write(reg::CMD, CMD_START | CMD_UPDATE_CLK | 1 << 13);
    assert_eq!(h.read(reg::CMD) & CMD_START, 0, "a clock update is taken at once");
    assert!(!h.pending());
    assert_eq!(h.read(reg::RINTSTS), 0, "and raises no CMD_DONE");
    h.write(reg::CMDARG, 0x1aa);
    h.write(reg::CMD, CMD_START | 1 << 16 | CMD_RESP | 8);
    assert!(h.pending() && h.read(reg::CMD) & CMD_START != 0, "a command waits for the CIU");
    h.run(&mut m);
    assert_eq!(h.read(reg::CMD) & CMD_START, 0, "taken");
    assert_eq!(h.read(reg::RESP0), 0x1aa);
    assert_eq!(h.read(reg::STATUS) >> 11 & 0x3f, 8, "STATUS response index");
    h.write(reg::BMOD, 1 | BMOD_DE);
    assert_eq!(h.read(reg::BMOD), BMOD_DE, "the IDMAC's software reset clears itself");
}

#[test]
fn interrupts_mask_and_clear() {
    let (mut h, mut m) = host();
    h.write(reg::CMD, CMD_START | 1 << 16);
    h.run(&mut m);
    assert_eq!(h.read(reg::RINTSTS), int::CMD_DONE, "CMD0: done, nothing expected back");
    assert_eq!(h.read(reg::MINTSTS), 0, "masked");
    assert!(!h.irq());
    h.write(reg::INTMASK, int::CMD_DONE | int::RTO);
    assert_eq!(h.read(reg::MINTSTS), int::CMD_DONE);
    assert!(!h.irq(), "the global interrupt enable is off");
    h.write(reg::CTRL, CTRL_INT_ENABLE);
    assert!(h.irq());
    h.write(reg::RINTSTS, int::DATA_OVER);
    assert_eq!(h.read(reg::RINTSTS), int::CMD_DONE, "write 1 to clear: other bits stay");
    h.write(reg::RINTSTS, int::CMD_DONE);
    assert!(!h.irq());
    h.write(reg::CMD, CMD_START | CMD_RESP | 55);
    h.run(&mut m);
    assert_eq!(h.read(reg::RINTSTS), int::RTO | int::CMD_DONE, "slot 0 holds nothing: response timeout");
    assert_eq!(h.stats.timeouts, 1);
}

#[test]
fn responses_short_and_long() {
    let (mut h, mut m) = host();
    host_to_tran(&mut h, &mut m);
    send(&mut h, &mut m, 7, 0, 0);
    let r = send(&mut h, &mut m, 9, RCA_ARG, CMD_RESP | CMD_RESP_LONG);
    assert_eq!(r, int::CMD_DONE);
    assert_eq!(h.read(reg::RESP0 + 12) >> 30, 1, "RESP3 holds CSD bits 127:96");
    send(&mut h, &mut m, 7, RCA_ARG, CMD_RESP);
    send(&mut h, &mut m, 13, RCA_ARG, CMD_RESP);
    assert_eq!(h.read(reg::RESP0) >> 9 & 0xf, CardState::Tran as u32);
}

/// IDMAC on, interrupts enabled, `bytes` in blocks of 512.
fn arm(h: &mut Sdmmc, bytes: u32, first: u32) {
    h.write(reg::CTRL, CTRL_INT_ENABLE | CTRL_USE_INTERNAL_DMA | 1 << 5);
    h.write(reg::BMOD, BMOD_DE | 1 << 1);
    h.write(reg::IDINTEN, idsts::TI | idsts::RI | idsts::NIS);
    h.write(reg::INTMASK, int::ALL);
    h.write(reg::BYTCNT, bytes);
    h.write(reg::BLKSIZ, 512);
    h.write(reg::DBADDR, first);
    h.write(reg::PLDMND, 1);
}

#[test]
fn the_idmac_walks_chained_descriptors_and_hands_them_back() {
    let (mut h, mut m) = host();
    host_to_tran(&mut h, &mut m);
    let (d0, d1, b0, b1) = (RAM, RAM + 0x40, RAM + 0x1000, RAM + 0x2000);
    m.desc(d0, des0::OWN | des0::CH | des0::FS, 1024, b0, d1);
    m.desc(d1, des0::OWN | des0::CH | des0::LD, 512, b1, 0);
    arm(&mut h, 3 * 512, d0);
    let r = send(&mut h, &mut m, 18, 20, CMD_RESP | CMD_DATA | CMD_AUTO_STOP);
    assert_eq!(r, int::CMD_DONE | int::DATA_OVER | int::ACD, "command, data and the auto-stop");
    let img = image();
    assert_eq!(&m.0[0x1000..0x1400], &img[20 * 512..22 * 512]);
    assert_eq!(&m.0[0x2000..0x2200], &img[22 * 512..23 * 512]);
    assert_eq!((m.word(d0) & des0::OWN, m.word(d1) & des0::OWN), (0, 0), "both descriptors handed back");
    assert_eq!(m.word(d1), des0::CH | des0::LD, "with their other bits kept");
    assert_eq!(h.read(reg::IDSTS), idsts::RI | idsts::NIS, "receive interrupt and its summary");
    assert!(h.irq());
    assert_eq!(h.read(reg::RESP0 + 4) >> 9 & 0xf, CardState::Data as u32, "RESP1: the auto-stop's R1b");
    assert_eq!((h.read(reg::TCBCNT), h.read(reg::TBBCNT)), (1536, 1536));
    assert_eq!(h.read(reg::DSCADDR), d1);
    h.write(reg::IDSTS, idsts::RI | idsts::NIS);
    assert!(!h.irq());
    send(&mut h, &mut m, 13, RCA_ARG, CMD_RESP);
    assert_eq!(h.read(reg::RESP0) >> 9 & 0xf, CardState::Tran as u32);
}

#[test]
fn the_idmac_suspends_on_a_cpu_owned_descriptor_and_resumes_on_poll_demand() {
    let (mut h, mut m) = host();
    host_to_tran(&mut h, &mut m);
    let (d0, d1) = (RAM, RAM + 0x40);
    m.desc(d0, des0::OWN | des0::CH, 512, RAM + 0x1000, d1);
    m.desc(d1, des0::CH | des0::LD, 512, RAM + 0x1200, 0);
    arm(&mut h, 1024, d0);
    h.write(reg::IDINTEN, idsts::TI | idsts::RI | idsts::NIS | idsts::DU | idsts::AIS);
    let r = send(&mut h, &mut m, 18, 0, CMD_RESP | CMD_DATA | CMD_AUTO_STOP);
    assert_eq!(r, int::CMD_DONE, "no data-over yet");
    assert_eq!(h.read(reg::IDSTS), idsts::RI | idsts::NIS | idsts::DU | idsts::AIS, "one block in, then descriptor unavailable");
    assert_eq!(h.read(reg::DSCADDR), d1);
    h.write(reg::IDSTS, idsts::W1C);
    h.write(reg::PLDMND, 1);
    h.run(&mut m);
    assert_eq!(h.read(reg::IDSTS), idsts::DU | idsts::AIS, "a poll demand re-reads the same descriptor");
    h.write(reg::IDSTS, idsts::W1C);
    m.desc(d1, des0::OWN | des0::CH | des0::LD, 512, RAM + 0x1200, 0);
    h.write(reg::PLDMND, 1);
    assert!(h.pending());
    h.run(&mut m);
    assert_eq!(h.read(reg::RINTSTS), int::DATA_OVER | int::ACD);
    assert_eq!(h.read(reg::IDSTS), idsts::RI | idsts::NIS);
    assert_eq!(&m.0[0x1000..0x1400], &image()[..1024]);
}

#[test]
fn a_write_moves_memory_to_the_card() {
    let (mut h, mut m) = host();
    host_to_tran(&mut h, &mut m);
    for (i, b) in m.0[0x1000..0x1600].iter_mut().enumerate() { *b = (i % 13) as u8; }
    m.desc(RAM, des0::OWN | des0::CH | des0::LD | des0::DIC, 0x600, RAM + 0x1000, 0);
    arm(&mut h, 0x600, RAM);
    let r = send(&mut h, &mut m, 25, 40, CMD_RESP | CMD_DATA | CMD_WRITE | CMD_AUTO_STOP);
    assert_eq!(r, int::CMD_DONE | int::DATA_OVER | int::ACD);
    assert_eq!(h.read(reg::IDSTS), 0, "DIC: no transmit interrupt for this descriptor");
    assert_eq!(m.word(RAM) & des0::OWN, 0);
    assert_eq!(h.stats.bytes_written, 0x600);
    m.desc(RAM, des0::OWN | des0::CH | des0::LD, 0x600, RAM + 0x3000, 0);
    arm(&mut h, 0x600, RAM);
    send(&mut h, &mut m, 18, 40, CMD_RESP | CMD_DATA | CMD_AUTO_STOP);
    assert_eq!(m.0[0x3000..0x3600], m.0[0x1000..0x1600], "read back what was written");
}

/// With CARDTHRCTL's busy-clear interrupt on, a write ends with BCI once DAT0 is released; the
/// card's programming time holds STATUS.data_busy until then.
#[test]
fn busy_clear_follows_the_card_after_a_write() {
    let mut h = Sdmmc::new(SdmmcLayout::Esp32p4);
    let mut c = card();
    c.prg_polls = 1;
    let i = h.attach(Box::new(c));
    h.route[1] = Some(i);
    let mut m = Ram(vec![0; 64 << 10]);
    host_to_tran(&mut h, &mut m);
    h.write(reg::CARDTHRCTL, 1 << 1);
    m.desc(RAM, des0::OWN | des0::CH | des0::LD, 512, RAM + 0x1000, 0);
    arm(&mut h, 512, RAM);
    let r = send(&mut h, &mut m, 24, 9, CMD_RESP | CMD_DATA | CMD_WRITE);
    assert_eq!(r, int::CMD_DONE | int::DATA_OVER, "the card is still programming: no BCI yet");
    assert_eq!(h.read(reg::STATUS) >> 9 & 1, 0, "it has released DAT0 at the next look");
    assert_eq!(h.read(reg::RINTSTS), int::BCI);
    h.write(reg::RINTSTS, int::BCI);
    h.write(reg::CARDTHRCTL, 0);
    m.desc(RAM, des0::OWN | des0::CH | des0::LD, 512, RAM + 0x1000, 0);
    arm(&mut h, 512, RAM);
    let r = send(&mut h, &mut m, 24, 10, CMD_RESP | CMD_DATA | CMD_WRITE);
    assert_eq!(r, int::CMD_DONE | int::DATA_OVER, "no BCI with the enable off");
    assert_eq!(h.read(reg::STATUS) >> 9 & 1, 1, "DAT0 busy while the card programs");
    assert_eq!(h.read(reg::STATUS) >> 9 & 1, 0);
    assert_eq!(h.read(reg::RINTSTS), 0);
}

#[test]
fn faults_widths_and_missing_data_end_the_transfer() {
    let (mut h, mut m) = host();
    host_to_tran(&mut h, &mut m);
    m.desc(RAM, des0::OWN | des0::CH | des0::LD, 512, 0x1000_0000, 0);
    arm(&mut h, 512, RAM);
    h.write(reg::IDINTEN, idsts::W1C);
    send(&mut h, &mut m, 17, 0, CMD_RESP | CMD_DATA);
    assert_eq!(h.read(reg::IDSTS) & (idsts::FBE | idsts::AIS), idsts::FBE | idsts::AIS, "an unreachable buffer is a fatal bus error");
    assert_eq!(h.read(reg::IDSTS) >> idsts::FBE_CODE_SHIFT & 7, 2, "during reception");
    send(&mut h, &mut m, 12, 0, CMD_RESP);
    h.write(reg::IDSTS, idsts::W1C);

    send(&mut h, &mut m, 55, RCA_ARG, CMD_RESP);
    send(&mut h, &mut m, 6, 2, CMD_RESP);
    m.desc(RAM, des0::OWN | des0::CH | des0::LD, 512, RAM + 0x1000, 0);
    arm(&mut h, 512, RAM);
    let r = send(&mut h, &mut m, 17, 0, CMD_RESP | CMD_DATA);
    assert_eq!(r, int::CMD_DONE | int::DCRC | int::DATA_OVER, "the card on four lines, the host on one: CRC errors");
    h.write(reg::CTYPE, 1 << 1);
    m.desc(RAM, des0::OWN | des0::CH | des0::LD, 1024, RAM + 0x1000, 0);
    arm(&mut h, 1024, RAM);
    let r = send(&mut h, &mut m, 17, 0, CMD_RESP | CMD_DATA);
    assert_eq!(r, int::CMD_DONE | int::DRTO | int::DATA_OVER, "one block asked for two: the second times out");
    assert_eq!(h.read(reg::IDSTS) & idsts::CES, idsts::CES);
}
