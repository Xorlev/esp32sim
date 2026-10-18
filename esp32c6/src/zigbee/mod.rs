//! A virtual Zigbee coordinator on the air, so an unmodified Zigbee end device — the ESP Zigbee
//! SDK's ZBOSS on the emulated 802.15.4 MAC — has something to join, and a script has a network
//! to drive it from: read and write its attributes, bind it, enrol its IAS zone, watch its reports.
//!
//! It is the host's side of the medium, not an emulated node: it answers beacon requests with a
//! beacon, associates the device, acknowledges its frames with the pending bit set when it holds
//! traffic for it, hands that traffic over when the (sleepy) device polls, and speaks enough of
//! NWK, APS, ZDO and ZCL — with the real security: AES-CCM* at the network layer, the default
//! trust-centre link key hashed into the key-transport and key-load keys, the R21 link-key
//! exchange — to be a trust centre the device cannot tell from a Zigbee2MQTT dongle.
//!
//! Timing is exact in the emulated clock: an acknowledgement starts on the air 12 symbols after
//! the frame it acknowledges ends, a queued frame leaves after the acknowledgement of the data
//! request that asked for it, and a frame the device's radio was not listening for is retried
//! and then held for the next poll. Everything it does is one `[zb]` line.
//!
//! `mod.rs` is the MAC and the clock: what to inject when. `stack.rs` is everything above it.

pub mod crypto;
mod stack;
#[cfg(test)]
mod tests;

use crate::periph::CPU_HZ;
use crate::radio::{air_cycles, ACK_TURNAROUND_NS, PHY_OVERHEAD_BYTES};
use std::collections::VecDeque;

/// Zigbee2MQTT's defaults, so a device paired here looks like one paired there.
pub const DEFAULT_PAN: u16 = 0x1a62;
pub const DEFAULT_EXT_PAN: [u8; 8] = [0xdd; 8];
pub const DEFAULT_CHANNEL: u8 = 11;
pub const DEFAULT_NWK_KEY: [u8; 16] = [0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0d];
/// "ZigBeeAlliance09": the well-known default trust-centre link key every joining device has.
pub const DEFAULT_TC_LINK_KEY: [u8; 16] = *b"ZigBeeAlliance09";
/// The coordinator's own IEEE address (on-air order, least significant byte first):
/// 00:12:4b:00:00:00:00:01.
pub const DEFAULT_IEEE: [u8; 8] = [0x01, 0x00, 0x00, 0x00, 0x00, 0x4b, 0x12, 0x00];
/// The endpoint the coordinator's own ZCL traffic comes from, as Zigbee2MQTT's does.
pub const COORD_ENDPOINT: u8 = 1;
pub const HA_PROFILE: u16 = 0x0104;

pub const RSSI_DBM: i8 = -60;
pub const LQI: u8 = 255;

const fn us(n: u64) -> u64 { n * (CPU_HZ / 1_000_000) }
const fn ms(n: u64) -> u64 { n * (CPU_HZ / 1_000) }
/// Preamble, SFD, length, PSDU, FCS: the whole PPDU on the air.
fn ppdu_cycles(psdu_len: usize) -> u64 { air_cycles(PHY_OVERHEAD_BYTES + psdu_len as u64 + 2) }
/// A 3-byte immediate acknowledgement on the air.
fn ack_cycles() -> u64 { ppdu_cycles(3) }
/// Long interframe spacing: 40 symbols.
const LIFS: u64 = us(640);

#[derive(Clone, Debug)]
pub struct Config {
    pub pan: u16,
    pub ext_pan: [u8; 8],
    pub channel: u8,
    pub nwk_key: [u8; 16],
    pub tc_link_key: [u8; 16],
    pub ieee: [u8; 8],
    pub permit_join: bool,
    /// narrate every frame, not only the events
    pub verbose: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config { pan: DEFAULT_PAN, ext_pan: DEFAULT_EXT_PAN, channel: DEFAULT_CHANNEL, nwk_key: DEFAULT_NWK_KEY, tc_link_key: DEFAULT_TC_LINK_KEY, ieee: DEFAULT_IEEE, permit_join: true, verbose: false }
    }
}

fn parse_hex_bytes<const N: usize>(s: &str, what: &str) -> Result<[u8; N], String> {
    let clean: String = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    if clean.len() != 2 * N { return Err(format!("{what}: expected {N} bytes of hex, got {s:?}")); }
    let mut out = [0u8; N];
    for (i, b) in out.iter_mut().enumerate() { *b = u8::from_str_radix(&clean[2 * i..2 * i + 2], 16).map_err(|e| format!("{what}: {e}"))?; }
    Ok(out)
}

impl Config {
    /// `chan=15,pan=0x1a62,epan=dddddddddddddddd,key=<32 hex>,tclk=<32 hex>,ieee=00124b0000000001,permit=0,verbose`.
    /// An IEEE address is written most significant byte first, as it is printed.
    pub fn parse(spec: &str) -> Result<Config, String> {
        let mut c = Config::default();
        for item in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
            let (k, v) = item.split_once('=').unwrap_or((item, ""));
            let num = |v: &str| -> Result<u64, String> {
                if let Some(h) = v.strip_prefix("0x") { u64::from_str_radix(h, 16) } else { v.parse() }.map_err(|_| format!("{k}: bad number {v:?}"))
            };
            match k {
                "chan" | "channel" => { let n = num(v)?; if !(11..=26).contains(&n) { return Err(format!("chan: {n} is not an 802.15.4 channel (11..26)")); } c.channel = n as u8; }
                "pan" => c.pan = num(v)? as u16,
                "epan" | "extpan" => { let mut b: [u8; 8] = parse_hex_bytes(v, "epan")?; b.reverse(); c.ext_pan = b; }
                "key" | "nwkkey" => c.nwk_key = parse_hex_bytes(v, "key")?,
                "tclk" | "linkkey" => c.tc_link_key = parse_hex_bytes(v, "tclk")?,
                "ieee" => { let mut b: [u8; 8] = parse_hex_bytes(v, "ieee")?; b.reverse(); c.ieee = b; }
                "permit" => c.permit_join = v != "0" && v != "false",
                "verbose" => c.verbose = v.is_empty() || (v != "0" && v != "false"),
                _ => return Err(format!("--zigbee: unknown key {k:?} (chan, pan, epan, key, tclk, ieee, permit, verbose)")),
            }
        }
        Ok(c)
    }
}

/// An IEEE address as it is printed: most significant byte first, colon separated.
pub fn ieee_str(a: &[u8; 8]) -> String { a.iter().rev().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":") }
pub fn hex(b: &[u8]) -> String { b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join("") }

// ------------------------------------------------------------------ the MAC header

/// A parsed 802.15.4 MAC header: enough to route and to answer.
#[derive(Clone, Debug, Default)]
pub struct Mac {
    pub frame_type: u8, pub security: bool, pub pending: bool, pub ack_request: bool, pub pan_compression: bool, pub version: u8,
    pub seq: u8,
    pub dst_pan: Option<u16>, pub dst: Addr,
    pub src_pan: Option<u16>, pub src: Addr,
    /// where the payload starts
    pub hdr_len: usize,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Addr { #[default] None, Short(u16), Ext([u8; 8]) }

impl Addr {
    fn mode(&self) -> u16 { match self { Addr::None => 0, Addr::Short(_) => 2, Addr::Ext(_) => 3 } }
    fn put(&self, out: &mut Vec<u8>) { match self { Addr::None => {}, Addr::Short(s) => out.extend_from_slice(&s.to_le_bytes()), Addr::Ext(e) => out.extend_from_slice(e) } }
    pub fn short(&self) -> Option<u16> { if let Addr::Short(s) = self { Some(*s) } else { None } }
    pub fn ext(&self) -> Option<[u8; 8]> { if let Addr::Ext(e) = self { Some(*e) } else { None } }
}

impl std::fmt::Display for Addr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self { Addr::None => write!(f, "-"), Addr::Short(s) => write!(f, "{s:#06x}"), Addr::Ext(e) => write!(f, "{}", ieee_str(e)) }
    }
}

pub fn parse_mac(f: &[u8]) -> Option<Mac> {
    if f.len() < 3 { return None; }
    let fcf = u16::from_le_bytes([f[0], f[1]]);
    let mut m = Mac { frame_type: (fcf & 7) as u8, security: fcf & 8 != 0, pending: fcf & 0x10 != 0, ack_request: fcf & 0x20 != 0, pan_compression: fcf & 0x40 != 0, version: ((fcf >> 12) & 3) as u8, seq: f[2], ..Default::default() };
    let (dst_mode, src_mode) = ((fcf >> 10) & 3, (fcf >> 14) & 3);
    let mut i = 3;
    fn addr(f: &[u8], i: &mut usize, mode: u16) -> Option<Addr> {
        Some(match mode {
            2 => { let s = f.get(*i..*i + 2)?; *i += 2; Addr::Short(u16::from_le_bytes([s[0], s[1]])) }
            3 => { let e = f.get(*i..*i + 8)?; *i += 8; let mut a = [0u8; 8]; a.copy_from_slice(e); Addr::Ext(a) }
            _ => Addr::None,
        })
    }
    fn pan(f: &[u8], i: &mut usize) -> Option<u16> { let p = f.get(*i..*i + 2)?; *i += 2; Some(u16::from_le_bytes([p[0], p[1]])) }
    if dst_mode != 0 { m.dst_pan = Some(pan(f, &mut i)?); m.dst = addr(f, &mut i, dst_mode)?; }
    if src_mode != 0 {
        if !m.pan_compression { m.src_pan = Some(pan(f, &mut i)?); } else { m.src_pan = m.dst_pan; }
        m.src = addr(f, &mut i, src_mode)?;
    }
    if m.security { return None; }   // MAC-level security: Zigbee does not use it
    m.hdr_len = i;
    Some(m)
}

/// Build a MAC frame. `pan_compression` whenever both addresses are present and on our PAN.
#[allow(clippy::too_many_arguments)]
fn mac_frame(frame_type: u8, ack_request: bool, pending: bool, seq: u8, dst_pan: Option<u16>, dst: Addr, src_pan: Option<u16>, src: Addr, payload: &[u8]) -> Vec<u8> {
    let compress = dst_pan.is_some() && src_pan.is_some() && dst_pan == src_pan && dst != Addr::None && src != Addr::None;
    let mut fcf = frame_type as u16 | (dst.mode() << 10) | (src.mode() << 14);
    if ack_request { fcf |= 0x20; }
    if pending { fcf |= 0x10; }
    if compress { fcf |= 0x40; }
    let mut out = fcf.to_le_bytes().to_vec();
    out.push(seq);
    if let Some(p) = dst_pan { if dst != Addr::None { out.extend_from_slice(&p.to_le_bytes()); dst.put(&mut out); } }
    if src != Addr::None { if !compress { out.extend_from_slice(&src_pan.unwrap_or(0xffff).to_le_bytes()); } src.put(&mut out); }
    out.extend_from_slice(payload);
    out
}

// ------------------------------------------------------------------ scheduling

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind { Ack, Beacon, Data }

/// A frame the coordinator will put on the air at `at` (a cycle of the device's clock).
#[derive(Clone, Debug)]
pub struct Pending { pub at: u64, pub frame: Vec<u8>, pub kind: Kind, pub tries: u8, pub what: String }

/// One device the coordinator knows: the only child a single-device network has, but a table
/// so a second one is a loop, not a rewrite.
#[derive(Clone, Debug)]
pub struct Device {
    pub ieee: [u8; 8],
    pub short: u16,
    /// the association response has been queued (MAC-level member)
    pub associated: bool,
    /// the network key has been sent
    pub keyed: bool,
    /// device announce seen
    pub announced: bool,
    /// the trust-centre link key this device holds: the default until the R21 exchange replaces it
    pub link_key: [u8; 16],
    pub link_key_updated: bool,
    pub endpoints: Vec<u8>,
    /// IAS zone ids handed out, by endpoint
    pub zones: Vec<(u8, u8)>,
    pub polls: u64,
    pub last_poll: u64,
}

pub struct Coordinator {
    pub cfg: Config,
    pub devices: Vec<Device>,
    next_short: u16,
    /// MAC data sequence number, beacon sequence number
    dsn: u8, bsn: u8,
    nwk_seq: u8, nwk_fc: u32, aps_counter: u8, aps_fc: u32, zcl_tsn: u8, zdo_tsn: u8,
    /// frames waiting for the device to poll, oldest first
    pub queue: VecDeque<(Vec<u8>, String)>,
    /// frames due on the air, by time
    inject: Vec<Pending>,
    /// our frame on the air that asked for an acknowledgement: (seq, deadline, frame, what)
    awaiting_ack: Option<(u8, u64, Vec<u8>, String, u8)>,
    /// the IAS zone id to hand out next
    next_zone: u8,
    pub stats: Stats,
    pub lines: Vec<String>,
    /// print `[zb]` lines as they happen (false: keep them for `lines` only)
    pub print: bool,
    /// the device the next script command addresses (set by the first join)
    target: Option<usize>,
    /// script commands that arrived before any device joined
    deferred: Vec<String>,
}

#[derive(Clone, Debug, Default)]
pub struct Stats { pub beacons: u64, pub acks_sent: u64, pub acks_received: u64, pub acks_missed: u64, pub frames_in: u64, pub frames_out: u64, pub polls: u64, pub not_taken: u64, pub nwk_bad_mic: u64, pub reports: u64, pub notifications: u64 }

impl Coordinator {
    pub fn new(cfg: Config) -> Self {
        Coordinator { cfg, devices: Vec::new(), next_short: 0x1234, dsn: 0x10, bsn: 0x40, nwk_seq: 0x80, nwk_fc: 0, aps_counter: 0x20, aps_fc: 0, zcl_tsn: 0, zdo_tsn: 0,
                      queue: VecDeque::new(), inject: Vec::new(), awaiting_ack: None, next_zone: 1, stats: Stats::default(), lines: Vec::new(), print: true, target: None, deferred: Vec::new() }
    }

    pub(crate) fn log(&mut self, now: u64, msg: impl AsRef<str>) {
        let line = format!("[zb] t={:.3}s {}", now as f64 / CPU_HZ as f64, msg.as_ref());
        if self.print { eprintln!("{line}"); }
        self.lines.push(line);
    }
    fn trace(&mut self, now: u64, msg: impl AsRef<str>) { if self.cfg.verbose { self.log(now, msg); } }

    fn next_dsn(&mut self) -> u8 { self.dsn = self.dsn.wrapping_add(1); self.dsn }

    /// When the coordinator next needs the clock: the earliest frame due on the air, or the
    /// acknowledgement it is waiting for timing out.
    pub fn next_deadline(&self) -> Option<u64> {
        let inj = self.inject.iter().map(|p| p.at).min();
        let ack = self.awaiting_ack.as_ref().map(|a| a.1);
        match (inj, ack) { (Some(a), Some(b)) => Some(a.min(b)), (a, b) => a.or(b) }
    }

    /// Frames due at or before `now`, in time order. The caller puts each on the air and reports
    /// back with `outcome`.
    pub fn take_due(&mut self, now: u64) -> Vec<Pending> {
        if let Some((seq, deadline, frame, what, tries)) = self.awaiting_ack.take() {
            if now >= deadline {
                self.stats.acks_missed += 1;
                if tries < 3 {
                    self.log(now, format!("no ack for {what} (seq {seq}); holding it for the next poll"));
                    self.queue.push_front((frame, what));
                } else {
                    self.log(now, format!("no ack for {what} after {tries} tries; dropped"));
                }
            } else {
                self.awaiting_ack = Some((seq, deadline, frame, what, tries));
            }
        }
        let mut due: Vec<Pending> = Vec::new();
        let mut i = 0;
        while i < self.inject.len() {
            if self.inject[i].at <= now { due.push(self.inject.remove(i)); } else { i += 1; }
        }
        due.sort_by_key(|p| p.at);
        due
    }

    /// What became of a frame handed to the radio: taken (on the air from `now`) or refused (the
    /// device was not listening).
    pub fn outcome(&mut self, now: u64, mut p: Pending, taken: bool) {
        if taken {
            self.stats.frames_out += 1;
            match p.kind {
                Kind::Ack => self.stats.acks_sent += 1,
                Kind::Beacon => self.stats.beacons += 1,
                Kind::Data => {
                    let ar = p.frame.first().is_some_and(|b| b & 0x20 != 0);
                    if ar {
                        // the acknowledgement follows the frame's end; allow it the turnaround and some slack
                        let deadline = now + ppdu_cycles(p.frame.len()) + us(ACK_TURNAROUND_NS / 1000) + ack_cycles() + ms(2);
                        self.awaiting_ack = Some((p.frame[2], deadline, p.frame.clone(), p.what.clone(), p.tries));
                    }
                }
            }
            self.trace(now, format!("-> {} ({} bytes)", p.what, p.frame.len()));
            return;
        }
        self.stats.not_taken += 1;
        p.tries += 1;
        match p.kind {
            Kind::Ack => self.trace(now, "ack not taken: the device was not waiting for it"),
            Kind::Beacon => { if p.tries < 3 { p.at = now + us(500); self.inject.push(p); } else { self.trace(now, "beacon not taken; the scan moved on"); } }
            Kind::Data => {
                if p.tries < 6 { p.at = now + us(500); self.inject.push(p); }
                else { self.log(now, format!("{} not taken by the radio; holding it for the next poll", p.what)); self.queue.push_front((p.frame, p.what)); }
            }
        }
    }

    fn schedule(&mut self, at: u64, frame: Vec<u8>, kind: Kind, what: String) { self.inject.push(Pending { at, frame, kind, tries: 0, what }); }

    /// Queue a frame for the device: it leaves when the device next polls.
    pub(crate) fn enqueue(&mut self, frame: Vec<u8>, what: String) { self.queue.push_back((frame, what)); }

    /// A MAC data frame to the device's short address, acknowledged, carrying `payload`.
    pub(crate) fn data_frame(&mut self, dst: u16, payload: &[u8]) -> Vec<u8> {
        let seq = self.next_dsn();
        mac_frame(1, true, false, seq, Some(self.cfg.pan), Addr::Short(dst), Some(self.cfg.pan), Addr::Short(0x0000), payload)
    }

    fn beacon(&mut self) -> Vec<u8> {
        self.bsn = self.bsn.wrapping_add(1);
        let mut payload = Vec::new();
        // superframe spec: beacon order 15, superframe order 15, final CAP slot 15, PAN coordinator, association permit
        let sf: u16 = 0x0fff | (1 << 14) | if self.cfg.permit_join { 1 << 15 } else { 0 };
        payload.extend_from_slice(&sf.to_le_bytes());
        payload.push(0x00);                       // GTS: none
        payload.push(0x00);                       // pending addresses: none
        // Zigbee beacon payload: protocol id, stack profile 2 / protocol version 2, router capacity
        // + depth 0 + end-device capacity, extended PAN id, tx offset, update id
        payload.push(0x00);
        payload.push(0x22);
        payload.push(0x84);
        payload.extend_from_slice(&self.cfg.ext_pan);
        payload.extend_from_slice(&[0xff, 0xff, 0xff]);
        payload.push(0x00);
        mac_frame(0, false, false, self.bsn, None, Addr::None, Some(self.cfg.pan), Addr::Short(0x0000), &payload)
    }

    fn device_by_ieee(&self, ieee: &[u8; 8]) -> Option<usize> { self.devices.iter().position(|d| d.ieee == *ieee) }
    fn device_by_short(&self, short: u16) -> Option<usize> { self.devices.iter().position(|d| d.short == short) }
    pub(crate) fn device_of(&self, src: &Addr) -> Option<usize> {
        match src { Addr::Short(s) => self.device_by_short(*s), Addr::Ext(e) => self.device_by_ieee(e), Addr::None => None }
    }

    /// The device transmitted `frame` (MAC header + payload, no FCS) on `channel`, starting at
    /// `now`. Everything the coordinator does follows from here.
    pub fn on_tx(&mut self, now: u64, channel: u8, frame: &[u8]) {
        let Some(m) = parse_mac(frame) else { self.trace(now, format!("unparsable frame on ch {channel}: {}", hex(frame))); return; };
        if channel != self.cfg.channel {
            if m.frame_type == 3 && frame.get(m.hdr_len) == Some(&0x07) { self.trace(now, format!("beacon request on channel {channel}: not ours")); }
            return;
        }
        self.stats.frames_in += 1;
        let end = now + ppdu_cycles(frame.len());
        if m.frame_type == 2 {
            self.stats.acks_received += 1;
            if let Some((seq, _, _, what, _)) = self.awaiting_ack.take() {
                if seq == m.seq { self.trace(now, format!("ack for {what}")); }
                else { self.log(now, format!("ack with seq {} while waiting for seq {seq} ({what})", m.seq)); }
            }
            return;
        }
        let payload = &frame[m.hdr_len..];
        // the immediate acknowledgement, with the pending bit if this is a poll we have traffic for
        let is_poll = m.frame_type == 3 && payload.first() == Some(&0x04);
        let pending = is_poll && !self.queue.is_empty();
        let ack_at = end + us(ACK_TURNAROUND_NS / 1000);
        if m.ack_request && m.version <= 1 {
            let ack = vec![0x02 | if pending { 0x10 } else { 0 }, 0x00, m.seq];
            self.schedule(ack_at, ack, Kind::Ack, format!("ack seq {}", m.seq));
        }
        match m.frame_type {
            3 => self.on_mac_command(now, end, ack_at, &m, payload, pending),
            1 => {
                let src = m.src;
                if self.device_of(&src).is_none() {
                    self.log(now, format!("data frame from unknown {src}; ignored"));
                    return;
                }
                self.on_nwk(now, &m, payload);
            }
            0 => self.trace(now, "a beacon from the device?"),
            _ => {}
        }
    }

    fn on_mac_command(&mut self, now: u64, end: u64, ack_at: u64, m: &Mac, payload: &[u8], pending: bool) {
        match payload.first() {
            Some(0x07) => {
                // beacon request: the beacon goes out after the request, within the scan's listen window
                let b = self.beacon();
                self.log(now, format!("beacon request on channel {}: beacon (PAN {:#06x}, permit join {})", self.cfg.channel, self.cfg.pan, self.cfg.permit_join));
                self.schedule(end + ms(2), b, Kind::Beacon, "beacon".into());
            }
            Some(0x01) => {
                let Some(ieee) = m.src.ext() else { self.log(now, "association request without an extended source address"); return; };
                let cap = payload.get(1).copied().unwrap_or(0);
                if !self.cfg.permit_join {
                    self.log(now, format!("association request from {} refused: joining not permitted", ieee_str(&ieee)));
                    let seq = self.next_dsn();
                    let rsp = mac_frame(3, true, false, seq, Some(self.cfg.pan), Addr::Ext(ieee), Some(self.cfg.pan), Addr::Ext(self.cfg.ieee), &[0x02, 0xff, 0xff, 0x01]);
                    self.enqueue(rsp, "association response (denied)".into());
                    return;
                }
                let idx = match self.device_by_ieee(&ieee) {
                    Some(i) => i,
                    None => {
                        let short = self.next_short; self.next_short += 1;
                        self.devices.push(Device { ieee, short, associated: false, keyed: false, announced: false, link_key: self.cfg.tc_link_key, link_key_updated: false, endpoints: Vec::new(), zones: Vec::new(), polls: 0, last_poll: 0 });
                        self.devices.len() - 1
                    }
                };
                let short = self.devices[idx].short;
                self.devices[idx].associated = true;
                self.devices[idx].keyed = false;
                self.target = Some(idx);
                let sleepy = cap & 0x08 == 0;
                self.log(now, format!("association request from {} (capability {cap:#04x}{}): short address {short:#06x}", ieee_str(&ieee), if sleepy { ", rx off when idle" } else { "" }));
                let seq = self.next_dsn();
                let mut p = vec![0x02];
                p.extend_from_slice(&short.to_le_bytes());
                p.push(0x00);
                let rsp = mac_frame(3, true, false, seq, Some(self.cfg.pan), Addr::Ext(ieee), Some(self.cfg.pan), Addr::Ext(self.cfg.ieee), &p);
                self.queue.retain(|(_, w)| !w.starts_with("association response"));
                self.enqueue(rsp, "association response".into());
            }
            Some(0x04) => {
                self.stats.polls += 1;
                if let Some(i) = self.device_of(&m.src) { self.devices[i].polls += 1; self.devices[i].last_poll = now; }
                if pending {
                    if let Some((frame, what)) = self.queue.pop_front() {
                        let at = ack_at + ack_cycles() + LIFS;
                        self.trace(now, format!("poll from {}: {what} follows", m.src));
                        self.schedule(at, frame, Kind::Data, what);
                    }
                } else {
                    self.trace(now, format!("poll from {}: nothing pending", m.src));
                }
                // the association response is ACKed and the device joined: the network key follows
                if let Some(i) = self.device_of(&m.src) {
                    if self.devices[i].associated && !self.devices[i].keyed && !self.queue.iter().any(|(_, w)| w.starts_with("association response")) {
                        self.devices[i].keyed = true;
                        self.send_network_key(now, i);
                    }
                }
            }
            Some(0x08) => self.log(now, format!("coordinator realignment from {}?", m.src)),
            Some(c) => self.log(now, format!("MAC command {c:#04x} from {}", m.src)),
            None => {}
        }
    }

    /// A script line: `zb <command> ...`. Commands that need a joined device wait for one.
    pub fn command(&mut self, now: u64, line: &str) -> Result<(), String> {
        let rest = line.strip_prefix("zb").map(str::trim).unwrap_or(line);
        let mut it = rest.split_whitespace();
        let cmd = it.next().unwrap_or("");
        match cmd {
            "permit" => { let on = it.next().map(|v| v != "0" && v != "off").unwrap_or(true); self.cfg.permit_join = on; self.log(now, format!("permit join: {on}")); Ok(()) }
            "" => Err("zb: empty command".into()),
            _ => {
                if self.target.is_none() || !self.devices[self.target.unwrap()].keyed {
                    self.log(now, format!("`zb {rest}` waits for a device to join"));
                    self.deferred.push(rest.to_string());
                    return Ok(());
                }
                self.device_command(now, self.target.unwrap(), rest)
            }
        }
    }

    /// Commands that waited for a join: run them now that it happened.
    pub(crate) fn run_deferred(&mut self, now: u64) {
        let Some(t) = self.target else { return };
        for cmd in std::mem::take(&mut self.deferred) {
            if let Err(e) = self.device_command(now, t, &cmd) { self.log(now, format!("zb {cmd}: {e}")); }
        }
    }

    /// Check a script line without running it.
    pub fn check(line: &str) -> Result<(), String> {
        let rest = line.strip_prefix("zb").map(str::trim).unwrap_or(line);
        let cmd = rest.split_whitespace().next().unwrap_or("");
        match cmd {
            "permit" | "read" | "write" | "cie" | "bind" | "report" | "cmd" | "interview" | "leave" | "identify" => Ok(()),
            _ => Err(format!("zb: unknown command {cmd:?} (permit, interview, read, write, cie, bind, report, cmd, identify, leave)")),
        }
    }

    pub fn report(&self) -> String {
        let s = &self.stats;
        let devs: Vec<String> = self.devices.iter().map(|d| format!("{} as {:#06x}{}{}{}, {} polls", ieee_str(&d.ieee), d.short, if d.keyed { ", keyed" } else { "" }, if d.link_key_updated { ", link key exchanged" } else { "" }, if d.announced { ", announced" } else { "" }, d.polls)).collect();
        format!("[zb] coordinator PAN {:#06x} channel {}: {} beacons, {} frames in, {} out, {} acks sent, {} received, {} missed, {} polls, {} refused by the radio, {} bad MICs, {} reports, {} notifications; devices: {}; {} queued",
                self.cfg.pan, self.cfg.channel, s.beacons, s.frames_in, s.frames_out, s.acks_sent, s.acks_received, s.acks_missed, s.polls, s.not_taken, s.nwk_bad_mic, s.reports, s.notifications,
                if devs.is_empty() { "none".into() } else { devs.join("; ") }, self.queue.len())
    }
}
