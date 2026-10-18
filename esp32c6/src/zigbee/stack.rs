//! Everything above the MAC: NWK (secured data and command frames, rejoin, end-device timeout),
//! APS (data, acknowledgements, the security commands of a trust centre), ZDO (the descriptors a
//! coordinator answers for itself and the requests it makes of a device) and ZCL (attribute
//! reads, writes, reporting, IAS Zone enrolment, and decoding what the device sends back).
//!
//! Frame layouts are the Zigbee specification's (document 05-3474-22) and the ZCL's (07-5123-08);
//! byte order on the air is least significant first throughout.

use super::crypto::{ccm_decrypt, ccm_encrypt, hmac_mmo, MIC_LEN};
use super::{hex, ieee_str, Coordinator, COORD_ENDPOINT, HA_PROFILE};

const NWK_RADIUS: u8 = 30;
/// Security level 5 restored into the security-control field for CCM* (it is 0 on the air).
const LEVEL: u8 = 0x05;
const KEY_ID_LINK: u8 = 0;
pub(super) const KEY_ID_NWK: u8 = 1;
const KEY_ID_TRANSPORT: u8 = 2;
const KEY_ID_LOAD: u8 = 3;
const EXT_NONCE: u8 = 0x20;

// APS command identifiers
const APS_CMD_TRANSPORT_KEY: u8 = 0x05;
const APS_CMD_UPDATE_DEVICE: u8 = 0x06;
const APS_CMD_REQUEST_KEY: u8 = 0x08;
const APS_CMD_VERIFY_KEY: u8 = 0x0f;
const APS_CMD_CONFIRM_KEY: u8 = 0x10;
const KEY_TYPE_STANDARD_NWK: u8 = 0x01;
const KEY_TYPE_TC_LINK: u8 = 0x04;

// ZDO clusters
const ZDO_NWK_ADDR_REQ: u16 = 0x0000;
const ZDO_IEEE_ADDR_REQ: u16 = 0x0001;
const ZDO_NODE_DESC_REQ: u16 = 0x0002;
const ZDO_SIMPLE_DESC_REQ: u16 = 0x0004;
const ZDO_ACTIVE_EP_REQ: u16 = 0x0005;
const ZDO_MATCH_DESC_REQ: u16 = 0x0006;
const ZDO_DEVICE_ANNCE: u16 = 0x0013;
const ZDO_BIND_REQ: u16 = 0x0021;
const ZDO_MGMT_PERMIT_JOINING_REQ: u16 = 0x0036;
const ZDO_RSP: u16 = 0x8000;

// ZCL
const CLUSTER_BASIC: u16 = 0x0000;
const CLUSTER_IDENTIFY: u16 = 0x0003;
const CLUSTER_IAS_ZONE: u16 = 0x0500;
const ATTR_IAS_CIE_ADDRESS: u16 = 0x0010;
const ZCL_READ: u8 = 0x00;
const ZCL_READ_RSP: u8 = 0x01;
const ZCL_WRITE: u8 = 0x02;
const ZCL_WRITE_RSP: u8 = 0x04;
const ZCL_CONFIGURE_REPORTING: u8 = 0x06;
const ZCL_CONFIGURE_REPORTING_RSP: u8 = 0x07;
const ZCL_REPORT: u8 = 0x0a;
const ZCL_DEFAULT_RSP: u8 = 0x0b;
const TYPE_IEEE: u8 = 0xf0;

fn le16(b: &[u8]) -> u16 { u16::from_le_bytes([b[0], b[1]]) }
fn le32(b: &[u8]) -> u32 { u32::from_le_bytes([b[0], b[1], b[2], b[3]]) }
fn ieee_of(b: &[u8]) -> [u8; 8] { let mut a = [0u8; 8]; a.copy_from_slice(&b[..8]); a }

/// How many bytes a ZCL value of data type `t` occupies in `data`.
fn zcl_type_len(t: u8, data: &[u8]) -> Option<usize> {
    Some(match t {
        0x00 => 0,
        0x08..=0x0f => (t - 0x07) as usize,
        0x10 => 1,
        0x18..=0x1f => (t - 0x17) as usize,
        0x20..=0x27 => (t - 0x1f) as usize,
        0x28..=0x2f => (t - 0x27) as usize,
        0x30 => 1, 0x31 => 2,
        0x38 => 2, 0x39 => 4, 0x3a => 8,
        0x41 | 0x42 => { let n = *data.first()? as usize; if n == 0xff { 1 } else { 1 + n } }
        0x43 | 0x44 => { if data.len() < 2 { return None; } let n = le16(data) as usize; if n == 0xffff { 2 } else { 2 + n } }
        0xe0..=0xe2 => 4,
        0xe8 | 0xe9 => 2, 0xea => 4,
        0xf0 => 8, 0xf1 => 16,
        _ => return None,
    })
}

/// A ZCL value as text: `(text, bytes used)`.
pub(super) fn zcl_value(t: u8, data: &[u8]) -> Option<(String, usize)> {
    let n = zcl_type_len(t, data)?;
    if data.len() < n { return None; }
    let v = &data[..n];
    let text = match t {
        0x10 => match v[0] { 0 => "false".into(), 1 => "true".into(), 0xff => "invalid".into(), x => format!("{x}") },
        0x20..=0x27 | 0x30 | 0x31 | 0x08..=0x0f | 0x18..=0x1f | 0xe0..=0xe2 | 0xe8..=0xea => {
            let mut x: u64 = 0; for (i, b) in v.iter().enumerate() { x |= (*b as u64) << (8 * i); }
            if matches!(t, 0x18..=0x1f) { format!("{x:#x}") } else { format!("{x}") }
        }
        0x28..=0x2f => { let mut x: i64 = 0; for (i, b) in v.iter().enumerate() { x |= (*b as i64) << (8 * i); } let bits = 8 * n as u32; if bits < 64 && x & (1 << (bits - 1)) != 0 { x -= 1 << bits; } format!("{x}") }
        0x39 => format!("{}", f32::from_le_bytes([v[0], v[1], v[2], v[3]])),
        0x41 | 0x42 => if v[0] == 0xff { "invalid".into() } else if t == 0x42 { format!("{:?}", String::from_utf8_lossy(&v[1..])) } else { hex(&v[1..]) },
        0xf0 => ieee_str(&ieee_of(v)),
        _ => hex(v),
    };
    Some((text, n))
}

fn zcl_type_name(t: u8) -> &'static str {
    match t { 0x10 => "bool", 0x20 => "u8", 0x21 => "u16", 0x22 => "u24", 0x23 => "u32", 0x28 => "i8", 0x29 => "i16", 0x2b => "i32", 0x30 => "enum8", 0x31 => "enum16", 0x18 => "bitmap8", 0x19 => "bitmap16", 0x39 => "float", 0x41 => "octstr", 0x42 => "string", 0xe2 => "utc", 0xf0 => "ieee", 0xf1 => "key", _ => "" }
}

fn zcl_status(s: u8) -> String {
    match s { 0x00 => "success".into(), 0x01 => "failure".into(), 0x7e => "not authorized".into(), 0x80 => "malformed command".into(), 0x81 => "unsupported cluster command".into(), 0x82 => "unsupported general command".into(), 0x86 => "unsupported attribute".into(), 0x87 => "invalid value".into(), 0x88 => "read only".into(), 0x8c => "unreportable attribute".into(), 0x8d => "invalid data type".into(), 0xc3 => "unsupported cluster".into(), x => format!("status {x:#04x}") }
}

fn parse_num(s: &str) -> Result<u64, String> {
    if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) { u64::from_str_radix(h, 16) } else { s.parse() }.map_err(|_| format!("bad number {s:?}"))
}

fn type_of_name(s: &str) -> Result<u8, String> {
    Ok(match s {
        "bool" => 0x10, "u8" => 0x20, "u16" => 0x21, "u24" => 0x22, "u32" => 0x23, "i8" => 0x28, "i16" => 0x29, "i32" => 0x2b,
        "enum8" => 0x30, "enum16" => 0x31, "bitmap8" => 0x18, "bitmap16" => 0x19, "ieee" => 0xf0, "string" => 0x42,
        _ => parse_num(s).map_err(|_| format!("unknown ZCL type {s:?} (bool u8 u16 u32 i8 i16 i32 enum8 enum16 bitmap8 bitmap16 ieee string, or a type id)"))? as u8,
    })
}

/// The parsed NWK header.
#[allow(dead_code)]
pub(super) struct Nwk { pub frame_type: u8, pub security: bool, pub dst: u16, pub src: u16, pub seq: u8, pub src_ieee: Option<[u8; 8]>, pub dst_ieee: Option<[u8; 8]>, pub hdr_len: usize }

pub(super) fn parse_nwk(f: &[u8]) -> Option<Nwk> {
    if f.len() < 8 { return None; }
    let fc = le16(f);
    let mut n = Nwk { frame_type: (fc & 3) as u8, security: fc & 0x200 != 0, dst: le16(&f[2..]), src: le16(&f[4..]), seq: f[7], src_ieee: None, dst_ieee: None, hdr_len: 8 };
    let mut i = 8;
    if fc & 0x800 != 0 { n.dst_ieee = Some(ieee_of(f.get(i..i + 8)?)); i += 8; }
    if fc & 0x1000 != 0 { n.src_ieee = Some(ieee_of(f.get(i..i + 8)?)); i += 8; }
    if fc & 0x100 != 0 { i += 1; }                      // multicast control
    if fc & 0x400 != 0 { let cnt = *f.get(i)? as usize; i += 2 + 2 * cnt; }   // source route subframe
    if i > f.len() { return None; }
    n.hdr_len = i;
    Some(n)
}

/// The auxiliary security header, as read from the air.
#[allow(dead_code)]
pub(super) struct Aux { pub control: u8, pub frame_counter: u32, pub src: Option<[u8; 8]>, pub key_seq: Option<u8>, pub len: usize }

fn parse_aux(b: &[u8]) -> Option<Aux> {
    let control = *b.first()?;
    let frame_counter = le32(b.get(1..5)?);
    let mut i = 5;
    let src = if control & EXT_NONCE != 0 { let s = ieee_of(b.get(i..i + 8)?); i += 8; Some(s) } else { None };
    let key_seq = if (control >> 3) & 3 == KEY_ID_NWK { let k = *b.get(i)?; i += 1; Some(k) } else { None };
    Some(Aux { control, frame_counter, src, key_seq, len: i })
}

fn nonce(src: &[u8; 8], fc: u32, control: u8) -> [u8; 13] {
    let mut n = [0u8; 13];
    n[..8].copy_from_slice(src);
    n[8..12].copy_from_slice(&fc.to_le_bytes());
    n[12] = (control & !0x07) | LEVEL;
    n
}

/// Secure `payload` behind `header` (the NWK or APS header the aux header follows): returns
/// aux || ciphertext || MIC, with the security level 0 on the air and 5 in the computation.
pub(super) fn secure(key: &[u8; 16], key_id: u8, key_seq: Option<u8>, src: &[u8; 8], fc: u32, header: &[u8], payload: &[u8]) -> Vec<u8> {
    let control_air = (key_id << 3) | EXT_NONCE;
    let mut aux = vec![control_air];
    aux.extend_from_slice(&fc.to_le_bytes());
    aux.extend_from_slice(src);
    if let Some(k) = key_seq { aux.push(k); }
    let mut a = header.to_vec();
    a.extend_from_slice(&aux);
    a[header.len()] = control_air | LEVEL;
    let c = ccm_encrypt(key, &nonce(src, fc, control_air), &a, payload);
    let mut out = aux;
    out.extend_from_slice(&c);
    out
}

/// The inverse for a frame read from the air: `header` is everything before the aux header,
/// `rest` the aux header onwards. `default_src` when the aux header carries no address.
pub(super) fn unsecure(key: &[u8; 16], header: &[u8], rest: &[u8], default_src: Option<[u8; 8]>) -> Option<(Aux, Vec<u8>)> {
    let aux = parse_aux(rest)?;
    let src = aux.src.or(default_src)?;
    let mut a = header.to_vec();
    a.extend_from_slice(&rest[..aux.len]);
    a[header.len()] = (aux.control & !0x07) | LEVEL;
    let m = ccm_decrypt(key, &nonce(&src, aux.frame_counter, aux.control), &a, &rest[aux.len..])?;
    Some((aux, m))
}

impl Coordinator {
    // ------------------------------------------------------------------ building outgoing frames

    fn next_nwk_seq(&mut self) -> u8 { self.nwk_seq = self.nwk_seq.wrapping_add(1); self.nwk_seq }
    fn next_aps_counter(&mut self) -> u8 { self.aps_counter = self.aps_counter.wrapping_add(1); self.aps_counter }
    fn next_zcl_tsn(&mut self) -> u8 { self.zcl_tsn = self.zcl_tsn.wrapping_add(1); self.zcl_tsn }
    fn next_zdo_tsn(&mut self) -> u8 { self.zdo_tsn = self.zdo_tsn.wrapping_add(1); self.zdo_tsn }

    /// A NWK frame to `dst`: data (`frame_type` 0) or command (1), secured with the network key
    /// unless `secured` is false (only the first Transport-Key goes out in the clear).
    fn nwk_frame(&mut self, frame_type: u8, dst: u16, dst_ieee: Option<[u8; 8]>, with_src_ieee: bool, secured: bool, payload: &[u8]) -> Vec<u8> {
        let mut fc: u16 = frame_type as u16 | (2 << 2);
        if secured { fc |= 0x200; }
        if dst_ieee.is_some() { fc |= 0x800; }
        if with_src_ieee { fc |= 0x1000; }
        let seq = self.next_nwk_seq();
        let mut h = fc.to_le_bytes().to_vec();
        h.extend_from_slice(&dst.to_le_bytes());
        h.extend_from_slice(&0u16.to_le_bytes());
        h.push(NWK_RADIUS);
        h.push(seq);
        if let Some(d) = dst_ieee { h.extend_from_slice(&d); }
        if with_src_ieee { h.extend_from_slice(&self.cfg.ieee); }
        if secured {
            self.nwk_fc += 1;
            let body = secure(&self.cfg.nwk_key, KEY_ID_NWK, Some(0), &self.cfg.ieee, self.nwk_fc, &h, payload);
            h.extend_from_slice(&body);
        } else {
            h.extend_from_slice(payload);
        }
        h
    }

    /// An APS data frame header + payload (no APS security; HA traffic is NWK-secured).
    fn aps_data(&mut self, dst_ep: u8, cluster: u16, profile: u16, src_ep: u8, ack_request: bool, payload: &[u8]) -> Vec<u8> {
        let fc: u8 = if ack_request { 0x40 } else { 0 };
        let counter = self.next_aps_counter();
        let mut f = vec![fc, dst_ep];
        f.extend_from_slice(&cluster.to_le_bytes());
        f.extend_from_slice(&profile.to_le_bytes());
        f.push(src_ep);
        f.push(counter);
        f.extend_from_slice(payload);
        f
    }

    /// An APS command, APS-secured with `key` under `key_id`.
    fn aps_command_secured(&mut self, key: &[u8; 16], key_id: u8, payload: &[u8]) -> Vec<u8> {
        let counter = self.next_aps_counter();
        let h = vec![0x01 | 0x20, counter];
        self.aps_fc += 1;
        let mut f = h.clone();
        f.extend_from_slice(&secure(key, key_id, None, &self.cfg.ieee, self.aps_fc, &h, payload));
        f
    }

    /// Queue an APS data frame for device `idx`.
    fn send_aps(&mut self, idx: usize, dst_ep: u8, cluster: u16, profile: u16, payload: &[u8], what: String) {
        let short = self.devices[idx].short;
        let aps = self.aps_data(dst_ep, cluster, profile, if profile == 0 { 0 } else { COORD_ENDPOINT }, true, payload);
        let nwk = self.nwk_frame(0, short, None, false, true, &aps);
        let mac = self.data_frame(short, &nwk);
        self.enqueue(mac, what);
    }

    fn send_zdo(&mut self, idx: usize, cluster: u16, payload: &[u8], what: String) {
        let tsn = self.next_zdo_tsn();
        let mut p = vec![tsn];
        p.extend_from_slice(payload);
        self.send_aps(idx, 0, cluster, 0, &p, what);
    }

    /// A ZCL command: `specific` for a cluster-specific command, else a global one.
    #[allow(clippy::too_many_arguments)]
    fn send_zcl(&mut self, idx: usize, ep: u8, cluster: u16, specific: bool, cmd: u8, payload: &[u8], what: String) {
        let tsn = self.next_zcl_tsn();
        let mut p = vec![if specific { 0x01 } else { 0x00 }, tsn, cmd];
        p.extend_from_slice(payload);
        self.send_aps(idx, ep, cluster, HA_PROFILE, &p, format!("{what} (tsn {tsn})"));
    }

    fn send_nwk_command(&mut self, idx: usize, dst_ieee: Option<[u8; 8]>, payload: &[u8], what: String) {
        let short = self.devices[idx].short;
        let nwk = self.nwk_frame(1, short, dst_ieee, dst_ieee.is_some(), true, payload);
        let mac = self.data_frame(short, &nwk);
        self.enqueue(mac, what);
    }

    /// The network key, in a Transport-Key secured with the key-transport key derived from the
    /// device's trust-centre link key, inside an unsecured NWK frame — what a joining device can
    /// decrypt before it has anything else.
    pub(super) fn send_network_key(&mut self, now: u64, idx: usize) {
        let d = self.devices[idx].clone();
        let ktk = hmac_mmo(&d.link_key, &[0x00]);
        let mut cmd = vec![APS_CMD_TRANSPORT_KEY, KEY_TYPE_STANDARD_NWK];
        cmd.extend_from_slice(&self.cfg.nwk_key);
        cmd.push(0);                                   // key sequence number
        cmd.extend_from_slice(&d.ieee);
        cmd.extend_from_slice(&self.cfg.ieee);
        let aps = self.aps_command_secured(&ktk, KEY_ID_TRANSPORT, &cmd);
        let nwk = self.nwk_frame(0, d.short, Some(d.ieee), true, false, &aps);
        let mac = self.data_frame(d.short, &nwk);
        self.enqueue(mac, "transport key (network key)".into());
        self.log(now, format!("{} joined as {:#06x}: network key queued", ieee_str(&d.ieee), d.short));
    }

    // ------------------------------------------------------------------ incoming: NWK

    pub(super) fn on_nwk(&mut self, now: u64, m: &super::Mac, payload: &[u8]) {
        let Some(n) = parse_nwk(payload) else { self.log(now, format!("unparsable NWK frame from {}", m.src)); return; };
        let Some(idx) = self.device_of(&m.src).or_else(|| n.src_ieee.and_then(|i| self.device_of(&super::Addr::Ext(i)))) else { return; };
        let src_ieee = n.src_ieee.unwrap_or(self.devices[idx].ieee);
        let body: Vec<u8> = if n.security {
            match unsecure(&self.cfg.nwk_key, &payload[..n.hdr_len], &payload[n.hdr_len..], Some(src_ieee)) {
                Some((_aux, body)) => body,
                None => { self.stats.nwk_bad_mic += 1; self.log(now, format!("NWK frame from {:#06x} failed to decrypt (wrong network key, or a frame the device encrypted before it had ours)", n.src)); return; }
            }
        } else {
            payload[n.hdr_len..].to_vec()
        };
        match n.frame_type {
            0 => self.on_aps(now, idx, &n, &body),
            1 => self.on_nwk_command(now, idx, &n, &body),
            t => self.log(now, format!("NWK frame type {t} from {:#06x}", n.src)),
        }
    }

    fn on_nwk_command(&mut self, now: u64, idx: usize, n: &Nwk, body: &[u8]) {
        match body.first() {
            Some(0x04) => {
                let opt = body.get(1).copied().unwrap_or(0);
                self.log(now, format!("{:#06x} left the network{}", n.src, if opt & 0x20 != 0 { " to rejoin" } else { "" }));
                self.devices[idx].associated = false;
                self.devices[idx].keyed = false;
                self.devices[idx].announced = false;
            }
            Some(0x06) => {
                let cap = body.get(1).copied().unwrap_or(0);
                let ieee = n.src_ieee.unwrap_or(self.devices[idx].ieee);
                let short = self.devices[idx].short;
                self.devices[idx].associated = true;
                self.devices[idx].keyed = true;
                self.devices[idx].announced = false;
                self.target = Some(idx);
                self.log(now, format!("rejoin request from {} (capability {cap:#04x}): rejoin response, {short:#06x}", ieee_str(&ieee)));
                let mut p = vec![0x07];
                p.extend_from_slice(&short.to_le_bytes());
                p.push(0x00);
                self.send_nwk_command(idx, Some(ieee), &p, "rejoin response".into());
            }
            Some(0x0b) => {
                let req = body.get(1).copied().unwrap_or(0);
                let secs: u64 = if req == 0 { 10 } else { 60 << (req - 1).min(14) };
                self.log(now, format!("end device timeout request: {secs} s; accepted"));
                self.send_nwk_command(idx, None, &[0x0c, 0x00, 0x03], "end device timeout response".into());
            }
            Some(0x0c) => {}
            Some(0x08) => self.log(now, "link status from an end device?"),
            Some(c) => self.log(now, format!("NWK command {c:#04x} from {:#06x}: {}", n.src, hex(&body[1..]))),
            None => {}
        }
    }

    // ------------------------------------------------------------------ incoming: APS

    fn on_aps(&mut self, now: u64, idx: usize, n: &Nwk, aps: &[u8]) {
        if self.on_aps_inner(now, idx, n, aps).is_none() { self.log(now, format!("short APS frame from {:#06x}: {}", n.src, hex(aps))); }
    }

    fn on_aps_inner(&mut self, now: u64, idx: usize, n: &Nwk, aps: &[u8]) -> Option<()> {
        let fc = *aps.first()?;
        let frame_type = fc & 3;
        let delivery = (fc >> 2) & 3;
        let security = fc & 0x20 != 0;
        let ack_request = fc & 0x40 != 0;
        match frame_type {
            0 => {
                // data: [fc][dst ep][group if group delivery][cluster][profile][src ep][counter]
                let mut i = 1;
                let dst_ep = if delivery == 3 { i += 2; 0xff } else { let e = *aps.get(i)?; i += 1; e };
                let cluster = le16(aps.get(i..i + 2)?); i += 2;
                let profile = le16(aps.get(i..i + 2)?); i += 2;
                let src_ep = *aps.get(i)?; i += 1;
                let counter = *aps.get(i)?; i += 1;
                if fc & 0x80 != 0 { i += 1; }           // extended header (fragmentation): not supported
                let payload = if security {
                    let key = self.devices[idx].link_key;
                    match unsecure(&key, &aps[..i], &aps[i..], Some(self.devices[idx].ieee)) { Some((_, p)) => p, None => { self.log(now, "APS data frame failed to decrypt"); return Some(()); } }
                } else { aps.get(i..)?.to_vec() };
                if ack_request && n.dst != 0xffff && n.dst < 0xfff8 {
                    // APS acknowledgement: the addressing reflected, the same counter
                    let mut ack = vec![0x02, src_ep];
                    ack.extend_from_slice(&cluster.to_le_bytes());
                    ack.extend_from_slice(&profile.to_le_bytes());
                    ack.push(dst_ep);
                    ack.push(counter);
                    let short = self.devices[idx].short;
                    let nwk = self.nwk_frame(0, short, None, false, true, &ack);
                    let mac = self.data_frame(short, &nwk);
                    self.enqueue(mac, format!("aps ack {counter}"));
                }
                if profile == 0 { self.on_zdo(now, idx, cluster, &payload); }
                else { self.on_zcl(now, idx, src_ep, dst_ep, cluster, profile, &payload); }
            }
            1 => {
                let counter = *aps.get(1)?;
                let (key_id, payload) = if security {
                    let aux = parse_aux(aps.get(2..)?)?;
                    let key_id = (aux.control >> 3) & 3;
                    let key = match key_id { KEY_ID_LINK => self.devices[idx].link_key, KEY_ID_TRANSPORT => hmac_mmo(&self.devices[idx].link_key, &[0x00]), KEY_ID_LOAD => hmac_mmo(&self.devices[idx].link_key, &[0x02]), _ => self.cfg.nwk_key };
                    match unsecure(&key, &aps[..2], &aps[2..], Some(self.devices[idx].ieee)) { Some((_, p)) => (Some(key_id), p), None => { self.log(now, format!("APS command (counter {counter}) failed to decrypt with key id {key_id}")); return Some(()); } }
                } else { (None, aps.get(2..)?.to_vec()) };
                if ack_request {
                    let short = self.devices[idx].short;
                    let nwk = self.nwk_frame(0, short, None, false, true, &[0x12, counter]);
                    let mac = self.data_frame(short, &nwk);
                    self.enqueue(mac, format!("aps ack {counter}"));
                }
                self.on_aps_command(now, idx, key_id, &payload);
            }
            2 => { let counter = if fc & 0x10 != 0 { aps.get(1) } else { aps.get(7) }; self.trace(now, format!("aps ack {}", counter.map(|c| c.to_string()).unwrap_or_default())); }
            _ => {}
        }
        Some(())
    }

    fn on_aps_command(&mut self, now: u64, idx: usize, key_id: Option<u8>, p: &[u8]) {
        match p.first() {
            Some(&APS_CMD_REQUEST_KEY) => {
                let kt = p.get(1).copied().unwrap_or(0);
                if kt != KEY_TYPE_TC_LINK { self.log(now, format!("request key of type {kt:#04x}: only the trust-centre link key is handed out")); return; }
                if key_id != Some(KEY_ID_LINK) { self.log(now, format!("request key not secured with the link key (key id {key_id:?})")); }
                // a fresh link key for this device, derived so a run is reproducible
                let d = self.devices[idx].clone();
                let new_key = hmac_mmo(&self.cfg.tc_link_key, &d.ieee);
                let load_key = hmac_mmo(&d.link_key, &[0x02]);
                let mut cmd = vec![APS_CMD_TRANSPORT_KEY, KEY_TYPE_TC_LINK];
                cmd.extend_from_slice(&new_key);
                cmd.extend_from_slice(&d.ieee);
                cmd.extend_from_slice(&self.cfg.ieee);
                let aps = self.aps_command_secured(&load_key, KEY_ID_LOAD, &cmd);
                let nwk = self.nwk_frame(0, d.short, None, false, true, &aps);
                let mac = self.data_frame(d.short, &nwk);
                self.enqueue(mac, "transport key (trust-centre link key)".into());
                self.devices[idx].link_key = new_key;      // Verify-Key will come under the new one
                self.log(now, format!("{:#06x} requested a trust-centre link key: a unique one queued under the key-load key", d.short));
            }
            Some(&APS_CMD_VERIFY_KEY) => {
                let kt = p.get(1).copied().unwrap_or(0);
                let d = self.devices[idx].clone();
                let hash = p.get(10..26);
                let expect = hmac_mmo(&d.link_key, &[0x03]);
                let ok = kt == KEY_TYPE_TC_LINK && hash == Some(&expect[..]);
                let mut cmd = vec![APS_CMD_CONFIRM_KEY, if ok { 0x00 } else { 0xad }, KEY_TYPE_TC_LINK];
                cmd.extend_from_slice(&d.ieee);
                let aps = self.aps_command_secured(&d.link_key, KEY_ID_LINK, &cmd);
                let nwk = self.nwk_frame(0, d.short, None, false, true, &aps);
                let mac = self.data_frame(d.short, &nwk);
                self.enqueue(mac, "confirm key".into());
                if ok { self.devices[idx].link_key_updated = true; }
                self.log(now, format!("verify key from {:#06x}: {}; confirm key queued", d.short, if ok { "hash matches the new link key" } else { "HASH MISMATCH" }));
            }
            Some(&APS_CMD_UPDATE_DEVICE) => self.log(now, "update device from an end device?"),
            Some(c) => self.log(now, format!("APS command {c:#04x}: {}", hex(&p[1..]))),
            None => {}
        }
    }

    // ------------------------------------------------------------------ ZDO

    fn on_zdo(&mut self, now: u64, idx: usize, cluster: u16, p: &[u8]) {
        let Some(&tsn) = p.first() else { return };
        let b = &p[1..];
        let short = self.devices[idx].short;
        match cluster {
            ZDO_DEVICE_ANNCE => {
                if b.len() >= 11 {
                    let (nwk, ieee, cap) = (le16(b), ieee_of(&b[2..]), b[10]);
                    self.log(now, format!("device announce: {} is {nwk:#06x} (capability {cap:#04x})", ieee_str(&ieee)));
                    if nwk != short { self.devices[idx].short = nwk; }
                }
                self.devices[idx].announced = true;
                self.run_deferred(now);
            }
            ZDO_NODE_DESC_REQ => {
                let who = b.get(..2).map(le16).unwrap_or(0);
                if who == 0x0000 {
                    // our node descriptor: coordinator, 2.4 GHz, FFD mains-powered rx-on, buffer 82,
                    // transfer 128, primary trust centre with stack compliance revision 22
                    let r = vec![tsn, 0x00, 0x00, 0x00, 0x00, 0x40, 0x8e, 0x37, 0x10, 0x52, 0x80, 0x00, 0x01, 0x2c, 0x80, 0x00, 0x00];
                    self.log(now, "node descriptor request for us: coordinator, revision 22");
                    self.send_aps(idx, 0, ZDO_NODE_DESC_REQ | ZDO_RSP, 0, &r, "node descriptor response".into());
                } else {
                    self.log(now, format!("node descriptor request for {who:#06x}: not us"));
                }
            }
            ZDO_ACTIVE_EP_REQ => {
                let r = vec![tsn, 0x00, 0x00, 0x00, 1, COORD_ENDPOINT];
                self.send_aps(idx, 0, ZDO_ACTIVE_EP_REQ | ZDO_RSP, 0, &r, "active endpoints response".into());
            }
            ZDO_SIMPLE_DESC_REQ => {
                let mut r = vec![tsn, 0x00, 0x00, 0x00];
                let desc = [COORD_ENDPOINT, 0x04, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00];   // ep 1, HA, device 0x0005, no clusters
                r.push(desc.len() as u8);
                r.extend_from_slice(&desc);
                self.send_aps(idx, 0, ZDO_SIMPLE_DESC_REQ | ZDO_RSP, 0, &r, "simple descriptor response".into());
            }
            ZDO_NWK_ADDR_REQ => {
                let ieee = b.get(..8).map(ieee_of);
                let mut r = vec![tsn, if ieee == Some(self.cfg.ieee) { 0x00 } else { 0x81 }];
                r.extend_from_slice(&self.cfg.ieee);
                r.extend_from_slice(&[0x00, 0x00]);
                self.send_aps(idx, 0, ZDO_NWK_ADDR_REQ | ZDO_RSP, 0, &r, "network address response".into());
            }
            ZDO_IEEE_ADDR_REQ => {
                let who = b.get(..2).map(le16).unwrap_or(0);
                let mut r = vec![tsn, if who == 0 { 0x00 } else { 0x81 }];
                r.extend_from_slice(&self.cfg.ieee);
                r.extend_from_slice(&[0x00, 0x00]);
                self.send_aps(idx, 0, ZDO_IEEE_ADDR_REQ | ZDO_RSP, 0, &r, "IEEE address response".into());
            }
            ZDO_MATCH_DESC_REQ => self.trace(now, "match descriptor request: no match offered"),
            ZDO_MGMT_PERMIT_JOINING_REQ => {
                let secs = b.first().copied().unwrap_or(0);
                self.log(now, format!("permit joining request from {short:#06x}: {secs} s; accepted (joining stays {})", if self.cfg.permit_join { "open" } else { "closed" }));
                self.send_aps(idx, 0, ZDO_MGMT_PERMIT_JOINING_REQ | ZDO_RSP, 0, &[tsn, 0x00], "permit joining response".into());
            }
            c if c & ZDO_RSP != 0 => self.on_zdo_response(now, idx, c & !ZDO_RSP, b),
            c => self.log(now, format!("ZDO request {c:#06x} from {short:#06x}: {}", hex(b))),
        }
    }

    fn on_zdo_response(&mut self, now: u64, idx: usize, req: u16, b: &[u8]) {
        let status = b.first().copied().unwrap_or(0xff);
        match req {
            ZDO_NODE_DESC_REQ if status == 0 && b.len() >= 16 => {
                let d = &b[3..16];
                let logical = ["coordinator", "router", "end device"].get((d[0] & 7) as usize).copied().unwrap_or("?");
                let rev = le16(&d[8..10]) >> 9;
                self.log(now, format!("node descriptor: {logical}, mac capability {:#04x}{}, manufacturer {:#06x}, stack revision {rev}", d[2], if d[2] & 0x08 == 0 { " (rx off when idle)" } else { "" }, le16(&d[3..5])));
            }
            ZDO_ACTIVE_EP_REQ if status == 0 && b.len() >= 4 => {
                let n = b[3] as usize;
                let eps: Vec<u8> = b.get(4..4 + n).map(|s| s.to_vec()).unwrap_or_default();
                self.log(now, format!("active endpoints: {:?}", eps));
                self.devices[idx].endpoints = eps.clone();
                for ep in &eps {
                    let short = self.devices[idx].short;
                    let mut p = short.to_le_bytes().to_vec(); p.push(*ep);
                    self.send_zdo(idx, ZDO_SIMPLE_DESC_REQ, &p, format!("simple descriptor request ep {ep}"));
                }
                if let Some(&ep) = eps.first() {
                    let attrs = [0x0004u16, 0x0005, 0x0007, 0x4000];
                    let mut p = Vec::new(); for a in attrs { p.extend_from_slice(&a.to_le_bytes()); }
                    self.send_zcl(idx, ep, CLUSTER_BASIC, false, ZCL_READ, &p, format!("read basic attributes ep {ep}"));
                }
            }
            ZDO_SIMPLE_DESC_REQ if status == 0 && b.len() >= 12 => {
                let d = &b[4..];
                let (ep, profile, device, version) = (d[0], le16(&d[1..]), le16(&d[3..]), d[5] & 0x0f);
                let nin = d[6] as usize;
                let ins: Vec<String> = (0..nin).filter_map(|i| d.get(7 + 2 * i..9 + 2 * i).map(|c| format!("{:#06x}", le16(c)))).collect();
                let o = 7 + 2 * nin;
                let nout = d.get(o).copied().unwrap_or(0) as usize;
                let outs: Vec<String> = (0..nout).filter_map(|i| d.get(o + 1 + 2 * i..o + 3 + 2 * i).map(|c| format!("{:#06x}", le16(c)))).collect();
                self.log(now, format!("endpoint {ep}: profile {profile:#06x} device {device:#06x} v{version}; in [{}] out [{}]", ins.join(", "), outs.join(", ")));
            }
            ZDO_BIND_REQ => self.log(now, format!("bind response: {}", if status == 0 { "success".into() } else { format!("status {status:#04x}") })),
            r => self.log(now, format!("ZDO response {:#06x}: status {status:#04x} {}", r | ZDO_RSP, hex(b.get(1..).unwrap_or(&[])))),
        }
    }

    // ------------------------------------------------------------------ ZCL

    #[allow(clippy::too_many_arguments)]
    fn on_zcl(&mut self, now: u64, idx: usize, src_ep: u8, dst_ep: u8, cluster: u16, profile: u16, p: &[u8]) {
        let Some(&fc) = p.first() else { return };
        let specific = fc & 3 == 1;
        let manuf = fc & 4 != 0;
        let to_client = fc & 8 != 0;
        let no_default = fc & 0x10 != 0;
        let mut i = 1;
        let manuf_code = if manuf { let c = p.get(i..i + 2).map(le16); i += 2; c } else { None };
        let (Some(&tsn), Some(&cmd)) = (p.get(i), p.get(i + 1)) else { return };
        let b = p.get(i + 2..).unwrap_or(&[]);
        let where_ = format!("ep {src_ep} cluster {cluster:#06x}");
        let mut default_status: Option<u8> = if !no_default && !(cmd == ZCL_DEFAULT_RSP && !specific) { Some(0x00) } else { None };
        if !specific {
            match cmd {
                ZCL_READ_RSP => {
                    let mut j = 0; let mut items = Vec::new();
                    while j + 3 <= b.len() {
                        let (attr, status) = (le16(&b[j..]), b[j + 2]); j += 3;
                        if status != 0 { items.push(format!("{attr:#06x}: {}", zcl_status(status))); continue; }
                        let Some(&t) = b.get(j) else { break }; j += 1;
                        match zcl_value(t, &b[j..]) { Some((v, n)) => { items.push(format!("{attr:#06x} = {v} ({})", zcl_type_name(t))); j += n; } None => { items.push(format!("{attr:#06x}: type {t:#04x} {}", hex(&b[j..]))); break; } }
                    }
                    self.log(now, format!("read response {where_}: {}", items.join(", ")));
                    default_status = None;
                }
                ZCL_WRITE_RSP => {
                    let txt = if b.len() == 1 { zcl_status(b[0]) } else { b.chunks(3).map(|c| format!("{:#06x}: {}", le16(&c[1..]), zcl_status(c[0]))).collect::<Vec<_>>().join(", ") };
                    self.log(now, format!("write response {where_}: {txt}"));
                    default_status = None;
                }
                ZCL_CONFIGURE_REPORTING_RSP => {
                    let txt = if b.len() == 1 { zcl_status(b[0]) } else { b.chunks(4).map(|c| format!("{:#06x}: {}", le16(&c[2..]), zcl_status(c[0]))).collect::<Vec<_>>().join(", ") };
                    self.log(now, format!("configure reporting response {where_}: {txt}"));
                    default_status = None;
                }
                ZCL_REPORT => {
                    self.stats.reports += 1;
                    let mut j = 0; let mut items = Vec::new();
                    while j + 3 <= b.len() {
                        let (attr, t) = (le16(&b[j..]), b[j + 2]); j += 3;
                        match zcl_value(t, &b[j..]) { Some((v, n)) => { items.push(format!("{attr:#06x} = {v} ({})", zcl_type_name(t))); j += n; } None => { items.push(format!("{attr:#06x}: type {t:#04x} {}", hex(&b[j..]))); break; } }
                    }
                    self.log(now, format!("report {where_}: {}", items.join(", ")));
                }
                ZCL_DEFAULT_RSP => {
                    let (c, s) = (b.first().copied().unwrap_or(0), b.get(1).copied().unwrap_or(0));
                    self.log(now, format!("default response {where_} to command {c:#04x} (tsn {tsn}): {}", zcl_status(s)));
                }
                c => self.log(now, format!("ZCL global command {c:#04x} {where_}: {}", hex(b))),
            }
        } else if cluster == CLUSTER_IAS_ZONE && to_client {
            match cmd {
                0x00 if b.len() >= 6 => {
                    self.stats.notifications += 1;
                    let (status, ext, zone, delay) = (le16(b), b[2], b[3], le16(&b[4..]));
                    let mut flags = Vec::new();
                    for (bit, name) in [(0, "ALARM1"), (1, "ALARM2"), (2, "TAMPER"), (3, "BATTERY"), (4, "SUPERVISION"), (5, "RESTORE"), (6, "TROUBLE"), (7, "AC_MAINS"), (8, "TEST"), (9, "BATTERY_DEFECT")] {
                        if status & (1 << bit) != 0 { flags.push(name); }
                    }
                    self.log(now, format!("IAS zone status change from ep {src_ep}: {status:#06x} [{}] zone {zone} ext {ext:#04x} delay {delay}", flags.join(" ")));
                }
                0x01 => {
                    let (zt, mc) = (b.get(..2).map(le16).unwrap_or(0), b.get(2..4).map(le16).unwrap_or(0));
                    let zone = match self.devices[idx].zones.iter().find(|(e, _)| *e == src_ep) { Some((_, z)) => *z, None => { let z = self.next_zone; self.next_zone += 1; self.devices[idx].zones.push((src_ep, z)); z } };
                    self.log(now, format!("IAS zone enroll request from ep {src_ep}: zone type {zt:#06x} manufacturer {mc:#06x}; enrolled as zone {zone}"));
                    self.send_zcl(idx, src_ep, CLUSTER_IAS_ZONE, true, 0x00, &[0x00, zone], format!("zone enroll response ep {src_ep}"));
                    default_status = None;
                }
                c => self.log(now, format!("IAS zone command {c:#04x} from ep {src_ep}: {}", hex(b))),
            }
        } else {
            self.log(now, format!("ZCL cluster command {cmd:#04x} {where_}{}{}: {}", if to_client { " (server to client)" } else { "" }, manuf_code.map(|m| format!(" manufacturer {m:#06x}")).unwrap_or_default(), hex(b)));
        }
        if let Some(status) = default_status {
            // a Default Response, the direction flipped, the same transaction number
            let mut f = vec![0x10 | if to_client { 0 } else { 0x08 }];
            if let Some(m) = manuf_code { f[0] |= 0x04; f.extend_from_slice(&m.to_le_bytes()); }
            f.extend_from_slice(&[tsn, ZCL_DEFAULT_RSP, cmd, status]);
            self.send_aps(idx, src_ep, cluster, profile, &f, format!("default response ep {src_ep} tsn {tsn}"));
            let _ = dst_ep;
        }
    }

    // ------------------------------------------------------------------ script commands

    pub(super) fn device_command(&mut self, now: u64, idx: usize, rest: &str) -> Result<(), String> {
        let args: Vec<&str> = rest.split_whitespace().collect();
        let arg = |i: usize| -> Result<&str, String> { args.get(i).copied().ok_or_else(|| format!("zb {}: missing argument {i}", args.first().unwrap_or(&""))) };
        let num = |i: usize| -> Result<u64, String> { parse_num(arg(i)?) };
        let short = self.devices[idx].short;
        match args.first().copied().unwrap_or("") {
            "interview" => {
                let p = short.to_le_bytes();
                self.send_zdo(idx, ZDO_NODE_DESC_REQ, &p, "node descriptor request".into());
                self.send_zdo(idx, ZDO_ACTIVE_EP_REQ, &p, "active endpoints request".into());
                self.log(now, format!("interviewing {short:#06x}"));
            }
            "read" => {
                let (ep, cluster) = (num(1)? as u8, num(2)? as u16);
                let mut p = Vec::new();
                for a in arg(3)?.split(',') { p.extend_from_slice(&(parse_num(a)? as u16).to_le_bytes()); }
                self.send_zcl(idx, ep, cluster, false, ZCL_READ, &p, format!("read ep {ep} cluster {cluster:#06x} {}", arg(3)?));
            }
            "write" => {
                let (ep, cluster, attr) = (num(1)? as u8, num(2)? as u16, num(3)? as u16);
                let t = type_of_name(arg(4)?)?;
                let mut p = attr.to_le_bytes().to_vec();
                p.push(t);
                let val = arg(5)?;
                match t {
                    TYPE_IEEE => { if val == "me" || val == "coordinator" { p.extend_from_slice(&self.cfg.ieee); } else { let mut b: [u8; 8] = super::parse_hex_bytes(val, "ieee")?; b.reverse(); p.extend_from_slice(&b); } }
                    0x42 => { p.push(val.len() as u8); p.extend_from_slice(val.as_bytes()); }
                    0x10 => p.push(match val { "1" | "true" | "on" => 1, "0" | "false" | "off" => 0, v => return Err(format!("bool: {v:?}")) }),
                    _ => { let n = zcl_type_len(t, &[0; 8]).ok_or_else(|| format!("type {t:#04x}: unknown size"))?; let v = parse_num(val)?; p.extend_from_slice(&v.to_le_bytes()[..n]); }
                }
                self.send_zcl(idx, ep, cluster, false, ZCL_WRITE, &p, format!("write ep {ep} cluster {cluster:#06x} attr {attr:#06x} = {val}"));
            }
            "cie" => {
                let ep = num(1)? as u8;
                let mut p = ATTR_IAS_CIE_ADDRESS.to_le_bytes().to_vec();
                p.push(TYPE_IEEE);
                p.extend_from_slice(&self.cfg.ieee);
                self.send_zcl(idx, ep, CLUSTER_IAS_ZONE, false, ZCL_WRITE, &p, format!("write CIE address ep {ep}"));
                self.log(now, format!("writing our IEEE address {} as CIE of ep {ep}", ieee_str(&self.cfg.ieee)));
            }
            "bind" => {
                let (ep, cluster) = (num(1)? as u8, num(2)? as u16);
                let mut p = self.devices[idx].ieee.to_vec();
                p.push(ep);
                p.extend_from_slice(&cluster.to_le_bytes());
                p.push(0x03);
                p.extend_from_slice(&self.cfg.ieee);
                p.push(COORD_ENDPOINT);
                self.send_zdo(idx, ZDO_BIND_REQ, &p, format!("bind ep {ep} cluster {cluster:#06x} to us"));
            }
            "report" => {
                let (ep, cluster, attr) = (num(1)? as u8, num(2)? as u16, num(3)? as u16);
                let t = type_of_name(arg(4)?)?;
                let (min, max) = (num(5)? as u16, num(6)? as u16);
                let mut p = vec![0x00];
                p.extend_from_slice(&attr.to_le_bytes());
                p.push(t);
                p.extend_from_slice(&min.to_le_bytes());
                p.extend_from_slice(&max.to_le_bytes());
                let analog = matches!(t, 0x20..=0x2f | 0x38..=0x3a | 0xe0..=0xe2);
                if analog { let n = zcl_type_len(t, &[0; 8]).unwrap_or(1); let ch = args.get(7).map(|v| parse_num(v)).transpose()?.unwrap_or(1); p.extend_from_slice(&ch.to_le_bytes()[..n]); }
                self.send_zcl(idx, ep, cluster, false, ZCL_CONFIGURE_REPORTING, &p, format!("configure reporting ep {ep} cluster {cluster:#06x} attr {attr:#06x} {min}..{max} s"));
            }
            "cmd" => {
                let (ep, cluster, cmd) = (num(1)? as u8, num(2)? as u16, num(3)? as u8);
                let payload: Vec<u8> = match args.get(4) { Some(h) => { let clean: String = h.chars().filter(|c| c.is_ascii_hexdigit()).collect(); (0..clean.len() / 2).map(|i| u8::from_str_radix(&clean[2 * i..2 * i + 2], 16).unwrap()).collect() } None => Vec::new() };
                self.send_zcl(idx, ep, cluster, true, cmd, &payload, format!("command {cmd:#04x} ep {ep} cluster {cluster:#06x}"));
            }
            "identify" => {
                let (ep, secs) = (num(1)? as u8, args.get(2).map(|v| parse_num(v)).transpose()?.unwrap_or(5) as u16);
                self.send_zcl(idx, ep, CLUSTER_IDENTIFY, true, 0x00, &secs.to_le_bytes(), format!("identify ep {ep} for {secs} s"));
            }
            "leave" => {
                let rejoin = args.get(1).is_some_and(|v| *v == "rejoin");
                let opt = 0x40 | if rejoin { 0x20 } else { 0 };
                let ieee = self.devices[idx].ieee;
                self.send_nwk_command(idx, Some(ieee), &[0x04, opt], format!("leave{}", if rejoin { " (rejoin)" } else { "" }));
                self.log(now, format!("asking {short:#06x} to leave{}", if rejoin { " and rejoin" } else { "" }));
            }
            c => return Err(format!("zb: unknown command {c:?}")),
        }
        let _ = MIC_LEN;
        Ok(())
    }
}
