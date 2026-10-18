//! The coordinator against hand-built frames: what a ZBOSS end device sends, byte for byte as
//! the bring-up captured it, and what must come back.

use super::crypto::{ccm_decrypt, hmac_mmo};
use super::stack::{parse_nwk, secure, unsecure, zcl_value, KEY_ID_NWK};
use super::*;

/// The alarm's IEEE address (dc:1e:d5:ff:fe:6e:8c:dc), on-air order.
const DEV: [u8; 8] = [0xdc, 0x8c, 0x6e, 0xfe, 0xff, 0xd5, 0x1e, 0xdc];
const MS: u64 = CPU_HZ / 1000;

fn coordinator() -> Coordinator { let mut c = Coordinator::new(Config::default()); c.print = false; c }
fn beacon_request() -> Vec<u8> { vec![0x03, 0x08, 0x01, 0xff, 0xff, 0xff, 0xff, 0x07] }
fn association_request(seq: u8) -> Vec<u8> { mac_frame(3, true, false, seq, Some(DEFAULT_PAN), Addr::Short(0), Some(0xffff), Addr::Ext(DEV), &[0x01, 0x80]) }
fn data_request(seq: u8, src: Addr) -> Vec<u8> { mac_frame(3, true, false, seq, Some(DEFAULT_PAN), Addr::Short(0), Some(DEFAULT_PAN), src, &[0x04]) }
fn ack(seq: u8) -> Vec<u8> { vec![0x02, 0x00, seq] }

/// Put every due frame on a radio that takes everything; return them in order.
fn air(c: &mut Coordinator, now: u64) -> Vec<Pending> {
    let due = c.take_due(now);
    for p in &due { c.outcome(now, p.clone(), true); }
    due
}

/// Join from the beacon request to the network key: returns the frame carrying the key.
fn join(c: &mut Coordinator) -> Vec<u8> {
    let mut t = 1000 * MS;
    c.on_tx(t, 11, &beacon_request());
    let due = air(c, u64::MAX);
    assert_eq!(due.len(), 1);
    assert_eq!(due[0].kind, Kind::Beacon);
    t += 100 * MS;
    c.on_tx(t, 11, &association_request(5));
    let due = air(c, u64::MAX);
    assert_eq!(due.len(), 1, "the acknowledgement only; the response waits for a poll");
    assert_eq!(due[0].frame, ack(5));
    t += 500 * MS;
    c.on_tx(t, 11, &data_request(6, Addr::Ext(DEV)));
    let due = air(c, u64::MAX);
    assert_eq!(due.len(), 2);
    assert_eq!(due[0].frame, vec![0x12, 0x00, 6], "pending bit set: the association response follows");
    let rsp = parse_mac(&due[1].frame).unwrap();
    assert_eq!((rsp.frame_type, rsp.dst, rsp.src), (3, Addr::Ext(DEV), Addr::Ext(DEFAULT_IEEE)));
    assert_eq!(&due[1].frame[rsp.hdr_len..], &[0x02, 0x34, 0x12, 0x00], "short address 0x1234, success");
    c.on_tx(t + 2 * MS, 11, &ack(due[1].frame[2]));
    t += 600 * MS;
    c.on_tx(t, 11, &data_request(7, Addr::Short(0x1234)));
    let due = air(c, u64::MAX);
    assert_eq!(due.len(), 2);
    assert_eq!(due[1].what, "transport key (network key)");
    c.on_tx(t + 5 * MS, 11, &ack(due[1].frame[2]));
    due[1].frame.clone()
}

#[test]
fn beacon_answers_the_request_on_its_channel_only() {
    let mut c = coordinator();
    c.on_tx(1000 * MS, 15, &beacon_request());
    assert!(c.take_due(u64::MAX).is_empty());
    c.on_tx(1000 * MS, 11, &beacon_request());
    let due = air(&mut c, u64::MAX);
    let b = &due[0].frame;
    assert_eq!(&b[..2], &[0x00, 0x80], "beacon, short source address, no destination");
    assert_eq!(&b[3..7], &[0x62, 0x1a, 0x00, 0x00], "PAN 0x1a62, coordinator 0x0000");
    assert_eq!(&b[7..9], &[0xff, 0xcf], "beaconless, PAN coordinator, association permitted");
    assert_eq!(&b[11..14], &[0x00, 0x22, 0x84], "Zigbee PRO, protocol version 2, router and end device capacity");
    assert_eq!(&b[14..22], &DEFAULT_EXT_PAN);
    assert_eq!(c.stats.beacons, 1);
}

#[test]
fn the_network_key_travels_under_the_key_transport_key() {
    let mut c = coordinator();
    let frame = join(&mut c);
    let m = parse_mac(&frame).unwrap();
    assert_eq!((m.frame_type, m.ack_request, m.dst), (1, true, Addr::Short(0x1234)));
    let nwk = &frame[m.hdr_len..];
    let n = parse_nwk(nwk).unwrap();
    assert!(!n.security, "the first Transport-Key is not network-secured: the device has no key yet");
    assert_eq!((n.dst_ieee, n.src_ieee), (Some(DEV), Some(DEFAULT_IEEE)));
    let aps = &nwk[n.hdr_len..];
    assert_eq!(aps[0], 0x21, "APS command, APS security");
    let ktk = hmac_mmo(&DEFAULT_TC_LINK_KEY, &[0x00]);
    let (aux, plain) = unsecure(&ktk, &aps[..2], &aps[2..], None).expect("decrypts with the key-transport key");
    assert_eq!(aux.control >> 3 & 3, 2, "key id: key-transport key");
    assert_eq!(plain[0], 0x05);
    assert_eq!(plain[1], 0x01, "standard network key");
    assert_eq!(&plain[2..18], &DEFAULT_NWK_KEY);
    assert_eq!(&plain[19..27], &DEV);
    assert_eq!(&plain[27..35], &DEFAULT_IEEE);
    let bad = [0u8; 16];
    assert!(ccm_decrypt(&bad, &[0; 13], &[], &aps[15..]).is_none());
    assert!(c.devices[0].keyed);
}

/// A NWK-secured frame from the device: a device announce, as ZBOSS sends it after the key.
fn device_frame(fc: u32, dst: u16, aps: &[u8]) -> Vec<u8> {
    let mut h = vec![0x08, 0x12 | 0x02];     // data, version 2, security, source IEEE present
    h.extend_from_slice(&dst.to_le_bytes());
    h.extend_from_slice(&0x1234u16.to_le_bytes());
    h.push(30); h.push(0x42);
    h.extend_from_slice(&DEV);
    let body = secure(&DEFAULT_NWK_KEY, KEY_ID_NWK, Some(0), &DEV, fc, &h, aps);
    h.extend_from_slice(&body);
    mac_frame(1, true, false, 9, Some(DEFAULT_PAN), Addr::Short(dst), Some(DEFAULT_PAN), Addr::Short(0x1234), &h)
}

#[test]
fn a_secured_device_announce_is_read_and_a_write_goes_out_secured() {
    let mut c = coordinator();
    join(&mut c);
    // device announce: ZDO cluster 0x0013, broadcast to rx-on devices
    let mut zdo = vec![0x08, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x11, 0x34, 0x12];
    zdo.extend_from_slice(&DEV); zdo.push(0x80);
    c.on_tx(3000 * MS, 11, &device_frame(1, 0xfffd, &zdo));
    assert!(c.devices[0].announced);
    assert_eq!(c.stats.nwk_bad_mic, 0);
    assert_eq!(air(&mut c, u64::MAX).len(), 1, "the acknowledgement");
    // a tampered one does not decrypt
    let mut bad = device_frame(2, 0xfffd, &zdo);
    let l = bad.len(); bad[l - 3] ^= 1;
    c.on_tx(3100 * MS, 11, &bad);
    assert_eq!(c.stats.nwk_bad_mic, 1);
    air(&mut c, u64::MAX);
    // a script write, delivered on the next poll, decrypts with the network key to the ZCL frame
    c.command(3200 * MS, "zb write 11 0x0010 0x0055 bool 1").unwrap();
    c.on_tx(3300 * MS, 11, &data_request(10, Addr::Short(0x1234)));
    let due = air(&mut c, u64::MAX);
    let frame = &due[1].frame;
    let m = parse_mac(frame).unwrap();
    let nwk = &frame[m.hdr_len..];
    let n = parse_nwk(nwk).unwrap();
    assert!(n.security);
    let (_, aps) = unsecure(&DEFAULT_NWK_KEY, &nwk[..n.hdr_len], &nwk[n.hdr_len..], Some(DEFAULT_IEEE)).expect("network key");
    assert_eq!(&aps[..7], &[0x40, 11, 0x10, 0x00, 0x04, 0x01, COORD_ENDPOINT], "APS data with ack request to endpoint 11, Binary Output, HA, from endpoint 1");
    let zcl = &aps[8..];
    assert_eq!((zcl[0], zcl[2]), (0x00, 0x02), "ZCL global frame, Write Attributes");
    assert_eq!(&zcl[3..], &[0x55, 0x00, 0x10, 0x01], "attribute 0x0055, bool, true");
}

#[test]
fn an_unacknowledged_frame_is_held_for_the_next_poll() {
    let mut c = coordinator();
    join(&mut c);
    c.command(0, "zb identify 10 1").unwrap();
    let t = 5000 * MS;
    c.on_tx(t, 11, &data_request(20, Addr::Short(0x1234)));
    let due = air(&mut c, u64::MAX);
    assert_eq!(due.len(), 2);
    assert!(c.queue.is_empty());
    // no acknowledgement arrives: after the deadline the frame is queued again, counting the miss
    assert!(c.take_due(t + 100 * MS).is_empty());
    assert_eq!(c.queue.len(), 1);
    assert_eq!((c.stats.acks_missed, c.queue[0].misses), (1, 1));
    // an acknowledgement with another sequence number does not count, and does not lose the frame
    let t = t + 200 * MS;
    c.on_tx(t, 11, &data_request(21, Addr::Short(0x1234)));
    let due = air(&mut c, u64::MAX);
    let seq = due[1].frame[2];
    c.on_tx(t + 3 * MS, 11, &ack(seq.wrapping_add(7)));
    assert!(c.queue.is_empty());
    assert!(c.take_due(t + 100 * MS).is_empty());
    assert_eq!((c.stats.acks_missed, c.queue.len(), c.queue[0].misses), (2, 1, 2));
    // the third miss drops it, and nothing behind it is held up
    c.command(0, "zb identify 10 2").unwrap();
    let t = t + 400 * MS;
    c.on_tx(t, 11, &data_request(22, Addr::Short(0x1234)));
    air(&mut c, u64::MAX);
    assert!(c.take_due(t + 100 * MS).is_empty());
    assert_eq!(c.queue.len(), 1, "only the second identify remains");
    assert_eq!(c.queue[0].misses, 0);
    assert_eq!(c.stats.acks_missed, 3);
}

/// Every prefix of every frame the device can send, and a bit flipped in each: nothing panics.
#[test]
fn hostile_frames_do_not_panic() {
    let mut c = coordinator();
    join(&mut c);
    let mut zdo = vec![0x08, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x11, 0x34, 0x12];
    zdo.extend_from_slice(&DEV); zdo.push(0x80);
    // APS data with the extended-header and security bits, a short write response, a short
    // configure-reporting response, a report cut mid-value, an APS command with no payload
    let frames: Vec<Vec<u8>> = vec![
        device_frame(1, 0xfffd, &zdo),
        device_frame(2, 0x0000, &[0xa0, 1, 0x10, 0x00, 0x04, 0x01, 11, 0x33]),
        device_frame(3, 0x0000, &[0x40, 1, 0x10, 0x00, 0x04, 0x01, 11, 0x34, 0x08, 0x05, 0x04, 0x86, 0x55]),
        device_frame(4, 0x0000, &[0x40, 1, 0x01, 0x00, 0x04, 0x01, 10, 0x35, 0x08, 0x06, 0x07, 0x8c, 0x00]),
        device_frame(5, 0x0000, &[0x40, 1, 0x01, 0x00, 0x04, 0x01, 10, 0x36, 0x08, 0x07, 0x0a, 0x21, 0x00, 0x21, 0x34]),
        device_frame(6, 0x0000, &[0x21, 0x40]),
        device_frame(7, 0x0000, &[0x01, 0x41]),
        association_request(3), data_request(4, Addr::Short(0x1234)), beacon_request(), ack(1),
    ];
    let mut t = 4000 * MS;
    for f in &frames {
        for n in 0..=f.len() {
            c.on_tx(t, 11, &f[..n]);
            t += MS;
            air(&mut c, u64::MAX);
        }
        for i in 0..f.len() {
            let mut g = f.clone(); g[i] ^= 0x10;
            c.on_tx(t, 11, &g);
            t += MS;
            air(&mut c, u64::MAX);
        }
    }
}

#[test]
fn script_commands_wait_for_a_device() {
    let mut c = coordinator();
    c.command(0, "zb read 10 0x0500 0x0002").unwrap();
    assert!(c.queue.is_empty());
    join(&mut c);
    assert!(c.queue.is_empty(), "still waiting: the device has not announced");
    let mut zdo = vec![0x08, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x11, 0x34, 0x12];
    zdo.extend_from_slice(&DEV); zdo.push(0x80);
    c.on_tx(3000 * MS, 11, &device_frame(1, 0xfffd, &zdo));
    assert_eq!(c.queue.len(), 1);
    assert!(c.queue[0].what.starts_with("read ep 10"));
    assert!(Coordinator::check("zb frobnicate").is_err());
    assert!(c.command(0, "zb write 10 0x0500 0x0010 bogus 1").is_err());
}

#[test]
fn zcl_values_decode_by_type() {
    assert_eq!(zcl_value(0x10, &[1]), Some(("true".into(), 1)));
    assert_eq!(zcl_value(0x21, &[0x34, 0x12]), Some(("4660".into(), 2)));
    assert_eq!(zcl_value(0x29, &[0xff, 0xff]), Some(("-1".into(), 2)));
    assert_eq!(zcl_value(0x42, b"\x03abc"), Some(("\"abc\"".into(), 4)));
    assert_eq!(zcl_value(0xf0, &DEV), Some(("dc:1e:d5:ff:fe:6e:8c:dc".into(), 8)));
    assert_eq!(zcl_value(0x19, &[0x09, 0x00]), Some(("0x9".into(), 2)));
    assert!(zcl_value(0x21, &[1]).is_none());
}

#[test]
fn config_spec_parses() {
    let c = Config::parse("chan=15,pan=0x1234,epan=0102030405060708,permit=0,verbose").unwrap();
    assert_eq!((c.channel, c.pan, c.permit_join, c.verbose), (15, 0x1234, false, true));
    assert_eq!(c.ext_pan, [8, 7, 6, 5, 4, 3, 2, 1]);
    assert!(Config::parse("chan=27").is_err());
    assert!(Config::parse("key=abc").is_err());
    assert!(Config::parse("bogus=1").is_err());
}
