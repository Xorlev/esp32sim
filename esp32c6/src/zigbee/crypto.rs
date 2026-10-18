//! The three primitives Zigbee security is built from, on the AES-128 block cipher
//! `esp_periph::crypto` already has: CCM* (IEEE 802.15.4 Annex B, as Zigbee uses it — a 13-byte
//! nonce, a 2-byte length field, a 32-bit MIC), the Matyas-Meyer-Oseas hash on AES (Zigbee
//! specification B.6) and the HMAC over it (B.1.4) that derives the key-transport, key-load and
//! verify-key values from a link key.

use esp_periph::crypto::aes128_encrypt_block;

/// Security level 5: encryption with a 4-byte MIC — the only level Zigbee PRO networks use.
pub const MIC_LEN: usize = 4;

fn xor_into(dst: &mut [u8], src: &[u8]) { for (d, s) in dst.iter_mut().zip(src) { *d ^= *s; } }

/// CCM* with M = 4 and L = 2 (IEEE 802.15.4-2006 Annex B). `nonce` is 13 bytes;
/// `a` is authenticated only, `m` is authenticated and encrypted. Returns ciphertext || MIC.
pub fn ccm_encrypt(key: &[u8; 16], nonce: &[u8; 13], a: &[u8], m: &[u8]) -> Vec<u8> {
    let mic = cbc_mac(key, nonce, a, m);
    let mut out = ctr(key, nonce, m, 1);
    let mut u = [0u8; MIC_LEN];
    u.copy_from_slice(&mic[..MIC_LEN]);
    let s0 = ctr_block(key, nonce, 0);
    xor_into(&mut u, &s0[..MIC_LEN]);
    out.extend_from_slice(&u);
    out
}

/// The inverse: `c` is ciphertext || MIC. `None` if the MIC does not check.
pub fn ccm_decrypt(key: &[u8; 16], nonce: &[u8; 13], a: &[u8], c: &[u8]) -> Option<Vec<u8>> {
    if c.len() < MIC_LEN { return None; }
    let (ct, u) = c.split_at(c.len() - MIC_LEN);
    let m = ctr(key, nonce, ct, 1);
    let s0 = ctr_block(key, nonce, 0);
    let mut t = [0u8; MIC_LEN];
    t.copy_from_slice(u);
    xor_into(&mut t, &s0[..MIC_LEN]);
    let mic = cbc_mac(key, nonce, a, &m);
    if mic[..MIC_LEN] == t { Some(m) } else { None }
}

/// B0 || encoded a || m, through AES-CBC-MAC. Flags: Adata, M' = (4-2)/2 = 1, L' = 1.
fn cbc_mac(key: &[u8; 16], nonce: &[u8; 13], a: &[u8], m: &[u8]) -> [u8; 16] {
    let mut b0 = [0u8; 16];
    b0[0] = (if a.is_empty() { 0 } else { 0x40 }) | (((MIC_LEN as u8 - 2) / 2) << 3) | 1;
    b0[1..14].copy_from_slice(nonce);
    b0[14..16].copy_from_slice(&(m.len() as u16).to_be_bytes());
    let mut x = aes128_encrypt_block(key, &b0);
    let mut blocks: Vec<u8> = Vec::new();
    if !a.is_empty() {
        // a < 2^16 - 2^8 always holds for an 802.15.4 frame
        blocks.extend_from_slice(&(a.len() as u16).to_be_bytes());
        blocks.extend_from_slice(a);
        while !blocks.len().is_multiple_of(16) { blocks.push(0); }
    }
    blocks.extend_from_slice(m);
    while !blocks.len().is_multiple_of(16) { blocks.push(0); }
    for chunk in blocks.chunks(16) {
        xor_into(&mut x, chunk);
        x = aes128_encrypt_block(key, &x);
    }
    x
}

fn ctr_block(key: &[u8; 16], nonce: &[u8; 13], i: u16) -> [u8; 16] {
    let mut a = [0u8; 16];
    a[0] = 1;                                   // flags: L' = 1
    a[1..14].copy_from_slice(nonce);
    a[14..16].copy_from_slice(&i.to_be_bytes());
    aes128_encrypt_block(key, &a)
}

fn ctr(key: &[u8; 16], nonce: &[u8; 13], data: &[u8], first: u16) -> Vec<u8> {
    let mut out = data.to_vec();
    for (k, chunk) in out.chunks_mut(16).enumerate() {
        let s = ctr_block(key, nonce, first + k as u16);
        xor_into(chunk, &s[..chunk.len()]);
    }
    out
}

/// The Matyas-Meyer-Oseas hash on AES-128 (Zigbee B.6): H_i = E(H_{i-1}, M_i) xor M_i over
/// 16-byte blocks, padded with 0x80, zeros, and the bit length in 16 bits.
pub fn mmo_hash(data: &[u8]) -> [u8; 16] {
    let mut padded = data.to_vec();
    padded.push(0x80);
    while padded.len() % 16 != 14 { padded.push(0); }
    padded.extend_from_slice(&((data.len() as u64 * 8) as u16).to_be_bytes());
    let mut h = [0u8; 16];
    for block in padded.chunks(16) {
        let mut b = [0u8; 16];
        b.copy_from_slice(block);
        let e = aes128_encrypt_block(&h, &b);
        for i in 0..16 { h[i] = e[i] ^ b[i]; }
    }
    h
}

/// HMAC with the MMO hash (B.1.4, FIPS 198 with a 16-byte block). `hmac_mmo(link_key, &[0x00])`
/// is the key-transport key, `[0x02]` the key-load key, `[0x03]` the verify-key hash.
pub fn hmac_mmo(key: &[u8; 16], text: &[u8]) -> [u8; 16] {
    let mut ipad = [0x36u8; 16];
    let mut opad = [0x5cu8; 16];
    xor_into(&mut ipad, key);
    xor_into(&mut opad, key);
    let mut inner = ipad.to_vec();
    inner.extend_from_slice(text);
    let h1 = mmo_hash(&inner);
    let mut outer = opad.to_vec();
    outer.extend_from_slice(&h1);
    mmo_hash(&outer)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ccm_round_trip_and_tamper() {
        let key = [0x11u8; 16];
        let nonce = [0x22u8; 13];
        let a = b"header bytes";
        let m = b"the payload, three blocks long or so..............";
        let c = ccm_encrypt(&key, &nonce, a, m);
        assert_eq!(c.len(), m.len() + MIC_LEN);
        assert_eq!(ccm_decrypt(&key, &nonce, a, &c).as_deref(), Some(&m[..]));
        let mut bad = c.clone(); bad[3] ^= 1;
        assert!(ccm_decrypt(&key, &nonce, a, &bad).is_none());
        assert!(ccm_decrypt(&key, &nonce, b"other", &c).is_none());
    }

    /// The key-transport key of the default trust-centre link key "ZigBeeAlliance09": the value
    /// Wireshark derives for it (its `zbee_sec_key_hash`, cross-checked here against an OpenSSL
    /// AES), and the key every Zigbee sniffer uses to read a joining device's Transport-Key.
    #[test]
    fn key_transport_key_of_the_default_link_key() {
        let out = hmac_mmo(b"ZigBeeAlliance09", &[0x00]);
        assert_eq!(out, [0x4b, 0xab, 0x0f, 0x17, 0x3e, 0x14, 0x34, 0xa2, 0xd5, 0x72, 0xe1, 0xc1, 0xef, 0x47, 0x87, 0x82]);
        let load = hmac_mmo(b"ZigBeeAlliance09", &[0x02]);
        assert_eq!(load, [0xc5, 0xa4, 0x70, 0x35, 0xc3, 0x32, 0xcc, 0xbf, 0x25, 0x15, 0x71, 0xd8, 0xba, 0xde, 0xd1, 0x88]);
    }

    /// The MMO hash of "abc" and the HMAC of the byte 0xC0 under the key C0..CF, as the reference
    /// padding (0x80, zeros to 14 bytes, the bit length big-endian) gives them.
    #[test]
    fn mmo_and_hmac_reference_values() {
        assert_eq!(mmo_hash(b"abc"), [0xbd, 0x2f, 0x2e, 0xbd, 0x93, 0xfa, 0xdc, 0x48, 0xbc, 0x00, 0x17, 0x4d, 0x95, 0x42, 0x27, 0x41]);
        let key: [u8; 16] = core::array::from_fn(|i| 0xC0 + i as u8);
        assert_eq!(hmac_mmo(&key, &[0xC0]), [0x17, 0xc6, 0x2b, 0x95, 0x29, 0x02, 0x5a, 0x0d, 0xb2, 0xc2, 0x41, 0x60, 0x45, 0xe7, 0x83, 0xd3]);
    }
}
