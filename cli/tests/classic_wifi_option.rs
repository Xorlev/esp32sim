// The classic ESP32 has a station link (#209); `--wifi` must attach it as on the other chips instead of
// being refused as unavailable. The run attaches the access point and then exits on the missing firmware
// (with a ROM dir it executes one instruction); either way the AP line has been printed.
#[test]
fn classic_esp32_accepts_the_wifi_option() {
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_esp32sim"))
        .args(["--chip", "esp32", "--wifi", "ssid=demo,chan=6", "--net", "none", "--max-insns", "1"]).output().unwrap();
    let stderr = String::from_utf8(output.stderr).unwrap();
    assert!(!stderr.contains("not available on the classic ESP32"), "{stderr}");
    assert!(stderr.contains("virtual AP 'demo'"), "{stderr}");
    assert!(!stderr.contains("panicked"), "{stderr}");
    // The other gates stay.
    let output = std::process::Command::new(env!("CARGO_BIN_EXE_esp32sim")).args(["--chip", "esp32", "--psram-mb", "2"]).output().unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert!(String::from_utf8(output.stderr).unwrap().contains("--psram-mb is not available on the classic ESP32"));
}

// `--net`'s forwards: each printed as it is set up, a malformed one or an unknown mode refused
// with the usage status.
#[test]
fn the_net_option_takes_forwards_and_refuses_bad_ones() {
    let run = |net: &str| std::process::Command::new(env!("CARGO_BIN_EXE_esp32sim"))
        .args(["--chip", "esp32", "--wifi", "ssid=demo", "--net", net, "--max-insns", "1"]).output().unwrap();
    let ok = run("none,epoch=1767225600,fwd=10.0.2.100:445=127.0.0.1:4445,fwd=10.0.2.3:1883=127.0.0.1:1884");
    let stderr = String::from_utf8(ok.stderr).unwrap();
    assert!(stderr.contains("TCP to 10.0.2.100:445 is forwarded to 127.0.0.1:4445") && stderr.contains("TCP to 10.0.2.3:1883 is forwarded to 127.0.0.1:1884"), "{stderr}");
    assert!(!stderr.contains("NAT to the host network"), "none relays only the forwards: {stderr}");
    for bad in ["none,fwd=10.0.2.100:445", "none,fwd=nas:445=127.0.0.1:1", "nat,fwd=10.0.2.100:445=127.0.0.1:0", "nta"] {
        let out = run(bad);
        assert_eq!(out.status.code(), Some(2), "{bad}: {}", String::from_utf8_lossy(&out.stderr));
    }
}
