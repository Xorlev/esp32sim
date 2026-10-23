//! Real network services for `external_` tests: mosquitto and samba's smbd, unprivileged, each
//! listening on 127.0.0.1 from a directory of its own, stopped when dropped. Included with
//! `#[path]` like `common`. The binaries come from the environment (`MOSQUITTO`, `SMBD`, ...) or
//! the usual install paths; [esp32p4.md](../docs/esp32p4.md), "Services", has the setup.
#![allow(dead_code)]
use std::net::{Ipv4Addr, SocketAddr, TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

/// A port on 127.0.0.1 that nothing listened on a moment ago. Another process can take it before
/// the service binds it; the service then fails to start and its log says so.
pub fn free_port() -> u16 { TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap().local_addr().unwrap().port() }

/// A new empty directory under the system temp directory, unique to this process and call.
pub fn temp_dir(name: &str) -> PathBuf {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let id = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let d = std::env::temp_dir().join(format!("esp32sim-svc-{}-{id}-{name}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// `var` if set, else the first of `paths` that exists, else the first of `names` found on PATH.
pub fn find_tool(var: &str, paths: &[&str], names: &[&str]) -> Result<PathBuf, String> {
    if let Some(v) = std::env::var_os(var) { return Ok(PathBuf::from(v)); }
    if let Some(p) = paths.iter().map(PathBuf::from).find(|p| p.is_file()) { return Ok(p); }
    let path = std::env::var_os("PATH").unwrap_or_default();
    names.iter().flat_map(|n| std::env::split_paths(&path).map(move |d| d.join(n))).find(|p| p.is_file())
        .ok_or_else(|| format!("{} not found: set {var} or install it (tried {})", names.first().unwrap_or(&var), paths.iter().chain(names).copied().collect::<Vec<_>>().join(", ")))
}

fn tail(text: &str) -> String { let n = text.len(); text[n.saturating_sub(2000)..].to_string() }

/// Run a tool to completion; its output, or why it failed.
pub fn run_tool(cmd: &mut Command, stdin: &str) -> Result<String, String> {
    use std::io::Write;
    let mut child = cmd.stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().map_err(|e| format!("{cmd:?}: {e}"))?;
    child.stdin.take().unwrap().write_all(stdin.as_bytes()).map_err(|e| format!("{cmd:?}: {e}"))?;
    let out = child.wait_with_output().map_err(|e| format!("{cmd:?}: {e}"))?;
    let text = format!("{}{}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    if out.status.success() { Ok(text) } else { Err(format!("{cmd:?}: {}\n{}", out.status, tail(&text))) }
}

/// A server process and its children in a process group of their own, listening on
/// 127.0.0.1:`port`. Dropping it sends the group SIGTERM, then SIGKILL after 5 s: smbd's
/// parent stops its children on SIGTERM, and a SIGKILL to the parent alone would leave them.
pub struct Service { pub name: String, pub port: u16, pub dir: PathBuf, log: PathBuf, child: Child }

impl Service {
    /// Start `cmd`, its output in `dir/NAME.log`, and wait until 127.0.0.1:`port` accepts a
    /// connection. A process that exits first, or does not listen within `timeout`, is an error
    /// with the end of its log.
    pub fn start(name: &str, mut cmd: Command, port: u16, dir: &Path, timeout: Duration) -> Result<Service, String> {
        use std::os::unix::process::CommandExt;
        let log = dir.join(format!("{name}.log"));
        let out = std::fs::File::create(&log).map_err(|e| format!("{}: {e}", log.display()))?;
        cmd.stdin(Stdio::null()).stdout(out.try_clone().unwrap()).stderr(out).process_group(0);
        let child = cmd.spawn().map_err(|e| format!("{name}: {cmd:?}: {e}"))?;
        let mut s = Service { name: name.into(), port, dir: dir.into(), log, child };
        s.wait_listening(timeout)?;
        Ok(s)
    }

    pub fn pid(&self) -> u32 { self.child.id() }
    pub fn addr(&self) -> SocketAddr { SocketAddr::from((Ipv4Addr::LOCALHOST, self.port)) }
    pub fn log_text(&self) -> String { std::fs::read_to_string(&self.log).unwrap_or_default() }

    fn wait_listening(&mut self, timeout: Duration) -> Result<(), String> {
        let t0 = Instant::now();
        loop {
            if TcpStream::connect_timeout(&self.addr(), Duration::from_millis(200)).is_ok() { return Ok(()); }
            if let Ok(Some(status)) = self.child.try_wait() {
                return Err(format!("{} exited ({status}) before listening on {}:\n{}", self.name, self.addr(), tail(&self.log_text())));
            }
            if t0.elapsed() > timeout {
                return Err(format!("{} not listening on {} after {:?}:\n{}", self.name, self.addr(), timeout, tail(&self.log_text())));
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// Stop the group and reap the process. Once the leader is reaped its id can be reused, so the
    /// group gets SIGKILL only while a member is still there to hold the id.
    pub fn stop(&mut self) {
        let pgid = self.child.id();
        let group = format!("-{pgid}");
        let signal = |sig: &str| { let _ = Command::new("kill").args([sig, "--", &group]).stderr(Stdio::null()).status(); };
        signal("-TERM");
        let t0 = Instant::now();
        let mut exited = false;
        while t0.elapsed() < Duration::from_secs(5) {
            if let Ok(Some(_)) = self.child.try_wait() { exited = true; break; }
            std::thread::sleep(Duration::from_millis(20));
        }
        if !exited || group_alive(pgid) { signal("-KILL"); }
        let _ = self.child.wait();
    }
}

impl Drop for Service { fn drop(&mut self) { self.stop(); } }

/// Whether any process of the process group `pgid` is still there (`kill -0`).
pub fn group_alive(pgid: u32) -> bool {
    Command::new("kill").args(["-0", "--", &format!("-{pgid}")]).stderr(Stdio::null()).status().is_ok_and(|s| s.success())
}

fn mosquitto_tool(name: &str) -> Result<PathBuf, String> {
    let var = name.to_uppercase();
    let paths: Vec<String> = ["/opt/homebrew/bin", "/usr/local/bin", "/usr/bin"].iter().map(|d| format!("{d}/{name}")).collect();
    find_tool(&var, &paths.iter().map(String::as_str).collect::<Vec<_>>(), &[name])
}

/// The broker's configuration: one listener on 127.0.0.1, no anonymous clients, the users in
/// `passwd`, nothing kept on disk.
pub fn mosquitto_conf(port: u16, passwd: &Path) -> String {
    format!("listener {port} 127.0.0.1\nallow_anonymous false\npassword_file {}\npersistence false\nlog_dest stderr\nlog_type error\nlog_type warning\nlog_type notice\nlog_type information\n", passwd.display())
}

/// mosquitto with one user. `MOSQUITTO` names the broker, `MOSQUITTO_PASSWD`, `MOSQUITTO_SUB`
/// and `MOSQUITTO_PUB` the clients.
pub struct Mosquitto { pub svc: Service, pub client: MqttClient }

/// The broker's address and the user, for its command-line clients.
#[derive(Clone, Debug)]
pub struct MqttClient { pub port: u16, pub user: String, pub pass: String }

impl Mosquitto {
    pub fn start(user: &str, pass: &str) -> Result<Mosquitto, String> {
        let broker = find_tool("MOSQUITTO", &["/opt/homebrew/sbin/mosquitto", "/usr/local/sbin/mosquitto", "/usr/sbin/mosquitto"], &["mosquitto"])?;
        let dir = temp_dir("mosquitto");
        let passwd = dir.join("passwd");
        run_tool(Command::new(mosquitto_tool("mosquitto_passwd")?).arg("-c").arg("-b").arg(&passwd).args([user, pass]), "")?;
        let port = free_port();
        let conf = dir.join("mosquitto.conf");
        std::fs::write(&conf, mosquitto_conf(port, &passwd)).unwrap();
        let mut cmd = Command::new(broker);
        cmd.arg("-c").arg(&conf);
        let svc = Service::start("mosquitto", cmd, port, &dir, Duration::from_secs(10))?;
        Ok(Mosquitto { svc, client: MqttClient { port, user: user.into(), pass: pass.into() } })
    }
}

impl MqttClient {
    fn tool(&self, tool: &str) -> Result<Command, String> {
        let mut c = Command::new(mosquitto_tool(tool)?);
        c.args(["-h", "127.0.0.1", "-p", &self.port.to_string(), "-u", &self.user, "-P", &self.pass]);
        Ok(c)
    }

    /// The retained messages on `topics` (filters allowed), as (topic, payload): what a client
    /// subscribing now is sent first. Waits up to `wait` for them.
    pub fn retained(&self, topics: &[&str], wait: Duration) -> Result<Vec<(String, String)>, String> {
        let mut c = self.tool("mosquitto_sub")?;
        c.args(["-v", "--retained-only", "-W", &wait.as_secs().max(1).to_string()]);
        for t in topics { c.args(["-t", t]); }
        let out = c.stdin(Stdio::null()).output().map_err(|e| format!("mosquitto_sub: {e}"))?;
        // 27 is mosquitto_sub's exit status when -W runs out
        if !out.status.success() && out.status.code() != Some(27) {
            return Err(format!("mosquitto_sub: {}: {}", out.status, String::from_utf8_lossy(&out.stderr)));
        }
        Ok(String::from_utf8_lossy(&out.stdout).lines().filter_map(|l| l.split_once(' ').map(|(t, p)| (t.to_string(), p.to_string()))).collect())
    }

    pub fn publish(&self, topic: &str, payload: &str) -> Result<(), String> {
        run_tool(self.tool("mosquitto_pub")?.args(["-t", topic, "-m", payload]), "").map(|_| ())
    }

    /// At QoS 1 over MQTT 5, with a message expiry of `expiry_s` if given: queued by the broker
    /// for a persistent session that is away, and dropped unsent once it expires. The broker
    /// expires by its own clock, not the emulator's.
    pub fn publish_v5(&self, topic: &str, payload: &str, expiry_s: Option<u32>) -> Result<(), String> {
        let mut c = self.tool("mosquitto_pub")?;
        c.args(["-V", "mqttv5", "-q", "1", "-t", topic, "-m", payload]);
        if let Some(s) = expiry_s { c.args(["-D", "publish", "message-expiry-interval", &s.to_string()]); }
        run_tool(&mut c, "").map(|_| ())
    }
    /// A subscriber on `topics` from now until the `Listener` is dropped, for messages that are
    /// not retained. mosquitto_sub writes each message as it arrives; the wait after starting it
    /// lets the subscription reach the broker before the caller runs the firmware on.
    pub fn listen(&self, topics: &[&str]) -> Result<Listener, String> {
        let out = temp_dir("mosquitto-sub").join("messages");
        let file = std::fs::File::create(&out).map_err(|e| format!("{}: {e}", out.display()))?;
        let mut c = self.tool("mosquitto_sub")?;
        c.arg("-v");
        for t in topics { c.args(["-t", t]); }
        let child = c.stdin(Stdio::null()).stdout(file).stderr(Stdio::null()).spawn().map_err(|e| format!("mosquitto_sub: {e}"))?;
        std::thread::sleep(Duration::from_millis(500));
        Ok(Listener { child, out })
    }
}

/// A running `mosquitto_sub`; `MqttClient::listen`.
pub struct Listener { child: Child, out: PathBuf }

impl Listener {
    /// Every message so far, as (topic, payload).
    pub fn messages(&self) -> Vec<(String, String)> {
        std::fs::read_to_string(&self.out).unwrap_or_default().lines().filter_map(|l| l.split_once(' ').map(|(t, p)| (t.to_string(), p.to_string()))).collect()
    }
}

impl Drop for Listener {
    fn drop(&mut self) { let _ = self.child.kill(); let _ = self.child.wait(); }
}

/// A TCP relay on 127.0.0.1 in front of a service, which a test can cut (every connection
/// closed, new ones closed as they arrive) or make deaf one way (what the client sends is read
/// and dropped, what the service sends still arrives), standing in for a network that drops the
/// device or loses its acknowledgements. Its threads end with the process.
pub struct Relay { pub port: u16, state: std::sync::Arc<RelayState> }

struct RelayState { mode: std::sync::atomic::AtomicU8, live: std::sync::Mutex<Vec<TcpStream>> }

const RELAY_PASS: u8 = 0;
const RELAY_DEAF: u8 = 1;
const RELAY_CUT: u8 = 2;

impl Relay {
    pub fn start(target: SocketAddr) -> Result<Relay, String> {
        use std::sync::{atomic::Ordering, Arc};
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).map_err(|e| format!("relay: {e}"))?;
        let port = listener.local_addr().unwrap().port();
        let state = Arc::new(RelayState { mode: RELAY_PASS.into(), live: Default::default() });
        let st = state.clone();
        std::thread::spawn(move || {
            for client in listener.incoming().flatten() {
                if st.mode.load(Ordering::SeqCst) == RELAY_CUT { continue; }
                let Ok(server) = TcpStream::connect(target) else { continue };
                let (Ok(c2), Ok(s2), Ok(c3), Ok(s3)) = (client.try_clone(), server.try_clone(), client.try_clone(), server.try_clone()) else { continue };
                st.live.lock().unwrap().extend([c3, s3]);
                let up = st.clone();
                std::thread::spawn(move || pump(client, server, Some(up.as_ref())));
                std::thread::spawn(move || pump(s2, c2, None));
            }
        });
        Ok(Relay { port, state })
    }

    pub fn addr(&self) -> SocketAddr { SocketAddr::from((Ipv4Addr::LOCALHOST, self.port)) }

    /// Close every connection both ways, and every new one as it arrives, until `restore`.
    pub fn cut(&self) {
        self.state.mode.store(RELAY_CUT, std::sync::atomic::Ordering::SeqCst);
        for s in self.state.live.lock().unwrap().drain(..) { let _ = s.shutdown(std::net::Shutdown::Both); }
    }

    /// Drop what clients send from now on; the service's side still flows to them.
    pub fn deafen(&self) { self.state.mode.store(RELAY_DEAF, std::sync::atomic::Ordering::SeqCst); }

    pub fn restore(&self) { self.state.mode.store(RELAY_PASS, std::sync::atomic::Ordering::SeqCst); }
}

/// Copy `from` to `to` until either end closes, then close both. `deaf_if` is the relay whose
/// deaf mode drops this direction's bytes.
fn pump(mut from: TcpStream, mut to: TcpStream, deaf_if: Option<&RelayState>) {
    use std::io::{Read, Write};
    let mut buf = [0u8; 4096];
    loop {
        let n = match from.read(&mut buf) { Ok(0) | Err(_) => break, Ok(n) => n };
        if deaf_if.is_some_and(|s| s.mode.load(std::sync::atomic::Ordering::SeqCst) == RELAY_DEAF) { continue; }
        if to.write_all(&buf[..n]).is_err() { break; }
    }
    let _ = from.shutdown(std::net::Shutdown::Both);
    let _ = to.shutdown(std::net::Shutdown::Both);
}

fn samba_tool(var: &str, name: &str) -> Result<PathBuf, String> {
    let paths: Vec<String> = ["/opt/homebrew/opt/samba/bin", "/usr/local/opt/samba/bin", "/usr/bin"].iter().map(|d| format!("{d}/{name}")).collect();
    find_tool(var, &paths.iter().map(String::as_str).collect::<Vec<_>>(), &[name])
}

/// smbd's configuration: a standalone server on 127.0.0.1:`port`, every directory it keeps state
/// in under `dir`, SMB 2.1 to 3.1.1 with signing required (`SAMBA_SERVER_SIGNING` sets another
/// `server signing`, such as `auto`: offered, not required), and one share writable by the user
/// that `users.map` maps to `unix_user`. `vfs mkdir use tmp name` is off: samba 4.25 makes a
/// directory under a temporary name with mode 0 and renames it into place, and on macOS that
/// rename fails (EACCES), leaving a mode-0 directory and the client ACCESS_DENIED.
pub fn samba_conf(dir: &Path, port: u16, share: &str, unix_user: &str) -> String {
    let d = dir.display();
    let signing = std::env::var("SAMBA_SERVER_SIGNING").unwrap_or_else(|_| "mandatory".into());
    format!("[global]
  smb ports = {port}
  bind interfaces only = yes
  interfaces = 127.0.0.1
  server role = standalone server
  server min protocol = SMB2_10
  server max protocol = SMB3_11
  server signing = {signing}
  map to guest = never
  passdb backend = tdbsam:{d}/private/passdb.tdb
  username map = {d}/users.map
  private dir = {d}/private
  lock directory = {d}/lock
  state directory = {d}/state
  cache directory = {d}/cache
  pid directory = {d}/pid
  ncalrpc dir = {d}/ncalrpc
  log file = {d}/log/smbd.log
  vfs mkdir use tmp name = no
  disable spoolss = yes
  load printers = no
[{share}]
  path = {d}/share
  read only = no
  valid users = {unix_user}
")
}

/// smbd serving one share to one user. samba keeps a password only for an account of the host,
/// so the user is mapped (`username map`) to the account running the test. `SMBD`, `PDBEDIT` and
/// `SMBCLIENT` name the tools; on macOS Homebrew installs smbd as `samba-dot-org-smbd`, and
/// /usr/sbin/smbd is the system's own server, never this one.
pub struct Samba { pub svc: Service, pub share: String, pub user: String, pub pass: String, pub conf: PathBuf }

impl Samba {
    pub fn start(share: &str, user: &str, pass: &str) -> Result<Samba, String> {
        let mut paths = vec!["/opt/homebrew/opt/samba/sbin/samba-dot-org-smbd", "/usr/local/opt/samba/sbin/samba-dot-org-smbd"];
        if !cfg!(target_os = "macos") { paths.push("/usr/sbin/smbd"); }
        let smbd = find_tool("SMBD", &paths, &[if cfg!(target_os = "macos") { "samba-dot-org-smbd" } else { "smbd" }])?;
        let unix_user = run_tool(Command::new("id").arg("-un"), "")?.trim().to_string();
        let dir = temp_dir("samba");
        for d in ["private", "lock", "state", "cache", "pid", "ncalrpc", "log", "share"] { std::fs::create_dir_all(dir.join(d)).unwrap(); }
        std::fs::write(dir.join("users.map"), format!("{unix_user} = {user}\n")).unwrap();
        let port = free_port();
        let conf = dir.join("smb.conf");
        std::fs::write(&conf, samba_conf(&dir, port, share, &unix_user)).unwrap();
        run_tool(Command::new(samba_tool("PDBEDIT", "pdbedit")?).arg("-s").arg(&conf).args(["-a", "-u", &unix_user, "-t"]), &format!("{pass}\n{pass}\n"))?;
        let mut cmd = Command::new(smbd);
        cmd.arg("-s").arg(&conf).args(["--foreground", "--no-process-group"]);
        let svc = Service::start("smbd", cmd, port, &dir, Duration::from_secs(20))?;
        Ok(Samba { svc, share: share.into(), user: user.into(), pass: pass.into(), conf })
    }

    /// The share's directory on the host.
    pub fn path(&self) -> PathBuf { self.svc.dir.join("share") }

    /// smbclient's `-c` commands against the share, as the user, over SMB3.
    pub fn smbclient(&self, commands: &str) -> Result<String, String> {
        run_tool(Command::new(samba_tool("SMBCLIENT", "smbclient")?).arg("-s").arg(&self.conf)
            .args(["-p", &self.svc.port.to_string(), "-m", "SMB3", &format!("//127.0.0.1/{}", self.share), "-U", &format!("{}%{}", self.user, self.pass), "-c", commands]), "")
    }
}

/// The fixture's own lifecycle with a listener the test holds: ready as soon as the port
/// accepts, and the whole process group gone after the drop.
#[test]
fn a_service_is_ready_when_its_port_accepts_and_stops_with_its_group() {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let port = listener.local_addr().unwrap().port();
    let mut cmd = Command::new("sh");
    cmd.args(["-c", "sleep 60 & sleep 60"]);
    let svc = Service::start("sleepers", cmd, port, &temp_dir("sleepers"), Duration::from_secs(5)).unwrap();
    let pgid = svc.pid();
    assert!(group_alive(pgid));
    drop(svc);
    assert!(!group_alive(pgid), "a process of group {pgid} outlived the drop");
}

#[test]
fn a_service_that_exits_or_never_listens_is_an_error_with_its_log() {
    let mut cmd = Command::new("sh");
    cmd.args(["-c", "echo cannot bind >&2; exit 3"]);
    let e = Service::start("quitter", cmd, free_port(), &temp_dir("quitter"), Duration::from_secs(5)).err().unwrap();
    assert!(e.contains("exited") && e.contains("cannot bind"), "{e}");
    let mut cmd = Command::new("sleep");
    cmd.arg("60");
    let t0 = Instant::now();
    let e = Service::start("sleeper", cmd, free_port(), &temp_dir("sleeper"), Duration::from_millis(300)).err().unwrap();
    assert!(e.contains("not listening"), "{e}");
    assert!(t0.elapsed() < Duration::from_secs(10), "the timeout held");
}

#[test]
fn the_service_configurations_keep_to_loopback_and_their_directory() {
    let dir = Path::new("/tmp/svc");
    let smb = samba_conf(dir, 4445, "archive", "someone");
    for l in smb.lines().filter(|l| l.contains(" dir") || l.contains("directory") || l.contains("file =") || l.contains("path =")) {
        assert!(l.split_once(" = ").is_some_and(|(_, v)| v.starts_with("/tmp/svc/") || v.starts_with("tdbsam:/tmp/svc/")), "{l}");
    }
    for want in ["smb ports = 4445", "interfaces = 127.0.0.1", "bind interfaces only = yes", "server signing = mandatory", "server max protocol = SMB3_11", "vfs mkdir use tmp name = no", "[archive]", "valid users = someone"] {
        assert!(smb.contains(want), "{want}");
    }
    let mq = mosquitto_conf(1884, Path::new("/tmp/svc/passwd"));
    assert!(mq.starts_with("listener 1884 127.0.0.1\n") && mq.contains("allow_anonymous false\n") && mq.contains("password_file /tmp/svc/passwd\n"));
}

/// The relay against an echo server: it passes both ways, drops only what the client sends
/// while deaf, closes the live connection and refuses new ones while cut, and passes again after.
#[test]
fn a_relay_passes_goes_deaf_one_way_and_cuts() {
    use std::io::{Read, Write};
    let echo = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let target = echo.local_addr().unwrap();
    std::thread::spawn(move || {
        for mut s in echo.incoming().flatten() {
            std::thread::spawn(move || { let mut b = [0u8; 64]; while let Ok(n @ 1..) = s.read(&mut b) { if s.write_all(&b[..n]).is_err() { break; } } });
        }
    });
    let relay = Relay::start(target).unwrap();
    let open = || { let s = TcpStream::connect(relay.addr()).unwrap(); s.set_read_timeout(Some(Duration::from_millis(500))).unwrap(); s };
    let echoed = |s: &mut TcpStream, b: &[u8]| -> Option<Vec<u8>> {
        s.write_all(b).ok()?;
        let mut got = vec![0u8; b.len()];
        s.read_exact(&mut got).ok().map(|_| got)
    };
    let mut s = open();
    assert_eq!(echoed(&mut s, b"one").as_deref(), Some(&b"one"[..]));
    relay.deafen();
    assert_eq!(echoed(&mut s, b"two"), None, "the server never heard it, so nothing came back");
    relay.restore();
    assert_eq!(echoed(&mut s, b"three").as_deref(), Some(&b"three"[..]));
    relay.cut();
    let mut b = [0u8; 8];
    assert!(matches!(s.read(&mut b), Ok(0) | Err(_)), "the live connection is closed");
    let mut late = open();
    assert!(matches!(late.read(&mut b), Ok(0) | Err(_)), "a new connection is closed while cut");
    relay.restore();
    let mut again = open();
    assert_eq!(echoed(&mut again, b"four").as_deref(), Some(&b"four"[..]));
}

/// Both servers from this machine's install: a file through the share and back, a retained
/// message through the broker.
#[test]
#[ignore = "needs samba (smbd, pdbedit, smbclient) and mosquitto with its clients installed"]
fn external_samba_and_mosquitto_serve_a_test_client() {
    let smb = Samba::start("archive", "esp32sim", "esp32sim-smb").unwrap_or_else(|e| panic!("{e}"));
    let src = smb.svc.dir.join("hello.txt");
    std::fs::write(&src, b"hello over SMB3").unwrap();
    smb.smbclient(&format!("put {} hello.txt; mkdir a; mkdir a/b; put {} a/b/hello.txt", src.display(), src.display())).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(std::fs::read(smb.path().join("hello.txt")).unwrap(), b"hello over SMB3");
    assert_eq!(std::fs::read(smb.path().join("a/b/hello.txt")).unwrap(), b"hello over SMB3", "a file in a directory the client made");
    assert!(smb.smbclient("ls").unwrap().contains("hello.txt"));
    let mq = Mosquitto::start("esp32sim", "esp32sim-mqtt").unwrap_or_else(|e| panic!("{e}"));
    mq.client.publish("esp32sim/test", "retained?").unwrap_or_else(|e| panic!("{e}"));
    let none = mq.client.retained(&["esp32sim/#"], Duration::from_secs(1)).unwrap();
    assert!(none.is_empty(), "a message published without -r is not retained: {none:?}");
    run_tool(mq.client.tool("mosquitto_pub").unwrap().args(["-r", "-t", "esp32sim/test", "-m", "kept"]), "").unwrap();
    let kept = mq.client.retained(&["esp32sim/#"], Duration::from_secs(1)).unwrap();
    assert_eq!(kept, [("esp32sim/test".to_string(), "kept".to_string())]);
}
