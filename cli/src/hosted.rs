//! The ESP32-C6 run loop with host-side models beside the machine: the virtual Zigbee
//! coordinator (`--zigbee`) and the PWM timeline (`--pwm-log`). Both need the clock cut at exact
//! cycles — a frame goes on the air at the acknowledgement turnaround, a duty change is stamped
//! within a millisecond — so this runs `Machine::run_until_cycle` from deadline to deadline instead
//! of `Machine::run`, and otherwise behaves as the plain loop does: scripts, stubs, console,
//! reboots through the ROM.

use esp32c6::zigbee::{Config, Coordinator, LQI, RSSI_DBM};
use esp32c6::Machine;
use esp_soc::{RunUntil, SocBus, Stop};

const CPU_HZ: u64 = esp32c6::periph::CPU_HZ;

pub struct Hosted {
    pub coordinator: Option<Coordinator>,
    pub pwm_pins: Vec<u8>,
    /// how often the PWM pins are sampled
    pub pwm_step: u64,
    last_pwm: Vec<Option<(f64, u32)>>,
    pub pwm_lines: Vec<String>,
}

impl Hosted {
    pub fn new(zigbee: Option<Config>, pwm_pins: Vec<u8>) -> Self {
        let last_pwm = vec![None; pwm_pins.len()];
        Hosted { coordinator: zigbee.map(Coordinator::new), pwm_pins, pwm_step: CPU_HZ / 1000, last_pwm, pwm_lines: Vec::new() }
    }

    fn sample_pwm(&mut self, m: &Machine) {
        let now = m.bus.cycles();
        for (i, &pin) in self.pwm_pins.iter().enumerate() {
            let v = esp_soc::SocBus::pwm_output(&m.bus, pin).map(|(hz, duty)| ((hz * 1000.0).round() / 1000.0, duty));
            if v != self.last_pwm[i] {
                let line = match v {
                    Some((hz, duty)) => format!("[pwm] t={:.3}s GPIO{pin}: {hz:.1} Hz {:.2}%", now as f64 / CPU_HZ as f64, duty as f64 * 100.0 / 65535.0),
                    None => format!("[pwm] t={:.3}s GPIO{pin}: off", now as f64 / CPU_HZ as f64),
                };
                eprintln!("{line}");
                self.pwm_lines.push(line);
                self.last_pwm[i] = v;
            }
        }
    }

    /// Run until `m.max_cycles`, a script `stop`, or a halt; chip resets reboot through the ROM
    /// when `reboot` is set, as the plain loop does.
    pub fn run(&mut self, m: &mut Machine, reboot: bool) -> Stop {
        let end = m.max_cycles;
        let mut next_pwm = 0u64;
        loop {
            let now = m.bus.cycles();
            // script commands for the host models, at the time they were placed
            for (t, cmd) in m.take_host_actions() {
                match &mut self.coordinator {
                    Some(c) => { if let Err(e) = c.command(t, &cmd) { eprintln!("[zb] {e}"); } }
                    None => eprintln!("[script] `{cmd}` needs --zigbee"),
                }
            }
            // frames due on the air
            if let Some(c) = &mut self.coordinator {
                for p in c.take_due(now) {
                    let on_channel = m.bus.periph.radio.channel() == c.cfg.channel;
                    let taken = on_channel && m.bus.radio_receive(&p.frame, RSSI_DBM, LQI, Some(0));
                    m.sync_irq();
                    c.outcome(now, p, taken);
                }
            }
            if !self.pwm_pins.is_empty() && now >= next_pwm { self.sample_pwm(m); next_pwm = now + self.pwm_step; }
            if now >= end { return Stop::Halted; }
            let mut target = end;
            if let Some(c) = &self.coordinator { if let Some(d) = c.next_deadline() { target = target.min(d); } }
            if !self.pwm_pins.is_empty() { target = target.min(next_pwm); }
            if let Some((at, _)) = m.script.events.get(m.script.pos) { target = target.min(*at + 1); }
            if target <= now { target = now + 1; }
            let r = m.run_until_cycle(target);
            m.drain_console();
            match r {
                RunUntil::Reached => {}
                RunUntil::Yield => {
                    let (ch, frame) = { let r = &m.bus.periph.radio; (r.channel(), r.tx_frame.clone()) };
                    if let Some(c) = &mut self.coordinator { c.on_tx(m.bus.cycles(), ch, &frame); }
                }
                RunUntil::Stop(Stop::SwReset) => {
                    let cause = m.bus.reset_cause();
                    eprintln!("[emu] chip reset at t={:.3}s: cause {:#x} ({})", m.seconds(), cause, esp_periph::reset_cause_name(cause));
                    if !reboot { return Stop::SwReset; }
                    m.reboot();
                    self.last_pwm.iter_mut().for_each(|v| *v = None);
                }
                RunUntil::Stop(s) => return s,
            }
        }
    }

    pub fn report(&self) -> Vec<String> {
        let mut out = Vec::new();
        if let Some(c) = &self.coordinator { out.push(c.report()); }
        out
    }
}
