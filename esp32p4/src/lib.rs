//! ESP32-P4: two RISC-V HP cores (RV32IMAFC), 768 KB of L2MEM (code and data in one space, an uncached alias
//! at +0x4000_0000), 8 KB of TCM, 32 KB of LP RAM, and separate flash and PSRAM windows, each
//! behind its own 1024-entry MMU. No radio.
//!
//! The silicon revision is a type parameter (`Revision`), because rev < 3 and rev >= 3 differ in
//! ISA, interrupt-threshold location and ROM. `P4` is rev < 3 (v0.x, v1.x), the only one so far.
//!
//! Shared IP (UART, USB-Serial/JTAG, systimer, timer groups, SPI flash controller, efuse) comes
//! from `esp-periph`. Written here: the memory map, the interrupt matrices and CLIC register
//! blocks, the cache controller, HP_SYSTEM, HP_SYS_CLKRST, the LP system blocks and the analog
//! I2C master. The interrupt registers are modelled, but nothing is delivered to the core: its
//! CLIC trap path is not there yet (`periph::Clic`).
pub mod bus;
pub mod periph;
pub mod soc;

pub use esp_soc::Stop;
pub use soc::{machine, Esp32p4, Machine, RevLt3, Revision, P4};
