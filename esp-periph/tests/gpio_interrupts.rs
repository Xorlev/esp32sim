//! GPIO interrupt status as the ISR reads it.
/// A level-triggered interrupt (IDF's `GPIO_INTR_LOW_LEVEL`, what `gpio_wakeup_enable` arms a
/// wake button with) must show its pin in `STATUS` and `PCPU_INT` while the level holds: that is
/// how `gpio_isr` finds the handler to call. Until it did, the line was raised and no handler ran.
#[test]
fn level_interrupt_shows_its_pin_in_the_status_registers() {
    let mut gpio = esp_periph::Gpio::new();
    gpio.write(0x74 + 4, (4 << 7) | (1 << 13));   // GPIO1: low-level interrupt, enabled for core 0
    assert!(!gpio.irq());
    assert_eq!(gpio.read(0x5c) & 2, 0);
    gpio.set_input(1, false);
    assert!(gpio.irq());
    assert_eq!(gpio.read(0x5c) & 2, 2, "PCPU_INT shows the pin while the level holds");
    assert_eq!(gpio.read(0x44) & 2, 2, "so does STATUS");
    gpio.write(0x4c, 2);                            // STATUS_W1TC: no effect on a level that persists
    assert_eq!(gpio.read(0x5c) & 2, 2);
    gpio.set_input(1, true);
    assert!(!gpio.irq());
    assert_eq!(gpio.read(0x5c) & 2, 0);
    // a high-level interrupt, the other way round
    gpio.write(0x74 + 8, (5 << 7) | (1 << 13));
    assert_eq!(gpio.read(0x5c) & 4, 4);
    gpio.set_input(2, false);
    assert_eq!(gpio.read(0x5c) & 4, 0);
    // the edge path is as it was: a falling edge latches and W1TC clears it
    gpio.write(0x74 + 12, (2 << 7) | (1 << 13));
    gpio.set_input(3, true);
    gpio.set_input(3, false);
    assert_eq!(gpio.read(0x5c) & 8, 8);
    gpio.write(0x4c, 8);
    assert_eq!(gpio.read(0x5c) & 8, 0);
}
