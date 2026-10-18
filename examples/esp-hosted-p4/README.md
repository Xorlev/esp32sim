# ESP-Hosted: an ESP32-P4 with an ESP32-C6 co-processor

Two firmware images that give an ESP32-P4 WiFi through an ESP32-C6 on SDIO, as on the Waveshare
ESP32-P4-WIFI6 and Espressif's P4 boards:

- `station/`: ESP-IDF v5.5.3's `wifi/getting_started/station` example (Unlicense or CC0-1.0, as
  its sources say) for the ESP32-P4 rev < 3, unchanged except for `sdkconfig.defaults`, which adds
  the target, the revision and the virtual access point's SSID `esp32sim` and password
  `esp32sim-pass`, and `main/idf_component.yml`, which pins espressif/esp_hosted to `==2.12.13`
  so the host matches the slave exactly. `dependencies.lock` pins the rest: esp_wifi_remote 1.6.5,
  eppp_link 1.1.6, esp_serial_slave_link 1.1.2, wifi_remote_over_eppp 0.3.3. Its ESP-Hosted
  defaults for the P4 are the reference board's: SDIO slot 1 through the GPIO matrix, CLK 18,
  CMD 19, D0-D3 14-17, four lines at 40 MHz, streaming receive, the C6's reset on GPIO 54
  (active high: low resets).
- `slave/`: ESP-Hosted 2.12.13's `slave` example (Apache-2.0), unchanged, for the ESP32-C6:
  `idf.py create-project-from-example "espressif/esp_hosted==2.12.13:slave"`, then
  `idf.py set-target esp32c6 build`. SDIO is the default transport on the C6 (CMD 18, CLK 19,
  D0-D3 20-23, streaming send, high speed). Only its `dependencies.lock` is kept here.

Built with ESP-IDF v5.5.3: `idf.py set-target esp32p4 build` in `station/`. The images are
committed as `web/wasm/fw/public/p4-hosted-*.bin` and `c6-hosted-*.bin` (licences in
`hosted-NOTICE.txt`) and are the input of the `hosted_p4` golden:

    ../../target/release/esp32sim-p4 --flash-mb 2 \
      --bootloader station/build/bootloader/bootloader.bin \
      --ptable station/build/partition_table/partition-table.bin --app station/build/wifi_station.bin \
      --coproc c6:bootloader=slave/build/bootloader/bootloader.bin,ptable=slave/build/partition_table/partition-table.bin,app=slave/build/network_adapter.bin,otadata=slave/build/ota_data_initial.bin,stub=bb_init=0,elf=slave/build/network_adapter.elf \
      --wifi ssid=esp32sim,psk=esp32sim-pass --net none --max-seconds 4

The P4 resets the C6 on GPIO 54, waits 1.5 s, brings the SDIO card up (CMD52 I/O reset, CMD0,
CMD8, CMD5, CMD3, CMD7, the CCCR and the function's block size), opens the data path with a
slave interrupt, receives the slave's INIT event (chip esp32c6, capabilities, its firmware
version, which matches the host's and so prints no "Version mismatch"), sends the WiFi RPCs, and
takes a DHCP lease through the C6: `wifi station: got ip:10.0.2.15` at 3.77 s. The C6's console
follows on stdout with `[c6] ` in front; see ../../docs/esp32p4.md, "The co-processor".

`stub=bb_init=0` skips the C6 PHY's baseband calibration, as for every C6 WiFi run here
(../../docs/esp32c6.md); the golden gives its address, `0x420bac5c`, since it ships no ELF.
