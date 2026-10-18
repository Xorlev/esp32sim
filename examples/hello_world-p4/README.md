# IDF hello_world for the ESP32-P4

The ESP-IDF v5.5.3 `get-started/hello_world` example for the ESP32-P4 silicon rev < 3 (v1.x),
single-core FreeRTOS, built with `idf.py set-target esp32p4 build` from `sdkconfig.defaults`. Its
bootloader, partition table and app are committed as `web/wasm/fw/public/p4-hello-*`, the input
of the `hello-p4` goldens.

    ../../target/release/esp32sim-p4 --flash-mb 2 \
      --bootloader build/bootloader/bootloader.bin --ptable build/partition_table/partition-table.bin \
      --app build/hello_world.bin --elf build/hello_world.elf --max-seconds 0.1

How far it gets, and why it stops there, is in ../../docs/esp32p4.md.
