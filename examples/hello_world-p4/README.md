# IDF hello_world for the ESP32-P4

The ESP-IDF v5.5.3 `get-started/hello_world` example for the ESP32-P4 silicon rev < 3 (v1.x),
built with `idf.py set-target esp32p4 build`: single-core FreeRTOS from `sdkconfig.defaults`, and
both cores with `idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.defaults.smp build`. The bootloader,
partition table and app are committed as `web/wasm/fw/public/p4-hello-*` (single-core) and
`p4-hello-smp*` (the two cores; same partition table), the input of the `hello-p4` goldens.

    ../../target/release/esp32sim-p4 --flash-mb 2 \
      --bootloader build/bootloader/bootloader.bin --ptable build/partition_table/partition-table.bin \
      --app build/hello_world.bin --elf build/hello_world.elf --max-seconds 27

Expect `Hello world!`, the ten-second countdown and `rst:0xc (SW_CPU_RESET)` after each
`esp_restart()`. See ../../docs/esp32p4.md.
