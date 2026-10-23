# EX242: the ESP32-P4 chip crate (`p4/03-chip`)

Branch commit `4394cd37` (parts 1 to 3: the `esp32p4` crate, `--chip p4` and `esp32sim-p4`, the
browser machine, `Soc::check_image` and the rev ≥ 3 refusal) against upstream `main` `20117b08`,
as [README.md](README.md) describes; cumulative, parts 1 and 2 included. Results:
[03-chip.json](03-chip.json).

The chip is new: the other chips' paths gain only `Soc::check_image`, a default that accepts every
image and runs once per loaded file before the first instruction, and the CLI's new match arm.
Every run of a workload retired the same instructions and printed the same console on both
builds.

## run 1, started 2026-10-22T20:41:54-0600

| workload | main median (range) | branch median (range) | change | branch slower | instructions | console SHA-256 | 1-min load before / after |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| s3-hello | 0.40 (0.40-0.41) | 0.41 (0.40-0.41) | +2.50% | 4/7 | 11,990,488 + 6,798,360 | `119a1216fb34…` | 15.2 / 14.9 |
| c3-hello | 3.92 (3.91-4.01) | 3.92 (3.92-4.02) | +0.00% | 2/7 | 4,800,000,000 | `3b9d8d3eb053…` | 14.9 / 7.4 |
| c6-hello | 2.83 (2.82-2.84) | 2.84 (2.83-2.84) | +0.35% | 4/7 | 4,800,000,000 | `bd69fbb9ac5c…` | 7.4 / 4.5 |
| pocket-tank | 14.28 (13.94-15.25) | 14.16 (13.98-14.67) | -0.84% | 3/7 | 1,113,634,993 + 1,490,529,616 | `62b7af05631b…` | 4.5 / 4.4 |

Nothing is flagged. S3 `hello` differs by one 10 ms tick (0.41 s against 0.40 s, slower 4/7);
`c3-hello`, flagged after parts 1 and 2, measures +0.00% here (slower 2/7), so that difference
moves with the build and not with the code the C3 runs. The first workload started right after a
test build (load 15).
