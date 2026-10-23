# EX245: the C6's SDIO slave and the P4/C6 lock-step (`p4/06-coproc`)

Branch commit `dfaff360` (parts 1 to 6: the C6's SDIO slave, SDIO functions on the P4's host,
`--coproc`, the fixed network clock and the receive-ring count) against upstream `main`
`20117b08`, as [README.md](README.md) describes; cumulative. Run 2 compares it with part 5
(`ca2ee63a`) directly. This part changes the C6 and the shared WiFi path, so `c6-wifi-station`
(the `wifi_station_c6` golden's run) is measured too. Results: [06-coproc.json](06-coproc.json).

What the C6 pays without a host: the slave is an optional `device_set!` entry (ticked and asked
for deadlines only while active, which it never is here) with SLC0 among the interrupt sources,
plus one address compare after a peripheral write. The WiFi path gains the epoch test in SNTP
and the receive-ring count. Every run of a workload retired the same instructions and printed the
same console on both builds.

The runs, against main and against part 5, were made before the branch was reworded on
2026-10-22: identifiers, comments, log strings and documentation of part 5's USB host and its
tests changed, which none of these workloads runs, and the binary's layout moved with them.
`dfaff360` retires the same instructions and prints the same consoles (one run of each
workload); the times were not measured again.

## run 1, against main, started 2026-10-22T21:16:26-0600

| workload | main median (range) | branch median (range) | change | branch slower | instructions | console SHA-256 | 1-min load before / after |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| s3-hello | 0.40 (0.39-0.40) | 0.40 (0.40-0.41) | +0.00% | 3/7 | 11,990,488 + 6,798,360 | `119a1216fb34…` | 10.1 / 9.5 |
| c3-hello | 3.90 (3.88-3.96) | 3.95 (3.93-4.03) | +1.28% | 7/7 | 4,800,000,000 | `3b9d8d3eb053…` | 9.5 / 5.7 |
| c6-hello | 2.88 (2.87-2.89) | 2.91 (2.90-2.91) | +1.04% | 7/7 | 4,800,000,000 | `bd69fbb9ac5c…` | 5.7 / 5.3 |
| pocket-tank | 14.62 (14.58-15.03) | 15.03 (14.62-15.29) | +2.80% | 6/7 | 1,113,634,993 + 1,490,529,616 | `62b7af05631b…` | 5.3 / 4.2 |
| c6-wifi-station | 1.78 (1.77-1.83) | 1.79 (1.79-1.84) | +0.56% | 6/7 | 2,240,000,000 | `75b99af7d413…` | 4.2 / 4.9 |

## run 2, against p4/05, started 2026-10-22T21:23:26-0600

| workload | p4/05 median (range) | branch median (range) | change | branch slower | instructions | console SHA-256 | 1-min load before / after |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| c6-hello | 2.89 (2.89-2.90) | 2.92 (2.91-2.92) | +1.04% | 7/7 | 4,800,000,000 | `bd69fbb9ac5c…` | 5.1 / 5.3 |
| c6-wifi-station | 1.79 (1.78-1.79) | 1.80 (1.79-1.82) | +0.56% | 7/7 | 2,240,000,000 | `75b99af7d413…` | 5.3 / 5.9 |
| c3-hello | 4.01 (4.00-4.02) | 4.02 (4.02-4.03) | +0.25% | 6/7 | 4,800,000,000 | `3b9d8d3eb053…` | 5.9 / 5.5 |
| pocket-tank | 14.60 (14.26-14.69) | 14.58 (14.28-14.62) | -0.14% | 2/7 | 1,113,634,993 + 1,490,529,616 | `62b7af05631b…` | 5.5 / 4.1 |

**Flag: `c6-hello` is +1.04% slower in 7 of 7 pairs, against main and against part 5**, and
`c6-wifi-station` +0.56% (6/7 and 7/7): about 30 ms in 2.9 s and 10 ms in 1.8 s. A `sample`
profile of `c6-hello` on both builds (4 s each, about 3,000 samples) has the same functions on
top in the same order (`Bus::tick` 1,075 against 1,061 samples, `SocBus::next_deadline` 1,037
against 1,031, `run_unmodeled` 284 against 258) and nothing new among them; the slave's functions
do not appear. The cost is not attributed further.

Pocket Tank's +2.80% against main (slower 6/7) is not this part's: against part 5 it is −0.14%
(slower 2/7), and this part changes nothing the S3 runs. `c3-hello` against part 5 is one 10 ms
tick (+0.25%, 6/7); its 1-2% swings between builds are in [04-sdmmc.md](04-sdmmc.md).
