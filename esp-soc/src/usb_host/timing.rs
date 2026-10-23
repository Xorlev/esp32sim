//! Per-command times on the host's clock, and the latency distribution they make: what a recorder
//! sees of each SCSI command it sends, from submission (the CBW about to go out) to completion (its
//! CSW read and checked, or its failure and any recovery done), in emulated nanoseconds.

/// One command as the host timed it (`msc::Msc::times`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CmdTime {
    /// the SCSI operation code
    pub op: u8,
    /// a READ(10) or WRITE(10)'s first block and block count; 0 for other commands
    pub lba: u32,
    pub blocks: u16,
    pub submit_ns: u64,
    pub done_ns: u64,
    /// ended in a CSW with good status
    pub ok: bool,
}

impl CmdTime {
    /// A command from its CDB (SBC-3 READ(10), WRITE(10): LBA in bytes 2-5, count in 7-8).
    pub fn new(cdb: &[u8], submit_ns: u64, done_ns: u64, ok: bool) -> Self {
        let rw = matches!(cdb.first(), Some(&(super::msc::op::READ10 | super::msc::op::WRITE10))) && cdb.len() >= 9;
        let (lba, blocks) = if rw { (u32::from_be_bytes([cdb[2], cdb[3], cdb[4], cdb[5]]), u16::from_be_bytes([cdb[7], cdb[8]])) } else { (0, 0) };
        CmdTime { op: cdb.first().copied().unwrap_or(0), lba, blocks, submit_ns, done_ns, ok }
    }
    pub fn latency_ns(&self) -> u64 { self.done_ns.saturating_sub(self.submit_ns) }
}

/// A set of latencies, sorted, with order statistics by nearest rank.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Latency { sorted: Vec<u64> }

/// Histogram bucket edges for write latencies: 1, 2, 3, 4, 5, 6, 8, 10, 15, 20, 50 and 100 ms.
pub const MS_EDGES: [u64; 12] = [1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000, 6_000_000, 8_000_000, 10_000_000, 15_000_000, 20_000_000, 50_000_000, 100_000_000];

impl Latency {
    pub fn new(mut ns: Vec<u64>) -> Self { ns.sort_unstable(); Latency { sorted: ns } }
    pub fn len(&self) -> usize { self.sorted.len() }
    pub fn is_empty(&self) -> bool { self.sorted.is_empty() }
    /// The smallest value with at least `p` percent of the set at or below it; 0 when empty.
    pub fn percentile(&self, p: f64) -> u64 {
        if self.sorted.is_empty() { return 0; }
        let rank = (p / 100.0 * self.sorted.len() as f64).ceil() as usize;
        self.sorted[rank.clamp(1, self.sorted.len()) - 1]
    }
    pub fn min(&self) -> u64 { self.sorted.first().copied().unwrap_or(0) }
    pub fn max(&self) -> u64 { self.sorted.last().copied().unwrap_or(0) }
    pub fn median(&self) -> u64 { self.percentile(50.0) }
    pub fn mean(&self) -> f64 { if self.sorted.is_empty() { 0.0 } else { self.sorted.iter().map(|&v| v as f64).sum::<f64>() / self.sorted.len() as f64 } }
    /// Counts below `edges[0]`, in each [edges[i], edges[i + 1]), and at or above the last edge:
    /// `edges.len() + 1` buckets. `edges` ascends.
    pub fn histogram(&self, edges: &[u64]) -> Vec<usize> {
        let mut at = 0;
        let mut out = Vec::with_capacity(edges.len() + 1);
        for &e in edges {
            let n = self.sorted.partition_point(|&v| v < e);
            out.push(n - at);
            at = n;
        }
        out.push(self.sorted.len() - at);
        out
    }
    /// `n`, min, median, p90, p99 and max in milliseconds.
    pub fn summary(&self) -> String {
        let ms = |v: u64| v as f64 / 1e6;
        format!("n={} min {:.2} median {:.2} p90 {:.2} p99 {:.2} max {:.2} ms", self.len(), ms(self.min()), ms(self.median()), ms(self.percentile(90.0)), ms(self.percentile(99.0)), ms(self.max()))
    }
    /// The histogram over `edges` as `<1ms:n 1-2ms:n ... >=100ms:n`, empty buckets left out.
    pub fn histogram_line(&self, edges: &[u64]) -> String {
        let ms = |v: u64| format!("{}", v as f64 / 1e6);
        let h = self.histogram(edges);
        let label = |i: usize| match i {
            0 => format!("<{}ms", ms(edges[0])),
            i if i == edges.len() => format!(">={}ms", ms(edges[i - 1])),
            i => format!("{}-{}ms", ms(edges[i - 1]), ms(edges[i])),
        };
        h.iter().enumerate().filter(|(_, &n)| n > 0).map(|(i, n)| format!("{}:{n}", label(i))).collect::<Vec<_>>().join(" ")
    }
}

/// How much of [`from`, `to`) the `intervals` cover, each a [start, end) in nanoseconds. They are
/// sorted by start and do not overlap one another, as the transfers of one bus do.
pub fn overlap_ns(intervals: &[(u64, u64)], from: u64, to: u64) -> u64 {
    let first = intervals.partition_point(|&(_, end)| end <= from);
    intervals[first..].iter().take_while(|&&(start, _)| start < to).map(|&(s, e)| e.min(to).saturating_sub(s.max(from))).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb_host::msc::op;

    #[test]
    fn a_command_keeps_the_lba_and_count_of_a_read_or_write() {
        let w = CmdTime::new(&[op::WRITE10, 0, 0, 1, 0x86, 0xa0, 0, 0, 128, 0], 1_000, 4_500, true);
        assert_eq!((w.op, w.lba, w.blocks, w.latency_ns()), (op::WRITE10, 100_000, 128, 3_500));
        let r = CmdTime::new(&[op::READ10, 0, 0xff, 0, 0, 1, 0, 0x01, 0x00, 0], 0, 0, false);
        assert_eq!((r.lba, r.blocks, r.ok), (0xff00_0001, 256, false));
        let t = CmdTime::new(&[op::TEST_UNIT_READY, 0, 0, 0, 0, 0], 7, 9, true);
        assert_eq!((t.op, t.lba, t.blocks, t.latency_ns()), (op::TEST_UNIT_READY, 0, 0, 2));
    }

    /// Nearest rank: the p-th percentile is the value at rank ceil(p/100 x n).
    #[test]
    fn percentiles_by_nearest_rank() {
        let l = Latency::new((1..=100).rev().collect());
        assert_eq!((l.min(), l.median(), l.percentile(90.0), l.percentile(99.0), l.max()), (1, 50, 90, 99, 100));
        assert_eq!(l.percentile(0.0), 1, "the 0th is the minimum");
        assert_eq!(l.mean(), 50.5);
        let one = Latency::new(vec![7]);
        assert_eq!((one.median(), one.percentile(99.0)), (7, 7));
        let odd = Latency::new(vec![30, 10, 20]);
        assert_eq!((odd.median(), odd.percentile(90.0)), (20, 30));
        assert_eq!(Latency::default().percentile(99.0), 0);
        assert_eq!(Latency::new(vec![1_500_000, 2_000_000, 3_250_000, 9_000_000]).summary(), "n=4 min 1.50 median 2.00 p90 9.00 p99 9.00 max 9.00 ms");
    }

    #[test]
    fn the_histogram_counts_each_value_once_at_its_lower_edge() {
        let ms = |v: u64| v * 1_000_000;
        let l = Latency::new(vec![ms(0), 999_999, ms(1), ms(2) - 1, ms(5), ms(100), ms(250)]);
        let h = l.histogram(&MS_EDGES);
        assert_eq!(h.len(), MS_EDGES.len() + 1);
        assert_eq!(h.iter().sum::<usize>(), l.len());
        assert_eq!((h[0], h[1], h[5], h[12]), (2, 2, 1, 2), "below 1 ms; [1, 2) ms; [5, 6) ms; at or above 100 ms");
        assert_eq!(l.histogram_line(&MS_EDGES), "<1ms:2 1-2ms:2 5-6ms:1 >=100ms:2");
        assert_eq!(Latency::default().histogram(&[10]), [0, 0]);
    }

    #[test]
    fn overlap_cuts_each_interval_to_the_window() {
        let busy = [(10, 20), (30, 40), (50, 100)];
        assert_eq!(overlap_ns(&busy, 0, 5), 0);
        assert_eq!(overlap_ns(&busy, 15, 35), 10, "the end of one and the start of the next");
        assert_eq!(overlap_ns(&busy, 0, 1000), 70);
        assert_eq!(overlap_ns(&busy, 20, 30), 0, "ends are exclusive");
        assert_eq!(overlap_ns(&busy, 60, 70), 10, "inside one");
        assert_eq!(overlap_ns(&[], 0, 10), 0);
    }
}
