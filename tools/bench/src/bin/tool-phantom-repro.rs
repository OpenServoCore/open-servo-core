//! Phantom RX-ring aliasing probe — write-length sweep.
//!
//! The bug: the 32-byte RX byte ring laps when a Write frame (10B header +
//! L payload + 2B CRC) is longer than the ring AND the consumer stalls. The
//! trailing CRC bytes overwrite the ring cell exactly 32 positions back.
//!
//! Prediction from the ring arithmetic: the CRC's first byte sits at frame
//! byte `10 + L`, which maps to ring cell `(10 + L) % 32`. When that cell
//! falls inside the staged payload region (cell >= 10) it corrupts control-
//! table offset `(10 + L) % 32 - 10`. So sweeping L should march the corrupt
//! offset: L=32 -> off 0, L=34 -> off 2, ... L=40 -> off 8; L<32 lands in the
//! header region and is invisible in the staged data.
//!
//! Run:  cargo run --release --bin tool-phantom-repro -- [BAUD] [ITERS]
//!   BAUD default 3000000   ITERS default 300

use anyhow::Result;
use bench::{Bus, BusArgs, build_write};
use dxl_protocol::types::Id;

const LINK_BASE: u16 = 0x023C;
const SENTINEL: u8 = 0xAA;
const DATA_START_IN_FRAME: usize = 10;
const RING: usize = 32;

fn frame_crc(id: Id, addr: u16, data: &[u8]) -> (u8, u8) {
    let f = build_write(id, addr, data).unwrap();
    let n = f.len();
    (f[n - 2], f[n - 1])
}

/// Predicted corrupt CT offset for a payload of length `l`, or None if the
/// CRC lands in the header region (invisible in staged data).
fn predicted_offset(l: usize) -> Option<usize> {
    let cell = (DATA_START_IN_FRAME + l) % RING;
    (cell >= DATA_START_IN_FRAME).then(|| cell - DATA_START_IN_FRAME)
}

fn main() -> Result<()> {
    let mut a = std::env::args().skip(1);
    let baud: u32 = a.next().map(|s| s.parse().unwrap()).unwrap_or(3_000_000);
    let iters: u32 = a.next().map(|s| s.parse().unwrap()).unwrap_or(300);

    let mut bus = Bus::start(BusArgs {
        port: None,
        target_baud: baud,
    })?;
    let id = Id::new(bus.id());
    println!(
        "chip {} @ {} baud, {iters} iters/length\n",
        bus.id(),
        bus.baud()
    );
    println!("  L   frame   CRC     predicted   observed (offset: value xNN)");
    println!("  --  -----   -----   ---------   ---------------------------");

    for l in [28usize, 30, 32, 34, 36, 38, 40] {
        let payload = vec![SENTINEL; l];
        let (crc_l, crc_h) = frame_crc(id, LINK_BASE, &payload);
        // (offset -> (count, value))
        let mut hist: std::collections::BTreeMap<usize, (u32, u8)> = Default::default();
        let mut hit_iters = 0u32;

        for _ in 0..iters {
            bus.write_register(id, LINK_BASE, &payload)?;
            let rb = bus.read_register(id, LINK_BASE, l as u16)?;
            let mut any = false;
            for (k, &b) in rb.iter().enumerate() {
                if b != SENTINEL {
                    any = true;
                    let e = hist.entry(k).or_insert((0, b));
                    e.0 += 1;
                    e.1 = b;
                }
            }
            if any {
                hit_iters += 1;
            }
        }

        let pred = match predicted_offset(l) {
            Some(o) => format!("+{o}"),
            None => "header".to_string(),
        };
        let obs = if hist.is_empty() {
            format!("clean ({hit_iters}/{iters})")
        } else {
            let parts: Vec<String> = hist
                .iter()
                .map(|(k, (c, v))| {
                    let tag = if *v == crc_l {
                        "=CRC_L"
                    } else if *v == crc_h {
                        "=CRC_H"
                    } else {
                        ""
                    };
                    format!("+{k}: {v:02x}{tag} x{c}")
                })
                .collect();
            format!("{}/{iters}  [{}]", hit_iters, parts.join(", "))
        };
        println!(
            "  {l:<2}  {:<5}   {crc_l:02x} {crc_h:02x}   {pred:<9}   {obs}",
            l + 12
        );
    }
    Ok(())
}
