use core::cell::SyncUnsafeCell;

use ch32_metapac::USART1;
use dxl_protocol::crc16_continue;
use osc_core::SnoopWindow;

use crate::hal::{dma, gpio, systick, usart};
use crate::statics::{DXL_RX_BUF, DXL_RX_BUF_LEN, DXL_TX_BUF, DXL_TX_EN};

#[derive(Copy, Clone, PartialEq, Eq)]
enum Stage {
    Idle,
    WaitingPlain,
    WaitingSwitch,
    WaitingFire,
}

struct State {
    stage: Stage,
    fire_tick: u32,
    snoop_head: u16,
    bulk_crc: u16,
}

static STATE: SyncUnsafeCell<State> = SyncUnsafeCell::new(State {
    stage: Stage::Idle,
    fire_tick: 0,
    snoop_head: 0,
    bulk_crc: 0,
});

const _: () = assert!(DXL_RX_BUF_LEN.is_power_of_two() && DXL_RX_BUF_LEN <= u16::MAX as usize + 1);
const RX_MASK_U16: u16 = (DXL_RX_BUF_LEN as u16).wrapping_sub(1);
const RX_MASK_U32: u32 = (DXL_RX_BUF_LEN - 1) as u32;

/// Configures TX DMA + TC IRQ from the current `DXL_TX_BUF`. Does NOT flip
/// TX_EN or enable the DMA channel — `fire_now` does both at the slot deadline
/// so the bus stays in RX during any preceding snoop window.
pub fn arm_tx() -> bool {
    // SAFETY: caller has &mut Ch32DxlIo (sole writer to DXL_TX_BUF).
    let len = unsafe { (*DXL_TX_BUF.get()).as_slice().len() };
    if len == 0 {
        return false;
    }
    dma::set_count(dma::Channel::CH4, len as u16);
    usart::clear_tc(USART1);
    usart::set_dma_tx(USART1, true);
    usart::set_tc_irq(USART1, true);
    true
}

pub fn fire_now() {
    // SAFETY: written once during bring-up before USART1 IRQ unmask.
    if let Some(t) = unsafe { *DXL_TX_EN.get() } {
        gpio::set_level(t.pin, t.tx_level);
    }
    dma::enable(dma::Channel::CH4);
}

pub fn start_plain_after(idle_tick: u32, delay_us: u32) {
    systick::set_irq(false);
    systick::clear_match();

    let needed = delay_us.saturating_mul(systick::TICKS_PER_US);

    if systick::ticks().wrapping_sub(idle_tick) >= needed {
        if arm_tx() {
            fire_now();
        }
        return;
    }
    if !arm_tx() {
        return;
    }

    set_state(Stage::WaitingPlain, idle_tick.wrapping_add(needed), 0, 0);
    systick::set_cmp(idle_tick.wrapping_add(needed));
    systick::set_irq(true);

    // CNTIF latches only on CNT==CMP up-count; if CNT crossed the deadline
    // before CMP was written, the next match is ~89 s away on wrap.
    if systick::ticks().wrapping_sub(idle_tick) >= needed {
        on_systick();
    }
}

pub fn start_fast_after(idle_tick: u32, fire_us: u32, snoop: Option<SnoopWindow>) {
    systick::set_irq(false);
    systick::clear_match();

    if !arm_tx() {
        set_stage(Stage::Idle);
        return;
    }

    let fire_needed = fire_us.saturating_mul(systick::TICKS_PER_US);
    let fire_tick = idle_tick.wrapping_add(fire_needed);

    let Some(snoop) = snoop else {
        // Only slot — no predecessors to snoop, fire directly.
        set_state(Stage::WaitingFire, fire_tick, 0, 0);
        systick::set_cmp(fire_tick);
        systick::set_irq(true);
        if systick::ticks().wrapping_sub(idle_tick) >= fire_needed {
            on_systick();
        }
        return;
    };

    let snoop_head = (snoop.rx_start & RX_MASK_U32) as u16;
    let switch_needed = snoop.open_us.saturating_mul(systick::TICKS_PER_US);
    let switch_tick = idle_tick.wrapping_add(switch_needed);
    set_state(Stage::WaitingSwitch, fire_tick, snoop_head, 0);
    systick::set_cmp(switch_tick);
    systick::set_irq(true);
    if systick::ticks().wrapping_sub(idle_tick) >= switch_needed {
        on_systick();
    }
}

pub fn on_systick() {
    systick::clear_match();
    systick::set_irq(false);

    // SAFETY: SysTick and USART1 share PFIC priority — neither preempts the
    // other, so STATE access from this ISR is uncontested w.r.t. on_rxne.
    let stage = unsafe { (*STATE.get()).stage };
    match stage {
        Stage::Idle => {}
        Stage::WaitingPlain => {
            set_stage(Stage::Idle);
            fire_now();
        }
        Stage::WaitingSwitch => {
            // Bulk-CRC every byte landed since the request IDLE (slots 0..N-3
            // and any prefix of N-2 already arrived); then enable RXNEIE so
            // remaining N-2 bytes accumulate one-by-one until FIRE.
            accumulate_snoop();
            usart::set_rxne_irq(USART1, true);
            // SAFETY: see top-of-fn note.
            let fire_tick = unsafe {
                let s = &mut *STATE.get();
                s.stage = Stage::WaitingFire;
                s.fire_tick
            };
            systick::set_cmp(fire_tick);
            systick::set_irq(true);
            if (systick::ticks().wrapping_sub(fire_tick) as i32) >= 0 {
                on_systick();
            }
        }
        Stage::WaitingFire => {
            usart::set_rxne_irq(USART1, false);
            // Drain any byte that landed between the last RXNE and now.
            accumulate_snoop();
            // Enable TX FIRST — jitter cap is one byte time (3.33 µs at
            // 3 Mbaud), CRC compute below races the DMA pre-fetch with
            // (payload_end - 1) byte-times of slack.
            set_stage(Stage::Idle);
            fire_now();
            patch_crc();
        }
    }
}

pub fn on_rxne() {
    // SAFETY: see on_systick.
    let stage = unsafe { (*STATE.get()).stage };
    if stage != Stage::WaitingFire {
        return;
    }
    accumulate_snoop();
}

pub fn cancel() {
    set_stage(Stage::Idle);
}

fn patch_crc() {
    // SAFETY: see on_systick. Sole writer to DXL_TX_BUF in ISR context.
    unsafe {
        let buf = &mut *DXL_TX_BUF.get();
        let n = buf.len();
        let payload_end = n.saturating_sub(2);
        let seed = (*STATE.get()).bulk_crc;
        let crc = crc16_continue(seed, &buf[..payload_end]);
        let bytes = crc.to_le_bytes();
        let slice = buf.as_mut_slice();
        slice[n - 2] = bytes[0];
        slice[n - 1] = bytes[1];
    }
}

fn accumulate_snoop() {
    let write_pos = current_rx_write_pos();
    // SAFETY: STATE accessed only from SysTick/USART1 ISRs (no mutual
    // preemption); ring read is non-aliasing with DMA writes ahead of
    // write_pos.
    unsafe {
        let s = &mut *STATE.get();
        if s.snoop_head == write_pos {
            return;
        }
        let ring = &*DXL_RX_BUF.get();
        s.bulk_crc = ring_crc(s.bulk_crc, ring, s.snoop_head, write_pos);
        s.snoop_head = write_pos;
    }
}

fn ring_crc(seed: u16, ring: &[u8], head: u16, write_pos: u16) -> u16 {
    if head == write_pos {
        return seed;
    }
    let start = head as usize;
    let end = write_pos as usize;
    if start < end {
        crc16_continue(seed, &ring[start..end])
    } else {
        let mid = crc16_continue(seed, &ring[start..]);
        crc16_continue(mid, &ring[..end])
    }
}

fn current_rx_write_pos() -> u16 {
    let remaining = dma::remaining(dma::Channel::CH5);
    (DXL_RX_BUF_LEN as u16).wrapping_sub(remaining) & RX_MASK_U16
}

fn set_stage(stage: Stage) {
    // SAFETY: caller is either main loop (called before any ISR can race)
    // or an ISR (cannot be preempted by the other STATE-touching ISR).
    unsafe { (*STATE.get()).stage = stage };
}

fn set_state(stage: Stage, fire_tick: u32, snoop_head: u16, bulk_crc: u16) {
    // SAFETY: see set_stage.
    unsafe {
        let s = &mut *STATE.get();
        s.stage = stage;
        s.fire_tick = fire_tick;
        s.snoop_head = snoop_head;
        s.bulk_crc = bulk_crc;
    }
}

#[cfg(test)]
mod tests {
    use super::ring_crc;
    use dxl_protocol::crc16;

    const RING: [u8; 8] = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];

    #[test]
    fn empty_walk_returns_seed_unchanged() {
        assert_eq!(ring_crc(0x1234, &RING, 3, 3), 0x1234);
        assert_eq!(ring_crc(0, &RING, 0, 0), 0);
    }

    #[test]
    fn non_wrap_walk_matches_contiguous_crc() {
        // head=2..write_pos=5 covers bytes [0x30, 0x40, 0x50].
        assert_eq!(ring_crc(0, &RING, 2, 5), crc16(&[0x30, 0x40, 0x50]));
    }

    #[test]
    fn wrap_walk_stitches_tail_and_head() {
        // head=6, write_pos=2 covers [0x70, 0x80, 0x10, 0x20] across the wrap.
        assert_eq!(ring_crc(0, &RING, 6, 2), crc16(&[0x70, 0x80, 0x10, 0x20]));
    }

    #[test]
    fn seed_chaining_matches_single_pass() {
        // Walking 1..4 then 4..7 must equal a single walk over 1..7.
        let mid = ring_crc(0, &RING, 1, 4);
        let chained = ring_crc(mid, &RING, 4, 7);
        assert_eq!(chained, ring_crc(0, &RING, 1, 7));
    }

    #[test]
    fn seed_chaining_survives_wrap_boundary() {
        // Tail-of-ring then head-of-ring must equal one wrap-spanning walk.
        let pre = ring_crc(0, &RING, 5, 0);
        let post = ring_crc(pre, &RING, 0, 3);
        assert_eq!(post, ring_crc(0, &RING, 5, 3));
    }
}
