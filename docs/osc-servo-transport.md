# osc servo transport — the deadline-pipelined receiver

Status: AGREED DESIGN (2026-07-08, A3 decided = carve-out). Describes the
transport as restored by the pre-FIFO revert and evolved by the CRC/framing
side quest (CRC-16/ARC LSB-first, no pad/parity rules, snapshot reads), then
specifies the three amendments that close the zero-gap defect, with
first-principles arguments that the amended design keeps the measured speed.
Companion pillars: `osc-native-protocol.md` (the wire), `driver-pattern.md`
(the layering). Code is authoritative; when this doc and the code disagree,
fix the doc.

Measured baselines this doc must explain and preserve (turnaround =
instruction wire-end → status break fall):

| build                | 1M ping | 1M read | 1M write | 3M ping | zero-gap hot loop |
|----------------------|---------|---------|----------|---------|-------------------|
| this base (pre-FIFO) | 37.4 µs | 32.3 µs | 31.2 µs  | 36.7 µs | 0/200 (defect)    |
| FIFO band (reverted) | 50.8 µs | 58.7 µs | 105.6 µs | 56.3 µs | 300/300           |

The goal of the amendments: the left columns of row 1 with the right column
of row 2.

## 1. Architecture in one paragraph

All byte movement is hardware. The CPU never receives or transmits a byte:
DMA writes every received byte into a circular ring, DMA streams every reply
out of buffers and the control table, and a DMA-fed SPI engine computes every
CRC. Software is three pure state machines (`Framer`, `Chain`, `TxEngine`)
composed by `ServoBus`, driven by exactly two interrupt vectors at one
priority, whose only jobs are to compute two kinds of numbers — *where*
frames sit in the ring, and *when* something is due — and to react to two
kinds of events: "a break happened" (USART framing error) and "it is time
now" (one tick comparator, multiplexed over every deadline the transport
has).

## 2. Hardware ledger

Every hardware resource the transport touches, and its duty cycle:

| resource        | role                                             | budget/event |
|-----------------|--------------------------------------------------|--------------|
| USART1          | half-duplex wire; HDSEL, no self-echo (F9)       | —            |
| USART1 vector   | PFIC HIGH. (a) FE/RX-error = break/garble event; (b) TC = TX arm drained | FE body ~1–2 µs + settle spin; TC body ~2–4 µs/arm |
| SysTick CNT/CMP | the transport clock (48 MHz, 32-bit) + the ONE comparator | — |
| SysTick vector  | PFIC HIGH. Deadline mux: framer A/B, covered, chain trigger, rescue confirm | arithmetic slots ~1–5 µs; **dispatch runs here in this base — see §6** |
| DMA1 CH5        | USART1 RX → 512 B ring, circular, silent (no IRQ) | zero CPU |
| DMA1 CH4        | TX arms → USART1 DR (header, snapshot payload, CRC tail) | zero CPU; TC surfaces as USART TC |
| DMA1 CH3        | CRC feeds → SPI1 DR, 16-bit halfwords            | zero CPU, ~0.36 µs/B engine time |
| SPI1            | CRC-16/ARC coprocessor (16-bit LSB-first, bitrev16 at the register), accumulates across feeds | runs ~8× wire speed (F6) |
| DMA1 CH6/CH7    | snapshot copies → the 256 B snapshot buffer (payloads + linearized RX spans); VERYHIGH, above CH3/CH4 | ~0.125 µs/B, zero CPU |
| DMA1 CH1 vector | PFIC LOW: ADC sample set complete → motor kernel tick | ~10 µs body |
| PC0 CNF         | drive discipline: open-drain listening / push-pull TX window | flipped at trigger/release |
| main loop       | deferred reboot poll only                        | cold path |

PFIC preemption is two-level (IPRIOR bit 7). USART1 + SysTick share HIGH and
therefore serialize against each other; the kernel sits alone at LOW. Free
and reserved: TIM2 (reserved: future encoders), TIM1 (motor PWM), I2C1
vectors 30/31 (unused peripheral — usable as software-pended wake targets).

## 3. One exchange, tick by tick (ping at 1M; byte-time = 10 µs)

```
t=0    break's wire end. DMA has already ringed the 0x00. FE ISR (~2 µs):
       framer anchors at the ring position, schedules deadline A =
       anchor_tick + 3 byte-times + ½ byte slack; arms rescue candidacy.
t=10/20/30  ID, LEN, INST land in the ring by DMA. CPU idle.
t=35   deadline A (SysTick): header parse + validate → footprint 6, frame
       end computable. A ping's CRC-covered span IS its header, so the
       covered checkpoint fires from this same wake:
         · CRC feed of the covered span starts (CH3 arm, ~1 µs of CPU)
         · SPECULATION: decode + dispatch run NOW, the reply is built and
           staged into the TX engine — all before the frame has ended.
       Deadline B armed at packet_end estimate + 5 µs slack.
t=40/50  the two wire-CRC bytes land. SPI engine finishes the covered span
       long before (4 B ≈ 1.5 µs). CPU idle again.
t≈56   deadline B (SysTick): poll CRC result (ready) == wire CRC → commit;
       chain sequences the staged reply: trigger due at packet_end + T_turn
       (2 byte-times). Body is a few µs — the work already happened.
t=71   trigger (SysTick): INST finalized, PC0 → push-pull, break sent, first
       DMA arm armed. Status break falls. TX CRC is computed by the same SPI
       engine IN PARALLEL with transmission and patched into the final arm.
t=…    per-arm TC ISRs stream the remaining arms; final TC releases the wire
       and applies any deferred id/baud config.
```

Measured: 37.4 µs from instruction end to status break fall. The estimate
chain above accounts for ~21 µs of it (T_turn grid + slack); the remainder
is trigger-body, SBK commit, and ISR-entry overheads.

## 4. Why this is fast — first principles

1. **Work hides under wire time.** The speculation window (covered
   checkpoint → frame end) runs decode + dispatch + reply build while the
   last two CRC bytes are still in flight. Deadline B — the only step on the
   reply's critical path — is reduced to "poll a finished CRC and arm the
   trigger". The reply rides the T_turn grid, not the dispatch time.
2. **Zero hops.** Every pipeline stage runs at HIGH; stages hand off by
   falling through within one ISR invocation (the deadline drain loop) or by
   the next event's entry. There is no cross-priority handoff, no wake
   round-trip, no critical-section toll. (The FIFO band's 13–20 µs
   regression was almost entirely this, multiplied by 2–3 hops per frame.)
3. **Speculation is almost always right.** It bets that the frame will pass
   CRC. On a clean bus that is ~100% of frames; the loser (a corrupted
   frame) pays a revert, which is off the happy path by definition.
4. **Hardware CRC in both directions, in parallel.** RX: fed at the covered
   checkpoint, chewing while the CRC bytes fly, polled at B. TX: fed per
   arm, patched into the trailing arm before DMA reaches it. The CPU never
   computes or waits a full CRC.
5. **Copy-once TX.** Reply payloads are DMA-snapshotted once (~0.125 µs/B,
   fire-and-forget) and streamed from the snapshot by both the wire and the
   CRC — snapshot-consistent reads for the price of one copy hidden under
   T_turn (§4.2 of the wire spec).
6. **One comparator, muxed.** Every deadline (framer A/B/covered, chain
   trigger, rescue) folds onto SysTick CMP with pend-on-past semantics; the
   drain loop consumes every slot due at the same wake.
7. **The single-context CRC engine needs no arbitration protocol.** TX
   generation and RX validation share one engine safely because both run at
   HIGH — ownership is serialized by the PFIC, for free.

## 5. The defect this base still has — staleness, not latency

Zero-gap bursts (`GWRITE COMMIT GREAD` back-to-back) lose frames: 0/200 on
every build of this base. The post-mortem, with the FIFO band's Step-0
probe evidence:

- Dispatch inside the SysTick ISR can run long (GWRITE measured 69 µs even
  with the flat table). While it runs, following breaks pend, and two
  pended breaks COALESCE into one PFIC entry.
- The late FE handler then samples `ring.cursor()` and classifies the event
  by `ring[cursor-1]`. The cursor has moved past the break byte — the test
  reads frame DATA, the framer (in HUNT) ignores the "garble", and a fully
  intact frame in the ring is never anchored. Silent loss: no counter moves.
- The same failure survives ANY queueing fix that stores cursor snapshots:
  the FIFO band's residual 3M losses (~2–4%) were reproduced 1:1 by a probe
  counting garble-classified captures — the snapshot was already stale at
  capture time (FE entry delay ≥ 1 byte-time at 3.33 µs/byte).

**The lesson, as a design rule: position is derived from the stream, never
sampled at ISR entry.** Latency merely amplifies staleness; remove the
staleness and latency stops meaning loss. This is what the amendments do —
the FIFO (which attacked latency and kept the snapshots) attacked the wrong
term, which is why it was reverted.

## 6. Amendments

### A1 — `last_break_tick`: one timestamp, no snapshots

The FE handler records exactly one thing: the tick of the most recent FE
event (plus its existing bounded wire work: staged-reply kill, chain
suspend, rescue-candidacy arming). No cursor sample for framing — the
FE→DMA settle spin deletes with it. Rationale: only the NEWEST in-flight
frame
ever needs a wall-clock anchor (its deadlines A/covered/B are estimates
against its break time); every older frame is complete in the ring and
needs no clock at all (A2). A garble FE overwriting the timestamp makes an
estimate later = safer, never earlier. Cursor reads survive only at
provably-quiet bootstrap moments: boot (position 0 by construction) and
rescue confirm (a held-low line delivers no start edges — the cursor is
still).

### A2 — successor-available discrimination: the ring is the queue

Frames are contiguous: `next anchor = anchor + footprint` (stream
continuity). The cursor is read only at the cold bootstrap (boot / rescue —
moments when the wire is provably quiet and the ring position is defined;
there is no ring rearm anymore). From then on:

- **Fast path (successor available).** If newer activity exists beyond the
  current frame's computed end — its end byte is ringed and more followed —
  then every byte of this frame is already present. Resolve it purely from
  ring data: header, covered, CRC, dispatch, in one pass, no deadlines. A
  backlog of N frames is walked N frames at a time with zero scheduling;
  the 512 B ring (≥ 15 hot-loop frames) is the queue, and it cannot "drop"
  an entry the way a snapshot queue can.
- **Scheduled path (newest frame only).** The frame still arriving keeps
  today's deadline pipeline unchanged: A → covered (speculate) → B, timed
  from `last_break_ts`.
- Classification falls out: an FE while the current frame is short of its
  end and no fresh anchor position validates is mid-frame garble by
  position, not by a `cursor-1` byte value. The Step-0 failure class
  becomes unwritable.
- Corruption recovery: a corrupted LEN mis-strides the ladder; the next
  resolution finds no `0x00` at the expected anchor (or the frame fails
  CRC) → drop + count, and the ladder re-verifies at every following
  boundary — anchors are parity-free (§3.2 side quest) so recovery needs
  no reload, only the next break's arrival. Loss is bounded to ≤ 2 frames
  per corruption event; the host's timeout+retry contract closes the loop.

### A3 — where dispatch runs: the interleave decision

Policy (locked): a live request always beats the motor kernel; between
queued requests a pending kernel tick gets its slot
(request → kernel → request). PFIC arbitration serves equal-priority pends
lowest-vector-first: SW=14 beats the kernel's DMA1_CH1=22, which beats
I2C1_EV=30. Two placements satisfy A1/A2; they differ on this policy and on
trigger jitter:

**(a) Full pre-FIFO: dispatch stays at HIGH.** Zero hops, the §3 timeline
verbatim. Costs: the kernel is preempted for every dispatch (69 µs worst
measured — at FOC-tier loop rates that is real control jitter, and the
interleave policy is unimplementable at HIGH); a due reply/chain trigger
waits out any running dispatch body (the old soft-timing tail, up to
~70 µs on the wrong day).

**(b) Cost carve-out: heavy phases hop to LOW, timing stays at HIGH.**
HIGH keeps everything bounded: FE body, deadline arithmetic, framer
resolution (ladder walks + header parse), covered-checkpoint CRC feed,
verify + tail fold at the end, commit (a staged-write apply), reply
sequencing + trigger, TX arms, and snoop handling (CRC-free since the side
quest — pure chain arithmetic) — each ≤ ~5 µs. The covered checkpoint
hands the frame to the consumer instead of dispatching inline; decode +
dispatch + reply build (including the snapshot kick) run at LOW; the end
deadline at HIGH verifies the CRC and sequences whatever the consumer
staged, elastically if the consumer is still working. Wake policy encodes
the interleave: fresh live-edge work pends SW (14) — a live request beats
the kernel; a backlog frame handed while the consumer was busy pends
I2C1_EV (30) — a pending kernel tick gets its slot between queued
requests. The handoff is one work slot each way under a register-scale
critical section (backpressure — the framer holds position until the
consumer frees the slot; the ring absorbs the backlog per A2). The CRC
engine is still touched only at HIGH: no arbitration protocol.

First-principles cost of (b) vs (a): one pend+entry per frame (~4 µs) paid
inside the speculation window, where it is hidden if
`hop + decode + dispatch ≤ covered→B window + T_turn`:

| frame     | window+T_turn 1M | hop+work (est.) | grid held? | window+T_turn 3M | grid held? |
|-----------|------------------|-----------------|------------|-------------------|------------|
| ping      | 45 µs            | ~8 µs           | yes        | 18 µs             | yes        |
| read 4 B  | 45 µs            | ~13 µs          | yes        | 18 µs             | marginal   |
| write 16 B (noreply) | n/a (no grid) | ~15 µs | n/a        | n/a               | n/a        |
| GREAD 4 B | 45 µs            | ~15 µs          | yes        | 18 µs             | marginal   |

Misses degrade elastically (reply leaves when ready, a few µs past the
grid), they do not fail. Expected turnaround: (b) ≈ (a) at 1M; (b) within
~0–8 µs of (a) at 3M — against (a)'s unbounded kernel preemption and
trigger tail. **DECIDED: (b)** (Aaron, 2026-07-08).

## 7. First-principles losslessness (the zero-gap argument)

1. Capture: every byte is DMA'd regardless of CPU state; breaks pend and
   may coalesce, but A2 derives frame positions from the stream, so a
   coalesced or delayed FE costs nothing — the fast path recovers every
   completed frame from data.
2. Backlog: bounded by the ring (512 B ≈ 15 minimal hot-loop frames). The
   consumer outruns the wire (~2× worst case today, improving with decode
   work); sustained overrun is a THROUGHPUT invariant, pinned by a flood
   test (DES + bench: sustained zero-gap minimal frames at 3M, assert zero
   loss), not by a runtime guard.
3. Ordering: the single staged-reply slot + in-order ring walk preserve
   frame order; a newer instruction superseding a staged, not-yet-streaming
   reply is unchanged from this base.
4. Acceptance gates (all pre-existing assets): the DES zero-gap suite
   (rebuilt against the composite API), `tool-osc-burst` hot-loop and plain
   matrices at 1M and 3M ≥ 300/300, turnaround regression ≤ baseline table
   row 1, and the diag counters accounting for every intentional drop.

## 8. Considered alternatives (recorded so they are not re-derived)

- **Capture FIFO / deferred dispatch (the reverted band):** fixed latency,
  kept the stale snapshots (the actual root cause), cost 13–20 µs/exchange
  in hops plus a cross-context CRC arbitration protocol. Reverted 2026-07-08;
  forensics in the band memory and `backup/deferred-dispatch-band`.
- **Break-bounded frames (closing double-break):** deletes every receiver
  clock and all speculation; +2.6 byte-times per burst and reply; gives up
  pre-end overlap (≈ +10 µs at 1M). Rejected in favor of keeping
  speculation. Revisit only if a clock-free receiver becomes worth more
  than the overlap (e.g. a minimal bridge implementation).
- **Covered-position break (payload/CRC boundary):** wire-native
  speculation trigger, but resurrects the commit/revert machinery it was
  meant to delete. Rejected.
- **Walker without LEN:** impossible — the ring carries no in-band break
  marker (a break DMAs as 0x00, same as payload zeros), and both cursor
  samples and FE-interval timing are unsound position sources. LEN stays,
  demoted to a stream stride that the next break audits.
- **Sync bytes / header checksum / parity bit:** each adds wire cost to
  buy detection the break + CRC + host-retry contract already provide;
  none deletes receiver machinery. Break framing is the strong form of
  sync — DXL's `FF FF FD` + stuffing is the workaround for not having an
  out-of-band delimiter at all.

## 9. Glossary

- **break** — line held low ≥ a byte-time; the out-of-band frame
  delimiter. Rings as one 0x00 byte, raises FE.
- **garble** — an FE that is not a break: a corrupted byte (slot occupied,
  CRC will fail) or a phantom byte (noise-invented byte between frames).
- **anchor / footprint** — a frame's start index in the ring / its total
  ring length including the break byte (always even).
- **covered span** — everything the CRC protects: the frame minus its two
  trailing CRC bytes.
- **covered checkpoint** — the moment the covered span is fully ringed
  (2 byte-times before frame end); the speculation trigger.
- **speculation** — decode + dispatch + reply build performed before the
  CRC verdict; committed on pass, reverted on fail.
- **deadline A / B** — header-readable check / frame-end check, both
  cursor-verified estimates from the break timestamp.
- **T_turn** — the mandated 2-byte-time wire gap between a frame and its
  reply; the reply grid.
- **fast path / scheduled path** — A2's split: complete-in-ring frames
  resolved from data with no clock / the newest frame timed by deadlines.
