# osc servo transport — the deadline-pipelined receiver

Status: A1 + A2 LANDED (2026-07-08, `84680bd2`); A3(b) LANDED (2026-07-08,
`c83f6306`/`0c60bfda`) with the U2c silicon fixes (`db75cff7` FE-promise
wait, `a19d9377` RX-flag park — see §6 A4). Describes the transport as
restored by the pre-FIFO revert and evolved by the CRC/framing side quest
(CRC-16/ARC LSB-first, no pad/parity rules, snapshot reads), then the three
amendments that close the zero-gap defect, with first-principles arguments
that the amended design keeps the measured speed. Companion pillars:
`osc-native-protocol.md` (the wire), `driver-pattern.md` (the layering).
Code is authoritative; when this doc and the code disagree, fix the doc.

Measured turnarounds (instruction wire-end → status break fall; read = 16 B,
write = goal_position 4 B; flash-layout swings between builds are ±5 µs):

| build                | 1M ping | 1M read | 1M write | 3M ping | zero-gap hot loop |
|----------------------|---------|---------|----------|---------|-------------------|
| pre-FIFO base        | 31.5 µs | 36.7 µs | 83.3 µs  | —       | 0/200 (defect)    |
| FIFO band (reverted) | 50.8 µs | 58.7 µs | 105.6 µs | 56.3 µs | 300/300           |
| A1+A2 landed         | 38.6 µs | 45.7 µs | 93.3 µs  | 41.9 µs | 300/300           |
| A3(b) landed         | 35.4 µs | 41.6 µs | 84.5 µs  | 58.4 µs | 2400/2400         |

A3(b) same-day 3M row: read-16 66.6 µs, write 113.7 µs. At 1M every column
improved 3–9 µs over A1+A2 (dispatch left the deadline bodies). At 3M the
carve-out costs ~+17 µs over (a): the covered checkpoint leads the frame
end by only ~13 µs there, so short frames routinely resolve complete
(fast path) and ride the LOW-consumer hop instead of speculating — the §6
estimate (~0–8 µs elastic miss) under-counted the hop at ~17 µs. OPEN: 3M
sits over the ~41 µs slim-ping budget; candidate levers are hop-path
`.highcode` placement (the consumer path is cold flash — the same
placement tax measured as 63.6 µs write-speculation vs 26.7 µs read at
HIGH) and widening the speculation window at 3M. Accept-vs-optimize is an
explicit follow-up decision, not part of this band.

(An earlier revision of row 1 quoted 37.4/32.3/31.2/36.7 from the
turnaround-band memory — those columns were ping/read-16/read-240/ping from
a different build; rows above are same-day measurements on the exact bases.)

A1+A2 hold the losslessness column: hot loop AND plain
`8×WRITE(NOREPLY)+READ` bursts 300/300 at every baud (500k/1M/2M/3M,
2026-07-08). The earlier ~2 % plain-burst residual at ≤2M was the
pirate, not the servo: its burst path released the bus drive (PB10
push-pull → open-drain) from preemptible thread mode, so a walker drain
landing at burst end kept the pirate driving idle-high into the servo's
reply — IC-edge forensics (`tool-reply-edges`) showed the reply fought to
mark for 2–57 bits until the late release, reading back as a "malformed
break". Fixed pirate-side (TC-armed release + a release checkpoint in the
walk loop, `1f57ac51`); the servo transport needed no change. The ~+7 µs
over the pre-FIFO base is resolver arithmetic on the covered/B wakes,
inside the layout-swing noise band; the dominant latency (burst-cycle READ
replies ride elastically behind 14-68 µs dispatch bodies at HIGH) is what
A3(b) removes.

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
| USART1 vector   | PFIC HIGH. (a) FE/RX-error = break/garble event (never reads DR — §6 A4 flag park); (b) TC = TX arm drained | FE body ~1–2 µs + settle spin; TC body ~2–4 µs/arm |
| SysTick CNT/CMP | the transport clock (48 MHz, 32-bit) + the ONE comparator | — |
| SysTick vector  | PFIC HIGH. Deadline mux: framer A/B, covered, chain trigger, rescue confirm, adoption (reply-ready pend) | arithmetic slots ~1–5 µs; speculation dispatch runs here (§6 A3(b)) |
| Software vector (14) | PFIC LOW: live-lane dispatch-consumer wake (§6 A3(b)) | decode + dispatch, ~10–70 µs |
| I2C1_EV vector (30) | PFIC LOW: backlog-lane dispatch-consumer wake — a pending kernel tick (22) slots in first | same body as 14 |
| DMA1 CH5        | USART1 RX → 512 B ring, circular, silent (no IRQ) | zero CPU |
| DMA1 CH4        | TX arms → USART1 DR (header, snapshot payload, CRC tail) | zero CPU; TC surfaces as USART TC |
| DMA1 CH3        | CRC feeds → SPI1 DR, 16-bit halfwords            | zero CPU, ~0.36 µs/B engine time |
| SPI1            | CRC-16/ARC coprocessor (16-bit LSB-first, bitrev16 at the register), accumulates across feeds | runs ~8× wire speed (F6) |
| DMA1 CH6/CH7    | snapshot copies → the 256 B snapshot buffer (payloads + linearized RX spans); VERYHIGH, above CH3/CH4 | ~0.125 µs/B, zero CPU |
| DMA1 CH1 vector | PFIC LOW: ADC sample set complete → motor kernel tick | ~10 µs body |
| PC0 CNF         | drive discipline: open-drain listening / push-pull TX window | flipped at trigger/release |
| main loop       | deferred reboot poll only                        | cold path |

PFIC preemption is two-level (IPRIOR bit 7). USART1 + SysTick share HIGH and
therefore serialize against each other; LOW holds the kernel between the two
consumer lanes (same-class arbitration is lowest-vector-first — the locked
interleave policy, §6 A3). Free and reserved: TIM2 (reserved: future
encoders), TIM1 (motor PWM), I2C1_ER (31).

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
trigger tail. **DECIDED: (b)** (Aaron, 2026-07-08). **LANDED** same day
(`c83f6306` one-slot handoff + LOW consumer, `0c60bfda` chip vectors);
measured reality is in the header table — 1M better across the board, 3M
short frames ride the hop (the elastic-miss estimate under-counted; see
the header's OPEN note).

### A3(b) as landed — the handoff contract

One work slot each way (`Handoff`: EMPTY→JOB→WORKING→REPLY, single-writer
transitions, plain loads/stores + compiler fences — single hart, no A
extension). HIGH publishes a CRC-verified frame; the LOW consumer decodes,
dispatches, and records the reply verbs into a `ReplyRecord` (raw payload
span under the §4.2 zero-copy contract — every non-empty core reply
payload is table-backed); HIGH adopts: applies deferred config, stages the
reply into the TX engine, sequences the chain from the frame's packet end.
Backpressure is the slot: while occupied the framer holds position and the
ring absorbs the backlog (A2). SESSION exclusivity: HIGH touches the
dispatcher only on speculation paths, which are unreachable while a job is
claimed — the two borrows are temporally exclusive across preemption.
Adoption guard (positional truth): a slot-0 reply whose frame has newer
ringed bytes behind it is dropped — the zero-gap collision the FE-kill can
no longer catch.

### A4 — RX-error flag discipline: the stream clears its own flags

V006's FE/ORE/NE/PE are read-only, cleared exclusively by the
SR-read-then-DR-read sequence (write-0 bench-disproven: the vector storms
at ~8.6 µs/entry forever). The DR read races the RX-DMA drain: landing
between a byte's completion and its ~10-cycle drain STEALS the byte from
the ring — and the alignment is structural, not random (consumer
publish/adoption bodies at HIGH delay FE delivery onto the same grid
slot). Byte-level proof: burst write frames captured in the ring as
perfect 11-of-12-byte prefixes; a lost LEN byte reads `00 01 34 84` →
junk footprint 55 with INST bit 7 set → the whole span silently consumed
as a snooped status — zero counters, stale reply after the giveup hunt.

Therefore **the error vector never reads DR**. Its entry STATR read arms
the sequence; the stream's own next DMA drain is the DR half and clears
the flag in hardware. EIE is masked meanwhile (a parked flag must not
storm), and `maintain_rx_flags` — run at every HIGH wake — unmasks once
the flags read clean, or CPU-clears only in a provably quiet window: our
own reply streaming (TCIE set — the host is contractually silent and
HDSEL has no self-echo), or the cursor idle past ~2 byte-times at the
rescue floor. Break delivery is not framing truth (A2): an FE masked
through the window surfaces as ring data and resolves on the fast path.

The park's liveness rests on the FE-promise (`db75cff7`): every FE
promises a ring byte, but a prompt entry can beat the byte's own DMA
drain — the resolver then sees a caught-up ladder and must not go
aimless (register-dump post-mortem: masked flags + cancelled comparator =
permanently deaf servo; a one-shot recheck outside the framer was
destroyed by an adoption wake's walk). The framer owns it as state
(`fe_pending`): a caught-up resolve returns a bounded wait one byte-time
from the FE tick, re-derived on every walk — immune to wake interleaving —
and surrenders past the window (a phantom FE delivers nothing). Probe
caveat: event-ring probes (~1 µs/event) mask this whole class by letting
the byte ring before the walks — verify on stripped builds only.

## 7. First-principles losslessness (the zero-gap argument)

1. Capture: every byte is DMA'd regardless of CPU state; breaks pend and
   may coalesce, but A2 derives frame positions from the stream, so a
   coalesced or delayed FE costs nothing — the fast path recovers every
   completed frame from data.
2. Backlog: bounded by the ring (512 B ≈ 15 minimal hot-loop frames). The
   consumer outruns the wire (~2× worst case today, improving with decode
   work); sustained overrun is a THROUGHPUT invariant, pinned by the DES
   flood test (`zero_gap_flood` in `hot_loop.rs`: 100 zero-gap minimal
   writes + READ, ring laps ~2×, dispatch-cost sweep inside the contract,
   zero loss at 1M and 3M), not by a runtime guard.
3. Ordering: the single staged-reply slot + in-order ring walk preserve
   frame order; a newer instruction superseding a staged, not-yet-streaming
   reply is unchanged from this base.
4. Acceptance gates, status 2026-07-08: DES zero-gap suite green (incl.
   gap-jitter and mid-frame-stall wire models — stalls capped below the
   64-byte-time dead-transmitter horizon, past it the sacrifice is by
   design); `tool-osc-burst` plain+hot matrices 300/300 at ALL four bauds
   (2400/2400); `tool-reply-edges` soak 15000/15000 at 1M + 10000/10000 at
   3M, diag counters zero throughout; turnaround ≤ baseline at 1M ✓, 3M
   over budget (header OPEN note — explicit follow-up decision).
5. RESPONSE_DEADLINE must cover the full carve-out path — hop + decode +
   dispatch + adoption — not just the dispatch body: with ~70 µs handler
   bodies the 60 µs default is dishonest and a chain slot reclaims into a
   live-but-slow predecessor (DES-pinned in `hot_loop.rs`); deployments
   tune the register to their measured worst case.

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
