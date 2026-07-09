# osc servo transport — the deadline-pipelined receiver

Status: A1 + A2 LANDED (2026-07-08, `84680bd2`); DISPATCH-BEFORE-VERDICT
unified as the spine (2026-07-08); DMA priority ladder + direct-from-ring RX
CRC LANDED (2026-07-08) — RX alone owns the top of the DMA ladder, which
closes the FE-before-byte / byte-steal window at the hardware arbiter and
retired the `db75cff7` FE-promise + `a19d9377` RX-flag-park bandaid, and RX
CRC now feeds the ring directly (no M2M staging copy) — see §6 A4. Dispatch
always runs ahead of the CRC verdict,
routed by instruction class, and the verdict gates EFFECTS, not work (§4,
§6 A3(b); "speculation" survives only in the decision-history of §6 A3 and
§8). Describes the transport as restored
by the pre-FIFO revert and evolved by the CRC/framing side quest (CRC-16/ARC
LSB-first, no pad/parity rules, snapshot reads), then the amendments that
close the zero-gap defect and the spine unification. Companion pillars:
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
| spine unified        | 38.3 µs | 44.7 µs | 127.3 µs | 40.7 µs | 2400/2400         |

Spine same-day 3M row: read-16 49.7 µs, write 138.7 µs. The class routing
(§6 A3(b)) trades write latency for ping/read latency and kernel isolation:

- **Ping/read recover fully at 3M.** They dispatch inline at HIGH with zero
  hop (wire class), so the A3(b) 3M regression is gone: ping 58.4 → 40.7,
  read-16 66.6 → 49.7. 1M ping/read are flat (within the layout swing).
- **Ack-bearing writes regress** (1M 84.5 → 127.3, 3M 113.7 → 138.7): a
  write now stages through the LOW consumer, so its ack rides the hop +
  adoption round-trip instead of the covered-window overlap. For a short
  write there is almost no wire tail to hide the hop behind, so the reply
  lands ~25–43 µs late and the ack turnaround goes roughly baud-independent
  (~127–139 µs). This is the accepted cost of moving the write-dispatch body
  (~63.6 µs, cold flash) off HIGH — the kernel-isolation win. The intended
  hot loop writes are NOREPLY (§7): they cost pure wire time and pay no ack
  turnaround at all, so this regression is off the hot path by construction.

OPEN: 3M ping 40.7 µs is at the ~41 µs slim-ping budget (it swung to 42.9 on
a pre-refactor layout — hop-path `.highcode` placement is the lever if a
build lands over). Whether the ack-bearing-write cost is worth revisiting
(e.g. a covered-lead widening, or inline-HIGH dispatch for short writes at
the expense of kernel isolation) is an explicit follow-up decision, not part
of this band.

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
| USART1 vector   | PFIC HIGH. (a) FE/RX-error = break/garble event (plain SR-then-DR clear after on_break, §6 A4); (b) TC = TX arm drained | FE body ~1 µs; TC body ~2–4 µs/arm |
| SysTick CNT/CMP | the transport clock (48 MHz, 32-bit) + the ONE comparator | — |
| SysTick vector  | PFIC HIGH. Deadline mux: framer A/B, covered, chain trigger, rescue confirm, adoption (reply-ready pend) | arithmetic slots ~1–5 µs; wire-class dispatch runs here (§6 A3(b)) |
| Software vector (14) | PFIC LOW: live-lane dispatch-consumer wake — table-class dispatch (§6 A3(b)) | decode + dispatch, ~10–70 µs |
| I2C1_EV vector (30) | PFIC LOW: backlog-lane dispatch-consumer wake — a pending kernel tick (22) slots in first | same body as 14 |
| DMA1 CH5        | USART1 RX → 512 B ring, circular, silent (no IRQ); **VERYHIGH, alone atop the ladder** (§6 A4) | zero CPU |
| DMA1 CH4        | TX arms → USART1 DR (header, snapshot payload, CRC tail); HIGH | zero CPU; TC surfaces as USART TC |
| DMA1 CH3        | CRC feeds → SPI1 DR, 16-bit halfwords (RX span straight from the ring); MEDIUM, below CH6 | zero CPU, ~0.36 µs/B engine time |
| SPI1            | CRC-16/ARC coprocessor (16-bit LSB-first, bitrev16 at the register), accumulates across feeds | runs ~8× wire speed (F6) |
| DMA1 CH6        | snapshot copy → the 256 B snapshot buffer (reply payloads only — RX CRC feeds the ring directly); HIGH, above CH3; CH7 now unused | ~0.125 µs/B, zero CPU |
| DMA1 CH1        | ADC sample set → buffer; DMA HIGH (wins HIGH ties by channel number); TC vector = motor kernel tick at PFIC LOW | ~10 µs body |
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
         · DISPATCH (wire class, at HIGH): decode + dispatch run NOW, the
           reply is built and staged into the TX engine — all before the
           frame has ended. The verdict at deadline B will SEND or DON'T-SEND
           it; the work is already done either way.
       Deadline B armed at packet_end estimate + 5 µs slack.
t=40/50  the two wire-CRC bytes land. SPI engine finishes the covered span
       long before (4 B ≈ 1.5 µs). CPU idle again.
t≈56   deadline B (SysTick): the verdict — poll CRC result (ready) == wire
       CRC → SEND: chain sequences the staged reply, trigger due at
       packet_end + T_turn (2 byte-times). Body is a few µs — the work
       already happened.
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

0. **Dispatch before the verdict — the spine.** Dispatch never waits on the
   CRC. It runs the moment the covered span is ringed (frontier) or the
   frame is complete (backlog), and STAGES its effects; the CRC verdict then
   gates those effects, never the work. This is not an optimization layered
   on a "safe" path — it IS the default, for every class. The alternative (a
   non-dispatching path that schedules a CRC check and blocks on the result
   before doing any work) spends ~5–7 µs of HIGH per frame spinning on a
   finished engine, and in a zero-gap burst those spins stack onto the
   burst-cycle critical path and widen FE-delivery lag (the byte-steal
   lesson, §6 A4). Deleting that path is the win. The verdict gates two
   effect kinds: the **wire effect** (a staged reply — SEND on pass,
   DON'T-SEND on fail) and the **table effect** (staged writes — COMMIT on
   pass, REVERT on fail). Ping/read stage only a wire effect; a NOREPLY
   write only a table effect; a reply-bearing write both, under one verdict.
1. **Work hides under wire time.** The dispatch window (covered checkpoint →
   frame end) runs decode + dispatch + reply build while the last two CRC
   bytes are still in flight. Deadline B — the only step on the reply's
   critical path — is reduced to "poll a finished CRC and arm the trigger".
   The reply rides the T_turn grid, not the dispatch time.
2. **The verdict is almost always PASS.** Dispatch bets the frame passes
   CRC. On a clean bus that is ~100% of frames; the loser (a corrupted
   frame) pays a revert + don't-send, which is off the happy path by
   definition — and the ladder is untouched by it (§6 A3(b)), so the hunt
   just starts one hop later, on corrupt frames only, inside the host's
   retry contract.
3. **Hardware CRC in both directions, in parallel.** RX: fed at the covered
   checkpoint (or publish), chewing while the CRC bytes fly, polled at the
   verdict. TX: fed per arm, patched into the trailing arm before DMA
   reaches it. The CPU never computes or waits a full CRC.
4. **Zero hops where latency is the product; a hop where isolation is.**
   Wire-class frames (ping/read) dispatch inline at HIGH — every stage hands
   off by falling through within one ISR invocation or the next event's
   entry, no cross-priority round-trip. Table-class frames (write/gwrite)
   take one HIGH→LOW→HIGH hop, deliberately: it moves the write-dispatch
   body off HIGH so the motor kernel is never preempted mid-write, and the
   hot loop's writes are NOREPLY so the hop never sits on a reply's critical
   path (§6 A3(b)).
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
  today's deadline pipeline unchanged: A → covered (dispatch, §6 A3(b)) → B
  (verdict), timed from `last_break_ts`.
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

### A3(b) as landed — class routing and the handoff contract

The carve-out is not "heavy phases hop" — it is **routing by instruction
class**, decided from the INST byte the moment the covered span (or the
whole frame) is ringed. Same byte the decoder reads at dispatch, so the two
can never disagree:

- **Wire class — PING/READ/GREAD.** Side-effect-free: the only effect is a
  reply. Dispatch inline at HIGH, right at the covered checkpoint (frontier)
  or in the resolve wake (backlog); the CRC feed chews underneath. The
  verdict at the frame end gates SEND / DON'T-SEND of the staged reply.
  Zero hop — this is what recovers 3M ping/read (header table).
- **Table class — WRITE/GWRITE.** Effect is a table mutation, ± a reply.
  Publish to the LOW consumer at the covered checkpoint (decode needs only
  covered bytes, so hop + decode + dispatch overlap the wire tail), where
  the write validates and STAGES above a watermark. The verdict fires when
  both halves are in — the frame end AND the consumer's record, in either
  order — and gates COMMIT / REVERT of the staged write plus SEND /
  DON'T-SEND of the recorded reply, at HIGH. Moving the write-dispatch body
  off HIGH is the kernel-isolation win; the ack-turnaround cost is in the
  header's OPEN note.
- **Verdict-first — COMMIT/MGMT.** Their effects cannot be staged (COMMIT
  applies the whole buffer; MGMT reboots), so the CRC is checked FIRST and
  dispatch runs only on a pass. Rare, short frames — the ~2 µs blocking CRC
  poll is affordable where staging isn't possible.

The handoff: one work slot each way (`Handoff`: EMPTY→JOB→WORKING→REPLY,
single-writer transitions, plain loads/stores + compiler fences — single
hart, no A extension). The LOW consumer decodes, dispatches, and records the
reply verbs into a `ReplyRecord` (raw payload span under the §4.2 zero-copy
contract — every non-empty core reply payload is table-backed, plus a
`pending` flag = "a table effect is staged, adopter owes the verdict");
HIGH adopts: resolves the verdict, applies deferred config, stages the reply
into the TX engine, sequences the chain from the frame's packet end.
Backpressure is the slot: while occupied the framer holds position and the
ring absorbs the backlog (A2) — which is also what bounds the pending frame
to ONE (single staging slot + single CRC accumulator, never contended) and
keeps the CRC engine quiet across the hop (nothing feeds or resets it
between the publish-time feed and the adoption-time verify). SESSION
exclusivity: HIGH touches the dispatcher only on the spine's HIGH paths,
unreachable while a job is claimed — the two borrows are temporally
exclusive across preemption. Adoption guard (positional truth): a slot-0
reply whose frame has newer ringed bytes behind it is dropped — the
zero-gap collision the FE-kill can no longer catch.

### A4 — RX owns the DMA top: the drain always beats the IRQ

The two RX-error hazards are one window: the RX-DMA drain of an inbound
byte racing the CPU. If the drain lags, the break's ERR IRQ enters with
the ring cursor not yet advanced past the byte — the resolver sees a
caught-up ladder and stays silent (no-reply) — or the SR-then-DR error
clear's DR read pops the byte before DMA does (ring loss, a stale reply
after the giveup hunt). The bringup spike `rx_dma_drain_latency` resolved
the mechanism on silicon: in isolation the drain is effectively
instantaneous versus the IRQ (0/114 RXNE-set-at-entry at 3M, 0/100 at 1M),
so the window never opens; it opens ONLY under DMA1-arbiter contention (a
competitor delaying the CH5 drain — 22% at a moderate burst, 94% at a
heavy one). The arbiter preempts **per-beat** (proven: RX stays clean even
against a 256-transfer competitor once it outranks it), so RX alone at the
top bounds its drain wait to a single in-flight transfer — a couple of
cycles, far under the IRQ entry latency — regardless of the competitor's
burst length.

So the fix is the priority ladder, not a software mitigation. VERYHIGH:
CH5 RX (alone at the top). HIGH: CH1 ADC, CH4 TX, CH6 snapshot (ADC wins
the HIGH ties by channel number, keeping its interleave ahead of the
copy). MEDIUM: CH3 CRC feed (below CH6 so a reply copy is written before
the feed reads it). With RX on top the break byte is always ringed before
`on_break` reads the cursor, so the framer needs no FE-promise; and the
error vector's DR read gets stale data, never an in-flight byte, so it
clears with the plain SR-then-DR sequence after `on_break` (FE/ORE/NE/PE
stay read-only; write-0 is bench-disproven — the vector storms). Silicon:
25k/25k zero-gap hot loops at 2M and 3M with zero no-reply/stale. A tiny
low-baud residual (~7/25k at 1M, 1/25k at 0.5M) is a separate,
baud-dependent break-artifact — NOT this window, which is baud-independent
(clean at 2M/3M) — tracked as its own investigation.

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
3. Ordering: at most one pending-verdict frame at a time (backpressure: the
   framer holds while the handoff slot is occupied, and a wire-class pending
   frame IS the frontier), so the single staging slot + single CRC
   accumulator are never contended and the in-order ring walk preserves
   frame order. A backlog write commits before the frame behind it
   dispatches (the walk resumes only at adoption) — DES-pinned by
   `backlog_write_then_read_processes_in_order`.
4. Acceptance gates, status 2026-07-08 (spine build): DES zero-gap suite
   green (incl. gap-jitter and mid-frame-stall wire models — stalls capped
   below the 64-byte-time dead-transmitter horizon, past it the sacrifice is
   by design); `tool-osc-burst` plain+hot matrices 300/300 at ALL four bauds
   (2400/2400), servo clean afterward; `tool-reply-edges` soak 10000/10000
   at 3M, zero failures. Turnaround: 3M ping/read recovered (58.4→40.7,
   66.6→49.7); 1M ping/read flat; ack-bearing writes regressed by design
   (header table + OPEN note).
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
  (2 byte-times before frame end); the dispatch point.
- **dispatch (the spine)** — decode + dispatch + reply build performed
  before the CRC verdict — the default for every class, not an optimization
  (§4). Effects STAGE; they do not apply until the verdict.
- **verdict** — the CRC result at the frame end, gating the staged effects:
  the **wire effect** (a staged reply — SEND on pass, DON'T-SEND on fail)
  and the **table effect** (staged writes — COMMIT on pass, REVERT on fail).
  Ping/read stage only wire; a NOREPLY write only table; a reply-bearing
  write both, under one verdict.
- **instruction class** — the dispatch routing (§6 A3(b)): *wire*
  (ping/read/gread, inline at HIGH), *table* (write/gwrite, staged via the
  LOW consumer), *verdict-first* (commit/mgmt, CRC checked before dispatch).
- **deadline A / B** — header-readable check / frame-end check, both
  cursor-verified estimates from the break timestamp.
- **T_turn** — the mandated 2-byte-time wire gap between a frame and its
  reply; the reply grid.
- **fast path / scheduled path** — A2's split: complete-in-ring frames
  resolved from data with no clock / the newest frame timed by deadlines.
