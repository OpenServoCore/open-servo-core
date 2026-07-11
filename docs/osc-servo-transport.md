# osc servo transport — the deadline-pipelined receiver

Status: A1 + A2 LANDED (2026-07-08, `84680bd2`); DISPATCH-BEFORE-VERDICT
unified as the spine (2026-07-08); DMA priority ladder + direct-from-ring RX
CRC LANDED (2026-07-08) — RX alone owns the top of the DMA ladder, which
closes the FE-before-byte / byte-steal window at the hardware arbiter and
retired the `db75cff7` FE-promise + `a19d9377` RX-flag-park bandaid, and RX
CRC now feeds the ring directly (no M2M staging copy) — see §6 A4.
EVERYTHING RUNS AT HIGH (inline-unify, 2026-07-08): the LOW dispatch
consumer and its two wake lanes are DELETED — every instruction class
except verdict-first dispatches inline on the HIGH vectors, and the motor
kernel's tick coalescing under bursts is measured and accepted (§2).
Dispatch always runs ahead of the CRC verdict, and the verdict gates
EFFECTS, not work (§4; "speculation" and the class-routing carve-out
survive only in the decision-history of §6 A3 and §8). Low-baud flood
residual ROOT-CAUSED AND FIXED (2026-07-09): a CPU DATAR read mid-reception
kills the in-flight RX byte (§6 A4 — the RX/error path never touches DATAR;
a latched leftover flag is conditionally retired at reply release, under
our own line drive), and the rescue confirm is two-phase against data-bit
aliasing (`osc-native-protocol.md` §9.1). RING-CADENCE TIME (2026-07-09):
the FE clock is DELETED — every aim and estimate projects `now + missing
byte-times` from live ring state (§6 A1), reply gap is a fixed 12 µs at every
baud (§7 spec change), and the turnaround U-shape's low side is gone
(0.5M ping 47.6 → 35.6 µs). Companion pillars:
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
| inline-unify + DATAR discipline | 35.3 µs | 39.9 µs | 86.8 µs | 45.6 µs | 8000/8000 (+24k floods ≤1M) |
| ring-cadence + reply gap 12 µs | 34.3 µs | 38.6 µs | 87.7 µs | 44.3 µs | 8000/8000 (+24k floods ≤1M) |
| in-place chain trigger | 30.4 µs | — | — | 41.3 µs | 8000/8000 (+24k floods ≤1M) |

(In-place-trigger row: ping 38.8 µs at 2M, 32.2 µs at 0.5M; burst means
dropped again — hot loop @1M 35.4 µs, plain flood @1M 31.3 µs.)

(Profile reads, 2026-07-10 build: a 3-span/8 B §5.2 profile read runs
39.9/48.2/46.6/50.5 µs at 0.5M/1M/2M/3M vs 36.4/37.6/40.0/39.1 for the
same 8 B as one contiguous READ — the span resolution and two extra
snapshot copies cost ~4–11 µs, hidden under wire time only at 0.5M.
50/50 clean at every baud; the 2M soak surfaced a PIRATE capture bug
(idle-clear DATAR read eats the reply's final byte every 128 exchanges —
task #7), servo exonerated by BICSNAP edge counts.)

(DATAR-discipline row: ping 42.2 µs at 2M, 47.2 µs at 0.5M — flat vs the
pre-fix baseline; the burst columns include the formerly-failing ≤1M legs.
Ring-cadence row: ping 41.0 µs at 2M, **35.6 µs at 0.5M** (was 47.6) — the
grid's baud-dependence is gone; and burst replies stopped inheriting
dispatch lag: hot-loop mean turnaround at 1M 78 → 38 µs, plain-flood 76 →
35 µs, with the strict gates green throughout.)

Spine same-day 3M row: read-16 49.7 µs, write 138.7 µs. The class-routing
carve-out was then SUPERSEDED (inline-unify, 2026-07-08): the LOW consumer
and its hop were deleted outright, every class dispatches inline at HIGH,
and the ack-bearing-write regression it caused reversed (1M write
127.3 → 89); the kernel-isolation cost of losing the carve-out is the
measured, accepted tick coalescing in §2.

Turnaround shape after the ring-cadence band: the reply's tail after the
instruction's wire end is `max(serialized CPU pipeline, reply-gap grid)`. The
grid is now flat (12 µs at every baud) and the estimate it hangs off is
delivery-noise-free, so the low side collapsed — 0.5M/1M pings sit at
~35 µs together. 2M/3M remain pipeline-bound (~41/44 µs): the covered
window shrinks below the dispatch body — at 3M a short frame is fully
ringed before the first deadline wake even fires — so the pipeline
serializes after the frame end. Follow-ups pulled and retired
(2026-07-09): an already-due chain wait is now consumed in place instead of
taking a scheduling round trip (sequencing→trigger 10.1 → 4.4 µs at 3M),
and the deadline-A arming-order lever died with the FE clock (the on_break
body overlaps the frame's own wire time; ~2 µs remained). The one lever
deliberately left unpulled is dispatch-body RAM placement: bench-probed
2026-07-09 — the ~1.8 kB dispatch monomorphization does NOT fit the V006's
RAM budget (the stack dipped into statics and silently corrupted the
table), and smaller placements bought only ~2 µs. Revisit only with a
real RAM budget, by explicit decision.

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
| USART1 vector   | PFIC HIGH. (a) FE/RX-error = break/garble event (never reads DATAR — flags self-clear via the CH5 drain; a latched leftover is retired at reply release, §6 A4); (b) TC = TX arm drained | FE body ~1 µs; TC body ~2–4 µs/arm |
| SysTick CNT/CMP | the transport clock (48 MHz, 32-bit) + the ONE comparator | — |
| SysTick vector  | PFIC HIGH. Deadline mux: framer A/B, covered, chain trigger, rescue confirm — and dispatch, inline (every class except verdict-first runs at the covered checkpoint or the fast path) | arithmetic slots ~1–5 µs; dispatch bodies ~10–70 µs |
| DMA1 CH5        | USART1 RX → 512 B ring, circular, silent (no IRQ); **VERYHIGH, alone atop the ladder** (§6 A4) | zero CPU |
| DMA1 CH4        | TX arms → USART1 DR (header, snapshot payload, CRC tail); HIGH | zero CPU; TC surfaces as USART TC |
| DMA1 CH3        | CRC feeds → SPI1 DR, 16-bit halfwords (RX span straight from the ring); MEDIUM, below CH6 | zero CPU, ~0.36 µs/B engine time |
| SPI1            | CRC-16/ARC coprocessor (16-bit LSB-first, bitrev16 at the register), accumulates across feeds | runs ~8× wire speed (F6) |
| DMA1 CH6        | snapshot copy → the 256 B snapshot buffer (reply payloads only — RX CRC feeds the ring directly); HIGH, above CH3; CH7 now unused | ~0.125 µs/B, zero CPU |
| DMA1 CH1        | ADC sample set → buffer; DMA HIGH (wins HIGH ties by channel number); TC vector = motor kernel tick at PFIC LOW | ~10 µs body |
| PC0 CNF         | drive discipline: open-drain listening / push-pull TX window | flipped at trigger/release |
| main loop       | deferred reboot poll only                        | cold path |

Rev B buffered boards (board config `wire-buffered`) swap the wire rows:
USART1 runs plain full duplex (no HDSEL, RX on PC1 through the 74LVC2G241's
mute-gated receive buffer), and the PC0-CNF flip becomes the TX_EN (PC2)
level — high claims the wire and mutes RX, low releases. Every discipline
in this doc (release-point flag retire, the no-DATAR rule, F9 no-echo)
holds identically; only the claim/release lines in `TxWire` differ.

PFIC preemption is two-level (IPRIOR bit 7). USART1 + SysTick share HIGH and
therefore serialize against each other; LOW holds only the motor kernel
(DMA1_CH1 = 22), which HIGH preempts and which runs in the wire gaps
between frames. Free and reserved: TIM2 (reserved: future encoders), TIM1
(motor PWM), SW (14), I2C1_EV (30), I2C1_ER (31).

**Kernel ticks under load — measured and accepted.** Everything-at-HIGH
means transport work preempts the kernel, and a kernel tick that pends
while a ≥50 µs HIGH chunk runs coalesces with the next one (the PFIC pend
bit is one bit). Silicon (2026-07-09, `sample_tick` over the wire, idle
baseline 20.11 kHz): tick loss ≈ 1.2–1.4× the transport-HIGH duty — 14%
under a sustained 9-frame zero-gap flood (~11% duty), 23% under a
21-frame flood (~17% duty). Latency stays bounded (the longest HIGH chunk
is ~60–80 µs ≈ 1–2 ticks); ticks are never starved outright. Accepted
because the duty profile that loses ticks is rare in practice: config
sessions run torque-off (kernel idle by definition), and the production
hot loop is a few small GWRITEs + COMMIT + GREAD per cycle — short
bursts, wire-gapped, single-digit duty. A use case that sustains heavy
bus duty under live control is the signal to revisit (hardware-counted
ticks, or re-introducing an isolation lane).

## 3. One exchange, tick by tick (ping at 1M; byte-time = 10 µs)

```
t=0    break's wire end. DMA has already ringed the 0x00. FE ISR (~2 µs) —
       a pure wake: the framer resolves from ring data, projects deadline A
       = now + 3 byte-times at wire pace + ½ byte for the break tail [F5],
       and arms rescue candidacy. No timestamp is recorded.
t=10/20/30  ID, LEN, INST land in the ring by DMA. CPU idle.
t=35   deadline A (SysTick): header parse + validate → footprint 6, frame
       end computable. A ping's CRC-covered span IS its header, so the
       covered checkpoint fires from this same wake:
         · packet_end FROZEN by ring cadence: now + missing(2)·byte-times
           (+ the 1.6% drift pad) — `now` and the cursor advance together,
           so ISR lag cancels and the estimate is late by under a
           byte-time, never early.
         · CRC feed of the covered span starts (CH3 arm, ~1 µs of CPU)
         · DISPATCH (inline, at HIGH): decode + dispatch run NOW, the
           reply is built and staged into the TX engine — all before the
           frame has ended. The verdict at deadline B will SEND or
           DON'T-SEND it; the work is already done either way.
       Deadline B armed at the frozen packet_end, exactly.
t=40/50  the two wire-CRC bytes land. SPI engine finishes the covered span
       long before (4 B ≈ 1.5 µs). CPU idle again.
t≈55   deadline B (SysTick): the verdict — poll CRC result (ready) == wire
       CRC → SEND: chain sequences the staged reply, trigger due at
       packet_end + reply gap (12 µs, fixed at every baud). Body is a few µs —
       the work already happened.
t≈67   trigger (SysTick): INST finalized, PC0 → push-pull, break sent, first
       DMA arm armed. Status break falls. TX CRC is computed by the same SPI
       engine IN PARALLEL with transmission and patched into the final arm.
t=…    per-arm TC ISRs stream the remaining arms; final TC releases the wire
       and applies any deferred id/baud config.
```

Measured: 34.3 µs from instruction end to status break fall (ring-cadence
build, 2026-07-09); ~16 µs of it is the estimate chain above (reply-gap grid +
sub-byte estimate lateness), the remainder trigger-body, SBK commit, and
ISR-entry overheads.

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
   The reply rides the reply-gap grid, not the dispatch time.
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
4. **Zero hops, everywhere.** Every stageable class (ping/read/gread,
   write/gwrite) dispatches inline at HIGH — each stage hands off by
   falling through within one ISR invocation or the next event's entry,
   no cross-priority round-trip (the LOW-consumer carve-out that briefly
   traded write latency for kernel isolation is decision history, §6 A3).
   The kernel-side cost is the measured, accepted tick coalescing in §2.
5. **Copy-once TX.** Reply payloads are DMA-snapshotted once (~0.125 µs/B,
   fire-and-forget) and streamed from the snapshot by both the wire and the
   CRC — snapshot-consistent reads for the price of one copy hidden under
   reply gap (§4.2 of the wire spec).
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

### A1 — ring-cadence time: the FE carries nothing

A1 originally kept one timestamp — the newest FE delivery tick — as the
frontier frame's clock anchor. DELETED (ring-cadence band, 2026-07-09):
the FE now carries neither position (A2) nor time; it is a pure wake, and
its handler does only the bounded wire work (staged-reply kill, chain
suspend, rescue-candidacy arming). Every aim and estimate is instead
projected from live ring state at each resolve:

```
aim / estimate = now + missing·tpb (+ span·tpb ≫ 6 drift pad)
```

`now` and the cursor advance together, so ISR delivery lag — the thing the
timestamp inherited wholesale (a 60 µs-late FE made the reply 60 µs late) —
cancels out of the projection. The error is bounded by one byte-time plus
one wake latency and is always on the LATE side: a byte counted as missing
can only finish sooner than assumed, so a reply grid measured from the
estimate never fires into its own frame's tail (the reply gap safety
property, sim-pinned). The reply-grid estimate is frozen at the covered
checkpoint, where missing ≤ 2 makes it tightest. Aims are self-pacing for
free: a stalled wire re-projects at each wake ≥ one byte-time out, which
deleted the recheck/park budget machinery wholesale; the 64-byte-time
progress-idle horizon remains the sole death authority. One epsilon
survives, on header aims only: the break rings at its FE point ~4
bit-times before the line rises [F5], so the first data byte sits that far
outside the byte cadence. Cursor reads survive only at provably-quiet
bootstrap moments: boot (position 0 by construction) and rescue confirm (a
held-low line delivers no start edges — the cursor is still).

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
- Garble dies by DATA only (the fault contract, osc-native §3.4): a
  phantom candidate born from garble is killed by its footprint-fill CRC
  verdict or by the starve horizon — never by fault position. **The
  wire-fault fence is deliberately dead** (shipped 2026-07-10, deleted
  2026-07-11): it recorded service-time cursors as fault positions, and
  those lie under exactly the conditions a fleet produces — latched-flag
  re-fires after NOREPLY frames (nothing transmits, so nothing retires
  the flag) and coalesced/lagged break service at high ISR occupancy.
  Two such services during one live frame's flight planted the fence
  inside the frame and killed it (bench 2026-07-11: hot
  GWRITE+COMMIT+GREAD chains fell 95%→80%, silent chain slots, framing
  drops on every servo; the DES pin is
  `latched_refires_mid_frame_never_kill_the_trusted_stream`). The cost
  of the deletion is bounded, documented recovery latency (§3.4 host
  pacing rule) instead of fault-speed phantom kills — the #9 indefinite
  one-late cascade stays dead because the phantom's CRC rejection flips
  the hunt on and the hunt converges from ring data.
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
`hop + decode + dispatch ≤ covered→B window + reply gap`:

| frame     | window+reply gap 1M | hop+work (est.) | grid held? | window+reply gap 3M | grid held? |
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

### A3(b) → inline-unify — every class at HIGH

A3(b) landed as **routing by instruction class** — wire class
(PING/READ/GREAD) inline at HIGH, table class (WRITE/GWRITE) staged
through a LOW consumer over a one-slot handoff, verdict-first
(COMMIT/MGMT) CRC-checked before dispatch. The carve-out bought kernel
isolation but cost ack-bearing writes a ~25–43 µs hop + adoption
round-trip, and the handoff machinery (slot FSM, two wake lanes, adoption
guard, ReplyRecord) was the single largest complexity center in the
composite. **Inline-unify (2026-07-08) deleted it** (−827 LOC): the class
split survives only as *stageability* —

- **Stageable — PING/READ/GREAD/WRITE/GWRITE.** Dispatch inline at HIGH at
  the covered checkpoint (frontier) or the resolve wake (backlog); the CRC
  feed chews underneath. The verdict at the frame end gates the staged
  effects: SEND/DON'T-SEND of a reply, COMMIT/REVERT of a table write.
- **Verdict-first — COMMIT/MGMT.** Their effects cannot be staged (COMMIT
  applies the whole buffer; MGMT reboots), so the CRC is checked FIRST and
  dispatch runs only on a pass. Rare, short frames — the ~2 µs blocking
  CRC poll is affordable where staging isn't possible.

The kernel-isolation question the carve-out answered is now settled by
measurement instead of structure: dispatch bodies preempt the kernel and
coalesce ticks in proportion to bus duty (§2), which the intended duty
profiles make negligible. Backpressure survives unchanged: at most one
pending-verdict frame (the pending frame IS the frontier), so the single
staging slot and single CRC accumulator are never contended.

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
`on_break` reads the cursor, so the framer needs no FE-promise. Silicon:
25k/25k zero-gap hot loops at 2M and 3M with zero no-reply/stale.

The ladder protects only the byte already in RDR. The RX/error path must
additionally never read DATAR (2026-07-09, the former "low-baud
residual"): a CPU DATAR read while a byte is mid-reception kills the byte
in the SHIFTER — no flag, no ring entry, no NDTR count — and no drain
priority can help, because the byte dies before a DMA request ever
exists. The window scales with bit time: unhittable at 2M/3M, rare at
≤1M, and one alignment (idle-entry FE handler body ≈ first-byte
completion at 1M) made it systematic. The flag clear therefore splits by
mechanism (FE/ORE/NE/PE stay read-only; write-0 is bench-disproven — the
vector storms):

- **Drain-side, the normal case:** a STATR read arms the SR half of the
  SR-then-DR sequence and the CH5 drain's own DATAR access is the DR
  half — every flag-setting event rings a byte (F2/F4), so flags
  self-clear within a bit-time. An exit before the drain lands re-enters
  once with STATR clean, which is why any non-TC entry is treated as an
  RX error regardless of surviving flags.
- **Release-point, the latched corner:** a flag whose byte drained before
  any STATR read never formed the pair and stays latched — e.g. a lagged
  FE delivery on a burst's LAST frame, whose reply produces no further RX
  drains. Left alone it storms the vector once the wire idles (bench
  signature: the first ack-bearing exchange after a hot-loop leg dies).
  A garble tail with no reply leaves the same latch with no release to
  retire it, and the storm's armed SR-half is then completed by the NEXT
  frame's break-byte drain — consuming the flag at the very event that
  should have re-fired the vector. That frame would sit complete but
  unresolved (bench 2026-07-10: the post-garble one-instruction-late
  residue); the composite closes it with the evidence-in-flight recheck —
  a wire-fault service backed by no fresh ring bytes arms a one-byte-time
  framer recheck, so the frame resolves by data even when its FE was eaten.
  `TxWire::release` retires it with an SR-DR-SR clear while our push-pull
  drive still holds the line high — no byte can be mid-reception, so the
  DATAR read is provably safe there. The trailing STATR read is
  load-bearing: a DATAR read consumes the armed SR-half, and without the
  re-arm the NEXT break cannot drain-self-clear — its FE re-fires until
  the first data byte lands, dragging the frontier tick and the reply
  grid with it (bench: +12 µs of ping turnaround at 0.5M with a plain
  SR-DR clear; flat with the trailing re-arm).
- **Storm throttle, the quiet-bus corner:** the same garble-tail latch has
  no release point (nothing staged to send) and no next drain in sight —
  with level-pend PFIC the vector re-enters continuously until the next RX
  byte, a silent burn at transport priority that starves the motor kernel
  for the whole inter-frame gap (correctness-neutral since the
  evidence-in-flight recheck; pure CPU theft). Zero ring progress since the
  last fault is only the storm's SUSPECT signature — the routine mid-burst
  latch looks identical for one byte-time (a NOREPLY frame has no release
  to retire its flag, and its lagged re-entry lands just before the next
  frame's bytes), and muting at the fault service made a muted wake miss
  live breaks (bench 2026-07-11: hot GWRITE+COMMIT+GREAD chains fell
  95%→83%, silent slots + mid-stream fence drops). So the fault service
  only records the suspect cursor; the verdict is the recheck's: a
  deadline that finds the framer idle with the cursor still unmoved mutes
  the fault wake (`LineSense::set_fault_wake`, chip: EIE — flags keep
  latching and DMAR keeps ringing), bounding the storm to ~one byte-time
  of re-entries, and the deadline slot polls the ring at
  `FAULT_MUTE_POLL_BYTE_TIMES` instead, with a line-low mirror
  (gated on an empty rescue slot, so sub-100 µs polls never push a pending
  confirm out) keeping rescue pulses detectable. Ring progress is the
  all-clear — the drain that moved the cursor completed the storm entry's
  armed SR-half and retired the flag — and restores the wake; a reply
  release restores it too (its SR-DR-SR clear needs no ring progress to be
  pend-safe). Bounds: one deadline service per poll while quiet, and the
  first post-garble frame resolves at most the poll cadence late.

Silicon: 12k/12k plain floods at 1M and 0.5M (production shape) with the
RX-path DATAR read removed (vs 44/12k with it present); hot-loop matrix
green at all four bauds with the conditional release-point clear, ping
turnarounds flat vs the pre-fix baseline at every baud.

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
3. Ordering: at most one pending-verdict frame at a time (backpressure: a
   pending frame IS the frontier), so the single staging slot + single CRC
   accumulator are never contended and the in-order ring walk preserves
   frame order. A backlog write commits before the frame behind it
   dispatches — DES-pinned by
   `backlog_write_then_read_processes_in_order`.
4. Acceptance gates, status 2026-07-09 (DATAR-discipline build): gears 1+2
   green; hardware suite green at all four bauds INCLUDING the burst
   tests' zero-tolerance gates (the former "low-baud glitch" is fixed);
   plain-flood soaks 12k/12k at 1M and 0.5M, hot-loop matrix clean;
   `tool-reply-edges` soak 10000/10000 at 3M (2026-07-08 build).
5. RESPONSE_DEADLINE must cover the full reply path — decode + dispatch +
   verify, elastically late under a backlog — not just the happy-path
   grid: with ~70 µs dispatch bodies the 60 µs default is dishonest and a
   chain slot reclaims into a live-but-slow predecessor (DES-pinned in
   `hot_loop.rs`); deployments tune the register to their measured worst
   case.

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
- **instruction class** — stageability (§6 A3): *stageable*
  (ping/read/gread/write/gwrite — dispatch inline at HIGH, effects gated
  by the verdict), *verdict-first* (commit/mgmt, CRC checked before
  dispatch).
- **deadline A / B** — header-readable check / frame-end check, both
  ring-cadence projections (`now + missing byte-times`) re-derived at every
  wake.
- **reply gap** — the mandated wire gap between a frame and its reply (12 µs,
  fixed at every baud — host TC→release is time-domain, §7); the reply
  grid.
- **fast path / scheduled path** — A2's split: complete-in-ring frames
  resolved from data with no clock / the newest frame timed by deadlines.
