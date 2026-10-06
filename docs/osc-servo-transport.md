# osc servo transport — the deadline-pipelined receiver

The servo-side transport of the osc-native protocol: how a sub-$0.20 MCU
with no input capture and no crystal turns break-framed wire traffic into
dispatched instructions and on-grid replies. Companion pillars:
`osc-native-protocol.md` (the wire; §3.4 is the two-signal fault
contract), `driver-pattern.md` (the layering). `[Fn]` cites the measured
foundation in `osc-native-protocol.md` §11.

Code is authoritative; when this doc and the code disagree, fix the doc.

## 1. Architecture in one paragraph

All byte movement is hardware. The CPU never receives or transmits a
byte: DMA writes every received byte into a circular ring, DMA streams
every reply out of buffers and the control table, and a DMA-fed SPI
engine computes every CRC. Software is pure state machines — `Framer`,
`Chain`, `TxEngine`, and the clock tracker — composed by `ServoBus`,
driven by interrupt vectors at one priority, whose only jobs are to
compute two kinds of numbers - *where* frames sit in the ring, and *when*
something is due - and to react to two kinds of events: "a break
happened" (a timer on the bus pin, length-qualified at 9.25 bit-times)
and "it is time now" (one tick comparator, multiplexed over every
deadline the transport has).

## 2. Hardware ledger

Every hardware resource the transport touches, and its duty cycle:

| resource        | role                                             | budget/event |
|-----------------|--------------------------------------------------|--------------|
| USART1          | half-duplex wire; HDSEL, no self-echo (F9)       | —            |
| TIM2 vector     | PFIC HIGH. The break wake: TI1 on PC0 (remap 4), counter gated to run only while the line is low and zeroed by DMA1 CH7 at every rising edge, so its overflow at 9.25 bit-times is a break (§7) | one entry per break; ~16.5 µs with the framer's break body (ping, bench probe) |
| USART1 vector   | PFIC HIGH. TC = TX arm drained, the one enabled source (never a DATAR read; FE/NE/ORE have no interrupt enable - they latch silently, §7) | TC body ~2-4 µs/arm |
| SysTick CNT/CMP | the transport clock (48 MHz, 32-bit) + the ONE comparator | — |
| SysTick vector  | PFIC HIGH. Deadline mux: framer A/B, covered, chain trigger — and dispatch, inline (every class except verdict-first runs at the covered checkpoint or the fast path, §6) | arithmetic slots ~1–5 µs; dispatch bodies ~10–70 µs |
| DMA1 CH5        | USART1 RX → 512 B ring, circular, silent (no IRQ); **VERYHIGH, atop the ladder** (§7) | zero CPU |
| DMA1 CH7        | TIM2_CH2 (IC2, rising edge) → a RAM zero into TIM2 CNT, circular, one halfword per rising edge; VERYHIGH, below CH5 (§7) | zero CPU |
| DMA1 CH4        | TX arms → USART1 DR (header, snapshot payload, CRC tail); HIGH | zero CPU; TC surfaces as USART TC |
| DMA1 CH3        | CRC feeds → SPI1 DR, 16-bit halfwords (RX span straight from the ring); MEDIUM, below CH6 | zero CPU, ~0.36 µs/B engine time |
| SPI1            | CRC-16/ARC coprocessor (16-bit LSB-first, bitrev16 at the register), accumulates across feeds | runs ~8× wire speed (F6) |
| DMA1 CH6        | snapshot copy → the 256 B snapshot buffer (reply payloads only — RX CRC feeds the ring directly); HIGH, above CH3 | ~0.125 µs/B, zero CPU |
| DMA1 CH1        | ADC sample set → buffer; DMA HIGH (wins HIGH ties by channel number); TC vector = motor kernel tick at PFIC LOW | ~10 µs body |
| PC0 CNF         | drive discipline: open-drain listening / push-pull TX window | flipped at trigger/release |
| main loop       | deferred reboot poll + rescue line sampler (protocol sec 9.1: line pin + CH5 NDTR + own-TX state in one critical section per wfi wake, the window restarting while the servo transmits - the break detector fires once per span, a break-length in, so the slow loop is the only observer of a pulse's length) | cold path; sampler ~0.3 µs/wake |

PFIC preemption is two-level (IPRIOR bit 7). TIM2 + USART1 + SysTick
share HIGH and therefore serialize against each other; LOW holds only the
motor kernel (DMA1_CH1 = 22), which HIGH preempts and which runs in the
wire gaps between frames. Free and reserved: TIM1 (motor PWM), SW (14),
I2C1_EV (30), I2C1_ER (31), DMA1 CH2. The break detector holds DMA1
CH7, I2C1_RX's request channel.

**Kernel ticks under load — measured and accepted.** Everything-at-HIGH
means transport work preempts the kernel, and a kernel tick that pends
while a ≥50 µs HIGH chunk runs coalesces with the next one (the PFIC pend
bit is one bit). Silicon, against a 20.11 kHz idle tick baseline: tick
loss ≈ 1.2–1.4× the transport-HIGH duty — 14% under a sustained 9-frame
zero-gap flood (~11% duty), 23% under a 21-frame flood (~17% duty).
Latency stays bounded (the longest HIGH chunk is ~60–80 µs ≈ 1–2 ticks);
ticks are never starved outright. Accepted because the duty profile that
loses ticks is rare in practice: config sessions run torque-off (kernel
idle by definition), and the production hot loop is a few small GWRITEs +
COMMIT + GREAD per cycle — short bursts, wire-gapped, single-digit duty.
A use case that sustains heavy bus duty under live control is the signal
to revisit (hardware-counted ticks, or an isolation lane).

**Frames for other servos.** In a fleet most traffic on the wire is
somebody else's, and every servo pays for all of it, so a foreign frame
gets the least work that keeps the ladder honest. Its header is read at
deadline A like any frame's; from there a plain op addressed to another
id (from an idle reply pipeline) and a status no chain slot waits on
schedule no milestone (§5.2). The CRC verdict, with no feed ahead of
it and no decode, runs at the next wake, usually the next frame's
break - for the status too: its LEN moves the ladder, and a garbled one
would swallow the frames behind it (sec 5.3), so it costs what the op
costs (~13 µs of verdict, from the static count). A chain slot keeps the
frame end of the status it waits on, and only the end. Per host request plus
the reply it draws, a bystander pays two break wakes and three deadline
wakes (two headers, and the starve horizon that resolves the reply on a
quiet bus), where every frame used to pay header, covered and end. DES:
`bystander_entries_per_foreign_exchange_are_pinned`,
`chain_entries_per_gread_are_pinned`.

## 3. One exchange, tick by tick (ping at 1M; byte-time = 10 µs)

```
t=0    break detector overflow, 9.25 bit-times into the break; the 0x00
       rings at its stop-bit sample. TIM2 ISR - a pure wake: the
       framer resolves from ring data and projects deadline A = now + 3
       byte-times at wire pace + ½ byte for the break tail [F5]. A wake
       that beat its 0x00 waits one byte-time for it first (§5.1). No
       timestamp is recorded.
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
       packet_end + reply gap (12 µs, fixed at every baud). A staged write
       COMMITs in the same body, after the sequencing: the break never
       waits on the commit, and the commit still lands before the reply's
       trailing CRC arm can stream (its TC vector is HIGH too, so it pends
       behind this body). Body is a few µs — the work already happened.
t≈67   trigger (SysTick): INST finalized, PC0 → push-pull, break sent, first
       DMA arm armed. Status break falls. TX CRC is computed by the same SPI
       engine IN PARALLEL with transmission and patched into the final arm.
t=…    per-arm TC ISRs stream the remaining arms; final TC releases the wire
       and applies any deferred id/baud config.
```

Measured: 30.4 µs from instruction end to status break fall (§10). The
estimate chain above — reply-gap grid plus sub-byte estimate lateness —
accounts for roughly half; the remainder is trigger-body, break commit,
and ISR-entry overheads.

## 4. Why this is fast — first principles

1. **Dispatch before the verdict — the spine.** Dispatch never waits on
   the CRC. It runs the moment the covered span is ringed (frontier) or
   the frame is complete (backlog), and STAGES its effects; the CRC
   verdict then gates those effects, never the work. This is not an
   optimization layered on a "safe" path — it IS the default, for every
   class. The alternative (a non-dispatching path that schedules a CRC
   check and blocks on the result before doing any work) spends ~5–7 µs
   of HIGH per frame spinning on a finished engine, and in a zero-gap
   burst those spins stack onto the burst-cycle critical path and widen
   break-delivery lag — so no such path exists. The verdict gates two
   effect kinds: the **wire effect** (a staged reply — SEND on pass,
   DON'T-SEND on fail) and the **table effect** (staged writes — COMMIT
   on pass, REVERT on fail). Ping/read stage only a wire effect; a
   NOREPLY write only a table effect; a reply-bearing write both, under
   one verdict, wire effect first: the ack was decided at dispatch and
   the commit cannot fail, so the status break leaves before the commit
   body runs (a commit's post-commit bookkeeping is tens of µs on the
   chip), and the host still cannot resolve a complete status before the
   commit has landed, because the reply's CRC arm is streamed from the TC
   vector, which pends behind the verdict body.
2. **Work hides under wire time.** The dispatch window (covered
   checkpoint → frame end) runs decode + dispatch + reply build while the
   last two CRC bytes are still in flight. Deadline B — the only step on
   the reply's critical path — is reduced to "poll a finished CRC and arm
   the trigger". The reply rides the reply-gap grid, not the dispatch
   time.
3. **The verdict is almost always PASS.** Dispatch bets the frame passes
   CRC. On a clean bus that is ~100% of frames; the loser (a corrupted
   frame) pays a revert + don't-send, which is off the happy path by
   definition — and the frame-position ladder is untouched by it (§5), so
   the hunt just starts one hop later, on corrupt frames only, inside the
   host's retry contract.
4. **Hardware CRC in both directions, in parallel.** RX: fed at the
   covered checkpoint (or publish), chewing while the CRC bytes fly,
   polled at the verdict. TX: fed per arm, patched into the trailing arm
   before DMA reaches it. The CPU never computes or waits a full CRC.
5. **Zero hops, everywhere.** Every stageable class (ping/read/gread,
   write/gwrite) dispatches inline at HIGH — each stage hands off by
   falling through within one ISR invocation or the next event's entry,
   no cross-priority round-trip. The kernel-side cost is the measured,
   accepted tick coalescing in §2.
6. **Copy-once TX.** Reply payloads are DMA-snapshotted once
   (~0.125 µs/B, fire-and-forget) and streamed from the snapshot by both
   the wire and the CRC — snapshot-consistent reads for the price of one
   copy hidden under reply gap (protocol §4.2).
7. **One comparator, muxed.** Every deadline (framer A/B/covered, chain
   trigger, rescue) folds onto SysTick CMP with pend-on-past semantics;
   the drain loop consumes every slot due at the same wake.
8. **The single-context CRC engine needs no arbitration protocol.** TX
   generation and RX validation share one engine safely because both run
   at HIGH — ownership is serialized by the PFIC, for free.

## 5. Position and time from the stream

The transport's central design rule: **position is derived from the
stream, never sampled at ISR entry; times come from data-cadence
projections, never wake arrival.**

The rationale is staleness, not latency. A handler that samples the ring
cursor at ISR entry and classifies the event by `ring[cursor−1]` reads
the wrong byte whenever service lags the wire by a byte-time — and at 3M
(3.33 µs/byte) any dispatch body guarantees that lag, while two pended
breaks coalesce into one PFIC entry. The failure is silent: the cursor
has moved past the break byte, the test reads frame data, the framer
ignores the "garble", and a fully intact frame in the ring is never
anchored. A queue of cursor snapshots inherits the same staleness at
capture time — entry delay ≥ 1 byte-time makes the snapshot stale before
it is stored. Latency merely amplifies staleness; deriving position from
the stream removes the staleness, and latency stops meaning loss.

### 5.1 The break wake carries nothing

The break wake carries neither position nor time. It is a pure wake, and
its handler does only the bounded wire work: staged-reply kill (from
which broadcast-ENUM replies are exempt — colliding is their contract,
and they ride a UID-keyed slot delay so twin matchers don't collide in
undetectable unison, protocol §9.2) and chain suspend. Every aim and
estimate is instead projected from live ring state at each resolve:

```
aim / estimate = now + missing·tpb (+ span·tpb ≫ 6 drift pad)
```

`now` and the cursor advance together, so ISR delivery lag — the thing a
wake timestamp inherits wholesale (a 60 µs-late wake makes the reply
60 µs late) — cancels out of the projection. The error is bounded by one
byte-time plus one wake latency and is always on the LATE side: a byte
counted as missing can only finish sooner than assumed, so a reply grid
measured from the estimate never fires into its own frame's tail (the
reply-gap safety property, sim-pinned). The reply-grid estimate is frozen
at the covered checkpoint, where missing ≤ 2 makes it tightest. Aims are
self-pacing for free: a stalled wire re-projects at each wake ≥ one
byte-time out, so no recheck/park budget machinery exists; the
64-byte-time progress-idle horizon (the starve horizon) is the sole death
authority. One epsilon survives, on header aims only: the break rings at
its wake point ~4 bit-times before the line rises [F5], so the first data
byte sits that far outside the byte cadence.

The wake can beat its own byte. The detector fires 9.25 bit-times into
the break, ahead of the stop-bit sample that rings the `0x00`, so a wake
whose break byte is not yet the newest ringed byte gets its
ring-dependent service - the CAL ruler mark or drift stamp, the
resolver, the staged-reply kill - one byte-time later, at a third
deadline slot. The first byte ringed since the wake decides: a `0x00` is
the break, served at the wake's stamp (§8 pairs stamps, so they stay
the wake's); anything else, or nothing, only re-drives the resolver. No
position comes from the wake; the cursor it saw only says where to look.
DES: `tests/break_wake.rs` runs its pins with the wake ahead of its
byte, behind it, and alternating between the two.

### 5.2 The ring is the queue

Frames are contiguous: `next anchor = anchor + footprint` (stream
continuity). The cursor is read only at provably-quiet bootstrap moments —
boot (position 0 by construction) and rescue confirm (a held-low line
delivers no start edges, so the cursor is still); the ring is armed once
and never reloaded. From then on:

- **Fast path (successor available).** If newer activity exists beyond
  the current frame's computed end — its end byte is ringed and more
  followed — then every byte of this frame is already present. Resolve it
  purely from ring data: header, covered, CRC, dispatch, in one pass, no
  deadlines. A backlog of N frames is walked N frames at a time with zero
  scheduling; the 512 B ring (≥ 15 hot-loop frames) is the queue, and it
  cannot "drop" an entry the way a snapshot queue can.
- **Scheduled path (newest frame only).** The frame still arriving rides
  the deadline pipeline: A (header) → covered checkpoint (dispatch, §6) →
  B (verdict), every deadline a ring-cadence projection.
- **No milestone.** A frame that can change nothing until it is whole -
  another servo's plain op while the reply pipeline is idle, or a status
  no chain slot is waiting on - schedules nothing after A. The next
  wake resolves it from the ring like any backlog frame (it still gets
  its CRC verdict, which keeps the stride trusted), and only
  the starve horizon is armed for it, so a truncated one still dies by
  starve. DES: `truncated_foreign_frame_starves_then_recovers`,
  `snooped_slot_answers_a_reply_gap_after_its_predecessor`.

### 5.3 Garble dies by data only

Per the fault contract (protocol §3.4), no frame is dropped, killed, or
rejected on wake evidence — wake-time observations (service-time cursors,
latched re-fires) lie under exactly the conditions a busy fleet produces:
coalesced or lagged break service at high ISR occupancy, and latched-flag
re-fires after NOREPLY frames (nothing transmits, so nothing retires a
flag). A death decision derived from them plants itself inside live
frames. Data is the only death authority: a phantom candidate born from
garble is killed by its footprint-fill CRC verdict or by the starve
horizon — never by fault position. The cost is bounded, documented
recovery latency (the host pacing rule, protocol §3.4) instead of
fault-speed phantom kills; a phantom's CRC rejection flips the hunt on,
and the hunt converges from ring data.

Corruption recovery is bounded: a corrupted LEN mis-strides the ladder;
the next resolution finds no `0x00` at the expected anchor (or the frame
fails CRC) → drop + count, and the ladder re-verifies at every following
boundary. Anchors are parity-free (protocol §3.2), so recovery needs no
ring reload, only the next break's arrival. Loss is bounded to ≤ 2 frames
per corruption event; the host's timeout+retry contract closes the loop.

Statuses are no exception, though nothing acts on their bodies. A status
LEN garbled long would otherwise swallow up to 258 B of the frames behind
it, so every status is checked before the ladder steps past it. One a
chain slot waits on is checked for free: a uniform GREAD sizes every
predecessor's reply, so a LEN that is neither that span's nor an empty
error status's is a framing drop before it counts (a short one would time
the slot into its predecessor's tail; the slot answers at its reclaim
instead, flagged). Any other status gets the CRC verdict at the next
wake, like a foreign op. DES: `garbled_status_len_costs_only_the_status`,
`waiting_slot_drops_a_status_len_its_gread_rules_out`.

## 6. Instruction classes

The class split is *stageability* — whether an instruction's effects can
be staged behind the CRC verdict:

- **Stageable — PING/READ/GREAD/WRITE/GWRITE.** Dispatch inline at HIGH
  at the covered checkpoint (frontier) or the resolve wake (backlog); the
  CRC feed chews underneath. The verdict at the frame end gates the
  staged effects: SEND/DON'T-SEND of a reply, then COMMIT/REVERT of a
  table write (sec 4: the reply is sequenced first, the commit follows in
  the same body).
- **Verdict-first — COMMIT/MGMT.** Their effects cannot be staged (COMMIT
  applies the whole buffer; MGMT reboots), so the CRC is checked FIRST
  and dispatch runs only on a pass. Rare, short frames — the ~2 µs
  blocking CRC poll is affordable where staging isn't possible.
- **Unsolicited - TEL bursts (protocol sec 5.6).** The one place the
  servo transmits without an instruction on the wire. A committed
  nonzero `tel_count` arms it; the host is silent by contract until the
  LAST-flagged frame, so the reply TX engine and CRC engine are
  structurally idle and the burst borrows them whole - no second TX
  path exists. Mechanics: the kernel encodes each control-tick sample
  once, directly at its final wire offset in a ping-pong buffer pair
  (LOW context); the same tick's ISR-masked `poll_tel` stages a ready
  buffer through the ordinary stage/trigger path (HIGH-owned state). It
  runs at the tick, not in the main loop: a six-field frame leaves
  ~110 µs of its 800 µs batch window spare, and a driving tick starves
  the main loop for longer than that. Cross-context traffic is three
  flags and an arm mailbox, single-writer volatile discipline, no
  atomics. Any RX break aborts the burst with the speculation-kill trio
  (disarm, tx.abort, chain reset); buffers published under a dead arm
  epoch are discarded on sight, which closes the mid-tick re-arm race. A
  stalled consumer drops rows and counts them - the fast tick never
  blocks.

Backpressure is structural: at most one pending-verdict frame exists at a
time (the pending frame IS the frontier), so the single staging slot and
the single CRC accumulator are never contended. Own TX holds the same
accumulator from trigger to release, so the ladder resolves nothing while
a reply streams; the release resumes it from the ring, and a frame that
landed meanwhile (a peer talking over the reply) gets its verdict then.
DES: `status_verdict_waits_for_own_tx_release`. The kernel-isolation
question is settled by measurement instead of structure: dispatch bodies
preempt the kernel and coalesce ticks in proportion to bus duty (§2),
which the intended duty profiles make negligible.

## 7. The DMA priority ladder and receive-side discipline

The receive side's one hazard window is the RX-DMA drain of an inbound
byte racing the CPU: if the drain lags, the break wake enters with the
ring cursor not yet advanced past the byte — the resolver sees a
caught-up ladder and stays silent (no-reply). In isolation the drain is
effectively instantaneous versus the interrupt (zero
byte-still-undrained entries observed across soaks at 1M and 3M); the
window opens ONLY under DMA1-arbiter contention, where a competitor
delays the CH5 drain — 22% of frames at a moderate competing burst, 94%
at a heavy one. The arbiter preempts **per-beat** (RX stays clean even
against a 256-transfer competitor once it outranks it), so RX at the top
bounds its drain wait to a single in-flight transfer - a couple of
cycles, far under interrupt entry latency - regardless of the
competitor's burst length.

So the guarantee is the priority ladder, not a software mitigation:

- **VERYHIGH:** CH5 RX and CH7 detector zero. The zero is one halfword
  per rising bus edge and must land before the next low has counted the
  detector's last quarter bit-time on top of a 9-bit data low, about one
  bit-time (16 HCLK at 3M); RX wins the tie on channel number, so each
  waits at most one beat of the other. At HIGH it would lose every beat
  of a snapshot copy.
- **HIGH:** CH1 ADC, CH4 TX, CH6 snapshot (ADC wins the HIGH ties by
  channel number, keeping its interleave ahead of the copy).
- **MEDIUM:** CH3 CRC feed - below CH6 so a reply copy is written before
  the feed reads it.
- **free:** CH2.

With RX on top no ringed byte is ever missing from a cursor read. The
ladder cannot put the break's own `0x00` ahead of its wake, though - the
detector fires before that byte exists - and that case is the one-byte
re-inspection's (§5.1). Silicon: 25k/25k zero-gap hot loops at 2M and 3M
with zero no-reply/stale.

**The no-DATAR rule.** The ladder protects only the byte already in RDR.
A CPU DATAR read while a byte is mid-reception additionally kills the
byte in the SHIFTER — no flag, no ring entry, no NDTR count — and no
drain priority can help, because the byte dies before a DMA request ever
exists. The window scales with bit time: unhittable at 2M/3M, systematic
at 1M under one alignment. So nothing on the receive side ever touches
DATAR. Silicon: 12k/12k plain floods at 1M and 0.5M with no RX-path DATAR
read, versus 44/12k with one present.

**The break detector is the only receive interrupt.** HDSEL disables
the USART's LIN break detector on this silicon (protocol F16), so the
detector is TIM2 on the bus pin, and the hardware does all of the
qualifying. Remap 4 puts TI1 on PC0. Slave gated mode on TI1FP1 with
CC1P inverting it runs the counter only while the line is low. IC2,
mapped onto the same TI1, captures each rising edge, and its DMA request
(DMA1 CH7, circular) copies a RAM zero into the counter. The overflow
at 9.25 bit-times of continuous low (ARR 888/444/222/148 at
0.5M/1M/2M/3M, rewritten with every BRR write) raises the update, the
one enabled interrupt: an idle line and frame data never reach it.

A low that is still held when the service runs (often the break's own
tail, a rescue pulse, a stuck line) would
overflow again every 9.25 bit-times, so the service parks the counter
at ARR + 1: it must wrap through 0xFFFF, 65536 ticks of low, before it
can overflow again, and the rising edge's zero un-parks it with no CPU.
CC2IF, set by every rising edge and cleared only by the park, tells a
parked low's re-fire from a fresh break, so a low of any length is one
wake; nothing reads CH2CVR, whose read would clear it. A rising edge
between the pin read and the park, whose zero the park overwrote, shows
as the pin high on a re-read and is replayed by a software capture
(CC2G), which sets CC2IF and requests the zero as the edge did. The
servo's own break is sent with the update masked: the receiver hears
nothing of its own TX (F9), but the pin does. The per-character error
flags (FE/NE/ORE) are never serviced: EIE stays 0 from boot, so a latched
error flag pends nothing, storms nothing, and needs no retire protocol.
The flags self-clear incidentally when ordinary traffic pairs a STATR
read with the drain's DATAR access, but nothing observes or depends on
it; at most they may be polled from a cold path as line-noise telemetry
(protocol §3.4).

The rationale for waking on a length-qualified break detector rather
than the error flags is first-principles: the error flags are latched, positionless, and
coalescing, so a wake built on them must be throttled against garble
storms (wrong-baud traffic heard as continuous framing errors), and any
mute needs a restore path that itself cannot be starved — a structural
liability. A wake that never needs muting deletes the problem: the
detector is length-qualified (only a dominant span past 9.25 bit-times
fires it), so real errors never interrupt at all, and their consequences
surface exactly where data-driven handling already looks (§5.3).
Silicon: law breaks 100/100 at every rate, one entry each; zero entries
on an idle line; zero false breaks from frame data or faster-baud garble
(protocol F17).

## 8. Clock discipline

Chain snoop is servo→servo (protocol §9.3): slot k>0 times its reply off
the predecessor's snooped status frame, so the clock budget is
pairwise-HSI — two uncalibrated RC oscillators against each other.
Factory spread (7k+ ppm measured across five chips) garbles snooped
status tails at 3M; trimming is what makes high-baud chains work. The
host owns the only crystal on the bus, so it is the syntonization root:
the servo measures the host's cadence and slews HSITRIM toward it.

Two estimators feed the trim loop, both built on break-wake stamps at
frame boundaries — hardware-anchored and same-ISR-flavor, so entry
latency cancels in every pair. (Mid-frame progress stamps do not have
this property: they are whole-byte-quantized with software-chosen phase,
and they need frames far longer than real hot-loop traffic provides.)
These stamps are the one sanctioned exception to §5's no-wake-time rule —
the trim machinery's entire subject is the break-service stamp itself,
and it is gated, paired, and baseline-anchored accordingly (protocol
§3.4, §9.3). A break's service first resolves every frame that ended
before the break, then stamps: a frame with no milestone (§5.2) is still
in the ring when the next break wakes, and recording it after the stamp
would credit it to the wrong pair.

- **Absolute, rarely — MGMT CAL.** A broadcast instruction announcing N
  breaks spaced exactly T µs; the servo stamps `deadline.now()` at each
  break-wake entry, gates each gap at |Δ−T| ≤ T/16, and trims off the
  sum. ~±260 ppm from 8 × 400 µs gaps. Fired at boot (≥2 trains — the
  first identifies the chip's step effect, protocol §9.3 boot guidance)
  and at host-known events, not on a timer. Break decode is
  threshold-free across the full HSITRIM throw, so CAL also rescues a
  railed servo.
- **Differential, continuously — chain pairs.** Break-wake stamps of
  adjacent host instructions (GWRITE→COMMIT seams):
  `err = measured − footprint·tpb = seam + drift·span`. The host's
  queuing seam is unknown but stationary; a baseline captured right after
  each trim application subtracts it, so windows read pure drift. Any
  constant — seam, detector latch offset, ISR flavor residue — dies in
  the subtraction; only changes survive, and the sanity band catches
  non-thermal jumps.

Both feed the `TrimLoop`: `steps = round(err / step_effect)`, clamped ±4;
round-to-nearest IS the deadband. The step effect is self-measured (chip
steps are nonuniform, 1.4–3.2k ppm; sanity-banded 0.8–4k); the total is
applied via HSITRIM between frames and mirrored read-only at
`telemetry.clock.trim_steps`. Wire format, qualification rules, and the
tracker's baseline mechanics are normative in protocol §9.3. DES:
`tests/trim.rs` — trains converge/reject/watchdog, the tracker follows
mid-run drift injection, constant seam+skew cancel exactly, solicited
shapes never pair, and alternating footprints pair each frame with its
own breaks (`tracker_pairs_each_frame_with_its_own_breaks`).

## 9. Losslessness

The zero-gap argument, from first principles:

1. **Capture.** Every byte is DMA'd regardless of CPU state; breaks pend
   and may coalesce, but §5 derives frame positions from the stream, so a
   coalesced or delayed wake costs nothing — the fast path recovers every
   completed frame from data.
2. **Backlog.** Bounded by the ring (512 B ≈ 15 minimal hot-loop frames).
   The consumer outruns the wire (~2× worst case); sustained overrun is a
   THROUGHPUT invariant, pinned by the discrete-event flood test
   (`zero_gap_flood` in `hot_loop.rs`: 100 zero-gap minimal writes +
   READ, ring laps ~2×, dispatch-cost sweep inside the contract, zero
   loss at 1M and 3M), not by a runtime guard.
3. **Ordering.** At most one pending-verdict frame at a time (§6), so the
   in-order ring walk preserves frame order. A backlog write commits
   before the frame behind it dispatches — DES-pinned by
   `backlog_write_then_read_processes_in_order`.
4. **The host's side of the contract.** RESPONSE_DEADLINE must cover the
   full reply path — decode + dispatch + verify, elastically late under a
   backlog — not just the happy-path grid: with ~70 µs dispatch bodies a
   60 µs default is dishonest, and a chain slot reclaims into a
   live-but-slow predecessor (DES-pinned in `hot_loop.rs`). Deployments
   tune the register to their measured worst case.

## 10. Measured turnarounds

Turnaround = instruction wire-end → status break fall. Read = 16 B READ,
write = goal_position 4 B WRITE. Flash-layout swings between builds are
±5 µs.

| baud  | ping    | read 16 B | write 4 B |
|-------|---------|-----------|-----------|
| 0.5M  | 32.2 µs | —         | —         |
| 1M    | 30.4 µs | 38.6 µs   | 87.7 µs   |
| 2M    | 38.8 µs | —         | —         |
| 3M    | 41.3 µs | —         | —         |

Under burst load the elasticity is small: hot-loop mean turnaround at 1M
is 35.4 µs, plain-flood 31.3 µs.

The shape: the reply's tail after the instruction's wire end is
`max(serialized CPU pipeline, reply-gap grid)`. The grid is flat (12 µs
at every baud, protocol §7) and the estimate it hangs off is
delivery-noise-free, so 0.5M/1M pings sit together — grid-bound. 2M/3M
are pipeline-bound: the covered window shrinks below the dispatch body —
at 3M a short frame is fully ringed before the first deadline wake even
fires — so the pipeline serializes after the frame end.

## 11. Glossary

- **break** — line held low ≥ a byte-time; the out-of-band frame
  delimiter. Rings as one 0x00 byte and fires the break detector - the
  transport's only receive wake (FE latches too, unserviced). Break framing is the strong
  form of sync: DXL 2.0's `FF FF FD` hunting + byte stuffing is the
  workaround for not having an out-of-band delimiter at all.
- **garble** — line damage that is not a break: a corrupted byte (slot
  occupied, CRC will fail) or a phantom byte (noise-invented byte between
  frames). Raises no wake at all (lows under 9.25 bit-times are
  invisible to the detector, errors never interrupt); dies by data
  (§5.3).
- **anchor / footprint** — a frame's start index in the ring / its total
  ring length including the break byte.
- **covered span** — everything the CRC protects: the frame minus its two
  trailing CRC bytes.
- **covered checkpoint** — the moment the covered span is fully ringed
  (2 byte-times before frame end); the dispatch point.
- **dispatch (the spine)** — decode + dispatch + reply build performed
  before the CRC verdict — the default for every class, not an
  optimization (§4). Effects STAGE; they do not apply until the verdict.
- **verdict** — the CRC result at the frame end, gating the staged
  effects: the **wire effect** (a staged reply — SEND on pass, DON'T-SEND
  on fail) and the **table effect** (staged writes — COMMIT on pass,
  REVERT on fail). Ping/read stage only wire; a NOREPLY write only table;
  a reply-bearing write both, under one verdict, sequenced then
  committed (sec 4).
- **instruction class** — stageability (§6): *stageable*
  (ping/read/gread/write/gwrite — dispatch inline at HIGH, effects gated
  by the verdict), *verdict-first* (commit/mgmt, CRC checked before
  dispatch).
- **deadline A / B** — header-readable check / frame-end check, both
  ring-cadence projections (`now + missing byte-times`) re-derived at
  every wake (§5).
- **reply gap** — the mandated wire gap between a frame and its reply
  (12 µs, fixed at every baud — host TC→release is time-domain, protocol
  §7); the reply grid.
- **fast path / scheduled path** — §5.2's split: complete-in-ring frames
  resolved from data with no clock / the newest frame timed by deadlines.
- **starve horizon** — 64 byte-times of ring silence on a pending frame;
  the fallback death authority for garble-born candidates (§5.3).
