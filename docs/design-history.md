# Design history — what we tried, and what it taught us

The transport did not arrive at its shape by foresight. It got there by
building the obvious thing, measuring it, and deleting it — repeatedly. This
file is the itemized record of the abandoned paths and the lessons they
distilled, so a reader gets the war stories without the git archaeology. The
arc in one line: the firmware first spoke Dynamixel Protocol 2.0, made it
work, and then replaced the protocol itself once we accepted that we control
both ends of the wire.

Companion pillars: `osc-native-protocol.md` (the wire),
`osc-servo-transport.md` (the receiver), `driver-pattern.md` (the layering).

## The protocol pivot

### Dynamixel Protocol 2.0 as the wire protocol

We implemented and tuned a complete DXL 2.0 stack — parser, hardware reply
timing, Fast chain CRC, clock calibration — and it worked: 62.8 µs measured
ping turnaround at the end of its tuning arc. We replaced it anyway. Every
hard problem in that stack traced to one wire-format fact: DXL has no
out-of-band delimiter, so it needs `FF FF FD` header hunting, byte stuffing
to keep the header out of payloads, and a stuffed length that isn't known
until the whole payload is walked — which forbids streaming TX outright. Add
10–12 bytes of overhead per frame, the RDT reply-grid tuning surface, and
byte-granular indirect registers that force replies through a per-byte
pointer chase (defeating both the hardware CRC engine and copy-once TX), and
the stack's complexity was the protocol's, not ours. A UART break is an
out-of-band delimiter: framing became 5 bytes + a break, the receiver became
a two-state framer, and the stuffing, hunting, and reply-grid subsystems
stopped existing. Lesson: when you control both ends of the wire, fix the
wire — a delimiter the data cannot forge deletes entire subsystems.

## Receive machinery built, measured, deleted

### Per-byte streaming RX parser

The first receiver walked the wire byte by byte: an event-per-protocol-field
iterator pumped into a stateful dispatcher that tracked each in-flight
instruction across the event sequence. The per-byte pipeline measured ~55%
of the CPU during sustained 3 Mbaud reception — spent producing a
granularity no consumer used. Reply timing needs one number per frame, chain
sequencing needs frame boundaries, drift estimation needs a rate over a
burst. Deleted for whole-frame classification: probe the header, copy once,
decode once, one verdict per frame. Lesson: precision nobody downstream
spends is pure cost — find what consumers actually need before building the
producer.

### Hardware input-capture RX timing

Early bringup timestamped every falling edge on the RX pin through timer
input capture: a DMA-drained edge-stamp ring and a classifier walker turned
edge times into per-byte wire ticks. It worked, and it was the single
largest CPU consumer in the transport. All ~3000 lines were deleted and RX
timing became fully software once measurement showed every timing consumer
met its tolerance from plain clock reads. Lesson: RX jitter does not
propagate to the wire — only TX jitter does. Spend hardware where error
reaches the wire; software is fine everywhere else.

## Transmit timing the break made obsolete

### Hardware-timed TX kickoff + RDT

To hit DXL's reply grid, a timer-compare match fired a DMA write that
enabled the TX stream — zero CPU in the deadline path — backed by
per-fire-path lead-calibration constants and a wrap-guard arming dance. All
of it existed because DXL schedules replies on a clock grid. Break framing
made reply timing event-driven: send the break when ready, and the break
itself tells every listener a frame is starting. The kickoff hardware, the
RDT register, and the whole lead-tuning surface were deleted. Lesson: the
best exit from a precision-timing problem is a protocol that doesn't pose
it.

## Clock discipline: the long road to CAL

### Fire-intercept calibration (DXL era)

The chip has no crystal, and its RC oscillator drifts enough to break
chain timing at 3 Mbaud. The first calibration was a three-knob
decomposition — HSITRIM for the drift slope, a per-family structural-latency
intercept, a per-chip sub-trim residual — derived host-side from a
crystal-clocked timer stamping a stimulus reply. It was correct and it was
heavy: biased rounding rules, Q8.8 unit plumbing, per-family
characterization sweeps. The break-framed protocol deleted its main
consumer (the reply grid), and the replacement ruler is one broadcast
instruction: the host emits breaks spaced exactly T µs apart and the servo
stamps its own wake entries — same ISR path both ends of every gap, so
entry latency cancels in the difference. The lesson survives unchanged: a
chip cannot time itself. SysTick, the UART, and ISR latency all derive from
the oscillator under test; every self-measurement is structurally biased.
You need a reference on a clock domain that isn't the one being measured.

### Passive drift estimator

Between calibrations, thermal drift still needs tracking. The first tracker
eavesdropped: it sampled the host's byte cadence at mid-frame resolver
wakes. It starved — the frames long enough to qualify barely exist in real
traffic — and its software-phased stamps needed a dither/floor/gate pipeline
just to launder their quantization. Replaced by the differential chain-pair
tracker: break-wake stamps bracketing one CRC-verified silent instruction,
with a post-trim baseline subtraction that kills every constant (host
queuing seam, detector latch offset, ISR flavor residue) so only drift
survives. Lesson: when only the host has the truth — it owns the crystal —
ask the host to cooperate instead of eavesdropping on it.

## Position and time from wake evidence: three dead ends

### Capture FIFO / deferred dispatch

Zero-gap bursts lost frames, and the diagnosis pointed at ISR latency: a
long dispatch body delays break service, breaks coalesce, the late handler
misreads the ring. So we queued — a capture FIFO that snapshotted the ring
cursor at each event for later processing. Reverted: probes proved the
snapshots were already stale *at capture time*; fixing latency attacked the
wrong term. The design rule since: position is derived from the stream,
never sampled at ISR entry. Latency merely amplifies staleness — remove the
staleness and latency stops meaning loss.

### Per-event timestamps → ring-cadence time

The break wake once carried one timestamp — its own delivery tick — as the
frame's clock anchor. Everything timed from it inherited ISR delivery lag
wholesale: a wake serviced 60 µs late made the reply 60 µs late. Replaced by
projecting `now + missing byte-times` from live ring state at every resolve.
`now` and the ring cursor advance together, so delivery lag cancels out of
the projection; the residual error is bounded under a byte-time and always
on the late side, never early. The wake carries nothing now — not position,
not time. It is only a wake.

### Error-flag wake + fault fence + storm mute

The receive wake originally lived on the UART's framing-error flag, and we
twice shipped machinery that extracted more than a wake from it: a
positional fault fence (record where the error was seen, kill frames behind
it) and a service-time storm mute (throttle a flag that re-fires). Both
passed every quiet-wire test. Both killed live frames under fleet load —
service-time cursors lie under ISR lag and latch re-fires — and the mute's
restore chain wedged servos deaf to rescue. The resolution inverted the
premise: wake on the length-qualified break detector, and leave the error
flags with no interrupt enabled at all. Errors never interrupt; their
consequences surface where data-driven handling already looks (a corrupt
byte fails its frame's CRC). Lesson: latched, positionless, coalescing
signals cannot be event sources — don't build event machinery on one.

## Dispatch placement

### Class-routing carve-out (heavy dispatch at low priority)

To keep long dispatch bodies from preempting the motor kernel, heavy
instruction classes were routed across a priority boundary: staged through a
one-slot handoff to a low-priority consumer. It bought kernel isolation and
cost ack-bearing writes a cross-priority hop and an adoption round-trip —
and the handoff machinery (slot FSM, two wake lanes, adoption guard) became
the single largest complexity center in the transport. Deleted, ~800 lines,
once silicon measurement settled the question the structure was built to
avoid asking: kernel tick coalescing under realistic bus duty is negligible.
Lesson: measure the cost before building structure to avoid it.

## Smaller calls that shaped the design

### Torque-gated EEPROM writes

DXL gates every config-region write behind torque-off — a category error
that conflates the storage medium with the data's mutability. The actual
mid-motion hazard is the flash program operation, which stalls instruction
fetch for milliseconds under a live control loop. So only SAVE is gated;
config writes always land in the RAM table, volatile until saved. Flash wear
drops to program-per-session and the write stall happens exactly once, at a
moment the user chose. Lesson: gate the hazard, not the category.

### Async/await on the servo side

Rust async was considered for the bus driver and rejected. Executor
wake→poll overhead per frame is real money against a ~41 µs turnaround
budget on a 48 MHz RV32EC, and async costs control of code placement — hot
paths must land in RAM sections deliberately, which the executor's generated
state machines defeat. The driver is instead a composed set of explicitly
scheduled state machines driven by two interrupt vectors: the scheduling is
the design, not plumbing to be abstracted away.

### Framing alternatives that lost to the break

Recorded so they are not re-derived. Closing double-break (a break at both
ends of every frame): deletes every receiver clock, but costs wire time per
frame and gives up dispatching under the instruction's own wire time.
Covered-position break (delimiter at the payload/CRC boundary): a
wire-native dispatch trigger, but it resurrects the commit/revert machinery
it was meant to delete. Sync bytes, header checksums, parity: each adds wire
cost to buy detection the break + CRC + host-retry contract already
provides, and none deletes receiver machinery. Break framing is the strong
form of sync — DXL's `FF FF FD` + stuffing is the workaround for not having
an out-of-band delimiter at all.

## Recurring lessons

- **Measure before structuring.** Twice we built structure to avoid a cost
  nobody had measured; both times the measurement, once taken, deleted the
  structure.
- **Delete machinery rather than tuning it.** Every major win here was a
  subtraction; a fix that adds a knob is usually aimed at the wrong term.
- **Data over wake evidence.** Positions come from the stream, times from
  cadence projections; nothing derived at ISR entry survives load.
- **A clock needs an external reference.** Every self-measurement path
  shares the oscillator under test.
- **Gate hazards, not categories.** Find the actual dangerous operation and
  gate exactly that.
- **Precision nobody spends is pure cost.** Identify what consumers need
  before building the producer.
- **When you control both ends, fix the wire.** Complexity moved into the
  wire format's construction is deleted from every implementation, forever.
