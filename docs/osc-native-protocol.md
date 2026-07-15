# OSC-Native Protocol

The normative specification of the osc-native servo-bus protocol:
break-framed, 5 overhead bytes per frame, hardware-CRC-friendly, designed
to run whole on sub-$0.20 MCUs. Every physical-layer behavior the design
leans on was measured on real silicon (V006 servo, V203 HSE host); the
measured facts are collected in §11 and cited inline as [F1]..[F15].

## 1. Goals and non-goals

The philosophy in one line: **an efficient wire and a quick turnaround —
low latency as the product of both.** Spend the fewest bytes per exchange,
and never make the wire wait on the CPU. Simplicity is the mechanism, not
a trade-off: every byte and every microsecond this protocol saves over
DXL comes from deleting machinery, not adding it.

Goals, in priority order:

1. **Simplicity** — the servo-side transport should be a two-state framer, a
   counted DMA ring, and a dispatcher. No per-byte parsing, no byte stuffing,
   no unstuffing pass, no hardware-timed reply grid, no RDT tuning surface.
2. **Wire efficiency and turnaround** — 5 overhead bytes + a break per
   frame (DXL: 10–12 plus stuffing), and length is known two bytes in, so
   dispatch and reply staging overlap the instruction's own wire time; the
   reply's first byte waits on nothing — not a completed encode (streaming
   TX), not a folded CRC (hardware CRC engine), not a reply grid
   (enable-when-ready).
3. **Cheap-MCU fit** — everything must run on the V006 tier: one UART, one
   DMA ring, the SPI block as a CRC engine, no input capture, no crystal.
4. **Recoverability in the field** — a servo must be reachable regardless of
   its configured baud or ID (rescue break, UID enumeration).

Non-goals: DXL wire compatibility; multi-host arbitration (a single host
schedules the bus); encryption/auth.

## 2. Physical layer

Single-wire half-duplex TTL bus, 3.3 V, host-scheduled (exactly one talker
at any time by protocol construction). Two bus roles, and the names mean
the roles: the **host** is the end that schedules every exchange; a
**servo** is any addressable device that responds — a bus-device role, not
a motor (a sensor node or a downstream gateway speaks the servo role
unchanged).

- **Servo pin**: the USART TX pin with `HDSEL` (single-wire mode). RX is
  internally tied to the pin; the direct wire needs no dedicated RX pin and
  no direction buffer [F7] — rev C omits both. Bus side: series R +
  pull-up (+ optional TVS); the buffer's roles collapse into the drive
  discipline below.
- **Buffered boards (rev B)**: the default board config (`half-duplex`
  selects the direct wire — tinyboot's flag, same convention).
  The USART runs plain full duplex behind the 74LVC2G241: TX drives only
  the buffer input (push-pull, never released), and TX_EN gates the wire —
  high drives TX onto the data line and hardware-mutes the receive path
  (inverted enable, same signal), low releases it to the board pull-up.
  Same observables as HDSEL: no own-TX echo, RX held at mark through the
  TX window. The drive discipline below applies to the wire side of the
  buffer — TX_EN assert is the claim, TX_EN release the handback.
- **Drive discipline (all nodes, host included)**: idle/listening = AF
  open-drain (wire released, pull-up holds mark); transmitting = AF
  push-pull for the duration of the frame, then release. One GPIO CNF write
  each way. A node that idles push-pull clamps every other talker [F8] —
  this rule is the buffer replacement, not an optimization.
- **Own-TX echo**: none on V006 — HDSEL gates RX during TX [F9]. Firmware
  never needs echo masking. (Chips that do echo would mask in the framer;
  the protocol itself is agnostic.)
- **Baud**: operational baud is a config register selecting from
  **{0.5 M, 1 M, 2 M, 3 M}**, default **1 M**. DXL's legacy low rates are
  pointless on this bus — recovery is the rescue break's job (§9.1), not a
  crawl-speed fallback. Servos run HSI; the host must be crystal-clocked.
  Measured margin: the V006 cannot be detuned far enough (±3.4 % full
  HSITRIM throw) to break 3 M framing or data in either direction [F10] —
  ≥3× the trimmed-HSI ±1 % budget, and lower rates only widen it.
- **Rescue baud**: 0.5 M — the floor of the option set, not a fifth rate.
  Rescue must work anywhere the protocol can work at all: a bus that can't
  carry the lowest operational rate can't run any configuration either, so
  the floor is by construction sufficient. Entered only via rescue break
  (§9.1).

## 3. Framing

A frame is delimited by a **UART break**. **Protocol law: a transmitted
break is exactly 10 bit-times dominant — one character time.** The shape
is one 9-bit `0x00` character (M=1, bit 8 = 0): start + 9 data lows =
10 low bit-times, then a clean stop bit. Unforgeable by data (every UART
byte contains a high stop bit within 10 bit-times); no sync header, no
byte stuffing, no content restrictions.

Why a law and not a floor: break ≡ one character time keeps every
timing model exact — the framer's footprint algebra and the §9.3
chain-pair gates count the break as one byte slot, so an over-long
break is a constant error tax on every span — and the law shape is
precisely the LIN break definition (LBDL=0), so any LIN-capable
receiver gets hardware break detection with a deterministic 10-bit
anchor. Every node on the bus uses it: bridge-class hosts
and the servo itself (LBD-sans-LINEN break wake, §3.4 — the detector
runs with the LIN engine off on the target silicon [F15]). Hardware
`SBK` is off-law (~14 bit-times measured, F5).

**Receivers stay length-tolerant**: any ≥10-bit dominant span is one
break (rescue pulses, garble, and F3 all require this). Both ends
transmit the law shape: the host sends it directly, and the V006 reply
path sends the bracketed-M `0x00` character rather than hardware `SBK`
(same blocking shifter contract, 3 bits shorter).

Measured break behavior that the framer relies on:

- The break detector (LBD, LINEN off) fires 1:1 per law break and the
  break rings as exactly one `0x00` byte via DMA [F15][F1][F2].
- A break of _any_ length is exactly one event — the detector re-arms only
  on a fresh falling edge, so long breaks cannot spam [F3][F15].
- A mid-frame framing error does not halt reception: the garbled byte
  rings and the stream continues [F4] — and it raises nothing at all
  (sub-10-bit lows are invisible to the detector, and no interrupt is
  enabled on the error flags, §3.4 [F15]). Ring + NDTR are the only
  ground truth.
- Hardware `SBK` sends ~14-bit breaks (4.7 µs at 3 M, zero variance,
  both chip families) [F5] — which is why the law shape is a 9-bit
  `0x00` character, not `SBK`; receivers accept both (≥10 = break).

### 3.1 Frame anatomy

Both directions use one shape (symmetric framing keeps the framer identical
for hosts, servos, and chain-snooping peers):

```
BREAK | ID | LEN | INST | payload[0..p] | CRC_lo | CRC_hi
```

- `ID` — 1 byte. `0x01..0xF9` unicast, `0xFE` broadcast, `0x00`/`0xFF`
  invalid (never valid on the wire: `0x00` is the break's ring byte, `0xFF`
  is idle-line noise), `0xFA..0xFD` reserved. In status frames, `ID` is the
  responder's ID.
- `LEN` — u8: count of bytes following it (`INST` + payload + CRC =
  `3 + p`, any value ≥ 3). Frame end is knowable at byte 2:
  `end = len_pos + 1 + LEN`. Max payload is 252 bytes — deliberately
  sized so the largest legal frame (258 ring bytes) fits whole in the RX
  ring, which deletes chunked consumption from the framer (§4.1) and all
  per-transfer capability limits (§5.1). Fleet-scale group ops fit
  comfortably (§5.1); bigger transfers split into frames. A corrupted
  `LEN` cannot wedge the framer: the frame fails CRC at deadline B, and
  any subsequent break re-anchors (the break wake fires in LOCKED too).
- `INST` — bit 7: `0` = instruction, `1` = status (snoopers classify frames
  without state; there is no status opcode). Instructions: bits [6:4]
  opcode, bits [3:0] flags (§5). Status: bits [6:2] result code, bit 1
  reserved (0), bit 0 = ALERT (§5.3).
- `CRC` — little-endian osc-CRC-16 (§3.2) over the frame bytes
  `ID .. payload`.

### 3.2 osc-CRC-16

The covered bytes are **the frame bytes `ID, LEN, INST, payload`** —
`3 + p` bytes, any parity, in natural wire order. Over that span:

**CRC-16/ARC**: poly `0x8005` reflected (table form `0xA001`), init
`0x0000`, reflected input and output, no output XOR. Check value:
`crc("123456789") = 0xBB3D`. This is the textbook CRC-16 — any catalog
implementation works verbatim; no prefix, no byte swapping, no framing
quirks.

Hardware rationale: the V006 SPI CRC unit in **16-bit LSB-first mode**
(poly register `0x8005`) shifts each little-endian halfword low byte
first, low bit first — the exact bit order the UART itself puts on the
wire — so DMA feeds the byte buffer unmodified and the engine computes
the reflected CRC natively, accumulating across split DMA arms (ring
wrap), at ~0.36 µs/B wall and ~zero CPU [F6][F11]. The engine's register
holds the **bit-reversed** checksum (its shifter mirrors the reflected
algorithm); the chip bit-reverses the 16-bit result once per frame
(~40 cycles via the multiply trick — no table) when patching TX bytes or
comparing an RX verdict. Verified on silicon [F6]:
`TCRCR("12345678") = 0xB93C = bitrev16(ARC = 0x3C9D)`.

The engine's halfword DMA appetite (even start address, whole halfwords
[F12]) is satisfied **by construction, at any frame parity**, because a
leading `0x00` is a mathematical no-op under this flavor (`init = 0`:
zero state shifting zero bits stays zero):

- **Feed start** — the covered span begins at `ID = anchor + 1`, and one
  of `anchor` / `anchor + 1` is always even. Even anchor: feed from the
  anchor, the break's ring byte leads as a no-op. Odd anchor: feed from
  `ID` directly. The break byte is a free alignment shim, included or
  excluded as parity demands.
- **Feed end** — a span with a trailing odd byte feeds its even bulk by
  DMA; the last byte is folded into the read-back CRC state in software
  (8 reflected shift steps, ~30 cycles — the only software CRC that
  exists on the servo).

On TX the frame buffer keeps a literal `0x00` at offset 0 for the same
alignment (CRC DMA reads from offset 0, UART TX DMA from offset 1: one
buffer, two channel MARs, no copies); an odd payload feeds `p − 1` bytes
by DMA and folds the last at CRC-patch time. These are silicon
conveniences, **not protocol**: the wire checksum is defined purely over
`ID .. payload`, and nothing at the wire level constrains length or
position parity.

Consequences worth naming: frame anchors may land at any ring parity
(there is no even-anchor invariant); the RX ring is armed once at boot
and **never reloaded**; a bare break on the bus is harmless (its lone
ring byte shifts parity, which does not matter); and a mid-frame FE
costs at most the one garbled frame [F4].

Test vectors (covered bytes → CRC):

```
01 03 10                 → 0xFC50  (PING id 1)
05 07 30 80 01 2C 01     → 0xB3B1  (WRITE id 5, addr 0x0180, data 2C 01)
03 07 20 00 02 08 00     → 0x7015  (READ id 3, addr 0x0200, count 8)
02 06 30 00 01 AA        → 0x0D07  (WRITE id 2, addr 0x0100, data AA; p=3, LEN even-legal)
```

### 3.3 Resync

Any CRC failure, LEN overrun, or framing anomaly drops the frame and
returns the framer to HUNT; the next break is a hardware resync point.
There is no FF-FF-FD-style hunting cold path — the break _is_ the hunt.
A mid-frame FE costs one frame (CRC rejects it), never the stream [F4].
What a framing anomaly may and may not tell the implementation is
normative — see §3.4.

### 3.4 Fault contract (normative)

The target silicon exposes two distinct receive-side signals, and the
protocol binds them to two distinct roles [F15]:

- **The break detector (LBD) is the wake.** It is length-qualified —
  only a genuine ≥10-dominant-bit span sets it, the exact §3 law shape —
  it runs with the LIN engine off, it clears by a safe flag-selective
  write (no data-register access), and any-length span raises exactly
  one event, **latched at the span's END** (the rising edge; for the
  10-bit law break that IS bit 10) [F15]. It is the ONLY receive
  interrupt an implementation enables.
- **The per-character error flags (FE/ORE/NE) are not events at all.**
  They are latched, positionless, coalescing, and unsafe to retire
  mid-stream — so no interrupt is ever enabled on them. They latch
  silently, self-clear incidentally under DMA drain traffic, and MAY be
  polled from a cold path as line-noise telemetry; nothing else may read
  them, and no decision may derive from them.

A real error therefore never interrupts anything. Its consequences
surface exactly where data-driven handling already looks: a corrupted
byte fails its frame's CRC verdict (drop + count, no reply, host
timeout+retry); a noise byte between frames rings into junk the next
break's resolution scans off; a corrupted LEN mis-strides and dies at
the next geometry check or CRC. Reception itself never halts on an
error character [F4].

The contract, binding on every implementation of this protocol:

- **Positions come from ring data only** (§4.1). A break event MUST NOT
  be assigned a position; no frame may be dropped, killed, or rejected
  on wake evidence. Data is the only death authority: CRC verdicts,
  geometry checks, and the starve horizon.
- **Times come from data-cadence projections only** (`now + missing
  byte-times`). A wake's arrival time MUST NOT enter any timing grid.
  (The one exception is the §9.3 trim machinery, whose entire subject is
  the break-service stamp itself — gated, paired, and baseline-anchored
  there.)
- **Breaks are not countable events.** Service can lag the wire, and N
  breaks can coalesce into one service; all break handling MUST be
  idempotent, and freshness (did bytes ring since the last service?)
  MUST be derived from the ring, never assumed.

**The accepted limitation** (the price of break-framed delimiting):
garble that forms a plausible frame header parks the resolver until data
kills it — footprint-fill CRC (≤ 258 bytes) or the starve horizon (64
byte-times of ring silence), whichever comes first, per plausible junk
anchor. The length qualification shrinks that surface: only garble
containing a genuine ≥10-bit dominant span (slower-baud traffic heard at
a faster-configured servo) can wake the resolver into junk at all —
noise and faster-baud garble ring silently and cost nothing until the
next real break [F15]. **Host pacing rule:** after traffic a servo may
have received as garble (wrong-baud probing, bus glitches), allow one
starve horizon of bus silence before expecting crisp turnarounds; under
continuous zero-gap retries, replies can lag by up to the parked span
until a gap appears.

## 4. Servo RX/TX paths

### 4.1 RX framer

One circular RX DMA channel, armed once at boot; NDTR is read as a
cursor, never reloaded (reloading a circular DMA channel drops or
latches in-flight requests; and since anchors may sit at any parity,
§3.2, nothing ever needs a reload). Two states:

- **HUNT** — on the break wake (LBD): record the anchor = current ring
  position (the `0x00` just ringed). Enter LOCKED.
- **LOCKED** — deadline A at anchor + 3 byte-times + a half-byte-time of
  wake slack: the full header (ID, LEN, INST) is in the ring — read it,
  compute frame end, prime the dispatcher from INST, set deadline B at
  end + margin. At B: confirm NDTR reached the end, CRC-check, dispatch.
  Short/failed → HUNT.

Deadlines come from SysTick compare; both are computed, not discovered —
"frame end is predictable at header time." Dispatch and reply staging run
under the instruction's own remaining wire time.

Every frame fits whole in the ring by construction: the largest legal
frame is 258 ring bytes (§3.1) against a 512 B ring. LOCKED is therefore
the entire framer — two deadlines, no per-chunk work, no HT/TC draining,
no staging buffers. The dispatch budget is generous: after deadline B the
frame's bytes stay valid until the host sends another ~254 bytes (~850 µs
at 3 M, more at lower bauds) — and a scheduling host is awaiting the
reply anyway.

### 4.2 TX path

Reply buffer: `[0x00][ID][LEN][INST|0x80][payload][crc][crc]`
in a halfword-aligned static — the `0x00` at offset 0 is an alignment
byte and CRC no-op; an odd payload's last byte folds into the CRC in
software at patch time (§3.2). Sequence: flip pin to
push-pull → law break (§3, a bracketed-M `0x00` character) → enable
UART TX DMA from offset 1 → simultaneously
enable SPI-CRC DMA from offset 0 → the CRC engine outruns the wire 8:1,
so `TCRCR` is patched into the trailing CRC bytes long before the shifter
needs them (fire-first, append-later, no deadline race) [F6]. On TC:
release pin to open-drain.

No hardware-timed kickoff: TX start is "enable the channel when ready" —
the break makes reply timing non-critical, which deletes the TIM-compare
kickoff machinery, the RDT register, and its whole tuning surface.

Payloads at or below a small threshold copy into the reply buffer
directly — cheaper than arming a DMA channel for a couple of bytes. Larger
payloads (the READ/GREAD case) are **copy-once**: staging the reply kicks
off a fire-and-forget copy DMA that streams the table span into a
dedicated 256 B engine-owned snapshot buffer; the CRC feed and the wire TX
arm, armed separately when the reply triggers, both read from that
snapshot instead of the table. Copy, CRC feed, and wire shift-out are
three independent DMA/hardware engines running concurrently, not a
blocking chain — nothing polls or waits on the copy. Correctness comes
from relative speed and scheduling order instead: the copy is kicked off
earliest (at stage time, ahead of the trigger) and is also the fastest of
the three (plain M2M DMA outruns the CRC engine, which itself runs ~8×
wire speed, F6), and its channel (CH6) sits above the CRC-feed channel on
the bus-arbitration ladder (the RX ring alone owns the top — see
`osc-servo-transport.md`, DMA priority ladder) — so the copy is
guaranteed done before either downstream consumer reaches the bytes it
needs, by construction, not by synchronization. None of this costs CPU:
all three engines run in hardware, freeing the core to run the motor
kernel tick underneath. The one copy still buys two things a
direct-from-table stream couldn't: the table address's parity becomes
irrelevant to the wire/CRC engines (the snapshot is always
halfword-based regardless of where the source span sits), and every
large reply carries a consistent point-in-time image even if a
control-loop write lands mid-span. Reads over 252 B split into multiple
frames (§5.1), each independently snapshotted and CRC'd.

## 5. Instruction set

`INST` bit 7 = 0; opcode in bits [6:4], flags in bits [3:0]:

| flag  | name           | meaning                                                                        |
| ----- | -------------- | ------------------------------------------------------------------------------ |
| bit 0 | HOLD / PROFILE | writes: staged, applied by COMMIT · reads: payload names a profile slot (§5.2) |
| bit 1 | —              | reserved (0) — future extension                                                |
| bit 2 | NOREPLY        | suppress the status frame                                                      |
| bit 3 | PER_TARGET     | group op uses per-target addressing                                            |

| op  | name    | payload                                                              | reply                                 |
| --- | ------- | -------------------------------------------------------------------- | ------------------------------------- |
| 0x0 | invalid | (INST 0x00 never valid, like ID 0x00)                                |                                       |
| 0x1 | PING    | —                                                                    | status: model(2), fw(1) — no UID: 16 more bytes on the hottest liveness check; the UID is an internal value and MGMT ENUM (§9.2) is its only reader |
| 0x2 | READ    | addr(2), count(2)                                                    | status: data(count)                   |
| 0x3 | WRITE   | addr(2), data(n)                                                     | status: empty (ack)                   |
| 0x4 | COMMIT  | — (broadcast)                                                        | none                                  |
| 0x5 | GREAD   | addr(2), count(2), id-list — or PER_TARGET: [id, addr(2), count(2)]× | status chain (§6)                     |
| 0x6 | GWRITE  | addr(2), count(1), [id, data(count)]× — or PER_TARGET variant        | none (NOREPLY implied unless flagged) |
| 0x7 | MGMT    | sub-op byte + args (§9)                                              | per sub-op                            |

Notes:

- READ/WRITE collapse DXL's five read variants and three write variants:
  a single-target read is a one-slot GREAD; WRITE+HOLD is RegWrite; COMMIT
  is Action; GWRITE is SyncWrite; GWRITE+HOLD+COMMIT is the atomic fleet
  update. Addressing mode is one flag, and it stops there.
- Status result codes: see §5.3.
- The control table is flat (address == offset, 1024 B); `addr` is
  2 bytes, `count` is 2 bytes for reads (kept u16 for field alignment in
  the payload view; values cap at 252, §5.1) and 1 byte per GWRITE slice.
- **READ/GREAD addressing is unconstrained** — any `addr`, any `count`.
  Every reply payload streams from the snapshot buffer (§4.2), which is
  halfword-aligned by construction, so the table address carries no
  constraint at all [F12 satisfied structurally]. WRITE addressing is
  likewise unconstrained: inbound payloads validate through the ring
  anchor, not the table address.

### 5.1 Size limits

`LEN` is the only size limit — one ceiling, no capability registers:

- Payload caps at 252 B. Fleet-scale group ops fit in one frame: a
  uniform GREAD lists 248 IDs — one shy of the full 249-ID unicast space
  (§3.1) — a PER_TARGET GREAD 50 targets, a uniform 4 B-data GWRITE 49
  targets.
- Every frame sits whole in the ring until its CRC passes (§4.1), so
  nothing is applied from an unverified frame — no partial-apply, no
  rollback, and no per-transfer staging caps or capability registers.
- Larger transfers split into multiple frames; a WRITE+HOLD sequence
  with one COMMIT keeps a multi-frame update atomic. Reads are status
  frames under the same ceiling: a whole-table dump is five READs.

### 5.2 Read profiles (indirect addressing, span-granular)

DXL's byte-granular indirect registers are deliberately not replicated:
a byte remap forces the reply through a per-byte pointer chase, defeating
both the copy-once TX path and the hardware CRC. The scattered-telemetry
need they serve (position + velocity + current + temperature live in
different table sections) is met span-granularly instead:

- A **profile region** in the flat table (`0x280..0x2C0`): 4 slots × 8
  packed span words, configured once with ordinary WRITEs — no new
  instruction, no hidden state. A span word is
  `u16 = [addr:10][count:6]` — raw byte addressing over the whole 1024 B
  table, spans of 1..63 bytes. `count = 0` **disables** the word, and
  disabled words are skipped rather than terminating the slot, so a host
  can toggle one span with a single 2-byte write; the all-zero boot image
  is an empty slot.
- READ/GREAD with the PROFILE flag name a slot instead of addr+count
  (uniform GREAD: `slot, id-list`; PER_TARGET: `[id, slot]×`). The
  hot-loop instruction stays minimal; the span list costs wire bytes once
  at setup and zero per cycle (the reason profiles beat inline
  scatter-gather lists for cyclic telemetry).
- Execution is §4.2's existing copy-once TX: each span is snapshotted at
  its cumulative offset and the wire and CRC arms stream the one
  contiguous copy, engine accumulating across arms [F6] — a scattered
  read costs the same one-copy-per-reply as a single-span read. Spans
  carry **no parity constraint** (addresses and lengths may be odd): the
  snapshot buffer is halfword-based by construction and only the total's
  parity engages the standard tail fold (§3.2) — same as §5's
  unconstrained plain reads.
- Errors are read-time (§5.3): a slot index past the region, an empty
  slot, or a span leaving the table is `range`; a slot totalling past the
  252 B reply ceiling (§5.1) is `limit`.
- Scattered _writes_ need no counterpart: `WRITE+HOLD` per span plus one
  `COMMIT` is already atomic — inline scatter-writes would add cross-span
  validation complexity for no capability gain.

Implementation note (tearing): the copy-once snapshot DMA (§4.2) reads the
live table byte-serially, so a field updated mid-span by a control ISR can
emit a torn multi-byte value in the snapshot — the wire and CRC then read
that frozen copy, so tearing is bounded to the one copy rather than
compounding per consumer, but it is not eliminated. Field-aligned spans
and the single-writer discipline bound tearing to one field; consumers
that care re-read.

### 5.3 Errors: three layers

There is no status opcode — a status frame is INST bit 7, and its result
code shares the byte (bits [6:2], 32 values). Errors split by layer:

1. **Frame-level** (CRC fail, malformed): **no reply, ever** — a corrupt
   frame's ID is untrustworthy, so a servo cannot know it was addressed.
   The host sees a timeout; the servo increments a diagnostics counter in
   the telemetry region (CRC-fail count, framing-drop count) readable on
   any later read. A latched "I saw a bad frame" reply code would be
   answering a question nobody can safely ask.
2. **Instruction-level** (valid frame, rejected request): the 5-bit result
   code, empty payload — `OK`, `instruction` (unknown op/flags), `range`
   (addr/count out of bounds, or a PROFILE read naming a bad, empty, or
   table-overrunning slot — §5.2), `access` (read-only, or SAVE with
   torque enabled),
   `validation` (value rejected by field rules), `busy`, `limit`
   (requested reply exceeds the frame ceiling, §5.1), `predecessor-silent`
   (§6),
   `hardware`. Exact numeric assignments live with the implementation.
3. **Device-level** (alarms: overtemperature, overcurrent, encoder fault):
   orthogonal to any one instruction's result, so it takes no result-code
   space — status bit 0 (**ALERT**) is set on *every* status frame while
   the alarm register is nonzero, prompting the host to read it. The same
   alert-bit semantics as DXL, because they're right.

### 5.4 Common register block

The table map is per-model ABI — a servo and a sensor node share the
protocol, not the register map (the DXL shape, and the right one: the
map is where models differ). What **is** protocol is a small register
set every node carries at fixed addresses, so model-agnostic tooling —
rescue and baud migration, CAL verification, health polls, discovery
triage — works on any node without knowing its model. Two 32-byte
blocks, one at each region front:

**CONFIG-COMMON** `0x000..0x020` — SAVE-persisted (§9.4); identity RO,
comms RW:

| addr  | name                   | width | access | notes                                  |
| ----- | ---------------------- | ----- | ------ | -------------------------------------- |
| 0x000 | `model_number`         | u16   | RO     | keys the per-model map                 |
| 0x002 | `firmware_version`     | u8    | RO     |                                        |
| 0x003 | `hardware_revision`    | u8    | RO     |                                        |
| 0x004 | `capability_flags`     | u32   | RO     | no bits defined yet                    |
| 0x008 | —                      | 8 B   | rsvd   |                                        |
| 0x010 | `id`                   | u8    | RW     | unicast address `0x01..=0xF9` (§3.1)   |
| 0x011 | `baud_rate_idx`        | u8    | RW     | §2 rate index                          |
| 0x012 | `response_deadline_us` | u16   | RW     | §7                                     |
| 0x014 | —                      | 12 B  | rsvd   |                                        |

**TELEMETRY-COMMON** `0x200..0x220` — volatile:

| addr  | name                 | width | access | notes                                       |
| ----- | -------------------- | ----- | ------ | ------------------------------------------- |
| 0x200 | `fault_flags`        | u8    | RO     | the §5.3 alarm register — ALERT's read target |
| 0x201 | `status_flags`       | u8    | RO     | bit 0 = config-dirty (§9.4); bits 1–7 reserved |
| 0x202 | `trim_steps`         | i8    | RO     | applied clock-trim total (§9.3)             |
| 0x203 | —                    | 1 B   | rsvd   |                                             |
| 0x204 | `crc_fail_count`     | u32   | RW     | §5.3 frame-level counters; hosts write 0 to clear |
| 0x208 | `framing_drop_count` | u32   | RW     |                                             |
| 0x20C | —                    | 20 B  | rsvd   |                                             |

- Reserved bytes read zero; writes touching them reject with `access`.
  Extension fills reserved slots, announced by `capability_flags` bits —
  a host that doesn't know a bit ignores the bytes it covers.
- Everything else is model-specific space (`0x020..0x200`,
  `0x220..0x280`), except the profile region's own pin at
  `0x280..0x2C0` (§5.2).
- Deliberately absent: a torque switch (SAVE self-gates via `access`,
  §9.4, and sensor nodes have no torque), the UID (MGMT ENUM is its only
  reader, §9.2), boot mode (MGMT REBOOT's payload owns it), and every
  motor semantic.

## 6. Coordinated reads (status chains)

GREAD replies arrive as a chain of ordinary status frames, one per listed
servo, in list order. Sequencing is snoop-driven and break-framed, which
makes it cheap and robust:

- Slot 0 replies to the instruction like a unicast read (≥ reply gap after
  instruction end, §7).
- Slot k>0 counts _status_ frames (INST bit 7) on the wire since the
  GREAD; when frame k−1 completes (its end is known at its LEN byte — no
  timing inference), slot k starts after reply gap. Snoopers do **not**
  CRC-validate predecessor statuses — the chain consumes nothing from the
  body, only the framing-level end, so validation would buy nothing and
  cost a CRC pass per snooped frame. A corrupt status mis-times one slot
  at worst, bounded by the reclaim window below.
- **Reclaim deadline** (DXL chains collapse silently past a dead
  responder; here the recovery is specified): if slot k's predecessor
  produces no break within RESPONSE_DEADLINE of its own trigger, slot k
  takes the slot and sets `predecessor-silent` in its status error field.
  The host sees both the gap and the flag. The window covers the
  trigger→break lead only — once the predecessor's break is observed it
  is alive, and the window suspends for a bounded max-frame allowance
  while its frame plays out (completion re-sequences the chain; a frame
  that garbles or wedges lets the suspended deadline fire as the
  reclaim). Keying on the break rather than the frame end is what keeps
  the default baud-independent: a frame's wire time exceeds 60 µs below
  3 M, its break lead never does.
- Error statuses keep the chain alive; only silence triggers reclaim.

There is no FAST/regular split and no per-block checkpoint CRC: each chain
element is a complete, independently CRC'd status frame, and the break
delimiter gives every snooper hardware resync per element — the problem the
DXL checkpoint format solved does not exist here.

## 7. Timing rules

| parameter               | value                                      | rationale                                                                                  |
| ----------------------- | ------------------------------------------ | ------------------------------------------------------------------------------------------ |
| reply gap               | 12 µs after frame end, at every baud       | host TC→release margin — a register poke, i.e. a time-domain quantity: fixed µs neither balloons at 0.5 M (2 byte-times was 40 µs of mandated silence) nor thins at 3 M |
| RESPONSE_DEADLINE       | config register, default 60 µs (all bauds) | chain reclaim (trigger→break lead, §6) + host timeout; NOT a reply-time prescription — a servo replies when ready |
| break length (TX)       | exactly 10 bit-times (§3 law; 9-bit 0x00 character) | break ≡ 1 character: exact span algebra + LIN-detectable; SBK (~14 bits, F5) is off-law |
| inter-frame gap (host)  | none required                              | breaks self-delimit; back-to-back host frames are legal                                    |

Ping turnaround (instruction wire-end → status break fall) is **34.3 µs
at 1 M and 44.3 µs at 3 M**, measured on the current transport, vs
62.8 µs measured for a DXL 2.0 stack on the same silicon — the
instruction's own 5 B + break costs nothing extra on top of that, since
dispatch overlaps its arrival (speculation; see
`osc-servo-transport.md`, dispatch speculation). The dominant turnaround
components are the tail (hardware CRC-check + dispatch handoff), reply
gap (12 µs), and the break itself (~4.7 µs, F5); see
`osc-servo-transport.md` (tick-by-tick exchange trace) for the full
trace and measured baseline table.

The intended hot loop leans on writes being free of turnaround entirely:
`GWRITE(HOLD|NOREPLY) × groups → COMMIT (broadcast, silent) → GREAD
telemetry chain` — writes cost pure wire time (back-to-back frames are
legal), the apply instant is one broadcast, and the telemetry chain is
the implicit ack: a rejected write surfaces within one cycle as the
ALERT bit on that servo's status (§5.3).

## 8. Host requirements

- Crystal-clocked UART with break send (the osc-adapter, or any
  USB-serial with SBK).
- osc-CRC (textbook CRC-16/ARC, §3.2).
- Drive discipline if on a buffer-less bus (release when idle) [F8].
- Schedule the bus: one outstanding instruction / chain at a time;
  timeout = RESPONSE_DEADLINE + frame time.
- Fault pacing (§3.4): after traffic a servo may have received as garble
  (wrong-baud probes, glitches), allow one starve horizon (64
  byte-times) of bus silence before expecting crisp turnarounds — don't
  hammer zero-gap retries into a parked resolver.

## 9. Management plane (MGMT sub-ops)

### 9.1 Rescue break

A dominant low ≥ 300 µs at _any_ configured baud commands: switch the UART
to the 0.5 M rescue rate — volatile only; ID retained, config registers
untouched, nothing persisted. A reboot exits rescue back to the configured
baud. The signal itself is baud-agnostic (raw GPIO low suffices at the
host), so it reaches a servo whose rate is unknown, and it unifies a
mixed-rate bus onto one channel in a single pulse. Detection is the slow
loop's job, not the transport's: the break detector latches only at a
span's END [F15], so no receive wake can observe a pulse in progress —
the servo's main loop samples the line pin and the RX ring's DMA counter
once per idle wake (~50 µs cadence off the ADC tick metronome) and
declares rescue after ≥300 µs of continuous low with the ring frozen. The
frozen-ring requirement is what makes the window aliasing-proof at any
host baud: data cannot hold the line low a whole byte-time without
completing a character, and a completed character rings and moves the
counter — the pulse's own ringed `0x00` (it chars ~a byte-time in)
re-anchors the window and everything after it is provably byte-less. The
declaration lands while the pulse still holds the line, so the transport
resyncs at a provably-still ring position. No EXTI storm, no edge
capture, no wake-path branches. Hosts should send pulses of ~1 ms (the
300 µs floor plus generous sampler-jitter margin under load; repeats are
free and idempotent). Recovery flow: rescue break →
talk at 0.5 M → fix the baud register → COMMIT/reboot. Limitation: it
cannot interrupt a servo wedged mid-transmit (RX is muted during own TX
[F9]) — it is config recovery, not a babble killer.

### 9.2 UID enumeration and ID assignment

The UID is a **fixed 16-byte field** — UUID-width, because the wire format
is the long-lived ABI and no catalog MCU burns in more than 128 bits
(64/96/128 all exist; 96 dominates the STM32-clone pool). A chip fills it
LSB-first from its silicon ID and zero-pads the tail: the V006's
factory-burned 96-bit ESIG (RM ch. 19) lands in the low 12 bytes, UNIID1's
low byte first. Read once at bringup and held as an internal value — not a
table register (it would spend 16 read-only table bytes on something only
discovery reads; ENUM is its sole consumer). The pad costs the prefix tree
nothing: descent depth is driven by where UIDs differ, and same-silicon
chips differ in the low bits.

Push-pull UART has no dominant-bit arbitration, so simultaneous responses
are garbage — and garbage _is_ the collision signal:

- `MGMT ENUM [prefix_len, prefix…]` (broadcast): `prefix_len` counts bits,
  0..=128; the prefix carries `ceil(prefix_len/8)` bytes. The stream is
  LSB-first — bit k of the UID is `uid[k/8] >> (k%8) & 1`, the same order
  the UART shifts bits onto the wire. A servo whose UID begins with the
  prefix replies `OK` with its full 16-byte UID; everyone else stays
  silent. Mismatches and malformed queries draw no reply on the broadcast
  wire — a nack storm is the one reply a broadcast must never produce
  (unicast keeps the §5.3 layer-2 `instruction` verdict). Matching servos
  are same-die replicas running cycle-identical firmware, so an unguarded
  collision is a lie waiting to happen: they answer in unison, and two
  near-equal frames superimposed sub-bit-aligned read back as ONE clean
  frame — the walk records a unique match and the loser's subtree goes
  invisible (measured: superimposed pair probes usually decode as the
  dominant servo verbatim; the residue is literal wire-AND bytes).
  Two rules keep the collision signal honest:
  - **Kill exemption** — colliding IS a matcher's contract, so a staged
    ENUM reply is exempt from any transport wire-safety kill a peer
    matcher's leading reply-break would trigger.
  - **Reply slot draw** — an ENUM reply delays its trigger by
    `(fold(osc-CRC(uid)) XOR tick) mod ENUM_REPLY_SLOTS` byte-times
    (16 slots). The UID term separates same-reel sequential serials; the
    free-running tick term (boot-offset + drift entropy) makes every draw
    fresh, so equal keys cannot hide a pair persistently — unison is a
    per-probe 1-in-16 accident, never a property of the pair.

  Host algorithm: clean reply with a quiet tail → unique match, CONFIRMED
  by probing both one-bit children once (twins differing at that bit
  split deterministically; twins agreeing re-roll their slot draws);
  trailing energy behind a clean frame, garble, or CRC-fail → collision,
  descend one bit and retry; timeout → empty subtree. O(bits · N)
  exchanges plus two confirm probes per servo, boot-time only.
- `MGMT ASSIGN [uid(16), new_id]` (broadcast): the servo whose UID matches
  takes `new_id` — validated 1..=249 (the sole matcher may nack
  `validation` without colliding), applied immediately so the ack already
  leaves from the new id, and mirrored into the config ID register so a
  later SAVE persists it; volatile until then. Solves the
  duplicate-default-ID field pain.

### 9.3 Clock discipline: the CAL break-pair ruler

Most consumers of clock discipline are covered by design:

- Reply timing is event-driven (break-led, when-ready) — nothing is
  scheduled against a clock, so there is no grid for drift to skew.
- HOST↔SERVO comms integrity has ≥3× margin over the worst possible HSI
  state, measured: the chip cannot be detuned far enough to break framing
  or data at 3 M [F10], and the 1 M default triples that.
- Cross-servo simultaneity is an *event* problem, not a clock problem:
  broadcast COMMIT applies a fleet's held writes in the same instant on
  the shared wire.
- Residual ±1 % scale error (velocity estimates, timeouts, PWM rate) is
  far below what any consumer cares about.

One consumer is NOT covered by the single-sided F10 margin: **servo→servo
snoop**. A chain slot fires off its predecessor's *status frame* (§6), so
one HSI receives another HSI — the clock budget is PAIRWISE, and factory
spread reaches 7k+ ppm. At 3 M that garbles snooped status tails
(crc/framing counters on every chained servo); trimming a fleet to a
1.4 k ppm worst pair zeroes them, causally.

A passive estimator of host byte cadence has neither reliable food nor
hardware-anchored stamps; the reference is therefore explicit and
hardware-anchored:

**`MGMT CAL [gap_us(2 LE), gaps(1)]` (broadcast ONLY).** The host follows
the frame with `gaps + 1` bare breaks spaced exactly `gap_us` apart, its
crystal (any timer/DMA pacing) keeping the spacing. Each servo stamps its
tick at every break-wake service entry: both ends of every gap ride the
SAME ISR path, so entry latency cancels in the difference, and what
remains is clock skew plus sub-µs jitter — ~±260 ppm from 8 × 400 µs
gaps, a tenth of the smallest trim step. Contract and hygiene:

- Broadcast-only: a unicast CAL decodes as an instruction error — its ack
  would put the replier's own break on the wire where the train starts.
  The whole fleet measures one train simultaneously.
- The train follows the announce immediately; no frames inside the train.
  Per-gap gate `|Δ − gap| ≤ gap/16` (wider than any legal clock state,
  far under a missed/spurious break); a train with fewer than half its
  announced gaps valid decides NOTHING. A silent train is abandoned by a
  2-gap watchdog; a stray FE mid-train costs its gap, never the train.
- The train's break bytes are ring noise the resolver's hunt scans off
  silently (§3.3) — CAL is invisible to the link counters.
- Breaks decode threshold-free across the entire HSITRIM throw [F10], so
  CAL also *rescues* a servo railed by a bad trim — the ruler works below
  the layer a bad trim breaks.

Thermal drift between CALs is the **differential chain-pair tracker**'s
job, passive and wire-invisible: adjacent break-wake stamps bracketing
exactly ONE CRC-verified *silent* instruction (GWRITE, or WRITE/COMMIT
with NOREPLY or broadcast — shapes no reply can follow, since a
responder's turnaround rides its clock, not the host's) measure
`seam + drift·span`. The host's queuing seam is unknown but stationary:
the mean pair error over the 32 pairs after any trim decision IS the seam
(baseline), and 128-pair windows read drift as their shift from it —
anything constant (seam, FE latch offset, entry-path residue) dies in the
subtraction. Byte-exactness (ring span == the verified footprint) and the
same 1/16 gate qualify pairs; window verdicts past ±8 k ppm are not
thermal and are discarded (a seam shift comes from a host behavior change
the host knows about — it re-anchors with a CAL).

Both feed the oscillator-trim loop (`steps = round(err/step_effect)`,
clamped ±4/decision; step effect self-measured — chip trim steps are
nonuniform, 1.4–3.2 k ppm/step measured), applied by the main loop
between frames; the total is readable at `telemetry.clock.trim_steps`.
Volatile by design: the host CALs at boot (~4 ms of bus per train) and at
moments it knows its own behavior changed — not on a timer; the tracker
holds the fleet through everything between.

**Boot guidance: send at least two trains.** Full convergence is a
two-point identification, not a precision problem: the first train's
correction divides by the seeded nominal step effect, and a chip's true
ppm-per-step is only knowable from the apply→remeasure pair — so chips
whose steps are weaker than nominal land one step short on the first
train and finish on the second. Longer trains cannot buy this back
(train noise ~±260 ppm is already a tenth of the smallest step); more
trains can. Converged = `trim_steps` read-back stable between trains;
two suffice in practice, a third confirms.

### 9.4 Config persistence (SAVE)

DXL gates all EEPROM-region writes behind torque-off — a category error
that conflates the storage medium with the data's mutability. The actual
mid-motion hazard is the **flash program operation** (it stalls
instruction fetch for milliseconds — lethal under a live control loop),
so that is what gets gated:

- Config-region writes are always allowed (normal field validation
  applies) and hit only the live RAM table — volatile until saved.
- `MGMT SAVE` is the only flash-touching operation: requires torque
  disabled (else `access`), programs the config page (power-safe A/B
  alternation on the reserved config pages), and acks **after**
  completion — the servo is genuinely stalled during program, so hosts
  use a SAVE-specific timeout (erase + program run 5–10 ms; ~50 ms is
  comfortable guidance). FACTORY shares the stall and the timeout.
- No write is torque-gated — there is no section lock. Field validation
  rules still apply to every write; anything genuinely unsafe to change
  mid-motion is the control kernel's job to sequence, not the table's to
  forbid.
- A config-dirty bit in telemetry reports modified-since-save
  (`status_flags` bit 0, §5.4).

Side effects: flash wear drops from program-per-write to
program-per-session, and the ~10 ms write stall stops ambushing hosts on
ordinary config writes — it happens exactly once, at a moment the user
chose, with torque provably off.

### 9.5 Reboot / factory

`MGMT REBOOT`, `MGMT FACTORY` — conventional semantics (as in DXL);
payload details live with the implementation. FACTORY resets the saved
config page, not just the live table.

## 10. V006 resource map

| resource            | use                                               |
| ------------------- | ------------------------------------------------- |
| USART1 + HDSEL, PC0 | the bus (rev B default config: full duplex, PC1 RX + PC2 TX_EN; the `half-duplex` feature frees both pins on rev-c) |
| DMA1 CH5            | RX ring (circular, armed once)                    |
| DMA1 CH4            | TX stream (enable-when-ready)                     |
| DMA1 CH3 + SPI1     | CRC engine (no pins) [F6]                         |
| DMA1 CH1 / CH2      | ADC / free                                        |
| DMA1 CH6            | copy-once snapshot buffer (§4.2); CH7 free        |
| SysTick             | framer deadlines A/B, reply gap, reclaim             |
| TIM1/TIM2           | motor control, freed from transport duty          |
| EXTI                | unused — no transport consumer at all (§9.3)      |

Notably absent (vs a DXL-style transport): input-capture edge timing,
TIM-compare TX kickoff, an RDT register and its tuning surface,
byte-stuffing encode/unstuff, the FF-FF-FD hunter, software fold-CRC —
and, on the direct wire, the 74LVC2G241 buffer + TX_EN pin (the rev B
default board config keeps them).

## 11. Measured foundation

| #   | fact                                                                                       | source                |
| --- | ------------------------------------------------------------------------------------------ | --------------------- |
| F1  | FE fires 1:1 per break at 3 M, HSE host, 50/50                                             | bringup measurement, V006 |
| F2  | break rings exactly one 0x00 via DMA; NDTR-exact framing                                   | bringup measurement, V006 |
| F3  | any-length break = one event (932 µs low → 1 FE)                                           | bringup measurement, V006 |
| F4  | mid-frame FE: no halt, byte rings, IRQs coalesce                                           | fault-injection matrix, V006 |
| F5  | SBK break ≈ 14 bit-times, both chips, zero variance                                        | bringup measurement, V006 + V203 |
| F6  | SPI CRC: 16-bit LSB-first = natural-order ARC (bitrev16 register), accumulates across DMA arms, 0.36 µs/B wall ~0 CPU | bringup measurement, V006 |
| F7  | HDSEL direct wire works both directions, no buffer needed                                  | bringup measurement, V006 |
| F8  | idle push-pull clamps other talkers; OD-idle/PP-talk is mandatory                          | bringup measurement, V006 |
| F9  | V006 HDSEL has no own-TX echo                                                              | bringup measurement, V006 |
| F10 | full HSITRIM throw −3.0..+3.4 %: framing AND data survive everywhere                       | HSITRIM sweep, V006   |
| F11 | production table CRC = 635 ns/B pure CPU                                                   | bringup measurement, V006 |
| F12 | DMA rounds odd MAR down; no unaligned 16-bit reads                                         | bringup measurement, V006 |
| F13 | EXTI edge ISRs storm during traffic (own TX stretched 3×)                                  | HSITRIM sweep, V006   |
| F14 | data decodes clean at ±3.4 % in both TX and RX directions                                  | HSITRIM sweep, V006   |
| F15 | LBD runs sans LINEN, both chip families: length-qualified (≥10-bit spans only — 0 fires on framing-error injection and high-baud garble), safe flag-selective write-0 clear mid-traffic, one event per any-length span, **latched at the span's END** (== bit 10 for the 10-bit law break; a rescue pulse's wake arrives after the line rises), entry stamps 4 ticks p-p on a 400 µs grid; latched FE/NE/ORE with no interrupt enabled are harmless through marination | bringup measurement, V006 + V203 |
