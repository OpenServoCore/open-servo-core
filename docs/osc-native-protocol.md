# OSC-Native Protocol

The normative specification of the osc-native servo-bus protocol:
break-framed, 5 overhead bytes per frame, hardware-CRC-friendly, designed
to run whole on sub-$0.20 MCUs. Every physical-layer behavior the design
leans on was measured on real silicon (V006 servo, V203 HSE host); the
measured facts are collected in §11 and cited inline as [F1]..[F17].

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
  internally tied to the pin; the wire needs no dedicated RX pin and no
  direction buffer [F7]. Bus side: series R + pull-up (+ optional TVS); a
  buffer's roles collapse into the drive discipline below.
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
anchor. Bridge-class hosts use exactly that. The servo cannot: its bus is
one pin under HDSEL, which disables the USART's LIN break detector on the
target silicon [F16], so it times the low on the pin instead - a
detector that qualifies at 9.25 bit-times, past the longest data low (9)
and inside the law break (10), §3.4 [F17]. Hardware `SBK` is off-law
(~14 bit-times measured, F5).

**Receivers stay length-tolerant**: any ≥10-bit dominant span is one
break (rescue pulses, garble, and F3 all require this). Both ends
transmit the law shape: the host sends it directly, and the V006 reply
path sends the bracketed-M `0x00` character rather than hardware `SBK`
(same blocking shifter contract, 3 bits shorter).

Measured break behavior that the framer relies on:

- The break detector fires 1:1 per law break [F17], and the break rings
  as exactly one `0x00` byte via DMA [F1][F2].
- A break of _any_ length is exactly one event - the detector re-arms only
  on the rising edge that ends it, so long breaks cannot spam [F3][F17].
- A mid-frame framing error does not halt reception: the garbled byte
  rings and the stream continues [F4] - and it raises nothing at all
  (lows under 9.25 bit-times are invisible to the detector, and no
  interrupt is enabled on the error flags, §3.4 [F17]). Ring + NDTR are
  the only ground truth.
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

The receive side has two distinct signals, and the protocol binds them
to two distinct roles:

- **The break detector is the wake.** It is length-qualified - only a
  dominant span held past 9.25 bit-times fires it, a length valid data
  never reaches (9 at most) and the §3 law break always does (10) - and
  any-length span raises exactly one event, **fired a break-length into
  the span**. That can be before the stop-bit sample rings the break's
  `0x00`: a wake may beat its own byte into the ring, so its
  ring-dependent service waits on the ring, never on the wake. On the
  target silicon HDSEL disables the USART's LIN break detector [F16], so
  the detector is a timer on the bus pin that counts only while the line
  is low and is zeroed by every rising edge, so its overflow can only be
  a break [F17]. It is the ONLY receive interrupt an implementation
  enables, and it is deaf to the implementation's own break.
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
containing a dominant span past 9.25 bit-times (slower-baud traffic heard
at a faster-configured servo) can wake the resolver into junk at all -
noise and faster-baud garble ring silently and cost nothing until the
next real break [F17]. **Host pacing rule:** after traffic a servo may
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

- **HUNT** - on the break wake: record the anchor = the ring position of
  the break's `0x00` (it may ring just after the wake). Enter LOCKED.
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
| 0x1 | PING    | —                                                                    | status: model(2), fw(2) — no UID: 16 more bytes on the hottest liveness check; the UID is an internal value and MGMT ENUM (§9.2) is its only reader |
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
   `hardware`. One more code is not an error at all: `stream` marks the
   unsolicited TEL burst frames of sec 5.6. Exact numeric assignments
   live with the implementation.
3. **Device-level** (alarms: overtemperature, overcurrent, encoder fault):
   orthogonal to any one instruction's result, so it takes no result-code
   space — status bit 0 (**ALERT**) is set on *every* status frame while
   the alarm register is nonzero, prompting the host to read it. The same
   alert-bit semantics as DXL, because they're right. Which alarms exist
   is per model; the osc-servo set is in sec 5.7.

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
| 0x002 | `firmware_version`     | u16   | RO     | semver packed 5.5.6: `[major:5][minor:5][patch:6]` |
| 0x004 | `capability_flags`     | u32   | RO     | no bits defined yet                    |
| 0x008 | `hardware_revision`    | u8    | RO     |                                        |
| 0x009 | —                      | 7 B   | rsvd   |                                        |
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

- `model_number` is class-structured `[class:8][model:8]`: the high byte
  names a device class, the low byte a model within it. `0x0000` is
  reserved unassigned - what unseeded or dev firmware reports. The
  registry (class names and assigned numbers) lives in
  `osc-protocol::models`.
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

**Versioning.** `firmware_version` is the firmware's semver applied to
the control table: MAJOR bumps on a breaking table change (a field
moved, removed, retyped, or its access changed), MINOR on an additive
one (new fields only), PATCH when the table is untouched. Saved config
is unaffected - the persisted image carries its own layout version
(§9.4). While MAJOR is 0 the firmware is in active development and
promises no compatibility: the table may change without a bump, and
hosts track the latest code. The bump rule binds from 1.0.0.

A descriptor - the exported device description at
`descriptors/<model>/<major>.<minor>.json` - names one layout; PATCH
never changes it. Host selection: read `model_number` and
`firmware_version`, require the same MAJOR, take the largest known
MINOR at or below the servo's (a same-major older descriptor is a valid
subset). A different MAJOR, or no descriptor at all, means typed access
to the common blocks above only, plus a warning.

### 5.5 Units: device counts

Control-table quantities that mirror a sensor reading or feed control
are raw device counts - ADC counts, encoder ticks - never engineering
units. Alongside them the table publishes the calibration primitives
that make counts interpretable, as integer rationals: shunt milliohms,
amplifier gain x1000, divider resistances, measured bias counts, VDD in
millivolts. Counts plus primitives are a complete description;
milliamps alone are not, because the scale that produced them is gone.

Conversion to physical units happens at the consumer that needs them: a
UI, an SDK, planning code. Intermediate tiers - gateways, adapters -
forward counts and calibration data uninterpreted; a tier that rescales
is one more place for a scale error to hide, and everything below it
loses the raw value.

No hardware floor is implied. Converting a stream costs one software
division per factor at connect time (building a fixed-point
reciprocal), then one multiply-shift per sample - the same arithmetic
discipline the servo itself uses. Divides sit at configuration edges,
never in the per-sample path.

Servo-side conversion would spend scarce cycles on a divide-free chip
to produce numbers the servo itself never consumes; counts keep the
loop math native and the calibration honest. It is also the established
bus-servo convention - hosts convert.

One obligation this puts on the host: coordinating servos in physical
space requires converting first, since per-unit scales differ between
servos.

Linearization keeps the unit. The osc-servo corrects its pot through a
per-unit table (sec 5.7), and the output is still ADC counts on the same
scale: the identity outside the stops, each stop mapping to itself, so
every count-denominated field (position limits, velocities in counts/s,
the gains fitted against them) means the same thing before and after a
table goes live. Only the raw sensor screen, the published raw sample
and the hold deadband stay raw: `pos_deadband_counts` is raw counts of
the position sensor, where its noise and quantisation live, and the
servo scales it by the table's local gain at the present position. The
table itself is calibration data like the primitives above: stored on
the servo, exported to the host, never converted on the way.

### 5.6 Telemetry stream (TEL bursts)

High-rate telemetry rides the bus as a bounded burst of status frames
the servo emits on its own schedule - no extra wire, no side channel,
and every frame under the same CRC as the rest of the protocol. Two
control registers arm it: `tel_mask` selects the per-sample fields (one
bit per field, canonical order; reserved bits reject), and a committed
nonzero write to `tel_count` starts a burst of that many control-tick
samples. Both compose with HOLD/COMMIT, so a goal write and the arm
commit together: the capture starts at the commit and the step edge
lands inside it, at the first medium tick after the commit (sec 5.7). A
committed `tel_count` of 0 disarms. The stream carries every control
tick; polled, the TELEMETRY sensor registers (the raw samples,
`vcal_lpf`, `current_bias_counts`) refresh once per medium tick (2 kHz),
like the estimates.

Burst frames are ordinary status frames with result code `stream` - the
one result code that marks a frame no instruction directly owes.
Payload, all LE:

```
[0]    stream_seq  u8, increments per frame, wraps
[1]    flags       bit 0 = LAST frame of the burst; rest reserved 0
[2..4] valid       u16 bitmap, bit i = sample i measured a fresh window
[4..]  samples     the mask-selected fields in bit order, 2 B each
```

`tel_mask` bits, in sample order (osc-servo): 0 `pos` (raw pot), 1
`current` (bias-subtracted window sample, times the board's settle gain
for its drive width, unity on a window wide enough to settle; held
through windows the shunt cannot read, 0 from the first tick nothing
drives: torque off, a fault, a brake, an OpenLoop zero goal), 2 `current_trough`, 3 `duty`, 4
`vdiff` (held through windows the terminal taps cannot read, which the
`valid` bit does not mark when the shunt reads them, sec 5.8), 5 `vbus`, 6 `current_raw`, 7 `vmotor_a`, 8 `vmotor_b`, 9
`vbus_raw`, 10 `ntc_raw`, 11 `pos_lin` (the linearized pot the kernel
controls on, the Q4 word itself, sec 5.7); bits 12-15 are reserved and
reject. A sample carries at most 6 fields (12 B): a mask selecting more
rejects at write time, since a 16-sample batch of it could not clear the
wire inside its own tick window at 3 M.

Samples batch up to 16 per frame (the burst's final frame may carry
fewer); the count is implicit in `LEN`. Batching is what makes the CRC
affordable: framing overhead amortizes to well under one byte per
sample, and the largest legal frame (six fields, 16 samples, 203 wire
bytes) fits its own 16-tick batch window at 3 M with margin - a full
six-field set sustains the tick rate, which the old per-tick side
channel could not.

The wire contract during a burst: the host is silent. The servo owns
the line from the arm's ack (or the arming COMMIT's silence) through
the LAST-flagged frame, then the line frees on its own - no polling, no
handshake. Any host break mid-burst aborts the burst immediately;
reclaiming the line IS the host's abort lever, and the one in-flight
frame it garbles is the host's chosen cost. The instruction that
arrived is then served normally, including a fresh re-arm.

Integrity is the point: a corrupted burst frame fails CRC and is
dropped whole by the host framer - it can never decode as plausible
data - and the drop is visible as a hole in the `stream_seq` numbering
(16 samples per missing frame). ALERT on a burst frame carries the OR
of the batch's fault state, per the sec 5.3 device-level contract.

Timing: a batch completes every 16 control ticks; the frame must clear
the wire inside that window or the producer drops whole batches
(drop-not-block, surfaced as seq holes). At 3 M every mask fits; below
2 M a full-rate burst outruns the wire by design - run captures at 3 M,
or accept the decimation the holes record. Safety through the silent
window is the servo's own - the current limit (held in every drive
mode, OpenLoop included, sec 5.8), soft-position clamps, the stall
timer and fault latches run in firmware regardless of the bus - and the
host's supervisory reads resume between bursts. The one thing a silent
host cannot do is renew a stall permit: a burst that needs one has to
end inside the lease (sec 5.8), and `osc` refuses a burst longer than
750 ms while it holds a permit.

### 5.7 Data state, plant stamp and position table (osc-servo)

Three osc-servo conventions that ride the same registers: a data state
that says whether the persisted images and the identified set behind the
closed loops are this servo's own, a plant stamp that makes the
identified values and the position table one transaction, and the
position table itself. They are model facts, not protocol, and live in
model-specific space; the descriptor (sec 5.4) carries the field
addresses and the stamp recipe, so a host needs no second copy of any
of it.

**Data state.** `data_flags` (u8, RO, `0x223` in TELEMETRY-MODE) names
every reason closed loop is refused, one bit per reason; 0 means the
saved images loaded and the set they hold is stamped and identified.

| bit | reason           | set                                                                                   | cleared                          |
| --- | ---------------- | ------------------------------------------------------------------------------------- | -------------------------------- |
| 0   | `CONFIG_VIRGIN`  | boot: both CONFIG slots erased                                                        | a successful SAVE                |
| 1   | `CONFIG_CORRUPT` | boot: CONFIG bytes present that parse under no version                                | FACTORY + reboot only            |
| 2   | `CALIB_VIRGIN`   | boot: both CALIB slots erased                                                         | a successful SAVE                |
| 3   | `CALIB_CORRUPT`  | boot: CALIB bytes present that parse under no version                                 | a successful SAVE                |
| 4   | `STAMP_MISMATCH` | a checkpoint recompute differs from `plant_stamp`, or a covered field was written since | a matching checkpoint            |
| 5   | `PLANT_UNSET`    | a checkpoint sees `recip_ke_q == 0` or `ke_vpc_q == 0`                                | a checkpoint with both nonzero   |
| 6   | `CONFIG_STALE`   | boot: a CRC-valid CONFIG image of another layout version                              | a successful SAVE                |
| 7   | `CALIB_STALE`    | boot: a CRC-valid CALIB image of another layout version                               | a successful SAVE                |

Boot classifies each image from its two slots: *loaded* when one parses,
*virgin* when every byte of both is erased, *stale* when a slot holds a
CRC-valid image of another layout version (a real save this firmware
cannot read: board defaults stand, nothing migrates), *corrupt*
otherwise. Stale beats corrupt when one slot is stale and the other is
rotten. `CONFIG_CORRUPT` is the one reason SAVE never retires: the
running limits and polarity are board defaults standing in for a lost
tuned config, and a blind SAVE must not bless them as this servo's own.

The verdict is a gate on use, never an alarm on state. At the
`torque_enable` 0 to 1 edge, and at a mode change while torque is on,
the kernel reads `data_flags`: `CONFIG_CORRUPT` refuses every mode; any
other reason refuses Velocity and Position and leaves OpenLoop and
Current open (they consume neither Ke nor the loop gains above the
current loop, and calibration and identification drive only those); 0
opens all. A refusal latches the `data` fault (below) and the servo
stays disabled for that run. A reason that appears mid-run - a live
edit of a covered field - waits for the next entry; the one thing that
stops a running loop is the physics belt. A CONFIG or CALIB write
reaches the kernel at the next medium tick boundary, within 0.5 ms of
the commit. If that write leaves `recip_ke_q == 0` or `ke_vpc_q == 0`
under a running Velocity or Position loop, the same fault latches at
that boundary and the drive stops. Until then the loop keeps running
on the Ke it had, so a live write of a zero Ke can never run a loop
open.
A virgin servo jogging in OpenLoop therefore shows no ALERT and its
captures stay clean (DES `virgin_servo_drives_openloop_and_current_without_alert`,
`live_zero_ke_write_stops_a_running_closed_loop`).

The osc-servo alarm register (`fault_flags`, sec 5.4) and the latest
newly-latched kind (`fault_code`, `0x221`):

| bit | code | fault            |
| --- | ---- | ---------------- |
| 0   | 1    | `over_current`   |
| 1   | 2    | `over_temp`      |
| 2   | 3    | `stall`          |
| 3   | 4    | `position_error` |
| 4   | 5    | `sensor`         |
| 5   | 6    | `under_volt`     |
| 6   | 7    | `data`           |

Any set bit forces the drive off; the `torque_enable` 0 to 1 edge is the
only acknowledgement, and a still-present condition re-latches at once.
The kernel reads `torque_enable`, `mode` and the goals once per medium
tick (2 kHz), so an enable, a disable, an acknowledgement, a mode change
and a new `goal_duty` take effect at the first medium tick after the
commit, within 0.5 ms; a `goal_current` or a closed-loop reference
follows three ticks after that, and the TELEMETRY registers show the
result later in the same medium period.
`stall` latches in OpenLoop as it does in the closed loops: there the
stall timer runs off the duty ceiling (sec 5.8), so with
`stall_response` at its boot value of Fault, an OpenLoop drive held
against a stop or a jam for longer than `stall_time_ms` latches it
unless a stall permit is live.

**Plant stamp.** `plant_stamp` (u16, RW, `0x0B2` in CALIB, persisted)
is the host's CRC over the set it *intended* to write: the identified
and calibrated fields plus the effective position table. Firmware
recomputes it over what actually landed at every checkpoint and reports
a difference as `STAMP_MISMATCH`, so a write that never landed, a torn
save, a hand edit and a table rebuilt under old constants all read the
same way.

```
stamp = max(1, CRC-16/ARC("osc-plant-1" ++ covered bytes ++ points))
```

The covered bytes are the 35 covered fields' own table bytes in table
order (77 B): the position limits, the loop gains, the deadband,
velocity and acceleration limits, drive polarity, the stall and
thermometer speed gates, the observer gains and the position-error
threshold in CONFIG; the pot stops, the motor constants, the friction
model and the angle map in CALIB. The points are the position table's
256 host-written calibration points i16 LE (sec below) while
`pos_lut_state` is LIVE, 256 zeros otherwise, so a table falling back to the identity changes the stamp by
itself. Not covered: identity and comms, the user-owned safety limits
(current, thermal, undervolt, `duty_max_q15`), the raw sensor screen,
the winding anchor, and the RO board facts install re-seeds. `0` is
reserved for *never stamped* and the recipe never produces it. The list
is exported once, in the descriptor's `stamp` block (`tag`, `covered`,
`pos_lut_points`); the CRC is the sec 3.2 checksum in software, ~600 B
and ~0.6 ms on the servo, on torque-off paths only (the SPI engine belongs
to the transport).

Checkpoints, the only places the verdict recomputes: boot after both
images load; a committed write to `plant_stamp` with torque off; a
table COMMIT; SAVE. Between checkpoints a committed span that intersects
a covered field marks `STAMP_MISMATCH` at once, whatever value it
carried, so a host that dies halfway through a set leaves the mismatch
behind (DES `partial_covered_write_blocks_next_enable`). A stamp write
under torque lands unverified: the mismatch stands until the next
checkpoint. SAVE checkpoints before it programs and still persists a
mismatched set (the reboot recomputes anyway; flash is never refused).

The recompute outlasts the reply deadline, so the stamp write and the
COMMIT checkpoints run in the servo's main loop after the reply: the
commit marks `STAMP_MISMATCH` (the refused direction) and posts the
job; the job clears the mark once the set verifies, ~0.6 ms later. A
write that lands while the job runs discards that run and it runs again
over the set as it is then, so the mark never clears against a set the
stamp did not cover (DES
`stamp_write_verifies_in_the_main_loop_after_the_reply`). SAVE runs a
posted job itself before it checkpoints and programs.

The host sequence that closes an identification or a calibration: torque
off; write the set, each write read-back verified; compute the stamp
over the intended set, not over a read-back, and write `plant_stamp`;
poll `data_flags` until `STAMP_MISMATCH` clears (20 ms is ample) - a
`STAMP_MISMATCH` still standing means a write did not land; SAVE, since
the virgin and stale reasons clear only on SAVE; only then verify closed
loop. `osc ident` (write, rollback), `osc cal` and
`osc recover --from` all commit this way, and `osc stamp [--save]`
blesses a hand-tuned set explicitly. The drives that measure a set
come before this sequence and write only the CONTROL fields a drive
uses (sec 5.8, around a drive): `osc ident run` writes none of the
set, write-back is its own command, and an `osc cal` that is refused
or ends early writes none of it either, the drive polarity aside once
both stops are read. Nothing restamps as a side effect:
`osc set` of a covered field leaves the mismatch standing, and `osc lut
write` never stamps, because a new table redefines the domain the
constants were fitted in - the way out is `osc ident`.

**Position table.** The kernel corrects each raw pot sample through a
per-unit position linearization table on a fixed grid over the 12-bit
ADC domain: 256 intervals of 16 raw counts and 257 calibration points,
each an i16 correction `c[k]` against the identity ramp. A calibration
point, called a knot in the math below, sits at raw `16 k`; knot 256
sits at 4096, is fixed at 0 and is never written. The all-zero table is
the identity. The output is linearized counts in Q4:

```
i     = raw >> 4
f     = raw & 15
lin   = ((raw + c[i]) << 4) + (c[i + 1] - c[i]) * f      (u16, Q4)
```

so the identity is `raw << 4` exactly and knot `k` lands at
`raw + c[k]`. Once per medium tick, while `pos_lut_state` reads LIVE, this
value seeds and innovates the position observer; `theta_hat_q16` and everything that
reads it (trajectory, position loop, soft limits, the stall and
thermometer speed gates) are in linearized counts. The raw sample stays
raw for the published `pos`, the TEL `pos` field and the sensor-delta
screen. The endstop brake and the soft limits trip at the same raw
counts as before, because the table is the identity outside the stops
(DES `endstop_trips_at_the_same_raw_counts_under_a_live_lut`).

The table lives in RAM behind a paged window in CONTROL:

| addr  | name             | width | access | notes                                                                         |
| ----- | ---------------- | ----- | ------ | ----------------------------------------------------------------------------- |
| 0x19C | `pos_lut_page`   | u8    | RW     | 0..7, 32 calibration points per page                                          |
| 0x19D | `pos_lut_cmd`    | u8    | RW     | 0 none, 1 STORE, 2 FETCH, 3 COMMIT; runs on commit, reads back 0              |
| 0x19E | `pos_lut_points` | 64 B  | RW     | `[i16; 32]` LE, the window                                                    |
| 0x1DE | `pos_lut_state`  | u8    | RO     | 0 IDENTITY, 1 LOADING, 2 LIVE, 3 REJECT_TORQUE, 4 REJECT_ENDS, 5 REJECT_SHAPE |

Page, command and points are contiguous, so one 66 B WRITE at `0x19C`
carries a page; an out-of-range page or command is a `validation` nack,
and only a committed span that covers `pos_lut_cmd` runs a command. STORE
copies the window into its page of the array and leaves LOADING (the
kernel applies the identity until a COMMIT); FETCH copies that page
back into the window; COMMIT validates the whole array against
`raw_min`/`raw_max` and lands LIVE or a REJECT, then runs the stamp
checkpoint. The reply leaves first: COMMIT reads back LOADING with
`STAMP_MISMATCH` marked until the servo's main loop lands the verdict
(~0.6 ms), so a host polls `pos_lut_state` past LOADING. A STORE behind
an unjudged COMMIT cancels it (the array is loading again; only the next
COMMIT judges it), and torque coming on before the verdict lands reads
REJECT_TORQUE (DES
`commit_lands_its_verdict_in_the_main_loop_after_the_reply`). STORE and
COMMIT are torque-gated: from LIVE a refusal leaves LIVE standing (the
state is what the kernel applies, and a refusal must not move it under
a running loop); from any other state they read REJECT_TORQUE. FETCH is
never gated. A rejected array stays in RAM to be fixed page by page,
and a STORE out of LIVE marks `STAMP_MISMATCH` the way a covered write
does. Firmware validation is physics sanity, never quality: every
calibration point at or beyond a stop is zero (knot
`k <= (raw_min + 15) >> 4` and `k >= raw_max >> 4`, so both stops map
to themselves whether or not they sit on a calibration point; stops
unset admit only the identity), and every interval's Q4 gain `16 + c[k+1] - c[k]` lies in
`1..=255`, local gain in `[1/16, 16)`: strictly monotone, and the u16
word cannot overflow. Ends are judged before shape. Grading a table is
the host's job (`osc lut grade`).

TEL `pos_lin` (bit 11) streams the Q4 word the kernel used that tick,
so a host can pin its own interpolation of `pos` against it exactly
(DES `tel_pos_lin_is_interp_q4_of_pos_on_every_sample`). SAVE persists
the table beside the calibration in the CALIB image (sec 9.4); boot
re-validates the loaded calibration points against the stops loaded
with them and goes LIVE only when they validate and correct something,
otherwise the
identity runs and the stamp reports the loss as `STAMP_MISMATCH` (DES
`lut_survives_save_and_reboot_until_factory`,
`corrupt_or_stale_calib_image_boots_identity_under_its_reason`).

### 5.8 Limits and the stall permit (osc-servo)

Every drive mode clamps against the same current band (control-theory
"Limits"): the current limit, the thermal derate, the stall fold and the
directional endstop. The closed loops clamp their current reference;
OpenLoop, which has none, holds a duty ceiling against the band from
the shunt alone, so a host-written `goal_duty` is a request the servo
may refuse to apply in full. Three registers let a host take part: a
permit to stall on purpose, a flag byte that names whatever is holding
the command back, and the duty under which the shunt reads nothing.
Like sec 5.7 these are model facts in model-specific space, and the
descriptor carries the addresses.

**Stall permit.** `stall_permit` (bool, RW, `0x181` in CONTROL) lets
the motor stall on purpose, which calibration needs to seat a hard
stop and identification's stop routes, when asked for, need to measure
the winding. It drops the stall trip (timer and collision check) and
the endstop, and nothing else: the
current limit and the thermal derate still compose. The byte is the
host's *request*; the *grant* is a lease the servo keeps:

- A committed write whose span covers the byte, leaving it true while
  `torque_enable` reads 1 at the moment the write commits, grants a
  lease of about one second: 63 slow ticks of 16 ms, counted from the
  medium tick that sees the write, so 0.99 to 1.01 s. One span over
  `torque_enable` and `stall_permit` is judged as committed, so a
  single WRITE of `[1, 1]` at `0x180` enables and grants; a HOLD write
  grants at its COMMIT.
- Each rewrite of true under torque restarts the lease from the
  rewrite. A host holding the permit rewrites it well inside the second
  (`osc` does it every 250 ms).
- A write of false revokes within one medium tick (0.5 ms). Torque off
  revokes, and turning torque back on does not revive the lease: only a
  fresh write does.
- A write with torque off grants nothing, even when torque comes on
  before the servo looks: the host writes the permit after the enable.
- The byte reads back the last request, never the grant; `limit_flags`
  bit 3 (below) is the grant. CONTROL is RAM, so a reboot clears both.

The lease is what bounds a host that dies mid-run: the servo is
unguarded for at most what is left of the second, then the stall timer
runs again. A permit written once against a locked rotor, with a
500 ms `stall_time_ms` and the boot Fault response, latches `stall`
between 1.5 and 1.6 s after the write (DES
`dead_host_stall_trips_within_the_lease`). A TEL burst is a silent host
too (sec 5.6).

**What governs.** `limit_flags` (u8, RO, `0x266` in TELEMETRY; `0x267`
is reserved) names what shaped the command at the last medium tick,
one bit per reason; 0 means nothing held it back and no permit is
live. It is published
every medium tick (2 kHz), torque on or off.

| bit | name      | set while                                                                                                                                                            |
| --- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0   | `CEILING` | the command sat at the current ceiling: in OpenLoop the duty ceiling held under the goal by the current, in the closed loops the current reference at `i_lim_counts` |
| 1   | `YIELD`   | the stall verdict folded the limit to `stall_yield_counts`                                                                                                           |
| 2   | `ENDSTOP` | exactly one side of the band is closed: a soft limit forbids current in one direction (a zero limit closes both and does not set it)                                 |
| 3   | `PERMIT`  | the stall permit lease is live                                                                                                                                       |

Bit 0 is the same pin the stall timer counts, so a bit 0 that stays set
on a still shaft is a stall in progress. The permit drops the fold and
the endstop, so bit 3 never appears with bits 1 or 2 (unit
`limit_flags_name_the_governor`). The flags are a polled register, not
a TEL field: the `valid` bitmap of sec 5.6 has no spare bit.

**Window floor.** `window_floor_q15` (u16, RO, `0x268` in TELEMETRY,
a Q15 duty magnitude) is the smallest duty whose drive window the shunt
reads: CALIB `i_window_min_ticks` turned into a duty against the board's
PWM period, 1734 (5.3%) for 64 ticks of 1200 on osc-dev-v006. It is
the value the OpenLoop limiter uses, where its ceiling restarts and its
blind band (below) begins, published at the kernel's first tick and
again whenever a CONFIG or CALIB write reaches the kernel, torque on or
off, so a rewrite of `i_window_min_ticks` shows within one medium tick
(0.5 ms).
0 means the servo publishes no floor: the kernel has not ticked yet, or
the firmware does not carry the field. A host planning a drive against
the current sensor reads the floor here rather than deriving it from a
board constant, and `osc` refuses a servo that reports 0 (DES
`window_floor_is_published_as_the_limiter_uses_it`).

The terminal taps have a floor of their own: `window_v_floor_q15` (u16,
RO, `0x274` in TELEMETRY, Q15) is CALIB `v_window_min_ticks` turned into
a duty the same way, 4356 (13.3%) for 160 ticks on osc-dev-v006, where
the third scan slot, tap B, closes its sample. It is published with
`window_floor_q15`, at the same moments. A window between the two floors
reads current but no `va - vb`, so the stream's `vdiff` and the ident
aggregate's `vdiff_mean` hold the last differential read there. Which
floor a host plans against follows what its fit reads: a drive judged
on current alone (the limiter's slew, an overcurrent abort, a current
step) takes `window_floor_q15`; a drive whose fit takes the differential
(`osc`'s resistance stop ladder) takes the higher of the two. Firmware
that predates the register reads 0 there, and a host then takes
`window_floor_q15` for both (DES
`window_v_floor_is_published_beside_the_current_floor`).

**Governed windows.** In OpenLoop the applied duty equals the goal only
when nothing governed it, and a host fitting a model to a capture needs
to know which windows those are. The TEL `duty` field (bit 3) is the
applied duty, the command whose window the sample measured. A goal
lands at the first medium tick after its commit (sec 5.7), within one
medium period (10 ticks). From there, to a magnitude above the window
floor, the applied duty climbs 128 (Q15) per tick from its start: the
previous applied duty, or the window floor when the change starts from
zero duty or reverses the sign. A window is *governed* when `duty`
reaches the goal later than `(|goal| - start) / 128 + 2 + 10` ticks
after the commit (the two ticks cover the sample alignment and the
rounding, the ten the medium period the goal may wait for), or falls
under the goal after reaching it. A goal at or under the floor applies
from the tick it lands unless the stall-safe base cuts it, and a cut
one never reaches the goal. Polled, the same test reads `duty_applied_q15` (or
the ident aggregate `duty_mean_q15`) against the goal written, once the
slew is over; `limit_flags` then says why.

**Blind band.** Under the window floor (`window_floor_q15`, 5.3% on
osc-dev-v006) the shunt reports nothing and the servo applies
a stall-safe base duty instead of trusting its ceiling:
`min(i_lim x R / Vbus, floor)` from the identified `r_q12`, whose stall
current is at most the limit. A servo with no identified resistance
(`r_q12` 0) has the floor as its base, so the lowest duty it applies to
a goal above the floor is the floor itself, and stall current in the
blind band is bounded by `floor x Vbus / R` of the actual winding:
0.085 A for a 4.9 ohm winding on 7.9 V, and under the 300 mA class
limit for any winding above 1.48 ohm on 8.4 V. If the floor
already draws more than the limit, the applied duty stays pinned at
the floor and the stall timer decides (DES
`virgin_blind_band_passes_to_the_window_floor`).

**Shunt burst.** The high-rate shunt capture (`arm` at `0x198` in
CONTROL, with `duty_q15` at `0x196` and `chans` at `0x19A`; read back
from the BURST section) steps the bridge to `duty_q15` halfway through
a window of about 1.04 ms in which the kernel does not tick: no
limiter, stall timer or endstop acts on the step. The step drives for
the half window, about 0.52 ms, and the first kernel tick after the
window replaces it. What bounds a burst is its arm. `arm` 1 is refused
unless all of these hold at the kernel tick that sees it:

- torque on, mode OpenLoop, no TEL burst running (`tel_count` 0), no
  fault latched, and `chans` naming only defined extras;
- the applied volts are at most 3.2 V: `(|duty_q15| x vbus_counts) >>
  15` at most the board's cap in vcounts, 794 on osc-dev-v006. That
  is a step of at most 13284 (40.5%) on a 7.9 V rail (`vbus_counts`
  1961) and 23899 (72.9%) on a 4.39 V USB rail (1090);
- at least 100 ms have passed since the previous accepted arm: 2000
  kernel ticks, and the kernel does not tick during a capture, so the
  gap in time is never shorter;
- the raw `pos` lies strictly between `pos_min_soft_counts` and
  `pos_max_soft_counts`, or the stall permit is granted (`limit_flags`
  bit 3; the `stall_permit` request byte alone does not count).

A refused arm drives nothing and publishes BURST `state` (`0x2C1`) 4,
`REJECTED` (0 idle, 1 armed, 2 capturing, 3 done); it stays there until
the host writes `arm` 0, and it starts no spacing. The most a host can
cause, dead or buggy, is one pulse of at most 3.2 V for about half a
millisecond every 100 ms inside the soft limits (control-theory "Open
Loop Under the Same Band"; unit `burst_over_the_volts_cap_is_rejected`,
`burst_inside_the_spacing_is_rejected`,
`burst_outside_the_soft_limits_needs_the_permit`,
`burst_at_the_cap_still_arms`).

**Around a drive.** What `osc` does with these registers, the
reference for any host that drives OpenLoop (control-theory "Open Loop
Under the Same Band" has the planning behind it):

- Before any drive it reads what it plans against:
  `current_limit_counts`, `stall_yield_counts`,
  `stall_tau_trip_counts`, the soft and physical position limits,
  `raw_min` and `raw_max`, `r_q12`, the shunt and amplifier constants
  that turn counts into amps, one telemetry read for `vbus_counts`
  and `window_floor_q15`, and `window_v_floor_q15`. A current floor of
  0 refuses every drive; a stall
  yield not under the limit, or a collision trip over twice it, is
  warned about. Identification also refuses soft limits that span the
  whole pot (a servo `osc cal` never ran on) and an abort threshold
  more than a quarter over the limit.
- Each drive writes `ident_agg` 1 (sec 5.10) before its first read,
  its `mode`, then `torque_enable` 1, then, only
  for a drive that stalls on purpose (cal's stop approach, the
  resistance stop ladder, the held burst route, the current verify's
  stop stalls), `stall_permit` 1. A held permit is rewritten every
  250 ms, between commands and inside every pause.
- Every exit of a drive - done, aborted, interrupted or failed on the
  wire - writes `goal_duty` 0, `torque_enable` 0, when it held one,
  `stall_permit` 0, and `ident_agg` 0, in that order, and a guard
  around the drive writes them again with the other goals, `tel_count`
  and `tel_mask` zeroed. A killed host skips that; the lease then runs out inside the
  second and the servo's own protections carry the rest.
- A shunt burst stages `duty_q15`, `chans` and `arm` 1 under HOLD and
  fires one COMMIT, then polls BURST `state`: 3 `DONE` walks the
  pages, 4 `REJECTED` is a refused arm. `REJECTED` does not say which
  rule refused it. `osc` plans every arm inside the rules above, so it
  treats a rejection as a failure: it writes `arm` 0 to release the
  section, and the run ends (unit `a_rejected_arm_reports_and_releases`).

### 5.9 Health (osc-servo)

Five u16 registers in TELEMETRY, after `window_floor_q15`, say how the
servo itself is keeping up. Every build carries them; the chip side
writes them, not the kernel.

| addr  | name                 | access | meaning                                                                                              |
| ----- | -------------------- | ------ | ---------------------------------------------------------------------------------------------------- |
| 0x26A | `tick_load_mean_q15` | RO     | mean share of the kernel period the tick interrupt took over the last 4096 ticks (0.2 s), Q15       |
| 0x26C | `tick_over_count`    | RW     | kernel ticks whose interrupt took longer than one period, transport preemption included; wraps      |
| 0x26E | `tick_lost_count`    | RW     | kernel ticks that never ran: the previous tick was still running or interrupts were held off; wraps |
| 0x270 | `tel_drop_count`     | RW     | TEL rows dropped because both stream buffers were waiting for the wire (sec 5.6); wraps              |
| 0x272 | `stack_free_min`     | RO     | smallest free stack seen since boot, bytes                                                           |

The RW registers follow the TELEMETRY-COMMON clear contract (sec 5.4):
a host writes 0 and the servo counts on from there. The mean load
reads as a percentage of the period (50 us at 20 kHz) as
`q15 x 100 / 32768`, 16384 is 50%, and covers 0.2 s so a single read
does not catch one medium tick. Each tick is timed from the first to
the last statement of the tick interrupt, so it counts any transport
interrupt that preempts the tick and leaves out interrupt entry and
exit. A bus transaction preempts a tick for longer than a period, the
read of these registers included, so a few `tick_over_count` counts per
transaction are normal; a kernel that overruns by itself shows hundreds
per second. A tick counts as lost when the interrupt runs a whole
period or more behind schedule, judged once per 16 ticks: a late window
that the next one catches up costs nothing, and one window counts at
most 16. Both counters update every 16 ticks (0.8 ms), the mean every
4096. No kernel tick runs during a shunt burst (sec 5.8); the ticks
before it that did not fill a 16-tick window count toward neither
counter, and the first window after it counts no lost ticks.
`stack_free_min` reads 0 until the first stack scan completes, about
10 ms after boot.

### 5.10 Identification aggregate (osc-servo)

Six RO registers in TELEMETRY, after `vmotor_bias_counts`, fold the
per-tick current, drive-window differential and applied duty over
16-tick windows (0.8 ms at 20 kHz), so a polling host reads
decimation-free means without a TEL burst. One RW byte in CONTROL,
after the position table window, switches the fold:

| addr  | name            | width | access | meaning                                                              |
| ----- | --------------- | ----- | ------ | -------------------------------------------------------------------- |
| 0x1E0 | `ident_agg`     | bool  | RW     | 1 runs the aggregate; 0, the default, costs the kernel nothing      |
| 0x25A | `i_mean_counts` | i16   | RO     | window mean of the bias-subtracted current, counts                   |
| 0x25C | `i_min_counts`  | i16   | RO     | window minimum of the same                                           |
| 0x25E | `i_max_counts`  | i16   | RO     | window maximum of the same                                           |
| 0x260 | `vdiff_mean`    | i16   | RO     | window mean of the drive-window `va - vb`, counts                    |
| 0x262 | `duty_mean_q15` | i16   | RO     | window mean of the applied duty, Q15                                 |
| 0x264 | `agg_seq`       | u16   | RO     | windows published; wraps                                             |

`ident_agg` is RAM, off at boot, and a write lands at the next medium
period's CONTROL read like every CONTROL field. While it is off the six
registers hold their last window and `agg_seq` stops. Turning it off
drops the window in progress, so the first window after the next enable
spans 16 fresh ticks and `agg_seq` counts on from where it stopped. The
servo writes the block mid-tick with `agg_seq` last: a host that reads
the block, re-reads `agg_seq` and finds it unchanged holds one window.
A tick whose window the shunt cannot read contributes the last valid
current, and one the terminal taps cannot read the last valid
differential (the two floors of sec 5.8 may differ), except that the
current reads 0 from the first tick nothing drives, as the stream's
`current` does (sec 5.6). A host
that reads the aggregate turns it on for its session and off when the
session ends (sec 5.8).

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
loop's job, not the transport's: the break detector fires once per
span, a break-length in [F17], so no receive wake can measure a pulse -
the servo's main loop samples the line pin and the RX ring's DMA counter
once per idle wake (~50 µs cadence off the ADC tick metronome) and
declares rescue after ≥300 µs of continuous low with the ring frozen. The
frozen-ring requirement is what makes the window aliasing-proof at any
host baud: data cannot hold the line low a whole byte-time without
completing a character, and a completed character rings and moves the
counter — the pulse's own ringed `0x00` (it chars ~a byte-time in)
re-anchors the window and everything after it is provably byte-less. The
servo's own TX is the one low the counter cannot see: HDSEL keeps own
bytes out of the ring [F9], so a sample landing on a low bit of an own
byte finds the ring frozen. A sample taken while the servo transmits
therefore restarts the window, and the pin, the TX state and the
declaration share one critical section, so a TX ending between sample
and declaration cannot slip through. Sample spacing needs no bound:
every new fall of the line rings a character within a byte-time, so
ring progress, not sample density, proves the low continuous. The
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

- Config- and calib-region writes are always allowed (normal field
  validation applies) and hit only the live RAM table - volatile until
  saved.
- `MGMT SAVE` is the only flash-touching operation: requires torque
  disabled (else `access`), programs the persisted images (power-safe
  A/B alternation on the reserved pages), and acks **after**
  completion - the servo is genuinely stalled during program, so hosts
  use a SAVE-specific timeout. FACTORY shares the stall and the timeout.
- No write is torque-gated — there is no section lock. Field validation
  rules still apply to every write; anything genuinely unsafe to change
  mid-motion is the control kernel's job to sequence, not the table's to
  forbid.
- A config-dirty bit in telemetry reports modified-since-save
  (`status_flags` bit 0, §5.4).

Two images, each with its own A/B slot pair and sequence number, one
8-byte header each: magic, layout version, seq u16, body length u16,
CRC-16/ARC u16 over the header and the body. The CONFIG image (magic
`C`, version 5) is the CONFIG region ++ the PROFILE region, 200 B in one
256 B page per slot. The CALIB image (magic `K`, version 3) is the
CALIB region ++ the position table's 256 host-written calibration
points i16 LE (the fixed last one is not stored): 776 B, four pages per slot, one CRC, so the
calibration and the table it validates save and load as one unit and a
save cannot tear between them; tables to come join this image (a layout
change bumps its version). A CRC-valid image of another version boots
*stale*, never migrated (sec 5.7). On the osc-servo the two CALIB slots
sit at the front of the 4 KB CALIB flash region, 1 KB apart, with 2 KB
spare behind them.

What SAVE does, in order: settle the position table to what the kernel
applies (a load in progress or a rejected array becomes the identity,
so a reboot never applies a table the kernel did not); run the
data-state checkpoint (a mismatched stamp still persists); program the
CONFIG image, then the CALIB image, each readback-verified, an image
advancing its A/B state only on its own verify so a later failure
leaves the earlier one durable and the `hardware` nack honest; on
success clear the dirty bit and retire the virgin and stale reasons
(sec 5.7). A power cut between the two images boots a new/old mix, and
the boot checkpoint reports it as `STAMP_MISMATCH` when anything
covered differed (DES `torn_save_boots_stamp_mismatch`,
`torn_calib_save_boots_the_previous_calibration_with_its_tables`).

Timing: SAVE erases and programs five pages (one CONFIG, four CALIB),
each erase 2.5 ms typical and 3.0 ms max, each program 1.5 ms typical
and 2.0 ms max on the V006, so the stall is ~20 ms typical and 25 ms
max. FACTORY erases all ten slot pages: ~25 ms typical, 30 ms max. A
~50 ms timeout covers both with margin.

Side effects: flash wear drops from program-per-write to
program-per-session, and the ~20 ms write stall stops ambushing hosts on
ordinary config writes - it happens exactly once, at a moment the user
chose, with torque provably off.

### 9.5 Reboot / factory

`MGMT REBOOT`, `MGMT FACTORY` - conventional semantics (as in DXL);
payload details live with the implementation. FACTORY erases every slot
of both saved images (config and calib with its tables, sec 9.4), not
just the live table, then stages the reboot: the erased store is the
factory state, and the servo comes back *virgin* (sec 5.7) on board
defaults, the position table at the identity. It shares SAVE's torque
gate, and a failed wipe nacks `hardware` without rebooting. It is the only
exit from `CONFIG_CORRUPT`.

## 10. V006 resource map

| resource            | use                                               |
| ------------------- | ------------------------------------------------- |
| USART1 + HDSEL, PC0 | the bus                                           |
| DMA1 CH5            | RX ring (circular, armed once)                    |
| DMA1 CH4            | TX stream (enable-when-ready)                     |
| DMA1 CH3 + SPI1     | CRC engine (no pins) [F6]                         |
| DMA1 CH1            | ADC                                               |
| DMA1 CH7            | TIM2_CH2: zeroes the break detector per rising edge (§3.4) |
| DMA1 CH6            | copy-once snapshot buffer (§4.2); CH2 free        |
| SysTick             | framer deadlines A/B, reply gap, reclaim             |
| TIM2, CH1 on PC0    | the break detector (§3.4)                         |
| TIM1                | motor control                                     |
| EXTI                | unused — no transport consumer at all (§9.3)      |

Notably absent (vs a DXL-style transport): input-capture edge timing,
TIM-compare TX kickoff, an RDT register and its tuning surface,
byte-stuffing encode/unstuff, the FF-FF-FD hunter, software fold-CRC,
and a direction buffer with its TX_EN pin.

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
| F15 | LBD runs sans LINEN, both chip families: length-qualified (≥10-bit spans only — 0 fires on framing-error injection and high-baud garble), safe flag-selective write-0 clear mid-traffic, one event per any-length span, **latched at the span's END** (== bit 10 for the 10-bit law break; a rescue pulse's wake arrives after the line rises), entry stamps 4 ticks p-p on a 400 µs grid; latched FE/NE/ORE with no interrupt enabled are harmless through marination. Measured with RX on its own pin (HDSEL=0); see F16 | bringup measurement, V006 + V203 |
| F16 | V006 never sets LBD while HDSEL=1, in any configuration tried (LINEN, LBDL, pin mode, RE alone, arm order); the same die sets it with HDSEL=0 | bringup measurement, V006 |
| F17 | TIM2 break detector on the HDSEL pin (gated to count only while the pin is low, zeroed by DMA on every rising edge, overflow at 9.25 bit-times): 100/100 law breaks at 0.5M/1M/2M/3M at one ISR entry each, zero entries on an idle line, zero false breaks from frame data (`0x00` bytes included) or faster-baud garble, frames intact; a swept low wakes exactly when its gated count passes the reload, monotone at 4-tick resolution; a 1 ms low is one entry, a 5 ms low one wake plus one silent re-fire per 65536 ticks | bringup measurement, V006 |
