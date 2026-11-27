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
- **Drive discipline**: a servo pin is AF open-drain for good, transmitting
  included (RM Table 7-3, the half-duplex TX configuration). A servo only
  ever pulls the line low; the host-end pull-up (plus a weak keeper on each
  servo board) holds mark. A servo therefore has no release deadline and
  writes no GPIO per frame. The osc adapter's pin is AF open-drain for good
  too, so its rising edges ride the pull-up, which must sit at its end. A
  host may instead drive its own frames push-pull for fast edges, releasing
  to open drain within the reply gap (sec 7). A node that idles push-pull
  clamps every other talker [F8]: this rule is the buffer replacement, not
  an optimization.
- **Own-TX echo**: none on V006 — HDSEL gates RX during TX [F9]. Firmware
  never needs echo masking. The osc adapter runs full duplex (its TX and
  RX pins both on the wire) and turns its receiver off while it holds the
  wire, so neither its own bytes nor its own breaks reach its ring [F18].
  The protocol itself is agnostic.
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
timing model exact - the framer's footprint algebra counts the break as
one byte slot, so an over-long break is a constant error tax on every
span - and the law shape is
precisely the LIN break definition (LBDL=0), so any LIN-capable
receiver gets hardware break detection with a deterministic 10-bit
anchor. Bridge-class hosts use exactly that: the osc adapter frames
every reply at its detected breaks [F18]. The servo cannot: its bus is
one pin under HDSEL, which disables the USART's LIN break detector on the
target silicon [F16], so it times the low on the pin instead - a
detector that qualifies at 9.5 bit-times, past the longest data low (9)
and inside the law break (10), §3.4 [F17]. The half bit above 9 absorbs
the open-drain rise that lengthens every low (sec 2). Hardware `SBK` is off-law
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
  (lows under 9.5 bit-times are invisible to the detector, and no
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
  dominant span held past 9.5 bit-times fires it, a length valid data
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
  (The sec 9.3 CAL ruler times breaks too, by the detector's hardware
  stamp, never by the wake.)
- **Breaks are not countable events.** Service can lag the wire, and N
  breaks can coalesce into one service; all break handling MUST be
  idempotent, and freshness (did bytes ring since the last service?)
  MUST be derived from the ring, never assumed.

**The accepted limitation** (the price of break-framed delimiting):
garble that forms a plausible frame header parks the resolver until data
kills it — footprint-fill CRC (≤ 258 bytes) or the starve horizon (64
byte-times of ring silence), whichever comes first, per plausible junk
anchor. The length qualification shrinks that surface: only garble
containing a dominant span past 9.5 bit-times (slower-baud traffic heard
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
software at patch time (§3.2). Sequence: law break (§3, a
bracketed-M `0x00` character) → enable
UART TX DMA from offset 1 → simultaneously
enable SPI-CRC DMA from offset 0 → the CRC engine outruns the wire 8:1,
so `TCRCR` is patched into the trailing CRC bytes long before the shifter
needs them (fire-first, append-later, no deadline race) [F6]. On the
last arm's TC: TX DMA off. The pin is never written (sec 2), so nothing
at TC has a wire deadline. A reply may pause between its DMA arms while
the servo's control kernel runs: each pause is about one kernel body
(under about 60 us at 20 kHz), far inside the starve horizon (sec 3.4).

No hardware-timed kickoff: TX start is "enable the channel when ready" —
the break makes reply timing non-critical, which deletes the TIM-compare
kickoff machinery, the RDT register, and its whole tuning surface.

Payloads of two bytes or less, and a 4-byte payload from a 4-byte
boundary, copy into the reply buffer directly - cheaper than arming a DMA
channel for a few bytes: a field of up to 4 bytes read on its own is
copied by one CPU load.
Larger payloads (the READ/GREAD case) are **copy-once**: staging the reply
copies the table span into a dedicated engine-owned snapshot buffer, and
the CRC feed and the wire TX arm, armed separately when the reply
triggers, both read from that snapshot instead of the table. The copy DMA
(CH6) takes the span's 4-byte-aligned middle in word beats, fire and
forget, and the CPU copies the up to three bytes at either end, one
naturally aligned load each; the frame sits in the snapshot shifted so
the payload lies at the same address mod 4 as its source. A span of up
to 7 bytes the CPU copies alone, one naturally aligned load per field.
Copy, CRC feed, and wire shift-out are three independent DMA/hardware engines
running concurrently, not a blocking chain - nothing polls or waits on
the copy. Correctness comes from relative speed and scheduling order
instead: the copy is kicked off earliest (at stage time, ahead of the
trigger) and is also the fastest of the three (a word beat costs what a
byte beat does, so 240 B take about 380 HCLK, and the CRC engine itself
runs ~8x wire speed, F6), and its channel (CH6) sits above the CRC-feed
channel on the bus-arbitration ladder (the RX ring alone owns the top -
see `osc-servo-transport.md`, DMA priority ladder) - so the copy is
guaranteed done before either downstream consumer reaches the bytes it
needs, by construction, not by synchronization. The CPU's ends are
written before the trigger, so neither consumer can reach them first.
The one copy still buys two things a direct-from-table stream couldn't:
the table address's parity becomes irrelevant to the wire/CRC engines,
and the wire and the CRC read the same bytes. With an odd frame base the
CRC feed starts one byte below it, on a byte kept 0, which leaves the
CRC unchanged as the alignment byte does (sec 3.2).

A read never delivers a field torn: every field it covers whole is
copied in one access, whatever its width. Fields of one read may come
from different control ticks. In a profile read (sec 5.2) the promise
holds for the first span, for every span of up to 7 bytes, and for a
longer span that starts at a 4-byte boundary when every span ahead of it
starts at one too and runs a multiple of 4 bytes; other longer spans
promise nothing, read such fields alone or through a TEL row. Values
that must be consistent with each other are read through a TEL row
(sec 5.6). Reads over 252 B split into multiple frames (sec 5.1), each
independently snapshotted and CRC'd.

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
  Every reply payload streams from a copy - the word-aligned reply buffer,
  or the snapshot buffer, which places the payload at its source's
  address mod 4 and feeds the CRC from an even byte (sec 4.2) - so the
  table address carries no constraint at all [F12 satisfied
  structurally]. WRITE addressing is likewise unconstrained: inbound
  payloads validate through the ring anchor, not the table address.

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
- Execution is sec 4.2's existing copy-once TX: each span is snapshotted
  at its cumulative offset and the wire and CRC arms stream the one
  contiguous copy, engine accumulating across arms [F6] - a scattered
  read costs the same one-copy-per-reply as a single-span read. Spans
  carry **no parity constraint** (addresses and lengths may be odd): only
  the total's parity engages the standard tail fold (sec 3.2) - same as
  sec 5's unconstrained plain reads. Alignment decides only how whole the
  fields arrive: the frame is shifted for the first span, so a later span
  lands at its own address mod 4 only when the spans ahead of it add up
  to that.
- Errors are read-time (§5.3): a slot index past the region, an empty
  slot, or a span leaving the table is `range`; a slot totalling past the
  252 B reply ceiling (§5.1) is `limit`.
- Scattered _writes_ need no counterpart: `WRITE+HOLD` per span plus one
  `COMMIT` is already atomic — inline scatter-writes would add cross-span
  validation complexity for no capability gain.

Implementation note (tearing): a span of up to 7 bytes is copied by the
CPU at any alignment, one natural load per field. A longer later span
whose payload offset matches its address mod 4 copies like a plain read,
every field whole. A longer mismatched span is copied in halfword beats
when both ends have the same parity, which can tear its 4-byte fields,
and in byte beats otherwise, which can tear any of its multi-byte
fields. Hence the sec 4.2 promise: read such fields alone or through a
TEL row (sec 5.6).

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
| 0x004 | `capability_flags`     | u32   | RO     | bit 0: the two TEL row registers below; bit 1: the experiment image (sec 5.8) |
| 0x008 | `hardware_revision`    | u8    | RO     |                                        |
| 0x009 | -                      | 1 B   | rsvd   |                                        |
| 0x00A | `tel_row_bytes_max`    | u8    | RO     | sec 5.6: the longest TEL row, bytes    |
| 0x00B | `tel_chunks_max`       | u8    | RO     | sec 5.6: the chunks a TEL row copies at most |
| 0x00C | -                      | 4 B   | rsvd   |                                        |
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
  `0x220..0x280`, `0x2C0..0x400`), except the profile region's own pin
  at `0x280..0x2C0` (§5.2).
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

A bias node that divider legs return to is published as its own divider
(`vmotor_bias_top_ohm` / `vmotor_bias_bot_ohm`, beside the boot-measured
`vmotor_bias_counts`). Leg current moves that node under drive: a
terminal difference cancels the move, a single-terminal reading does
not, so a host converting one terminal absolutely corrects the bias from
the legs.

Durations the kernel times are counts as well: `stall_time_ticks` and
`pos_error_time_ticks` (u32) are kernel ticks at CALIB `tick_hz`, and a
host converts milliseconds with it.

One pair is per amp instead: the position reading's current terms
`pot_steady_counts_per_a` and `pot_fast_counts_per_a` are linearized
pot counts per amp of the current reference, tuning parameters a user
sets for the load (kernel sec 2.2), so a value tuned on one servo carries
to a board with another shunt. The servo converts through its own sense
chain with a reciprocal fixed at build time, one multiply per sample and
no divide.

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

The angle map (`angle_min_cdeg` at `raw_min`, `angle_max_cdeg` at
`raw_max`) may descend: degrees are counterclockwise-positive facing the
horn (ROS REP 103), while counts follow the pot.

A conductance between the two sense chains is a ratio of their counts:
`g_window_q016` (sec 5.8) is counts of shunt current per count of the
terminal taps, Q0.16, so a host converts it with both chains' primitives,
`g [S] = g_window_q016 / 65536 x (current counts per A)^-1 x (terminal
counts per V)`. On the osc-dev-v006 1 mS is about 300.

### 5.6 Telemetry stream (TEL bursts)

High-rate telemetry rides the bus as a bounded burst of status frames
the servo emits on its own schedule - no extra wire, no side channel,
and every frame under the same CRC as the rest of the protocol. A burst
streams at 3 M only: below it a frame of the longest row does not clear
the wire inside its batch, so a short cable with good contacts is part
of the setup.

A row is a copy of the control table: up to two chunks of it, each a
run of table bytes at its own layout, copied whole once per control
tick. Five CONTROL registers arm a burst (osc-servo):

| addr  | name          | width | access | notes                                              |
| ----- | ------------- | ----- | ------ | -------------------------------------------------- |
| 0x1E2 | `chunk0_addr` | u16   | RW     | table byte address, even, under 1024               |
| 0x1E4 | `chunk0_len`  | u16   | RW     | bytes, even, at most 12                            |
| 0x1E6 | `chunk1_addr` | u16   | RW     | as chunk 0                                         |
| 0x1E8 | `chunk1_len`  | u16   | RW     | 0 = one chunk                                      |
| 0x1EA | `tel_count`   | u16   | RW     | a committed nonzero write streams that many rows   |

The chunks sit right ahead of the count, so one 10 B WRITE at `0x1E2`
sets the row and arms. The arm admits a row of at most
`tel_row_bytes_max` bytes in all (12, sec 5.4), the first chunk
nonempty and every chunk inside the table; a row that is not, or an arm
at any rate but 3 M, disarms instead of streaming. The write itself acks
(the ack precedes the commit that arms), so a host checks the row and the
rate first. The arm composes with HOLD/COMMIT, so a goal write and the
arm commit together, and the goal is taken at the first medium tick
after the commit (sec 5.7). A burst's first row lands on one of the two
control ticks of each medium period with room for the arm, up to 0.45 ms
after the arm (after its ack, when one is sent), and a frame can leave
up to 0.1 ms later than its rows alone would let it, when one of its
steps meets a tick the servo keeps for its heaviest work. The goal can
then be taken up to a medium period ahead of the first row; the shaft
has not moved by then, so the step's motion lands inside the capture.
Every row lands on its own tick; none is skipped. A committed
`tel_count` of 0 disarms.

A burst never shares a tick with the kernel taking a CONFIG or CALIB
write (sec 5.7): its first row waits while a committed write is not yet
taken, and never lands on the take's tick of the medium period. An arm
committed with such a write, or within a medium period after one,
streams its first row up to a medium period and a tick late (DES
`a_burst_never_streams_through_a_config_take`). A step committed in the
same COMMIT as a configuration write can then land ahead of the first
row; a host that captures a step commits the configuration first.

`tel_count` reads nonzero only while a burst is armed or streams: the
servo zeroes it once the burst's LAST frame is off the wire, and at once
for an arm that does not stream (a row that is no row, a rate under 3 M,
an arm a shunt burst parks, an arming ack the host's next frame cut
short). A host that reads it 0 after a burst knows the line is free.

Burst frames are ordinary status frames with result code `stream` - the
one result code that marks a frame no instruction directly owes.
Payload, all LE: a head, then the rows.

```
[0]    stream_seq  u8, increments per frame, wraps
[1]    flags       bit 0 = LAST frame of the burst; rest reserved 0
[2..4] tick        u16, the first row's sample_tick, low 16 bits
[4..]  rows        per row chunk 0's bytes, then chunk 1's
```

The row count is implicit in `LEN`: `(payload - 4) / row bytes`. A
frame carries at least one row; a host accepts any count the payload
holds and never assumes a batch size. osc-servo batches 16 rows per
frame, the burst's final frame fewer. Batching is what makes the CRC
affordable: framing overhead amortizes to well under a byte per row.

A host decodes a row by its chunks and the model's descriptor: a field
rides in a row when its bytes lie wholly inside one chunk, at the
chunk's offset in the row plus its offset in the chunk
(`osc-protocol::tel::Chunks::offset`). Every field keeps its width and
encoding from the table.

What a row means is when it was copied. osc-servo keeps a tick block
in TELEMETRY (`0x224..0x254`). The ADC's DMA lands both scans of every
PWM period at its front: the trough scan and the crest scan's taps and
shunt in reserved bytes (`0x224..0x236`), the crest scan from `pos` to
`ntc_raw` on its fields. The tick enters on the injected conversions
that sample the drive's low-side tap and the shunt around the crest: it
stamps `sample_tick`, copies that shunt to `current_raw` and the
trough's shunt and taps out of the next scan's way, and once the crest
scan has landed writes each crest tap under its terminal's name; the
kernel writes its conclusions on the window the previous command drove,
and both chunks are copied. The next scans land mid-tick (the trough)
and a period on (the crest), after the copies, so every row holds one
tick's tick block. A chunk over the medium-rate registers
(the estimates) carries them as last published, up to one tick old.
Host-written regions copy with per-field consistency only. A bus read
of the tick block is weaker (sec 4.2): each field it covers arrives
whole, but two fields may come from adjacent ticks (`pos` to `ntc_raw`
change when the next crest scan lands, the rest when the next tick
writes them). The DMA writes the terminal taps in sample order, high
side first, so no named field lies in its way.

| addr  | name              | width | notes                                                       |
| ----- | ----------------- | ----- | ----------------------------------------------------------- |
| 0x236 | `pos`             | u16   | raw pot |
| 0x238 | `vbus_raw`        | u16   | |
| 0x23A | `ntc_raw`         | u16   | |
| 0x23C | `current_raw`     | u16   | the shunt at the counter peak, 31 timer ticks past it; raw, the window term in it |
| 0x23E | `vmotor_a`        | u16   | |
| 0x240 | `vmotor_b`        | u16   | |
| 0x242 | `duty_ticks`      | i16   | the PWM compare width the window measured was driven at, logical direction, negative in reverse; full scale is `pwm_period_ticks` (CALIB, RO) |
| 0x244 | `current_trough`  | u16   | the shunt in the trough, 135 timer ticks past it |
| 0x246 | `i_tick_counts`   | i16   | the winding's current: bias-subtracted window current times the board's settle gain for the drive width, less the window term `g_window_q016` names (sec 5.8); through windows the shunt cannot read, held in OpenLoop and the current reference in the closed loops; 0 from the first tick nothing drives |
| 0x248 | `vdiff_counts`    | i16   | the window's `va - vb`, logical sign; held through windows the terminal taps cannot read |
| 0x24A | `vmotor_a_trough` | u16   | |
| 0x24C | `vmotor_b_trough` | u16   | |
| 0x24E | `tick_flags`      | u16   | bit 0 the shunt read the window, bit 1 a fault is latched, bit 2 the terminal taps read the window |
| 0x250 | `sample_tick`     | u32   | the tick's count |

The chip's 1.2 V internal reference is not in the scan: behind another
channel its reading follows that channel's code. `vref_raw` (u16, RO,
`0x274` in TELEMETRY, beside the boot references) is its one reading,
taken at boot alone on the converter with the driver parked: VDD = 1.2 V
x 4096 / `vref_raw`, 0 = no reading. No tick writes it, so a host reads
it once.

Common rows fit 12 B in at most two chunks: the six raw fields (`0x236`
2 B, `pos`, and `0x23C` 10 B, current to trough), identification's four
(`0x236` 2 B, `pos`, and `0x242` 8 B, duty to differential with the
trough riding along), the R/L set (`0x238` 12 B, rail to duty with the
NTC riding along). Whether a row's
current is a fresh window is a pure function of its duty:
`duty_ticks != 0` and `|duty_ticks| >= i_window_min_ticks` (CALIB), the
kernel's own window rule (`tick_flags` bit 0 says the same for a row
that carries it). The linearized pot is not streamed: a host linearizes
`pos` with the servo's own table (sec 5.7).

**Margin row (osc-servo, bench images).** A bench image, built with the
`loop-margin` feature (osc-servo-kernel sec 2.7), sums a triangle into
the velocity loop's reference in Velocity mode while a burst streams
under a loaded bus dead-man (sec 5.8), for a host that reads the speed
and position loops' margins. Two CONTROL fields set it and one TELEMETRY-EXT field publishes
the reference it lands on:

| addr  | name                  | width | access | notes                                                                                 |
| ----- | --------------------- | ----- | ------ | ------------------------------------------------------------------------------------- |
| 0x1EC | `inject_accel_q88`    | u16   | RW     | the triangle's slope, Q8.8 c/s per medium period (the unit of `accel_limit_q88`), read at most 0x7FFF; 0 off |
| 0x1EE | `inject_half_periods` | u16   | RW     | its half-period N in medium periods, read at most 512; 0 off                          |
| 0x3D4 | `omega_ref_cps`       | i32   | RO     | the velocity loop's reference, raw csQ16 as `omega_hat_cps`; 0 on the production and experiment images |

The triangle's peak is slope x N / 2 and its frequency 1000 / N Hz, at
most 1 kHz (Nyquist of the 2 kHz medium rate), at least 1.95 Hz. The sum
is clamped to `speed_ceiling_cps`, the bound on the Velocity reference
it rides (sec 5.8); the triangle pauses on a period after
any limit flag but the permit (`limit_flags`, sec 5.8), and its phase
holds through the pause. The fields are volatile and the servo never
clears them: a host zeroes both before it drives and when it is done. The
production and experiment images accept both and ignore them.

The margin row is 12 B in two chunks: `0x258` 8 B (`theta_hat_q16` and
`omega_hat_cps`) and `0x3D4` 4 B (`omega_ref_cps`). All three are written
at the same medium-period publish, so every row holds one period's
triple, and one row a period at a fixed offset of the unwrapped `tick`
is the period's sample. With U, W and TH the fundamentals at the
triangle's frequency over whole cycles of `omega_ref_cps`,
`omega_hat_cps` and `theta_hat_q16` (TH through the period difference,
which removes the cruise's ramp), the speed loop's gain is W / (U - W)
and the position loop's `kp` x TH / U, `kp` CONFIG `p_kp_q88` / 256.

The wire contract during a burst: the host is silent. The servo owns
the line from the arm's ack (or the arming COMMIT's silence) through
the LAST-flagged frame, then the line frees on its own - no polling, no
handshake. There is no abort: the servo ignores breaks and bytes that
reach it in that window, and whatever a host sends there is lost, along
with any burst frame it collides with (a CRC fail and a `stream_seq`
hole). A host waits for LAST, or for its own timeout, before it speaks
again; the first instruction after LAST is served normally, a fresh
re-arm included. The window is bounded by `tel_count`: any count the
u16 holds arms, about 3.3 s of control ticks at osc-servo's 20 kHz. A
drive under a long burst is bounded by the servo's own limits (soft
limits, stall timer, current limit), not by the host. An arm parks
while a shunt burst is armed, capturing or committed and not yet judged
(osc-servo, sec 5.8): the two share the ADC, and a shunt arm in turn
refuses while `tel_count` is nonzero.

Integrity is the point: a corrupted burst frame fails CRC and is
dropped whole by the host framer - it can never decode as plausible
data - and the drop is visible as a hole in the `stream_seq` numbering
and as a jump in the next frame's `tick`. A host times rows by the
stamps, never by `stream_seq` times a batch size: row i of a frame is
tick `tick + i`. ALERT on a burst frame is the device's fault state at
its first row, per the sec 5.3 device-level contract; faults latch and
the host is silent through a burst, so a fault in any later row shows
in the next frame's ALERT (and in the row's `tick_flags` bit 1).

Timing: a batch completes every 16 control ticks. The producer never
blocks: a row that finds both frame buffers waiting for the wire is
dropped and counted in `tel_drop_count` (sec 5.9), with no hole in
`stream_seq`; the next frame's `tick` jumps over it. Safety through the
silent window is the servo's own - the current limit (held in every
drive mode, OpenLoop included, sec 5.8), soft-position clamps, the
stall timer and fault latches run in firmware regardless of the bus -
and the host's supervisory reads resume between bursts. The one thing a
silent host cannot do is renew a stall permit: a burst that needs one
has to end inside the lease (sec 5.8), and `osc-ident` refuses a rung
longer than 0.7 s while it holds a permit.

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
reaches the kernel whole, at one tick near the end of a medium period,
within a medium period (0.5 ms) of the commit, or a period later when
that tick lands inside the next commit. If that write leaves
`recip_ke_q == 0` or `ke_vpc_q == 0` under a running Velocity or
Position loop, or a limit unset (below) under any running closed loop,
the same fault latches at the next medium tick and the drive stops.
Until then the loop keeps running on the Ke and the limits it had, so
a live write of a zero Ke can never run a loop open, nor a live write
of a zero limit run one unbounded.
A virgin servo jogging in OpenLoop therefore shows no ALERT and its
captures stay clean (DES `virgin_servo_drives_openloop_and_current_without_alert`,
`live_zero_ke_write_stops_a_running_closed_loop`).

**Limits unset.** The two limits every closed mode clamps against,
`current_limit_counts` (the current cap, sec 5.8) and
`speed_ceiling_cps` (the speed ceiling, sec 5.8), read 0 when unset,
and the factory block holds 0 in both: no compiled value is right for a
motor the board has never met. Neither has a meaningful 0 in a closed
loop, so 0 fails safe. While either is 0 the physics belt refuses
Current, Velocity and Position, whatever `data_flags` says: an enable
or a mode change into one latches the `data` fault (code 7) and the
servo stays disabled, and a live write of 0 stops a running loop as a
zero Ke does (above). `limit_flags` bit 5 `UNSET` (sec 5.8) names the
state every medium tick, torque on or off, so a host sees it before any
enable; the fields themselves read 0. OpenLoop is never refused: it
reads no ceiling, and with the cap unset its band is the board's
overcurrent trip `oc_trip_counts` (sec 5.8). Calibration writes the cap
first, the ceiling once the angle scale is known; with both set the bit
clears within a medium period of the write and Current mode is admitted
under the data reasons that leave it open (DES
`a_servo_with_its_cap_unset_refuses_closed_loop_and_runs_openloop`,
`a_live_zero_cap_write_stops_a_running_closed_loop`,
`live_zero_limit_write_stops_a_running_closed_loop`,
`the_factory_block_refuses_current_mode_and_runs_openloop`,
`virgin_servo_admits_current_mode_once_both_limits_are_written`).

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
| 7   | 9    | `deadman`        |

Code 8 is unassigned. Any set bit forces the drive off; the
`torque_enable` 0 to 1 edge is the only acknowledgement, and a
still-present condition re-latches at once. `position_error` latches in
Position when the profile's lead over the observer stays past
`pos_error_counts` for `pos_error_time_ticks`. The timer does not count
while the press fold is latched under `stall_response` Yield (`YIELD`,
sec 5.8): the fold sags the shaft on purpose, so the servo holds at
`i_cont_counts` instead of turning the drive off, and the timer starts
over, so once the fold releases the error gets the whole time at full
torque (DES
`a_yielded_hold_sags_past_the_error_with_torque_on_and_climbs_back`,
`a_hold_capped_under_the_rating_latches_position_error`). Besides the raw pot screen,
`sensor` latches when the crest scan a tick waits for never lands, and
the tick writes the drive off (osc-servo-kernel sec 2.3).
The kernel reads `torque_enable`, `mode` and the goals once per medium
tick (2 kHz), so an enable, a disable, an acknowledgement, a mode change
and a new `goal_duty` take effect at the first medium tick after the
commit, within 0.5 ms; a `goal_current` or a closed-loop reference
follows three ticks after that, and the TELEMETRY registers show the
result later in the same medium period.
`stall` latches in OpenLoop as it does in the closed loops: there the
stall timer runs off the duty ceiling (sec 5.8), so with
`stall_response` at its boot value of Fault, an OpenLoop drive held
against a stop or a jam for longer than `stall_time_ticks` latches it
unless a stall permit is live.

**Drive polarity.** `drive_polarity` is a calibration fact: which way
the motor turns the pot's counts under a positive drive. The firmware
does not check it against the motor at run time. `osc-ident` finds it
from pulses each way and leaves it in the set it stamps, and the field is
under the plant stamp (below), so a write that changes it reads as
`STAMP_MISMATCH` and the next enable of a closed loop is refused until a
tool stamps the set again (DES
`a_polarity_write_refuses_closed_loop_until_the_stamp`). After a motor
is rewired or replaced, run the calibration again; a tool that already
knows the wiring flips the field with torque off and stamps the set. A
hand edit alone never closes a loop.

A stored polarity that is wrong under a matching stamp inverts every
closed loop, and the soft-limit walls with it: the walls judge a current
by its sign, so past a limit they close the side that would bring the
shaft back. In Position and Velocity the loop drives the shaft away from
its goal and through the soft limit toward the hard stop, at the current
limit, until a timer acts. In Velocity it is the press's verdict on
the stop, `stall_time_ticks` after the observer reads the shaft still
(sec 5.8): `stall` under `stall_response` Fault, and the drive goes off;
under Yield the fold to `i_cont_counts`, which the servo then holds
against the stop. In Position the position error timer runs too, from
the error passing `pos_error_counts`, early in the run. When it ends
before the press's verdict, as it does whenever `pos_error_time_ticks`
is under the run plus `stall_time_ticks` on the stop, `position_error`
latches on the stop or short of it and the drive goes off, under either
response. Under Yield a fold that comes first stops the timer, and the
servo holds against the stop at `i_cont_counts`, as in Velocity. DES
`a_reversed_motor_runs_to_its_stop_and_a_timer_latches` has the travel,
the times and the currents on the bench MG90's plant, `stall_response`
Fault; `a_reversed_motor_under_yield_holds_its_stop_once_folded` has
Yield at a 500 ms and a 2 s timer. In OpenLoop and Current the drive is the host's: the shaft runs
to the stop the same way, and the press's rule applies there as at any
stop. The host's direction check before it closes a loop (`osc-ident`'s
probe, the bench harness) catches leads swapped at assembly.

**Plant stamp.** `plant_stamp` (u16, RW, `0x0AE` in CALIB, persisted)
is the host's CRC over the set it *intended* to write: the identified
and calibrated fields plus the effective position table. Firmware
recomputes it over what actually landed at every checkpoint and reports
a difference as `STAMP_MISMATCH`, so a write that never landed, a torn
save, a hand edit and a table rebuilt under old constants all read the
same way.

```
stamp = max(1, CRC-16/ARC("osc-plant-1" ++ covered bytes ++ points))
```

The covered bytes are the 44 covered fields' own table bytes in table
order (94 B): the position limits, the loop gains, the deadband, the
speed ceiling and the acceleration limit, drive polarity, the stall speed gate,
the observer gains and the position-error threshold in CONFIG; the pot
stops, the motor constants, the friction model, the brush resistance and
its full-weight speed, the window conductance (sec 5.8), the angle map, the
winding estimate's rise constant, the board NTC's reference
point, slope and curvature, and the gear class in CALIB. The points are the position table's
256 host-written calibration points i16 LE (sec below) while
`pos_lut_state` is LIVE, 256 zeros otherwise, so a table falling back to the identity changes the stamp by
itself. Not covered: identity and comms, the user-owned safety limits
(current, thermal, undervolt, `duty_max_q15`), the move speed
`velocity_limit_cps` (a user setting, sec 5.8), the position reading's
current terms (tuning parameters, sec 5.5), the raw sensor screen,
the ADC wide-code map, and the RO board facts install re-seeds. `0` is
reserved for *never stamped* and the recipe never produces it. The list
is exported once, in the descriptor's `stamp` block (`tag`, `covered`,
`pos_lut_points`); the CRC is the sec 3.2 checksum in software, ~600 B
and ~0.6 ms on the servo, on torque-off paths only (the SPI engine belongs
to the transport).

Checkpoints, the only places the verdict recomputes: boot after both
images load; a committed write to `plant_stamp` or to a covered field;
a table COMMIT; SAVE. A committed span that intersects a covered field
or the stamp marks `STAMP_MISMATCH` at once, whatever value it carried,
so a host that dies halfway through a set leaves the mismatch behind
(DES `partial_covered_write_blocks_next_enable`); its checkpoint
clears the mark once the live set matches the stamp, so a hand edit
put back reads clear again (DES
`restoring_a_covered_edit_clears_the_mismatch`). Under torque the
checkpoint waits: the write lands unverified and the recompute runs
when torque goes off (DES
`stamp_write_with_torque_on_verifies_at_torque_off`). SAVE checkpoints
before it programs and still persists a mismatched set (the reboot
recomputes anyway; flash is never refused).

The recompute outlasts the reply deadline, so the write and the
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
loop. `osc-ident` (install, save, restore) and `osc recover --from`
commit this way, and `osc stamp [--save]` blesses a hand-tuned set
explicitly. The drives that measure a set come before this sequence
and write only the CONTROL fields a drive uses (sec 5.8, around a
drive). Nothing restamps as a side effect: `osc set` of a covered
field leaves the mismatch standing until the value is put back, and a
new table redefines the domain the constants were fitted in, so it is
installed with its set under one stamp.

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
reads it (trajectory, position loop, soft limits and the stall speed
gate) are in linearized counts. The raw sample stays
raw for the published `pos` (which a TEL row carries) and the sensor-delta
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
`raw_min`/`raw_max` and lands LIVE, IDENTITY when the array corrects
nothing (every calibration point zero), or a REJECT, then runs the stamp
checkpoint. A table that corrects nothing is the identity at a COMMIT as
at boot, so a host returns a LIVE servo to IDENTITY by STOREing zeros
and COMMITting, and the state it then reads is the one the next boot
reads (unit `zero_table_commit_lands_the_identity_a_boot_gives`). The
reply leaves first: COMMIT reads back LOADING with
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
the host's job (`osc-ident table`).

A host linearizes a streamed `pos` with the same table and
interpolation (`osc-servo-core::pos_lut::interp_q4`). SAVE persists
the table beside the calibration in the CALIB image (sec 9.4); boot
re-validates the loaded calibration points against the stops loaded
with them and goes LIVE only when they validate and correct something,
the rule a COMMIT lands by; otherwise the
identity runs and the stamp reports the loss as `STAMP_MISMATCH` (DES
`lut_survives_save_and_reboot_until_factory`,
`corrupt_or_stale_calib_image_boots_identity_under_its_reason`).

**ADC wide codes.** The converter can stick at a code just under a
multiple of 256 while its input rises through the codes it then skips,
so one code stands for a band of inputs on any channel. The wide-code
map records them per board, since the converter is the chip's:

| addr  | name              | width | access | notes                                                       |
| ----- | ----------------- | ----- | ------ | ----------------------------------------------------------- |
| 0x0D5 | `adc_wide_mid_q4` | 16 B  | RW     | `[u8; 16]` in CALIB; slot `k` is ADC code `(k << 8) \| 0xff` |

A slot holds half the span the converter skips after its code, in Q4
counts; `0` is no wide code there, and the all-zero default changes
nothing. Calibration writes it; it is persisted with CALIB and not
covered by the stamp.

The kernel applies the map to every channel it consumes, where the
consumer reads the sample, so a stuck reading lands mid band. Its
derived fields carry the step and the raw fields beside them do not:

| derived field | from | step |
| ------------- | ---- | ---- |
| `i_tick_counts` | the drive window's shunt sample | whole counts, the slot rounded half up |
| `vdiff_counts` | the drive window's two taps | whole counts each, the slot rounded half up |
| `current_bias_counts` | the trough shunt sample as the zero | Q4 |
| `vbus_counts` | `vbus_raw` | Q4 |
| `t_ntc_cc`, `t_winding_cc` | `ntc_raw` | whole counts, the slot rounded half up |
| the estimates on the pot | `pos` | Q4, below |

`pos`, `vbus_raw`, `ntc_raw`, `current_raw`, `vmotor_a`,
`vmotor_b`, `current_trough`, `vmotor_a_trough` and `vmotor_b_trough`
stay the converter's codes in the table, in TEL rows and in the shunt
burst. A host converting a streamed raw field adds the slot itself,
`(code << 4) + wide_q4(code, map)` in Q4 counts
(`osc-servo-core::adc::wide_q4`). `vref_raw`, a boot mean of 16 codes,
has no consumer in the kernel, so its step is the host's alone.

On the pot the wide-code step is taken in raw counts
ahead of the table: a live table is read at the corrected raw count, in
the interval that holds it, floored to a Q4 count:

```
at  = (raw << 4) + map[raw >> 8]   when raw & 0xff == 0xff      (Q4 raw)
i   = at >> 8                      (at most 256)
lin = at + 16 c[i] + ((c[i+1] - c[i]) * (at & 0xff)) >> 4      (Q4)
```

A wide code is the last code of its interval, so a slot of a count or
more puts the band's middle in the interval above, at that interval's
gain. Past the last point (`i = 256`, `c[i+1]` read as `c[256]`) the
table is the identity; on the identity it is the bare slot. A stuck pot
reading lands mid band. A position table should
be built from readings that already carry the step, so the table holds
the pot alone. The raw sample stays raw, as for the table, and a host
linearizing a streamed `pos` takes the same step in raw counts first
(`osc-servo-core::adc::wide_q4`; DES `hold_rests_inside_a_wide_code`).

### 5.8 Limits and the stall permit (osc-servo)

Every drive mode clamps against the same current band (control-theory
"Limits"): the peak cap, the rail fold and
`goal_current`, then the press fold and the directional endstop. The closed loops clamp their current reference;
OpenLoop, which has none, holds a duty ceiling against the band from
the shunt alone, so a host-written `goal_duty` is a request the servo
may refuse to apply in full. With the cap unset (0, sec 5.7) OpenLoop,
the one mode that runs then, takes the overcurrent trip
`oc_trip_counts` as its cap, and `i_lim_counts` reads it (DES
`virgin_servo_stalls_under_the_board_trip`). Three registers let a host take part: a
permit to stall on purpose, a flag byte that names whatever is holding
the command back, and the duty under which the shunt reads nothing.
Like sec 5.7 these are model facts in model-specific space, and the
descriptor carries the addresses.

**STAT flash.** `stat_flash_ticks` (u16, RW, `0x194` in CONTROL) is the
bench clapper: a committed nonzero write lights the STAT lamp for that
many control ticks (20 kHz, so 2000 is 100 ms and 65535 about 3.3 s), above
the fault and setup lamp rules, after which the field reads 0 again. The
host's own time of the write is the event a recording is lined up to; the
lamp follows within one main-loop pass of the servo. Zero is a no-op.

**Soft limits.** `pos_min_soft_counts` and `pos_max_soft_counts` (i32,
RW, CONFIG, right behind `pos_min_phys_counts` / `pos_max_phys_counts`)
are an application's travel: a servo built into a joint that moves less
than the servo narrows its travel with them. Every reader of the travel
holds to them: the goal clamp, the endstop wall, the Velocity wall ramp
and the burst arm. By default they are off, and 0 / 0 means off: the
physical stops `pos_min_phys_counts` / `pos_max_phys_counts` then stand
in their place everywhere, the same wall at the hard stops, and the
stall permit lifts it as it lifts a soft wall (unit
`off_soft_limits_wall_at_the_physical_stops`,
`off_soft_limits_clamp_goals_to_the_physical_stops`). Any other pair
must lie inside the physical stops, min under max, within +/-8192;
one side 0 with the other nonzero is such a pair, judged by those rules.
Each side is judged against the other as stored, so a host writes both
in one WRITE, which is the only way to reach 0 / 0 from limits that are
on. A write to the physical stops reaches the kernel as a soft limit
write does, so off limits follow the stops at once.

**Stall permit.** `stall_permit` (bool, RW, `0x181` in CONTROL) lets
the motor stall on purpose, which calibration needs to seat a hard
stop and identification's stop routes, when asked for, need to measure
the winding. It drops the detectors (the press timer and the collision
check) and the endstop, and nothing else: the peak cap, the rail fold
and `goal_current` still compose. The byte is the
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
  (`osc-ident`'s stop finder does it every 250 ms).
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
500 ms `stall_time_ticks` and the boot Fault response, latches `stall`
between 1.5 and 1.6 s after the write (DES
`dead_host_stall_trips_within_the_lease`). A TEL burst is a silent host
too (sec 5.6).

**Bus dead-man.** `bus_deadman_ticks` (u16, RW, `0x192` in CONTROL) is
the host-loss guard, opt-in, in kernel ticks (20 kHz, so 20000 is one
second and 65535 about 3.3 s), on the production and the experiment
image alike (osc-servo-kernel sec 2.7). A nonzero value with
`torque_enable` 1 is a countdown: every instruction the bus dispatches
to this servo (a read, a write, a ping, anything addressed to it)
reloads it, and a streaming TEL burst holds it, since the host is
silent by design while the burst runs. At zero the kernel latches
`deadman` (bit 7, code 9, the table above) and the drive coasts until
the `torque_enable` 0 to 1 ack. Torque off clears the countdown; a
fresh enable starts it from the field. It counts once a medium period
(10 ticks, 0.5 ms): on the production image the fault latches no
sooner than the field rounded up to a whole period after the last
instruction, and at most one period later (osc-servo-kernel sec 2.7).
0 is off and the boot value:
CONTROL is RAM, so a reboot clears it. `osc-ident`'s drives run under
50 ms when the host times the drive and 100 ms, five poll periods, when
the host polls it, so a host that dies mid-drive leaves a servo that
coasts within a tenth of a second.

**The experiment image.** `capability_flags` bit 1 (sec 5.4) names a
servo built with the `experiment` feature (osc-servo-kernel sec 2.7):
the image the notebooks capture on, where the host owns safety. It
runs OpenLoop alone at the goal as written, with the overcurrent trip
and the bus dead-man its only protections; the `mode` byte and the
stall permit are accepted and ignored, and the duty limiter, the
limits fold, the closed loops, the estimators and the detectors are
compiled out. A servo on it answers every register; the stripped ones
(the estimates, `limit_flags`, the thermal fields) read zero.

**Speed ceiling and move speed.** Two CONFIG speeds bound the closed
loops' motion, both u16 in counts per second of the linearized count.
`speed_ceiling_cps` (RW, `0x044`, at most 32767, under the plant stamp,
sec 5.7) is the hard speed limit calibration writes; 0 is unset and
refuses every closed mode (sec 5.7, limits unset). `velocity_limit_cps`
(RW, `0x046`, not covered) is the move speed, the cruise of the Position
profile: a user setting from 0 to the ceiling. A write of a move speed
over the ceiling is refused, the table's convention for a field bounded
by another (`goal_current` against the cap). The ceiling sits ahead of
the move speed, so an install in table order lands it before the rule
that reads it, and one WRITE over both is judged against the incoming
ceiling. A later write that lowers the ceiling under a stored move speed
is accepted, and the servo runs the move speed at the ceiling. The
kernel resolves both at the configuration take (osc-servo-kernel sec
6.2): a move speed of 0 reads as the ceiling, as Dynamixel's Profile
Velocity 0 reads as the Velocity Limit, and the move speed runs at most
the ceiling. A ceiling of 0 is not a speed: the servo refuses every
closed mode until one is written.

In Position mode the profile cruises at the move speed and the position
loop's output clamps at the ceiling, so a shaft that falls behind its
profile is asked for up to the ceiling to catch up. In Velocity mode
`goal_velocity` clamps at the ceiling and the move speed bounds nothing.
In both the ceiling bounds the reference the velocity loop follows, not
the shaft: the loop's transient overshoot can carry the shaft past it,
and the derate a calibration takes under the datasheet speed is that
margin (DES `velocity_mode_clamps_its_goal_at_the_ceiling`,
`a_zero_move_speed_cruises_at_the_ceiling`,
`a_held_shaft_is_asked_over_the_move_speed_up_to_the_ceiling`).

Current mode reads the ceiling as a fold of its current band: the side
along the motion is at most `(speed_ceiling_cps - |omega_hat_cps|) x
v_kp_q88 / 256` counts, the velocity loop's own gain on the speed short
of the ceiling, and 0 at and over it; the other side keeps the band.
Near the ceiling `goal_current` is cut back, the shaft settles a little
under the ceiling where friction meets the fold, and over the ceiling
(an overhauling load) the servo applies no torque along the motion and
never brakes. `i_lim_counts` stays the composed limit and `limit_flags`
bit 4 names the fold. The fold rides the speed gain: with `v_kp_q88` 0
it reads 0 at every speed and Current mode delivers no current, so a
servo with no speed gain moves in no closed mode (DES
`current_mode_folds_under_the_speed_ceiling`). OpenLoop reads neither
field.

**What governs.** `limit_flags` (u8, RO, `0x27C` in TELEMETRY; `0x27D`
is reserved) names what shaped the command at the last medium tick,
one bit per reason; 0 means nothing held it back and no permit is
live. It is published
every medium tick (2 kHz), torque on or off.

| bit | name      | set while                                                                                                                                                            |
| --- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0   | `CEILING` | the command sat at the current ceiling: in OpenLoop the duty ceiling held under the goal by the current, in the closed loops the current reference at `i_lim_counts` |
| 1   | `YIELD`   | the press or collision verdict folded the limit to `i_cont_counts`                                                                                                   |
| 2   | `ENDSTOP` | a soft limit, or a physical stop while they are off, closed a side of the band: at or past a wall, current toward it is forbidden                                    |
| 3   | `PERMIT`  | the stall permit lease is live                                                                                                                                       |
| 4   | `SPEED`   | Current mode only: the speed fold sits under the composed limit, so near `speed_ceiling_cps` the side of the band along the motion folds toward 0                    |
| 5   | `UNSET`   | `current_limit_counts` or `speed_ceiling_cps` is 0: every closed mode is refused (sec 5.7, limits unset)                                                             |
| 6   | `RAIL`    | the rail fold binds before the press fold, under the cap                                                                                                                       |
| 7   | `GOAL`    | `goal_current` binds before the press fold, under the cap                                                                                                                      |

Bits 6 and 7 name the ceiling that binds before the press fold: neither
set means the peak cap does, and a tie sets both. The press trigger is the held current over `i_cont_counts` on
a still shaft, not bit 0, so bit 0 alone on a still shaft under
`i_cont_counts` is a hold the loop is content with. While bit 1 is
set in Position the position error timer does not count (sec 5.7): a
load over `i_cont_counts` sags the shaft to where the fold meets it,
past `pos_error_counts` if need be, with the drive on and the bit set,
and the shaft climbs back when the load lightens. Under Fault the
verdict latches `stall` instead (DES
`a_hold_over_the_rating_latches_stall_under_fault`). The permit drops
the fold and the endstop, so bit 3 never appears with bits 1 or 2, and
it drops no ceiling, so it does appear with bits 6 and 7 (unit
`limit_flags_name_the_governor`). Bit 4 is never set in Position,
Velocity or OpenLoop, where the ceiling bounds a reference or nothing.
Bit 5 comes from the configuration the kernel holds, not from the
command: it is set from the medium tick after a write leaves a limit at
0 until the one after both are written again, in every mode and with
torque off.
The flags are a polled register; a TEL row may carry them, as any table
field, at medium rate.

**Window floor.** `window_floor_q15` (u16, RO, `0x27E` in TELEMETRY,
a Q15 duty magnitude) is the smallest duty whose drive window the shunt
reads: CALIB `i_window_min_ticks` turned into a duty against the board's
PWM period, 1734 (5.3%) for 64 ticks of 1200 on osc-dev-v006. It is
the value the OpenLoop limiter uses, where its ceiling restarts and its
blind band (below) begins, published once, at the kernel's first tick,
torque on or off: `i_window_min_ticks` is read-only, so the floor never
changes after boot (unit `floor_inputs_are_read_only`).
0 means the servo publishes no floor: the kernel has not ticked yet, or
the firmware does not carry the field. A host planning a drive against
the current sensor reads the floor here rather than deriving it from a
board constant (DES `window_floor_is_published_as_the_limiter_uses_it`).

The terminal taps have a floor of their own: `window_v_floor_q15` (u16,
RO, `0x3CC` in TELEMETRY-EXT, Q15) is CALIB `v_window_min_ticks` turned into
a duty the same way, 2526 (7.7%) for 93 ticks on osc-dev-v006, where
the low-side tap, sampled 21 ticks before the crest, has finished its
rise, in either direction (the board README holds the instants and the
ladder the floor rests on). It is published at the
kernel's first tick and again within a medium period (0.5 ms) of a
CONFIG write that reaches the kernel. A window between the two floors
reads current but no `va - vb`, so the tick block's `vdiff_counts` holds the last
differential read there. Which
floor a host plans against follows what its fit reads: a drive judged
on current alone (the limiter's slew, an overcurrent abort, a current
step) takes `window_floor_q15`; a drive whose fit takes the differential
takes the higher of the two. Firmware
that predates the register reads 0 there, and a host then takes
`window_floor_q15` for both (DES
`window_v_floor_is_published_beside_the_current_floor`).

**Governed windows.** In OpenLoop the applied duty equals the goal only
when nothing governed it, and a host fitting a model to a capture needs
to know which windows those are. The tick block's `duty_ticks` (sec 5.6)
is the applied duty, the command whose window the row measured. A goal
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
one never reaches the goal. Polled, the same test reads `duty_ticks`
against the goal written, once the slew is over; `limit_flags` then says
why.

**Blind band.** Under the window floor (`window_floor_q15`, 5.3% on
osc-dev-v006) the shunt reports nothing and the servo applies
a stall-safe base duty instead of trusting its ceiling:
`min(i_lim x R / Vbus, floor)` from the identified `r_q12` as stored,
the winding at 25 °C (not `r_hat_q12`, below), whose stall current is at
most the limit on a winding at 25 °C or warmer; on a board under 25 °C it
sits over the limit by copper's slope on the difference, 4% at 15 °C,
where the stall current is tens of mA. A servo with no identified resistance
(`r_q12` 0) has the floor as its base, so the lowest duty it applies to
a goal above the floor is the floor itself, and stall current in the
blind band is bounded by `floor x Vbus / R` of the actual winding:
0.085 A for a 4.9 ohm winding on 7.9 V, and under 500 mA for any
winding above 0.89 ohm on 8.4 V. If the floor
already draws more than the limit, the applied duty stays pinned at
the floor and the stall timer decides (DES
`virgin_blind_band_passes_to_the_window_floor`).

**Winding resistance.** CALIB `r_q12` (u16, Q4.12 vcounts per current
count) is the winding's resistance at 25.00 °C, never the value as
measured. The field's definition is copper's law about that point, with
T in centi-°C:

```
r(T) = r_q12 x (1 + ALPHA x (T - 2500)),   ALPHA = 2637 / 2^26 (0.393% per °C)
```

A host converts a measurement with the board NTC's reading at the time
by the same constant, `r_q12 = r_meas / (1 + ALPHA x (t_ntc_cc - 2500))`,
so the reference temperature has no field of its own. `r_hat_q12` (u16,
RO, `0x26A` in TELEMETRY, the units of `r_q12`) is the winding
resistance the servo's control path uses: the law at the winding
estimate's temperature, `t_ntc_cc` plus the estimate's rise over it
(bounded at 150 °C, as the heat input's feedback is), saturating at
65535. It reads under `r_q12` on a board below 25 °C and over it above.
It is the heat model's number, not a measurement. The back-EMF speed
estimate and the current loop under the window floor read it; the blind
band's base duty above does not. It reads `r_q12` itself while no NTC
curve is set and on a failed NTC (`therm_flags`), and 0 while `r_q12` is
0. The servo publishes it at boot, every 16 ms with the estimate and
within a medium period (0.5 ms) of a CONFIG or CALIB write that reaches
the kernel (control-theory "The Winding Estimate"; pins
`r_hat_reads_the_25_c_resistance_at_the_estimates_temperature`,
`r_hat_follows_copper_on_25_c_for_every_resistance_and_temperature`).

**Brush resistance.** CALIB `r_brush_q12` (u16, RW, `0x08C`, the units
of `r_q12`) is the spinning brushes' resistance to a change in current
beyond the winding's, as the back-EMF speed estimate reads it, and
`brush_cps` (u16, RW, `0x08E`, counts per second of the linearized
count, the unit of `omega_hat_cps`) is the speed it was identified at.
Both sit behind `ke_vpc_q` in the motor block, both are under the plant
stamp (sec 5.7), and both read 0 at the seed and after FACTORY. The
estimate subtracts `r_brush_q12` times the current's swing about its
slow average, the swing bounded at 40 mA, weighted by the speed: 0 at
`brush_cps - 1024` and under, full from `brush_cps` up, linear between
(control-theory "Back-EMF, a Velocity Sensor for Free"; osc-servo-kernel
sec 2.2). The field is not raised along copper's slope: the brushes are
not copper, and `r_hat_q12` has no counterpart for it.

A write of `r_brush_q12` over the stored `r_q12` is refused, the
table's convention for a field bounded by another; `r_q12` sits ahead
of it, so one WRITE over both is judged against the incoming `r_q12`,
and with no winding stored (`r_q12` 0) only 0 is accepted (unit
`a_brush_resistance_over_the_winding_is_refused`). A later write that
lowers `r_q12` under a stored brush resistance is accepted. The kernel resolves
the pair at the configuration take (osc-servo-kernel sec 6.2): the size
runs at most half the stored `r_q12`, and a `brush_cps` under 1024 turns
the term off. A size of 0, the seed, is the estimate without the term,
bit for bit.

**Window conductance.** CALIB `g_window_q016` (u16, RW, `0x090`, counts
of current per count of the terminal taps, Q0.16, sec 5.5) is the
motor's loss conductance across its inductance, the iron's eddy loss.
The shunt's drive-window sample reads it times the volts the window
leaves across the inductance on top of the winding's current, `g x (1 -
D) x V_bus` once the current settles, along the drive, and averaging to
nothing over a period, so a meter in series never sees it. The servo
takes that term off every reading at the source (osc-servo-kernel sec
2.2), on the rail estimate and the measured width: the loops, the
limits and detectors, the winding estimate's heat, the observer, the
back-EMF speed and TEL `i_tick_counts` read the winding's current, so
every stored current limit means winding current. The overcurrent trip
compares the current before the subtraction, the one the bridge and the
shunt carry, and `current_raw` stays the raw sample. The field sits
behind `brush_cps` in the motor block, is under the plant stamp (sec
5.7), and reads 0 at the seed and after FACTORY: 0 is the reading
without the term, bit for bit. A host identifies it from the servo's
own OpenLoop starts from rest, free horn, where the winding is empty and
the first readings are the rise plus the term; nothing the servo stores
predicts it. The kernel runs it only while the stored winding bounds it,
`0 < g_window_q016 x r_q12 <= 2^25` (g R at most 1/8), and turns it off
otherwise, the protective side: the readings run high and the limits
err low. A value past that bound is not refused at the write, as the
table's write rules compare a field with a bound or one other field,
never a product (unit `the_take_fences_the_conductance_by_the_winding`).

**Shunt burst.** The high-rate shunt capture (`arm` at `0x198` in
CONTROL, with `duty_q15` at `0x196` and `chans` at `0x19A`; read back
from the BURST section) steps the bridge to `duty_q15` halfway through
a window of about 1.04 ms in which the kernel does not tick: no
limiter, stall timer or endstop acts on the step. The step drives for
the half window, about 0.52 ms, and the first kernel tick after the
window replaces it. `duty_q15` is logical, like an OpenLoop duty: under
`drive_polarity` 0 the servo negates it at the bridge, so a step drives
the same way as the hold before it, and it drives the A terminal high
when its sign is positive under `drive_polarity` 1 or negative under 0
(unit `burst_step_follows_the_drive_polarity`). What bounds a burst is
its arm. `arm` 1 is refused
unless all of these hold at the kernel tick that sees it:

- torque on, mode OpenLoop, no TEL burst armed or streaming
  (`tel_count` 0: the servo zeroes it when a burst ends, sec 5.6), no
  fault latched, and `chans` setting only defined bits;
- the applied volts are at most 3.2 V: `(|duty_q15| x vbus_counts) >>
  15` at most the board's cap in vcounts, 794 on osc-dev-v006. That
  is a step of at most 13284 (40.5%) on a 7.9 V rail (`vbus_counts`
  1961) and 23899 (72.9%) on a 4.39 V USB rail (1090);
- at least 100 ms have passed since the previous accepted arm: 2000
  kernel ticks, and the kernel does not tick during a capture, so the
  gap in time is never shorter;
- the raw `pos` lies strictly between `pos_min_soft_counts` and
  `pos_max_soft_counts` (the physical stops while those are 0 / 0,
  off), or the stall permit is granted (`limit_flags` bit 3; the
  `stall_permit` request byte alone does not count).

A refused arm drives nothing and publishes BURST `state` (`0x2C1`) 4,
`REJECTED` (0 idle, 1 armed, 2 capturing, 3 done); it stays there until
the host writes `arm` 0, and it starts no spacing. The most a host can
cause, dead or buggy, is one pulse of at most 3.2 V for about half a
millisecond every 100 ms inside the soft limits (control-theory "Open
Loop Under the Same Band"; unit `burst_over_the_volts_cap_is_rejected`,
`burst_inside_the_spacing_is_rejected`,
`burst_outside_the_soft_limits_needs_the_permit`,
`burst_with_the_soft_limits_off_arms_inside_the_physical_stops`,
`burst_at_the_cap_still_arms`).

The capture is 960 raw codes, one conversion every 1.083 us, frame
after frame. `chans` bit 0 adds the A terminal tap, bit 1 the B tap and
bit 2 the rail tap, in that order behind the shunt in slot 0; bit 3,
`INTERLEAVE`, puts another shunt slot ahead of every extra after the
first. Mask 3 is shunt, A, B: `frame_len` 3, the shunt every 3.25 us.
Mask 11 is shunt, A, shunt, B: `frame_len` 4, the shunt every 2.17 us and
each tap every 4.33 us. The BURST header echoes `chans_echo` and
`frame_len` for the whole capture, and a host reads the layout from them;
`step_index` counts raw codes, not frames (unit
`burst_frame_follows_the_mask`).

**Around a drive.** What `osc-ident`'s drives (the `measure` crate) do
with these registers, the reference for any host that drives OpenLoop
(control-theory "Open Loop Under the Same Band" has the planning behind
it):

- Every rung is priced before anything of it is written, for a free
  shaft, a blocked one and, until the sign is proven, the wrong sign,
  against the current (a press: the gear class's stop-hold ceiling, 300
  mA plastic or unknown, 500 mA metal; a free drive: the nameplate's
  stall current), the duty, the time (the burst counter, and at most
  0.7 s while the permit is held), the travel and the temperature. A
  rung that does not fit is refused in plain words.
- Each drive runs under the bus dead-man: 50 ms for a drive the host
  times, 100 ms for one it polls every 20 ms.
- Only a drive that presses on purpose holds `stall_permit` 1: the
  polarity step's unwedge press, the stop approach, a seated burst at or
  past a soft limit (where the firmware arms a burst only under a
  granted permit) and the table's stop rung. The stop finder, the one
  drive that holds it past the lease, rewrites it every 250 ms.
- Every exit of a step - done, refused, a bus error or the interrupt -
  writes zero duty, `torque_enable` 0 and reads it back, `stall_permit`
  0, `bus_deadman_ticks` 0 and `tel_count` 0, then puts back the
  volatile fields the run changed, or reboots the servo onto its saved
  state where one does not go back. With the bus gone the dead-man has
  already dropped torque, so it is left armed until torque reads off.
- A shunt burst arms, then polls BURST `state`: 3 `DONE` walks the eight
  pages, 4 `REJECTED` is a refused arm. `REJECTED` does not say which
  rule refused it, so the host treats it as a failure, and a rejected or
  wedged arm is released before the error goes up, so it never holds
  the burst machine.

### 5.9 Health (osc-servo)

Six u16 registers at the front of TELEMETRY-EXT (`0x3C0..0x400`, the
model's overflow of TELEMETRY) say how the servo itself is keeping up.
Every build carries them; the chip side writes them, not the kernel.

| addr  | name                 | access | meaning                                                                                              |
| ----- | -------------------- | ------ | ---------------------------------------------------------------------------------------------------- |
| 0x3C0 | `tick_load_mean_q15` | RO     | mean share of the kernel period the tick interrupt took over the last 4000 ticks (0.2 s), Q15       |
| 0x3C2 | `tick_over_count`    | RW     | kernel ticks whose interrupt took longer than one period; wraps                                     |
| 0x3C4 | `tick_lost_count`    | RW     | kernel ticks that never ran: the previous tick was still running, or a flash SAVE stalled the core; wraps |
| 0x3C6 | `tel_drop_count`     | RW     | TEL rows dropped because both stream buffers were waiting for the wire, each a jump in the next frame's `tick` (sec 5.6); wraps |
| 0x3C8 | `write_late_count`   | RW     | kernel ticks whose motor write landed after the trough: the width loaded at the next crest and split that pulse between two commands, and the window it drove read no current; wraps |
| 0x3CA | `stack_free_min`     | RO     | smallest free stack seen since boot, bytes                                                           |

The RW registers follow the TELEMETRY-COMMON clear contract (sec 5.4):
a host writes 0 and the servo counts on from there. The mean load
reads as a percentage of the period (50 us at 20 kHz) as
`q15 x 100 / 32768`, 16384 is 50%, and covers 0.2 s so a single read
does not catch one medium tick. Each tick is timed from the first to
the last statement of the tick interrupt and leaves out interrupt entry
and exit. The tick interrupt sits above the transport, so bus traffic
costs it no tick: it is the kernel alone. A tick counts as lost when the
interrupt runs a whole period or more behind schedule, judged once per
20 ticks: a late window that the next one catches up costs nothing, and
one window counts at most 16. Only a kernel body that overruns past two
periods or a flash SAVE (torque off) loses one, so any count under
traffic is the kernel's own. The kernel does not stretch for a lost
tick: every tick it runs advances one period, and `sample_tick` stays
the count of ticks that ran. Both counters update every 20 ticks
(1 ms), the mean every 4000. No kernel tick runs during a shunt burst (sec 5.8); the ticks before it
that did not fill a 20-tick window count toward neither counter, and
the first window after it counts no lost ticks.
`stack_free_min` reads 0 until the first stack scan completes, about
10 ms after boot.

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
  body, only the framing-level end. The end comes from LEN, so a uniform
  GREAD's slot checks it: a predecessor status is either the span the
  GREAD read or an empty error status (sec 5.3), and any other LEN is a
  dropped frame that does not count, leaving the slot to the reclaim
  below. A corrupt status otherwise mis-times one slot at worst, bounded
  by the reclaim window. A servo no slot of which waits on the status
  checks its CRC like any frame's, since its LEN still moves the framer.
- **Reclaim deadline** (DXL chains collapse silently past a dead
  responder; here the recovery is specified): if slot k's predecessor
  produces no break within RESPONSE_DEADLINE of the later of its due
  trigger and the moment slot k is ready (has dispatched the GREAD, or
  has seen the predecessor's predecessor end), slot k takes the slot and
  sets `predecessor-silent` in its status error field. The host sees
  both the gap and the flag. Counting from readiness matters because
  every servo works through the same backlog: absolute lateness can
  exceed any fixed window while the lateness between neighbours stays
  small. The window covers the trigger-to-break lead only - once the
  predecessor's break is observed it is alive, and the window suspends
  for the largest frame's wire time plus RESPONSE_DEADLINE (which covers
  the pauses inside a reply, sec 4.2) while its frame plays out
  (completion re-sequences the chain; a frame that garbles or wedges
  lets the suspended deadline fire as the reclaim). Keying on the break
  rather than the frame end is what keeps the window baud-independent:
  a frame's wire time grows as the baud falls, its break lead does not.
- Error statuses keep the chain alive; only silence triggers reclaim.

There is no FAST/regular split and no per-block checkpoint CRC: each chain
element is a complete, independently CRC'd status frame, and the break
delimiter gives every snooper hardware resync per element — the problem the
DXL checkpoint format solved does not exist here.

## 7. Timing rules

| parameter               | value                                      | rationale                                                                                  |
| ----------------------- | ------------------------------------------ | ------------------------------------------------------------------------------------------ |
| reply gap               | 12 µs after frame end, at every baud       | host TC→release margin — a register poke, i.e. a time-domain quantity: fixed µs neither balloons at 0.5 M (2 byte-times was 40 µs of mandated silence) nor thins at 3 M |
| RESPONSE_DEADLINE       | config register, default 1000 µs (all bauds) | chain reclaim (trigger-to-break lead, sec 6) + host timeout; covers the servo's turnaround and the idle gaps inside its reply under traffic up to the published capacity; NOT a reply-time prescription - a servo replies when ready |
| break length (TX)       | exactly 10 bit-times (§3 law; 9-bit 0x00 character) | break ≡ 1 character: exact span algebra + LIN-detectable; SBK (~14 bits, F5) is off-law |
| inter-frame gap (host)  | none required                              | breaks self-delimit; back-to-back host frames are legal                                    |

Turnaround (instruction wire-end to status break fall) is the
transport's CPU cost stretched by the control kernel, which runs above
the bus: each us of bus work costs 1 / (1 - U) us of wall time, U being
the kernel's share of the CPU (about 36% at rest, 67% while moving).
Measured with the motor at rest (torque off), ping p50 is about 70 us at
1 M and 77 us at 3 M (p99 99 us), a 32 B READ 81 us and a goal WRITE
166 us at 3 M. The instruction's own bytes cost nothing on top, since
dispatch overlaps their arrival (speculation; see
`osc-servo-transport.md`, dispatch speculation). Turnaround is a derived
quantity, checked against RESPONSE_DEADLINE, not a budget of its own;
see `osc-servo-transport.md` (tick-by-tick exchange trace) for the
pipeline.

The intended hot loop leans on writes being free of turnaround entirely:
`GWRITE(HOLD|NOREPLY) × groups → COMMIT (broadcast, silent) → GREAD
telemetry chain` — writes cost pure wire time (back-to-back frames are
legal), the apply instant is one broadcast, and the telemetry chain is
the implicit ack: a rejected write surfaces within one cycle as the
ALERT bit on that servo's status (§5.3).

## 8. Host requirements

- Crystal-clocked UART with break send (the osc-adapter, or any
  USB-serial with SBK).
- A break detector that places every received break in the RX stream: a
  frame starts only at a break, and a claimed frame that spans a later
  break is junk. The osc adapter reads its DMA ring count in the LIN break
  detector's interrupt, which preempts everything else so it lands within
  a byte-time of the break [F18].
- osc-CRC (textbook CRC-16/ARC, §3.2).
- The bus pull-up, at the host end: servos only pull low (sec 2).
- Drive discipline if on a buffer-less bus (release when idle) [F8].
- Schedule the bus: one outstanding instruction / chain at a time;
  timeout = RESPONSE_DEADLINE + frame time, plus per chain slot the
  larger of RESPONSE_DEADLINE and the slots' readiness lag (a silent
  slot's successor reclaims one RESPONSE_DEADLINE after it is ready, sec
  6, and readiness trails the GREAD by the bus backlog every servo works
  through, stretched by its kernel: osc-host allows 5 ms,
  `CHAIN_READY_LAG_US`, sized from the V006 budget table at its moving
  kernel load).
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

A UART has no bit arbitration (a servo does not hear its own TX, F9), so
simultaneous responses are garbage - and garbage _is_ the collision signal:

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
crystal (any timer/DMA pacing) keeping the spacing. Each servo times every
break by a stamp its hardware latches at the break detector, never by the
wake's service: every stamp sits the same offset into its break, so the
offset cancels in the difference, and what remains is clock skew plus the
latch's jitter. Service lag, however long, never enters. Contract and
hygiene:

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

CAL is the only input to the trim: frame traffic never trims, so nothing
the host sends between trains (any shape, any rate, any seam) moves a
servo's clock. Each train feeds the oscillator-trim loop
(`steps = round(err/step_effect)`, clamped +/-4 per train), applied by the
main loop between frames; the total is readable at
`telemetry.clock.trim_steps`. The step effect is self-measured - chip trim
steps are nonuniform, 1.4-3.2 k ppm/step measured - from one CAL's reading
to the next, over the steps the earlier train applied. Volatile by design.

The host owns the schedule (~4 ms of bus per train): it CALs when it
detects a servo (boot, a servo joining the bus), then periodically to
follow thermal drift (bench fleet offsets grew over a ~1.5 h warm-up);
the interval is the host's choice. A rate change needs no CAL: the trim corrects the
oscillator's frequency, which no baud setting moves.

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
`C`, version 6) is the CONFIG region ++ the PROFILE region, 200 B in one
256 B page per slot. The CALIB image (magic `K`, version 6) is the
CALIB region ++ the position table's 256 host-written calibration
points i16 LE (the fixed last one is not stored): 776 B, four pages per slot, one CRC, so the
calibration and the table it validates save and load as one unit and a
save cannot tear between them; tables to come join this image (a layout
change bumps its version). A CRC-valid image of another version boots
*stale*, never migrated (sec 5.7): a CALIB 5 image, whose blocks behind
the motor block sit 4 B earlier, boots stale and the seeds stand (unit
`calib_images_before_the_brush_term_are_stale`). On the osc-servo the two CALIB slots
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

`MGMT REBOOT`, `MGMT FACTORY` - conventional semantics (as in DXL).
REBOOT names where the servo goes, with no default:

| args              | effect                                           |
| ----------------- | ------------------------------------------------ |
| `0`               | reset into the app                               |
| `1 baud_idx`      | reset into the bootloader at that rate (sec 9.6) |

`baud_idx` uses the `baud_rate_idx` encoding (sec 2). Any other args nack
`range` and the servo stays up. The ack drains at the old rate before the
reset. FACTORY takes no args and always comes back as the app. FACTORY erases every slot
of both saved images (config and calib with its tables, sec 9.4), not
just the live table, then stages the reboot: the erased store is the
factory state, and the servo comes back *virgin* (sec 5.7) on board
defaults, the position table at the identity. It shares SAVE's torque
gate, and a failed wipe nacks `hardware` without rebooting. It is the only
exit from `CONFIG_CORRUPT`.

### 9.6 Firmware update

The servo's bootloader is [tinyboot](https://github.com/OpenServoCore/tinyboot),
built addressable: it holds no id and stays silent until a host selects it
by the chip's unique id. App servos never frame its traffic, and nothing
else uses the bus while it runs.

- Entry: `MGMT REBOOT 1 baud_idx` (sec 9.5). The host always names the
  bootloader's rate, so it can pick a slower one where the wiring is
  unknown, as in a field update.
- Rate: the bootloader runs at the baud stored in its metadata page. The
  REBOOT stores the named rate there first, a flash write only when it
  changed. A servo that falls back to its bootloader on its own, after a
  power cut mid-update or an image that never confirms, comes up at the
  rate the last update named. A board with only the bootloader comes up at
  0.5 M.
- Exit: the host resets the bootloader into the app once the image is
  written and verified. A new image boots on trial: its first bringup
  confirms it, and one that never confirms falls back to the bootloader
  once its trials run out.
- Compatibility: the boot and app images carry the board's compat key
  (`osc-dev-v006`), and the bootloader refuses an image built for another
  board.

## 10. V006 resource map

| resource            | use                                               |
| ------------------- | ------------------------------------------------- |
| USART1 + HDSEL, PC0 | the bus                                           |
| DMA1 CH5            | RX ring (circular, armed once)                    |
| DMA1 CH4            | TX stream (enable-when-ready)                     |
| DMA1 CH3 + SPI1     | CRC engine (no pins) [F6]                         |
| DMA1 CH1            | ADC                                               |
| DMA1 CH7            | TIM2_CH2: zeroes the break detector per rising edge (§3.4) |
| DMA1 CH6            | copy-once snapshot buffer (sec 4.2)               |
| DMA1 CH2, TIM3      | CAL break stamps: TIM2_UP latches TIM3's count (sec 9.3) |
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
| F8  | idle push-pull clamps other talkers; open-drain idle is mandatory                          | bringup measurement, V006 |
| F9  | V006 HDSEL has no own-TX echo                                                              | bringup measurement, V006 |
| F10 | full HSITRIM throw −3.0..+3.4 %: framing AND data survive everywhere                       | HSITRIM sweep, V006   |
| F11 | production table CRC = 635 ns/B pure CPU                                                   | bringup measurement, V006 |
| F12 | DMA rounds odd MAR down; no unaligned 16-bit reads                                         | bringup measurement, V006 |
| F13 | EXTI edge ISRs storm during traffic (own TX stretched 3×)                                  | HSITRIM sweep, V006   |
| F14 | data decodes clean at ±3.4 % in both TX and RX directions                                  | HSITRIM sweep, V006   |
| F15 | LBD runs sans LINEN, both chip families: length-qualified (≥10-bit spans only — 0 fires on framing-error injection and high-baud garble), safe flag-selective write-0 clear mid-traffic, one event per any-length span, **latched at the span's END** (== bit 10 for the 10-bit law break; a rescue pulse's wake arrives after the line rises), entry stamps 4 ticks p-p on a 400 µs grid; latched FE/NE/ORE with no interrupt enabled are harmless through marination. Measured with RX on its own pin (HDSEL=0); see F16 | bringup measurement, V006 + V203 |
| F16 | V006 never sets LBD while HDSEL=1, in any configuration tried (LINEN, LBDL, pin mode, RE alone, arm order); the same die sets it with HDSEL=0 | bringup measurement, V006 |
| F17 | TIM2 break detector on the HDSEL pin (gated to count only while the pin is low, zeroed by DMA on every rising edge, overflow at 9.25 bit-times): 100/100 law breaks at 0.5M/1M/2M/3M at one ISR entry each, zero entries on an idle line, zero false breaks from frame data (`0x00` bytes included) or faster-baud garble, frames intact; a swept low wakes exactly when its gated count passes the reload, monotone at 4-tick resolution; a 1 ms low is one entry, a 5 ms low one wake plus one silent re-fire per 65536 ticks | bringup measurement, V006 |
| F18 | V305 USART3 full duplex (RX on its own pin, tied to the wire): LBD sans LINEN fires 1:1 on the law break at 0.5M/1M/2M/3M (1000/1000 each, none on data `0x00` bytes); LBDL=1 never fires on it; the DMA ring count at the vector's entry, minus one, is the break's `0x00` while the entry lands within a byte-time (1000/1000 under a 50 µs masked main loop with the vector above it; 77% late, up to 14 bytes, with it masked); with LINEN=1 the first break after boot rings no `0x00`; RE off mutes own bytes and breaks | bringup measurement, V305 |
