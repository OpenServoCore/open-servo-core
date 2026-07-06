# OSC-Native Protocol

Design for the break-framed servo bus protocol that replaces DXL 2.0 on the
host↔servo wire when we control both ends. Every physical-layer behavior this
design leans on was measured on real silicon (V006 servo + V203 HSE host,
2026-07-05 bringup spikes `break_framing.rs` / `spi_crc.rs`); the measured
facts are collected in §12 and cited inline as [F1]..[F14].

DXL 2.0 support is frozen, not deleted: the tuned stack remains an alternate
build for ecosystem interop. `osc-core` and the control table are already
protocol-shaped, so both stacks share dispatch and the register file.

## 1. Goals and non-goals

Goals, in priority order:

1. **Simplicity** — the servo-side transport should be a two-state framer, a
   counted DMA ring, and a dispatcher. No per-byte parsing, no byte stuffing,
   no unstuffing pass, no hardware-timed reply grid, no RDT tuning surface.
2. **Turnaround** — length is known two bytes into a frame, so dispatch and
   reply staging overlap the instruction's own wire time; the reply's first
   byte no longer waits for a completed encode (streaming TX) or a folded
   CRC (hardware CRC engine).
3. **Cheap-MCU fit** — everything must run on the V006 tier: one UART, one
   DMA ring, the SPI block as a CRC engine, no input capture, no crystal.
4. **Recoverability in the field** — a servo must be reachable regardless of
   its configured baud or ID (rescue break, UID enumeration).

Non-goals: DXL wire compatibility (frozen build covers it); multi-host
arbitration (single host schedules the bus); encryption/auth.

## 2. Physical layer

Single-wire half-duplex TTL bus, 3.3 V, host-scheduled (exactly one talker
at any time by protocol construction).

- **Servo pin**: the USART TX pin with `HDSEL` (single-wire mode). RX is
  internally tied to the pin; the dedicated RX pin and the direction buffer
  (74LVC2G241 + TX_EN GPIO) are deleted [F7]. Bus side: series R + pull-up
  (+ optional TVS); the buffer's roles collapse into the drive discipline
  below.
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

A frame is delimited by a **UART break** — ≥10 bit-times dominant, sent with
`SBK`, unforgeable by data (every UART byte contains a high stop bit within
10 bit-times). No sync header, no byte stuffing, no content restrictions.

Measured break behavior that the framer relies on:

- FE raises exactly one ERR interrupt per break, 1:1 at 3 M from an HSE
  host, and the break rings as exactly one `0x00` byte via DMA [F1][F2].
- A break of _any_ length is exactly one event — the receiver re-arms only
  on a fresh falling edge, so long breaks cannot spam [F3].
- A mid-frame framing error does not halt reception: the garbled byte rings
  and the stream continues; consecutive FE bytes coalesce into one IRQ, so
  the FE IRQ is an event marker, never a counter. Ring + NDTR are the only
  ground truth [F4].
- This silicon sends ~14-bit breaks (4.7 µs at 3 M, zero variance, both
  chip families) [F5]; the spec requires only ≥10 bit-times.

### 3.1 Frame anatomy

Both directions use one shape (symmetric framing keeps the framer identical
for hosts, servos, and chain-snooping peers):

```
BREAK | ID | LEN | INST | payload[0..p] | (PAD) | CRC_lo | CRC_hi
```

- `ID` — 1 byte. `0x01..0xF9` unicast, `0xFE` broadcast, `0x00`/`0xFF`
  invalid (never valid on the wire: `0x00` is the break's ring byte, `0xFF`
  is idle-line noise), `0xFA..0xFD` reserved. In status frames, `ID` is the
  responder's ID.
- `LEN` — u8: count of bytes following it (`INST` + payload + pad + CRC =
  `3 + p + pad`). Frame end is knowable at byte 2:
  `end = len_pos + 1 + LEN`. Max payload is 252 bytes — deliberately
  sized so the largest legal frame (258 ring bytes) fits whole in the RX
  ring, which deletes chunked consumption from the framer (§4.1) and all
  per-transfer capability limits (§5.1). Fleet-scale group ops fit
  comfortably (§5.1); bigger transfers split into frames. A corrupted
  `LEN` cannot wedge the framer: the frame fails CRC at deadline B, and
  any subsequent break re-anchors (FE fires in LOCKED too).
- `INST` — bit 7: `0` = instruction, `1` = status (snoopers classify frames
  without state; there is no status opcode). Instructions: bits [6:4]
  opcode, bits [3:0] flags (§5). Status: bits [6:2] result code, bit 1 =
  PAD, bit 0 = ALERT (§5.3).
- `PAD` — one `0x00` after the payload, present iff the PAD flag is set.
  Rule: **pad iff the payload length is odd** — an invariant, not an
  option. It keeps the CRC-covered span (`4 + p + pad`) and the frame's
  ring footprint (`6 + p + pad`) even, which is what lets halfword CRC
  hardware validate every frame and holds frame anchors at constant
  parity (§3.2). Corollary: the `LEN` value is always odd — an even
  `LEN` is malformed, rejected at the header deadline. (The flag is
  still load-bearing: `LEN` alone cannot distinguish an even payload
  from an odd one plus pad.)
- `CRC` — little-endian osc-CRC-16 (§3.2) over the frame bytes
  `ID .. PAD` behind a fixed one-byte prefix.

### 3.2 osc-CRC-16

The covered bytes are **a fixed `0x00` prefix byte, then the frame bytes
`ID, LEN, INST, payload, PAD?`** — `4 + p + pad` bytes, always even (PAD
rule, §3.1). Over that span:

**CRC-16 poly `0x8005`, init `0x0000`, no reflection, no output XOR,
computed with each 2-byte pair swapped** — equivalently, CRC-16/BUYPASS
fed as little-endian 16-bit halfwords, MSB-first. Hosts implement it as:
prepend `0x00`, swap byte pairs, standard CRC-16/BUYPASS.

The prefix byte is protocol, not silicon: it exists so that on receivers
where a break rings a `0x00` into the buffer (V006, V203, and any UART
that DMAs FE bytes — a break is by definition an all-zero frame), the
covered span is *physically present* as the frame's exact ring footprint
starting at the anchor. Such receivers CRC the ring in place; everyone
else prepends the constant. On TX the frame buffer is built with a
literal `0x00` at offset 0 — the CRC engine DMA reads from offset 0 while
the UART TX DMA reads from offset 1: one buffer, two channel MARs, no
copies.

Rationale for the flavor: it is precisely what the V006 SPI CRC unit
computes when DMA feeds 16-bit frames straight from the byte buffer —
bit-exact, accumulating across split DMA arms (ring wrap), at ~0.36 µs/B
wall and ~zero CPU, vs 635 ns/B of pure CPU for the production table loop
(~25× CPU saving on a 64 B frame) [F6][F11]. Since we own the protocol, we
adopt the flavor the silicon computes for free rather than pay a per-pair
swap on every frame. The 4-byte covered prefix (`0x00, ID, LEN, INST`) is
2 whole halfwords, so payload halfword pairing is position-independent —
multi-arm TX (§4.2) feeds even-addressed table spans to the engine with
no re-packing.

Servo usage: the hardware engine is the **only** CRC implementation on
the servo — status generation (own buffers are even by construction) and
inbound validation both. There is no software CRC fallback, because there
is nothing for it to fall back on:

**Even anchors are an invariant, not an optimization.** Every frame's
ring footprint is even (§3.1); a mid-frame FE still rings its byte, so
garble is parity-neutral [F4]; own TX never rings (no echo [F9]); and the
servo arms its RX ring on a quiet line at boot, so it joins aligned. With
an even ring base, every anchor therefore lands halfword-aligned and the
engine covers every frame. An odd anchor is a *fault* — noise rang a
stray byte, or a transmitter violated framing — and V006 DMA cannot feed
an odd address anyway [F12], so the frame is dropped at layer 1 (§5.3):
no reply, diagnostics counter. Parity is bus-global (every listener hears
the same bytes), so recovery is one bare break (one ring byte) from the
host, flipping the whole bus back at once. Corollary: hosts must not send
bare breaks casually — a resync breather is *two* breaks, parity-neutral.

Test vectors (covered bytes → CRC):

```
00 01 03 10                 → 0x740A  (PING id 1)
00 05 07 30 80 01 2C 01     → 0xC9AE  (WRITE id 5, addr 0x0180, data 2C 01)
00 03 07 20 00 02 08 00     → 0x1970  (READ id 3, addr 0x0200, count 8)
00 02 07 32 00 01 AA 00     → 0x46A8  (WRITE id 2, addr 0x0100, data AA; p=3 → padded)
```

### 3.3 Resync

Any CRC failure, LEN overrun, or framing anomaly drops the frame and
returns the framer to HUNT; the next break is a hardware resync point.
There is no FF-FF-FD-style hunting cold path — the break _is_ the hunt.
A mid-frame FE costs one frame (CRC rejects it), never the stream [F4].

## 4. Servo RX/TX paths

### 4.1 RX framer

One circular RX DMA channel, armed once at boot, never reconfigured; NDTR
is read as a cursor, never reloaded (reloading drops or latches requests —
the #134 class). Two states:

- **HUNT** — on FE IRQ (break): record the anchor = current ring position
  (the `0x00` just ringed). Enter LOCKED.
- **LOCKED** — deadline A at anchor + 4 byte-times: the full header
  (ID, LEN, INST) is in the ring — read it, compute frame end, prime the
  dispatcher from INST, set deadline B at end + margin. At B: confirm
  NDTR reached the end, CRC-check, dispatch. Short/failed → HUNT.

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

Reply buffer: `[0x00][ID][LEN][INST|0x80][payload][PAD?][crc][crc]`
in a halfword-aligned static — the `0x00` at offset 0 is the CRC prefix
(§3.2), and the PAD rule keeps the buffer even, so it is hardware-CRC-able
by construction. Sequence: flip pin to
push-pull → `SBK` → enable UART TX DMA from offset 1 → simultaneously
enable SPI-CRC DMA from offset 0 → the CRC engine outruns the wire 8:1,
so `TCRCR` is patched into the trailing CRC bytes long before the shifter
needs them (fire-first, append-later, no deadline race) [F6]. On TC:
release pin to open-drain.

No hardware-timed kickoff: TX start is "enable the channel when ready" —
the break makes reply timing non-critical, which deletes the TIM-compare
kickoff machinery, the RDT register, and its whole tuning surface (K clip,
TX-start lead calibration).

Large reads are **zero-copy**: the header comes from the
small reply buffer, then the TX DMA is re-armed directly onto the control
table region (UART bytes tolerate arbitrary inter-byte gaps, so the µs-
scale re-arm between DMA arms is invisible framing-wise — nothing in this
protocol times on idle). The CRC engine consumes the same spans: the
4-byte covered prefix is 2 whole halfwords, so even-addressed table spans
arrive halfword-paired with no re-packing (§3.2, §5.2). Reads over 252 B
split into multiple frames (§5.1), each still zero-copy.

## 5. Instruction set

`INST` bit 7 = 0; opcode in bits [6:4], flags in bits [3:0]:

| flag  | name           | meaning                                                                        |
| ----- | -------------- | ------------------------------------------------------------------------------ |
| bit 0 | HOLD / PROFILE | writes: staged, applied by COMMIT · reads: payload names a profile slot (§5.2) |
| bit 1 | PAD            | payload carries one trailing pad byte                                          |
| bit 2 | NOREPLY        | suppress the status frame                                                      |
| bit 3 | PER_TARGET     | group op uses per-target addressing                                            |

| op  | name    | payload                                                              | reply                                 |
| --- | ------- | -------------------------------------------------------------------- | ------------------------------------- |
| 0x0 | invalid | (INST 0x00 never valid, like ID 0x00)                                |                                       |
| 0x1 | PING    | —                                                                    | status: model(2), fw(1), UID(8)       |
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
- The flat control table carries over unchanged (address == offset,
  1024 B); `addr` is 2 bytes, `count` is 2 bytes for reads (kept u16 for
  field alignment in the payload view; values cap at 252, §5.1) and
  1 byte per GWRITE slice.

### 5.1 Size limits

`LEN` is the only size limit — one ceiling, no capability registers:

- Payload caps at 252 B. Fleet-scale group ops fit in one frame: a
  uniform GREAD lists 248 IDs (the whole ID space), a PER_TARGET GREAD
  50 targets, a uniform 4 B-data GWRITE 49 targets.
- Every frame sits whole in the ring until its CRC passes (§4.1), so
  nothing is applied from an unverified frame — no partial-apply, no
  rollback, and no per-transfer staging caps (the `SLICE_MAX` /
  `WRITE_MAX` capability registers of earlier drafts are deleted).
- Larger transfers split into multiple frames; a WRITE+HOLD sequence
  with one COMMIT keeps a multi-frame update atomic. Reads are status
  frames under the same ceiling: a whole-table dump is five READs.

### 5.2 Read profiles (indirect addressing, span-granular)

DXL's byte-granular indirect registers are deliberately not carried over:
a byte remap forces the reply through a per-byte pointer chase, defeating
both zero-copy TX and the hardware CRC. The scattered-telemetry need they
served (position + velocity + current + temperature live in different
table sections) is met span-granularly instead:

- A **profile region** in the flat table: a few slots, each an ordered
  list of `(addr u16, count u16)` spans. Configured once with ordinary
  WRITEs — no new instruction, no hidden state.
- READ/GREAD with the PROFILE flag name a slot instead of addr+count. The
  hot-loop instruction stays minimal; the span list costs wire bytes once
  at setup and zero per cycle (the reason profiles beat inline
  scatter-gather lists for cyclic telemetry).
- Execution is §4.2's existing multi-arm TX: one DMA arm per span straight
  from table storage, CRC engine accumulating across arms [F6] — scattered
  reads remain zero-copy.
- Constraint: **spans must be even-addressed and even-length** so the
  hardware CRC's halfword pairing survives concatenation (an odd span
  shifts the pairing of everything after it). Lone `u8` fields round up to
  2 bytes. A slot's total span length caps at 252 B like any reply (§5.1).
- Scattered _writes_ need no counterpart: `WRITE+HOLD` per span plus one
  `COMMIT` is already atomic — inline scatter-writes would add cross-span
  validation complexity for no capability gain.

Implementation note (zero-copy generally): TX DMA reads the live table
byte-serially, so a field updated mid-span by a control ISR can emit a
torn multi-byte value — same exposure as any unlocked read of the table.
Field-aligned spans and the single-writer discipline bound tearing to one
field; consumers that care re-read.

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
   (addr/count out of bounds), `access` (read-only / torque-locked),
   `validation` (value rejected by field rules), `busy`, `limit`
   (requested reply exceeds the frame ceiling, §5.1), `predecessor-silent`
   (§6),
   `hardware`. Exact numeric assignments live with the implementation.
3. **Device-level** (alarms: overtemperature, overcurrent, encoder fault):
   orthogonal to any one instruction's result, so it takes no result-code
   space — status bit 0 (**ALERT**) is set on *every* status frame while
   the alarm register is nonzero, prompting the host to read it. DXL's
   alert-bit semantics, kept because they're right.

## 6. Coordinated reads (status chains)

GREAD replies arrive as a chain of ordinary status frames, one per listed
servo, in list order. Sequencing is snoop-driven and break-framed, which
makes it cheap and robust:

- Slot 0 replies to the instruction like a unicast read (≥ T_turn after
  instruction end, §7).
- Slot k>0 counts _status_ frames (INST bit 7) on the wire since the
  GREAD; when frame k−1 completes (its end is known at its LEN byte — no
  timing inference), slot k starts after T_turn.
- **Reclaim deadline** (replaces DXL's silent-collapse fragility, spec'd
  here rather than inherited): if slot k's predecessor produces no break
  within RESPONSE_DEADLINE of its own trigger, slot k takes the slot and
  sets `predecessor-silent` in its status error field. The host sees both
  the gap and the flag.
- Error statuses keep the chain alive; only silence triggers reclaim.

There is no FAST/regular split and no per-block checkpoint CRC: each chain
element is a complete, independently CRC'd status frame, and the break
delimiter gives every snooper hardware resync per element — the problem the
DXL checkpoint format solved does not exist here.

## 7. Timing rules

| parameter               | value                                      | rationale                                                                                  |
| ----------------------- | ------------------------------------------ | ------------------------------------------------------------------------------------------ |
| T_turn (min reply lead) | 2 byte-times after frame end               | previous driver's TC→release margin; measured release is a register poke, this is headroom |
| RESPONSE_DEADLINE       | config register, default ~60 µs @3 M       | chain reclaim + host timeout; NOT a reply-time prescription — a servo replies when ready   |
| break length (TX)       | hardware SBK (~14 bit-times measured [F5]) | spec floor is 10; no tuning                                                                |
| inter-frame gap (host)  | none required                              | breaks self-delimit; back-to-back host frames are legal                                    |

Projected single-target turnaround at 3 M (components all measured): ping
instruction 5 B + break ≈ 21.4 µs wire; dispatch overlaps arrival; reply =
T_turn (0.7 µs) + break (4.7 µs) + first byte — the post-frame tail is
CRC-check + dispatch remainder only. Target: comfortably under half of
DXL's 62.8 µs ping round trip; to be measured, not promised.

## 8. Host requirements

- Crystal-clocked UART with break send (any USB-serial with SBK, or the
  pirate).
- osc-CRC (prefix + pair-swap + BUYPASS, §3.2) and the PAD rule (§3.1).
- Drive discipline if on a buffer-less bus (release when idle) [F8].
- Schedule the bus: one outstanding instruction / chain at a time;
  timeout = RESPONSE_DEADLINE + frame time.

## 9. Management plane (MGMT sub-ops)

### 9.1 Rescue break

A dominant low ≥ 300 µs at _any_ configured baud commands: switch the UART
to the 0.5 M rescue rate — volatile only; ID retained, config registers
untouched, nothing persisted. A reboot exits rescue back to the configured
baud. The signal itself is baud-agnostic (raw GPIO low suffices at the
host), so it reaches a servo whose rate is unknown, and it unifies a
mixed-rate bus onto one channel in a single pulse. Detection costs nothing
extra: the FE ISR samples the pin level — a normal break has risen by ISR
entry [F5], a rescue low has not; a SysTick recheck ~100 µs later confirms
the span. No EXTI storm, no edge capture. Recovery flow: rescue break →
talk at 0.5 M → fix the baud register → COMMIT/reboot. Limitation: it
cannot interrupt a servo wedged mid-transmit (RX is muted during own TX
[F9]) — it is config recovery, not a babble killer.

### 9.2 UID enumeration and ID assignment

The V006 has a 64-bit ESIG UID. Push-pull UART has no dominant-bit
arbitration, so simultaneous responses are garbage — and garbage _is_ the
collision signal:

- `MGMT ENUM [prefix_len, prefix…]` (broadcast): servos whose UID matches
  the prefix reply with a full-UID status. Clean reply → unique match;
  garbled/CRC-fail → descend the prefix tree one bit and retry.
  O(bits · N) exchanges, boot-time only.
- `MGMT ASSIGN [uid(8), new_id]` (broadcast): the matching servo takes the
  ID and acks from it. Solves the duplicate-default-ID field pain.

### 9.3 Clock calibration: deliberately absent

There is none, and its absence is load-bearing evidence for the design.
Every consumer of clock discipline is gone or covered:

- Reply timing is event-driven (break-led, when-ready) — nothing is
  scheduled against a clock, so there is no grid for drift to skew.
- Comms integrity has ≥3× margin over the worst possible HSI state,
  measured: the chip cannot be detuned far enough to break framing or
  data at 3 M [F10], and the 1 M default triples that.
- Cross-servo simultaneity is an *event* problem, not a clock problem:
  broadcast COMMIT applies a fleet's held writes in the same instant on
  the shared wire — strictly simpler and lower-jitter than disciplined
  clocks.
- Residual ±1 % scale error (velocity estimates, timeouts, PWM rate) is
  far below what any consumer cares about. If a future feature needs
  better, the host can measure each servo's clock ratio passively from
  its status-frame byte cadence (HSE-stamped) and correct host-side —
  smarts belong in the fat node, not in servo machinery.

Consequences: no CALARM sub-op, no calibration pulse, no drift estimator,
and no EXTI use at all — the data pin needs only the USART.

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
  completion — hosts use a SAVE-specific timeout, the servo is genuinely
  stalled during program.
- Fields individually unsafe to change mid-motion (operating mode) keep
  per-field guards via the table's existing `write_locked_by` rules — the
  blanket section lock is gone.
- A config-dirty bit in telemetry reports modified-since-save.

Side effects: flash wear drops from program-per-write to
program-per-session, and the ~10 ms write stall stops ambushing hosts on
ordinary config writes — it happens exactly once, at a moment the user
chose, with torque provably off.

### 9.5 Reboot / factory

`MGMT REBOOT`, `MGMT FACTORY` — carried over from DXL semantics, payload
details with the implementation. FACTORY implies the saved config page,
not just the live table.

## 10. V006 resource map

| resource            | use                                               |
| ------------------- | ------------------------------------------------- |
| USART1 + HDSEL, PC0 | the bus (PC1, PC2 freed for rev-c)                |
| DMA1 CH5            | RX ring (circular, armed once)                    |
| DMA1 CH4            | TX stream (enable-when-ready)                     |
| DMA1 CH3 + SPI1     | CRC engine (no pins) [F6]                         |
| DMA1 CH1 / CH2      | ADC / free                                        |
| DMA1 CH6, CH7       | freed (kickoff machinery deleted) — motor/encoder |
| SysTick             | framer deadlines A/B, T_turn, reclaim             |
| TIM1/TIM2           | motor control, freed from transport duty          |
| EXTI                | unused — no transport consumer at all (§9.3)      |

Deleted relative to the DXL transport: edge IC (already gone), TIM-compare
TX kickoff, RDT + tuning tools, byte-stuffing encode/unstuff, FF-FF-FD
hunter, fold-CRC machinery, the 74LVC2G241 + TX_EN pin.

## 11. Open items

1. Ring size: 512 B proposed (must exceed the 258 B max frame with lap
   margin); confirm against the RAM budget when the transport band is
   planned.
2. RESPONSE_DEADLINE default and T_turn value: set from bench measurement
   of dispatch-under-arrival, not theory.
3. Status error-code assignment and control-table register moves (RDT
   register retires; RESPONSE_DEADLINE takes a slot).
4. Host-side library (pirate already speaks breaks; needs osc-CRC + frame
   layer).
5. Pirate scheduled-send idle window: it drives push-pull from schedule to
   wire-start by design — acceptable for a scheduling host, documented.
6. DES/sim model + integration tests before firmware (the automation
   ladder), then implementation bands.

## 12. Measured foundation

| #   | fact                                                                                       | source                |
| --- | ------------------------------------------------------------------------------------------ | --------------------- |
| F1  | FE fires 1:1 per break at 3 M, HSE host, 50/50                                             | break_framing phase A |
| F2  | break rings exactly one 0x00 via DMA; NDTR-exact framing                                   | phase A               |
| F3  | any-length break = one event (932 µs low → 1 FE)                                           | phase C               |
| F4  | mid-frame FE: no halt, byte rings, IRQs coalesce                                           | FEINJ matrix          |
| F5  | SBK break ≈ 14 bit-times, both chips, zero variance                                        | phases A/D            |
| F6  | SPI CRC: bit-exact BUYPASS (16-bit BE), accumulates across DMA arms, 0.36 µs/B wall ~0 CPU | spi_crc               |
| F7  | HDSEL direct wire works both directions, buffer deleted                                    | HDSEL runs            |
| F8  | idle push-pull clamps other talkers; OD-idle/PP-talk is mandatory                          | drive-discipline runs |
| F9  | V006 HDSEL has no own-TX echo                                                              | phase D (HDSEL)       |
| F10 | full HSITRIM throw −3.0..+3.4 %: framing AND data survive everywhere                       | trim sweep            |
| F11 | production table CRC = 635 ns/B pure CPU                                                   | spi_crc case 11       |
| F12 | DMA rounds odd MAR down; no unaligned 16-bit reads                                         | spi_crc case 10       |
| F13 | EXTI edge ISRs storm during traffic (own TX stretched 3×)                                  | trim sweep gotcha     |
| F14 | data decodes clean at ±3.4 % in both TX and RX directions                                  | trim sweep            |
