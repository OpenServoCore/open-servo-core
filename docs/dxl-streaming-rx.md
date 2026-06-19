# DXL 2.0 Streaming RX on the CH32V006

## Abstract

A streaming, event-driven RX design for the DXL 2.0 transport on the CH32V006, decoupling protocol parsing from byte-time recovery. The parser consumes the RX DMA ring one byte at a time and emits typed protocol events; the classifier walker advances in lockstep and exposes per-byte ticks at the consumer's point of interest, eliminating the derived byte-time ring used by the prior hardware-timed design. Per-packet anchors are recovered from the DXL header's unique on-wire edge signature, so timing drift cannot cross packet boundaries. Foreign packets — in-chain predecessor packets, foreign single-target instructions and replies — are consumed by a universal byte-skip that advances the RX ring tail past each packet's body without parser walk, CRC fold, or timestamp; the skip is load-bearing for keeping the parser's view of the ring tail synchronized with the wire. Plain (non-Fast) Sync / Bulk Read chain timing — left at fixed slot offsets in the prior design — reduces for slots k > 0 to a single piece of state: the chip records its immediate predecessor's ID at slot-demarcation parsing and arms CCR3 in-handler at that ID's byte-skip exhaust, under whichever of USART1 IDLE, DMA1_CH7 HT, or DMA1_CH7 TC drains the predecessor's last byte. Slot 0 falls outside the chain mechanic — it is mechanically a single-target reply to the chain instruction and is scheduled at `packet_end + RDT`, inheriting the existing single-target reply path unchanged. The chain mechanic (slots k > 0) schedules nothing and so has nothing to cancel; a silent predecessor manifests as a missing reply at the host's transaction layer. No forward arithmetic, no derived timestamps inside the chain. The Fast Last post-fire CRC residue fold reduces to a short NDTR-driven loop over raw bytes, with no classifier involvement. Transport-owned RAM drops from 704 B to ~320 B, and CPU during foreign-packet handling scales linearly with what the chip actually consumes rather than with bus traffic.

**Terminology.** *Chain* and *slot* both appear throughout, each carrying a specific semantic load. *Chain* names the Plain Sync / Bulk Read mode — a multi-reply transaction where slot 0 fires per the standard single-target rule (`packet_end + RDT`) and slots k > 0 fire sequence-driven on their immediate predecessor's packet-end. When "chain mechanic" or "chain fire" appears unqualified, it refers to the sequence behavior of slots k > 0; slot 0 is the bootstrap and inherits the single-target reply path unchanged. *Slot* serves two roles, both standard DXL vocabulary: the position of a servo ID in a request's param list (used for both Plain and Fast contexts — "slot 0", "the chip's slot"), and a timing-driven reply window in the Fast Sync / Bulk Read variant (the "slot grid" of §6). The shorthand: Plain semantics = chain (sequence, slots k > 0) + single-target bootstrap (slot 0); Fast semantics = slot (timing window).

## 1. Background

### 1.1 Plain Sync Read and Plain Bulk Read

A Plain Sync Read or Plain Bulk Read request from the host names `N` target servo IDs. Each addressed servo, in the order listed, emits a complete Status packet:

    Status[k] = header(FF FF FD 00) ID len err params CRC

`len` is the DXL 2.0 on-wire byte count, inclusive of CRC and inclusive of stuffed `0xFD` bytes; each slot is a fully independent Status packet, with no coalesced CRC across slots and no shared header.

### 1.2 Per-slot timing constraints

Slot `k`'s start bit has one hard constraint and one observed-behavior baseline.

The hard constraint is bus non-contention: slot `k` cannot begin its start bit while slot `k−1` is still driving the bus. The DXL 2.0 spec is otherwise silent on inter-slot timing. Return Delay Time (RDT) is defined for single-target replies (Ping, single Read, single Write, RegWrite, Action); slot 0 of a chain reply is mechanically a single-target reply to the chain instruction and observes RDT the same way (fire at `packet_end + RDT`). Slots k > 0 fire on the immediate predecessor's packet-end and ignore RDT — RDT does not enter the chain fire rule for k > 0.

The observed baseline comes from Bestmann et al. [1], who measured Robotis MX-64 servos with a Saleae logic analyzer across multiple baud rates. Their key findings:

- Inter-status gap is **~50 µs mean per servo, independent of baud rate** (their eq. 2). Figure 5 at 4 MBaud shows a bootstrap gap (host instruction packet-end → first slot reply) ≈ 150 µs and steady-state inter-slot gaps of ~60–80 µs.
- The baud-independence implicates a fixed firmware cost: their hypothesis is that the servo's STM32F103 parses the predecessor's status byte-by-byte before deciding to fire, and that loop's cost dominates IDLE latency at every supported baud.
- A single chain member with non-zero RDT bottlenecks the whole chain: observed firmware applies RDT counted from instruction-end before firing, even mid-chain. Their lessons-learned recommendation is to write RDT = 0 to every chain member at boot.

Two consequences shape the design that follows. First, slot timing is not deterministic from the request alone; predecessor body length and predecessor identity both matter, so a slot's fire instant must be derived from observed bus events rather than computed forward. Second, a silent predecessor stalls the entire chain — a single missing reply is the protocol's failure mode at the wire, and host implementations treat it as a transaction failure.

## 2. Design principles

Three principles underlie the design.

**Parser drives, classifier serves.** The protocol parser consumes the RX DMA ring one byte at a time and emits typed events per protocol field. The classifier walker — which turns falling-edge timestamps into per-byte start ticks — advances in lockstep with the parser, exposing the current byte's tick at the consumer's point of interest. Per-byte ticks are consumed at event-handling time and discarded; nothing is stored in a derived ring.

**Per-packet anchoring.** The DXL header `FF FF FD 00` has a unique signature on the wire as bytes (DXL stuffing guarantees the sequence cannot occur in payload) and as falling edges (a five-edge pattern with distinctive inter-edge intervals followed by a guaranteed quiet window). The classifier recovers its anchor from this signature at every header; drift, glitches, and overruns cannot accumulate across packet boundaries.

**Sequence, not schedule.** For slots k > 0, Plain chain timing reduces to one piece of state — the immediate predecessor's ID, resolved at slot-demarcation parsing as the slot ID listed just before the chip's own. Foreign packet bodies are consumed by a universal byte-skip that runs whether the chip is in a chain or not — load-bearing for parser sync. The chain-fire check is a rider on the skip-exhaust event: at the moment the byte-skip on the chip's immediate predecessor's packet exhausts, CCR3 is armed in the same drain-handler invocation (USART1 IDLE, DMA1_CH7 HT, or DMA1_CH7 TC). The chain path for k > 0 schedules nothing, so there is nothing to cancel on stall — a silent predecessor simply never produces its skip-exhaust, CCR3 never arms, and the host's transaction layer sees a missing reply. Slot 0 takes the single-target path: scheduled at `packet_end + RDT`, reusing the existing single-target reply infrastructure unchanged. No forward arithmetic, no fixed slot grid, no inter-slot gap constant, no RDT applied to slots k > 0.

These principles compose. The prior design's byte-time ring existed because the classifier produced byte ticks that the parser then read by index; once the parser drives, the walker hands ticks directly to each consumer at the moment they are useful, and the ring becomes a small set of scalars per consumer.

## 3. Streaming parser

Three layers compose the receive path. The **universal streaming parser** is a byte-range-to-event iterator: no business logic, no host/servo role, no DMA awareness. The **transport driver** pumps the parser from the RX DMA ring's tail-to-NDTR slice, owns per-packet state, translates `(offset, size)` chunk events into concrete byte slices, and decides which complete instructions reach the dispatcher and which are skipped at the ring tail. The **dispatcher** is event-driven and stateful — it tracks each in-flight instruction across the event sequence (header → payload chunks → CRC verdict), copies payload bytes into its validation-before-commit staging buffer as chunks arrive, and commits or discards atomically at the verdict event. The full receive path holds one buffer copy: the dispatcher's staging ground.

**Parser state.** Owns no payload buffer; total parser state is fixed scalars on the order of 32 bytes (cursor, FSM tag, cached header fields, CRC accumulator).

**Event granularity.** One event per protocol field: header match, instruction-header (id, instruction, addr, length), status-header (id, length, error), per-slot demarcations inside Sync Write / Bulk Write / Sync Read / Bulk Read parameters, payload region start and end markers, payload chunk markers carrying `(offset, size)` ranges into the RX DMA ring, CRC verdict, and resync. Payload data flows as ring-relative offsets — no borrow lifetime, no parser-side copy.

**Chunk continuity.** Each payload chunk is a single contiguous memory region (no modular wrap on the consumer side); consecutive chunks within a packet are contiguous in the byte stream; the sum of chunk sizes between payload start and end equals the header's advertised length. Chunks split at two boundaries only: the RX ring wrap, and the end of a poll input slice. This lets the consumer copy each chunk naively into its own staging buffer without coordinating with the parser's drain cadence.

**Resumability.** Two resumption shapes, both driver-driven. If the input range ends mid-payload, the in-flight chunk is emitted up to the boundary and parser state is preserved — the next poll resumes from the next byte and continues seamlessly. If the driver breaks out of the iterator mid-stream to skip a foreign body, the driver explicitly resets the parser before continuing and feeds it again only after the skipped byte count reaches zero. This driver-gated countdown is load-bearing — it keeps the parser fed only at packet boundaries, so parser FSM state and ring tail stay aligned across packets the chip does not parse fully.

**Driver rules.**

- *Drop foreign packet bodies at the ring tail, not in the parser.* When a header event identifies a foreign packet (a foreign single-target instruction, a foreign single-target reply, or any foreign status during a chain the chip is participating in), the driver breaks the iterator, records the body byte count from `length` and the packet's ID, resets the parser, and advances the RX ring tail past those bytes as they arrive — no parser invocation on the body, no CRC fold, no timestamp consumer. The parser is fed again only after the skipped byte count reaches zero. The parser owns no skip FSM; the universal byte-skip is a driver concern, load-bearing for keeping the parser fed only at packet boundaries across packets the chip does not parse fully. The chain-fire check (§5.2) hooks onto the skip-exhaust event as a separate driver concern. Fast Last (§6) is parser-independent: it reads RX ring bytes directly to accumulate the CRC over predecessor packets, via the SysTick catchup grid and CC3 NDTR-tap. The chip's outgoing reply CRC folds these bytes without parser involvement.
- *Never reset the parser inside an outer chain instruction.* For mixed-ownership Sync Write or Bulk Write packets, the parser is mid-packet inside the outer instruction; the outer CRC accumulator must be preserved end-to-end. The driver feeds all bytes uniformly and gates dispatcher forwarding at the slot level — forward chunks for the chip's own slot, drop chunks for foreign slots.
- *CRC matters in two places only.* Inbound instructions the chip will act on (verdict reported at the closing CRC event) and the chip's own Fast Last reply (where the trailing CRC is patched after folding predecessor bytes — §6). Everywhere else the parser still folds the bytes it sees, but the driver does not consult the verdict.

**Dispatcher rule.**

- *Validate before commit.* At a Good CRC verdict on an instruction the chip owns, the dispatcher validates its staging buffer against the control table layout — range and type checks, write permissions. Pass commits atomically; fail discards. Corrupted CRC and resync both discard staging without touching live registers.

## 4. Header-anchored classifier

### 4.1 The header's edge signature

UART frames LSB-first with active-low start and active-high stop bits. For the DXL header `FF FF FD 00`:

- `0xFF` = `1111_1111`: one falling edge — the start bit.
- `0xFD` = `1111_1101`: two falling edges — start bit, then the bit 0 → bit 1 transition.
- `0x00` = `0000_0000`: one falling edge (start bit), then no further falling edges through the byte (eight LOW data bits + HIGH stop is a single up-transition).

Wire timeline at one bit-time per cell, bytes back-to-back:

```
byte:        0xFF                0xFF                0xFD                0x00            (next)
edges:       ↓                   ↓                   ↓ ↓                 ↓               …
offsets:     0                   10                  20  22              30
intervals:        ~10·bit              ~10·bit           ~2·bit    ~8·bit    ≥7·bit quiet
```

Five edges, four distinctive inter-edge intervals (`~10, ~10, ~2, ~8` bit-times), then a guaranteed quiet window of ≥7·bit_time through the `0x00` body. DXL stuffing — insert `0xFD` after any payload `FF FF FD` — guarantees this five-edge pattern cannot occur in payload. The quiet window disambiguates a stuffed `FF FF FD FD` near-match: a fifth `FD` would place a sixth edge ~2·bit_time after the fifth, where the header has ≥7·bit_time silence.

### 4.2 Anchor establishment

The parser drives. On each header event, the classifier walks the ET ring backward from its tail, looking for a five-entry window whose inter-entry deltas match the signature within tolerance, followed by the quiet gap before the next entry. A match yields the anchor: the start tick of header byte 0.

The back-search latency budget is comfortable. By the time the parser emits the header event, the corresponding falling edges have been latched into ET for at least 10·bit_times — all five header edges plus the start bit of byte 4 are present. The search bound is at most a few entries past the most recent header in the worst case; loose bounds are inexpensive.

### 4.3 Walker advance and tick exposure

Once anchored, the walker uses the `[9·bit, 11·bit]` start-bit window from the prior design to identify each successive byte's start tick. The trigger model is unchanged: DMA1_CH7 HT/TC drives the walker forward over newly-captured edges, and USART1 IDLE drains the tail for small packets that do not fill the ET ring enough to trip HT.

What changes is the output. Instead of writing each byte tick into a derived ring, the walker exposes per-byte ticks at two consumer touch points:

- A current-tick accessor queried by the caller at event-handling time. Used for one-shot stamps — start tick of a header, packet-end tick of an instruction the chip will reply to. The chain path for slots k > 0 uses neither (slot 0 uses the packet-end stamp via the single-target reply path).
- A flag-gated push to the HSI drift accumulator, running on every body byte while an instruction is in flight. The flag is set at any instruction-header event and cleared at the matching CRC event or earlier on resync. Status packets never contribute; the source clock for drift sampling is always the host's HSE.

Per-packet timing state collapses to anchor, sync-start tick, byte count, plus the drift accumulator's running scalars. Chain-participation state owns no ticks at all — a `predecessor_id` and a chain-pending flag, with the universal byte-skip's `skip_remaining` / `current_skip_id` riding alongside as foreign-packet handling state.

### 4.4 Freshness and failure modes

ET DMA captures every falling edge unconditionally; HT/TC fires every ET/2 edges and IDLE drains the tail. Worst-case walker staleness at 3 Mbaud peak (1 edge/byte minimum, ET = 128) is 64 bytes of wire time ≈ 213 µs, well under any reply window. ET overflow is detected by comparing NDTR against the walker's last-consumed cursor at HT/TC; the overflow is a counted fault and the next header re-anchors from scratch.

Header pattern mismatch under EMI or HSI excursion fails for that packet only, with distinct impacts per consumer. HSI drift sampling — the only consumer requiring byte-precise anchored ticks — is skipped: the drift accumulator's flag stays cleared throughout the body so unanchored samples cannot poison the calibration loop. Single-target reply timing (slot 0 of a chain, or any Ping/Read/Write reply) is degraded but not lost: the fire scheduler falls back to a TIM2 read in the parser-drain handler at packet close, which sits within RX DMA drain latency of the wire packet end — well inside the host's read window for any realistic RDT. No IDLE backdate is used; the fallback still sources TIM2's free-running counter, with the event-driven moment chosen for proximity to packet end rather than back-computed from IDLE. Chain slots k > 0 fire on predecessor skip-exhaust rather than on a stamped tick, so they are anchor-independent and unaffected. The next packet's header re-attempts anchor from scratch. First-edge ambiguity at boot or post-IDLE is no longer a hazard: anchors come from header pattern matches, and until the first header arrives there is no timing state to be wrong about.

The HSI drift budget is unchanged from the prior design. Static window tolerance of ±10% gives roughly ten times margin over realistic V006 HSI excursion. The pattern-match tolerances on `~10, ~10, ~2, ~8` need similar slack — approximately ±1·bit on the wide intervals and ±0.5·bit on the narrow `~2·bit` gap.

### 4.5 Ring sizing

The prior design sized the byte-time ring to match the RX ring so that byte index `i` in RX mapped to byte time `BT[i mod 64]`. With no derived ring, that constraint vanishes. RX and ET sit on independent axes, linked only by drain cadence.

ET sizing balances three pressures: (a) HT/TC ISR-entry overhead, which falls as ET grows; (b) anchor back-search depth, capped at `ET/2 − 5` edges by the half-period drain bound; (c) SRAM cost, linear in ET. On V006 (48 MHz HCLK, 8 KB SRAM) across the 1–3 Mbaud envelope, ET = 128 (`u16` slots, 256 B) is the smallest size at which all three sit comfortably.

**Edge cadence.** Peak edge rate is `baud × edges_per_byte / 10` with `edges_per_byte ∈ [1, 5]` — `1` for sustained `0xFF`/`0x00` (start bit only), `5` for sustained `0xAA` (start bit plus four bit transitions). At 3 Mbaud high-density burst that's ~1.5 Medges/sec; at 1 Mbaud high-density, ~500 kedges/sec. Real packet streams mix densities, but the design must absorb the peak instantaneously.

**HT/TC ISR rate vs ET.** Each event drains ET/2 edges; HT + TC together pace at one event per ET/2 edges. Through the 3 Mbaud peak on V006:

| ET (slots) | HT period | Entry rate | Entry overhead at 48 MHz |
| --- | --- | --- | --- |
| 32 | ~10.7 µs | ~94 kHz | ~6% CPU |
| 64 | ~21.3 µs | ~47 kHz | ~3% CPU |
| 128 | ~42.6 µs | ~23 kHz | ~1.5% CPU |
| 256 | ~85 µs | ~12 kHz | ~0.7% CPU |

Walker per-edge work (~25 cycles) is set by edge rate, not by ET — smaller ET only adds entry overhead and never reduces walker cost. ET = 128 fits the ISR-entry budget at the 3 Mbaud upper edge with headroom; ET = 64 starts to crowd the ~25% transport CPU envelope (§8) under sustained burst.

**Anchor lookback budget.** At every header event the classifier walks ET backward for the 5-edge signature, up to `ET/2 − 5` slots back. The header event fires exactly once per packet — at the first drain that exposes the header bytes to the parser — and the drain cadence caps that drain at `ET/2` fresh edges. So the walker tail at header-event time is at most `ET/2 − 1` past the header, and back-search distance is at most `ET/2 − 5`, regardless of packet size: large packets that trip HT mid-packet hit the bound at HT-wake (walker tail at edge `~ET/2 − 1`, back-search `ET/2 − 5`); small packets that wake on IDLE land below it (walker tail at edge `E − 1` of an `E ≤ ET/2` packet, back-search `E − 5`); subsequent HT/TC within the same packet reuse the established anchor and trigger no second back-search. The budget is tight, not a margin choice — picking smaller fails the HT-wake case, picking larger reaches further back than the drain cadence allows. Whether the bound is sufficient reduces to whether `ET/2` edges covers typical packets at the first drain. At ~3 edges/byte (random content) ET = 128 gives a ceiling of 64 edges ≈ 21 bytes of packet — comfortable for typical Ping / single-Read replies (~12–18 bytes); larger packets simply have their header in an earlier HT batch and anchor cleanly at first wake. ET = 64 collapses the ceiling to 32 edges ≈ 10 bytes, *below* a typical 14-byte Ping reply (~30 edges at random content); anchor misses become the common case rather than the exception. Anchor misses fall back to the §4.4 TIM2-read path for `packet_end_tick`, but the design point is to anchor on every typical packet so HSI drift sampling and slot-0 single-target timing track without degradation.

**RX sizing.** RX must hold the bytes that arrived between drains. Worst case is 1 edge/byte content (`0xFF`/`0x00`-dense), where ET fills at byte rate and HT fires every ET/2 bytes — RX must hold ≥ ET/2 bytes. With ET = 128, RX = 64 sits at the constraint floor; RX = 96 buys headroom against burst alignment if the budget allows.

**Total.** ET (256 B) + RX (64 B) + classifier scalars (~16 B) ≈ 336 B, ~4% of V006 SRAM. Doubling ET halves HT cadence and grows the lookback budget but adds 256 B for marginal benefit on typical traffic; halving ET drops 160 B but undercuts the lookback budget below typical packet edge counts. ET = 128 / RX = 64 is the design point at which all three pressures are simultaneously comfortable on V006.

## 5. Plain chain timing

### 5.1 Fire rule

For chain replies at slots k > 0 (inside a Plain Sync Read or Plain Bulk Read), the chip arms CCR3 at

    fire_tick = TIM2.CNT + ccr3_setup_offset

at the moment any parser-drain handler — USART1 IDLE, DMA1_CH7 HT, or DMA1_CH7 TC — observes the byte-skip exhaust of the chip's immediate predecessor, matched by ID. The setup offset is just enough to clear CCR3 arm/match (a few µs). There is no `predecessor_end + something` term, no `packet_end + RDT` term, no `max()`. The wait-for-predecessor guarantee is structural: a packet-end observation cannot occur until the predecessor's complete bytes have been consumed, which by definition means the predecessor has stopped driving the bus.

Slot 0 is outside this rule. As the first reply to the host's chain instruction, slot 0 is mechanically a single-target reply and uses the standard single-target fire rule:

    fire_tick = packet_end_tick + RDT + ccr3_setup_offset

where `packet_end_tick` is the chain-instruction's packet-end stamped by the classifier walker at the parser's CRC-good event. The slot 0 fire is scheduled, not in-handler, and behaves identically to a Ping or single-Read reply.

For slots k > 0, the chip does not wait for any subsequent event. The handler that observes the predecessor's packet-end arms CCR3 in the same invocation, regardless of which trigger drove it. None of these handlers stamps a time onto the fire; the fire tick is derived from the current TIM2 counter at handler entry, so no backdated timestamp survives in the chain path.

### 5.2 Algorithm

**Universal byte-skip.** Every foreign packet — chain predecessors, foreign single-target instructions, foreign single-target replies — is consumed by byte-skip, not by parser walk of the body. At a foreign header event the chip records `skip_remaining` (body bytes left in this packet) and `current_skip_id` (the packet's ID), then advances the RX ring tail one byte per arrival until `skip_remaining` reaches zero. No parser invocation on the body, no CRC fold, no timestamp consumer. The skip-exhaust marks the foreign packet's packet-end. This always-on skip is load-bearing for parser sync — without it the parser's view of the ring tail would lag the wire across every foreign packet, and `length`-driven arithmetic against the RX DMA NDTR would drift across the next header.

**Predecessor identification.** At the chip's own Sync/Bulk Read instruction header, the chip enters chain-pending state. The slot index is not available at this event — slot IDs arrive as a stream of subsequent per-slot demarcation events as the parser walks the request parameters. The chip walks the slot-ID list tracking the most-recently-seen ID; when it sees its own ID, the previous slot ID becomes `predecessor_id`. If the chip's ID is the first demarcation (slot 0), the chain mechanic does not apply — the chip exits chain-pending and takes the single-target reply path (next paragraph). If the chip's ID never appears in the param list, chain-pending is cleared at end-of-param-list.

**Slot 0 as single-target.** When the chip's ID is the first slot demarcation, the chip behaves exactly as it would for a single-target Read reply. At the chain-instruction's CRC-good event, the classifier walker stamps `packet_end_tick`; the chip then arms CCR3 at `packet_end_tick + RDT + ccr3_setup_offset`. The fire is scheduled — between the arm and the match, the chain instruction has finished, the bus has been idle for RDT, and CCR3 fires on its own. Slot 0 inherits the existing single-target reply infrastructure unchanged; no chain-pending state, no skip-exhaust check.

**Fire on predecessor's packet-end (slots k > 0).** The chip awaits the byte-skip exhaust of the foreign status packet matching `predecessor_id`. The parser drains on three triggers — USART1 IDLE, DMA1_CH7 HT, DMA1_CH7 TC — and skip-exhaust can land inside any of them. At each skip-exhaust the chip tests: if chain-pending is set and `current_skip_id == predecessor_id`, arm CCR3 at `fire_tick = TIM2.CNT + ccr3_setup_offset` in the same handler invocation and clear chain-pending.

**Silent predecessor (slots k > 0).** A silent immediate predecessor never produces a status-header for `predecessor_id`; the skip-exhaust check never matches; the chip never fires; the host's transaction layer sees a missing reply. Nothing needs cleanup because nothing was armed — the stale TX DMA buffer sits harmlessly and the next instruction's reply overwrites it. Upstream silence propagates the same way: if a slot before `predecessor_id` stalls, that slot's reply never transmits, the chain collapses upstream of the chip, and `predecessor_id` itself never transmits — the check still never matches. Slot 0 has no servo predecessor to go silent; it depends only on the chip's own ability to parse the chain instruction.

### 5.3 Silent predecessor

The §5.2 silent-predecessor handling matches observed Robotis behavior — a single silent slot kills the whole chain and host implementations do not attempt to skip past the gap. The chip's chain state implicitly resets at the next instruction-header event, when the chip re-derives whether it owns a slot in the new request. No cascading-silence recovery, no synthetic predecessor end, no per-slot or chain-level timeout.

The next-instruction-header boundary is sufficient only when the universal byte-skip (§5.2) drains or expires before that header's bytes reach the parser. At slow baud the skip's per-byte deadline can outlive the inter-packet gap, eating the next preamble's leading bytes before the header event fires — at which point the parser never sees the new instruction and the skip-cleared-at-header trigger never gets the chance to run. The chip's own TX completion is the additional reset boundary: at `on_tx_complete` any in-flight skip is cleared, since our chain participation is over regardless of whether the chain ran clean or collapsed. For Plain (non-chain) replies the skip is already `None` and the clear is a no-op.

### 5.4 Drift sampling coverage

The HSI drift estimator runs on every instruction packet regardless of target ID — instructions are always host-transmitted (HSE-clocked), so the source clock is consistent. Status packets never contribute, gated by the `hsi_active` flag described in §4.3. The ~7 header-byte ticks before flag activation are lost per instruction, which is acceptable: body bytes dominate the sample rate, and packets with no body (Ping) contribute little either way.

### 5.5 Inter-slot gap on the wire

For slots k > 0, the natural gap between the chip's slot packet-start and the immediate predecessor's packet-end is `drain trigger latency + handler entry + drain + CCR3 arm + CCR3 → match`. The drain trigger is whichever of USART1 IDLE, DMA1_CH7 HT, or DMA1_CH7 TC fires first after the predecessor's last byte is UART-buffered; the UART buffering itself adds ~1·byte_time regardless of which trigger wins. Handler entry and CCR3 add ~10 µs roughly baud-independent. At 1 MBaud the gap is ~20 µs; at 4 MBaud ~13 µs. HT/TC trigger paths can land earlier than IDLE-only by up to one IDLE-assertion delay (~1·byte_time), but the wait-for-predecessor guarantee is structural, not timing-derived: the byte-skip cannot exhaust until the predecessor's complete wire bytes have been consumed.

Slot 0's bootstrap gap (instruction packet-end → slot 0 packet-start) is the configured RDT, identical to the single-target reply gap. Bestmann observed ~150 µs bootstrap at 4 MBaud [1], consistent with RDT in that range.

Against the Robotis baseline of ~50–80 µs steady-state inter-slot gap [1], the design lands at or below the baseline at typical deployment baud (1–4 MBaud). The baseline is dominated by the Robotis firmware's byte-by-byte parse loop, which this design collapses by routing RX through DMA and keeping the IDLE handler lean. Hosts read bytes as they arrive, so faster-than-baseline is invisible on the wire.

The Robotis per-servo RDT-in-chain quirk is not inherited — RDT applies only at slot 0 (matching spec single-target semantics); the chain fire rule for slots k > 0 ignores RDT. A non-zero RDT on a slot k > 0 chip configures its single-target reply timing but has no effect on its chain participation.

## 6. Fast Last residue fold

The Fast Last post-fire CRC fold mechanic — SysTick catchup grid, CC3 owns post-fire residue, deadline-bounded busy-wait — is unchanged; see [dxl-hw-timed-transport.md §10.6](dxl-hw-timed-transport.md) for the full path. The only delta is what the fold consumes: the streaming RX redesign retires the parser-and-classifier walk inside the fold loop. Both the catchup-grid CMP bodies and the CC3 post-fire body now read raw bytes directly off the RX DMA ring and accumulate them into the running CRC. No event emission, no walker advance, no per-byte tick. NDTR supplies the byte count; nothing in the fold path needs a tick.

**RX-tail ownership during the fold.** Three consumers can advance the RX ring tail: the parser drain, the universal byte-skip (§5.2), and the Fast Last fold. While the fold is active they are mutually exclusive — the fold's CRC depends on observing exactly the `predecessor_bytes` between arm-time tail and the wire end of the chip's last predecessor, and any parser-side or skip-driven advance during that window silently steals bytes the fold needs. The fold owns the tail for the duration of the window, bracketed by two events:

- **Arm** — at `send_slot(Last)` the driver suppresses edge-driven re-entry (DMA1_CH7 HT/TC, the walker's primary pump) at the source. ET DMA continues to capture; only the IRQ is masked. The in-flight `poll()` call that drove the arm event yields without draining further bytes, leaving them in the ring for the fold's drain path.
- **Release** — the fold finalizes naturally when `predecessor_bytes` have been consumed (CRC patched, active cleared) or cancels at `on_tx_complete` if a silent predecessor stalled the chain (matching the universal-skip clear in §5.3). Both transitions implicitly re-open the tail to the parser, and the next poll re-enters normally.

Between arm and release, USART1 IDLE and DMA1_CH5 HT/TC continue to fire — they cannot be masked without losing TX-half awareness — and may re-trigger `poll()`. Each such entry checks whether a fold is active and returns immediately without touching the parser or the tail cursor. The two suppression points — edge-IRQ mask at arm and `poll()` self-gate at re-entry — close the same failure (parser-side tail advance during the fold) at two different sources.

`patch_deadline_tick` survives the redesign as a bench-defended floor signal: at 3 Mbaud worst-case Fast Last, 10K-cycle runs see zero miss at `GUARD = 1` and ~3% miss at `GUARD = 2`. Wire CRC stays correct in either case — the deadline is a telemetry signal at the 3 Mbaud floor, not a kill switch.

## 7. Resource shape

### 7.1 CPU

Savings are chain-deployment dependent — and that is the design point. DXL deployments at scale are buses; the chip's interest in the bus is asymmetric (~one instruction per cycle plus zero interest in N−1 of every N foreign Status frames in a chain reply). At a single-servo bench the savings are nominal. At a populated chain they scale linearly with `N`.

| Scenario | Walker work | Persisted timing scalars |
| --- | --- | --- |
| Single-servo instruction → own reply | Per-byte advance across the instruction; anchor recovery at header | anchor, packet-end tick, drift accumulator |
| N-servo Plain Sync / Bulk Read, in-chain receive, slots k > 0 | Walker advances per byte across instruction + every predecessor (for anchor coherence); parser drains only the instruction and each foreign packet's header — foreign packet bodies are byte-skipped past the ring tail (universal mechanic, §3) | anchor, drift accumulator, `predecessor_id` (u8), `chain_pending` (bool); universal-skip state (`skip_remaining: u8`, `current_skip_id: u8`) — no fire-side ticks |
| N-servo Plain Sync / Bulk Read, slot 0 | Same as single-servo instruction → own reply; slot 0 takes the single-target path at `packet_end + RDT` | Same as single-servo |
| N-servo Fast Sync / Bulk Read, Fast Last slot | Per-byte advance across instruction + each predecessor (for anchor coherence); fold reads RX ring raw | Single-servo state + Fast Last CRC state |

Per-byte fixed cost still scales with byte count — the walker has to keep its anchor synchronized for the quiet-window check at the next header — but the store drops to scalars and the consume drops to zero on uninterested bytes.

### 7.2 Memory

| Region | Prior design | Streaming RX |
| --- | --- | --- |
| RX ring | 64 | 64–96 |
| ET ring | 256 | 256 |
| BT ring | 128 | — |
| Parser scratch | 256 | — |
| **Transport-owned total** | **704 B** | **320–352 B** |

Two structural collapses. The byte-time ring's 128 B of derived storage collapses to a handful of scalars per packet plus per-consumer scalars; per-packet timing state fits in ~20 bytes, and chain-participation state is ~6 bytes regardless of chain depth, carrying no fire-side timestamps at all. The parser scratch collapses entirely: payload data for Writes flows as offset/size ranges directly into the control-table staging buffer — which already exists for validate-then-commit semantics — and parser state is ~32 bytes of scalars.

~352–384 bytes freed against the prior design on an 8 KB chip. Modest in absolute terms; structurally clean because both buffers dropped were duplicating storage that exists elsewhere — the byte-time ring was an index parallel to RX, and parser scratch was a staging area parallel to control-table staging.

## 8. Verification points

The following items require bench measurement or implementation-time decisions on V006.

- **Header pattern-match tolerance under HSI excursion.** The design's single point of failure for resync. The `~10, ~10, ~2, ~8` intervals plus quiet window need slack that survives cold-boot HSI without false-rejecting valid headers, and tight enough that stuffed-payload near-matches (`FF FF FD FD`) do not false-accept. False-reject costs one packet; false-accept poisons the whole packet's timing. Bench-tune both bands against adversarial stuffed payloads and cold HSI.
- **Resync event semantics in the streaming surface.** The parser already resyncs internally; surfacing the existing resync as a typed event must not change FSM behavior. Confirm via parser unit tests.
- **Walker / parser ordering for the HSI flag.** The walker pushes per-byte ticks gated on a flag toggled by parser events. If both advance in the same ISR body, the relative order determines whether the byte that closes an instruction (CRC event) has its tick pushed before the flag clears. Specify the ordering — parser advance and event emission complete first, then walker pushes ticks for any newly identifiable bytes — or equivalently sample the flag after parser advances.
- **Sustained-burst walker cost.** The walker advances its anchor across every byte to keep the quiet-window check valid for the next header. Confirm under sustained 3 Mbaud bursts that walker + parser + event handling fits the ~25% CPU envelope of the prior design.
- **Parser-drain coverage across triggers.** The chain-fire path relies on the parser advancing on all of USART1 IDLE, DMA1_CH7 HT, and DMA1_CH7 TC, with each handler's drain promptly observing the predecessor's skip-exhaust as bytes cross the ring tail. Confirm the runtime routes the parser into each handler with consistent ordering relative to the walker and the skip-exhaust check.
- **Parser-off window after own reply.** The chip can stop calling poll after its own reply's TC; IDLE-driven re-entry on the next instruction must reliably resync at the next header. Verify on bench.
- **Outer CRC reset rule.** The drop-and-reset pattern is safe for top-level foreign packets but unsafe inside an outer Sync Write or Bulk Write — the outer CRC accumulator must be preserved end-to-end. The rule is: never reset the parser while inside an outer chain instruction; gate payload handling at the slot level, feed all bytes uniformly.
- **Predecessor-ID resolution shape.** Slot IDs arrive as a stream of per-slot demarcation events following the chain-instruction header, not as a single field. The chip walks the list tracking the most-recently-seen ID; `predecessor_id` resolves when the chip's own ID arrives in the stream. Three cases: own-ID-first → chain mechanic does not apply, chip exits chain-pending and takes the single-target reply path (`packet_end + RDT`) at the chain-instruction's CRC-good event; own-ID-not-first → `predecessor_id = previous_seen_id`, chain-pending remains set; own-ID-absent → chain-pending clears at end-of-param-list. The chain-pending flag may be set at the instruction header and resolved per-case as the demarcation stream arrives.
- **SysTick CMP ownership.** SysTick CMP is owned exclusively by Fast Last catchup; the Plain chain path holds no SysTick state. Verify the runtime arms and disarms the catchup CMP cleanly around each Fast Last request, with no stale arm surviving a Plain request that interleaves.
- **NDTR wrap math in the residue fold.** The fold reads from the RX ring between arm-time tail and current NDTR; the absolute byte-delta must handle a single NDTR register wrap correctly across the residue window. Single-wrap is the expected case.
- **CC3 NDTR-tap fold deadline.** Re-measure end-to-end fold cost with the slim CC3 body. The §6 hw-timed baseline should hold or improve, sitting well under the existing `patch_deadline_tick` margin.
- **State-scoping discipline.** Each persisted tick scalar has a deliberate owner — packet-end tick for the reply scheduler, anchor tick for the walker. Chain-participation state owns no tick scalars. Each lifetime must be bounded by its consumer's done-condition so stale values cannot poison subsequent packets.

## 9. Summary

The design replaces the prior hardware-timed transport's derived byte-time ring and parser scratch with a streaming, event-driven parser walking the RX DMA ring directly. Per-byte ticks reach consumers via the walker's current-tick accessor and a flag-gated push to the drift accumulator, queried at event-handling time and discarded thereafter. Per-packet anchors are recovered from the DXL header's unique on-wire edge signature, guaranteed distinct by DXL stuffing, so timing drift never accumulates across packet boundaries. Foreign packets — in-chain predecessor packets, foreign single-target packets — are consumed by a universal byte-skip that keeps the parser's view of the ring tail synchronized with the wire. Plain Sync / Bulk Read chain timing — left at fixed slot offsets in the prior design — splits cleanly between slot 0 and slots k > 0. Slot 0 inherits the single-target reply path unchanged (`packet_end + RDT`). Slots k > 0 reduce to a single piece of state, the immediate predecessor's ID, resolved at slot-demarcation parsing; CCR3 is armed in-handler by whichever of USART1 IDLE, DMA1_CH7 HT, or DMA1_CH7 TC observes that ID's byte-skip exhaust. Nothing in the slot k > 0 chain path is scheduled, so nothing needs cleanup on stall — a silent predecessor manifests as a missing reply at the host's transaction layer. RDT applies at slot 0 (matching spec single-target semantics) and not at slots k > 0. The Fast Last post-fire CRC residue fold shrinks to a short loop over raw bytes derived from NDTR, with no classifier involvement. Inter-slot gap on the wire sits at or below the observed Robotis baseline at typical deployment baud. Transport-owned RAM drops by ~352–384 B against the prior design; CPU during foreign-packet handling scales linearly with bytes the chip actually consumes rather than with bus traffic. The trigger model (TIM2 input capture, DMA1_CH7 HT/TC, USART1 IDLE), priority allocation, and Fast Last catchup grid are unchanged.

## References

[1] Bestmann, M.; Güldenstein, J.; Zhang, J. *High-Frequency Multi Bus Servo and Sensor Communication Using the Dynamixel Protocol*. RoboCup 2019, Hamburg Bit-Bots, University of Hamburg. <https://2019.robocup.org/downloads/program/BestmannEtAl2019.pdf>
