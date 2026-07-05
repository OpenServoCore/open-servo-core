# DXL 2.0 Streaming RX on the CH32V006

## Abstract

A streaming, event-driven RX design for the DXL 2.0 transport on the CH32V006, decoupling protocol parsing from reply timing. The parser consumes the RX DMA ring one byte at a time and emits typed protocol events; timing is a **software packet-end estimate** — a drain-flavor-corrected WireClock reading taken at drain-ISR entry ([dxl-hw-timed-transport.md §8](dxl-hw-timed-transport.md)), attached to the parser's `Crc` event and consumed once. There is no derived byte-time ring and no per-byte edge timestamp: reply timing needs one number per request (packet-end), FAST k > 0 slots need one anchor (the chain Status packet's start, from a byte-count estimate), and HSI drift needs a rate over a burst (NDTR/byte-count spans) — none require per-byte resolution. Foreign packets — in-chain predecessor packets, foreign single-target instructions and replies — are consumed by a universal byte-skip that advances the RX ring tail past each packet's body without parser walk, CRC fold, or timestamp; the skip is load-bearing for keeping the parser's view of the ring tail synchronized with the wire. Plain (non-Fast) Sync / Bulk Read chain timing — left at fixed slot offsets in the prior design — reduces for slots k > 0 to a single piece of state: the chip records its immediate predecessor's ID at slot-demarcation parsing and starts its TX in-handler at that ID's byte-skip exhaust, under whichever of USART1 IDLE, DMA1_CH5 HT, or DMA1_CH5 TC drains the predecessor's last byte. Slot 0 falls outside the chain mechanic — it is mechanically a single-target reply to the chain instruction and is scheduled at `packet_end + RDT`, inheriting the existing single-target reply path unchanged. The chain mechanic (slots k > 0) schedules nothing and so has nothing to cancel; a silent predecessor manifests as a missing reply at the host's transaction layer. The Fast successor chain-CRC pipeline reduces to a checkpoint pickup — the official per-block layout puts the running chain CRC on the wire after every slot, so a successor reads its predecessor's checkpoint instead of folding the window; O(own reply) CPU. Transport-owned RAM drops to ~320 B, and CPU during foreign-packet handling scales linearly with what the chip actually consumes rather than with bus traffic.

**Terminology.** *Chain* and *slot* both appear throughout, each carrying a specific semantic load. *Chain* names the Plain Sync / Bulk Read mode — a multi-reply transaction where slot 0 fires per the standard single-target rule (`packet_end + RDT`) and slots k > 0 fire sequence-driven on their immediate predecessor's packet-end. When "chain mechanic" or "chain fire" appears unqualified, it refers to the sequence behavior of slots k > 0; slot 0 is the bootstrap and inherits the single-target reply path unchanged. *Slot* serves two roles, both standard DXL vocabulary: the position of a servo ID in a request's param list (used for both Plain and Fast contexts — "slot 0", "the chip's slot"), and a timing-driven reply window in the Fast Sync / Bulk Read variant (the "slot grid" of §5.6). The shorthand: Plain semantics = chain (sequence, slots k > 0) + single-target bootstrap (slot 0); Fast semantics = slot (timing window).

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

**Parser drives; timing is a software estimate.** The protocol parser consumes the RX DMA ring one byte at a time and emits typed events per protocol field. The transport needs one timing datum per reply-bound request — the request's packet-end tick — and it takes it in software: the codec stashes the WireClock value and drain flavor at every drain-ISR entry, and at the parser's `Crc` event resolves them into `packet_end_tick` ([dxl-hw-timed-transport.md §8.1](dxl-hw-timed-transport.md)). The tick is consumed at event-handling time and discarded; nothing is stored in a derived ring. There is no edge parser and no per-byte tick — the RX side keeps a handful of scalars, not a parallel timestamp axis.

**No per-byte resolution, because nothing needs it.** The three timing consumers each tolerate a coarse reading: the reply deadline is `packet_end + RDT` (tens of µs, inside a wider host read window); the FAST k > 0 anchor is one Status-packet start (byte-count estimate, §5.6); HSI drift is a rate over a burst (NDTR/byte-count spans, §5.4). An earlier design recovered per-byte start ticks from the DXL header's on-wire edge signature via TIM2 input capture; measurement showed the protocol never spends that precision and the classifier that produced it dominated RX CPU, so it was deleted.

**Sequence, not schedule.** For slots k > 0, Plain chain timing reduces to one piece of state — the immediate predecessor's ID, resolved at slot-demarcation parsing as the slot ID listed just before the chip's own. Foreign packet bodies are consumed by a universal byte-skip that runs whether the chip is in a chain or not — load-bearing for parser sync. The chain-start check is a rider on the skip-exhaust event: at the moment the byte-skip on the chip's immediate predecessor's packet exhausts, the wire starts in the same drain-handler invocation (USART1 IDLE, DMA1_CH5 HT, or DMA1_CH5 TC). The chain path for k > 0 schedules nothing, so there is nothing to cancel on stall — a silent predecessor simply never produces its skip-exhaust, the TX never starts, and the host's transaction layer sees a missing reply. Slot 0 takes the single-target path: scheduled at `packet_end + RDT`, reusing the existing single-target reply infrastructure unchanged. No forward arithmetic, no fixed slot grid, no inter-slot gap constant, no RDT applied to slots k > 0.

These principles compose. The prior design's byte-time ring existed because a classifier produced byte ticks that the parser then read by index; once the parser drives and timing is a per-event software estimate, the ring collapses to a stash of `(now, flavor)` refreshed each drain.

## 3. Streaming parser

Three layers compose the receive path. The **universal streaming parser** is a byte-range-to-event iterator: no business logic, no host/servo role, no DMA awareness. The **transport driver** pumps the parser from the RX DMA ring's tail-to-NDTR slice, owns per-packet state, translates `(offset, size)` chunk events into concrete byte slices, and decides which complete instructions reach the dispatcher and which are skipped at the ring tail. The **dispatcher** is event-driven and stateful — it tracks each in-flight instruction across the event sequence (header → payload chunks → CRC verdict), copies payload bytes into its validation-before-commit staging buffer as chunks arrive, and commits or discards atomically at the verdict event. The full receive path holds one buffer copy: the dispatcher's staging ground.

**Parser state.** Owns no payload buffer; total parser state is fixed scalars on the order of 32 bytes (cursor, FSM tag, cached header fields, CRC accumulator).

**Event granularity.** One event per protocol field: header match, instruction-header (id, instruction, addr, length), status-header (id, length, error), per-slot demarcations inside Sync Write / Bulk Write / Sync Read / Bulk Read parameters, payload region start and end markers, payload chunk markers carrying `(offset, size)` ranges into the RX DMA ring, CRC verdict (`Event::Crc(Good)` / `Event::Crc(Bad)`), and resync (mid-packet aborts: bad length, ring overflow). Payload data flows as ring-relative offsets — no borrow lifetime, no parser-side copy.

**Chunk continuity.** Each payload chunk is a single contiguous memory region (no modular wrap on the consumer side); consecutive chunks within a packet are contiguous in the byte stream; the sum of chunk sizes between payload start and end equals the header's advertised length. Chunks split at two boundaries only: the RX ring wrap, and the end of a poll input slice. This lets the consumer copy each chunk naively into its own staging buffer without coordinating with the parser's drain cadence.

**Resumability.** Two resumption shapes, both driver-driven. If the input range ends mid-payload, the in-flight chunk is emitted up to the boundary and parser state is preserved — the next poll resumes from the next byte and continues seamlessly. If the driver breaks out of the iterator mid-stream to skip a foreign body, the driver explicitly resets the parser before continuing and feeds it again only after the skipped byte count reaches zero. This driver-gated countdown is load-bearing — it keeps the parser fed only at packet boundaries, so parser FSM state and ring tail stay aligned across packets the chip does not parse fully.

**Driver rules.**

- *Drop foreign packet bodies at the ring tail, not in the parser.* When a header event identifies a foreign packet (a foreign single-target instruction, a foreign single-target reply, or any foreign status during a chain the chip is participating in), the driver breaks the iterator, records the body byte count from `length` and the packet's ID, resets the parser, and advances the RX ring tail past those bytes as they arrive — no parser invocation on the body, no CRC fold, no timestamp consumer. The parser is fed again only after the skipped byte count reaches zero. The parser owns no skip FSM; the universal byte-skip is a driver concern, load-bearing for keeping the parser fed only at packet boundaries across packets the chip does not parse fully. The chain-fire check (§5.2) hooks onto the skip-exhaust event as a separate driver concern. The Fast successor pipeline (§6) is parser-independent: it consumes the predecessor window raw and reads the chain-state checkpoint (the window's trailing CRC bytes) directly off the RX ring via the SysTick wake. The chip's outgoing reply CRC extends that checkpoint without parser involvement.
- *Never reset the parser inside an outer chain instruction.* For mixed-ownership Sync Write or Bulk Write packets, the parser is mid-packet inside the outer instruction; the outer CRC accumulator must be preserved end-to-end. The driver feeds all bytes uniformly and gates dispatcher forwarding at the slot level — forward chunks for the chip's own slot, drop chunks for foreign slots.
- *CRC matters in two places only.* Inbound instructions the chip will act on (verdict reported at the closing CRC event) and the chip's own Fast successor reply (where the trailing CRC is patched from the predecessor's wire checkpoint — §6). Everywhere else the parser still folds the bytes it sees, but the driver does not consult the verdict.

**Dispatcher rule.**

- *Validate before commit.* At a Good CRC verdict on an instruction the chip owns, the dispatcher validates its staging buffer against the control table layout — range and type checks, write permissions. Pass commits atomically; fail discards. Corrupted CRC and resync both discard staging without touching live registers.

## 4. Software packet-end timing

The RX side derives its timing from WireClock reads at drain-ISR entry, corrected by drain flavor. No edge hardware, no per-byte tick, no anchored back-search. This section covers the packet-end estimate the codec attaches to a reply-bound request (§4.1–§4.3) and the failure modes the software estimate has (§4.4); ring sizing is §4.5.

### 4.1 Drain stash

The codec stashes `(now, PollSrc)` at every drain-ISR entry, where `now = WireClock::now()` (the free-running SysTick u32) and `PollSrc` names the drain flavor:

- `PollSrc::ByteBatch` — a DMA1_CH5 HT/TC drain (the byte ring published a batch).
- `PollSrc::LineIdle` — a USART1 IDLE drain (the line went quiet).

The stash is a two-field scalar, overwritten each drain. It is the only timing state the RX path keeps per packet, alongside the drift accumulator's running scalars and the chain-participation flags.

### 4.2 Packet-end resolution

At the parser's `Crc` event the codec resolves the stash into `packet_end_tick` via the drain-flavor formula ([dxl-hw-timed-transport.md §8.1](dxl-hw-timed-transport.md)):

    ByteBatch: packet_end = now                               # CRC byte hit DMA just before the poll
    LineIdle:  packet_end = now − BITS_PER_FRAME · tpb         # IDLE latched one frame past wire-end
    then subtract PACKET_END_ENTRY_COMP_TICKS (0 today)

`[[no_idle_timing]]` compliance: IDLE only *selects* the formula (ByteBatch vs LineIdle); the tick itself is a WireClock reading taken at ISR entry, never an IDLE-derived measurement of the wire end. The estimate is good to a few µs — invisible against `packet_end + RDT` and the host read window (plain-reply wire excess min +0.17 / median +5.4 µs at 3M).

### 4.3 Tick exposure and drift spans

Per-request timing reaches consumers at two touch points, both resolved from the drain stash:

- **`packet_end_tick`** — queried by the codec at the `Crc` event for a one-shot stamp: the packet-end tick of an instruction the chip will reply to. Slot 0 of a Plain chain uses it via the single-target reply path; slots k > 0 use no tick at all (they fire on predecessor skip-exhaust, §5.2).
- **FAST status-start** (`status_start_tick`, §5.6) — a deferred FAST slot k > 0 resolves the chain Status packet's start from the RX ring byte cursor at a per-byte RXNE wake, back-projected by the published byte count. One physical reading anchors the whole chain-wide grid.

The **HSI drift estimate** rides the same drain stamps as `(Δticks, Δbytes)` spans (`codec/span.rs`), routed to the drift accumulator. Each drain stamp records `(now, published_cursor, flavor)`; a `SpanTracker` emits a span only when the new stamp pairs with the previous over one contiguous **same-flavor** burst of **Instruction** bytes (foreign Status bytes are another servo's HSI-clocked TX and must never contribute, per `[[drift_sampling_instruction_only]]`). A same-burst gate rejects any span whose tick length strays from `Δbytes · byte_ticks` by more than `expected/16`, so host inter-packet gaps drop out while a real ~2% trim offset passes. Isolated short packets trip only one drain ISR and form no `SpanTracker` pair; the RXNE cold-start window (`DriftWindow` + `RxWakeGate`, §5.4) restores their sampling by accumulating one long span across per-byte wakes. Drift feedback (HSITRIM step + Q8.8 sub-step residual into the fire deadline) is unchanged — §5.4.

### 4.4 Freshness and failure modes

The codec refreshes the RX DMA producer head on every ISR entry (HT/TC or IDLE) via NDTR readback, so the parser drains against the freshest published bytes. The drain triggers are three: **USART1 IDLE**, **DMA1_CH5 HT**, and **DMA1_CH5 TC** — the parser advances under all three, and packet-end / skip-exhaust are observed under whichever fires first (per [dxl-hw-timed-transport.md §9](dxl-hw-timed-transport.md)). The per-byte **RXNE wake window** is a fourth entry point, but only for the FAST status-start wait and the drift sampler — it does not drive the parser.

Failure modes of the software estimate:

- **Drain flavor at the boundary.** A packet that ends with a byte-batch drain (HT/TC lands on the CRC byte) stamps near-exact; one that ends on IDLE stamps one frame back (the §4.2 correction). A packet straddling both is stamped by whichever drain the codec last saw at the `Crc` event — the formula matches the flavor, so the estimate is consistent either way. Same-priority poll blocking (a drain ISR entering behind another High body) shifts `now` a few µs later; this is the median tail in the wire-excess distribution, protocol-irrelevant at RDT scale.
- **No timing state is ever wrong across a boundary.** There is no anchor to carry, no walker to overshoot. Each packet's estimate is resolved fresh at its own `Crc` from the most recent drain; a bad estimate on one packet cannot poison the next.
- **Slot 0 / single-target degradation.** If the drain stash is stale (no drain between the previous packet and this `Crc` — structurally rare, since the CRC byte's own arrival drains the ring), `packet_end_tick` reads an old `now` and the reply fires early or late by the staleness. In practice the CRC byte's HT/TC or the trailing IDLE always refreshes the stash before `Crc`, so this is a defensive concern, not an observed one.
- **Plain chain slots k > 0 are timing-source-independent.** They fire on predecessor skip-exhaust (§5.2), not on a stamped tick, so they are unaffected by any packet-end estimate error.
- **FAST chain slots k > 0 depend on the byte-count status-start** (§5.6): a missed resolution retries on the next byte's wake; if nothing resolvable arrives the slot simply never fires (the FAST collapse contract), and stale traffic past the staleness window drops the parked slot.

The HSI drift budget is unchanged from the prior design: static window tolerance of ±10% gives roughly ten times margin over realistic V006 HSI excursion (≤0.5% at 25 °C post-cal, ~3% cold uncal), and re-centering each span on the measured burst keeps drift from compounding across a packet.

### 4.5 Ring sizing

With no derived timestamp ring, RX sizing is a single question: how many bytes must the RX ring hold between drains? The producer is DMA1_CH5 (circular from USART1.DR); the consumer is the parser drain plus, during a FAST window, the checkpoint pickup. HT/TC publish pins the producer-head lag to `RX_BUF_LEN/2` regardless of content, so the ring must hold one half-period of bytes at the peak arrival rate plus the deepest FAST predecessor window the O(1) drains service (which consume-to-cap by cursor arithmetic each half-ring, so the window need not fit whole).

On V006 (48 MHz HCLK, 8 KB SRAM) across the 1–3 Mbaud envelope, **RX = 64 bytes** sits at the constraint with the HT/TC publish cadence; 128 buys headroom against long Write polls that overrun a half-period deadline (the RX-ring-lap corner, `[[rx_ring_lap_bug]]`). The TX buffer is sized separately to `DXL_TX_MAX_BYTES`.

**Total.** RX ring (64–128 B) + parser scalars (~32 B) + per-consumer timing scalars (~40 B) ≈ **~320 B** transport-owned — the byte-time ring (128 B) and parser scratch (256 B) of the hardware-RX draft are both gone (§7.2).

## 5. Plain chain timing

### 5.1 Start rule

For chain replies at slots k > 0 (inside a Plain Sync Read or Plain Bulk Read), the chip starts the wire inline (`TxBus::start_now` — TX_EN force-active + TX DMA enable, no compare involved) at the moment the parser-drain handler observes the byte-skip exhaust of the chip's immediate predecessor, matched by ID. There is no `predecessor_end + something` term, no `packet_end + RDT` term, no `max()`. The wait-for-predecessor guarantee is structural: a packet-end observation cannot occur until the predecessor's complete bytes have been consumed, which by definition means the predecessor has stopped driving the bus.

Slot 0 is outside this rule. As the first reply to the host's chain instruction, slot 0 is mechanically a single-target reply and uses the standard single-target schedule:

    start_tick = packet_end_tick + RDT

where `packet_end_tick` is the chain-instruction's software packet-end estimate (§4.2) at the parser's `Event::Crc(Good)`. The slot 0 start is scheduled through the hardware TX kickoff ([dxl-hw-timed-transport.md §5](dxl-hw-timed-transport.md)), not in-handler, and behaves identically to a Ping or single-Read reply.

For slots k > 0, the chip does not wait for any subsequent event. The handler that observes the predecessor's packet-end starts the wire in the same invocation, regardless of which trigger drove it. No timestamp survives in the chain path.

### 5.2 Algorithm

**Universal byte-skip.** Every foreign packet — chain predecessors, foreign single-target instructions, foreign single-target replies — is consumed by byte-skip, not by parser walk of the body. At a foreign header event the chip records `skip_remaining` (body bytes left in this packet) and `current_skip_id` (the packet's ID), then advances the RX ring tail one byte per arrival until `skip_remaining` reaches zero. No parser invocation on the body, no CRC fold, no timestamp consumer. The skip-exhaust marks the foreign packet's packet-end. This always-on skip is load-bearing for parser sync — without it the parser's view of the ring tail would lag the wire across every foreign packet, and `length`-driven arithmetic against the RX DMA NDTR would drift across the next header.

**Predecessor identification.** At the chip's own Sync/Bulk Read instruction header, the chip enters chain-pending state. The slot index is not available at this event — slot IDs arrive as a stream of subsequent per-slot demarcation events as the parser walks the request parameters. The chip walks the slot-ID list tracking the most-recently-seen ID; when it sees its own ID, the previous slot ID becomes `predecessor_id`. If the chip's ID is the first demarcation (slot 0), the chain mechanic does not apply — the chip exits chain-pending and takes the single-target reply path (next paragraph). If the chip's ID never appears in the param list, chain-pending is cleared at end-of-param-list.

**Slot 0 as single-target.** When the chip's ID is the first slot demarcation, the chip behaves exactly as it would for a single-target Read reply. At the chain-instruction's `Event::Crc(Good)`, the codec stamps `packet_end_tick`; the chip then schedules the wire start at `packet_end_tick + RDT` through the hardware TX kickoff. Between the arm and the compare match, the chain instruction has finished, the bus has been idle for RDT, and the kickoff fires on its own. Slot 0 inherits the existing single-target reply infrastructure unchanged; no chain-pending state, no skip-exhaust check.

**Start on predecessor's packet-end (slots k > 0).** The chip awaits the byte-skip exhaust of the foreign status packet matching `predecessor_id`; skip-exhaust can land inside any parser-drain trigger. At each skip-exhaust the chip tests: if chain-pending is set and `current_skip_id == predecessor_id`, start the wire (`start_now`) in the same handler invocation and clear chain-pending.

**Silent predecessor (slots k > 0).** A silent immediate predecessor never produces a status-header for `predecessor_id`; the skip-exhaust check never matches; the chip never fires; the host's transaction layer sees a missing reply. Nothing needs cleanup because nothing was armed — the stale TX DMA buffer sits harmlessly and the next instruction's reply overwrites it. Upstream silence propagates the same way: if a slot before `predecessor_id` stalls, that slot's reply never transmits, the chain collapses upstream of the chip, and `predecessor_id` itself never transmits — the check still never matches. Slot 0 has no servo predecessor to go silent; it depends only on the chip's own ability to parse the chain instruction.

### 5.3 Silent predecessor

The §5.2 silent-predecessor handling matches observed Robotis behavior — a single silent slot kills the whole chain and host implementations do not attempt to skip past the gap. The chip's chain state implicitly resets at the next instruction-header event, when the chip re-derives whether it owns a slot in the new request. No cascading-silence recovery, no synthetic predecessor end, no per-slot or chain-level timeout.

The next-instruction-header boundary is sufficient only when the universal byte-skip (§5.2) drains or expires before that header's bytes reach the parser. At slow baud the skip's per-byte deadline can outlive the inter-packet gap, eating the next preamble's leading bytes before the header event fires — at which point the parser never sees the new instruction and the skip-cleared-at-header trigger never gets the chance to run. The chip's own TX completion is the additional reset boundary: at `on_tx_complete` any in-flight skip is cleared, since our chain participation is over regardless of whether the chain ran clean or collapsed. For Plain (non-chain) replies the skip is already `None` and the clear is a no-op.

### 5.4 Drift sampling coverage

The HSI drift estimator runs on every instruction packet regardless of target ID — instructions are always host-transmitted (HSE-clocked), so the source clock is consistent. Status packets never contribute, gated by the instruction flag on the span emitters (§4.3). The signal source is `(Δticks, Δbytes)` spans, not per-byte ticks:

- **Natural spans** pair consecutive drain stamps across a same-flavor Instruction burst (`SpanTracker`). Multi-byte packets drain more than once and pair naturally.
- **RXNE-window spans** cover isolated short packets that drain only once. While the window is open, USART1 RXNEIE wakes per byte; the driver records the burst's first wake `(now, cursor)` and overwrites the last, and `DriftWindow::settle` emits one long span at the packet boundary — gated on an 8-byte floor (`DRIFT_WINDOW_MIN_BYTES`) and the same `±expected/16` same-burst rule, so a low-SNR 1-byte span never reaches the integrator. The window's lifecycle (`RxWakeGate`) shares the single RXNEIE bit with the FAST k > 0 wait via a two-bool reason set, opens at cold boot / applied baud change / staleness, closes on any drift-batch close (below-deadband included — per-byte IRQs must not outlive the sampling), gives up after `DRIFT_WINDOW_MAX_PACKETS` fruitless instruction packets, and reopens after `DRIFT_STALENESS_INSTRUCTIONS` with no accepted span. See [dxl-hw-timed-transport.md §8.3](dxl-hw-timed-transport.md).

The integrator emits two corrections at every batch close: an integer-step HSITRIM write (committed at the next `on_rx_packet_end`) and a Q8.8 sub-step residual that feeds `phase_adjust` into every fire deadline — `deadline = packet_end_tick + delay_ticks + phase_adjust` at both `send_status` and `send_slot` schedule sites. The residual handles the ±half-step quantization gap the integer HSITRIM register can't resolve.

### 5.5 Inter-slot gap on the wire

For slots k > 0, the natural gap between the chip's slot packet-start and the immediate predecessor's packet-end is `drain trigger latency + handler entry + drain + start_now`. The drain trigger is whichever of USART1 IDLE, DMA1_CH5 HT, or DMA1_CH5 TC fires first after the predecessor's last byte is UART-buffered; the UART buffering itself adds ~1·byte_time regardless of which trigger wins. Handler entry and the inline start add ~10 µs roughly baud-independent. At 1 MBaud the gap is ~20 µs; at 4 MBaud ~13 µs. HT/TC trigger paths can land earlier than IDLE-only by up to one IDLE-assertion delay (~1·byte_time), but the wait-for-predecessor guarantee is structural, not timing-derived: the byte-skip cannot exhaust until the predecessor's complete wire bytes have been consumed.

Slot 0's bootstrap gap (instruction packet-end → slot 0 packet-start) is the configured RDT, identical to the single-target reply gap. Bestmann observed ~150 µs bootstrap at 4 MBaud [1], consistent with RDT in that range. The single-target turnaround at RDT = 0 measures ~101 µs on V006 (down from 151 µs in the ISR-fire era).

Against the Robotis baseline of ~50–80 µs steady-state inter-slot gap [1], the design lands at or below the baseline at typical deployment baud (1–4 MBaud). The baseline is dominated by the Robotis firmware's byte-by-byte parse loop, which this design collapses by routing RX through DMA and keeping the drain handlers lean. Hosts read bytes as they arrive, so faster-than-baseline is invisible on the wire.

The Robotis per-servo RDT-in-chain quirk is not inherited — RDT applies only at slot 0 (matching spec single-target semantics); the chain fire rule for slots k > 0 ignores RDT. A non-zero RDT on a slot k > 0 chip configures its single-target reply timing but has no effect on its chain participation.

### 5.6 FAST chain timing (status-start anchor)

FAST Sync/Bulk Read inverts the Plain shape: the whole chain reply is ONE Status packet in the official per-block layout — slot 0 emits the header plus its block, and every block (slot 0 included) ends with the CUMULATIVE packet CRC, the chain-state checkpoint the next device picks up. There are no per-slot frames for the skip-exhaust rule to observe. Slots k > 0 instead anchor on the observed start of that single Status packet:

    deadline_slot_k = status_start_tick + bytes_before(k) × byte_time

- **Slot 0** stays a single-target reply: `packet_end + RDT` per spec, exactly as §5.2.
- **Slots k > 0** carry no RDT term (RDT is single-target-reply-only) and no IDLE floor. At `send_slot` the reply is encoded and *deferred*: the reply gate parks a status-start wait and the RxDma provider opens the per-byte RXNE wake window (USART1 RXNEIE on V006 — wake-only; the tick comes from the RX ring byte cursor, never from wake-entry time, per `[[no_idle_timing]]`). The first wake with a byte past the Status packet's wire cursor resolves the anchor — `status_start_tick` is back-projected from the published byte count ([dxl-hw-timed-transport.md §8.2](dxl-hw-timed-transport.md)) — and arms the hardware TX kickoff; from that point the wire start is locked in regardless of CPU state ([dxl-hw-timed-transport.md §5](dxl-hw-timed-transport.md)). Every successor slot also starts the checkpoint pickup off the same anchor; only a far-horizon deadline (low baud, long predecessor) defers the hardware arm to the pickup's wake body via the scheduler's stash + commit.

Because every osc chip in the chain computes its deadline from the SAME observed status start, the response grid is coherent chain-wide — contiguity by construction at any baud, including RDT = 0, and slot 0's chip-side turnaround (the dominant real-world unknown, ~50 µs/servo on measured MX chains [1]) is absorbed by observation rather than assumed by formula. The design assumes grid-keeping (osc) servos in the chain; a mixed chain with slow third-party servos mid-chain should use Plain Sync/Bulk Read, whose per-predecessor skip-exhaust rule tolerates arbitrary turnaround.

Failure shape mirrors §5.3's Plain contract at the chain level:

- **Silent slot 0** → the Status packet never starts → no slot k > 0 has an anchor → the whole chain stays silent. Nothing was armed; nothing needs cleanup.
- **Silent middle slot** → the anchor already exists, so later slots still fire on schedule; the frame arrives with a hole, the starved pickup ships its placeholder CRC (one `crc_patch_deadline_miss`), and host-side per-block validation localizes the dead slot. Host retries.
- **Stale wake traffic.** A parked wait survives a dead chain until new bytes arrive. Those bytes are the host's retry — same `FF FF FD 00` preamble shape as the awaited reply — so the wake path bounds acceptance by a staleness window: `latest_start = packet_end + effective_RDT + slack` (slack ≈ 500 µs, covering legitimate slot-0 turnaround while staying far under host retry timeouts). A stamp past the window drops the parked slot instead of scheduling a TX into the host's instruction, and the window closes.
- **Ring ownership.** A parked successor wait owns the Status bytes the same way the armed pickup does (§6): `poll()` gates on it at entry and the in-flight poll stops at the event that parked it, so the parser never consumes the window bytes the pipeline owns. The per-byte wake is not a poll and proceeds regardless; the staleness drop un-gates the parser on the next wire traffic.

## 6. Fast successor chain-CRC checkpoint pickup

Under the official Fast layout every block ends with the cumulative packet CRC — the running chain state, checkpointed on the wire after each device. A successor slot never folds the predecessor window: its wake body reads the window's trailing two bytes (the predecessor's checkpoint) directly off the RX DMA ring, seeds the chain state from their value, extends over those same two bytes and the chip's own reply bytes, and patches the reply's trailing CRC slot. One SysTick wake just before the checkpoint lands (plus O(1) ring drains for windows deeper than half the ring), no fire-time CPU site, no per-byte fold — see [dxl-hw-timed-transport.md §10.6](dxl-hw-timed-transport.md) for the full path and the patch-vs-DMA budget. No event emission, no per-byte tick. NDTR supplies the byte accounting; nothing in the pickup path needs a tick.

**RX-tail ownership during the window.** Three consumers can advance the RX ring tail: the parser drain, the universal byte-skip (§5.2), and the pickup pipeline. While the pipeline is armed they are mutually exclusive — the checkpoint's position is pure cursor arithmetic from the observed status start, and any parser-side or skip-driven advance during the window would desynchronize it (and could mis-parse a First header whose LENGTH advertises the whole chain). The pipeline owns the tail for the duration of the window, bracketed by two events:

- **Arm** — at the status-start observation (`send_slot` for a successor parks the wait; the per-byte wake resolves it — §5.6). Between `send_slot` and the observation the parked wait already owns the ring tail via the `poll()` gate, so the Status packet's leading bytes stay unparsed. The in-flight `poll()` call that parked the wait yields without draining further bytes. **There is no edge channel to mask** — the fold's RX-tail ownership is enforced by the poll gates alone (in the hardware-RX draft this bracket masked an edge-capture channel's HT/TC; that machinery is deleted).
- **Release** — the pickup finalizes naturally (checkpoint read, CRC patched, active cleared; window bytes consumed raw up to its end) or cancels at `on_tx_complete` if a silent predecessor stalled the chain (matching the universal-skip clear in §5.3; the hardware kickoff fired regardless, so TC always comes). The cancel path disposes of the window's published bytes raw (`release_window`) so the parser resumes at the wire frontier — never inside the stale chain frame. Both transitions re-open the tail to the parser, and the next poll re-enters normally.

Between arm and release, USART1 IDLE and DMA1_CH5 HT/TC continue to fire — they cannot be masked without losing TX-half awareness — and may re-trigger `poll()`. Each such entry checks whether the pipeline is armed and returns immediately without touching the parser or the tail cursor.

The patch window — `patch_crc` landing before the TX DMA reads `tx_buf[len−2..len]` — is `(tx_len − 2) × byte_time` past the wire start. The checkpoint publishes ~one byte-time BEFORE the fire, and the pickup completes in a fraction of a byte-time, so the patch beats the read by construction even for the shortest reply at 3M. A starved pickup (silent predecessor) surfaces as one `crc_patch_deadline_miss`; the frame ships with its placeholder CRC, host-side per-block validation localizes the dead slot, and the host retries.

## 7. Resource shape

### 7.1 CPU

Savings are chain-deployment dependent — and that is the design point. DXL deployments at scale are buses; the chip's interest in the bus is asymmetric (~one instruction per cycle plus zero interest in N−1 of every N foreign Status frames in a chain reply). At a single-servo bench the savings are nominal. At a populated chain they scale linearly with `N`. The deleted edge subsystem cost this on top: up to ~55% CPU classifying edges during sustained 3M RX, spent on precision no consumer used.

| Scenario | Parser work | Persisted timing scalars |
| --- | --- | --- |
| Single-servo instruction → own reply | Parser drains the instruction; packet-end stamped at Crc from the drain stash | drain stash, packet-end tick, drift accumulator |
| N-servo Plain Sync / Bulk Read, in-chain receive, slots k > 0 | Parser drains only the instruction and each foreign packet's header — foreign bodies are byte-skipped past the ring tail (universal mechanic, §3) | drift accumulator, `predecessor_id` (u8), `chain_pending` (bool); universal-skip state (`skip_remaining: u8`, `current_skip_id: u8`) — no fire-side ticks |
| N-servo Plain Sync / Bulk Read, slot 0 | Same as single-servo instruction → own reply; slot 0 takes the single-target path at `packet_end + RDT` | Same as single-servo |
| N-servo Fast Sync / Bulk Read, successor slot | Parser drains the instruction; the predecessor window is consumed raw by the pickup, no parser walk of the body | Single-servo state + pickup window state |

The RX side keeps no per-byte tick and no derived ring; timing collapses to the drain stash plus per-consumer scalars.

### 7.2 Memory

| Region | Hardware-RX draft | Streaming RX (shipped) |
| --- | --- | --- |
| RX ring | 64 | 64–128 |
| ET ring | 256 | — |
| BT ring | 128 | — |
| Parser scratch | 256 | — |
| Timing scalars | (in rings) | ~72 |
| **Transport-owned total** | **704 B** | **~320 B** |

Three structural collapses. The edge-timestamp rings (ET + BT, 384 B) are gone — no per-byte timing means no timestamp axis. The byte-time ring's derived storage collapses to the two-field drain stash plus per-consumer scalars; per-packet timing state fits in ~20 bytes, and chain-participation state is ~6 bytes regardless of chain depth, carrying no fire-side timestamps at all. Parser scratch collapses entirely: payload data for Writes flows as offset/size ranges directly into the control-table staging buffer — which already exists for validate-then-commit semantics — and parser state is ~32 bytes of scalars.

~380 B freed against the hardware-RX draft on an 8 KB chip. Modest in absolute terms; structurally clean because every buffer dropped was duplicating storage that exists elsewhere or reconstructing precision no consumer spends.

## 8. Verification points

The following items require bench measurement or implementation-time decisions on V006.

- **Packet-end estimate quality across drain flavor.** ByteBatch drains stamp near-exact (min wire excess +0.17 µs at 3M); IDLE drains carry the one-frame back-date. Confirm the median tail (+5.4 µs) is same-priority poll blocking, not an estimator bias, and that no realistic drain sequence leaves the stash stale at `Crc`.
- **FAST middle-slot bias.** FAST k > 0 middle slots show a ~+7 µs median shift in the status-start-anchored grid at 3M — likely the RXNE-wake entry offset feeding `status_start_tick`. Clean 50/50 today; tightening target, not a correctness bug (tracked in [dxl-hw-timed-transport.md §12](dxl-hw-timed-transport.md)).
- **Resync event semantics in the streaming surface.** The parser already resyncs internally; surfacing the existing resync as a typed event must not change FSM behavior. Confirm via parser unit tests.
- **Instruction-flag ordering for drift.** The span emitters gate on the instruction flag; confirm it is set at the Instruction-Header event before any body span pairs and cleared at the matching packet boundary (`Event::Crc(_)`, `PollEvent::SkipComplete`, `Event::Resync(_)`), so foreign-Status bytes never feed the accumulator.
- **Parser-drain coverage across triggers.** The chain-start path relies on the parser advancing on all of USART1 IDLE, DMA1_CH5 HT, and DMA1_CH5 TC, each handler's drain promptly observing the predecessor's skip-exhaust as bytes cross the ring tail. Confirm the runtime routes the parser into each handler with consistent ordering.
- **Parser-off window after own reply.** The chip can stop calling poll after its own reply's TC; IDLE-driven re-entry on the next instruction must reliably resync at the next header. Verify on bench.
- **Outer CRC reset rule.** The drop-and-reset pattern is safe for top-level foreign packets but unsafe inside an outer Sync Write or Bulk Write — the outer CRC accumulator must be preserved end-to-end. Never reset the parser while inside an outer chain instruction; gate payload handling at the slot level, feed all bytes uniformly.
- **Predecessor-ID resolution shape.** Slot IDs arrive as a stream of per-slot demarcation events following the chain-instruction header. The chip walks the list tracking the most-recently-seen ID; `predecessor_id` resolves when the chip's own ID arrives. Three cases: own-ID-first → chain mechanic does not apply, chip takes the single-target reply path; own-ID-not-first → `predecessor_id = previous_seen_id`; own-ID-absent → chain-pending clears at end-of-param-list.
- **RXNEIE reason-set coexistence.** The FAST k > 0 wait and the drift window share the one RXNEIE bit through `RxWakeGate`'s OR-edge toggling. Verify a FAST exchange opening and resolving over an open drift window never toggles the provider, and that a batch close closes the window even from inside a drain-ISR handler.
- **SysTick CMP ownership.** SysTick CMP is owned exclusively by the FAST successor pickup; the Plain chain path holds no SysTick state. Verify the runtime arms and disarms cleanly around each FAST request, with no stale arm surviving a Plain request that interleaves.
- **RX ring lap under long Write polls.** A >32 B Write poll that overruns the half-period publish deadline can lap a 64 B ring (`[[rx_ring_lap_bug]]`). Verify the shipped ring depth + lap-accurate `on_publish` + poll-cost budget hold at 3M under the worst Write.
- **State-scoping discipline.** Each persisted scalar has a deliberate owner — packet-end tick for the reply scheduler, drain stash for the codec. Chain-participation state owns no tick scalars. Each lifetime must be bounded by its consumer's done-condition so stale values cannot poison subsequent packets.

## 9. Summary

The design consumes the RX DMA ring with a streaming, event-driven byte parser and times replies from a **software packet-end estimate** — a drain-flavor-corrected WireClock reading taken at drain-ISR entry, resolved at the parser's `Crc` event and discarded after use. No edge parser, no derived byte-time ring, no per-byte tick: the three timing consumers (reply deadline, FAST status-start, HSI drift) each meet their tolerance from a coarse reading, so the per-byte edge-timestamp subsystem an earlier draft specified is deleted. Foreign packets — in-chain predecessors, foreign single-target packets — are consumed by a universal byte-skip that keeps the parser's view of the ring tail synchronized with the wire. Plain Sync / Bulk Read chain timing splits cleanly: slot 0 inherits the single-target reply path (`packet_end + RDT`) through the hardware TX kickoff; slots k > 0 reduce to a single piece of state, the immediate predecessor's ID, and start the wire in-handler under whichever of USART1 IDLE, DMA1_CH5 HT, or DMA1_CH5 TC observes that ID's byte-skip exhaust — nothing scheduled, so nothing to cancel on a silent predecessor. FAST k > 0 slots anchor on a byte-count estimate of the chain Status packet's start, resolved at a per-byte RXNE wake, so the whole chain-wide grid keys off one physical reading. HSI drift samples `(Δticks, Δbytes)` NDTR/byte-count spans — same-flavor, instruction-only, gated — with an RXNE window restoring sampling for isolated short packets. The Fast successor chain-CRC pipeline shrinks to a checkpoint pickup off the predecessor's trailing CRC bytes, O(own reply) CPU, its RX-tail ownership enforced by the poll gates alone now that no edge channel exists. Transport-owned RAM drops to ~320 B; CPU during foreign-packet handling scales with bytes the chip actually consumes rather than bus traffic, and the deleted edge classifier's up-to-55% RX overhead returns to the motor loop. The trigger model (USART1 IDLE, DMA1_CH5 HT/TC, the per-byte RXNE wake window) and the all-High PFIC priority allocation are the transport's timing surface; the FAST successor pipeline rides a single SysTick wake.

## References

[1] Bestmann, M.; Güldenstein, J.; Zhang, J. *High-Frequency Multi Bus Servo and Sensor Communication Using the Dynamixel Protocol*. RoboCup 2019, Hamburg Bit-Bots, University of Hamburg. <https://2019.robocup.org/downloads/program/BestmannEtAl2019.pdf>
