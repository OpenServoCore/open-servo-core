# OSC Adapter Link

The normative specification of the link between a client (the `osc` CLI,
osc-web, or anyone's own program) and an osc adapter: the device that owns
the servo bus and runs every bus protocol deadline itself. A client never
touches the wire. It sends requests over a reliable ordered byte pipe, and
the adapter answers with records.

The servo bus speaks two protocols, and the adapter speaks both: the
osc-native protocol (`docs/osc-native-protocol.md`) to running servos, and
tinyboot to servos waiting in their bootloader. Exactly one is live at a
time (sec 5). The adapter enforces that, not the client, so a client
written by anyone in any language cannot mix the two on the wire.

## 1. Goals and non-goals

1. **One envelope.** Every request has the same header, and exactly one
   final record answers it. A client needs one correlation rule.
2. **The adapter owns the wire.** Framing, CRC, breaks, reply windows and
   pacing live in the adapter. A request names what to exchange, never how.
3. **Self-describing adapters.** INFO reports the features the adapter
   has and its size limits. A client never probes for
   a feature by trying it.
4. **Carrier-agnostic records.** USB bulk is the first carrier, not the
   definition (sec 8). Records are length-prefixed, so transfer boundaries
   carry no meaning.

Non-goals: more than one client per adapter at a time; compatibility
between a client and an adapter built from different trees before 1.0.

## 2. Sessions

A session is one client's ownership of the pipe. It opens with a carrier
event (sec 8.2 for USB) and ends when the next one opens. Opening a session:

- drops every record the adapter had not yet delivered and any request it
  had only partly received;
- cancels a bus request still running: the adapter stops driving the wire
  and collecting replies, and reports nothing. The new session's first bus
  request is accepted at once, but its frame waits until whatever the old
  exchange still draws from the bus has landed: the rest of the old reply
  window, re-armed by each byte as the old await would have, then 64
  byte-times of silence (protocol sec 3.4);
- returns the bus to osc mode (sec 5), at its current rate when that is an
  osc rate, else at 500000.

The bus rate, the response deadline and the rails persist across sessions:
they belong to the bus, not to the client. INFO reports them (sec 4.1).

The first request of a session is `GET_INFO`. Because the open dropped the
old records, its answer is the first record the client reads.

## 3. Records

Every record, either direction, is `len(2 LE) kind(1) seq(2 LE) body`.
`len` counts every byte after itself. All multi-byte fields are
little-endian.

- **Requests** (client to adapter) use kinds `0x00..=0x7F`.
- **Responses** (adapter to client) use kinds `0x80..=0xFF`.
- **`seq`** is chosen by the client and echoed in every response to that
  request. `0xFFFF` is reserved for responses no request owns (sec 3.3).

The largest request and the largest response are reported in INFO
(sec 4.1). A request longer than the adapter's limit is a framing breach.

### 3.1 Request classes

- **Immediate requests** are answered at once, whether or not the bus is
  busy: `GET_INFO`, `RAILS`, `EDGE_DRAIN`, `CAPTURE_RESET`.
- **Bus requests** own the bus until their `DONE`. One is outstanding at a
  time. Another bus request sent meanwhile answers `ERROR BUSY`. Every other
  kind is a bus request.

A client may send immediate requests while a bus request is outstanding;
the responses interleave, told apart by `seq`.

### 3.2 Responses

| Kind | Record | Body |
|---|---|---|
| `0x80` | `FRAME` | `slot(1) frame` |
| `0x81` | `DONE` | `kind(1) result` |
| `0x82` | `ERROR` | `kind(1) code(1)` |

The correlation rule: every request draws exactly one `DONE` or one
`ERROR`, and nothing for that `seq` follows it. A bus exchange may stream
`FRAME`s first.

- **`FRAME`** carries one clean frame received from the bus: CRC checked,
  framing stripped. `slot` is the reply's position: 0 for a single reply,
  the list position in a group read chain, the arrival order under collect,
  a wrapping arrival index in a telemetry stream. The frame's layout depends
  on the protocol (sec 6.1, sec 7.1).
- **`DONE`** echoes the request kind, then the kind's result (sec 4 to 7).
- **`ERROR`** echoes the request kind, then a code. Nothing happened on the
  wire, except as `BUSY` notes.

| Code | Name | Meaning |
|---|---|---|
| `0x00` | `MALFORMED` | The body does not parse: short, extra bytes, a reserved bit set, a window past sec 3.4's limit. |
| `0x01` | `UNKNOWN_KIND` | No such request kind. |
| `0x02` | `UNSUPPORTED` | The kind exists, but this adapter lacks the feature (INFO `features`). |
| `0x03` | `WRONG_MODE` | The kind is not valid in the bus's current mode (sec 5). |
| `0x04` | `BUSY` | A bus request of this session is outstanding. |
| `0x05` | `BAD_RATE` | The mode does not define the rate, or the adapter cannot produce it. |
| `0x06` | `BAD_ID` | osc: the id is not legal for the instruction. |
| `0x07` | `BAD_INST` | osc: the instruction byte is not legal. |
| `0x08` | `BAD_PAYLOAD` | osc: the payload does not fit the instruction's shape. |
| `0x09` | `TOO_LONG` | The frame would exceed the protocol's or the adapter's limit. |
| `0x0A` | `FRAMING` | A record's `len` broke the limits (sec 3.3). |

### 3.3 Framing breaches

A length-prefixed stream has no resync point. When a request's `len` is
below 3 (no room for `kind` and `seq`) or above the adapter's limit, the
adapter drops everything it has received, answers `ERROR` with `seq`
`0xFFFF`, kind `0xFF` (no request kind) and code `FRAMING`, and ignores
the pipe until the next session opens. The client must open a new session.

### 3.4 Ticks

Times in results are raw adapter ticks, a free-running `u32` that wraps.
INFO reports `ticks_per_us` once, so records never convert. A tick in a
result names a moment; the difference of two ticks is a span.

A `window_us` in a request spans less than 2^31 ticks: the adapter tells
a deadline from a moment already past by the half of the tick range ahead
of it. A longer window answers `MALFORMED`.

## 4. Adapter requests (`0x0_`)

| Kind | Request | Body | `DONE` result |
|---|---|---|---|
| `0x00` | `GET_INFO` | - | INFO (sec 4.1) |
| `0x01` | `RAILS` | `state(1) mask(1)` | `state(1)` |
| `0x02` | `ADAPTER_IAP` | - | - |

### 4.1 INFO

`features(2) ticks_per_us(4) max_request(2) max_response(2)
max_tb_payload(2) mode(1) bps(4) deadline_us(2) diag(30)`

- **`features`**: bit 0 `INSTRUMENT` (sec 9), bit 1 `RAILS`, bit 2
  `ADAPTER_IAP`, bit 3 `TINYBOOT` (sec 7). Other bits are 0. osc mode is
  always present.
- **`max_request`, `max_response`**: the largest record each way, in bytes,
  `len` included.
- **`max_tb_payload`**: the largest tinyboot data the adapter carries in
  one frame, either direction. 0 without `TINYBOOT`.
- **`mode`, `bps`, `deadline_us`**: the bus state this session inherited
  (sec 2): the mode and rate (sec 5) and the response deadline (sec 6.4).
  A client that knows the fleet's rate from INFO need not sweep for it.
- **`diag`**: adapter health, `reset(1) phase(1) crash_seq(4) mcause(4)
  mepc(4) mtval(4) hse_fail(4) resets(4) uptime_ms(4)`.
  - `reset`: the reset-cause flags read at the adapter's boot (bit 0 pin,
    1 power-on, 2 software, 3 independent watchdog, 4 window watchdog,
    5 low power).
  - `phase`: where the adapter's main loop stood when that reset hit.
  - `crash_seq`, `mcause`, `mepc`, `mtval`: the last exception the
    adapter trapped, kept across resets. `crash_seq` 0 means none on
    record.
  - `hse_fail`: crystal start failures. `resets`: resets since power-on.
  - `uptime_ms`: live, wraps at about 49 days. Shorter than the time since
    the client last saw the adapter means it restarted.

  An adapter without these facts (a simulator) reports zeros.

### 4.2 RAILS

Drives the adapter's target power rails, where it has them (`RAILS`
feature). Bit 0 is 3V3, bit 1 is 5V. Only bits set in `mask` change;
`mask` 0 reads the rails without driving them. The result is the rails'
state after the change.

### 4.3 ADAPTER_IAP

Hands the adapter to its resident loader to replace the adapter's own
firmware (`ADAPTER_IAP` feature). A bus request, so it waits for the bus
to be idle. The `DONE` is the last record of the session; the adapter then
leaves USB and comes back as its loader. This is the adapter's own update,
unrelated to updating a servo (sec 7).

## 5. Bus modes (`0x1_`)

| Kind | Request | Body | `DONE` result |
|---|---|---|---|
| `0x10` | `SET_BUS` | `mode(1) bps(4)` | `mode(1) bps(4)` |

The bus is in one mode at a time. Mode 0 is osc, mode 1 is tinyboot
(`TINYBOOT` feature). `SET_BUS` always names both the mode and the rate,
so the rate after a switch is never a leftover.

- **osc rates**: 500000, 1000000, 2000000, 3000000 (protocol sec 2).
- **tinyboot rates**: tinyboot-protocol's rate catalog, less any rate the
  adapter cannot produce within 2%.

Any other rate answers `BAD_RATE`. A switch is a bus request, so it waits
for the bus to be idle. After every `SET_BUS` the adapter keeps the wire
quiet for 64 byte-times at the new rate before the next frame (protocol
sec 3.4), so every receiver's framer has settled.

What each mode accepts:

| Kinds | osc mode | tinyboot mode |
|---|---|---|
| `0x0_` adapter, `0x1_` bus | yes | yes |
| `0x2_` osc | yes | `WRONG_MODE` |
| `0x30` `TB_EXCHANGE` | `WRONG_MODE` | yes |
| `0x60..=0x64` wire ops | yes | `WRONG_MODE` |
| `0x65`, `0x66` edge capture | yes | yes |

The mode is the adapter's receive framer and its promise about the wire.
In osc mode every frame follows a break and replies are break-framed. In
tinyboot mode the adapter never puts a break or a pulse on the wire, and
replies are found by their sync word (sec 7.1). The mode keeps the wire
tinyboot-only whatever the client sends, which is why the wire ops, which
all start with a break, are `WRONG_MODE` in tinyboot mode.

Tinyboot mode assumes no osc app is listening: the host moves every servo
on the wire to its bootloader before the first `TB_EXCHANGE` and resets
them all to their apps after the last. An app servo frames only after a
break (transport sec 7), but breakless bytes still land in its receive
ring, and its framer can park on them at the next break, so its next reply
comes late by whole starve horizons (protocol sec 3.4). A tinyboot rate
below the app's holds lows long enough to fire its break detector, and at
19200 and below a `0x00` is long enough to read as a rescue pulse
(protocol sec 9.1). Every bootloader on the wire is addressable and silent until
selected, so the whole fleet can wait in tinyboot together.

A new session returns the bus to osc mode (sec 2). A client that dies
mid-update leaves no adapter stuck in tinyboot mode; the servos wait in
their bootloaders for the next session to find them.

## 6. osc requests (`0x2_`)

| Kind | Request | Body | `DONE` result |
|---|---|---|---|
| `0x20` | `OSC_EXCHANGE` | `flags(1) id(1) inst(1) window_us(4) payload` | exchange (sec 6.2) |
| `0x21` | `OSC_RESCUE` | - | exchange (sec 6.2) |
| `0x22` | `OSC_DEADLINE` | `us(2)` | - |

### 6.1 OSC_EXCHANGE

One osc-native instruction, any opcode, management included. The adapter
derives the instruction's shape from `id`, `inst` and `payload` (protocol
sec 5) and answers `BAD_ID`, `BAD_INST`, `BAD_PAYLOAD` or `TOO_LONG`
before anything touches the wire. It then sends the break and the frame,
and collects what the shape owes:

- **no reply** (`NOREPLY`, a silent broadcast, a group write, a CAL train):
  `DONE` once the frame (or the train) has left the wire;
- **one reply**, or **a chain** of them for a group read (protocol sec 6);
- **collect until quiet** for a broadcast ENUM (protocol sec 9.2).

`flags` bit 0 is `STREAM`: the exchange arms a servo telemetry burst. After
its own reply, the adapter keeps collecting status frames until one is
marked last, inside one window of `window_us`. Without `STREAM`,
`window_us` must be 0 and the adapter sizes every window itself from the
response deadline, the reply's wire time and the instruction's flash
allowance. Other `flags` bits are reserved.

A `FRAME` for osc is `id(1) inst(1) payload`: every status frame received,
CRC checked.

### 6.2 Exchange results

`outcome(1) slot(1) tick(4) frames(2) garble(2) flags(1) laps(2)`

- **`outcome`**: 0 `SENT` (no reply owed, the wire is done), 1 `COMPLETE`
  (every owed reply arrived, or collect went quiet), 2 `TIMEOUT` (`slot`
  names the reply that never came).
- **`tick`**: when the exchange finished.
- **`frames`**: clean frames delivered.
- **`garble`**: received bytes that formed no clean frame (noise, CRC
  failures, colliding replies).
- **`flags`** bit 0: garble arrived after the last clean frame, the trailing
  energy the ENUM walk reads as a collision (protocol sec 9.2).
- **`laps`**: whole receive buffers lost before the adapter read them. The
  garble after a lap is its wreckage, not the wire's.

Any garble makes the adapter keep the wire quiet for 64 byte-times before
the next frame (protocol sec 3.4).

### 6.3 OSC_RESCUE

Holds the wire low for 1 ms, the rescue pulse (protocol sec 9.1), then
moves the bus to 500000 in the same request: every servo that saw the
pulse is at that rate now. The rate change carries the same 64 byte-time
quiet as `SET_BUS` (sec 5). The result is an exchange result with outcome
`SENT`.

### 6.4 OSC_DEADLINE

Sets the response deadline the adapter sizes its reply windows from. It
must match the servos' RESPONSE_DEADLINE register (protocol sec 5.4);
changing the servos is the client's job. Default 1000 µs.

## 7. tinyboot requests (`0x3_`)

| Kind | Request | Body | `DONE` result |
|---|---|---|---|
| `0x30` | `TB_EXCHANGE` | `expect(1) window_us(4) cmd(1) addr(3) flags(1) data` | exchange (sec 6.2) |

The adapter frames tinyboot and does nothing else with it. It adds the
sync word, sets the status to request, adds the length and the CRC, and
sends the frame with no break. It never reads `cmd`, so a tinyboot command
the adapter has never heard of passes through unchanged. Data longer than
`max_tb_payload` answers `TOO_LONG`.

The frame layout the adapter depends on is this document's, not
tinyboot-protocol's version: `AA 55 cmd(1) status(1) addr(3) flags(1)
len(2 LE) data crc(2 LE)`, the CRC being CRC-16/CCITT (poly `0x1021`,
init `0xFFFF`) over everything before it. A tinyboot release that keeps
this layout needs no adapter change, whatever it adds; one that changes
it changes this document and the adapter with it. Which tinyboot versions
a device and an image suit each other is the client's question, answered
by the device's own Info reply.

`expect` names the replies the command draws:

- **0, none**: `DONE` once the frame has left the wire. `window_us` must be
  0.
- **1, one**: `DONE` at the first clean reply, or `TIMEOUT` at the end of
  `window_us`.
- **2, collect**: every clean reply until `window_us` ends, then `COMPLETE`.
  For a Select that many devices may answer in their own slots.

The client sizes `window_us`: it knows how long the command takes on the
device (an erase runs milliseconds), the adapter does not.

### 7.1 tinyboot frames

A `FRAME` for tinyboot is `cmd(1) status(1) addr(3) flags(1) data`: the
reply with its sync word, length and CRC stripped.

The adapter finds replies by their sync word, there being no break: it
scans the received bytes for `AA 55`, reads the header, bounds `len` by
`max_tb_payload` (a false sync inside garble would otherwise swallow the
rest of the window), reads the data and checks the CRC. Every byte outside
a clean frame is garble, as is a clean frame whose status is request:
no device sends one, and the adapter does not hear its own (its receiver
is off while it transmits), so such a frame is another talker on the wire.

The exchange result is the osc one (sec 6.2): `frames` counts tinyboot
frames. The Select walk reads any garble at all as a collision: two
replies are each 24 bytes and start at most 15 byte-times apart, so they
always overlap, and a collision shows as no clean frame and some garble.

## 8. USB carrier

### 8.1 Device

A vendor-class interface 0 with one bulk pair: EP1 OUT carries requests,
EP1 IN carries responses. High speed when the host supports it (512-byte
packets), full speed otherwise (64). A record may span packets, and a
packet may carry several records.

Records go out in bursts of whatever the adapter has queued, and every
burst ends in a short packet: when the last packet of a burst is a full
one, a zero-length packet follows it. Without that a burst of exactly one
packet's worth of records sits in the host's read until the next response
arrives, which a client sees as a reply that never came.

The osc adapter is 1209:0001 (pid.codes' test PID, before release). Its BOS
descriptor carries MS OS 2.0 descriptors, so Windows binds WinUSB with no
driver install.

### 8.2 Opening a session

A vendor OUT control request on EP0 opens the session (sec 2):
bmRequestType `0x41` (vendor, interface), bRequest `0x01`, wValue 0,
wIndex 0, no data. The client sends it after claiming the interface and
before any bulk traffic.

A bus reset and SET_CONFIGURATION open a session too, but a client cannot
rely on them: WebUSB sends no SET_CONFIGURATION when the configuration is
already active, so a reopened page would inherit the previous page's
records. A control request rides EP0, outside the bulk stream, so whatever
the stream still holds cannot get in its way. An adapter that stalls the
request predates sessions altogether.

## 9. Instrument requests (`0x6_`)

Wire-level test equipment for the bench (`INSTRUMENT` feature), outside
the protocols on purpose: a malformed frame is often the point. Wire ops
are bus requests; edge capture requests are immediate.

| Kind | Request | Body | `DONE` result |
|---|---|---|---|
| `0x60` | `WIRE_SEND` | `bytes` | `tick(4)` |
| `0x61` | `WIRE_BURST` | `[len(1) bytes]+` | `tick(4)` |
| `0x62` | `WIRE_PULSE` | `us(2)` | `tick(4)` |
| `0x63` | `WIRE_TRAIN` | `gap_us(2) breaks(1) announce` | `tick(4)` |
| `0x64` | `WIRE_BAUD` | `bps(4)` | - |
| `0x65` | `EDGE_DRAIN` | `max(1)` | edges |
| `0x66` | `CAPTURE_RESET` | - | - |

- **`WIRE_SEND`**: one break, then the bytes as given. No shape check.
- **`WIRE_BURST`**: frames back to back, each after its own break.
- **`WIRE_PULSE`**: the wire held low for `us`, any width. A pulse of
  300 µs or more is a real rescue to any servo on the bus.
- **`WIRE_TRAIN`**: an announce frame, then `breaks` bare breaks every
  `gap_us`. The gap need not match the announce: that is how a lying train
  is injected.
- **`WIRE_BAUD`**: the adapter's UART at any rate, off the catalog
  included, to detune the host. The adapter's own timing stays on the
  nearest catalog rate.
- **`tick`**: the moment the adapter released the wire.

`EDGE_DRAIN`'s result is `flags(1) now(4) fall_n(1) rise_n(1)
falls(fall_n x 2) rises(rise_n x 2)`: up to `max` captured edges of each
polarity, in capture order, as the low 16 bits of the tick. `now` is the
drain's own 32-bit tick, the anchor to unwrap them against. `flags` bit 0
is overflow: an edge buffer lapped since the last reset, sticky until
`CAPTURE_RESET`. Drain at least every few milliseconds of wire activity.
Edge capture works in both modes, so it can time a bootloader's bits too.
