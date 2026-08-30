# PIPE_WRITE Wire Protocol

Sliding-window, selectively-acknowledged image transfer for OpenDisplay panels.
Opcodes **0x0080–0x0082**. Introduced in commit `bcfad8b`.

This document walks the protocol end to end: **negotiation → data transfer →
acknowledgement → error recovery → completion**. It is written against the
firmware in [src/display_service.cpp](../src/display_service.cpp),
[src/structs.h](../src/structs.h), [src/communication.cpp](../src/communication.cpp),
and [src/esp32_ble_callbacks.h](../src/esp32_ble_callbacks.h).

---

## 1. Overview & motivation

The legacy image path (`0x70/0x71/0x72`) is strictly in-order and stop-and-wait:
the client sends a chunk, waits for an ACK, sends the next. Over BLE, that
round-trip latency dominates transfer time.

PIPE_WRITE replaces the stop-and-wait loop with a **sliding window**. The client
may have up to *W* frames outstanding (unacknowledged) at once, and the device
acknowledges with a **QUIC-style selective ACK (SACK)** that reports the highest
sequence number seen plus a bitmask of which earlier frames arrived. This lets a
single lost frame be retransmitted without stalling the whole stream.

Key design point: **data streams directly to the panel controller — there is no
framebuffer.** Bytes for frame *k* are written to the display IC as soon as frame
*k* is accepted in order. Because the controller stream cannot skip a hole, a
small **reorder queue** holds out-of-order frames until the missing one arrives.

The legacy `0x70/0x71/0x72` (and partial `0x76`) paths are byte-for-byte
unchanged. A new client probes with `0x0080`; if it goes unanswered (old
firmware) the client falls back to legacy. Old clients are unaffected.

### Opcode family

| Opcode   | Name              | Direction     | Purpose                              |
|----------|-------------------|---------------|--------------------------------------|
| `0x0080` | PIPE_WRITE_START  | client→device | Negotiate window / cadence / frame   |
| `0x0081` | PIPE_WRITE_DATA   | client→device | Carry one sequenced payload frame     |
| `0x0082` | PIPE_WRITE_END    | client→device | Finalize, refresh, commit etag        |

The device replies on the same notify channel with status-prefixed responses
(`0x00…` success / SACK, `0xFF…` NACK) described below.

### Framing convention

The 2-byte **big-endian** opcode is stripped by the dispatcher
([communication.cpp:626-639](../src/communication.cpp)) before the handler runs.
Inside a handler, `data[0]` is therefore the first *post-opcode* byte and `len`
is the post-opcode payload length. All multi-byte fields **in the transfer body
are little-endian** — except the END etag, which mirrors legacy and is big-endian
(see §6).

> **Etag endianness contract (read this before "fixing" it).** The same uint32
> etag is carried in two byte orders on purpose, and both are matched by the
> firmware and every client:
>
> | Frame | Field | Byte order | Rationale |
> |-------|-------|-----------|-----------|
> | `0x0080` partial START extension | `old_etag` | **little-endian** | consistent with the rest of the LE pipe header (§2.1.1) |
> | `0x0082` END | `new_etag` | **big-endian** | reuses the legacy `0x72` finalizer + `parse_be_u32` verbatim (§6) |
> | legacy `0x76` partial START | `old_etag`/`new_etag` | **big-endian** | legacy layout, unchanged |
>
> So within a single pipe-partial exchange the inbound `old_etag` is LE and the
> outbound `new_etag` is BE. This *looks* like an endianness bug and is not — see
> the cross-repo validation in the HA-integration audit notes. Do not "harmonize"
> the two without a `PIPE_VERSION` bump: firmware `parse_be_u32`
> ([display_service.cpp:2906](../src/display_service.cpp)) and the manual LE decode
> at [display_service.cpp:2301-2302](../src/display_service.cpp) both have shipped
> clients depending on them (py-opendisplay `commands.py` `<IHHHH` / `>I`,
> opendisplay.org `ble-common.js`).

### Negotiated constants

Split across two files. The **protocol constants** — `PIPE_VERSION`,
`PIPE_MAX_FRAME`, `PIPE_FLAG_COMPRESSED`, `PIPE_FLAG_PARTIAL`, `PIPE_ACK_MASK_BITS`
— come from the vendored canonical header
[include/opendisplay_protocol.h:722-727](../include/opendisplay_protocol.h). The
**profile / window constants** — `PIPE_MAX_W`, `PIPE_MAX_N`, `PIPE_REORDER_SLOTS`,
`PIPE_REORDER_SLOT_SIZE`, and the `PIPE_SMALL_DRAM_WINDOW` split — remain in
[structs.h:104-132](../src/structs.h). Two profiles: the default and a
reduced profile (`PIPE_SMALL_DRAM_WINDOW`) used only by the classic-ESP32
`env:esp32-N4` build, whose static DRAM cannot hold the full queue.

| Constant                 | Default | Small-DRAM | Meaning                                    |
|--------------------------|---------|-----------|--------------------------------------------|
| `PIPE_VERSION`           | `0x01`  | `0x01`    | Protocol version                           |
| `PIPE_MAX_W`             | 32      | 16        | Device max window                          |
| `PIPE_MAX_N`             | 32      | 16        | Device max ACK cadence                     |
| `PIPE_MAX_FRAME`         | 244     | 244       | Device max frame size (HA ATT write ceiling)|
| `PIPE_ACK_MASK_BITS`     | 32      | 32        | SACK bitmask width                         |
| `PIPE_REORDER_SLOTS`     | 33      | 17        | Reorder queue depth (= max window + 1)     |
| `PIPE_REORDER_SLOT_SIZE` | 248     | 248       | Per-slot buffer bytes                      |
| `PIPE_FLAG_COMPRESSED`   | `0x01`  | `0x01`    | START flag bit0: payload is a zlib stream  |
| `PIPE_FLAG_PARTIAL`      | `0x02`  | `0x02`    | START flag bit1: partial-region refresh (§2.1.1, §9) |
| `PIPE_FLAG_SLOT_TARGET`  | `0x04`  | `0x04`    | START flag bit2: write to a PSRAM slot, not the panel (§10) — **LOCAL FORK DIVERGENCE, not upstream** |

---

## 2. Negotiation — PIPE_WRITE_START (0x0080)

One round trip establishes the whole session. Handler:
[handlePipeWriteStart, display_service.cpp:2268-2429](../src/display_service.cpp).

A new START **aborts any in-flight transfer** (of any family) and calls
`resetPipeWriteState()` before parsing.

### 2.1 Request (client → device)

10 bytes, little-endian, after the `00 80` opcode:

| Offset | Field              | Size | Notes                                             |
|--------|--------------------|------|---------------------------------------------------|
| 0      | `ver`              | 1    | Must equal `PIPE_VERSION` (`0x01`)                |
| 1      | `flags`            | 1    | bit0 = `PIPE_FLAG_COMPRESSED` (zlib); bit1 = `PIPE_FLAG_PARTIAL` (§2.1.1); other bits reserved |
| 2      | `req_w`            | 1    | Proposed window size                              |
| 3      | `req_n`            | 1    | Proposed ACK cadence (ack every N)                |
| 4–5    | `client_max_frame` | 2 LE | Proposed max frame size                           |
| 6–9    | `total_size`       | 4 LE | Decompressed panel byte total                     |

The guard is `if (len < 10)` — trailing bytes are tolerated for future fields
(the `PIPE_FLAG_PARTIAL` extension in §2.1.1 rides in exactly this trailing space).

### 2.1.1 Partial-region extension (`PIPE_FLAG_PARTIAL`)

When `flags` bit1 (`PIPE_FLAG_PARTIAL`, `0x02`) is set, the START carries a
**12-byte little-endian extension appended after `total_size`** (post-opcode
length 22, not 10). Parsed at
[display_service.cpp:2300-2306](../src/display_service.cpp):

| Offset | Field      | Size | Notes                                                     |
|--------|------------|------|-----------------------------------------------------------|
| 10–13  | `old_etag` | 4 LE | Etag currently on the panel; must be nonzero and equal `displayed_etag` |
| 14–15  | `x`        | 2 LE | Rectangle left; must be byte-aligned (`x & 7 == 0`)       |
| 16–17  | `y`        | 2 LE | Rectangle top                                             |
| 18–19  | `w`        | 2 LE | Rectangle width; byte-aligned (`w & 7 == 0`), nonzero, in-bounds |
| 20–21  | `h`        | 2 LE | Rectangle height; nonzero, in-bounds                     |

> **`old_etag` is little-endian here**, unlike the legacy `0x76` partial START
> (which packs the same field big-endian) and unlike the `0x0082` END `new_etag`
> (big-endian). This is deliberate — see the endianness contract box in §1. The
> guard is `if (partial && len < 22)` → NACK `0x01`.

For a partial START, `total_size` must equal `plane_size * 2`, where `plane_size =
calc_controller_plane_bytes(w, h)` — the two flat 1bpp controller planes (old then
new) that the `0x76` partial path also streams
([display_service.cpp:2332-2342](../src/display_service.cpp)). The partial
geometry, etag, and rectangle checks all run **before any hardware is touched**;
every partial-request NACK clears `displayed_etag` for parity with the `0x76`
handler. See §9 for the full partial data/END flow.

> **Historical bug (fixed in `a39082e`).** The handler is dispatched as
> `handlePipeWriteStart(data+2, len-2)`, so `len` is the 10-byte post-opcode
> length. The original guard required `len >= 12` (the full on-wire frame
> including the opcode), so **every valid request was rejected** with
> `sendPipeStartNack(0x01)` and clients silently fell back to legacy on every
> upload. The parse itself was correct; only the guard was wrong.

### 2.2 Negotiation rule (min of both sides)

The device computes effective values ([display_service.cpp:2344-2351](../src/display_service.cpp)):

- `W_eff  = clamp(min(req_w, PIPE_MAX_W), 1..)` — additionally clamped to ≤32
  when encryption is active (the ACK mask is 32 bits wide).
- `N_eff  = min(clamp(min(req_n, PIPE_MAX_N), 1..), W_eff)` — cadence never
  exceeds the window.
- `frame_eff = min(client_max_frame, PIPE_MAX_FRAME)` (≤ 244).

`total_size` must **exactly** match the device-computed geometry
(`directWriteComputeGeometry`), else NACK `0x03`.

### 2.3 Response (device → client)

8 bytes ([display_service.cpp:2401-2404](../src/display_service.cpp)):

```
00 80  VER  MAX_W  MAX_N  FRAME_LO FRAME_HI  FLAGS
```

| Offset | Value                         | Meaning                                    |
|--------|-------------------------------|--------------------------------------------|
| 0–1    | `00 80`                       | Success + opcode echo                       |
| 2      | `0x01`                        | Device protocol version                     |
| 3      | `PIPE_MAX_W`                  | Device max window                           |
| 4      | `PIPE_MAX_N`                  | Device max ACK cadence                      |
| 5–6    | `PIPE_MAX_FRAME` (244 → `F4 00`) | Device max frame, LE                     |
| 7      | flags — **bit0 = 1** always; **bit1 = 1** iff the START requested (and the device accepted) `PIPE_FLAG_PARTIAL` | bit0 = device buffers out-of-order (selective repeat); bit1 = partial-region refresh accepted |

A client that set `PIPE_FLAG_PARTIAL` in its START **must** verify bit1 is set in
the response; a device that lacks partial support (older firmware) echoes bit1 = 0,
and the client falls back to the legacy `0x76` partial path.

The response returns device **maxima**, not the effective values. The client
applies the same `min` rule to arrive at the identical `W_eff / N_eff /
frame_eff`. Both sides now agree without a second round trip.

> The response is sent **before** the panel is powered up
> ([display_service.cpp:2390-2428](../src/display_service.cpp)) so that slow
> panel init cannot exceed the client's 2-second probe timeout.

### 2.4 START NACK

`sendPipeStartNack(err)` → 4 bytes `FF 80 <err> 00`. No teardown is needed (the
geometry check is pure config math before any hardware is touched).

| `err`  | Cause                                                     |
|--------|-----------------------------------------------------------|
| `0x01` | Bad length (`len < 10`, `len < 22` when partial, or `len < 16` when slot-target) or version mismatch |
| `0x02` | Unsupported flag bits set (includes setting both `PIPE_FLAG_PARTIAL` and `PIPE_FLAG_SLOT_TARGET` — mutually exclusive) |
| `0x03` | `total_size` disagrees with device geometry (full: `directWriteComputeGeometry`; partial: `plane_size * 2`) |
| `0x04` | **Slot-target:** `slot_id` out of range for this board, or slot storage disabled here (`OD_SLOT_COUNT == 0`) (`SLOT_INVALID`, §10) — **LOCAL FORK DIVERGENCE** |
| `0x05` | **Partial:** `old_etag` is 0 or does not match the on-panel `displayed_etag` (`ETAG_MISMATCH`) |
| `0x06` | **Partial:** panel is not 1bpp, or is a seeed/IT8951 driver with no controller-plane mechanism (`PARTIAL_UNSUPPORTED`) |
| `0x07` | **Partial:** rectangle is empty, out of bounds, or `x`/`w` not byte-aligned (`RECT_INVALID`) |
| `0x08` | **Slot-target:** `total_size` exceeds this board's `OD_SLOT_SIZE_BYTES` (`SLOT_TOO_LARGE`, §10) — **LOCAL FORK DIVERGENCE** |

Codes `0x05`–`0x07` are partial-only and each **clears `displayed_etag`** before
NACKing, so a client that retries as a full upload sees a clean slate. `0x04`
and `0x08` are slot-target-only and do **not** touch `displayed_etag` (a
slot-target request never reads or writes it). Parsed by
py-opendisplay as `PIPE_START_NACK_ETAG_MISMATCH` / `PARTIAL_UNSUPPORTED` /
`RECT_INVALID` ([responses.py:281](../../py-opendisplay/src/opendisplay/protocol/responses.py)),
which drive its fallback ladder (see §9).

---

## 3. Data transfer — PIPE_WRITE_DATA (0x0081)

Handler: [handlePipeWriteData, display_service.cpp:2431-2520](../src/display_service.cpp).

### 3.1 Frame layout

After the `00 81` opcode:

| Offset | Field     | Size   | Notes                                    |
|--------|-----------|--------|------------------------------------------|
| 0      | `seq`     | 1      | Rolling sequence number, wraps mod 256    |
| 1…     | `payload` | ≤241   | Controller bytes (or zlib stream)         |

`plen = len - 1`. A frame carries at most `frame_eff - 3` payload bytes (the
2-byte opcode + 1-byte seq consume three — `PIPE_FRAME_OVERHEAD`), so at the
244-byte cap the payload is ≤241 bytes.

### 3.2 Sequence numbers and the window

- The transfer starts at `expected_seq = 0` and increments per in-order accept.
- `seq` is a `uint8_t`; it wraps 255 → 0. Distances use unsigned 8-bit
  subtraction so wrap is automatic:
  - `fwd  = (uint8_t)(seq - expected_seq)` — `0` = exactly in order; `1..W-1` =
    ahead, within window.
  - `back = (uint8_t)(expected_seq - seq)` — `≥1` = at/below expected (duplicate
    or stale).
- The window width is `W = W_eff`. Because a live window spans at most *W* < 33
  distinct seqs, `seq % PIPE_REORDER_SLOTS` indexes the reorder queue without
  collisions even across the mod-256 wrap.

### 3.3 The three cases

**(a) In order — `fwd == 0`** ([display_service.cpp:2443-2476](../src/display_service.cpp)):

1. `pipeConsumePayload()` streams the bytes straight to the panel controller
   (same machinery as legacy `0x71`: raw write, or zlib inflate, or gray4 plane
   split). A failure NACKs (`0x02` compressed / `0x03` uncompressed).
2. `expected_seq++`, counters advance, `highest_seen` updates.
3. **Drain the queue:** while the slot for the new `expected_seq` holds a
   matching frame, consume it, free the slot, `queued_count--`, and advance
   `expected_seq` again. This is how the stream catches up past a filled hole.
4. If the queue is now empty, `gap_open = false`.
5. Cadence: if `frames_since_ack >= N_eff`, send a SACK (§4).

**(b) Ahead, in window — `0 < fwd < W`** ([display_service.cpp:2478-2503](../src/display_service.cpp)):

This is the **pause point** — nothing past the hole reaches the controller. The
frame is `memcpy`'d into `pipeReorder[seq % SLOTS]`, `queued_count++`. If this
opens a new gap (`gap_open` was false), a SACK is sent **immediately** so the
sender learns which frame is missing (fast retransmit). While the gap stays open,
further out-of-order arrivals are rate-limited to one SACK per `N_eff` arrivals.
Queue overflow (`queued_count >= PIPE_REORDER_SLOTS`) NACKs `0x03` — a protocol
violation the sender's window rule should make impossible.

**(c) Duplicate / stale — `back <= W`** ([display_service.cpp:2507-2516](../src/display_service.cpp)):

The frame is discarded but a rate-limited SACK is sent so the sender re-learns
the receiver's position. Anything outside the window on both sides (`fwd >= W`
and `back > W`) is a protocol violation → NACK `0x04`.

### 3.4 ESP32 ingest ring (transport, not protocol)

On **both** targets (nRF since the Phase 3 transport work, ESP32 since the NimBLE
port) the BLE callback copies each command into a lock-free SPSC ring with
acquire/release atomics; the main loop drains up to `COMMAND_QUEUE_SIZE` per pass
and flushes responses **between** commands so small ACK cadences can't overflow
the response ring.

Depth is **derived**, not hardcoded: `COMMAND_QUEUE_SIZE = PIPE_MAX_W + 2`
(`src/command_queue.h`, with a `static_assert` enforcing it). The ring reserves
one slot to distinguish full from empty, so usable capacity is `SLOTS - 1`, and
the worst case it must absorb across an SPI stall is a full `PIPE_MAX_W` window
of DATA **plus END** — END carries no `seq`, sits outside the window's sequence
space, and is therefore not bounded by the sender's window rule. That gives 34
slots at `PIPE_MAX_W = 32`, and 18 on `env:esp32-N4`
(`PIPE_SMALL_DRAM_WINDOW`, `PIPE_MAX_W = 16`).

> This constant read a hardcoded `33` until 2026-07-27, which yielded 32 usable
> slots — one short of the "full window plus END" case it was documented to
> cover, so the END was rejected at exactly the stall it was sized for. Note that
> `PIPE_REORDER_SLOTS = W + 1` is *correct* for its own purpose (collision-free
> `seq % SLOTS` indexing); the two constants have different rationales and must
> not be kept equal by habit.

---

## 4. Acknowledgement — QUIC-style SACK

Builder: [pipeBuildAckPayload, display_service.cpp:2162-2174](../src/display_service.cpp).

### 4.1 Format

A data ACK is 7 bytes:

```
00 81  highest_seen  mask[0] mask[1] mask[2] mask[3]
```

- Byte 2 `highest_seen` — the highest seq received (accepted **or** queued),
  mod 256. If nothing has arrived yet, it reports `expected_seq - 1`.
  `highest_seen` is itself implicitly acknowledged.
- Bytes 3–6 — a **32-bit mask, little-endian**. Bit *i* (LSB first, i = 0..31)
  means **chunk `(highest_seen - 1 - i)` was received**. So bit0 = `hs−1`,
  bit1 = `hs−2`, …, bit31 = `hs−32`.

"Received" means either accepted in the in-order prefix *or* currently held in
the reorder queue ([pipeChunkReceived, display_service.cpp:2152-2158](../src/display_service.cpp)).
The accepted-prefix depth is bounded by `received_count` so the first 32 chunks
of a transfer never set phantom bits from mod-256 wrap. A zeroed bit is the
sender's cue to retransmit that seq.

### 4.2 When a SACK is sent

| Trigger                        | Behaviour                                          |
|--------------------------------|----------------------------------------------------|
| Cadence                        | Every `N_eff` in-order accepts                      |
| Gap opens                      | **Immediately** (fast retransmit)                   |
| Out-of-order / duplicate while gap open | Rate-limited: one per `N_eff` such arrivals |
| END / auto-complete            | Tail flush before the final result                  |

`sendPipeAck()` resets both the cadence counter (`frames_since_ack`) and the
gap-ACK rate-limit counter (`ooo_acks_since_gap`).

### 4.3 Worked example

Window 8, cadence 4. Frames 0,1,2 arrive, then 3 is lost, then 4,5 arrive:

- After 0,1,2 in order: `expected_seq = 3`. (No ACK yet — only 3 accepts, cadence
  is 4.)
- Frame 4 arrives, `fwd = 1` → queued, gap opens → **immediate SACK**
  `highest_seen = 4`, mask bit for seq 3 is **0** (missing), bit for seq 2 is 1.
  The sender sees the hole at 3.
- Frame 5 arrives, `fwd = 2` → queued; rate-limited, no new ACK yet.
- Sender retransmits 3. It arrives `fwd = 0` → written, then the drain loop
  writes queued 4 and 5, `expected_seq = 6`. The next cadence SACK reports
  `highest_seen = 5` with a full mask.

---

## 5. Error recovery

### 5.1 Data NACK — all fatal

`sendPipeNack(err)` → 8 bytes `FF 81 <err> highest_seen mask[0..3]`
([display_service.cpp:2194-2208](../src/display_service.cpp)). The SACK tail is
built from state **before** any teardown so the reported position stays
consistent.

Every `0x81` NACK is **fatal**:

1. `pipeState.error = true`.
2. `cleanupDirectWriteState(true)` runs the **same** hardware cleanup as the
   legacy mid-stream `0xFF 71` failure: sleep a powered controller cleanly, cut
   power, resume touch. (For a **partial** transfer the teardown instead runs
   through `cleanup_partial_write_state()`, since partial sessions own `partialCtx`
   rather than the full-frame direct-write session — §9.)
3. `pipeState` and the reorder queue are **deliberately not reset**, so
   subsequent `0x0081` frames are silently discarded until the next `0x0080`
   START or a BLE disconnect.

| `err`  | Cause                                                              |
|--------|--------------------------------------------------------------------|
| `0x02` | Compressed (zlib) consume failure                                  |
| `0x03` | Uncompressed consume failure, over-size frame, or reorder overflow |
| `0x04` | Out of window on both sides (protocol violation)                   |

### 5.2 Loss recovery flow (the normal case)

Loss recovery is **not** a NACK — it is the SACK mechanism:

1. A frame lands ahead of the hole → device queues it and sends an immediate SACK
   whose zeroed mask bits name the missing seqs.
2. The sender retransmits exactly those seqs.
3. The retransmit arrives in order → written, and the contiguous run of queued
   successors drains → the stream resumes.

NACKs are reserved for unrecoverable conditions (bad payload, protocol
violation), not ordinary packet loss.

### 5.3 Stuck-transfer timeout

The main loop enforces a 15-minute (`900000 ms`) ceiling on a stalled
direct-write / pipe transfer ([main.cpp](../src/main.cpp)), after which the panel
hardware is released even if no END or NACK ever arrives.

---

## 6. Completion — PIPE_WRITE_END (0x0082)

Handler: [handlePipeWriteEnd, display_service.cpp:2522-2600](../src/display_service.cpp).
For **full-frame** transfers END shares the legacy `0x72` finalizer
`directWriteFinishAndRefresh(data, len, 0x82)`. **Partial** transfers take a
dedicated branch ([display_service.cpp:2543-2580](../src/display_service.cpp))
that drives `partial_write_to_panel()` — see §9.

### 6.1 Payload (after `00 82`)

| Offset | Field         | Size   | Notes                                              |
|--------|---------------|--------|----------------------------------------------------|
| 0      | `refresh_mode`| 1      | full-frame: `1` = fast, anything else = full. Partial: `0` = FULL, `1` = FAST, `2`/absent = PARTIAL waveform (`REFRESH_PARTIAL`) |
| 1–4    | `new_etag`    | 4 **BE** | New etag, big-endian (`parse_be_u32`), optional  |

The `new_etag` is big-endian to match legacy `0x72` exactly — **note the contrast
with the partial START's `old_etag`, which is little-endian (§2.1.1)**; both are
matched by every client (see the endianness contract box in §1). A nonzero etag is
committed to `displayed_etag` on a successful refresh; a zero/absent etag clears
`displayed_etag` so a later partial update falls back cleanly on an etag mismatch.
For partial transfers the `refresh_mode` selector at offset 0 is decoded
distinctly from the etag: `0`/`1` force FULL/FAST, while `2` or an absent byte
selects the partial waveform ([display_service.cpp:2561-2566](../src/display_service.cpp)).

### 6.2 Flow

1. Not active → `FF 82`. Already in fatal error → `FF 82` + defensive cleanup +
   reset.
2. **Tail-flush SACK** (`sendPipeAck()`) so the sender sees the final receiver
   state.
3. **Completeness check:** if the reorder queue is non-empty (`queued_count > 0`),
   or (uncompressed) fewer than `total_size` bytes were written, the transfer is
   incomplete → `FF 82` + `cleanupDirectWriteState(true)` + reset. (Compressed
   incompleteness surfaces as a zlib-flush NACK inside the shared finalizer.)
4. Otherwise finalize: success `00 82`, panel refresh, then `00 73` on refresh
   success or `00 74` on refresh timeout, then `resetPipeWriteState()`.

### 6.3 Auto-complete (uncompressed only)

When an in-order accept pushes `directWriteBytesWritten >= total_size`, the device
finalizes **without** waiting for an END frame
([display_service.cpp:2468-2473](../src/display_service.cpp)): it flushes a final
SACK, calls `directWriteFinishAndRefresh(nullptr, 0, 0x82)` (an unsolicited
`00 82` + full refresh, no etag), and resets. This mirrors the legacy
auto-finish behaviour.

Auto-complete is **gated on `!pipeState.partial`**: a partial transfer never
touches `directWrite*` (both byte counters stay 0, so `0 >= 0` would false-fire on
the very first frame). **Partial transfers therefore complete only on the explicit
`0x0082` END** — which is also the only frame carrying the refresh selector and
`new_etag`. See §9.

---

## 7. End-to-end sequence

```
Client                                   Device
  |  00 80  ver flags W N frame total  -->|   negotiate
  |<-- 00 80  01 MAXW MAXN FRAME flags     |   grant (device maxima)
  |                                        |
  |  00 81  00 <payload>              ---->|   seq 0  -> panel
  |  00 81  01 <payload>              ---->|   seq 1  -> panel
  |  00 81  02 <payload>              ---->|   seq 2  -> panel
  |  00 81  03 <payload>   (LOST)      -x  |
  |  00 81  04 <payload>              ---->|   queued; gap opens
  |<-- 00 81  04  mask(bit@3=0)             |   immediate SACK
  |  00 81  03 <payload>  (retransmit)---->|   seq 3 -> panel, drains 4
  |             ... cadence SACKs ...       |
  |  00 82  mode etag                 ---->|   finalize
  |<-- 00 81  hs mask   (tail flush)        |
  |<-- 00 82                                |   success
  |<-- 00 73                                |   refresh complete
```

---

## 8. Compatibility

- Legacy `0x70/0x71/0x72` and partial `0x76` paths are **byte-identical** — old
  clients are unaffected.
- A new client that sends `0x0080` and gets no response within its 2 s probe
  assumes legacy-only firmware and falls back to `0x70`.
- Pipe data ACKs at `highest_seen` = `0xFE`/`0xFF` stay **encrypted**: the
  `sendResponse` encryption-skip heuristic (which normally treats a `0xFE`/`0xFF`
  status byte as an unencrypted response) is scoped to exclude the 7-byte
  `00 81 …` ACK shape ([communication.cpp:177-187](../src/communication.cpp)).

---

## 9. Partial-region refresh (`PIPE_FLAG_PARTIAL`)

A single-rectangle partial update can ride the sliding window instead of the
legacy stop-and-wait `0x76` path. It reuses the whole pipe machinery
(negotiation, SACK, reorder queue, error recovery); only three things differ from
a full-frame transfer.

**1. START negotiates the rectangle and base etag.** The client sets `flags` bit1
(`PIPE_FLAG_PARTIAL`) and appends the 12-byte LE extension of §2.1.1
(`[old_etag:4 LE][x:2][y:2][w:2][h:2]`). `total_size` is `plane_size * 2` — the two
flat 1bpp controller planes (old plane then new plane) the `0x76` path also
streams. The device confirms by setting **bit1 in the START response flags**
([display_service.cpp:2401-2404](../src/display_service.cpp)); geometry/etag/rect
are validated exactly like the `0x76` handler
([display_service.cpp:2308-2342](../src/display_service.cpp)).

**2. DATA streams into `partialCtx`, not the full-frame writer.** When
`pipeState.partial` is set, `pipeConsumePayload` routes bytes to
`partial_consume_bytes` (zlib-vs-raw + plane-at-a-time sub-window handling) instead
of the full-frame controller write. The two 1bpp planes are the partial mechanism,
which is why partial is rejected (NACK `0x06`) on non-1bpp panels and on
seeed/IT8951 drivers that have no controller-plane equivalent.

**3. Completion is END-only, and END carries the refresh selector + `new_etag`.**
Partial transfers **never auto-complete** (§6.3): the explicit `0x0082` END is the
only frame that finalizes them, and it alone carries the refresh selector
(`0`→FULL, `1`→FAST, `2`/absent→PARTIAL waveform) and the big-endian `new_etag`
([display_service.cpp:2543-2580](../src/display_service.cpp)). On success the device
commits `new_etag` to `displayed_etag`; on any failure or incomplete transfer it
clears `displayed_etag` and cleans up via `cleanup_partial_write_state()` (partial
sessions own `partialCtx`, not the full-frame direct-write session, so pipe NACKs
tear down through the partial path — [display_service.cpp:2202-2207](../src/display_service.cpp)).

### 9.1 Client fallback ladder

The negotiation is designed so a client can always degrade to legacy. As
implemented in py-opendisplay (`device.py` `_maybe_upload_partial` /
`_pipe_partial_upload`, ~`device.py:2200-2290`; NACK constants
[responses.py:281](../../py-opendisplay/src/opendisplay/protocol/responses.py)):

| START outcome | Client action |
|---------------|---------------|
| Response bit1 set | Proceed with pipe-partial DATA/END |
| NACK `0x05` `ETAG_MISMATCH` | Skip `0x76` (device already cleared its etag) → **full** upload |
| NACK `0x06` `PARTIAL_UNSUPPORTED` | Go **full** (`0x76` would fail identically); cache a per-connection negative so later partials skip straight to full |
| NACK `0x07` `RECT_INVALID` | Go **full** |
| NACK `0x02` on a partial START (after one uncompressed-still-partial retry), or a success **without** bit1 | Disable pipe-partial for the connection, fall back to legacy `0x76` |
| Silence / garbled response | Fall back to legacy `0x76` |

> **Cross-repo note.** The opendisplay.org web tooling does **not** implement
> pipe-partial: its partial updates use legacy `0x76` (big-endian etags,
> `ble-common.js`), and it attaches only a big-endian `new_etag` to full-frame
> pipe ENDs. The Home Assistant integration is a read-only config consumer and
> constructs no pipe frames at all. If a pipe-partial encoder is ever added to the
> web client, the START extension `old_etag` is **little-endian** (§2.1.1) — do not
> copy the big-endian `0x76` packing.

---

## 10. PSRAM slot storage (`PIPE_FLAG_SLOT_TARGET`) — LOCAL FORK DIVERGENCE

> **This entire section documents a local fork divergence.** `PIPE_FLAG_SLOT_TARGET`,
> its wire extension, the two new NACK codes, and the on-device switching path are
> **not** part of the canonical `opendisplay-protocol` spec and are **not** synced
> from it — they were hand-added directly to `include/opendisplay_protocol.h` and
> `include/opendisplay_structs.h` in this fork (normally those two files are a
> byte-for-byte vendored copy; see `CLAUDE.md`'s "vendored protocol header"
> section). A future sync from upstream will not remove this feature, but will
> need manual reconciling on those two files if upstream ever adds something that
> collides with it.

### 10.1 Motivation

Every path described in §1–§9 streams straight to the panel controller — "there
is no framebuffer." That makes switching between several pre-rendered pages (e.g.
a trains page and a weather page) a full BLE round trip every time: negotiate,
stream, refresh. On BLE that round trip dominates — 10s+ — even though the
underlying SPI transfer and panel refresh together take a couple of seconds.

Slot storage lets a client push several pages ahead of time into fixed,
board-sized regions of PSRAM, kept independently up to date (e.g. the trains
page re-pushed every 5 minutes, the weather page whenever the forecast changes).
A physical button press then switches which slot is on the panel entirely
on-device — SPI + decompress + refresh, no BLE involved at press time. See the
[`gavinmcnair/opendisplay-pages`](https://github.com/gavinmcnair/opendisplay-pages)
Rust client for a working multi-page implementation built on this: it renders
each page (trains, weather, an auto-generated index) independently, pushes each
to its own slot only when that page's content actually changed, and always
reserves slot 0 for the index.

### 10.2 Per-board slot count and size

Slot storage is compile-time gated on `OD_SLOT_STORE_ENABLED`
([structs.h](../src/structs.h)), which requires both `BOARD_HAS_PSRAM` and a
board-supplied `OD_SLOT_STORE_PSRAM_BUDGET_BYTES` build flag
([platformio.ini](../platformio.ini)). Boards without PSRAM compile the feature
out entirely — `OD_SLOT_COUNT` is `0`, and every slot-target START NACKs
`SLOT_INVALID` (§10.4) on those builds, same as an out-of-range `slot_id` on a
PSRAM board.

Each slot is a fixed `OD_SLOT_SIZE_BYTES` (32KB) region, sized with real margin
over measured real-page compression ratios (8–15KB typical). `OD_SLOT_COUNT =
OD_SLOT_STORE_PSRAM_BUDGET_BYTES / OD_SLOT_SIZE_BYTES` — derived per board, not
a single constant across the fleet: 8MB-PSRAM boards get 100 slots (3.2MB
budget), the smaller `esp32-s3-N4R2` gets fewer. The wire field `slot_id` is
always `0..99` regardless of board (§10.4); a board with a smaller derived
`OD_SLOT_COUNT` just NACKs the ids above its own ceiling.

All `OD_SLOT_COUNT` slots share **one contiguous** `heap_caps_calloc` allocation
made once at boot in `odDisplayReserveBuffers()`
([display_service.cpp:649-672](../src/display_service.cpp)) — not one
allocation per slot — matching this repo's "avoid heap allocation where
possible, and justify what you can't avoid" rule. `slots[i].data` points
`OD_SLOT_SIZE_BYTES` apart into that one block
([display_service.cpp:622-664](../src/display_service.cpp)).

Slots store their payload **compressed at rest**; decompression happens only at
switch time (§10.5). 100 slots held decompressed (96,000 bytes/page) would need
9.6MB — more PSRAM than an 8MB board has at all. Compressed-at-rest is what
makes the slot count/size arithmetic work.

### 10.3 What a slot-target START looks like on the wire

Same `CMD_PIPE_WRITE_START` (`0x0080`) opcode as every other pipe transfer, with
`flags` bit2 (`PIPE_FLAG_SLOT_TARGET`, `0x04`) set. This is **mutually exclusive
with `PIPE_FLAG_PARTIAL`** (bit1) — a request setting both NACKs
`UNKNOWN_FLAG` (`0x02`) — since slot storage never needs partial-region
semantics (a slot always holds a whole page).

The request carries a **6-byte little-endian `PipeSlotExt` extension** appended
after `total_size` (post-opcode length 16, not 10):

| Offset | Field                | Size | Notes                                                              |
|--------|----------------------|------|---------------------------------------------------------------------|
| 10     | `slot_id`            | 1    | `0..99`; must be `< OD_SLOT_COUNT` for this board                   |
| 11     | `reserved`           | 1    | Must be `0`                                                          |
| 12–15  | `decompressed_size`  | 4 LE | Optional parity-check hint used at switch time (§10.5); `0` skips the check — **not** used to size anything during the write itself |

Unlike the full-frame and partial paths, `total_size` here means **the
compressed byte total actually being stored** (slot storage is
compressed-at-rest), and must satisfy `0 < total_size <= OD_SLOT_SIZE_BYTES`
for this board — not the decompressed panel/plane geometry §2.2 computes for
the other two paths. `decompressed_size` in the extension is a separate,
optional field for the switch-time decode, unrelated to the write-time size
check.

### 10.4 START validation and NACKs

Validated in `handlePipeWriteStart`
([display_service.cpp:2982-3120](../src/display_service.cpp), slot-target logic
at 3083-3120), before any hardware or PSRAM slot is touched (same
validate-before-touching-state pattern as the partial path):

1. `slot_id >= OD_SLOT_COUNT` → NACK `SLOT_INVALID` (`0x04`). This correctly
   also covers "slot storage unsupported on this board" — a board with no PSRAM
   budget has `OD_SLOT_COUNT == 0`, so every `slot_id` is out of range.
2. `total_size == 0 || total_size > OD_SLOT_SIZE_BYTES` → NACK `SLOT_TOO_LARGE`
   (`0x08`).
3. On success: `pipeState.to_slot = true; pipeState.target_slot = slot_id;`
   ([display_service.cpp:3119-3120](../src/display_service.cpp)), and that
   slot's `valid` flag is cleared so an aborted transfer never leaves stale
   bytes marked valid.

A slot-target START **never calls** `directWriteComputeGeometry()` or
`directWriteActivatePanel()` — this path never touches the panel at negotiation
time, unlike every other START variant in this document.

### 10.5 DATA and END — storage only, no panel, no refresh wait

`pipeConsumePayload()`'s `to_slot` branch
([display_service.cpp:2926-2947](../src/display_service.cpp), the `to_slot` arm
at 2935-2946) is the simplest of the three payload routes in this document: a
bounds-checked `memcpy` of each frame's bytes into `slots[target_slot].data` at
the current write offset, `length` bumped to match. Compressed bytes are
stored exactly as received — **no zlib inflate, no gray4 plane split, no
controller write** on this path at all (contrast with the full-frame and
partial routes, both of which decode/write to the panel as data arrives).

`CMD_PIPE_WRITE_END` for a slot-target transfer
([display_service.cpp:3321-3357](../src/display_service.cpp)) marks
`slots[target_slot].valid = true` and sends the END ACK (`0x00 0x82`)
immediately — **before** any refresh, not after waiting for one. It does
**not** send or wait for `RESP_DIRECT_WRITE_REFRESH_COMPLETE`
(`0x00 0x73`/`0x00 0x74`, §6.4) **over BLE, ever**, for this path. What happens
next depends on whether the written slot is the one currently selected on the
panel: if `target_slot == currentSlotIndex`, `odDisplaySwitchToSlot()` (§10.6)
runs immediately afterward and *does* decompress and refresh the panel — just
entirely on-device, with no BLE response framing of its own. Pushing to any
*other* slot stays silent on the panel until a button later selects it. Either
way, the client never has BLE traffic to wait for after the END ACK — this is
the actual payoff described in §10.1: the END ACK is never gated on a refresh,
so finishing a slot upload is as fast as the BLE transfer itself, with no
refresh latency added on top. **A client that reuses the full-frame
END-then-wait-for-refresh logic for slot writes will hang waiting for a
response that is never sent** — this is the one behavior most worth getting
right in a new client (see the auto-complete gating precedent in §6.3 for the
same class of mistake on the partial path).

Auto-complete (§6.3) does not apply to slot-target transfers either, for the
same reason it's gated off for partial transfers: it's driven by
`directWriteBytesWritten`, which a slot-target transfer never touches.
Completion is END-only.

### 10.6 On-device switching — no BLE opcode

There is **no BLE command to request a slot switch.** Slot contents reach the
panel only through a local, non-BLE path: `odDisplaySwitchToSlot(slot_index)`
([display_service.cpp:2618-2699](../src/display_service.cpp)), which:

1. Refuses if `slot_index` is out of range, that slot's `valid` flag is unset,
   or a BLE transfer is currently active (`transferActive()`) — a switch
   request racing a live transfer is silently dropped, not queued.
2. Decompresses the whole stored slot in one call via the existing streaming
   `tinfl`/`od_zlib_stream_*` API, reusing the same 2048-byte
   `decompressionChunk` scratch buffer the BLE-streaming paths already use —
   **no new heap allocation** for the switch itself.
3. Drains the decompressed stream to the panel via the same
   `directWriteActivatePanel()`/`directWriteFinishAndRefresh()` primitives the
   BLE paths use internally, but as a standalone sequence with no BLE response
   framing and no etag handling (`displayed_etag` is cleared on switch, same
   as any other non-partial-aware panel write — the next partial-region
   request falls back to full).

Two higher-level wrappers, both gated the same `OD_SLOT_STORE_ENABLED` way
(returning `false` unconditionally when slot storage is compiled out,
[display_service.cpp:2740-2741](../src/display_service.cpp)):

- `odDisplayCycleSlot(int8_t direction)`
  ([display_service.cpp:2706-2733](../src/display_service.cpp)) — scans forward
  (`+1`) or backward (`-1`) from a file-static "current slot" index for the
  next **valid** slot, wrapping at `OD_SLOT_COUNT`, and **deliberately skips
  slot 0** ([display_service.cpp:2718-2726](../src/display_service.cpp)) — the
  index page (§10.1) is reachable directly via `odDisplayJumpToSlot(0)`
  instead, so cycling through content pages never lands on it by accident.
- `odDisplayJumpToSlot(slot_index)`
  ([display_service.cpp:2734-2739](../src/display_service.cpp)) — switches
  directly to a specific slot, no scan, no skip rule (used for slot 0).

### 10.7 Button wiring

`processButtonEvents()`
([device_control.cpp:674-676](../src/device_control.cpp)) maps the three
physical buttons:

| Button | `button_id` | Action                          |
|--------|-------------|----------------------------------|
| KEY1   | 0           | `odDisplayCycleSlot(-1)` — previous populated slot (skips slot 0) |
| KEY2   | 1           | `odDisplayCycleSlot(+1)` — next populated slot (skips slot 0)     |
| KEY3   | 2           | `odDisplayJumpToSlot(0)` — jump straight to the index (short press only; KEY3's long-press bootloader entry is unrelated firmware/BootROM behavior and untouched by this feature) |

This hook runs **after** the existing advertising-boost/`updatemsdata()` call
in the same handler — that ordering is load-bearing (see the surrounding code
comment): it was previously tuned to fix a real button-press-missed-by-scanner
regression, and a slot switch (SPI transfer + refresh, up to ~2s) running
before it would reintroduce that regression.

### 10.8 py-opendisplay support

[`OpenDisplay/py-opendisplay`](https://github.com/OpenDisplay/py-opendisplay)
(fork: `gavinmcnair/py-opendisplay`, branch `feat/pipe-slot-write`) mirrors this
extension: `PIPE_FLAG_SLOT_TARGET` and a `PipeSlotRequest` dataclass in
`protocol/commands.py`, the two new NACK codes in `protocol/responses.py`, and
a `write_slot(slot_id, data)` entry point in `device.py` whose upload helper
(`_run_pipe_slot_upload`) deliberately **skips** the refresh-wait §10.5
describes — the one behavior a naive copy of the full-frame upload helper would
get wrong.
