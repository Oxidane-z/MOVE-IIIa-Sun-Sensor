# SUN_SENSOR_SPI Protocol Specification

**Version:** 2
**Sensor MCU:** MSP430i2041 (4-quadrant photodiode pinhole sun sensor)
**Role:** SPI **slave**

**Changes in v2:** the FRAME temperature field (bytes 22..23) now carries the
external AT30TS74 temperature converted on-MCU to centi-°C (0.01 °C steps),
replacing v1's raw AT30TS74 register. Masters must update their temperature
parse (see §5, §12).

**Conformance fix (v2.1):** the firmware now emits a **true unit** sun-vector
(gnomonic normalization). Earlier builds returned the correct *direction* but a
sub-unit *magnitude* (up to ~13% short near the FOV corners — the cube-corner
working point). Wire format and version byte (`0x02`) are unchanged — §5 always
specified a unit-vector, so this only makes the implementation conform. Masters
that renormalized are unaffected; masters that fed the raw vector into
QUEST/TRIAD/EKF should drop the now-unnecessary renormalization and will see the
bias removed.

This document fully specifies the wire protocol between the host MCU (master)
and the sun sensor (slave). It is self-contained — implementing a master-side
driver from this document alone should require no further consultation.

---

## 1. Overview

The sensor produces an averaged measurement frame every **8 ms (125 Hz)** that
contains the sun unit-vector, raw quadrant ADC values, temperature, status
flags, and a CRC. The host polls the sensor over SPI at any rate it likes
(typically the ADCS attitude-loop rate, 10–200 Hz); frames are tagged with a
monotonic `sample_count` so duplicate / missed frames are unambiguous.

The sensor never drives MISO autonomously and never asserts an external
"data ready" line — all communication is master-initiated. Reliability is
provided by CRC16 on each frame and by the `sample_count` sequence.

---

## 2. Electrical Interface

| Parameter | Value | Notes |
|---|---|---|
| SPI mode | **Mode 0** (CPOL = 0, CPHA = 0) | clock idle low, data captured on rising edge |
| Bit order | **MSB first** | |
| Word size | 8 bits | |
| Recommended master clock | **≤ 1 MHz** | bring-up sweet spot 250–500 kHz; margin shrinks above ~1 MHz — see §9.7 |
| Supply | 3.3 V | sensor is 3.3 V CMOS, **not 5 V tolerant** |
| Slave clock recovery | clock from master (slave is passive) | |
| Idle state | CS high; sensor performs no SPI activity | sensor's internal ADC and temperature loop run regardless |

### Pinout (sensor side)

| Sensor pin | Function | Direction (from master) |
|---|---|---|
| `P1.0` | CS (active low) | master → sensor |
| `P1.1` | SCK | master → sensor |
| `P1.2` | MISO (sensor output) | sensor → master |
| `P1.3` | MOSI (sensor input) | master → sensor |
| `GND` | common ground | — |

### CS handling

The sensor uses **4-wire hardware-framed SPI**: the CS pin is wired to the
eUSCI STE input (active-low), so the slave peripheral itself handles framing.
While CS is low the slave shifts; when CS goes high MISO is tri-stated and the
peripheral realigns its bit counter, so every transaction starts on a clean
byte boundary. (This is an internal change from an earlier 3-wire/GPIO build —
the master-side behavior below is identical, so existing masters need no
change.) The master must:

- Hold CS **low for the duration of an entire transaction** (CMD + all
  dummy bytes).
- Release CS **between transactions**; the rising edge is what realigns the
  slave. Glitching CS mid-transaction will desync the byte counter, but it
  always recovers on the next CS rising edge.
- Provide setup/hold around SCK: 250 ns either side of CS is plenty.

---

## 3. Transaction Format

Every transaction looks like this on the wire:

```
master TX (MOSI):  [ CMD  ][ 0x00 ][ 0x00 ] ... [ 0x00 ]    N + 1 + K bytes
master RX (MISO):  [ 0xFF ]...[ 0xFF ][ rsp0 ][ rsp1 ] ... [rspN-1][ trailing 0xFF... ]
                     ^                  ^                  ^
                     |                  |                  +-- last response byte
                     |                  +-- first response byte (offset varies, 1..LEAD_MAX)
                     +-- "idle" lead byte(s); ALWAYS at least one 0xFF
```

### Lead-byte count is variable — the master MUST resync

The sensor returns **one or more** 0xFF idle bytes before the actual N-byte
response begins. The exact count depends on which firmware the sensor runs:

- **Current firmware (4-wire):** a **deterministic 2 lead bytes** for every
  command — see the mechanism in the next subsection.
- **Older 3-wire firmware:** a *variable* **1–2 lead bytes**, depending on the
  slave's per-CMD interrupt latency vs. the master's SPI clock.
- **Hard upper bound: ≤ 3 lead bytes** across all firmware versions and clock
  rates. The master sizes its resync window for this bound and never needs to
  know which firmware the sensor is running.

To handle this, the master MUST:

1. **Clock K extra "overclock" bytes** beyond the nominal `N + 1`. **Use
   K = 4** as a safe default — it tolerates any plausible lead-byte count and
   adds negligible bus time.

2. **Resynchronize on the received bytes**, using the per-command technique
   below. Each command has a built-in way for the master to identify the
   true start of the response:

   | Command       | Sync technique                                          |
   |---------------|---------------------------------------------------------|
   | `READ_FRAME`  | Slide a 27-byte CRC-16 window over offsets `0..K`; accept the first offset whose CRC validates. |
   | `READ_ID`     | Scan for the magic prefix `'S' 'U' 'N'` and accept the version byte after it. |
   | `READ_STATUS` | Skip leading `0xFF` bytes; the first non-`0xFF` byte is `flags`, the next is `new_data`. (Both bytes are guaranteed < `0xFF` by the firmware — `flags` is at most `0x07`, `new_data` is `0x00` or `0x01`.) |
   | `NOP`         | Skip leading `0xFF` bytes; the first `0x00` is the NOP response. |

3. After the transaction, raise CS. In 4-wire mode CS high only tri-states
   MISO and hardware-realigns the slave's bit counter — it does **not** reset
   the software response stream (there is no STE interrupt). That stream is
   re-armed by the next command opcode in the CMD slot (see item 4).

4. **Send only `0x00` dummy bytes.** The slave reframes its response on *any*
   received byte in the command range `0xA0..0xAF`, so a dummy in that range is
   taken as a new command and corrupts the in-flight response. `0x00` is the
   canonical dummy; never put a byte in `0xA0..0xAF` anywhere except the CMD
   (first) slot.

### Why the lead byte exists (and why you still resync)

The slave is an MSP430i2041 running at 16.384 MHz with no DMA. Its SPI ISR is
**always-RX**: every received byte (the CMD plus each dummy) fires `UCRXIFG`,
and the handler queues the *next* response byte into `UCA0TXBUF`, which the
hardware loads at the following byte boundary. This gives every byte a full
byte-time of load margin, so the response stream after the lead is always
clean.

A lead byte is unavoidable because the CMD-decode and byte 1's `TXSHIFT` load
happen on the same clock edge — the decode cannot reach `TXSHIFT` in time for
byte 1. The current firmware therefore emits a **deterministic 2-byte lead**:
bytes 0 and 1 read back the preloaded `0xFF`, and `response[0]` lands at byte 2.

**The master must still resync anyway — do not hard-code offset 2.** The
protocol's contract is the defensive `1..3` envelope above, and the
overclock-and-resync pattern costs nothing: it keeps the master compatible
with sensors running older firmware (which produced a *variable* 1–2 lead) and
robust against CS glitches, while the CRC still catches any misframe. Resync is
the master's job, not the slave's; this keeps the slave's ISR minimal (it can
never use DMA on this MCU) while still guaranteeing reliable data.

---

## 4. Command Set

| CMD byte | Name | Response length N | Total clocked bytes | Description |
|---|---|---|---|---|
| `0xA0` | `READ_ID` | 4 | **9** = 1 CMD + 4 response + 4 overclock | Returns magic identifier `'S' 'U' 'N' 0x02`. Use for sensor presence detection and protocol-version check. |
| `0xA1` | `READ_STATUS` | 2 | **7** = 1 + 2 + 4 | Returns `[flags, new_data]`. Cheap — useful for polling whether a new frame is available without reading the whole FRAME. |
| `0xA2` | `READ_FRAME` | 27 | **32** = 1 + 27 + 4 | Returns the full atomic sensor frame (see §5). This is the primary command. |
| `0xAF` | `NOP` | 1 | **6** = 1 + 1 + 4 | Returns `0x00`. Sanity ping. |
| any other | — | 1 | — | Returns `0xFF` (error indicator). **Weak NAK:** this is the same value as the idle/lead byte, so the master cannot positively distinguish "illegal command" from "no / garbled response". Don't use it for error signalling — rely on `READ_ID` and the FRAME CRC instead. |

All multi-byte values in responses are **big-endian** unless noted otherwise.

The lead bytes are **absorbed by the overclock window**, not added to it — the
N-byte response shifts right by 1..3 within the `(response + overclock)`
region, never beyond. The §3 per-command resync technique finds the true
offset.

---

## 5. FRAME Layout

The `READ_FRAME` response is **27 bytes**:

| Offset | Field | Size | Type | Description |
|--------|--------------|---|--------|--------------------------------------------------|
| 0..3   | `sample_count` | 4 | u32 BE | Monotonic frame sequence. Advances by **32** per emitted frame; each unit = 250 µs of raw ADC time. The value marks the **last raw sample of the 32-sample averaging window** (the window's trailing edge). The measurement *epoch* (boxcar centre) is therefore **`(sample_count − 16) × 250 µs`** since boot — see §9.3. Rolls over after ~12 days. |
| 4..5   | `sx`           | 2 | i16 BE | Sun unit-vector X component, **scaled ×10000** (so `+9876` means `+0.9876`). Sensor body frame: +X = "right". |
| 6..7   | `sy`           | 2 | i16 BE | Sun unit-vector Y component, ×10000. +Y = "top". |
| 8..9   | `sz`           | 2 | i16 BE | Sun unit-vector Z component, ×10000. +Z = sensor normal. |
| 10..11 | `A0`           | 2 | i16 BE | Raw ADC quadrant 0 (Bottom-Left), 32-sample averaged. |
| 12..13 | `A1`           | 2 | i16 BE | Raw ADC quadrant 1 (Top-Left), 32-sample averaged. |
| 14..15 | `A2`           | 2 | i16 BE | Raw ADC quadrant 2 (Top-Right), 32-sample averaged. |
| 16..17 | `A3`           | 2 | i16 BE | Raw ADC quadrant 3 (Bottom-Right), 32-sample averaged. |
| 18..21 | `sum`          | 4 | u32 BE | Sum of all four quadrants after dark-offset correction. Drops when the light spot leaves the photodiode array — useful for diagnosing past-FOV conditions. |
| 22..23 | `temp_c100`    | 2 | i16 BE | **Board** temperature from the external **AT30TS74** I²C sensor, converted on the sensor MCU to **centi-°C** (0.01 °C/LSB) — *not* the MSP430 internal sensor. Convert: `T_celsius = temp_c100 / 100.0`. Range −55.00…+125.00 °C; resolution 0.5 °C (9-bit mode). Sentinel `0x8000` (−327.68) = AT30TS74 not responding (fail-soft; see §11). |
| 24     | `flags`        | 1 | u8     | Bit 0 = `NO_SUN` (sensor sees no sun, vector is zeroed). Bit 1 = `OFF_FOV` (sun is near or past the FOV edge, ~±55°; vector still emitted but accuracy degrades). Bit 2 = `SATURATED` (≥1 ADC channel is past 80% of full scale; centroid biased toward unsaturated channels). |
| 25..26 | `CRC16`        | 2 | u16 BE | CRC-16/CCITT-FALSE over bytes 0..24 (the 25 preceding bytes). See §6. |

**Total: 27 bytes.** Master clocks **32 bytes** total for the full transaction
(`READ_FRAME` CMD + 27 dummy + 4 overclock — see §3 and §4). Reading only
28 bytes triggers the variable-lead-byte failure mode warned about in §3.

### `STATUS` Layout (`READ_STATUS` response, 2 bytes)

| Offset | Field | Size | Description |
|---|---|---|---|
| 0 | `flags` | u8 | Same definition as FRAME.flags. |
| 1 | `new_data` | u8 | `0x01` if a fresh frame has been published since the previous `READ_STATUS` / `READ_FRAME`; `0x00` otherwise. **Both `READ_STATUS` and `READ_FRAME` clear this latch.** |

> **STATUS reliability caveat:** `READ_STATUS` has **no CRC**; resync relies on
> the invariants `flags ≤ 0x07` and `new_data ∈ {0,1}` (see the §7 driver). A
> single bit flip on a `0xFF` lead byte cannot fool this (reaching `≤ 0x07`
> takes ≥ 5 flipped bits), but a *grossly* corrupted lead byte that happens to
> satisfy both invariants would be silently accepted as a valid
> `[flags, new_data]`. The only consequence is a wrong "new frame?" hint —
> `READ_FRAME` is CRC-protected and the recommended `sample_count` dedup path
> doesn't use STATUS at all — so this is acceptable, but don't treat STATUS as a
> high-integrity channel.

### `ID` Layout (`READ_ID` response, 4 bytes)

| Offset | Value | Description |
|---|---|---|
| 0 | `'S'` (0x53) | |
| 1 | `'U'` (0x55) | |
| 2 | `'N'` (0x4E) | |
| 3 | `0x02` | SPI_PROTOCOL_VERSION (this document is v2). Increment on breaking changes. |

---

## 6. CRC Algorithm

**Algorithm: CRC-16/CCITT-FALSE**

| Parameter | Value |
|---|---|
| Polynomial | `0x1021` (x¹⁶ + x¹² + x⁵ + 1) |
| Initial value | `0xFFFF` |
| Reflect input bytes? | No |
| Reflect output? | No |
| Final XOR | `0x0000` (none) |
| Byte order in output | big-endian (MSB at offset 25, LSB at 26) |

This is the standard "CRC-16/CCITT-FALSE" variant. There are many other CRC16
variants that differ by initial value or reflection — **make sure you use
this exact one.**

### Reference implementation

```c
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000u) ? ((crc << 1) ^ 0x1021u) : (crc << 1);
        }
    }
    return crc;
}
```

### Test vector

To verify your CRC implementation, the standard CRC-16/CCITT-FALSE check
value for the ASCII string `"123456789"` (9 bytes, no terminator) is **`0x29B1`**.

If your function returns 0x29B1 for that input, your CRC is correct.

### Frame-level verification

The CRC must be verified by **sliding a 27-byte window over the overclocked
receive buffer** and accepting the first offset whose CRC validates. The master
clocks **32 bytes total** (1 CMD + 27 response + 4 overclock), but the CMD
slot's MISO byte is discarded, so the **receive buffer is 31 bytes**
(`SUN_FRAME_LEN + OVERCLOCK = 27 + 4`) and the window slides over offsets
`0..4`. See `sun_sensor_read_frame` in §7 for the complete pattern.

A naive fixed-offset check — computing the CRC over the first 27 bytes
received and comparing to bytes [25..26] — **will fail intermittently** any
time the slave returns 2 or more lead bytes (which is normal at higher
clocks). Don't write that code; use the sliding window.

---

## 7. Master-side Driver — Reference Skeleton

This is a complete, hardware-agnostic driver skeleton. You provide the two
HAL hooks at the top (`sun_cs_low`, `sun_cs_high`, `sun_spi_xfer`) for your
specific master MCU; the rest is portable.

**Important:** every read function below clocks `OVERCLOCK = 4` extra bytes
and resyncs on the received data. Don't shortcut this — the slave's lead-byte
count is not fixed (see §3).

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ===== HAL hooks you provide for your master MCU ===== */
extern void    sun_cs_low(void);          /* drive CS line low */
extern void    sun_cs_high(void);         /* drive CS line high (release) */
extern uint8_t sun_spi_xfer(uint8_t tx);  /* clock one byte; return MISO byte */

/* ===== Protocol constants ===== */
#define SUN_CMD_READ_ID      0xA0u
#define SUN_CMD_READ_STATUS  0xA1u
#define SUN_CMD_READ_FRAME   0xA2u
#define SUN_CMD_NOP          0xAFu

#define SUN_FRAME_LEN        27u
#define SUN_OVERCLOCK        4u    /* extra dummy bytes for slave-lag resync */
#define SUN_PROTOCOL_VERSION 0x02u

#define SUN_FLAG_NO_SUN      (1u << 0)
#define SUN_FLAG_OFF_FOV     (1u << 1)
#define SUN_FLAG_SATURATED   (1u << 2)
#define SUN_FLAGS_VALID_MASK 0x07u  /* bits 0..2 defined; bits 3..7 reserved/zero */

/* ===== Parsed frame ===== */
typedef struct {
    uint32_t sample_count;  /* 1 unit = 250 us */
    int16_t  sx, sy, sz;    /* unit-vector, scaled x10000 */
    int16_t  a0, a1, a2, a3;
    uint32_t sum;
    int16_t  temp_c100;     /* temperature, 0.01 degC; 0x8000 = invalid */
    uint8_t  flags;
} sun_frame_t;

/* ===== CRC-16/CCITT-FALSE ===== */
static uint16_t sun_crc16(const uint8_t *p, uint16_t n) {
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            c = (c & 0x8000u) ? ((c << 1) ^ 0x1021u) : (c << 1);
    }
    return c;
}

/* ===== READ_ID: scan for "SUN" magic + version =====
 *
 * The response 'S' 'U' 'N' 0x02 appears at an unknown offset >= 1 in the
 * received stream after 1..3 lead 0xFF bytes. Search for the magic AND
 * enforce the protocol version -- returning true on magic alone would
 * silently accept a future-version slave that the master may not understand.
 */
bool sun_sensor_read_id(uint8_t *version_out) {
    uint8_t r[4 + SUN_OVERCLOCK];   /* response + safety overclock */
    sun_cs_low();
    (void)sun_spi_xfer(SUN_CMD_READ_ID);
    for (uint8_t i = 0; i < sizeof(r); i++)
        r[i] = sun_spi_xfer(0x00);
    sun_cs_high();

    for (uint8_t off = 0; off + 4 <= sizeof(r); off++) {
        if (r[off] == 'S' && r[off + 1] == 'U' && r[off + 2] == 'N') {
            if (version_out) *version_out = r[off + 3];
            return r[off + 3] == SUN_PROTOCOL_VERSION;
        }
    }
    return false;
}

/* ===== READ_STATUS: skip 0xFF leads, then 2 data bytes =====
 *
 * STATUS has no CRC, so this is the only error detection available.  We
 * enforce the two invariants documented in §5: flags must fit in
 * SUN_FLAGS_VALID_MASK (bits 0..2), new_data must be 0 or 1.  Any deviation
 * from a clean (FF...FF flags new_data) pattern -> reject.  A garbled lead
 * byte that happens not to be 0xFF would otherwise be silently accepted as
 * "flags".  Caller should retry on false.
 */
bool sun_sensor_read_status(uint8_t *flags_out, bool *new_data_out) {
    uint8_t r[2 + SUN_OVERCLOCK];
    sun_cs_low();
    (void)sun_spi_xfer(SUN_CMD_READ_STATUS);
    for (uint8_t i = 0; i < sizeof(r); i++)
        r[i] = sun_spi_xfer(0x00);
    sun_cs_high();

    for (uint8_t off = 0; off + 2 <= sizeof(r); off++) {
        if (r[off] == 0xFFu) continue;                  /* still in lead */
        /* First non-0xFF byte: this must be `flags`. */
        if (r[off]     > SUN_FLAGS_VALID_MASK) return false;   /* invariant break */
        if (r[off + 1] > 0x01u)                return false;   /* new_data invariant break */
        if (flags_out)    *flags_out    = r[off];
        if (new_data_out) *new_data_out = (r[off + 1] != 0);
        return true;
    }
    return false;   /* slave returned only 0xFF -- treat as error */
}

/* ===== READ_FRAME: slide CRC window, accept first valid offset =====
 *
 * The whole 27-byte frame appears at offset 0..SUN_OVERCLOCK in the received
 * stream. The CRC over bytes [off..off+24] equals bytes [off+25..off+26] at
 * exactly one offset; that's the true frame start.
 */
bool sun_sensor_read_frame(sun_frame_t *out) {
    uint8_t r[SUN_FRAME_LEN + SUN_OVERCLOCK];
    sun_cs_low();
    (void)sun_spi_xfer(SUN_CMD_READ_FRAME);
    for (uint16_t i = 0; i < sizeof(r); i++)
        r[i] = sun_spi_xfer(0x00);
    sun_cs_high();

    const uint8_t max_off = (uint8_t)(sizeof(r) - SUN_FRAME_LEN);
    for (uint8_t off = 0; off <= max_off; off++) {
        const uint8_t *b = &r[off];
        uint16_t got_crc = ((uint16_t)b[25] << 8) | b[26];
        if (sun_crc16(b, 25) != got_crc) continue;

        out->sample_count = ((uint32_t)b[ 0] << 24) | ((uint32_t)b[ 1] << 16)
                          | ((uint32_t)b[ 2] <<  8) |  (uint32_t)b[ 3];
        out->sx           = (int16_t)(((uint16_t)b[ 4] << 8) | b[ 5]);
        out->sy           = (int16_t)(((uint16_t)b[ 6] << 8) | b[ 7]);
        out->sz           = (int16_t)(((uint16_t)b[ 8] << 8) | b[ 9]);
        out->a0           = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
        out->a1           = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
        out->a2           = (int16_t)(((uint16_t)b[14] << 8) | b[15]);
        out->a3           = (int16_t)(((uint16_t)b[16] << 8) | b[17]);
        out->sum          = ((uint32_t)b[18] << 24) | ((uint32_t)b[19] << 16)
                          | ((uint32_t)b[20] <<  8) |  (uint32_t)b[21];
        out->temp_c100    = (int16_t)(((uint16_t)b[22] << 8) | b[23]);
        out->flags        = b[24];
        return true;
    }
    return false;   /* no CRC-valid offset within overclock window */
}

/* ===== NOP: skip 0xFF leads, return true if the first data byte is 0x00 ===== */
bool sun_sensor_nop(void) {
    uint8_t r[1 + SUN_OVERCLOCK];
    sun_cs_low();
    (void)sun_spi_xfer(SUN_CMD_NOP);
    for (uint8_t i = 0; i < sizeof(r); i++)
        r[i] = sun_spi_xfer(0x00);
    sun_cs_high();

    for (uint8_t i = 0; i < sizeof(r); i++) {
        if (r[i] != 0xFFu) return r[i] == 0x00u;
    }
    return false;
}
```

---

## 8. Recommended Usage Pattern

The sensor publishes a fresh frame every 8 ms. The master can poll at any
rate without coordination, and uses `sample_count` to detect duplicates and
to measure inter-sample Δt.

```c
static uint32_t prev_sample = 0;

/* Call this from your ADCS attitude loop (10-200 Hz typical). */
void adcs_step(void) {
    sun_frame_t f;
    if (!sun_sensor_read_frame(&f)) {
        /* CRC failure: discard, optionally retry once. Persistent failure
           indicates wiring / clock / mode problem. */
        return;
    }
    if (f.sample_count == prev_sample) {
        /* Same frame as last poll -- master is faster than the sensor's
           125 Hz internal cadence. Skip this attitude tick. */
        return;
    }

    uint32_t delta_units = f.sample_count - prev_sample;   /* 250 us units */
    if (delta_units > 4000u) {
        /* Huge jump = sensor reset (count restarted) or a CRC-passed misaligned
           frame.  Treat as a gap: re-seed, optionally re-READ_ID, skip tick. */
        prev_sample = f.sample_count;
        return;
    }
    float    delta_sec   = delta_units * 250e-6f;
    prev_sample = f.sample_count;
    /* Absolute measurement epoch, if you align with a gyro/clock, is
       (f.sample_count - 16) * 250e-6f  -- the boxcar centre (see §9.3). */

    /* Quality gates */
    if (f.flags & SUN_FLAG_NO_SUN)    return;   /* eclipse / pointed away */
    if (f.flags & SUN_FLAG_SATURATED) {
        /* clipping: vector direction still usable but accuracy degraded */
    }
    if (f.flags & SUN_FLAG_OFF_FOV) {
        /* nonlinear region: weight this measurement down in your filter */
    }

    /* Convert i16 x10000 back to float */
    float sx = f.sx / 10000.0f;
    float sy = f.sy / 10000.0f;
    float sz = f.sz / 10000.0f;

    /* (sx,sy,sz) is already a unit vector (gnomonic-normalized on the sensor);
       no renormalization needed -- only i16 x10000 quantization remains. */
    feed_attitude_filter(sx, sy, sz, delta_sec);
}
```

### Recommended attitude-loop pull rate

- **Higher than 125 Hz (e.g., 250 Hz)**: every sensor frame is consumed at
  least once; some duplicates discarded. No frame loss. ~8 KB/s SPI bus load
  at 250 Hz (32 B/frame × 250 Hz).
- **Around 100 Hz (recommended)**: occasional duplicates filtered out, but
  most ADCS loops run at this rate anyway. Negligible bus load.
- **Below 50 Hz**: many frames discarded by the sensor (never read), but if
  the ADCS loop is slower than the sensor's update rate, this is fine —
  the sensor always exposes the most recent frame.

### Don't busy-wait

`READ_STATUS` is provided for occasional checks, **not for busy-wait
polling**. Don't do `while (status.new_data == 0) {}` — it wastes the SPI
bus. Each ADCS tick should perform **one** poll (either `READ_FRAME`
directly, or `READ_STATUS` then `READ_FRAME` if new_data was set) and
return.

---

## 9. Reliability Requirements (master side)

1. **Overclock and resync, every transaction.** Clock at least 4 extra bytes
   beyond `CMD + N` and use the per-command sync technique from §3.
   Hard-coding "the response is at byte 1" will fail intermittently because
   the slave's lead-byte count is not fixed.

2. **Verify CRC on every FRAME read, then sanity-check `sample_count`.** On
   mismatch (no offset in the overclock window validates), discard the frame
   and optionally retry once; two consecutive failures indicate a real wiring /
   clock / mode problem. **After** the CRC passes, gate the frame on a plausible
   `sample_count` delta: `0 < delta_units < DELTA_MAX` (e.g. `DELTA_MAX ≈ 4000`
   units ≈ 1 s). This near-free second check catches the two failure modes a
   lone CRC cannot:
   - **Sensor reset (brownout / SEU / watchdog):** `sample_count` restarts at 0,
     so the unsigned delta from the previous poll explodes. Reject it as a gap,
     re-`READ_ID`, and re-seed `prev_sample` instead of feeding a huge
     `delta_sec` into the filter.
   - **CRC misalignment:** the sliding window can pass CRC at a *wrong* offset
     with probability ≈ 2⁻¹⁶ per frame (the master tries offset 0 before the
     real frame at offset 1). A misaligned frame's `sample_count` bytes are
     garbage, so the delta gate rejects it almost certainly.

3. **Use `sample_count` for time-of-measurement, and subtract the window
   half-width.** Don't assume the SPI-transaction time is the measurement time.
   `sample_count` marks the **end** of the 32-sample boxcar average, so the
   effective measurement epoch is **`(sample_count − 16) × 250 µs`** since boot
   (the centre of the 8 ms window; −16.5 to be exact). Omitting this ~4 ms shift
   adds a *systematic* timestamp bias — small for slow pointing, but it skews
   the sun-vs-gyro time alignment in the filter during slews, so apply it. (A
   *relative* Δt between two frames is unaffected — the −16 cancels — so the §8
   dedup math needs no change; the shift only matters for the absolute epoch you
   hand the estimator.) Two consecutive polls give both Δt and absolute time.

4. **Read `READ_ID` at startup.** Scan for `'S' 'U' 'N'` magic in the
   overclocked response and verify the version byte. Detects: dead sensor,
   wrong slave selected, mode-bit error, protocol-version mismatch.

5. **Always raise CS between transactions.** Even if the master plans to
   issue back-to-back commands, raise CS briefly (≥ 1 µs) between them.
   This gives the bus a clean idle gap. (CS edges only realign the slave's bit
   counter in hardware; the software response stream is reframed by the command
   opcode, not by CS — see §3.)

6. **Hold CS low for the whole transaction.** The sensor's eUSCI peripheral
   uses the CS (STE) line for hardware framing; toggling CS mid-transaction
   realigns the slave's bit counter and aborts the in-flight response.

7. **Don't exceed 1 MHz SPI clock initially.** Sweet spot for bring-up is
   250–500 kHz. The always-RX ISR has one full byte-time to queue each response
   byte — the same wide margin for every byte, since the old byte-1 race is
   gone. At moderate clocks that margin is large; above ~1 MHz, especially
   under SD24 interrupt contention, it shrinks, and a missed deadline corrupts
   a response byte (which the CRC catches). The lead stays a deterministic 2
   throughout; if you see CRC failures at high clock, lower it.
   The true ceiling is the slave's **worst-case ISR latency**, not a round
   number: the SD24 conversion ISR can delay SPI-RX servicing, and on the i2041
   interrupt priority is **fixed in silicon** (not reorderable), so you can't
   force SPI-RX to be highest. The real constraint is the time **between
   consecutive byte edges**, so a master that inserts small inter-byte gaps can
   run a higher instantaneous SCK than one clocking 32 bytes back-to-back. If
   you need >1 MHz, gap the bytes and/or confirm with the sensor team that the
   SD24 ISR is short enough.

---

## 10. Verification / Bring-up Checklist

In this order — each step depends on the previous one. Use the §7 reference
driver (with overclock + resync) for every read; the lead-byte count is
variable and naive "byte 1 is the response" reads will fail intermittently.

1. **Power & ground**: 3.3 V on the sensor's VCC pin; common GND with master.
2. **`READ_ID`**: scan the overclocked response for the `'S' 'U' 'N'` magic
   and verify the trailing version byte is `0x02`. If no match found,
   debug electrical / mode / clock before proceeding.
3. **`NOP`**: skip leading 0xFF bytes; the first non-0xFF byte should be
   `0x00`.
4. **`READ_FRAME` with sensor covered (dark)**: slide the CRC window — some
   offset within `0..OVERCLOCK` should validate. `sx, sy, sz ≈ 0`,
   `flags == 0x01` (NO_SUN).
5. **`READ_FRAME` under bright direct light** (phone flashlight is fine):
   CRC matches; some of `A0..A3` are large positive; `sx, sy, sz` are
   non-zero; `flags == 0x00` or possibly `0x04` (SATURATED).
6. **Lead-byte stability check** (optional): the §7 `sun_sensor_read_frame`
   recovers the frame at an index `off` in its receive buffer `r[]`. Log that
   `off` over 100 consecutive calls. Note `off` is **one less than the
   on-the-wire lead count**, because the driver discards the CMD-slot MISO byte
   (`(void)sun_spi_xfer(CMD)`):
   - **Current firmware:** stable **`off = 1`** (2 wire lead bytes; response
     starts at wire byte 2).
   - **Older 3-wire sensor:** **`off = 0`** most of the time, occasionally 1
     (1–2 wire lead bytes).
   If `off` ever exceeds 2 (or no offset validates within the window), the wire
   lead has broken the ≤3 envelope — your clock is too fast; lower it.
7. **`sample_count` advances** between successive frames at the expected
   rate (~32 units per 8 ms), as long as you poll faster than 125 Hz.
8. **Move the light source** and observe `sx, sy` swinging through their
   range while `sz` stays near `+10000` for near-axis light and decreases
   for off-axis light.

---

## 11. Sensor side details (FYI, not required for master implementation)

- Sensor MCU: TI MSP430i2041 @ 16.384 MHz.
- ADC: SD24 sigma-delta, OSR=256, 4 channels in parallel at 4 kHz raw.
- Frame averaging: 32 raw samples averaged into one published frame
  → **125 Hz** internal frame rate; `sample_count` advances by 32 per frame.
- Temperature: external **AT30TS74** on the MCU's I²C (eUSCI_B), refreshed once
  per second; the raw register is converted to centi-°C on the MCU (`T =
  raw/256`, 9-bit / 0.5 °C resolution) and only that is shipped (FRAME bytes
  22..23). The read is **fail-soft**: on an I²C timeout/fault the firmware resets
  the eUSCI and returns the `0x8000` sentinel (never hangs), but it does not
  recover a physically stuck SDA/SCL bus — temperature then stays invalid.
- SPI slave: eUSCI_A0 in **4-pin hardware-framed** mode (UCMODE_2, STE
  active-low on the CS pin), SPI Mode 0, MSB-first. The ISR is **always-RX**
  (every received byte queues the next response byte), which yields the
  deterministic 2-byte lead described in §3. There is no CS GPIO interrupt and
  no DMA.
- The sensor guarantees that the frame the master reads via `READ_FRAME` is
  never partially updated or overwritten mid-transaction: it uses a **triple
  buffer** (the publisher writes the one slot that is neither published nor in
  flight), so even a transaction spanning several 8 ms publish cycles is safe.
- The sensor never initiates communication. It does not assert any
  external "data ready" or interrupt line. (A DRDY line was considered and
  ruled out — no spare GPIO on this board.)
- **No write / configuration command exists in v2** — the protocol is
  read-only. The master cannot set FOV / saturation thresholds, trigger a dark
  calibration, or push per-unit calibration coefficients over SPI. If in-orbit
  configuration is needed, a write command is planned for a future **v3**; it
  would also be the channel for the per-sensor calibration coefficients once
  bench calibration is integrated.

---

## 12. Appendix — Endianness and Sign Conventions

- **All multi-byte fields are big-endian** (MSB first). This is the standard
  network byte order. Be careful on little-endian masters (most ARM/AVR
  MCUs): you must assemble each multi-byte value byte-by-byte as shown in
  the reference driver, not by memcpy + cast.

- **Signed values use 2's complement.** `sx`, `sy`, `sz`, `temp_c100`, and
  `A0..A3` are `int16`. The sign bit is bit 7 of the first (high) byte.

- **`sum` is unsigned.** It's the dark-corrected sum of all four quadrants
  and is always ≥ 0 (the firmware clamps negative sums to 0 before
  emitting).

- **Sun vector components** are scaled by 10000, so `sx = +9876` means
  `sx_actual = +0.9876`. The firmware emits a **true unit vector**
  (`sx² + sy² + sz² = 1`, gnomonic normalization); the only deviation is the
  i16 ×10000 quantization (≤ 1×10⁻⁴ per component), so the host does **not**
  need to renormalize.

- **`temp_c100` is centi-°C.** `T_celsius = temp_c100 / 100.0` (e.g. `+2375`
  → `+23.75 °C`). The value `0x8000` (`-32768` = `-327.68`) is a sentinel
  meaning the AT30TS74 did not respond — treat it as "temperature invalid",
  not a real reading.

- **`sample_count` is unsigned** and wraps after `2³² ≈ 4.29 × 10⁹` units.
  At 250 µs/unit that's `2³² × 250e-6 s ≈ 1.07 × 10⁶ s ≈ **12.4 days**` of
  continuous operation. Use unsigned subtraction for deltas to handle wrap
  correctly: `delta = (uint32_t)(new - old);`

---

## 13. Protocol v3 (DRAFT — not yet implemented)

> **Status: paper draft for review. NOT implemented; the wire format may still
> change. v2 (version byte `0x02`) remains the shipping protocol.** v3 bundles
> the breaking changes deferred from v2 plus the additions agreed during review,
> and bumps the `READ_ID` version byte to `0x03`. Implement only once this layout
> is frozen.

### 13.1 Rationale

A version bump breaks master compatibility once, so v3 bundles everything that
needs the break: an expanded health/flags field, per-unit identity, reset
detection, calibration state + ops, a fusion-quality hint, and an optional
fixed-offset fast path. Static identity (serial, firmware version) lives in
`READ_ID`, **not** the per-frame payload, to avoid spending bus bytes every frame.

### 13.2 FRAME layout (v3) — 32 bytes

Bytes 0..24 are **unchanged from v2** (only `flags` gains bit definitions); the
frame is extended by 5 bytes and the CRC moves to the end.

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0..3 | `sample_count` | u32 BE | unchanged (epoch = `(count − 16) × 250 µs`) |
| 4..9 | `sx,sy,sz` | i16 BE ×3 | unit vector ×10000 (unchanged) |
| 10..17 | `A0..A3` | i16 BE ×4 | raw quadrant ADCs (unchanged) |
| 18..21 | `sum` | u32 BE | dark-corrected sum (unchanged) |
| 22..23 | `temp_c100` | i16 BE | external AT30TS74, centi-°C (unchanged) |
| 24 | `flags` | u8 | health/quality bits — see §13.3 |
| **25** | `reset_count` | u8 | **NEW** — increments each boot; the master detects a sensor reset when this changes (more robust than the `sample_count`-jump heuristic) |
| **26** | `quality` | u8 | **NEW** — 0..255 measurement confidence (fusion weight; 0 = unusable) |
| **27..28** | `cal_id` | u16 BE | **NEW** — loaded calibration version/CRC (`0x0000` = uncalibrated / linear baseline) |
| **29** | `reserved` | u8 | **NEW** — 0 (future) |
| **30..31** | `CRC16` | u16 BE | CRC-16/CCITT-FALSE over bytes **0..29** |

The master clocks `32 + 1 + OVERCLOCK` bytes for `READ_FRAME`.

### 13.3 `flags` bit definitions (v3)

The v2 `flags ≤ 0x07` invariant is **dropped** in v3 (resync no longer depends
on it — see §13.6), which frees the upper bits.

| Bit | Name | Meaning |
|---|---|---|
| 0 | `NO_SUN` | no sun; vector zeroed (as v2) |
| 1 | `OFF_FOV` | near/past FOV edge (as v2) |
| 2 | `SATURATED` | ≥1 channel past 80% FS (as v2) |
| 3 | `INTERNAL_FAULT` | internal error latched (e.g. SD24 overflow — `sd24_ovf_count`) |
| 4 | `TEMP_INVALID` | AT30TS74 read failed (mirrors `temp_c100 == 0x8000`) |
| 5 | `CALIBRATED` | this frame used the per-unit polynomial (else linear baseline) |
| 6 | `ALBEDO_SUSPECT` | reserved — diffuse/large-spot heuristic for Earth-albedo discrimination (future) |
| 7 | reserved | 0 |

### 13.4 `READ_ID` (v3) — 13 bytes

| Offset | Field | Notes |
|---|---|---|
| 0..2 | `'S' 'U' 'N'` | magic (resync scan, as v2) |
| 3 | `protocol_version` | `0x03` |
| 4 | `fw_major` | firmware build version |
| 5 | `fw_minor` | |
| 6..9 | `serial` u32 BE | per-unit ID (MSP430 TLV die record, or production-assigned) |
| 10 | `capabilities` u8 | bit0 `WRITE_CONFIG`, bit1 `CAL_RW`, bit2 `TRIGGER_DARK`, … |
| 11..12 | `CRC16` BE | over bytes 0..10 (`READ_ID` is now CRC-protected) |

### 13.5 Command set (v3)

Existing commands keep their opcodes (`READ_ID 0xA0`, `READ_STATUS 0xA1`,
`READ_FRAME 0xA2`, `NOP 0xAF`); response lengths follow the v3 layouts. New
write/config family:

| CMD | Name | Direction | Purpose |
|---|---|---|---|
| `0xA3` | `WRITE_CONFIG` | master→sensor | set runtime thresholds (`SUN_PRESENT` / `OFF_FOV` / `SAT`) |
| `0xA4` | `TRIGGER_DARK` | master→sensor | capture `dark_off[4]` now (sensor must be dark, e.g. eclipse) |
| `0xA5` | `WRITE_CALIB` | master→sensor | upload calibration coefficients to info-flash (chunked) |
| `0xA6` | `READ_CALIB` | sensor→master | read back stored `cal_id` + coefficients (QA / verification) |

Write framing (sketch): `CMD | len | payload… | CRC16`. The always-RX ISR
captures the bytes, verifies CRC, and **defers the action to the main loop**
(flash writes and threshold changes never happen in the ISR). Completion/result
is reported via `READ_STATUS`. `READ_STATUS` (v3) is therefore extended to
`[flags, new_data, reset_count, write_result]` (4 bytes; `write_result` 0 = idle/ok).

### 13.6 Fixed-offset fast path

v3 keeps the deterministic 2-byte wire lead. Because the lead is fixed, a v3
master **MAY** read at the fixed offset and verify the single CRC instead of
sliding a window. The overclock + sliding-window resync from §3 stays valid (and
recommended for robustness) as a fallback; v3 simply no longer *requires* it. The
`0xA0..0xAF` restriction on dummy bytes still applies (opcodes still reframe).

### 13.7 Compatibility / version negotiation

- A master reads `READ_ID` first and branches on the version byte: `0x02` → v2
  (27-byte frame, §3 resync), `0x03` → v3 (32-byte frame, this section).
- **v2 master ↔ v3 sensor:** the v2 master parses with the v2 layout/CRC range;
  the v3 CRC (different range and position) will not validate at the v2 offset,
  so the master gets *no frame* rather than wrong data — a safe degradation.
- **v3 master ↔ v2 sensor:** detected as version `0x02`; the master falls back to
  v2 parsing. A dual-version master is the recommended deployment.

### 13.8 Open questions to resolve before freezing

- **`WRITE_CALIB` is the hard part:** chunking, info-flash write atomicity (the
  i2041 has 4 × 256 B info segments), partial-write safety, and the ack/retry
  protocol all need detailed design. Compile-time bake (already wired via
  `sun_calib.h`, see firmware) covers the ground path; `WRITE_CALIB` is only for
  in-orbit / production-line loading without a debugger.
- `serial` source: MSP430 TLV die record (free, unique) vs production-assigned.
- `cal_id` definition: CRC of the coefficient blob vs a monotonic version number.
- `quality` metric: the exact formula from `sum`/SNR, `SATURATED`, `OFF_FOV`.
- Confirm 8-bit `flags` suffices, or widen to a 16-bit health word.

---

*End of specification. Questions about the sensor side go to {sensor team}.*
