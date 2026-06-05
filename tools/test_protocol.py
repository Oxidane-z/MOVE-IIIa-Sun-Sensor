"""Host-side regression test for the SUN_SENSOR_SPI v2 wire protocol.

Re-implements the 27-byte FRAME packing, CRC-16/CCITT-FALSE, and the §7
master-side sliding-window resync in pure Python, then cross-checks them so an
accidental change to the byte layout / CRC / resync gets caught:

  - the standard CRC check value  crc16("123456789") == 0x29B1
  - pack -> (emulated wire receive buffer with 0..3 lead 0xFF bytes) ->
    sliding-window parse  round-trips every field, at every plausible lead
  - a corrupted frame is rejected (no CRC-valid offset in the window)

Run:  python tools/test_protocol.py        (prints results; exit 0 = all pass)

Scope: this guards the protocol *logic* (layout, CRC, resync). It does not run
the firmware binary (which can't execute on the host), but it pins the exact
contract the firmware's spi_publish_frame()/crc16_ccitt() and the §7 reference
driver must both honour.
"""

import struct
import sys

FRAME_LEN = 27
OVERCLOCK = 4          # master clocks FRAME_LEN + OVERCLOCK dummy bytes


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def pack_frame(sample_count, sx, sy, sz, a0, a1, a2, a3, total, temp_c100, flags):
    """Build the 27-byte FRAME exactly as spi_publish_frame() does (big-endian)."""
    b = bytearray(FRAME_LEN)
    struct.pack_into(">I", b, 0, sample_count & 0xFFFFFFFF)   # 0..3  sample_count
    struct.pack_into(">hhh", b, 4, sx, sy, sz)                # 4..9  sun vector x10000
    struct.pack_into(">hhhh", b, 10, a0, a1, a2, a3)          # 10..17 raw quadrant ADCs
    struct.pack_into(">I", b, 18, total & 0xFFFFFFFF)         # 18..21 dark-corrected sum
    struct.pack_into(">h", b, 22, temp_c100)                  # 22..23 temp centi-degC
    b[24] = flags & 0xFF                                      # 24     flags
    struct.pack_into(">H", b, 25, crc16_ccitt(b[:25]))        # 25..26 CRC16 over 0..24
    return bytes(b)


def parse_frame(r):
    """§7 sliding-window resync: slide a 27-byte CRC window, take first valid offset."""
    for off in range(len(r) - FRAME_LEN + 1):
        w = r[off:off + FRAME_LEN]
        got_crc = (w[25] << 8) | w[26]
        if crc16_ccitt(w[:25]) != got_crc:
            continue
        sc, = struct.unpack_from(">I", w, 0)
        sx, sy, sz = struct.unpack_from(">hhh", w, 4)
        a0, a1, a2, a3 = struct.unpack_from(">hhhh", w, 10)
        total, = struct.unpack_from(">I", w, 18)
        temp, = struct.unpack_from(">h", w, 22)
        return dict(sample_count=sc, sx=sx, sy=sy, sz=sz,
                    a0=a0, a1=a1, a2=a2, a3=a3, total=total,
                    temp_c100=temp, flags=w[24], offset=off)
    return None


def emulate_rx(frame, lead):
    """Driver receive buffer r[]: `lead` leading 0xFF, the frame, then 0xFF pad
    to FRAME_LEN+OVERCLOCK (31) bytes -- the bytes left after the CMD slot."""
    return bytes([0xFF]) * lead + frame + bytes([0xFF]) * (OVERCLOCK - lead)


def main():
    fails = 0

    cv = crc16_ccitt(b"123456789")
    ok = (cv == 0x29B1)
    print(f"[{'PASS' if ok else 'FAIL'}] CRC16/CCITT-FALSE('123456789') = 0x{cv:04X} (want 0x29B1)")
    fails += not ok

    fields = dict(sample_count=32 * 1000, sx=9876, sy=-1234, sz=1500,
                  a0=100, a1=200, a2=300, a3=400, total=123456,
                  temp_c100=2375, flags=0x02)
    frame = pack_frame(**fields)

    for lead in (0, 1, 2, 3):                       # deterministic 2 on the wire -> driver off=1; test the whole <=3 envelope
        got = parse_frame(emulate_rx(frame, lead))
        ok = got is not None and got["offset"] == lead and all(
            got[k] == v for k, v in fields.items())
        print(f"[{'PASS' if ok else 'FAIL'}] round-trip, lead={lead}: "
              f"offset={None if got is None else got['offset']}, fields match={ok}")
        fails += not ok

    bad = bytearray(emulate_rx(frame, 1))
    bad[1 + 5] ^= 0xFF                              # flip a payload byte inside the frame
    rejected = parse_frame(bytes(bad)) is None
    print(f"[{'PASS' if rejected else 'FAIL'}] corrupted frame rejected (no valid offset): {rejected}")
    fails += not rejected

    print(f"\n{'ALL TESTS PASS' if fails == 0 else f'{fails} TEST(S) FAILED'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
