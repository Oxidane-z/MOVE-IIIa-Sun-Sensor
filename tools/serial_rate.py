"""Measure the real serial output rate of the sun-sensor firmware.

A perceived low data rate has three very different causes, and this script
tells them apart:

  1. Firmware genuinely slow  - the SD24/averaging produces frames slowly.
  2. Transport / host dropping - MCU produces fast, host catches few.
  3. Only the *display* is slow - data arrives fine, your terminal/viz just
     refreshes slowly.

To distinguish them it uses BOTH the wall clock AND the firmware's `timestamp`
field.  That timestamp increments by ADC_AVG_N (=32) per emitted frame, with
1 unit = 250 us (the 4 kHz raw SD24 sample rate).  So:

  * MCU frame-generation rate = (timestamp_span / 32) / wall_seconds
        -- the rate the firmware *generated* averaged frames, independent of
           how many the host actually received.
  * Host line rate            = lines_received / wall_seconds
        -- what actually made it into this script.

Comparing the two localizes the bottleneck.

Usage:
    python serial_rate.py --port COM18
    python serial_rate.py --port COM18 --baud 921600 --seconds 5
"""

import argparse
import time
import serial

ADC_AVG_N = 32           # firmware averages this many raw samples per frame
RAW_SAMPLE_US = 250.0    # 1 timestamp unit = 250 us  (4 kHz raw SD24 rate)
NOMINAL_HZ = (1e6 / RAW_SAMPLE_US) / ADC_AVG_N   # = 125.0 Hz expected


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial port, e.g. COM18")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--seconds", type=float, default=5.0,
                   help="measurement window length")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}")
        return

    lines = 0
    byte_count = 0
    bad_lines = 0
    first_ts = last_ts = prev_ts = None
    start = last_now = None
    deltas = {}                       # ts-delta -> occurrence count

    ser.reset_input_buffer()          # drop anything already buffered
    print(f"Measuring {args.seconds:.1f}s on {args.port} @ {args.baud} baud ...")

    with ser:
        while True:
            raw = ser.readline()
            now = time.perf_counter()
            if not raw:
                if start is not None and now >= start + args.seconds:
                    break
                continue
            byte_count += len(raw)
            line = raw.decode("ascii", errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 11:
                bad_lines += 1
                continue
            try:
                ts = int(parts[0])
            except ValueError:
                bad_lines += 1
                continue

            # First valid line starts the clock but isn't counted (we time
            # from it, so the byte/line counts that follow are within window).
            if start is None:
                start = now
                first_ts = prev_ts = ts
                byte_count = 0
                continue

            lines += 1
            last_ts = ts
            last_now = now
            d = ts - prev_ts
            deltas[d] = deltas.get(d, 0) + 1
            prev_ts = ts

            if now >= start + args.seconds:
                break

    if start is None or lines == 0 or last_now is None:
        print("No valid 11-field data lines received. Check port / baud / wiring,")
        print("and that the firmware is flashed and streaming.")
        return

    wall = last_now - start
    host_rate = lines / wall
    ts_span = last_ts - first_ts
    mcu_rate = (ts_span / ADC_AVG_N) / wall if ts_span > 0 else 0.0
    mcu_implied_s = ts_span * RAW_SAMPLE_US * 1e-6
    bps = byte_count / wall

    print()
    print(f"  Host lines received : {lines}")
    print(f"  Wall-clock window   : {wall:.2f} s")
    print(f"  Host line rate      : {host_rate:6.1f} lines/s")
    print()
    print(f"  Timestamp span      : {ts_span} units "
          f"(implies {mcu_implied_s:.2f} s at {RAW_SAMPLE_US:.0f}us/unit)")
    print(f"  MCU frame-gen rate  : {mcu_rate:6.1f} frames/s "
          f"(expected ~{NOMINAL_HZ:.0f})")
    print()
    print(f"  Bytes received      : {byte_count}  "
          f"({bps:.0f} B/s, {100*bps*10/args.baud:.0f}% of {args.baud} baud)")
    print(f"  Bad/partial lines   : {bad_lines}")
    print()
    print("  Timestamp delta histogram (units between consecutive RX frames):")
    for d in sorted(deltas):
        tag = "   <- expected (no skip)" if d == ADC_AVG_N else ""
        print(f"     delta={d:6d} : {deltas[d]:5d} x{tag}")
    print()

    print("  Verdict:")
    if mcu_rate < NOMINAL_HZ * 0.6:
        print(f"    Firmware is GENERATING only ~{mcu_rate:.0f} frames/s, well under"
              f" ~{NOMINAL_HZ:.0f}.")
        print( "    Bottleneck is the SD24 sample rate / averaging, NOT the UART.")
        print( "    -> Check the SD24 clock config and ADC_AVG_N in main.c.")
    elif host_rate < mcu_rate * 0.8:
        print(f"    Firmware generates ~{mcu_rate:.0f} Hz but host only caught "
              f"~{host_rate:.0f} Hz.")
        print( "    Frames are lost in transport/host (slow reader or buffer")
        print( "    overflow), or the firmware send loop is skipping (deltas > 32).")
    else:
        print(f"    Healthy: firmware ~{mcu_rate:.0f} Hz, host received "
              f"~{host_rate:.0f} Hz.")
        print( "    The data rate is fine. Any '~15 Hz' you saw was the display /")
        print( "    refresh rate of your terminal or the viz tool, not the link.")


if __name__ == "__main__":
    main()
