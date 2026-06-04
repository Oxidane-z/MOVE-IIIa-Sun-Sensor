"""Bulletproof serial logger -- owns the COM port, writes every line to a file.

This is meant to be the ONLY program that opens the serial port.  Run it once;
point the 3D viz and any analysis tools at the FILE it produces (the viz has a
`--file` option).  Because nothing else touches the port, a crash in the viz or
any other reader can never lock the port -- only this tiny, dependency-light
process holds it, and it always releases it on exit (Ctrl+C, error, or normal).

Usage:
    python serial_log.py --port COM19 --out sensor.log
    python serial_log.py --port COM19 --out sensor.log --echo   # also to stdout

Then, in another terminal:
    python sun_sensor_viz.py --file sensor.log
"""

import argparse
import sys
import serial


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port, e.g. COM19")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default="sensor.log", help="output log file")
    ap.add_argument("--echo", action="store_true",
                    help="also echo each line to stdout")
    args = ap.parse_args()

    ser = None
    fh = None
    n = 0
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        # line-buffered text file so other readers see lines as they arrive
        fh = open(args.out, "w", buffering=1, encoding="ascii", errors="ignore")
        print(f"logging {args.port} @ {args.baud} -> {args.out}  (Ctrl+C to stop)")
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore")
            fh.write(line)
            n += 1
            if args.echo:
                sys.stdout.write(line)
    except KeyboardInterrupt:
        pass                                    # clean Ctrl+C
    except serial.SerialException as e:
        print(f"serial error: {e}", file=sys.stderr)
    finally:
        # ALWAYS release the port and flush the file, no matter how we exit.
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if fh is not None:
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        print(f"\nstopped. {n} lines written to {args.out}. serial port released.")


if __name__ == "__main__":
    main()
