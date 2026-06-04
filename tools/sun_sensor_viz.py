"""Live 3D visualizer for the quadrant sun sensor.

Reads lines of the form
    <timestamp> <A0> <A1> <A2> <A3> <sx_x10000> <sy_x10000> <sz_x10000> <sum> <temp_c100> <flags>
from the MSP430i2041's UART (921600 8-N-1 by default) and plots the unit
sun vector in the sensor body frame, with the photodiode plane drawn as
a reference plane in isometric perspective.  Raw ADC counts are shown in
the title so you can see when the algorithm flags no_sun.

`temp_c100` is the on-chip computed temperature in centi-degrees Celsius
(int16, 0.01 °C/LSB): T_celsius = temp_c100 / 100.0.  The sentinel 0x8000
(-32768) means the sensor did not respond (invalid).

Usage:
    # RECOMMENDED: let serial_log.py own the port; the viz reads its file.
    # A viz crash then can NEVER lock the COM port for other programs.
    #   terminal 1:  python serial_log.py --port COM19 --out sensor.log
    #   terminal 2:  python sun_sensor_viz.py --file sensor.log
    #
    # Direct (viz owns the port).  Cleanup is hardened (try/finally) so the
    # port is released on window-close / Ctrl+C / exceptions, but a hard
    # backend segfault is still best avoided via the --file path above.
    #   python sun_sensor_viz.py --port COM19
    #   python sun_sensor_viz.py --port /dev/ttyUSB0 --baud 115200

Dependencies (install once):
    pip install pyserial matplotlib numpy
"""

import argparse
import sys
import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d.proj3d import proj_transform
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)
import serial

TRAIL_LEN = 500


FLAG_NO_SUN = 1
FLAG_OFF_FOV = 2
FLAG_SATURATED = 4


# ----- line parsing --------------------------------------------------------

def parse_line(line):
    """Parse one firmware output line into a tuple, or None if not a data row."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split()
    if len(parts) != 11:
        return None
    try:
        ts = int(parts[0])
        a0 = int(parts[1]); a1 = int(parts[2])
        a2 = int(parts[3]); a3 = int(parts[4])
        sx = int(parts[5]) / 10000.0
        sy = int(parts[6]) / 10000.0
        sz = int(parts[7]) / 10000.0
        s_sum = int(parts[8])
        temp_c100 = int(parts[9])
        flags = int(parts[10])
    except ValueError:
        return None
    return (ts, a0, a1, a2, a3, sx, sy, sz, s_sum, temp_c100, flags)


# ----- readers (background threads) ----------------------------------------

class _Reader(threading.Thread):
    """Common latest()/error()/close() plumbing for the concrete readers."""

    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._error = None

    def _publish(self, parsed):
        with self._lock:
            self._latest = parsed

    def latest(self):
        with self._lock:
            return self._latest

    def error(self):
        return self._error

    def stop(self):
        self._stop.set()

    def close(self):
        self.stop()


class SerialReader(_Reader):
    """Reads the serial port directly.  Guarantees the port is released on
    stop()/close() so it can't stay locked if the GUI thread dies."""

    def __init__(self, port, baud):
        super().__init__()
        self._port = port
        self._baud = baud
        self._ser = None

    def run(self):
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=1)
        except serial.SerialException as e:
            self._error = f"open {self._port}: {e}"
            return
        try:
            while not self._stop.is_set():
                try:
                    raw = self._ser.readline()
                except serial.SerialException as e:
                    self._error = f"read: {e}"
                    return
                if not raw:
                    continue
                parsed = parse_line(raw.decode("ascii", errors="ignore"))
                if parsed:
                    self._publish(parsed)
        finally:
            try:
                self._ser.close()
            except Exception:
                pass

    def close(self):
        self._stop.set()
        if self._ser is not None:
            try:
                self._ser.close()       # unblocks readline() in the thread
            except Exception:
                pass


class FileReader(_Reader):
    """Tail-follows a log file written by serial_log.py.  Never opens the
    serial port, so a crash here cannot lock it."""

    def __init__(self, path):
        super().__init__()
        self._path = path

    def run(self):
        try:
            f = open(self._path, "r", encoding="ascii", errors="ignore")
        except OSError as e:
            self._error = f"open {self._path}: {e}"
            return
        with f:
            f.seek(0, 2)                # start at end of file (live tail)
            while not self._stop.is_set():
                line = f.readline()
                if not line:
                    time.sleep(0.01)    # caught up; wait for more
                    continue
                parsed = parse_line(line)
                if parsed:
                    self._publish(parsed)


# ----- 3D arrow that survives axis redraws ---------------------------------

class Arrow3D(FancyArrowPatch):
    """3D arrow with a proper head; reuses one matplotlib artist."""

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def set_endpoints(self, x, y, z, dx, dy, dz):
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = x1 + dx, y1 + dy, z1 + dz
        xs, ys, _ = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min((z1, z2))


# ----- Plot setup ----------------------------------------------------------

def make_figure():
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Isometric-ish view
    ax.view_init(elev=25, azim=-55)

    lim = 1.2
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-0.2, lim)
    ax.set_box_aspect((1, 1, 0.7))

    ax.set_xlabel("X  (sensor right)")
    ax.set_ylabel("Y  (sensor top)")
    ax.set_zlabel("Z  (sensor normal)")

    # Reference plane (photodiode surface) at z=0, with a fine grid
    plane = 1.0
    g = np.linspace(-plane, plane, 11)
    gx, gy = np.meshgrid(g, g)
    ax.plot_surface(gx, gy, np.zeros_like(gx),
                    color="0.7", alpha=0.18, edgecolor="0.6", linewidth=0.3)

    # Sensor body axes (short colored sticks at origin)
    axis_len = 0.35
    ax.plot([0, axis_len], [0, 0], [0, 0], color="tab:red", lw=1.5)    # +X
    ax.plot([0, 0], [0, axis_len], [0, 0], color="tab:green", lw=1.5)  # +Y
    ax.plot([0, 0], [0, 0], [0, axis_len], color="tab:blue", lw=1.5)   # +Z

    # Quadrant labels on the reference plane
    ax.text(-0.5,  0.5, 0.01, "TL\n(a1)", color="0.4", fontsize=8, ha="center")
    ax.text( 0.5,  0.5, 0.01, "TR\n(a2)", color="0.4", fontsize=8, ha="center")
    ax.text(-0.5, -0.5, 0.01, "BL\n(a0)", color="0.4", fontsize=8, ha="center")
    ax.text( 0.5, -0.5, 0.01, "BR\n(a3)", color="0.4", fontsize=8, ha="center")

    # Trail of recent tip positions (drawn before the arrow so it sits behind)
    (trail_line,) = ax.plot([], [], [], color="tab:orange", alpha=0.35,
                            lw=1.0, solid_capstyle="round")

    # The sun vector arrow (we mutate this in place each frame)
    arrow = Arrow3D(0, 0, 0, 0, 0, 0,
                    mutation_scale=18, lw=2.5,
                    arrowstyle="-|>", color="tab:orange")
    ax.add_artist(arrow)

    # Origin dot
    ax.scatter([0], [0], [0], color="black", s=30, depthshade=False)

    title = ax.set_title("waiting for data...", fontsize=10)
    return fig, ax, arrow, trail_line, title


def flag_string(flags):
    bits = []
    if flags & FLAG_NO_SUN:    bits.append("NO_SUN")
    if flags & FLAG_OFF_FOV:   bits.append("OFF_FOV")
    if flags & FLAG_SATURATED: bits.append("SAT")
    return ", ".join(bits) if bits else "ok"


# ----- Main loop -----------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port to read directly (e.g. COM18). "
                                    "Prefer --file via serial_log.py so a viz "
                                    "crash can never lock the port.")
    src.add_argument("--file", help="tail-follow a log file written by "
                                    "serial_log.py (recommended)")
    p.add_argument("--baud", type=int, default=921600, help="baud rate (--port only)")
    p.add_argument("--refresh-ms", type=int, default=50,
                   help="plot refresh interval (ms)")
    args = p.parse_args()

    if args.file:
        reader = FileReader(args.file)
    else:
        reader = SerialReader(args.port, args.baud)
    reader.start()

    # try/finally guarantees the reader (and thus the serial port, if any) is
    # released no matter how we leave -- window close, Ctrl+C, or an exception
    # in the matplotlib backend.
    try:
        fig, ax, arrow, trail_line, title = make_figure()
        plt.ion()
        plt.show()

        trail = deque(maxlen=TRAIL_LEN)
        last_ts = None
        last_seen_at = time.time()

        while plt.fignum_exists(fig.number):
            if reader.error():
                print(f"reader error: {reader.error()}", file=sys.stderr)
                break

            try:
                sample = reader.latest()
                if sample is not None:
                    (ts, a0, a1, a2, a3, sx, sy, sz,
                     s_sum, temp_c100, flags) = sample
                    if ts != last_ts:
                        last_ts = ts
                        last_seen_at = time.time()

                        if flags & FLAG_NO_SUN:
                            arrow.set_endpoints(0, 0, 0, 0, 0, 0)
                        else:
                            arrow.set_endpoints(0, 0, 0, sx, sy, sz)
                            trail.append((sx, sy, sz))

                        if trail:
                            pts = np.asarray(trail)
                            trail_line.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])

                        if temp_c100 == -32768:  # 0x8000: sensor did not respond
                            temp_str = "T=invalid"
                        else:
                            temp_str = f"T={temp_c100 / 100.0:.2f}°C"

                        title.set_text(
                            f"t={ts}   ADC = [{a0:5d} {a1:5d} {a2:5d} {a3:5d}]"
                            f"   sum={s_sum}   {temp_str}   trail={len(trail)}\n"
                            f"S = ({sx:+.3f}, {sy:+.3f}, {sz:+.3f})   "
                            f"[{flag_string(flags)}]"
                        )

                if time.time() - last_seen_at > 2.0:
                    title.set_text(title.get_text().split("\n")[0]
                                   + "\n(no fresh data)")

                plt.pause(args.refresh_ms / 1000.0)
            except KeyboardInterrupt:
                break
            except Exception as e:
                # A transient backend/render glitch shouldn't crash the app
                # (and leave the port locked). Log and keep going.
                print(f"plot update error (continuing): {e}", file=sys.stderr)
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        src = "file" if args.file else "serial port"
        print(f"\nviz stopped; {src} released.")


if __name__ == "__main__":
    main()
