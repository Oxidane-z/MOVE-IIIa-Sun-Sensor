"""Sun-sensor intrinsic calibration -- offline model fit (radial / Option-B bench).

Fits the per-sensor map from the *measured normalized quadrant centroid*
(x_c, y_c) to the *true* gnomonic sun angles (u = tan alpha, v = tan beta) --
the trig-free quantity the MSP430i2041 firmware already consumes
(compute_sun_vector: u = x_c * K_GEOM -> closed-form unit vector).  Calibrating
in (u, v) space keeps the runtime path free of atan/trig, which is mandatory on
the FPU-less i2041.

Bench geometry (Option B, radial):
    X stage = precise geared tilt  -> OFF-AXIS ANGLE (radius) theta
    Y stage = boresight rotation    -> AZIMUTH phi
  True sun direction in the sensor frame is then clean spherical:
    s = (sin th cos ph,  sin th sin ph,  cos th)
    u_true = tan(th) cos(ph),   v_true = tan(th) sin(ph)
  i.e. NO stacked-gimbal 1/cos cross-coupling -- that is the whole point of B.

Pipeline:
    raw A0..A3 --(dark subtract, gain)--> centroid (x_c, y_c)
    stage (tilt, azim) --(+ boresight/azimuth offsets)--> truth (u_true, v_true)
    fit (x_c, y_c) -> (u_true, v_true) with a 2-D polynomial (cross terms),
    pick the degree by k-fold CV (guards against the old code's fixed order-7
    overfit), then export trig-free C coefficient tables for the firmware.

Input CSV columns (produced by the capture orchestrator -- schema is ours):
    utc_iso, tt_tilt_deg, tt_azim_deg, mcu_ts,
    A0, A1, A2, A3, sx, sy, sz, sum, temp_c100, flags
The dark CSV only needs A0..A3 (light blocked).

Quadrant map matches compute_sun_vector():  A0=BL, A1=TL, A2=TR, A3=BR.

Usage:
    # validate the machinery with synthetic data (no hardware needed):
    python fit_calibration.py --self-test

    # real run once you have data:
    python fit_calibration.py --data sweep.csv --dark dark.csv \
        --tilt0 0.3 --auto-azim0 --out-c sun_calib.h --plots

Dependencies:  pip install numpy   (matplotlib only needed for --plots)
"""

import argparse
import csv
import sys

import numpy as np

K_GEOM = 1.625            # firmware's nominal pinhole constant (a / 2h); linear baseline


# ----- centroid <-> quadrants ----------------------------------------------

def centroid_from_quadrants(A, dark, gain):
    """A: (...,4) raw counts A0..A3 (BL,TL,TR,BR). -> x_c, y_c, sum (intensity-normalized)."""
    p = gain * (A.astype(float) - dark)
    p0, p1, p2, p3 = p[..., 0], p[..., 1], p[..., 2], p[..., 3]
    s = p0 + p1 + p2 + p3
    x_c = ((p2 + p3) - (p1 + p0)) / s        # right column - left column
    y_c = ((p1 + p2) - (p0 + p3)) / s        # top row    - bottom row
    return x_c, y_c, s


def quadrants_from_centroid(x_c, y_c, total, dark, gain):
    """Inverse used only by the self-test: build A0..A3 that reproduce a centroid.

    Underdetermined (3 eqns, 4 unknowns); we pin the 4th (saddle) mode to 0, the
    natural choice for a symmetric spot.  Hadamard solve."""
    S = total
    X = S * x_c                              # (p2+p3)-(p1+p0)
    Y = S * y_c                              # (p1+p2)-(p0+p3)
    D = 0.0                                  # (p0+p2)-(p1+p3) saddle = 0
    p0 = (S - X - Y + D) / 4.0
    p1 = (S - X + Y - D) / 4.0
    p2 = (S + X + Y + D) / 4.0
    p3 = (S + X - Y - D) / 4.0
    p = np.stack([p0, p1, p2, p3], axis=-1)
    return p / gain + dark                    # invert p = gain*(A-dark)


# ----- bench kinematics: stage angles -> true (u, v) -----------------------

def stage_to_truth_radial(tilt_deg, azim_deg, tilt0_deg=0.0, azim0_deg=0.0):
    """Option B: X=tilt(off-axis radius), Y=azimuth.  Clean spherical truth."""
    th = np.radians(tilt_deg - tilt0_deg)
    ph = np.radians(azim_deg - azim0_deg)
    sx = np.sin(th) * np.cos(ph)
    sy = np.sin(th) * np.sin(ph)
    sz = np.cos(th)
    return sx / sz, sy / sz                   # u = tan(alpha), v = tan(beta)


# ----- 2-D polynomial (total degree, all cross terms) ----------------------

def poly_exponents(degree):
    return [(i, n - i) for n in range(degree + 1) for i in range(n + 1)]


def poly_design(x, y, exps):
    return np.column_stack([(x ** i) * (y ** j) for (i, j) in exps])


def poly_fit(x, y, target, degree):
    exps = poly_exponents(degree)
    coef, *_ = np.linalg.lstsq(poly_design(x, y, exps), target, rcond=None)
    return coef, exps


def poly_eval(coef, exps, x, y):
    out = np.zeros(np.shape(x), dtype=float)
    for c, (i, j) in zip(coef, exps):
        out = out + c * (x ** i) * (y ** j)
    return out


# ----- error metric: total angular error between unit vectors --------------

def angular_error_deg(u_pred, v_pred, u_true, v_true):
    def unit(u, v):
        n = np.sqrt(1.0 + u * u + v * v)
        return np.stack([u / n, v / n, 1.0 / n], axis=-1)
    a, b = unit(u_pred, v_pred), unit(u_true, v_true)
    dot = np.clip(np.sum(a * b, axis=-1), -1.0, 1.0)
    return np.degrees(np.arccos(dot))


def rms(e):
    return float(np.sqrt(np.mean(np.square(e))))


# ----- k-fold CV degree selection ------------------------------------------

def kfold_angular_rms(xc, yc, u_true, v_true, degree, k=5, seed=0):
    """Held-out total-angular RMS (deg) for a given polynomial degree."""
    idx = np.random.default_rng(seed).permutation(len(xc))
    folds = np.array_split(idx, k)
    errs = []
    for f in range(k):
        te = folds[f]
        tr = np.concatenate([folds[g] for g in range(k) if g != f])
        cu, ex = poly_fit(xc[tr], yc[tr], u_true[tr], degree)
        cv, _ = poly_fit(xc[tr], yc[tr], v_true[tr], degree)
        up = poly_eval(cu, ex, xc[te], yc[te])
        vp = poly_eval(cv, ex, xc[te], yc[te])
        errs.append(angular_error_deg(up, vp, u_true[te], v_true[te]))
    return rms(np.concatenate(errs))


def select_degree(xc, yc, u_true, v_true, degmin=2, degmax=6, k=5):
    rows = []
    for d in range(degmin, degmax + 1):
        cv = kfold_angular_rms(xc, yc, u_true, v_true, d, k=k)
        cu, ex = poly_fit(xc, yc, u_true, d)
        cvc, _ = poly_fit(xc, yc, v_true, d)
        tr = rms(angular_error_deg(poly_eval(cu, ex, xc, yc),
                                   poly_eval(cvc, ex, xc, yc), u_true, v_true))
        rows.append((d, len(ex), tr, cv))
    best = min(rows, key=lambda r: r[3])
    return best[0], rows


# ----- azimuth-zero auto-fit (1-D scan) ------------------------------------

def auto_azim0(tilt, azim, xc, yc, tilt0, degree=4):
    """Find the azimuth reference that minimises CV residual (orientation offset)."""
    best_a0, best_rms = 0.0, np.inf
    for a0 in np.arange(0.0, 360.0, 2.0):
        u_t, v_t = stage_to_truth_radial(tilt, azim, tilt0, a0)
        r = kfold_angular_rms(xc, yc, u_t, v_t, degree)
        if r < best_rms:
            best_rms, best_a0 = r, a0
    return best_a0


# ----- CSV I/O --------------------------------------------------------------

def _read_cols(path, cols):
    out = {c: [] for c in cols}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for c in cols:
                out[c].append(float(row[c]))
    return {c: np.array(v) for c, v in out.items()}


def load_sweep_csv(path):
    d = _read_cols(path, ["tt_tilt_deg", "tt_azim_deg", "A0", "A1", "A2", "A3"])
    A = np.stack([d["A0"], d["A1"], d["A2"], d["A3"]], axis=-1)
    return d["tt_tilt_deg"], d["tt_azim_deg"], A


def load_dark_csv(path):
    d = _read_cols(path, ["A0", "A1", "A2", "A3"])
    return np.stack([d["A0"], d["A1"], d["A2"], d["A3"]], axis=-1).mean(axis=0)


def group_mean(tilt, azim, xc, yc):
    """Average repeated samples at each (tilt, azim) grid point."""
    keys = {}
    for t, a, x, y in zip(tilt, azim, xc, yc):
        keys.setdefault((round(t, 3), round(a, 3)), []).append((t, a, x, y))
    T, Az, X, Y = [], [], [], []
    for vals in keys.values():
        v = np.array(vals)
        T.append(v[:, 0].mean()); Az.append(v[:, 1].mean())
        X.append(v[:, 2].mean()); Y.append(v[:, 3].mean())
    return np.array(T), np.array(Az), np.array(X), np.array(Y)


# ----- C export -------------------------------------------------------------

def export_c(path, degree, exps, cu, cv):
    ex = [i for (i, j) in exps]
    ey = [j for (i, j) in exps]
    n = len(exps)

    def arr(vals, fmt):
        return ", ".join(fmt % v for v in vals)

    txt = f"""// Auto-generated by fit_calibration.py -- per-sensor intrinsic calibration.
// Maps measured centroid (x_c, y_c) -> (u = tan alpha, v = tan beta).
// Feed u, v into the existing closed-form unit vector in compute_sun_vector().
// Trig-free (only float mul/add) -- safe for the FPU-less MSP430i2041 per-frame path.
#ifndef SUN_CALIB_H
#define SUN_CALIB_H
#include <stdint.h>

#define SUN_CALIB_PRESENT 1
#define SUN_CALIB_DEG     {degree}
#define SUN_CALIB_NTERMS  {n}

static const uint8_t sun_calib_ex[SUN_CALIB_NTERMS] = {{ {arr(ex, '%d')} }};
static const uint8_t sun_calib_ey[SUN_CALIB_NTERMS] = {{ {arr(ey, '%d')} }};
static const float   sun_calib_cu[SUN_CALIB_NTERMS] = {{ {arr(cu, '%.8ef')} }};
static const float   sun_calib_cv[SUN_CALIB_NTERMS] = {{ {arr(cv, '%.8ef')} }};

// u = sum_k cu[k] * x_c^ex[k] * y_c^ey[k];  v likewise with cv[].
static inline void sun_apply_calib(float xc, float yc, float *u, float *v)
{{
    float xp[SUN_CALIB_DEG + 1], yp[SUN_CALIB_DEG + 1];
    xp[0] = 1.0f; yp[0] = 1.0f;
    for (uint8_t i = 1; i <= SUN_CALIB_DEG; i++) {{ xp[i] = xp[i-1]*xc; yp[i] = yp[i-1]*yc; }}
    float su = 0.0f, sv = 0.0f;
    for (uint8_t k = 0; k < SUN_CALIB_NTERMS; k++) {{
        float t = xp[sun_calib_ex[k]] * yp[sun_calib_ey[k]];
        su += sun_calib_cu[k] * t;
        sv += sun_calib_cv[k] * t;
    }}
    *u = su; *v = sv;
}}
#endif // SUN_CALIB_H
"""
    with open(path, "w") as f:
        f.write(txt)
    return n


# ----- plots ----------------------------------------------------------------

def make_plots(xc, yc, u_true, v_true, coef_u, coef_v, exps, prefix="calib"):
    import matplotlib.pyplot as plt
    base = angular_error_deg(xc * K_GEOM, yc * K_GEOM, u_true, v_true)
    cal = angular_error_deg(poly_eval(coef_u, exps, xc, yc),
                            poly_eval(coef_v, exps, xc, yc), u_true, v_true)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5), subplot_kw={"projection": "3d"})
    for a, e, ttl in ((ax[0], base, f"linear K_GEOM  (RMS {rms(base):.3f} deg)"),
                      (ax[1], cal, f"calibrated     (RMS {rms(cal):.3f} deg)")):
        a.scatter(xc, yc, e, c=e, s=6)
        a.set_xlabel("x_c"); a.set_ylabel("y_c"); a.set_zlabel("angular err [deg]")
        a.set_title(ttl)
    fig.tight_layout(); fig.savefig(prefix + "_residual.png", dpi=120)
    print(f"  wrote {prefix}_residual.png")


# ----- real-data fit driver -------------------------------------------------

def run_fit(args):
    tilt, azim, A = load_sweep_csv(args.data)
    dark = load_dark_csv(args.dark) if args.dark else np.zeros(4)
    gain = np.ones(4)
    print(f"loaded {len(tilt)} samples; dark={dark}")

    xc, yc, s = centroid_from_quadrants(A, dark, gain)
    ok = np.isfinite(xc) & np.isfinite(yc) & (s > 0)
    tilt, azim, xc, yc = tilt[ok], azim[ok], xc[ok], yc[ok]
    tilt, azim, xc, yc = group_mean(tilt, azim, xc, yc)
    print(f"{len(xc)} unique grid points after averaging repeats")

    azim0 = auto_azim0(tilt, azim, xc, yc, args.tilt0) if args.auto_azim0 else args.azim0
    if args.auto_azim0:
        print(f"auto azim0 = {azim0:.1f} deg")
    u_true, v_true = stage_to_truth_radial(tilt, azim, args.tilt0, azim0)

    deg, table = select_degree(xc, yc, u_true, v_true,
                               degmin=args.degree_min, degmax=args.degree_max)
    print("\n deg  terms  train_RMS  CV_RMS  [deg]")
    for d, nt, tr, cv in table:
        mark = " <-- pick" if d == deg else ""
        print(f"  {d:2d}   {nt:3d}   {tr:8.4f}  {cv:7.4f}{mark}")

    base = rms(angular_error_deg(xc * K_GEOM, yc * K_GEOM, u_true, v_true))
    cu, exps = poly_fit(xc, yc, u_true, deg)
    cv, _ = poly_fit(xc, yc, v_true, deg)
    cal = rms(angular_error_deg(poly_eval(cu, exps, xc, yc),
                                poly_eval(cv, exps, xc, yc), u_true, v_true))
    print(f"\nlinear-K baseline RMS: {base:.3f} deg   calibrated RMS: {cal:.3f} deg")

    if args.out_c:
        n = export_c(args.out_c, deg, exps, cu, cv)
        print(f"wrote {args.out_c}  ({deg=}, {n} terms, {2*n*4} bytes coeffs)")
    if args.plots:
        make_plots(xc, yc, u_true, v_true, cu, cv, exps)
    return 0


# ----- synthetic self-test --------------------------------------------------

def self_test():
    """Synthesize distorted centroids from known true angles; confirm recovery."""
    rng = np.random.default_rng(1)
    tilts = np.arange(0.0, 57.0, 7.0)            # rings 0..56 deg
    azims = np.arange(0.0, 360.0, 15.0)
    T, Az = [], []
    for t in tilts:
        for a in (azims if t > 0 else [0.0]):    # center degenerate in azimuth
            for _ in range(5):                    # 5 repeats / point
                T.append(t); Az.append(a)
    tilt = np.array(T); azim = np.array(Az)

    # ground truth (u,v) with a known azimuth reference of 0
    u_t, v_t = stage_to_truth_radial(tilt, azim, 0.0, 0.0)

    # forward distortion: a realistic radial barrel + tiny asymmetry on the
    # IDEAL centroid (u/K, v/K), then back out A0..A3 to exercise that path too.
    xc_i, yc_i = u_t / K_GEOM, v_t / K_GEOM
    rho = np.sqrt(xc_i**2 + yc_i**2)
    warp = 1.0 + 0.10 * rho - 0.06 * rho**2
    xc_real = xc_i * warp + 0.01 * yc_i**2
    yc_real = yc_i * warp - 0.01 * xc_i**2
    dark, gain, total = np.zeros(4), np.ones(4), 20000.0
    A = quadrants_from_centroid(xc_real, yc_real, total, dark, gain)
    A = np.round(A + rng.normal(0, 6.0, A.shape))   # ADC quantization + noise
    xc, yc, _ = centroid_from_quadrants(A, dark, gain)

    tilt_g, azim_g, xc_g, yc_g = group_mean(tilt, azim, xc, yc)
    u_g, v_g = stage_to_truth_radial(tilt_g, azim_g, 0.0, 0.0)

    deg, table = select_degree(xc_g, yc_g, u_g, v_g)
    print(" deg  terms  train_RMS  CV_RMS  [deg]")
    for d, nt, tr, cv in table:
        print(f"  {d:2d}   {nt:3d}   {tr:8.4f}  {cv:7.4f}{'  <-- pick' if d==deg else ''}")

    base = rms(angular_error_deg(xc_g * K_GEOM, yc_g * K_GEOM, u_g, v_g))
    cu, exps = poly_fit(xc_g, yc_g, u_g, deg)
    cv, _ = poly_fit(xc_g, yc_g, v_g, deg)
    cal = rms(angular_error_deg(poly_eval(cu, exps, xc_g, yc_g),
                                poly_eval(cv, exps, xc_g, yc_g), u_g, v_g))
    print(f"\nlinear-K baseline RMS: {base:.3f} deg   calibrated RMS: {cal:.4f} deg")

    n = export_c("sun_calib_selftest.h", deg, exps, cu, cv)
    print(f"C export OK: sun_calib_selftest.h ({n} terms)")

    ok = (cal < 0.1) and (cal < base / 3.0)
    print(f"\nSELF-TEST {'PASS' if ok else 'FAIL'} "
          f"(calibrated {cal:.3f} deg vs baseline {base:.3f} deg)")
    return 0 if ok else 1


# ----- CLI ------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", help="sweep CSV")
    ap.add_argument("--dark", help="dark CSV (A0..A3 with light blocked)")
    ap.add_argument("--tilt0", type=float, default=0.0,
                    help="boresight tilt offset [deg] from the auto-zero step")
    ap.add_argument("--azim0", type=float, default=0.0, help="azimuth reference [deg]")
    ap.add_argument("--auto-azim0", action="store_true", help="fit azimuth reference")
    ap.add_argument("--degree-min", type=int, default=2)
    ap.add_argument("--degree-max", type=int, default=6)
    ap.add_argument("--out-c", help="write C coefficient header here")
    ap.add_argument("--plots", action="store_true", help="save residual plots (matplotlib)")
    ap.add_argument("--self-test", action="store_true", help="run synthetic validation")
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not args.data:
        ap.error("need --data (or --self-test)")
    return run_fit(args)


if __name__ == "__main__":
    sys.exit(main())
