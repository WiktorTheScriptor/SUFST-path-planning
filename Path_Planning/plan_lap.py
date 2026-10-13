#!/usr/bin/env python3

import argparse
import os
import sys

import numpy as np

from track_data import load_points
from lap_output import (write_speed_profile, read_speed_profile,
                        make_plots, write_report, print_summary)

# ---------------------------------------------------------------- limits
V_MAX = 70.0      # m/s
A_ACC = 6.0       # m/s^2 forward
A_BRK = 10.0      # m/s^2 braking magnitude
A_LAT = 12.0      # m/s^2 lateral
LIMITS = {"v_max": V_MAX, "a_acc": A_ACC, "a_brk": A_BRK, "a_lat": A_LAT}

# Tiny safety margin so that floating-point rounding in the written CSV can
# never push a check over a limit. Costs well under a microsecond of lap time.
MARGIN = 1e-9


# ---------------------------------------------------------------- geometry
def segment_lengths(P):
    """ds_i = |p_(i+1) - p_i|, including the closing segment."""
    return np.linalg.norm(np.roll(P, -1, axis=0) - P, axis=1)


def curvature(P):
    """Unsigned three-point (circumcircle) curvature at every point, 1/m."""
    prev, nxt = np.roll(P, 1, axis=0), np.roll(P, -1, axis=0)
    d1, d2 = P - prev, nxt - P
    a = np.linalg.norm(d1, axis=1)
    b = np.linalg.norm(d2, axis=1)
    c = np.linalg.norm(nxt - prev, axis=1)
    cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    denom = a * b * c
    with np.errstate(divide="ignore", invalid="ignore"):
        k = np.where(denom > 0, 2.0 * np.abs(cross) / denom, 0.0)
    return k


# ---------------------------------------------------------------- planning
def speed_caps(kappa):
    a_lat = A_LAT * (1 - MARGIN)
    v_max = V_MAX * (1 - MARGIN)
    with np.errstate(divide="ignore"):
        v_corner = np.where(kappa > 0, np.sqrt(a_lat / kappa), np.inf)
    return np.minimum(v_max, v_corner)


def plan(v_cap, ds, max_sweeps=100):
    #Forward and Back Passes
    n = len(v_cap)
    v = v_cap.copy()
    acc = 2 * A_ACC * (1 - MARGIN)
    brk = 2 * A_BRK * (1 - MARGIN)
    for _ in range(max_sweeps):
        old = v.copy()
        # forward pass (acceleration limit): start at the slowest point so
        # one sweep propagates all the way round, then wrap.
        start = int(np.argmin(v))
        for k in range(n):
            i = (start + k) % n
            j = (i + 1) % n
            v[j] = min(v[j], np.sqrt(v[i] ** 2 + acc * ds[i]))
        # backward pass (braking limit)
        start = int(np.argmin(v))
        for k in range(n):
            j = (start - k) % n
            i = (j - 1) % n
            v[i] = min(v[i], np.sqrt(v[j] ** 2 + brk * ds[i]))
        if np.max(np.abs(v - old)) < 1e-13:
            break
    return np.maximum(v, 0.0)


# ---------------------------------------------------------------- checks
def evaluate(v, ds, kappa):
    """Lap time and peak values, computed exactly as a scorer would."""
    v_next = np.roll(v, -1)
    vsum = v + v_next
    if np.any(vsum <= 0):
        raise ValueError("Car stops on a segment: lap time is infinite.")
    dt = 2 * ds / vsum
    a_long = (v_next ** 2 - v ** 2) / (2 * ds)
    a_lat = v ** 2 * kappa
    return {
        "lap_time": float(dt.sum()),
        "v_max": float(v.max()),
        "v_min": float(v.min()),
        "a_fwd_max": float(max(a_long.max(), 0.0)),
        "a_brk_max": float(max(-a_long.min(), 0.0)),
        "a_lat_max": float(a_lat.max()),
        "a_long": a_long,
        "a_lat": a_lat,
    }


def check_limits(r, v, tol=1e-9):
    problems = []
    if np.any(v < 0):
        problems.append("negative speed")
    if r["v_max"] > V_MAX + tol:
        problems.append(f"top speed {r['v_max']:.9f} > {V_MAX}")
    if r["a_fwd_max"] > A_ACC + tol:
        problems.append(f"forward accel {r['a_fwd_max']:.9f} > {A_ACC}")
    if r["a_brk_max"] > A_BRK + tol:
        problems.append(f"braking {r['a_brk_max']:.9f} > {A_BRK}")
    if r["a_lat_max"] > A_LAT + tol:
        problems.append(f"lateral accel {r['a_lat_max']:.9f} > {A_LAT}")
    return problems


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("track", nargs="?", default="silverstone.csv")
    ap.add_argument("--out", default=".")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    P = load_points(args.track)
    n = len(P)
    ds = segment_lengths(P)
    if np.any(ds <= 0):
        sys.exit("Error: duplicate consecutive points (zero-length segment).")
    kappa = curvature(P)

    v_cap = speed_caps(kappa)
    v = plan(v_cap, ds)

    csv_path = os.path.join(args.out, "speed_profile.csv")
    write_speed_profile(csv_path, v)

    # Everything reported is recomputed from the saved file (authoritative).
    v_saved = read_speed_profile(csv_path, n)
    r = evaluate(v_saved, ds, kappa)
    problems = check_limits(r, v_saved)

    write_report(os.path.join(args.out, "report.md"), r, n, ds.sum(), LIMITS)
    if not args.no_plots:
        make_plots(args.out, P, ds, v_saved, v_cap, V_MAX)

    print_summary(r, n, ds.sum(), problems)
    if problems:
        sys.exit(1)

if __name__ == "__main__":
    main()
