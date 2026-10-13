"""
Shared output for lap planners: speed-profile CSV, report, plots and the
console summary.

Any script can use it:

    from lap_output import (write_speed_profile, read_speed_profile,
                            plot_speed_vs_distance, plot_track_speed_map,
                            write_report, print_summary)

    plot_track_speed_map("track_speed_map.png", P, v)   # track colour graph

Nothing here plans or checks a lap. Vehicle limits are passed in as a dict
with keys "v_max", "a_acc", "a_brk" and "a_lat" so this file never needs to
import a particular planner.
"""

import os

import numpy as np

DECIMALS = 12     # brief requires >= 9


# ---------------------------------------------------------------- CSV
def write_speed_profile(path, v, decimals=DECIMALS):
    """Write speed_profile.csv (point_id,speed_mps), one row per point.

    Speeds are rounded DOWN so written values never exceed the planned
    (feasible) ones.
    """
    scale = 10 ** decimals
    v_out = np.floor(np.asarray(v, dtype=float) * scale) / scale
    with open(path, "w", newline="") as f:
        f.write("point_id,speed_mps\n")
        for i, s in enumerate(v_out):
            f.write(f"{i},{s:.{decimals}f}\n")


def read_speed_profile(path, n=None):
    """Read speed_profile.csv and return the speeds as an array.

    If n is given, checks there are exactly n rows with point_id 0..n-1 in
    order.
    """
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    ids = data[:, 0].astype(int)
    expected = len(ids) if n is None else n
    if len(ids) != expected or not np.array_equal(ids, np.arange(expected)):
        raise ValueError(f"{path} must have point_id 0..N-1 in order")
    return data[:, 1]


# ---------------------------------------------------------------- plots
def _pyplot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_speed_vs_distance(path, ds, v, v_cap=None, v_max=70.0):
    """Speed against distance from the timing line, including the closing
    segment. v_cap (optional) is drawn as the pointwise cornering cap."""
    plt = _pyplot()
    s = np.concatenate([[0.0], np.cumsum(ds)])
    v_loop = np.append(v, v[0])

    fig, ax = plt.subplots(figsize=(12, 4.5))
    if v_cap is not None:
        cap_loop = np.append(v_cap, v_cap[0])
        ax.plot(s, np.minimum(cap_loop, v_max), color="0.75", lw=1,
                label="Cornering / top-speed cap")
    ax.plot(s, v_loop, color="tab:red", lw=1.6, label="Planned speed")
    ax.set_xlabel("Distance from timing line (m)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlim(0, s[-1])
    ax.set_ylim(0, v_max * 1.08)
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right")
    ax.set_title("Speed vs distance")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_track_speed_map(path, P, v, v_max=70.0,
                         title="Track coloured by planned speed"):
    """Centreline P (N x 2) drawn as a closed lap, each segment coloured by
    the mean speed at its two ends, with a colourbar and point 0 marked."""
    plt = _pyplot()
    from matplotlib.collections import LineCollection

    P = np.asarray(P, dtype=float)
    v = np.asarray(v, dtype=float)
    loop = np.vstack([P, P[:1]])
    segs = np.stack([loop[:-1], loop[1:]], axis=1)
    seg_speed = 0.5 * (v + np.roll(v, -1))

    fig, ax = plt.subplots(figsize=(9, 8))
    lc = LineCollection(segs, cmap="plasma", linewidths=3.5, capstyle="round")
    lc.set_array(seg_speed)
    lc.set_clim(0, v_max)
    ax.add_collection(lc)
    ax.plot(*P[0], marker="o", color="k", ms=7, label="Timing line (point 0)")
    ax.annotate("", xy=P[min(5, len(P) - 1)], xytext=P[0],
                arrowprops=dict(arrowstyle="->", color="k"))
    ax.set_aspect("equal")
    ax.autoscale()
    ax.margins(0.05)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="best")
    cb = fig.colorbar(lc, ax=ax, shrink=0.8)
    cb.set_label("Speed (m/s)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def make_plots(out_dir, P, ds, v, v_cap=None, v_max=70.0):
    """Both submission plots, saved into out_dir."""
    plot_speed_vs_distance(os.path.join(out_dir, "speed_vs_distance.png"),
                           ds, v, v_cap, v_max)
    plot_track_speed_map(os.path.join(out_dir, "track_speed_map.png"),
                         P, v, v_max)


# ---------------------------------------------------------------- report
def write_report(path, r, n, length, limits):
    """Write report.md from the results dict r (as returned by the planner's
    evaluate()) and the vehicle limits dict."""
    L = limits
    text = f"""# Challenge 1: Fastest Centreline Lap

**Lap time: {r['lap_time']:.6f} s**  ({n} points, lap length {length:.3f} m)

## Approach

1. **Geometry.** Read the {n} points in their supplied order and close the lap
   from the last point back to point 0. Segment lengths are the straight-line
   distances between consecutive points, including the closing segment.
2. **Curvature.** At each point, curvature is taken from the circle through the
   previous, current and next points (indices wrapping round the lap):
   `kappa_i = 2 |(p_i - p_(i-1)) x (p_(i+1) - p_i)| / (a b c)`, where a, b and
   c are the three side lengths of that triangle.
3. **Speed caps.** Lateral acceleration at a point is `v^2 kappa`, so each
   point is capped at `min({L['v_max']:g}, sqrt({L['a_lat']:g} / kappa))`.
4. **Acceleration and braking.** Each segment is driven at constant
   longitudinal acceleration `a = (v_(i+1)^2 - v_i^2) / (2 ds)`. A forward pass
   enforces `v_(i+1)^2 <= v_i^2 + 2({L['a_acc']:g})ds` and a backward pass
   enforces `v_i^2 <= v_(i+1)^2 + 2({L['a_brk']:g})ds`. The passes wrap round the
   closed lap and repeat until nothing changes, so the speed returning to
   point 0 equals the starting speed. Starting from the caps, the passes only
   ever lower speeds, so the result is the fastest profile meeting every limit
   at every point.
5. **Timing.** Each segment takes `2 ds / (v_i + v_(i+1))` seconds, and the lap
   time is the sum over all {n} segments. All figures in this report are
   recomputed from the saved `speed_profile.csv`.

A margin of 1e-9 (relative) is applied to every limit, and written speeds are
rounded down, so rounding in the CSV cannot break a limit.

## Peak values in the submitted profile

| Quantity | Value | Limit |
| --- | --- | --- |
| Maximum speed | {r['v_max']:.9f} m/s | {L['v_max']} m/s |
| Minimum speed | {r['v_min']:.9f} m/s | >= 0 m/s |
| Maximum forward acceleration | {r['a_fwd_max']:.9f} m/s^2 | {L['a_acc']} m/s^2 |
| Maximum braking magnitude | {r['a_brk_max']:.9f} m/s^2 | {L['a_brk']} m/s^2 |
| Maximum lateral acceleration | {r['a_lat_max']:.9f} m/s^2 | {L['a_lat']} m/s^2 |

## Plots

- `speed_vs_distance.png`: planned speed and pointwise cap against distance
- `track_speed_map.png`: centreline coloured by speed
"""
    with open(path, "w") as f:
        f.write(text)


# ---------------------------------------------------------------- console
def print_summary(r, n, length, problems=()):
    print(f"Points:            {n}")
    print(f"Lap length:        {length:.3f} m")
    print(f"Lap time:          {r['lap_time']:.6f} s")
    print(f"Max speed:         {r['v_max']:.6f} m/s")
    print(f"Min speed:         {r['v_min']:.6f} m/s")
    print(f"Max forward accel: {r['a_fwd_max']:.6f} m/s^2")
    print(f"Max braking:       {r['a_brk_max']:.6f} m/s^2")
    print(f"Max lateral accel: {r['a_lat_max']:.6f} m/s^2")
    if problems:
        print("LIMIT VIOLATIONS: " + "; ".join(problems))
    else:
        print("All limits satisfied.")
