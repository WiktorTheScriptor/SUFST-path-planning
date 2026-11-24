"""
Left/right cone classification and centreline without colour.

Pipeline:
  1. triangulate()      Delaunay triangulation of cone positions (car frame)
  2. find_seed()        first crossing of the track: the first Delaunay edge the
                        car's forward ray passes through. Its left/right cones are
                        decided relative to that ray, so it is local and robust.
  3. walk_centreline()  step forward triangle by triangle. Each new cone is put on
                        the left or right relative to the current track direction,
                        and the midpoint of every crossing becomes a centreline point.
  4. smooth_centreline() optional smoothing spline through the midpoints.

Conventions (ROS REP 103, car frame):
  x forward, y left, car at (0, 0) heading along +x.
  Side labels: +1 = left, -1 = right, 0 = unknown.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.spatial import Delaunay, QhullError

LEFT, RIGHT, UNKNOWN = 1, -1, 0


@dataclass
class Config:
    min_width: float = 2.5          # plausible track width limits [m]
    max_width: float = 6.5
    seed_min_x: float = -1.5        # first crossing may be slightly behind the car [m]
    seed_max_x: float = 6.0         # ...but not further ahead than this [m]
    max_turn_deg: float = 60.0      # max change of track direction per step [deg]
    max_length: float = 40.0        # stop the walk after this much centreline [m]
    max_steps: int = 200


@dataclass
class Seed:
    left: int                       # index of left cone of first crossing
    right: int                      # index of right cone of first crossing
    midpoint: np.ndarray            # (2,) midpoint of the crossing
    width: float                    # distance between the two cones [m]


@dataclass
class WalkResult:
    centreline: np.ndarray                          # (N, 2) ordered midpoints
    crossings: list[tuple[int, int]]                # (left, right) cone index per midpoint
    labels: np.ndarray                              # side label per cone
    stop_reason: str = ""
    widths: list[float] = field(default_factory=list)


# --------------------------------------------------------------------------
# Delaunay
# --------------------------------------------------------------------------

def triangulate(xy: np.ndarray) -> Delaunay | None:
    """Delaunay triangulation of cone positions.

    Returns None if there are too few cones or they are degenerate
    (e.g. all in a line), so callers can fall back safely.
    """
    xy = np.asarray(xy, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 3:
        return None
    try:
        return Delaunay(xy)
    except QhullError:
        return None


def unique_edges(tri: Delaunay) -> np.ndarray:
    """All unique triangle edges as an (M, 2) array of cone indices (i < j)."""
    s = tri.simplices
    e = np.vstack([s[:, [0, 1]], s[:, [1, 2]], s[:, [2, 0]]])
    e.sort(axis=1)
    return np.unique(e, axis=0)


# --------------------------------------------------------------------------
# Small geometry helpers
# --------------------------------------------------------------------------

def cross2(a, b) -> float:
    """z-component of the 2D cross product. > 0 means b is to the left of a."""
    return float(a[0] * b[1] - a[1] * b[0])


def forward_of(xy, L, R) -> np.ndarray:
    """Unit track direction implied by a crossing: perpendicular to L->R,
    pointing so that L is on its left."""
    e = xy[R] - xy[L]                      # points from left cone to right cone
    t = np.array([-e[1], e[0]])            # rotate +90 deg
    return t / np.linalg.norm(t)


def angle_between(a, b) -> float:
    return float(np.degrees(np.arctan2(cross2(a, b), np.dot(a, b))))


# --------------------------------------------------------------------------
# 2. Close cones / first crossing
# --------------------------------------------------------------------------

def find_seed(xy: np.ndarray, tri: Delaunay, cfg: Config = Config()) -> Seed | None:
    """First crossing of the track, found with the car's forward ray.

    Follow the line y = 0 (straight ahead of the car). The first Delaunay edge
    it passes through, with a plausible width, joins one left and one right
    cone: the cone with y > 0 is left of the ray, the other right. Only the two
    cones at the crossing are judged, so it works when the car is angled or
    off-centre, and when the track bends sharply right after the car.
    """
    xy = np.asarray(xy, dtype=float)
    best, best_x = None, np.inf
    for i, j in unique_edges(tri):
        yi, yj = xy[i, 1], xy[j, 1]
        if yi * yj >= 0:                    # both on the same side of the ray
            continue
        f = yi / (yi - yj)                   # where the edge crosses y = 0
        x_cross = xy[i, 0] + f * (xy[j, 0] - xy[i, 0])
        if not (cfg.seed_min_x <= x_cross <= cfg.seed_max_x):
            continue
        width = float(np.linalg.norm(xy[i] - xy[j]))
        if not (cfg.min_width <= width <= cfg.max_width):
            continue
        # Prefer the first crossing ahead of the car; behind only as a fallback.
        score = x_cross if x_cross >= 0 else 100.0 - x_cross
        if score < best_x:
            L, R = (i, j) if yi > 0 else (j, i)
            best_x = score
            best = Seed(int(L), int(R), (xy[L] + xy[R]) / 2, width)
    return best


# --------------------------------------------------------------------------
# 3. Walk through the triangulation
# --------------------------------------------------------------------------

def _triangle_with_edge(tri: Delaunay, a: int, b: int, xy, mid, t) -> int:
    """Index of the triangle containing edge a-b whose third corner is ahead."""
    for k, simplex in enumerate(tri.simplices):
        if a in simplex and b in simplex:
            x = [v for v in simplex if v != a and v != b][0]
            if np.dot(xy[x] - mid, t) > 0:
                return k
    return -1


def walk_centreline(xy: np.ndarray, tri: Delaunay, seed: Seed,
                    cfg: Config = Config()) -> WalkResult:
    """Walk forward from the seed crossing, one triangle at a time.

    At each step the current crossing (L, R) is an edge of the triangle ahead.
    Its third corner X must continue either the left or the right boundary:
      * side test: X is left if it lies to the left of the current track
        direction t (through the crossing midpoint), otherwise right;
      * the next crossing is (X, R) if X is left, or (L, X) if X is right;
      * the next triangle is the neighbour across that new crossing.
    Stops on the convex hull, an implausible width, a sharp turn, a loop, or
    after cfg.max_length metres.
    """
    xy = np.asarray(xy, dtype=float)
    labels = np.full(len(xy), UNKNOWN, dtype=int)
    L, R = seed.left, seed.right
    labels[L], labels[R] = LEFT, RIGHT

    # Track direction: car heading for the first step (the seed is beside the car),
    # then the direction from the previous midpoint to the current one. The
    # perpendicular of a single crossing is not used: diagonal crossings skew it.
    t = np.array([1.0, 0.0])
    mid = (xy[L] + xy[R]) / 2
    centre, crossings, widths = [mid], [(L, R)], [seed.width]
    length, reason = 0.0, "max steps"

    k = _triangle_with_edge(tri, L, R, xy, mid, t)
    visited = set()
    for _ in range(cfg.max_steps):
        if k == -1:
            reason = "edge of triangulation"
            break
        if k in visited:
            reason = "loop"
            break
        visited.add(k)

        simplex = tri.simplices[k]
        X = int([v for v in simplex if v != L and v != R][0])

        # Side test against the current track direction.
        x_left = cross2(t, xy[X] - mid) > 0
        cand = [(X, R), (L, X)] if x_left else [(L, X), (X, R)]

        # Take the preferred option; if its width is implausible but the
        # other one's is fine, take the other (helps with noisy fans).
        chosen = None
        for nL, nR in cand:
            w = float(np.linalg.norm(xy[nL] - xy[nR]))
            if cfg.min_width <= w <= cfg.max_width:
                chosen = (nL, nR, w)
                break
        if chosen is None:
            reason = "width out of range"
            break
        nL, nR, w = chosen

        n_mid = (xy[nL] + xy[nR]) / 2
        step = n_mid - mid
        if np.linalg.norm(step) < 1e-6:
            reason = "no progress"
            break
        n_t = step / np.linalg.norm(step)
        if abs(angle_between(t, n_t)) > cfg.max_turn_deg:
            reason = "turn too sharp"
            break
        if np.dot(step, t) < 0:                        # going backwards
            reason = "moved backwards"
            break

        # Accept the step.
        dropped = L if nL == X else R                 # cone no longer in the crossing
        if labels[X] == UNKNOWN:
            labels[X] = LEFT if nL == X else RIGHT
        length += float(np.linalg.norm(step))
        L, R, mid, t = nL, nR, n_mid, n_t
        centre.append(mid)
        crossings.append((L, R))
        widths.append(w)
        if length >= cfg.max_length:
            reason = "max length"
            break

        # Next triangle: the neighbour opposite the dropped cone.
        idx = list(simplex).index(dropped)
        k = int(tri.neighbors[k][idx])

    return WalkResult(np.array(centre), crossings, labels, reason, widths)


# --------------------------------------------------------------------------
# 4. Smoothing
# --------------------------------------------------------------------------

def smooth_centreline(points: np.ndarray, spacing: float = 0.2,
                      rms_dev: float = 0.1) -> np.ndarray | None:
    """Smoothing spline through the ordered midpoints, resampled every
    `spacing` metres. `rms_dev` is how far [m, root-mean-square] the curve may
    stray from the midpoints: 0 passes through every point, larger values
    give a smoother curve that cuts corners. Returns None if too few points."""
    pts = np.asarray(points, dtype=float)
    keep = np.concatenate(([True], np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-3))
    pts = pts[keep]
    if len(pts) < 3:
        return None
    k = min(3, len(pts) - 1)
    tck, _ = splprep(pts.T, s=len(pts) * rms_dev ** 2, k=k)
    u = np.linspace(0, 1, 1000)
    x, y = splev(u, tck)
    arc = np.concatenate(([0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
    s_new = np.arange(0, arc[-1], spacing)
    u_new = np.interp(s_new, arc, u)
    return np.column_stack(splev(u_new, tck))


# --------------------------------------------------------------------------
# Test data
# --------------------------------------------------------------------------

# Test layouts. Each entry:
#   segs:     list of (length [m], curvature [1/m]) for the centreline; +curvature = left turn
#   width:    track width [m]
#   spacing:  max cone spacing along each boundary [m] (measured on that boundary)
#   car:      (lateral offset [m], yaw offset [deg]) of the car relative to the centreline
#   max_cones: cap on total cones (closest along the track are kept)
# FS autocross guide: width >= 3 m, hairpin outside diameter >= 9 m, cones <= 5 m apart.
TRACKS = {
    # Original longer layouts
    "straight": dict(segs=[(30.0, 0.0)]),
    "corner":   dict(segs=[(8.0, 0.0), (20.0, -1 / 15), (15.0, 0.0)]),
    "hairpin":  dict(segs=[(8.0, 0.0), (np.pi * 9.0, 1 / 9), (15.0, 0.0)]),

    # Short corners, <= 15 cones
    "gentle_left":    dict(segs=[(3.0, 0.0), (12.0, 1 / 20)], spacing=4.0, max_cones=12),
    "right_90":       dict(segs=[(3.0, 0.0), (np.pi / 2 * 9.0, -1 / 9), (4.0, 0.0)],
                           spacing=3.0, max_cones=15),
    "left_90_tight":  dict(segs=[(2.0, 0.0), (np.pi / 2 * 5.0, 1 / 5), (4.0, 0.0)],
                           spacing=2.5, max_cones=15),
    "tight_hairpin":  dict(segs=[(2.0, 0.0), (np.pi * 3.0, 1 / 3), (3.0, 0.0)],
                           spacing=2.0, max_cones=15),
    "chicane":        dict(segs=[(2.0, 0.0), (6.0, 1 / 8), (6.0, -1 / 8), (3.0, 0.0)],
                           spacing=3.0, max_cones=15),
    "wide_right":     dict(segs=[(3.0, 0.0), (12.0, -1 / 10)], width=5.0,
                           spacing=3.5, max_cones=12),
    "offset_entry":   dict(segs=[(3.0, 0.0), (10.0, -1 / 8), (3.0, 0.0)], spacing=3.0,
                           car=(0.8, 15.0), max_cones=14),
    "sparse_inside":  dict(segs=[(3.0, 0.0), (np.pi / 2 * 7.0, 1 / 7), (4.0, 0.0)],
                           spacing=5.0, max_cones=11),
}

SHORT_CASES = ["gentle_left", "right_90", "left_90_tight", "tight_hairpin",
               "chicane", "wide_right", "offset_entry", "sparse_inside"]


def make_test_track(kind: str = "corner", width: float | None = None,
                    spacing: float | None = None, noise: float = 0.05,
                    drop: int | None = None, seed: int = 0):
    """Synthetic cone layout in the car frame, with ground-truth sides.

    kind: a key of TRACKS. width/spacing override the layout's defaults.
    Cones are spaced along each boundary's own length, so the outside of a
    corner gets more cones than the inside (as on a real track).
    Returns (xy, true_labels).
    """
    if kind not in TRACKS:
        raise ValueError(f"unknown track {kind!r}; choose from {list(TRACKS)}")
    spec = TRACKS[kind]
    width = width if width is not None else spec.get("width", 3.0)
    spacing = spacing if spacing is not None else spec.get("spacing", 3.5)
    lat_off, yaw_off_deg = spec.get("car", (0.0, 0.0))
    max_cones = spec.get("max_cones")
    rng = np.random.default_rng(seed)
    ds = 0.05

    # Integrate a centreline from piecewise-constant curvature, starting 2 m behind the car.
    pts, hdg = [np.array([-2.0, 0.0])], [0.0]
    for length, k in spec["segs"]:
        for _ in range(int(round(length / ds))):
            h = hdg[-1] + k * ds
            pts.append(pts[-1] + ds * np.array([np.cos(h), np.sin(h)]))
            hdg.append(h)
    pts, hdg = np.array(pts), np.array(hdg)
    progress = np.arange(len(pts)) * ds                      # distance along centreline
    normal = np.stack([-np.sin(hdg), np.cos(hdg)], axis=1)

    cones, truth, prog = [], [], []
    for side in (LEFT, RIGHT):
        boundary = pts + side * normal * width / 2
        arc = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(boundary, axis=0).T))))
        n = int(np.ceil(arc[-1] / spacing)) + 1
        idx = np.searchsorted(arc, np.linspace(0, arc[-1], n))
        idx = np.clip(idx, 0, len(pts) - 1)
        cones.append(boundary[idx])
        truth.append(np.full(len(idx), side))
        prog.append(progress[idx])
    xy, truth, prog = np.vstack(cones), np.concatenate(truth), np.concatenate(prog)

    if max_cones is not None and len(xy) > max_cones:
        keep = np.sort(np.argsort(prog, kind="stable")[:max_cones])
        xy, truth = xy[keep], truth[keep]

    # Place the car off the centreline / at an angle: express cones in that car frame.
    yaw = np.radians(yaw_off_deg)
    c, s_ = np.cos(yaw), np.sin(yaw)
    xy = (xy - [0.0, lat_off]) @ np.array([[c, -s_], [s_, c]])

    xy = xy + rng.normal(0, noise, xy.shape)
    if drop is not None:
        xy, truth = np.delete(xy, drop, axis=0), np.delete(truth, drop)
    return xy, truth


# --------------------------------------------------------------------------
# Plotting / demo
# --------------------------------------------------------------------------

def run_case(kind, drop=None, noise=0.05, rng_seed=0, cfg: Config = Config(),
             rms_dev: float = 0.1, verbose=True):
    xy, truth = make_test_track(kind, noise=noise, drop=drop, seed=rng_seed)
    tri = triangulate(xy)
    seed = find_seed(xy, tri, cfg) if tri is not None else None
    walk = walk_centreline(xy, tri, seed, cfg) if seed is not None else None
    smooth = smooth_centreline(walk.centreline, rms_dev=rms_dev) if walk is not None else None

    if verbose:
        if walk is None:
            print(f"{kind:14s} drop={drop}: no first crossing found")
        else:
            lab = walk.labels
            known = lab != UNKNOWN
            correct = int(np.sum(lab[known] == truth[known]))
            print(f"{kind:14s} drop={drop}: {len(xy):2d} cones | labelled {known.sum():2d}, "
                  f"correct {correct:2d}/{known.sum():2d} | {len(walk.centreline):2d} centre points"
                  f" | stopped: {walk.stop_reason}")
    return xy, truth, tri, seed, walk, smooth


def plot(ax, xy, tri, walk, smooth, title=""):
    """Minimal view: triangles, cones (coloured by side once labelled), centreline, car."""
    from matplotlib.patches import FancyArrow

    ax.clear()
    ax.triplot(xy[:, 0], xy[:, 1], tri.simplices, color="0.7", lw=0.8, zorder=1)

    labels = walk.labels if walk is not None else np.full(len(xy), UNKNOWN)
    colours = np.where(labels == LEFT, "tab:blue", np.where(labels == RIGHT, "gold", "white"))
    ax.scatter(xy[:, 0], xy[:, 1], c=colours, edgecolors="k", s=60, zorder=3)

    if smooth is not None:
        ax.plot(smooth[:, 0], smooth[:, 1], color="tab:green", lw=2.5, zorder=2)

    ax.add_patch(FancyArrow(-0.8, 0, 1.6, 0, width=0.35, head_width=0.9, head_length=0.6,
                            length_includes_head=True, color="k", zorder=4))

    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_xlabel("x forward [m]")
    ax.set_ylabel("y left [m]")
    ax.grid(alpha=0.25)


if __name__ == "__main__":
    # ======================================================================
    #  CHOOSE THE TEST TRACK HERE
    # ======================================================================
    #  Short corners: "gentle_left", "right_90", "left_90_tight", "tight_hairpin",
    #                 "chicane", "wide_right", "offset_entry", "sparse_inside"
    #  Longer:        "straight", "corner", "hairpin"
    TRACK = "sparse_inside"

    NOISE = 0.05        # cone position noise, standard deviation [m]
    DROP = None         # index of a cone to remove, e.g. 3, or None
    SEED = 0            # random seed for the noise (change for a different pattern)
    SAVE_TO = None      # e.g. "track.png" to save an image instead of opening a window

    MAX_LENGTH = 100.0  # stop the walk after this much centreline [m]. On the car this
                        # would be the look-ahead (~20-40 m); raised here to see whole tracks
    SMOOTHING = 0.1     # how far the smoothed line may stray from the midpoints [m RMS]
    # ======================================================================

    import matplotlib
    if SAVE_TO:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cfg = Config(max_length=MAX_LENGTH)
    xy, truth, tri, seed, walk, smooth = run_case(TRACK, DROP, NOISE, SEED, cfg, SMOOTHING)
    fig, ax = plt.subplots(figsize=(9, 8))
    plot(ax, xy, tri, walk, smooth, title=TRACK)
    fig.tight_layout()

    if SAVE_TO:
        fig.savefig(SAVE_TO, dpi=110)
        print(f"saved {SAVE_TO}")
    else:
        plt.show()

