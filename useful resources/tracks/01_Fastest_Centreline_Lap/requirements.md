# Challenge 1: Fastest Centreline Lap

## Objective

Plan the fastest valid flying lap around the supplied Silverstone circuit
centreline. Your task is to choose the speed at every track point. The route is
fixed: you may not optimise a racing line, cut corners, or move off the centreline.

This is a simplified, two-dimensional speed-planning challenge on an F1 circuit.
The vehicle limits below are game parameters, not specifications for a real F1 car.

## Input

Use `silverstone.csv` in this folder. Its first line is a commented header:

```text
# x_m,y_m,w_tr_right_m,w_tr_left_m
```

There are **1,178 points**. Including the closing segment, the supplied polyline
is approximately **5,886.805 metres** long. See `track_preview.png` for its layout.

| Column | Meaning | Units |
| --- | --- | --- |
| `x_m` | Centreline X coordinate in a local Cartesian frame | metres |
| `y_m` | Centreline Y coordinate in the same frame | metres |
| `w_tr_right_m` | Track width to the right; unused in this challenge | metres |
| `w_tr_left_m` | Track width to the left; unused in this challenge | metres |

The first numerical row is point `0`. Follow the points in their supplied order
and close the lap by joining the last point back to point `0`. The last row does
not duplicate the first. The challenge's timing line is point `0`; it need not
coincide with the real circuit's official timing line.

Use the supplied coordinates as the authoritative challenge geometry. Do not
substitute another map, resample, smooth, reorder, reverse, or scale the input.
Internal calculations are unrestricted, but the submitted speeds and validity
checks must refer to the original points and the model defined here.

## Vehicle Limits

| Quantity | Limit |
| --- | --- |
| Maximum speed | 70.0 m/s (252 km/h) |
| Maximum forward acceleration | +6.0 m/s^2 |
| Maximum braking deceleration magnitude | 10.0 m/s^2 |
| Maximum lateral acceleration | 12.0 m/s^2 |
| Minimum speed | 0.0 m/s; reversing is forbidden |

Assume a point-mass vehicle, flat ground, constant grip, and perfect tracking of
the fixed path. Ignore drag, downforce, gradients, tyre temperature, power limits,
gear changes, reaction delay, and jerk limits. Acceleration may change immediately
between segments, but speed must remain continuous.

Longitudinal and lateral limits are **independent** for this exercise: no friction
circle or combined-grip constraint is imposed. Braking or accelerating while
cornering is allowed, provided each individual limit is satisfied.

## Shared Geometry and Motion Model

Use this exact discrete model so every submission can be scored consistently.
It defines the exercise's approximation to cornering; do not interpret the
straight chords between samples as having zero curvature and unlimited corner speed.

Let there be `N` input points, with `p_i = (x_i, y_i)` and submitted speed `v_i`.
All point indices wrap around the closed lap: `p_N = p_0`, `p_-1 = p_(N-1)`,
and `v_N = v_0`.

### Segment Length

For the segment from point `i` to point `i+1`:

```text
ds_i = length(p_(i+1) - p_i)
```

Include all `N` segments, including the last-to-first segment. Use the actual
distance between points; do not assume uniform spacing or an official lap length.


## Start and Finish

This is a **flying lap**, not a standing start. You choose the speed at point `0`
subject to the same rules as every other point. The lap must be repeatable:
the speed when crossing the timing line at the end is exactly the starting speed.
There is no separate free finish speed and no teleport or reset at the timing line.

## Submission

Submit:

1. Your Python planning program and any dependency list needed to run it.
2. `speed_profile.csv`, with exactly `N` data rows and this header:

   ```text
   point_id,speed_mps
   ```

   Include each integer point ID from `0` to `N-1` once, in order. Do not append
   a duplicate finish row. Write speeds to at least nine decimal places.
3. A short `report.md` containing the total lap time in seconds, a brief explanation
   of your approach, and the maximum speed, forward acceleration, braking magnitude,
   and lateral acceleration in your submitted profile.
4. A speed-versus-distance plot and a track plot colored by speed, each with units
   and a legend or colorbar.

The saved CSV is authoritative. Compute your reported lap time and constraint
checks from the values written to that file, including the closing segment.


There is no target lap time or real-world lap record to match. Optimise for the
supplied geometry and the rules above.

