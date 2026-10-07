# Challenge 2: Reconstruct the Track Boundaries

## Objective

Given an unordered map of cone positions, reconstruct the complete left and right
boundaries of a closed circuit. Produce two ordered, continuous loops, including
the stretches between widely spaced cones.

Your output should describe the track edges so a later planner can determine
where the car is allowed to drive. You do not need to produce a centreline,
racing line, speed profile, or vehicle controller.

## Provided Files

Choose one dataset and use all its input files together:

| Dataset | Cone counts | Origin |
| --- | --- | --- |
| [01_Simple_Track](01_Simple_Track/requirements.md) | 71 blue, 71 yellow | Synthetic introductory circuit |
| [02_Harder_Track](02_Harder_Track/requirements.md) | 131 blue, 115 yellow | Synthetic switchbacks and larger gaps |
| [03_Real_Track](03_Real_Track/requirements.md) | 66 blue, 70 yellow | Real FSD LiDAR map from StarkStrom Augsburg |

Each folder contains:

| File | Contents |
| --- | --- |
| `cones.csv` | Cone IDs, coordinates, and colors, in shuffled order |
| `start_pose.csv` | Starting position and heading |
| `start_gate.csv` | The left and right cones at the starting gate |
| `cone_map.png` | A visual reference showing input positions as blue and yellow dots |

The reconstruction rules and submission format below apply to all three tracks.
Each dataset's own requirements describe its origin, cone counts, and spacing.
All coordinates use the
same local Cartesian frame, in metres. Positive heading is counterclockwise
from the positive X axis. There is no GPS conversion to perform.

### Cone Map

`cones.csv` has this header:

```text
cone_id,x_m,y_m,color
```

For this challenge, **blue cones mark the left boundary** and **yellow cones mark
the right boundary**, relative to the direction of travel. Colors are reliable.

The positions are irregular and cones are not evenly spaced. Gap sizes vary
between datasets.
Every supplied cone is a valid boundary observation: there are no outliers,
unknown colors, or obstacle cones to discard.

Do not assume that opposite cones form aligned pairs or that both sides have the
same number of cones. Neither CSV row order nor cone IDs encode driving order.

### Start Pose and Gate

`start_pose.csv` has the header `x_m,y_m,heading_rad`. Read the starting position
and direction from this file; they are not necessarily zero.

`start_gate.csv` has the header `left_cone_id,right_cone_id`. These identify the
two cones at the same starting gate. Use them as the first vertices of their
respective output boundaries. This is the only supplied left/right pairing.

## Reconstruction Requirements

1. Build exactly two closed boundaries: one through the blue cones and one
   through the yellow cones.
2. Order both boundaries in the direction of travel. Start each at its specified
   gate cone and initially proceed in the supplied heading direction. Traverse each
   circuit once; do not reverse, make branches, or double back over an edge.
3. Use every cone exactly once in the ordered cone list for its own side. Preserve
   its ID, color, and measured coordinates. Do not introduce extra cones.
4. Each supplied cone position must appear as a vertex in its boundary, in the
   order declared by your ordered cone list. Allow an absolute coordinate error
   of at most `1e-6 m` per axis for file rounding. The first gate cone also appears
   at the end solely to close the output loop.
5. Fill all gaps with additional boundary points. Consecutive output points must
   be no more than `0.5 m` apart, including the segment arriving at the repeated
   first point. Every segment must have positive length.
6. Each boundary must be a simple loop: no self-intersections, self-touching,
   overlapping edges, or shortcuts across another part of the circuit.
7. The left and right boundaries must not touch or cross. Together they must
   define one continuous track corridor containing the supplied start position.
8. The shortest distance from every point along either boundary to the other
   boundary must be between `3.0 m` and `8.0 m`. This is a separation constraint,
   not a requirement to pair cones by index. Apply it to the line segments as
   well as the submitted vertices.
9. All coordinates must be finite. No missing values, infinities, or NaNs.

The output represents polylines: consecutive submitted points are joined by
straight segments. Smoothing is optional; any interpolated or smoothed result
must still meet all the requirements above. These rules apply to the measured
cone map; there is no hidden exact curve that you must recover.

## Submission

Submit your Python program, any dependency list, and these files:

### `ordered_cones.csv`

```text
side,order,cone_id
```

Use `left` and `right` for the side. Number the cones on each side from `0`,
without gaps, in driving order. Put all left rows first, followed by all right
rows. Each supplied cone ID must occur exactly once. Do not repeat the starting
cone in this file.

### `left_boundary.csv` and `right_boundary.csv`

Each file must have this header:

```text
x_m,y_m
```

Write at least six decimal places. Start at that side's gate cone and repeat
the first coordinate pair as the final row to make closure explicit. Include
the observed cone positions and the additional points between them. Added
points describe the boundary; they are not new cone observations.

### `track_boundaries.png`

Plot the original cones and both reconstructed boundaries on the same figure.
Use equal X/Y scale, distinct side colors, a legend, and clearly show the starting
position and direction. The gaps, closure, and complete circuit should be visible.

### `report.md`

Briefly explain your approach and report:

- Number of cones used on each side.
- Number of output points and total length of each closed boundary.
- Largest distance between consecutive output points on each side.
- Whether both loops close and whether any intersections were found.
- Minimum and maximum separation between the boundaries, with the method used
  to check separation along segments.

Compute these checks from your saved output files, not only intermediate values.

## Success Criteria

A submission completes the challenge when both reconstructed boundaries satisfy
all the requirements. There is no reward for shortening the boundaries, skipping
cones, or matching a particular number of output points. Multiple reconstructions
can be valid.

Use a numerical tolerance of `1e-6 m` for distance-limit checks. Topological
requirements still apply: rounding tolerance does not permit intersections or
overlapping edges.

No reconstruction code, ordered cone solution, or completed boundary files are
provided in this challenge package.
