# Challenge 2: Real Formula Student Driverless Track

Reconstruct both boundaries using the shared
[rules and submission format](../requirements.md) and this folder's inputs.
This is track 1 of StarkStrom Augsburg's FSD Racetrack Dataset, collected with
LiDAR during real test drives, not a synthetic layout. It is not presented as
an official competition circuit; the source does not identify its venue.

## Inputs

| File | Contents |
| --- | --- |
| `cones.csv` | 136 shuffled cones: 66 blue and 70 yellow |
| `start_pose.csv` | Derived exercise start position and heading, in metres/radians |
| `start_gate.csv` | Left cone `49`, right cone `5` |
| `cone_map.png` | Input cone positions as blue and yellow dots |
| `source_cone_map.yaml` | Unmodified upstream cone map for provenance |
| `source.md` | Source links, attribution, and conversion details |
| `TRACK_DATA_LICENSE` | Upstream LGPLv3 license |

## Measured Data

The source contains SLAM cone positions and manually annotated boundary
membership. Its stated positional accuracy is approximately `0.2-0.3 m`.
The published map does not contain measured colors: blue and yellow labels were
assigned from the annotated left/right membership for this exercise.

All 136 source cones belong to the annotated boundaries, so none were discarded
or added. Coordinates and IDs are preserved, with positions rounded to twelve
decimal places in the CSV. Rows have been shuffled. The source frame has not
been translated, rotated, or scaled.

The exercise start is the midpoint of the supplied gate. Its heading points
toward the midpoint of the next annotated cone on each side. This is a derived
starting condition, not a recorded vehicle pose. Use `start_pose.csv` rather
than assuming `(0, 0)` or a zero heading.

The common `3.0-8.0 m` boundary separation and `0.5 m` maximum output spacing
still apply. The source ordering was checked to admit two simple, disjoint
closed loops within the separation bounds. The measurement uncertainty does
not relax the requirement to preserve the supplied coordinates in your output.

Submit the same files as for the other datasets, saved separately. No ordered
cone solution or reconstructed boundaries are included locally. The public
source has annotations; do not use their ordering when attempting the challenge.

See [source.md](source.md) for the original dataset and associated publication.
