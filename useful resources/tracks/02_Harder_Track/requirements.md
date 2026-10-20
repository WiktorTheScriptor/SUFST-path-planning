# Challenge 2: Harder Track

Reconstruct the left and right boundaries from this folder's cone map using the
same [reconstruction rules and submission format](../requirements.md) as the
original track. This is a second dataset for the same challenge, not a solution.

## Inputs

| File | Contents |
| --- | --- |
| `cones.csv` | 246 shuffled cones: 131 blue and 115 yellow |
| `start_pose.csv` | Starting position `(0, 0)`, heading `0` radians (positive X) |
| `start_gate.csv` | The starting cone ID for each boundary |
| `cone_map.png` | Cone positions shown as blue and yellow dots |

Use all three CSV files from this folder together. The gate IDs belong to this
map and must not be taken from the original dataset. CSV column names and units
are unchanged.

## What Makes It Harder

- Several tight switchbacks and nearby parallel sections.
- A longer circuit with more changes of direction.
- Irregular cone spacing, with selected gaps reaching approximately 20 metres.
- Unequal numbers of cones on the two sides, sampled independently.
- Slight positional irregularity and a changing track width.

The cone counts and gap sizes above replace the original dataset's figures.
All other rules still apply. Blue is left and yellow is right in the direction
of travel. Colors remain reliable; all cones are valid and must be used.
Row order and cone IDs do not provide the boundary order.

## Required Result

Produce two complete, non-intersecting closed boundaries, starting at their gate
cones and initially proceeding in positive X. Use every cone on its correct side,
keep output point spacing at or below `0.5 m`, and maintain the specified
`3.0-8.0 m` separation between boundaries. Check the complete line segments,
not only the cone positions.

Submit `ordered_cones.csv`, `left_boundary.csv`, `right_boundary.csv`,
`track_boundaries.png`, `report.md`, and your Python program, as described in the
main requirements. Save these results separately from those for the original map.

This synthetic dataset has been checked to admit closed boundaries satisfying
the challenge's geometry constraints. No ordered cones or reconstructed boundary
solution is included.
