# Challenge 2: Simple Track

Use the shared [reconstruction rules and submission format](../requirements.md)
with the inputs in this folder. This is the original introductory dataset,
moved here without changing its data.

## Inputs

| File | Contents |
| --- | --- |
| `cones.csv` | 142 shuffled cones: 71 blue and 71 yellow |
| `start_pose.csv` | Starting position `(0, 0)`, heading `0` radians |
| `start_gate.csv` | Starting cone ID for each boundary |
| `cone_map.png` | Blue and yellow dots showing the input positions |

This is a synthetic closed circuit created for the exercise. Coordinates are
in metres. Cone spacing is irregular, with some gaps reaching approximately
nine metres. The two sides were sampled independently; do not pair by row or ID.
Every cone is valid. Blue marks the left side and yellow marks the right side
in the direction of travel.

Reconstruct both complete closed boundaries, using every cone and starting at
the supplied gate. Output point spacing must be at most `0.5 m`, and boundary
separation must remain between `3.0 m` and `8.0 m`. All shared rules apply.

Save your submission separately from the other datasets. No solution is included.
