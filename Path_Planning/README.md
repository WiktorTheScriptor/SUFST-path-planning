# Path Planning

## Challenge 1: Fastest Centreline Lap

Choose the fastest feasible speed profile around a fixed Silverstone centreline,
subject to acceleration, braking, cornering, and speed limits.

Start with [requirements.md](01_Fastest_Centreline_Lap/requirements.md).
The challenge folder contains the track input and preview. Its reference solution
is in [01_Solution](01_Fastest_Centreline_Lap/01_Solution/README.md).

## Challenge 2: Reconstruct the Track Boundaries

Given shuffled blue and yellow cone positions, reconstruct both complete track
boundaries as ordered, closed loops.

Start with [requirements.md](02_Track_Boundaries/requirements.md).
Each dataset folder contains its own cone map, start pose, start gate, and PNG:

- [01_Simple_Track](02_Track_Boundaries/01_Simple_Track/requirements.md): the original introductory circuit.
- [02_Harder_Track](02_Track_Boundaries/02_Harder_Track/requirements.md): switchbacks, nearby parallel sections, and larger gaps.
- [03_Real_Track](02_Track_Boundaries/03_Real_Track/requirements.md): real LiDAR data from StarkStrom Augsburg's FSD test drives, with [source and attribution](02_Track_Boundaries/03_Real_Track/source.md).

No boundary reconstruction solution is included.
