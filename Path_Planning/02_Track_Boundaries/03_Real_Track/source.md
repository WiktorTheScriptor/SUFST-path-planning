# Source and Attribution

## Original Dataset

- Dataset: [FSD Racetrack Dataset](https://github.com/iv461/fsd_racetrack_dataset), by StarkStrom Augsburg.
- Selected layout: track 1, measured with LiDAR during real test drives.
- Pinned revision: `02c7ebb54283010988058822d74214778e446d77`.
- [Original cone map](https://github.com/iv461/fsd_racetrack_dataset/blob/02c7ebb54283010988058822d74214778e446d77/dataset/cone_map_1.yaml).
- [Original manual boundary annotations](https://github.com/iv461/fsd_racetrack_dataset/blob/02c7ebb54283010988058822d74214778e446d77/dataset/boundaries_1.yaml).
- [Dataset description](https://github.com/iv461/fsd_racetrack_dataset/blob/02c7ebb54283010988058822d74214778e446d77/README.md).
- Publication: Ivo Ivanov and Carsten Markgraf (2024), [Lane Detection using Graph Search and Geometric Constraints for Formula Student Driverless](https://arxiv.org/abs/2405.16369).

The source describes nine layouts obtained from real LiDAR/SLAM data, with
manually annotated boundaries and cone position accuracy of approximately
`0.2-0.3 m`. It does not identify this track's venue or an official competition
event. This package therefore calls it a real FSD test track, not an FSG circuit.

## Adaptation for This Challenge

Adapted on 2026-10-06 for the track-boundary reconstruction exercise:

1. Read the YAML map as cone ID to `[x, y]`, in the original metric frame.
2. Assign `blue` to the 66 IDs annotated as left and `yellow` to the 70 IDs
   annotated as right. These are exercise labels, not recorded camera colors.
3. Keep all 136 cones and original IDs; no filtering, smoothing, coordinate
   transforms, or artificial noise. Round CSV positions to twelve decimals.
4. Shuffle rows with Python's `random.Random(3101)` so row order is not a solution.
5. Select the first published left/right pair (`49`, `5`) as the exercise gate.
   Set the start to its midpoint. Set the heading to
   `atan2(next_midpoint_y - start_y, next_midpoint_x - start_x)` using the next
   published pair (`17`, `10`). This pose is derived, not measured.
6. Plot only the supplied positions, with blue/yellow dots and equal axis scale.
   Do not include the source's ordered boundary lists in the challenge package.

The unmodified map is retained as [source_cone_map.yaml](source_cone_map.yaml).
The generated `cones.csv`, starting conditions, and preview are adaptations of
this source. The two synthetic sibling tracks are unrelated to this dataset.

## License

The upstream dataset is released under LGPLv3. Its license is retained verbatim
in [TRACK_DATA_LICENSE](TRACK_DATA_LICENSE). Keep this attribution, license, and
source links with redistributed copies of the adapted real-track data.
