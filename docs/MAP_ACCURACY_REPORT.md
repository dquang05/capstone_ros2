# restaurant_main map error relative to Gazebo

## Inputs and conclusion

The offline evaluation used:

- Map: `src/amr_navigation/maps/restaurant_main.pgm` and its YAML metadata.
- World: `src/amr_gazebo/worlds/restaurant_blueprint.world`.
- Registration: `scripts/navigation_validation/world_to_map.json`; hashes
  confirmed the map/world inputs at evaluation time. No refitting or map rescaling was performed.

**Within the evaluated boundaries, mean error is approximately 2.4–2.7 cm;
P95 is approximately 5.3–5.7 cm.** Most map boundaries match world geometry
within 1–2 map cells. Local world→map discrepancies reach 33.90 cm, so the
results do not establish sub-10 cm accuracy for every detail.

This is geometric error after rigid registration, affected by coordinate
registration, resolution, and surface selection. It is not trajectory error
or proof of loop closure. No PASS/FAIL threshold is imposed without acceptance requirements.

## Method

The map has **483 × 302 cells** at **0.05 m/cell**. Occupied/free/unknown
classification follows YAML `occupied_thresh`, `free_thresh`, and `negate`,
retaining the origin and image row direction.

The nominal LiDAR height derived from Xacro is **0.107 m** above ground.
World visual primitives are sliced at this height, overlapping regions are
united, and their boundary extracted; objects above the scan plane are not
represented by their full footprint. There are **563 box/cylinder sections**.
The reference raster spacing is **5 mm**. Visual and collision sections are
not identical: their rasterized areas differ by approximately **0.2414 m²**.
The evaluation uses visuals to compare GPU LiDAR scene geometry, but does not
run the sensor renderer directly.

Map boundaries consist of occupied cells adjacent to free cells. Nearest
boundary distances are measured in both directions to evaluate reconstructed
boundary placement and missing reference boundaries. World→map considers only
reference samples falling in known map cells; unknown regions are not counted
as missing-boundary errors. Metrics are point-sampled, not area-weighted.

## Quantitative results

| Metric | Map → world | World → map, known region |
|---|---:|---:|
| Mean | 2.44 cm | 2.69 cm |
| RMSE | 2.96 cm | 3.21 cm |
| Median | 2.18 cm | 2.44 cm |
| P95 | 5.65 cm | 5.27 cm |
| Maximum | 8.15 cm | 3.39 cm |
| Samples | 4,515 | 42,465 |

| Tolerance | Map boundary precision | Reference boundary recall in known region | F1 |
|---|---:|---:|---:|
| 5 cm | 92.40% | 93.44% | 92.92% |
| 10 cm | 100.00% | 99.60% | 99.80% |
| 15 cm | 100.00% | 99.78% | 99.89% |

Precision of 100% at 10 cm means that every map boundary sample **selected
for evaluation** has a world boundary within 10 cm; it does not mean the
entire map is 100% accurate.

## Coverage, IoU, and sensitivity

- There are **6,647 occupied**, **103,529 free**, and **35,690 unknown cells**.
- **75.53%** of the rectangular map is known, not of the entire restaurant.
  **78.81%** of world boundary samples within that rectangle fall in known cells.
- Occupied IoU sampled at known-cell centers is **49.41%**. This penalizes
  occupied thickness and grid alignment differences, so interpret it alongside
  boundary error. Accuracy dominated by free cells does not establish overall quality.
- Increasing reference raster spacing from 5 to 10 mm gives means of
  2.58/2.77 cm and P95 of 5.51/5.51 cm. Mean changes are under 1.5 mm; metrics
  at 5 cm tolerance change by about 2 percentage points, showing threshold sensitivity.
- Shifting the slice ±1 cm, from 0.097 to 0.117 m, leaves results unchanged
  at 10 mm spacing because the selected sections do not change over this height range.
- Five unit tests passed: rotated/translated boxes, unions without internal
  boundaries, image coordinates with rotated origins, cylinder area, and
  pose composition/height classification.

Registration RMSE is 3.28 cm on the two registration walls; independent check
walls have medians of 2.15–3.19 cm and P95 of 2.71–7.89 cm. This is an accompanying
uncertainty source and cannot be directly added/subtracted to derive pure SLAM error.

## Reproduction and output files

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/slam_validation/evaluate_map.py --spacing 0.005 \
  --output report/map_quality/new_evaluation
python3 -m unittest discover -s scripts/slam_validation -p 'test_*.py'
```

The output directory must be new. The script only reads inputs and does not
rewrite the map/world or registration. A different map/world requires a
matching registration survey; mismatched hashes and unsupported geometry are rejected.

- [Generated report](../report/map_quality/restaurant_main_final/REPORT.md)
- [Overlay and error heatmap](../report/map_quality/restaurant_main_final/map_error_overlay.png)
- [Error distribution](../report/map_quality/restaurant_main_final/error_distribution.png)
- [Metrics, scope, and input hashes](../report/map_quality/restaurant_main_final/summary.json)
- Per-point error CSVs, geometry inventory, and reference raster are saved
  alongside these files in `report/map_quality/restaurant_main_final/`.

Git ignores `report/`; this document preserves the main results. Without scan
history, observed surface selection is approximated using known cells.
Nearest-neighbor matching may associate multiple points with one surface;
it does not replace topology, traversability, or live AMCL/Nav2 evaluation.

## Method basis and references

The method is **bidirectional nearest-neighbor geometric boundary comparison
after rigid SE(2) registration, combined with tolerance-based precision/recall**.
This is a project-specific implementation for 2D occupancy maps, not an
unmodified execution of a paper's benchmark suite.

For map boundary points M and reference points G, calculate
`d(m,G) = min(g in G) ||m-g||₂` and the reverse `d(g,M)`.
Mean, RMSE, median, and P95 are reported separately in each direction.
Precision is the fraction of M within tolerance of G; recall is the fraction
of G in the known region within tolerance of M. F1 is their harmonic mean.

1. **Fang et al. (2016), Comparative Evaluation of Time-of-Flight
   Depth-Imaging Sensors for Mapping and SLAM Applications**, ACRA, Section 2.
   Describes occupancy-map evaluation using mean distance from occupied
   reference cells to nearest occupied map cells after translation/rotation
   registration. This evaluation adapts it to boundary points and adds the reverse direction.
   [Full paper](https://www.araa.asn.au/acra/acra2016/papers/pap146s1.pdf).
2. **Knapitsch, Park, Zhou, and Koltun (2017), Tanks and Temples: Benchmarking
   Large-Scale Scene Reconstruction**, ACM Transactions on Graphics, Measures,
   equations (3)–(7). Defines bidirectional nearest distances, precision,
   recall, and F-score at a tolerance. The original evaluates 3D reconstructions;
   this project applies the measurement principle to 2D boundaries without
   adopting that benchmark's entire registration and evaluation-region procedure.
   [Full paper](https://storage.googleapis.com/t2-downloads/paper/tanks-and-temples.pdf).

The 0.107 m LiDAR slice, 5 mm raster, wall registration, known/unknown mask,
and 5/10/15 cm tolerances are conventions of this experiment, not universal
SLAM PASS/FAIL criteria. Tanks and Temples uses `< d`; this script uses
`<= d`, explicitly stated to reproduce counting at the threshold boundary.
