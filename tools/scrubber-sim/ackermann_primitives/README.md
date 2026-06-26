# ackermann_primitives

Ackermann-first path generation utilities for the scrubber simulation.

This package is the active coverage path line after V0. It exists because P2/P3 showed that coverage and wall-following paths must respect the chassis before they reach Nav2.

## Fixed chassis assumptions

- Rear-wheel drive
- Front-wheel steering
- No in-place turn
- No K-turn
- Large-room cleaning first; small residual corners can be left for later/manual handling

## Current staged roadmap

- **K0**: line/arc/U-turn primitives and curvature checks ✅ local tests pass
- **O0**: quick odom/chassis calibration analysis ✅ local tests pass; fresh remote calibration complete
- **K1**: pure large-room boustrophedon coverage ✅ local tests pass
- **V1**: Nav2 FollowPath validation — odom-frame control passes; gz-truth coverage pending turn compensation
- **K3**: later mixed outer edge-ring + inner boustrophedon

## Initial planning constants

- Physical minimum radius from URDF/gz steering limit: about 0.97m
- Control/design minimum from spec: `R_min = 1.2m`
- Initial planning safe radius: `R_safe = 2.0m`

V0 and O0 fresh runs showed stable turn asymmetry:

- Straight 5m: truth 4.875m, odom 4.884m, end error 0.009m
- R2 left: truth radius about 2.36-2.57m
- R2 right: truth radius about 1.73-1.87m
- R3 left: truth radius about 3.34-3.56m
- R3 right: truth radius about 2.71-2.84m

V1 currently passes in odom frame tracking (odom nearest mean 0.087m, end error 0.248m), but does not yet pass gz-truth evaluation (truth end error about 2.29m, truth/odom end gap about 2.07m). Treat this as a simulation/kinematics model mismatch until turn compensation or a corrected three-wheel gz model is in place.

## K0 implemented API

- `Pose2D(x, y, yaw)`: immutable 2D pose in meters/radians
- `sample_line(start, length, step=0.1)`: sample a straight segment
- `sample_arc(start, radius, angle, step=0.1, direction="left")`: sample a constant-radius arc
- `sample_u_turn(start, radius, step=0.1, direction="left")`: sample a half-circle U-turn
- `max_curvature(poses)`: estimate maximum discrete curvature from 3-point windows
- `assert_curvature_within(poses, max_allowed)`: reject paths tighter than the configured radius

Local verification command:

```bash
cd tools/scrubber-sim
PYTHONPATH=. pytest tests/test_ackermann_primitives.py -q
```

## O0 implemented API

- `TimedPose2D(t, x, y, yaw)`: timestamped 2D pose
- `load_xytheta_csv(path)`: load x/y/theta or x/y/yaw CSV
- `load_timed_xytheta_csv(path)`: load timed CSV; uses row order when time is absent
- `align_by_time(reference, target)`: interpolate target poses to reference timestamps
- `compute_trajectory_error(truth, odom)`: endpoint/max/mean XY and yaw errors
- `fit_circle_radius(rows)`: algebraic circle-fit radius estimate
- `fit_arc_radius_by_distance_and_heading(rows)`: arc-length / heading-change radius estimate
- `estimate_turn_radius(rows)`: report both radius estimates for O0 diagnostics

Local verification command:

```bash
cd tools/scrubber-sim
PYTHONPATH=. pytest tests/test_ackermann_calibration.py -q
```

## K1 implemented API

- `generate_lawnmower(width, height, lane_spacing, turn_radius, margin, step=0.1)`: generate a large-room pure boustrophedon path using straight sweeps and semicircle U-turns
- `BoustrophedonError`: raised when the room or parameters cannot produce an Ackermann-feasible K1 path

K1 initial constraint: `lane_spacing == 2 * turn_radius`. This keeps each lane transition as a true semicircle and avoids hidden small-radius connector curves. The generator also reserves one `turn_radius` horizontally at both ends so the U-turn bulge stays inside the margin-shrunk room.

Local verification command:

```bash
cd tools/scrubber-sim
PYTHONPATH=. pytest tests/test_ackermann_boustrophedon.py -q
```

## V1 export API

- `poses_to_rows(poses)`: convert `Pose2D` to `(x, y, yaw)` rows
- `write_path_csv(poses, path)`: write `x,y,yaw` CSV for remote runners
- `read_path_csv(path)`: read `x,y,yaw` or `x,y,theta` CSV
- CLI: `python3 -m ackermann_primitives.cli generate-lawnmower ... --output path.csv`

Example:

```bash
cd tools/scrubber-sim
PYTHONPATH=. python3 -m ackermann_primitives.cli generate-lawnmower \
  --width 14 --height 12 \
  --lane-spacing 4 --turn-radius 2 \
  --margin 2 --step 0.2 \
  --max-curvature 0.5 \
  --output /tmp/k1_path.csv
```

Local verification command:

```bash
cd tools/scrubber-sim
PYTHONPATH=. pytest tests/test_ackermann_export_cli.py -q
```

## Evaluation rule

In simulation, coverage, collision/overrun, and wall-clearance metrics must use gz ground truth or map-pose. Pure wheel odom is useful for control/debugging but is not the evaluation truth.
