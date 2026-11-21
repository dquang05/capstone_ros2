# Drive-wheel effective separation validation

## Conclusion

Gazebo Harmonic dynamics tests confirm that the current model requires an
**effective wheel separation of approximately 0.55 m** for wheel odometry
to match body rotation. At 0.50 m, odometry overestimates the ground-truth
angle by nearly 10%. Straight motion has no significant discrepancy in
either configuration.

Distinguish geometry from calibration:

- `properties.xacro`: wheel centers at Y = ±0.250 m; center separation **0.500 m**.
- `drive_wheels.xacro`: cylindrical collisions are **0.050 m** wide and
  centered on the joints; outer-edge separation **0.550 m**, inner gap **0.450 m**.
- The `Active_wheel.STL` mesh measures 0.150 × 0.050 × 0.150 m; its visual
  offset centers the mesh on the joint. No hidden offset places the centers 0.55 m apart.

`DiffDrive` uses its configured `wheel_separation`; it does not read mesh
edges to calculate the parameter. The results below confirm the influence
of collision width on effective kinematic separation in the current
contact model. Moving the joints to ±0.275 m would change the geometry and
should not be used to correct odometry.

## Method

`scripts/wheel_track_validation/run_test.py` expands the source Xacro,
converts URDF to SDF, and runs a robot copy in an independent flat world.
Joints, collisions, mass, inertia, and the drive plugin are retained;
visuals/sensors are removed. Each run has its own `GZ_PARTITION` and uses
no ROS bridge. Test commands go directly to DiffDrive, bypassing the wheel
limiter, Nav2, and EKF.

The main physics step is 0.004 s, matching the restaurant main world;
ground truth and wheel odometry come from separate plugins at 50 Hz.
After settling for 3 s, the robot drives straight at 0.1 m/s and rotates at
±0.3 and ±0.5 rad/s, with 10 s phases and 2 s pauses. Each measurement uses
the same 7.5 s simulation-time window, excluding the first 2 s of the phase.
Timestamps are interpolated and yaw unwrapped before calculating differences.
No trajectory fitting is performed.

For the same wheel motion:

```text
b_effective = b_configured × Δyaw_wheel / Δyaw_ground_truth
yaw_error_percent = 100 × (Δyaw_wheel / Δyaw_ground_truth − 1)
```

## Measured results

The angles below cover 7.5 s of the +0.5 rad/s rotation phase.

| Configuration | Physics step | Wheel Δyaw (rad) | Body Δyaw (rad) | Angular error | Effective b (m) |
|---|---:|---:|---:|---:|---:|
| Track 0.50; wheel width 0.05 | 0.004 s | 3.750000 | 3.409166 | +9.9976% | 0.549988 |
| Track 0.55; wheel width 0.05 | 0.004 s | 3.750000 | 3.750076 | −0.0020% | 0.549989 |
| Track 0.50; collision width 0.025 | 0.004 s | 3.750000 | 3.571513 | +4.9975% | 0.524988 |
| Track 0.50; wheel width 0.05 | 0.001 s | 3.750000 | 3.409110 | +9.9994% | 0.549997 |

At a 4 ms step, both rotation directions and both speeds give effective b
between 0.549980 and 0.549989 m with 0.05 m wheel width. The 0.55 m track has
a maximum absolute angular error of 0.00329%. Every straight phase measures
0.750000 m in both wheel odometry and ground truth, differing by less than 10⁻⁹ m.

Reducing only collision width while retaining joint centers and inertia
moves effective b to 0.524979–0.524988 m. This provides further evidence of
wheel-width contact effects; individual solver contact points have not been
identified. The results do not establish that every engine uses outer edges.

## Reproduction and data

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/wheel_track_validation/run_test.py --track 0.5 \
  --output benchmark/wheel_track/new_baseline
python3 scripts/wheel_track_validation/run_test.py --track 0.55 \
  --output benchmark/wheel_track/new_calibrated
python3 scripts/wheel_track_validation/run_test.py --track 0.5 \
  --collision-width 0.025 --output benchmark/wheel_track/new_width_control
python3 scripts/wheel_track_validation/run_test.py \
  --output benchmark/wheel_track/new_source_config
python3 -m unittest discover -s scripts/wheel_track_validation -p 'test_*.py'
```

The output directory must be new. Each run saves `robot.urdf`, `world.sdf`,
logs, `ground_truth.csv`, `wheel.csv`, and `summary.json`. Recorded data is in
`benchmark/wheel_track/{baseline_050_02,baseline_050_4ms,calibrated_055_4ms,width_control_025_4ms}/`.
`baseline_050_02` uses a 1 ms step; the other runs use 4 ms. Four unit tests
cover yaw unwrapping, timestamp offsets, rejection of missing data, and track
consistency between Xacro/DiffDrive and the wheel limiter without moving wheels.

This validates a simulated robot in a flat world, not hardware or the entire
restaurant floor. Changes to friction, collisions, or the engine require recalibration.

## Applied configuration

`properties.xacro` retains `wheel_track = 0.50 m` for geometry and adds
`wheel_separation_multiplier = 1.10`. `gazebo.xacro` passes their product to
DiffDrive's `wheel_separation`: **0.55 m**. This multiplication happens in
Xacro; it is not a separate multiplier parameter of the Gazebo plugin. The
wheel limiter also defaults to **0.55 m** for consistent wheel speed and
acceleration limiting. The EKF tool uses half the effective track, **0.275 m**,
when limiting wheel speeds.

After changes, rebuild `amr_description` and `amr_gazebo`, source the overlay,
and restart simulation to load the plugin configuration:

```bash
colcon build --symlink-install --packages-select amr_description amr_gazebo
source install/setup.bash
```

Omit `--track` to test the source configuration directly; `--track 0.5` and
`--track 0.55` remain available for controls using model copies.
`benchmark/wheel_track/source_calibrated_055_4ms/` directly tested the updated
source: 0.75 m straight travel matched ground truth and maximum angular error
was 0.00329% over four rotation phases. Both packages built successfully;
six wheel-limiter tests and four measurement/configuration tests passed.

Reference: [Gazebo Harmonic DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html).
