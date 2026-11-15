# AMCL Kidnapped Robot Recovery

## Purpose and implementation

The recovery benchmark localizes the restaurant AMR after an operator moves the complete robot model to another collision-free position in Gazebo. It uses native Nav2 AMCL, active LiDAR observations, and a bounded recovery policy. The controller receives robot-accessible evidence only. A separate evaluator uses Gazebo ground truth after the robot has stopped.

The current strategy identifier is:

```text
event_reseed_candidate_hold_v3_post_stop_50mm_5deg
```

Implementation references in the workspace:

| File | Responsibility |
| --- | --- |
| `scripts/navigation_validation/unified.launch.py` | Gazebo, EKF, Map Server, AMCL, Nav2, RViz and command ownership |
| `scripts/navigation_validation/runtime.py` | Command gate and observable navigation action proxy |
| `scripts/navigation_validation/run_kidnapped.py` | Session management, observation extraction, stopping, recording and independent evaluation |
| `scripts/navigation_validation/kidnapped_policy.py` | Robot-only recovery decisions, particle diagnostics and scan–map agreement |
| `scripts/navigation_validation/kidnapped_amcl.yaml` | AMCL overrides for kidnapped experiments |
| `scripts/navigation_validation/world_to_map.json` | Hash-validated geometric registration used by the evaluator |

The world is `src/amr_gazebo/worlds/restaurant_blueprint.world`. Map Server loads `src/amr_navigation/maps/restaurant_main.yaml`, which references `restaurant_main.pgm`. The YAML supplies image resolution and origin; the PGM alone does not define the map coordinates.

## Why the policy was changed

A global particle initialization explores the map, but resampling can subsequently concentrate particles around an incorrect hypothesis. Repeated restaurant features and the LiDAR's 150-degree field of view can make several poses look plausible. Increasing the maximum particle count alone does not prevent an incorrect mode from surviving.

The earlier policy reinitialized particles after a full turn when internal convergence had not been declared. This could discard an improving hypothesis. The current policy first preserves a promising candidate through stationary observations, and reinitializes only when sustained evidence indicates stagnation. It does not use the true robot location to decide which hypothesis to preserve.

The particle count and AMCL recovery gains were **not increased for v3**. The improvement is in observation scheduling, candidate preservation, evidence freshness and reseed timing.

## Session sequence

1. Acquire recovery command ownership before allowing recovery motion.
2. Request cancellation of native navigation goals. Reject the experiment if an observed goal remains active after the cancellation wait.
3. Send zero commands and confirm a stationary robot using fresh EKF twist and limited-command output.
4. Record the baseline. Wait for the operator to move the complete `amr_robot` model, resume simulation, verify clearance and enter `k`.
5. Check sensor/TF readiness and invoke `/reinitialize_global_localization` with an empty request. No target coordinates or ground-truth pose are sent to AMCL.
6. Rotate slowly to acquire observations from different directions. Stop when safety observations are insufficient.
7. Hold a promising candidate for stationary scan updates, or request one bounded reseed when evidence has stagnated.
8. On internal convergence, stop and collect a new stationary validation window.
9. Evaluate the recorded AMCL poses against independently transformed ground truth. Save the result and retain command ownership in a stopped state.

The operator must not publish a new initial pose after kidnapping. Such a publication invalidates the experiment.

## Command ownership and safety

Nav2 commands enter `/benchmark/nav_cmd_vel`. Recovery commands enter `/benchmark/recovery_cmd_vel`. Only the gate publishes `/benchmark/owned_cmd_vel`, which is remapped to the Wheel Limiter input for this launch. The ROS–Gazebo bridge receives the limited output. Recovery does not bypass wheel speed or acceleration limits.

In recovery mode the gate rejects competing recovery writers. The recorder checks the publisher graph and verifies that the limiter subscribes to the owned topic. The gate outputs zero when a selected command becomes stale. Navigation goals through the proxy are rejected while recovery owns the gate.

| Safety parameter | Value | Meaning |
| --- | --- | --- |
| Gate command watchdog | 0.35 wall seconds | Stale command produces zero output |
| Recovery linear command | 0 m/s | No autonomous translation |
| Recovery angular command | 0.20 rad/s | Slow in-place observation |
| Default total rotation limit | 4π rad | Two accumulated EKF turns, including direction changes |
| Default recovery timeout | 120 simulation seconds | Observation-policy budget |
| Wall-time recovery limit | Twice the configured timeout | Prevent an indefinitely slow simulation from extending recovery |
| CLI timeout cap | 180 simulation seconds | Maximum allowed `--timeout` |
| CLI rotation cap | Two turns | Maximum allowed `--max-turns` |
| Required finite scan returns | At least 60 | Invalid or insufficient observations cannot authorize motion |
| Consecutive valid scans | Three | Required before motion is enabled |
| Minimum observed radial clearance | 0.575 m from `base_footprint` | Approximately 0.425 m body corner radius plus 0.15 m margin |
| Safety scan age | Less than 0.5 simulation seconds | Stale scans cannot authorize motion |
| Sensor/odometry/clock receipt freshness | Less than 1.5 wall seconds | Robot-only readiness check |
| Clock advance watchdog | Two wall seconds | A stalled simulation stops the active experiment |
| Stop confirmation | At least 0.5 simulation seconds | EKF speed <0.01 m/s and <0.02 rad/s; limited commands <0.001 in both components |
| Stop confirmation timeout | Ten wall seconds | Failure to confirm stop makes the session incomplete |

NaN and negative-infinity scan values invalidate the safety observation. Positive infinity is not treated as proof that the entire swept region is free. Scan points are transformed into the robot frame before checking radial clearance.

The 150-degree LiDAR does not observe the complete swept footprint. The operator must independently verify full-circle clearance, including the rear and corners, before confirming kidnapping. These guards do not constitute a hardware protective-stop system.

## Evidence used for internal convergence

### Particle distribution

Particle weights are normalized. The dominant hypothesis mass is the weight fraction within 0.5 m and ±15 degrees of the current AMCL estimate. Effective sample size is also logged:

```text
ESS = 1 / sum(normalized_weight_i²)
```

ESS alone cannot establish localization accuracy. A resampled, multimodal cloud can have nearly uniform weights and a high ESS. The policy therefore combines dominant mass with covariance, scan–map agreement and TF stability.

### Scan–map agreement

The diagnostic projects every third usable finite return into `map` at the scan timestamp. It measures each endpoint's distance to occupied cells using a distance field. An endpoint matches when that distance is at most 0.20 m. Unknown or outside-map endpoints count as nonmatches.

This is a geometric diagnostic, not AMCL's exact likelihood function, a ray-consistency test, or a measurement of information gain. Similar map regions can still pass it at an incorrect pose.

### Thresholds

| Evidence parameter | Value |
| --- | --- |
| Maximum AMCL x/y variance | 0.0625 m² per axis, equivalent to 0.25 m standard deviation |
| Maximum AMCL yaw variance | (10°)², expressed in rad² |
| Minimum dominant particle mass | 0.90 |
| Minimum scan–map endpoint match fraction | 0.70 |
| Endpoint subsampling | Every third usable return; at least 20 sampled endpoints |
| Maximum `map → odom` translation drift | 0.10 m relative to the first retained evidence sample |
| Maximum `map → odom` yaw drift | 5° |
| Evidence hold for internal convergence | At least five simulation seconds and five distinct scan timestamps |
| Minimum observation rotation before candidate hold/convergence | π rad within the current attempt |
| Normal particle-cloud age limit | 1.5 simulation seconds |
| Particle-cloud age limit during candidate hold | Eight simulation seconds |

Covariance thresholds describe internal uncertainty. They are deliberately separate from the independent 50 mm / 5-degree accuracy requirement.

AMCL pose and particle messages must belong to the current global-localization epoch. Cached messages from before a reseed cannot establish convergence afterward. A mode switch resets the TF-stability history. Repeated polling of one scan does not add new evidence.

## Decision states

| State | Action and transition |
| --- | --- |
| `ROTATE` | Publish slow angular motion while safety scans remain valid; collect fresh localization evidence |
| `WAIT_SCAN` | Publish zero, reset the convergence history and wait for safe observations; sensor loss does not itself justify reseeding |
| `HOLD_CANDIDATE` | Stop and request stationary scan updates to preserve a promising mode |
| `RESEED` | Stop, call native global localization, advance the evidence epoch and start the second attempt |
| `INTERNAL_CONVERGED` | End recovery motion and start independent post-stop validation |
| `ASSISTANCE` | End bounded recovery without success; retain zero command ownership and request operator assistance |
| `ABORT_CLOCK` | Stop because simulation time moved backward |

### Preserving a promising candidate

A candidate enters `HOLD_CANDIDATE` after two simulation seconds of good, stable evidence, once the minimum observation rotation has been reached. The runner confirms stop and calls `/request_nomotion_update` every 0.5 simulation seconds.

If good evidence persists for the full internal hold, the policy declares internal convergence. If evidence degrades for 1.5 seconds, or candidate observation lasts eight seconds without convergence, rotation resumes. A five-second cooldown prevents immediate repeated candidate holds.

Jazzy AMCL can suppress particle-cloud publication during forced no-motion updates. During the bounded candidate hold only, the runner may retain the last measured cloud for up to eight seconds, while still requiring fresh AMCL poses, fresh scans and stable TF. Particle age and hold state are recorded. If localization evidence becomes stale but safety scans remain valid, rotation can resume to obtain new AMCL/cloud updates; the runner does not remain stuck waiting for a cloud that stationary updates do not publish.

### Event-triggered reseeding

The policy uses a diagnostic progress score:

```text
quality = min(dominant_mass, endpoint_match_fraction)
          + 0.10 when covariance passes
```

This score is a heuristic, not a posterior probability. An improvement of at least 0.08 resets the stagnation timer.

A reseed is allowed after at least 20 simulation seconds in the attempt and at least π/2 accumulated rotation when degraded evidence persists for eight seconds without material progress. Another trigger is a full-view budget of 2π accompanied by at least three seconds of degraded evidence, subject to the same cooldown. A full turn alone does not destroy a promising mode.

There is at most **one reseed after the initial global initialization**, giving at most two attempts. Total time and accumulated-rotation limits remain in force across attempts. The robot does not continue reseeding until ground truth says it is correct.

## AMCL profile and tuning history

Launch the experiment with `kidnapped_profile:=true` to apply the dedicated YAML layer:

```yaml
amcl:
  ros__parameters:
    min_particles: 500
    max_particles: 3000
    recovery_alpha_fast: 0.1
    recovery_alpha_slow: 0.001
```

| Parameter or behavior | Current setting | Tuning rationale/history |
| --- | --- | --- |
| Minimum particle count | 500 | Retained; not raised in v3 |
| Maximum particle count | 3000 | Existing kidnapped-specific resource cap; not increased in v3 |
| Fast/slow recovery gains | 0.1 / 0.001 | Retained; not interpreted as a fixed random-particle percentage |
| Candidate preservation | New in v3 | Allows additional stationary observations before a promising hypothesis is lost |
| Reseed trigger | Evidence-driven in v3 | Replaces unconditional full-turn reseeding with stagnation/cooldown checks |
| Observation freshness after reseed | Current epoch required | Prevents an old estimate or cloud from immediately satisfying the new attempt |
| Independent position/yaw acceptance | Strictly below 0.05 m and 5° | Tightened from the earlier 0.25 m / 10° evaluator |
| Independent validation window | New stationary window after stop | Fixes premature `UNVERIFIED` caused by clipping sparse AMCL samples to a pre-stop window |

AMCL's KLD sampling can reduce the actual count toward the minimum. A larger maximum is not a guarantee that the filter continuously maintains that many particles. Native random recovery and explicit global initialization are distinct mechanisms.

No robot geometry, wheel friction, wheel dimensions, LiDAR/IMU noise, EKF covariance or process-noise matrix was changed for this policy improvement.

## Independent accuracy validation

Ground truth is used only after the robot-only controller has terminated and stop has been confirmed. Robot readiness and candidate/reseed decisions do not depend on ground truth.

The evaluator loads the existing geometric `world → map` survey and verifies hashes of the world, PGM and YAML. It does not assume map coordinates equal world coordinates, fit the AMCL trajectory to ground truth, or force the spawn error to zero. Runtime map identity is monitored and Map Server must use the surveyed YAML.

A new stationary window begins after `stopped_sim_s`. The runner keeps publishing zero and requests no-motion updates every 0.5 simulation seconds until it has at least five distinct post-stop AMCL observations spanning the configured hold, normally five seconds. The maximum AMCL sample gap is 1.5 seconds. A bounded collection timeout prevents indefinite waiting.

GT is interpolated at AMCL header timestamps with a maximum bracketing gap of 0.10 seconds. There is no extrapolation and no interpolation across detected pose discontinuities. Success requires every evaluated sample to satisfy:

```text
position_error < 0.05 m
abs(wrapped_yaw_error) < 5 degrees
```

This is independent localization accuracy, not the robot's estimated goal-checker tolerance. The geometric survey previously showed held-out surface residuals up to approximately 8 cm at the 95th percentile in surveyed regions. Those statistics are not a global bound or proof of sub-5-cm registration accuracy. The acceptance threshold is applied as requested without subtracting registration uncertainty to improve results.

| Final status | Interpretation |
| --- | --- |
| `SUCCESS` | Internal convergence followed by a complete independent stationary window below both accuracy limits |
| `FAILURE` | Bounded observation exhausted, or adequate independent evidence exceeds an accuracy limit |
| `UNVERIFIED` | Missing survey, insufficient post-stop coverage, timestamp-incompatible GT or excessive AMCL sample gaps |
| `INCOMPLETE` | Operator interruption, safety/data failure or inability to confirm stop |

Recovery time to the controller's decision, stopped time and independent evaluation time are recorded separately. Accumulated angle is derived from EKF yaw; evaluation travel distance uses GT after recovery starts and excludes teleportation.

## Running the experiment

Terminal 1:

```bash
cd /home/quangtran/Projects/capstone_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=116
export GZ_PARTITION=amr_unified116
export ROS_LOG_DIR="$PWD/report/kidnapped_runtime_logs"
mkdir -p "$ROS_LOG_DIR"
ros2 launch "$PWD/scripts/navigation_validation/unified.launch.py" kidnapped_profile:=true
```

Terminal 2:

```bash
cd /home/quangtran/Projects/capstone_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=116
export GZ_PARTITION=amr_unified116
export ROS_LOG_DIR="$PWD/report/kidnapped_runtime_logs"
python3 scripts/navigation_validation/run_kidnapped.py --rotation-clearance-confirmed
```

Wait for `GATE HELD; robot stop confirmed`. Pause Gazebo, select the complete `amr_robot` model and move it to a collision-free location with full-circle clearance. Keep its vertical placement correct and do not move individual links. Resume simulation and enter `k` in Terminal 2. Do not use RViz `2D Pose Estimate` after kidnapping.

Use Ctrl+C to stop an active experiment. Before kidnapping, `q` followed by Enter also ends the session. Keep Terminal 1 running until Terminal 2 prints the final result and output directory. Only provide the clearance-confirmation flag when the swept region has actually been checked.

## Recorded data and reproducibility

Each experiment creates a new directory under `benchmark/kidnapped/run_*/`. Existing runs are not overwritten.

- `recovery_observations.txt`: scan safety, particle count/ESS/mass, map-match fraction, covariance/coherence, TF, cloud age and candidate-hold state.
- `events.txt` and `events.jsonl`: operator marker, observed GT discontinuities, phase transitions, reasons, attempts and rotation.
- `ground_truth_odom.txt`, `amcl_pose.txt`, `odometry_filtered.txt`, `localization_error.txt`: timestamped trajectories and evaluation data.
- `recovery_validation.txt`: the independent post-stop accuracy window when available.
- `runtime_parameters.yaml`: actual node parameters, including the AMCL profile.
- `metadata.json` and `summary.txt`: strategy, thresholds, map identity, survey compatibility, timing, final errors, rotation, travel and status reasons.
- `bag/`: original ROS messages, including full scans and particle poses/weights. The lightweight `particles.txt` records counts only.

Separate results by strategy identifier, parameter profile and thresholds. Navigation results, no-motion smoke sessions and complete kidnapped experiments are different experiment categories.

## Validation evidence and limits

The implementation has 30 kidnapped-specific automated tests and 42 automated tests across the navigation-validation directory. These cover candidate preservation, persistent degradation, progress-sensitive reseeding, mode switches, stale evidence, clock rollback, command ownership, stationary-window collection, strict accuracy boundaries and false convergence.

A short isolated Gazebo smoke test confirmed startup, command ownership, stopping and recording without teleportation or recovery motion. It was retained as `INCOMPLETE` with a smoke-only reason rather than presented as a successful recovery experiment. The operator has subsequently reported good recovery behavior; no aggregate success-rate or optimality claim is inferred from that feedback.

Remaining limits include sensor intermittency, map aliasing, registration uncertainty, the absence of a guaranteed 360-degree safety observation, and the lack of a controlled multi-location success-rate/CPU study. The policy deliberately requests operator assistance rather than translating through an unverified region.

## Technical references

- [Nav2 Jazzy AMCL source: global localization, no-motion updates and particle publication](https://api.nav2.org/nav2-jazzy/html/amcl__node_8cpp_source.html)
- [Nav2 Jazzy particle-filter source: resampling, adaptive counts and recovery averages](https://api.nav2.org/nav2-jazzy/html/pf_8c_source.html)
- [Nav2 Jazzy likelihood-field sensor model](https://api.nav2.org/nav2-jazzy/html/likelihood__field__model_8cpp_source.html)
- [Dieter Fox: research on active localization and observation selection](https://www.cs.cmu.edu/~dfox/research.html)
