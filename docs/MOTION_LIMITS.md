# Default motion limits

The tangential acceleration limit of each drive wheel is **0.5 m/s²**, applied
symmetrically during acceleration and braking. The geometric track remains
0.50 m; command conversion and odometry use the validated effective track
**b = 0.55 m**.

For forward velocity `v` and angular velocity `w`:

```text
v_left  = v − b*w/2
v_right = v + b*w/2
a_left  = a − b*alpha/2
a_right = a + b*alpha/2
```

Since `|a_left|, |a_right| <= 0.5`:

- Straight motion (`alpha=0`): **|a| <= 0.5 m/s²**.
- In-place rotation (`a=0`): **|alpha| <= 2×0.5/0.55 ≈ 1.818181818 rad/s²**.
- Combined motion: **|a| + 0.275×|alpha| <= 0.5**. For example, at
  `a=0.25 m/s²`, angular acceleration is limited to approximately
  `0.90909 rad/s²`.

The two independent maxima cannot both be applied while translating and
rotating. `wheel_limiter.py` limits wheel velocity increments at each tick,
preserving their ratio; this layer enforces the coupled command constraint.
The separate axis limits in Nav2/Gazebo do not represent the entire coupled
feasible region. Measured body acceleration still depends on dynamics and
has not been verified by command unit tests.

## Applied configuration

| Layer | Linear acceleration | Angular acceleration |
|---|---:|---:|
| DWB | ±0.5 m/s² | ±1.818181818 rad/s² |
| Velocity smoother | ±0.5 m/s² | ±1.818181818 rad/s² |
| Gazebo DiffDrive | ±0.5 m/s² | ±1.818181818 rad/s² |
| Behavior Spin | — | 1.818181818 rad/s² |

Gazebo limits are derived through Xacro expressions from
`wheel_acceleration_limit` and the effective track. DWB and the smoother
share the same braking limits, resolving the previous 0.8 versus 0.6 rad/s²
configuration discrepancy.

The default maximum navigation forward velocity is **0.30 m/s** in both DWB
(`max_vel_x`, `max_speed_xy`) and the smoother (`max_velocity[0]`). This does
not guarantee constant travel at 0.30 m/s: Nav2 slows down as needed. The
maximum angular velocity remains **0.80 rad/s**. Do not interpret 0.30 m/s
as the limit of each individual wheel during turns.

## Testing and loading the configuration

Unit tests cover straight motion, pure rotation, combined motion, both
directions, and consistency between Xacro, the wheel limiter, DWB, and the
smoother.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select amr_description amr_gazebo amr_navigation
source install/setup.bash
```

Restart Terminal 1 so Gazebo and Nav2 load the updated configuration. Critic
weights and goal tolerances were unchanged in this update. Reducing speed
and aligning acceleration limits does not establish that all oscillatory
rotation near the goal has been resolved.
