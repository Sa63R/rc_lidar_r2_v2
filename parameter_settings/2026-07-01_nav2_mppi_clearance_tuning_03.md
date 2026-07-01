# Nav2 MPPI Wall Clearance Tuning Log 03

- Date: 2026-07-01
- Workspace: `/home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio`
- Package: `src/rc_nav2_bringup`
- Active params file: `src/rc_nav2_bringup/config/fake_nav_params.yaml`
- Goal: reduce rectangular body corner scraping in the top narrow corridor.

## Observation From Pass 02

Default loop with second-pass settings:

```text
goals: all SUCCEEDED
collision_sample_count: 0
min_clearance_m: 0.01464466094067262
p05_clearance_m: 0.01464466094067262
p50_clearance_m: 0.08535533905932738
worst sample: x=-4.764, y=0.309, yaw=-2.943
```

The minimum clearance is no longer only caused by the initial pose. The worst running sample is in the top corridor, where the robot yaw is about 11 degrees away from the corridor direction.

A quick map sweep showed that the same corridor can provide about `0.115 m` clearance when the `0.70 m x 0.80 m` rectangular footprint is aligned with the corridor. The very low clearance therefore looks like a body-angle problem more than a pure inflation-distance problem.

## Third-Pass Applied Change

Applied to `src/rc_nav2_bringup/config/fake_nav_params.yaml`:

```yaml
FollowPath.PathAngleCritic.cost_weight: 2.0 -> 5.0
```

## Why This Change

`PathAngleCritic` penalizes heading disagreement with the path direction. For an omnidirectional robot, MPPI can move sideways while the body yaw is not perfectly aligned with the corridor. That is usually acceptable for a round robot, but this fake robot uses a rectangular footprint, so a small yaw error can make the corners scrape walls.

Increasing this critic should make MPPI keep the body more parallel to narrow corridors without further shrinking the traversable space by raising inflation again.

## Validation Commands

```bash
export ROS_DOMAIN_ID=87
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rc_nav2_bringup fake_nav.launch.py \
  map:=/home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio/src/rc_nav2_bringup/maps/red_manual_simple.yaml \
  start_x:=-0.60 \
  start_y:=0.40 \
  start_yaw:=3.14159
```

Default loop evaluator:

```bash
python3 src/rc_nav2_bringup/tools/evaluate_fake_nav_clearance.py \
  --map src/rc_nav2_bringup/maps/red_manual_simple.yaml \
  --goal-timeout 60 \
  --ignore-first-sec 2.0 \
  --require-all-goals \
  --report parameter_settings/nav2_eval_runs/2026-07-01_third_pass_default_clearance.md \
  --json parameter_settings/nav2_eval_runs/2026-07-01_third_pass_default_clearance.json
```

Centerline loop evaluator:

```bash
python3 src/rc_nav2_bringup/tools/evaluate_fake_nav_clearance.py \
  --map src/rc_nav2_bringup/maps/red_manual_simple.yaml \
  --goals=-8.70,0.15,3.14159\;-8.70,-4.45,-1.5708\;-1.60,-4.45,0.0\;-1.60,0.15,1.5708\;-0.60,0.40,3.14159 \
  --goal-timeout 60 \
  --ignore-first-sec 2.0 \
  --require-all-goals \
  --report parameter_settings/nav2_eval_runs/2026-07-01_third_pass_centerline_clearance.md \
  --json parameter_settings/nav2_eval_runs/2026-07-01_third_pass_centerline_clearance.json
```
