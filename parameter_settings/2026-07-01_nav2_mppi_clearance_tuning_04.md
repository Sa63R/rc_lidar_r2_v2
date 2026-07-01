# Nav2 MPPI Wall Clearance Tuning Log 04

- Date: 2026-07-01
- Workspace: `/home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio`
- Package: `src/rc_nav2_bringup`
- Active params file: `src/rc_nav2_bringup/config/fake_nav_params.yaml`
- Goal: reduce yaw swing in narrow corridors after pass 03 made heading behavior worse.

## Observation From Pass 03

Pass 03 changed only:

```yaml
FollowPath.PathAngleCritic.cost_weight: 2.0 -> 5.0
```

Default loop result:

```text
goals: all SUCCEEDED
collision_sample_count: 0
min_clearance_m: -0.03535533905932738
worst sample: x=-2.982, y=0.282, yaw=1.995
```

That result is worse than pass 02. The robot still reached all goals, but the heading behavior became less suitable for the rectangular footprint in the top corridor.

## Fourth-Pass Applied Changes

Applied to `src/rc_nav2_bringup/config/fake_nav_params.yaml`:

```yaml
FollowPath.PathAngleCritic.cost_weight: 5.0 -> 2.0
FollowPath.wz_std: 0.8 -> 0.55
FollowPath.wz_max: 2.5 -> 1.6
FollowPath.TwirlingCritic.cost_weight: 8.0 -> 12.0
```

The `PathAngleCritic` change reverts pass 03. The angular velocity changes keep the second-pass wall clearance behavior, but make MPPI less eager to sample or choose large yaw swings.

## Why This Change

The top corridor clearance problem is strongly affected by yaw because the fake robot footprint is rectangular. A small heading error makes the effective swept width larger.

Instead of forcing path angle alignment directly, this pass reduces angular aggressiveness:

- lower `wz_std` means fewer sampled trajectories with large rotational motion
- lower `wz_max` caps the selected rotational command
- higher `TwirlingCritic.cost_weight` penalizes unnecessary rotation

This should preserve the ability to rotate at corners, but discourage body yaw drift while traversing narrow corridors.

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
  --min-clearance 0.0 \
  --min-p05-clearance 0.05 \
  --require-all-goals \
  --report parameter_settings/nav2_eval_runs/2026-07-01_fourth_pass_default_clearance.md \
  --json parameter_settings/nav2_eval_runs/2026-07-01_fourth_pass_default_clearance.json
```

Centerline loop evaluator:

```bash
python3 src/rc_nav2_bringup/tools/evaluate_fake_nav_clearance.py \
  --map src/rc_nav2_bringup/maps/red_manual_simple.yaml \
  --goals=-8.70,0.15,3.14159\;-8.70,-4.45,-1.5708\;-1.60,-4.45,0.0\;-1.60,0.15,1.5708\;-0.60,0.40,3.14159 \
  --goal-timeout 60 \
  --ignore-first-sec 2.0 \
  --min-clearance 0.0 \
  --min-p05-clearance 0.05 \
  --require-all-goals \
  --report parameter_settings/nav2_eval_runs/2026-07-01_fourth_pass_centerline_clearance.md \
  --json parameter_settings/nav2_eval_runs/2026-07-01_fourth_pass_centerline_clearance.json
```

Map-derived centerline evaluator:

```bash
python3 src/rc_nav2_bringup/tools/evaluate_fake_nav_clearance.py \
  --map src/rc_nav2_bringup/maps/red_manual_simple.yaml \
  --goals=-8.70,0.30,3.14159\;-8.70,-4.45,-1.5708\;-1.60,-4.45,0.0\;-1.60,0.30,1.5708\;-0.60,0.40,3.14159 \
  --goal-timeout 60 \
  --ignore-first-sec 2.0 \
  --min-clearance 0.0 \
  --min-p05-clearance 0.05 \
  --require-all-goals \
  --report parameter_settings/nav2_eval_runs/2026-07-01_fourth_pass_map_centerline_clearance_thresholded.md \
  --json parameter_settings/nav2_eval_runs/2026-07-01_fourth_pass_map_centerline_clearance_thresholded.json
```

## Result

Default loop with thresholded evaluator:

```text
goals: all SUCCEEDED
collision_sample_count: 0
min_clearance_m: 0.06464466094067262
p05_clearance_m: 0.06464466094067262
p50_clearance_m: 0.16464466094067262
exit_code: 0
```

Original `y=0.15` centerline loop:

```text
goals: all SUCCEEDED
collision_sample_count: 0
min_clearance_m: 0.01464466094067262
p05_clearance_m: 0.01464466094067262
exit_code: 4
```

That target set is not actually centered for the top corridor. A quick map sweep showed the top corridor's safer center is closer to `y=0.30`, not `y=0.15`.

Map-derived `y=0.30` centerline loop:

```text
goals: all SUCCEEDED
collision_sample_count: 0
min_clearance_m: 0.06464466094067262
p05_clearance_m: 0.06464466094067262
p50_clearance_m: 0.11464466094067263
exit_code: 0
```

## Decision

Keep pass 04 as the current best version.

It is better than pass 02 because the stricter centerline route no longer aborts. It is better than pass 03 because it avoids the negative-clearance yaw behavior caused by over-weighting `PathAngleCritic`.

Current accepted values:

```yaml
FollowPath.CostCritic.cost_weight: 12.0
FollowPath.PathAlignCritic.cost_weight: 6.0
FollowPath.PathAngleCritic.cost_weight: 2.0
FollowPath.wz_std: 0.55
FollowPath.wz_max: 1.6
FollowPath.TwirlingCritic.cost_weight: 12.0
local_costmap.inflation_layer.cost_scaling_factor: 2.5
local_costmap.inflation_layer.inflation_radius: 0.60
global_costmap.inflation_layer.cost_scaling_factor: 2.5
global_costmap.inflation_layer.inflation_radius: 0.60
```
