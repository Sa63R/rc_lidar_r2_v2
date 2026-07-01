# Nav2 MPPI Wall Clearance Tuning Log 02

- Date: 2026-07-01
- Workspace: `/home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio`
- Package: `src/rc_nav2_bringup`
- Active params file: `src/rc_nav2_bringup/config/fake_nav_params.yaml`
- Goal: second-pass tuning after automated clearance evaluation found no collision, but very small minimum clearance.

## Previous Automated Result

First isolated run, default loop:

```text
collision_sample_count: 0
min_clearance_m: 0.01464466094067262
p05_clearance_m: 0.01464466094067262
p50_clearance_m: 0.06464466094067262
```

First isolated run, centerline loop:

```text
collision_sample_count: 0
min_clearance_m: 0.01464466094067262
p05_clearance_m: 0.01464466094067262
p50_clearance_m: 0.06464466094067262
```

Interpretation:

- The robot did not geometrically collide in the evaluator.
- The route still passes very close to obstacles.
- Part of the minimum clearance comes from the configured start pose `(-0.60, 0.40, 3.14159)`, which is already close to the wall.
- The top corridor near the inner obstacle is also tight, so the next pass should be more conservative but not overly aggressive.

## Second-Pass Applied Changes

Applied to `src/rc_nav2_bringup/config/fake_nav_params.yaml`:

```yaml
FollowPath.CostCritic.cost_weight: 8.0 -> 12.0
FollowPath.PathAlignCritic.cost_weight: 8.0 -> 6.0
local_costmap.inflation_layer.cost_scaling_factor: 3.0 -> 2.5
local_costmap.inflation_layer.inflation_radius: 0.55 -> 0.60
global_costmap.inflation_layer.cost_scaling_factor: 3.0 -> 2.5
global_costmap.inflation_layer.inflation_radius: 0.55 -> 0.60
```

## Why These Changes

`CostCritic.cost_weight` was increased so MPPI penalizes trajectories through inflated obstacle cost more strongly.

`PathAlignCritic.cost_weight` was reduced so the controller can deviate from the global path when the global path runs close to a wall.

`inflation_radius` was widened from `0.55` to `0.60` and `cost_scaling_factor` was lowered from `3.0` to `2.5` so obstacle cost decays more slowly and the controller sees a wider soft safety band.

## Validation Commands

Use an isolated ROS domain so this automated test does not talk to other running Nav2 nodes:

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
  --report parameter_settings/nav2_eval_runs/2026-07-01_second_pass_default_clearance.md \
  --json parameter_settings/nav2_eval_runs/2026-07-01_second_pass_default_clearance.json
```

Centerline loop evaluator:

```bash
python3 src/rc_nav2_bringup/tools/evaluate_fake_nav_clearance.py \
  --map src/rc_nav2_bringup/maps/red_manual_simple.yaml \
  --goals=-8.70,0.15,3.14159\;-8.70,-4.45,-1.5708\;-1.60,-4.45,0.0\;-1.60,0.15,1.5708\;-0.60,0.40,3.14159 \
  --goal-timeout 60 \
  --report parameter_settings/nav2_eval_runs/2026-07-01_second_pass_centerline_clearance.md \
  --json parameter_settings/nav2_eval_runs/2026-07-01_second_pass_centerline_clearance.json
```

## Acceptance Rule

Keep this second-pass tuning if:

- `collision_sample_count == 0`
- all default loop goals succeed
- clearance improves or at least does not degrade outside the known tight start pose
- the controller does not become noticeably less reliable

If the controller starts aborting more often, roll back to first-pass values or try a smaller middle point:

```yaml
CostCritic.cost_weight: 10.0
PathAlignCritic.cost_weight: 7.0
cost_scaling_factor: 2.8
inflation_radius: 0.58
```
