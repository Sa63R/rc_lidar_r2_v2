# Nav2 MPPI Wall Clearance Tuning Log

- Date: 2026-07-01
- Workspace: `/home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio`
- Package: `src/rc_nav2_bringup`
- Active launch: `ros2 launch rc_nav2_bringup fake_nav.launch.py`
- Active params file: `src/rc_nav2_bringup/config/fake_nav_params.yaml`
- Symptom: fake robot body marker scrapes or collides with map walls during MPPI navigation.
- Status: first-pass tuning applied to `src/rc_nav2_bringup/config/fake_nav_params.yaml`.

## Applied Changes

Applied after review:

```yaml
FollowPath.CostCritic.cost_weight: 4.0 -> 8.0
FollowPath.PathAlignCritic.cost_weight: 14.0 -> 8.0
local_costmap.inflation_layer.cost_scaling_factor: 5.0 -> 3.0
local_costmap.inflation_layer.inflation_radius: 0.40 -> 0.55
global_costmap.inflation_layer.cost_scaling_factor: 5.0 -> 3.0
global_costmap.inflation_layer.inflation_radius: 0.40 -> 0.55
```

## Current Controller

The fake navigation launch is using Nav2 MPPI:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      motion_model: "Omni"
```

The fake robot and costmaps use a rectangular footprint:

```yaml
footprint: "[[-0.35, -0.40], [-0.35, 0.40], [0.35, 0.40], [0.35, -0.40]]"
```

This matches the fake robot marker dimensions in `fake_nav.launch.py`:

```yaml
body_length: 0.70
body_width: 0.80
```

The collision symptom is therefore more likely caused by insufficient clearance cost, not by an obviously missing footprint.

## Tuning Goal

Make MPPI keep more distance from walls while preserving the current omnidirectional behavior.

First-pass goal:

- Do not change the motion model.
- Do not change the fake robot size.
- Do not change the map.
- Prefer safer clearance over tight path tracking.
- Change only a small set of high-impact parameters first.

## Recommended First-Pass Changes

### 1. Local Costmap Inflation

Current:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        cost_scaling_factor: 5.0
        inflation_radius: 0.40
```

Recommended:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

Reason:

- `inflation_radius` controls how far obstacle cost expands from walls.
- Larger radius makes MPPI see wall clearance earlier.
- Lower `cost_scaling_factor` makes the inflated cost decay more slowly, creating a wider soft safety zone.
- This should reduce wall-hugging behavior.

### 2. Global Costmap Inflation

Current:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        cost_scaling_factor: 5.0
        inflation_radius: 0.40
```

Recommended:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

Reason:

- The global planner can create paths close to walls if the global inflation is too narrow.
- If the global path is wall-hugging, MPPI may fight between following the path and avoiding the wall.
- Matching local/global inflation makes the global path and local controller agree better.

### 3. MPPI CostCritic

Current:

```yaml
FollowPath:
  CostCritic:
    enabled: true
    cost_power: 1
    cost_weight: 4.0
    critical_cost: 300.0
    consider_footprint: true
    collision_cost: 1000000.0
    near_goal_distance: 0.8
```

Recommended:

```yaml
FollowPath:
  CostCritic:
    enabled: true
    cost_power: 1
    cost_weight: 8.0
    critical_cost: 300.0
    consider_footprint: true
    collision_cost: 1000000.0
    near_goal_distance: 0.8
```

Reason:

- `CostCritic` is the main critic that makes MPPI prefer trajectories away from occupied/inflated costmap cells.
- `consider_footprint: true` is already correct because the robot is rectangular.
- Increasing `cost_weight` makes clearance more important during trajectory scoring.

### 4. MPPI PathAlignCritic

Current:

```yaml
FollowPath:
  PathAlignCritic:
    enabled: true
    cost_power: 1
    cost_weight: 14.0
    offset_from_furthest: 20
    trajectory_point_step: 4
    threshold_to_consider: 0.5
```

Recommended:

```yaml
FollowPath:
  PathAlignCritic:
    enabled: true
    cost_power: 1
    cost_weight: 8.0
    offset_from_furthest: 20
    trajectory_point_step: 4
    threshold_to_consider: 0.5
```

Reason:

- `PathAlignCritic` encourages the robot to stay close to the global path.
- If the global path is close to a wall, a high path alignment weight can make MPPI hug that path too tightly.
- Reducing it gives `CostCritic` more authority to choose a safer offset trajectory.

## Combined Patch Candidate

Apply this candidate only after reviewing the log:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      CostCritic:
        cost_weight: 8.0
      PathAlignCritic:
        cost_weight: 8.0

local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Parameters Not Changed In First Pass

Do not change these yet:

```yaml
time_steps: 40
model_dt: 0.05
batch_size: 1200
vx_std: 0.5
vy_std: 0.5
wz_std: 0.8
vx_max: 1.2
vx_min: -0.6
vy_max: 1.2
wz_max: 2.5
temperature: 0.3
gamma: 0.015
motion_model: "Omni"
```

Reason:

- Current prediction horizon is `40 * 0.05 = 2.0s`.
- Current local costmap is `6m x 6m`, so the robot has about `3m` local radius.
- At `1.2m/s`, a 2 second prediction projects about `2.4m`, which fits inside the local costmap.
- The current issue looks more like clearance-cost tuning than prediction-window tuning.

## Expected Behavior After First Pass

Expected improvements:

- Robot center should choose paths farther from walls.
- Rectangular body marker should stop scraping wall boundaries in common turns.
- MPPI may look slightly more conservative near narrow passages.

Possible side effects:

- Paths may become less tight.
- Navigation may slow down near walls.
- Narrow passages may become harder if the map is very tight relative to the 0.70m x 0.80m footprint.

## Validation Checklist

Run:

```bash
cd /home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rc_nav2_bringup fake_nav.launch.py \
  map:=/home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio/src/rc_nav2_bringup/maps/red_manual_simple.yaml \
  start_x:=-0.60 \
  start_y:=0.40 \
  start_yaw:=3.14159
```

Check:

- The body marker no longer intersects walls.
- Global path is not glued to wall pixels.
- Local costmap inflation creates visible clearance.
- MPPI does not oscillate while avoiding walls.
- Goal can still be reached in narrow areas.

## If It Still Scrapes Walls

Second-pass candidates:

```yaml
inflation_radius: 0.65
cost_scaling_factor: 2.5
CostCritic.cost_weight: 12.0
PathAlignCritic.cost_weight: 6.0
```

Use these only if the first pass still hugs walls.

## If It Becomes Too Conservative

Rollback candidates:

```yaml
inflation_radius: 0.50
cost_scaling_factor: 3.5
CostCritic.cost_weight: 6.0
PathAlignCritic.cost_weight: 10.0
```

Use these if the robot refuses reasonable narrow passages.
