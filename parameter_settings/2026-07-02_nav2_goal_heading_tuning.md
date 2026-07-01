# Nav2 Goal Heading Tuning

- Date: 2026-07-02
- Workspace: `/home/jwt/jwt/rc/rc_lidar_r2_v3_pointlio`
- Package: `src/rc_nav2_bringup`
- Active params file: `src/rc_nav2_bringup/config/fake_nav_params.yaml`
- Goal: make RViz/Nav2 green goal arrow orientation matter more at the final pose.

## Problem

RViz `Nav2 Goal` can publish target orientation, but the robot can report success after reaching the target position while still visibly not matching the green arrow direction.

The previous config allowed about `0.35 rad`, roughly `20 deg`, of yaw error:

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.25
  yaw_goal_tolerance: 0.35
```

That makes Nav2 accept a goal before the body fully aligns with the requested orientation.

## Applied Changes

Applied to `src/rc_nav2_bringup/config/fake_nav_params.yaml`:

```yaml
general_goal_checker.xy_goal_tolerance: 0.25 -> 0.15
general_goal_checker.yaw_goal_tolerance: 0.35 -> 0.10
FollowPath.GoalAngleCritic.cost_weight: 3.0 -> 8.0
FollowPath.GoalAngleCritic.threshold_to_consider: 0.8 -> 1.2
```

## Why

`yaw_goal_tolerance: 0.10` means the final accepted orientation error is about `5.7 deg`, so the robot must align much closer to the RViz green arrow before Nav2 returns success.

`GoalAngleCritic.cost_weight: 8.0` makes MPPI care more about final heading while approaching the goal.

`GoalAngleCritic.threshold_to_consider: 1.2` makes the final heading critic start influencing trajectories farther from the goal, instead of waiting until the last moment.

`xy_goal_tolerance: 0.15` keeps the final position acceptance tighter as well, so the robot does not accept a goal too early while still offset from the clicked point.

## Watch Items

This may slightly increase final rotation near walls. If wall clearance gets worse, keep `yaw_goal_tolerance: 0.10` but reduce:

```yaml
GoalAngleCritic.cost_weight: 6.0
GoalAngleCritic.threshold_to_consider: 1.0
```

If final alignment is still too loose, try:

```yaml
yaw_goal_tolerance: 0.05
```

but that is stricter and may make goals near obstacles harder to finish.
