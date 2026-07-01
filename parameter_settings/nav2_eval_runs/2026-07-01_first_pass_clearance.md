# Fake Nav Clearance Evaluation

- Map: `src/rc_nav2_bringup/maps/red_manual_simple.yaml`
- Odom topic: `/odom`
- Action: `/navigate_to_pose`
- Samples: 9060
- Collision samples: 3598
- Min clearance: -0.035 m
- P05 clearance: -0.035 m
- P50 clearance: 0.065 m

## Goals

- 1: (-8.70, 0.35, 3.14) -> ABORTED
- 2: (-8.70, -4.45, -1.57) -> ABORTED
- 3: (-0.80, -4.45, 0.00) -> ABORTED
- 4: (-0.80, 0.35, 1.57) -> ABORTED
- 5: (-0.60, 0.40, 3.14) -> ABORTED

## Worst Sample

- x: -0.600
- y: 0.400
- yaw: -2.982
- clearance: -0.035 m
- collision: False
