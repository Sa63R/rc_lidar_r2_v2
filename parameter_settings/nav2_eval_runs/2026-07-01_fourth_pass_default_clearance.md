# Fake Nav Clearance Evaluation

- Map: `src/rc_nav2_bringup/maps/red_manual_simple.yaml`
- Odom topic: `/odom`
- Action: `/navigate_to_pose`
- Samples: 1814
- Ignored initial seconds: 2.00
- Evaluated samples: 1714
- Raw collision samples: 0
- Raw min clearance: 0.015 m
- Collision samples: 0
- Min clearance: 0.015 m
- P05 clearance: 0.065 m
- P50 clearance: 0.165 m

## Goals

- 1: (-8.70, 0.35, 3.14) -> SUCCEEDED
- 2: (-8.70, -4.45, -1.57) -> SUCCEEDED
- 3: (-0.80, -4.45, 0.00) -> SUCCEEDED
- 4: (-0.80, 0.35, 1.57) -> SUCCEEDED
- 5: (-0.60, 0.40, 3.14) -> SUCCEEDED

## Worst Sample

- x: -4.318
- y: -4.451
- yaw: -1.067
- clearance: 0.015 m
- collision: False
