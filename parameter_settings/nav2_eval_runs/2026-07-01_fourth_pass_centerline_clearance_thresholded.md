# Fake Nav Clearance Evaluation

- Map: `src/rc_nav2_bringup/maps/red_manual_simple.yaml`
- Odom topic: `/odom`
- Action: `/navigate_to_pose`
- Samples: 2380
- Ignored initial seconds: 2.00
- Evaluated samples: 2279
- Raw collision samples: 0
- Raw min clearance: 0.015 m
- Collision samples: 0
- Min clearance: 0.015 m
- P05 clearance: 0.015 m
- P50 clearance: 0.165 m

## Goals

- 1: (-8.70, 0.15, 3.14) -> SUCCEEDED
- 2: (-8.70, -4.45, -1.57) -> SUCCEEDED
- 3: (-1.60, -4.45, 0.00) -> SUCCEEDED
- 4: (-1.60, 0.15, 1.57) -> SUCCEEDED
- 5: (-0.60, 0.40, 3.14) -> SUCCEEDED

## Worst Sample

- x: -2.898
- y: 0.272
- yaw: -3.065
- clearance: 0.015 m
- collision: False
