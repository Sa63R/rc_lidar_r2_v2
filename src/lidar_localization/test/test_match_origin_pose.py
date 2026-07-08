#!/usr/bin/env python3
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import Pose

from lidar_localization.match_origin_pose import (
    MatchOriginState,
    PoseSample,
    pose_relative_to_planar_origin,
    pose_yaw,
)
from lidar_localization.pose_math import quat_from_rpy


def pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> Pose:
    msg = Pose()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.orientation = quat_from_rpy(0.0, 0.0, yaw)
    return msg


def sample(t: float, x: float, y: float, yaw: float = 0.0) -> PoseSample:
    p = pose(x, y, 0.0, yaw)
    return PoseSample(wall_sec=t, stamp_sec=t, pose=p, yaw=yaw)


class MatchOriginPoseTest(unittest.TestCase):
    def test_stable_window_locks_origin(self):
        state = MatchOriginState(
            min_lock_sec=0.4,
            min_samples=5,
            max_xy_std=0.02,
            max_yaw_std_rad=math.radians(2.0),
        )

        decision = None
        for i in range(5):
            decision = state.add_sample(sample(i * 0.1, 1.0 + i * 0.001, -2.0, 0.01))

        self.assertTrue(decision.locked)
        self.assertIsNotNone(state.origin)
        self.assertAlmostEqual(state.origin.position.x, 1.002, places=6)
        self.assertAlmostEqual(state.origin.position.y, -2.0, places=6)
        self.assertAlmostEqual(pose_yaw(state.origin), 0.01, places=6)

    def test_unstable_window_does_not_lock(self):
        state = MatchOriginState(
            min_lock_sec=0.4,
            min_samples=5,
            max_xy_std=0.02,
            max_yaw_std_rad=math.radians(2.0),
        )

        decision = None
        for i in range(5):
            decision = state.add_sample(sample(i * 0.1, float(i) * 0.1, 0.0, 0.0))

        self.assertFalse(decision.locked)
        self.assertIsNone(state.origin)

    def test_planar_relative_pose_rotates_into_origin_frame(self):
        origin = pose(10.0, 5.0, 0.2, math.pi / 2.0)
        current = pose(10.0, 6.0, -1.0, math.pi)

        rel = pose_relative_to_planar_origin(origin, current)

        self.assertAlmostEqual(rel.position.x, 1.0, places=6)
        self.assertAlmostEqual(rel.position.y, 0.0, places=6)
        self.assertAlmostEqual(rel.position.z, 0.0, places=6)
        self.assertAlmostEqual(pose_yaw(rel), math.pi / 2.0, places=6)

    def test_reset_clears_locked_origin(self):
        state = MatchOriginState(min_lock_sec=0.0, min_samples=1)
        decision = state.add_sample(sample(0.0, 1.0, 2.0, 0.3))
        self.assertTrue(decision.locked)

        state.reset()

        self.assertFalse(state.locked)
        self.assertIsNone(state.origin)
        self.assertEqual(len(state.samples), 0)


if __name__ == "__main__":
    unittest.main()
