#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar_localization.pose_stability_filter import PoseFilterState


class PoseFilterStateTest(unittest.TestCase):
    def test_first_sample_is_accepted_without_delay(self):
        state = PoseFilterState()

        result = state.update((1.0, 2.0, 0.0, 0.0, 0.0, 0.5), stamp_sec=1.0)

        self.assertTrue(result.accepted)
        self.assertEqual(result.reason, "initial")
        self.assertAlmostEqual(result.pose[0], 1.0)
        self.assertAlmostEqual(result.pose[1], 2.0)
        self.assertAlmostEqual(result.pose[5], 0.5)

    def test_stationary_small_motion_is_smoothed(self):
        state = PoseFilterState(
            static_alpha=0.2,
            moving_alpha=0.7,
            static_speed_threshold=0.25,
            full_speed_threshold=1.0,
        )
        state.update((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.0)

        result = state.update((0.01, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.1)

        self.assertTrue(result.accepted)
        self.assertLess(result.motion_factor, 0.5)
        self.assertGreater(result.pose[0], 0.001)
        self.assertLess(result.pose[0], 0.01)

    def test_normal_large_stationary_spike_is_rejected_once(self):
        state = PoseFilterState(
            static_alpha=0.2,
            moving_alpha=0.7,
            static_speed_threshold=0.25,
            full_speed_threshold=1.0,
            spike_step_m=0.05,
            filter_mode="normal",
        )
        state.update((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.0)
        state.update((0.002, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.1)

        result = state.update((0.20, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.2)

        self.assertFalse(result.accepted)
        self.assertTrue(result.spike_candidate)
        self.assertEqual(result.reason, "stationary_spike")
        self.assertLess(result.pose[0], 0.01)

    def test_beta_single_small_spike_is_held_and_rejected(self):
        state = PoseFilterState(
            static_alpha=0.2,
            moving_alpha=0.7,
            static_speed_threshold=0.25,
            full_speed_threshold=1.0,
            spike_step_m=0.012,
            filter_mode="beta",
            beta_confirm_samples=3,
        )
        state.update((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.0)

        held = state.update((0.020, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.1)
        rejected = state.update((0.001, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.2)

        self.assertFalse(held.accepted)
        self.assertEqual(held.reason, "beta_hold")
        self.assertEqual(held.hold_state, "holding")
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "beta_spike_rejected")
        self.assertAlmostEqual(rejected.pose[0], 0.0, places=6)

    def test_beta_continuous_motion_is_released_after_confirmation(self):
        state = PoseFilterState(
            static_alpha=0.2,
            moving_alpha=0.7,
            static_speed_threshold=0.25,
            full_speed_threshold=1.0,
            spike_step_m=0.012,
            filter_mode="beta",
            beta_confirm_samples=3,
        )
        state.update((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.0)
        first = state.update((0.020, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.1)
        second = state.update((0.040, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.2)
        released = state.update((0.060, 0.0, 0.0, 0.0, 0.0, 0.0), stamp_sec=1.3)

        self.assertFalse(first.accepted)
        self.assertFalse(second.accepted)
        self.assertTrue(released.accepted)
        self.assertEqual(released.reason, "beta_confirmed_motion")
        self.assertEqual(released.hold_state, "released")
        self.assertEqual(released.confirm_count, 3)
        self.assertGreater(released.pose[0], 0.020)


if __name__ == "__main__":
    unittest.main()
