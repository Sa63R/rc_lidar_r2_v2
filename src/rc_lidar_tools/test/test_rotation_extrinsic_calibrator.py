"""Tests for rotation extrinsic calibration math."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rc_lidar_tools.rotation_extrinsic_calibrator import (  # noqa: E402
    fit_rotation_error,
    summarize_yaw_values,
)


def test_fit_rotation_error_recovers_known_delta():
    """Known circular wobble should recover the injected XY error."""

    center = np.asarray([0.045, -0.025], dtype=float)
    delta = np.asarray([0.018, -0.011], dtype=float)
    yaws = np.linspace(-math.pi, math.pi, 240)

    observed = []
    for index, yaw in enumerate(yaws):
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        rotated_delta = np.asarray(
            [
                c * delta[0] - s * delta[1],
                s * delta[0] + c * delta[1],
            ],
            dtype=float,
        )
        deterministic_noise = 0.0001 * math.sin(index * 0.37)
        observed.append(center + rotated_delta + deterministic_noise)

    fit = fit_rotation_error(yaws, np.asarray(observed, dtype=float))

    assert fit['delta_x'] == pytest_approx(delta[0], abs_tol=0.0003)
    assert fit['delta_y'] == pytest_approx(delta[1], abs_tol=0.0003)
    assert fit['after_rms'] < fit['before_rms'] * 0.2


def test_summarize_yaw_values_keeps_multiple_turns():
    """Yaw diagnostics should preserve wrap jumps across multiple turns."""

    values = [
        math.radians(value)
        for value in [170, 178, -175, -120, -30, 45, 120, 179, -170]
    ]

    summary = summarize_yaw_values(values)

    assert summary['wrap_jump_count'] == 2
    assert summary['span_deg'] > 340.0
    assert summary['abs_travel_deg'] > 340.0


def pytest_approx(value, abs_tol):
    """Small wrapper to avoid importing pytest in generated docs."""

    import pytest

    return pytest.approx(value, abs=abs_tol)
