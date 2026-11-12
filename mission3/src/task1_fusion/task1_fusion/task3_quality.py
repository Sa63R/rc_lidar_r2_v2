"""Causal XY jitter reduction and stationary-window origin estimation.

Adaptive cutoff follows Casiez et al., CHI 2012 (1 Euro filter):
https://gery.casiez.net/1euro/
This implementation caps the XY correction and never corrects drift or yaw.
"""
from collections import deque
import math
import numpy as np
from .corner import CornerPose


class BoundedXYFilter:
    def __init__(self, enabled=False, min_cutoff_hz=1., beta=40.,
                 derivative_cutoff_hz=1., max_correction_m=.003, max_gap_sec=.5):
        self.enabled = enabled
        self.min_cutoff = min_cutoff_hz
        self.beta = beta
        self.derivative_cutoff = derivative_cutoff_hz
        self.cap = max_correction_m
        self.max_gap = max_gap_sec
        for value in (min_cutoff_hz, derivative_cutoff_hz, max_correction_m, max_gap_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('filter cutoffs, correction cap and max gap must be positive')
        if not math.isfinite(beta) or beta < 0 or type(enabled) is not bool:
            raise ValueError('filter beta must be nonnegative and enabled must be boolean')
        self.reset()

    def reset(self):
        self.stamp = self.raw = self.filtered = None
        self.derivative = np.zeros(2)
        self.correction_m = 0.

    @staticmethod
    def alpha(dt, cutoff):
        return 1./(1.+1./(2.*math.pi*cutoff*dt))

    def apply(self, stamp_ns, pose):
        xy = pose.xyz[:2]
        if self.stamp is not None and stamp_ns <= self.stamp:
            raise ValueError('filter requires strictly increasing acquisition stamps')
        dt = None if self.stamp is None else (stamp_ns-self.stamp)*1e-9
        if not self.enabled or dt is None or dt > self.max_gap:
            result = xy.copy()
            self.derivative = np.zeros(2)
        else:
            speed = (xy-self.raw)/dt
            a = self.alpha(dt, self.derivative_cutoff)
            self.derivative += a*(speed-self.derivative)
            cutoff = self.min_cutoff+self.beta*np.linalg.norm(self.derivative)
            result = self.filtered+self.alpha(dt, cutoff)*(xy-self.filtered)
            delta = result-xy
            distance = np.linalg.norm(delta)
            if distance > self.cap:
                result = xy+delta*(self.cap/distance)
        delta = result-xy
        self.stamp, self.raw, self.filtered = stamp_ns, xy.copy(), result.copy()
        self.correction_m = float(np.linalg.norm(delta))
        xyz, covariance = pose.xyz.copy(), pose.covariance.copy()
        xyz[:2] = result
        # Do not shrink covariance to imply a physically validated improvement.
        covariance[:2, :2] += np.outer(delta, delta)
        return CornerPose(xyz, np.asarray(pose.quaternion, float).copy(), covariance)


def xy_filter(parameters):
    return BoundedXYFilter(parameters['xy_smoothing_enabled'],
                           parameters['xy_smoothing_min_cutoff_hz'],
                           parameters['xy_smoothing_beta'],
                           parameters['xy_smoothing_derivative_cutoff_hz'],
                           parameters['xy_smoothing_max_correction_m'],
                           parameters['input_timeout_sec'])


class StableStartWindow:
    """Require the operator to keep the car still; reject visibly moving windows.

    Low observed variation alone cannot prove zero motion. No automatic re-zero
    after initialization. Orientation is averaged on unit quaternions, not Euler
    angles. Gaps discard earlier samples.
    """
    def __init__(self, projector, duration_sec=2., min_samples=15,
                 max_xy_span_m=.015, max_rotation_span_rad=math.radians(.5),
                 max_gap_sec=.5):
        self.projector = projector
        self.duration = duration_sec
        self.min_samples = min_samples
        self.max_xy_span = max_xy_span_m
        self.max_angle = max_rotation_span_rad
        self.max_gap = max_gap_sec
        self.rows = deque()
        self.last_stamp = None

    def add(self, stamp_ns, xyz, quaternion):
        if self.last_stamp is not None:
            if stamp_ns <= self.last_stamp:
                return False
            if (stamp_ns-self.last_stamp)*1e-9 > self.max_gap:
                self.rows.clear()
        q = np.asarray(quaternion, float)
        xyz = np.asarray(xyz, float)
        pose = self.projector.pose(xyz, q, np.zeros((6, 6)))
        self.last_stamp = stamp_ns
        self.rows.append((stamp_ns, xyz.copy(), q/np.linalg.norm(q), pose.xyz[:2]))
        # Keep a window whose first/last timestamps span at least duration.
        while len(self.rows) > 1 and (stamp_ns-self.rows[1][0])*1e-9 >= self.duration:
            self.rows.popleft()
        return True

    def estimate(self):
        if len(self.rows) < self.min_samples:
            return None
        span = (self.rows[-1][0]-self.rows[0][0])*1e-9
        if span+1e-9 < self.duration:
            return None
        qs = np.array([r[2] for r in self.rows])
        _, vectors = np.linalg.eigh(qs.T @ qs)
        q = vectors[:, -1]
        if q[3] < 0:
            q = -q
        spread = np.ptp([r[3] for r in self.rows], axis=0)
        # Pairwise quaternion distance bounds full 3D angular span, including wrap.
        angle = float(np.max(2*np.arccos(np.clip(np.abs(qs @ qs.T), 0., 1.))))
        if np.max(spread) > self.max_xy_span or angle > self.max_angle:
            return None
        return {'source_xyz_m': np.median([r[1] for r in self.rows], axis=0).tolist(),
                'source_quaternion_xyzw': q.tolist(),
                'source_stamp_ns': self.rows[-1][0],
                'aggregation': {'method': 'stationary_window_xyz_median_quaternion_mean',
                                'sample_count': len(self.rows), 'duration_sec': span,
                                'first_stamp_ns': self.rows[0][0],
                                'last_stamp_ns': self.rows[-1][0],
                                'corner_xy_span_m': spread.tolist(),
                                'rotation_span_deg': math.degrees(angle),
                                'zero_motion_independently_verified': False}}
