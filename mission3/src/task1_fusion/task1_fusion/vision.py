"""Freshness and validity contract for the vision team's corner coordinates."""
import math
import numpy as np
from .corner import vector3
from .math_utils import quat_matrix


class VisionFeed:
    def __init__(self, timeout_sec=.5, future_tolerance_sec=.05, *,
                 allow_unverified_extrinsics=False, reference_id='', data_source='live'):
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('timeout_sec must be finite and positive')
        if not math.isfinite(future_tolerance_sec) or future_tolerance_sec < 0:
            raise ValueError('future_tolerance_sec must be finite and nonnegative')
        self.timeout = timeout_sec
        self.future_tolerance = future_tolerance_sec
        if type(allow_unverified_extrinsics) is not bool:
            raise ValueError('allow_unverified_extrinsics must be boolean')
        self.allow_unverified = allow_unverified_extrinsics
        self.reference_id = reference_id
        self.data_source = data_source
        self.rejected = 0
        self.last_rejection = ''
        self.reset()

    def reset(self):
        self.sample = None
        self.upstream_receipt = None
        self.calibrated = self.upstream_tracking = False

    def upstream(self, calibrated, tracking, monotonic_now):
        self.calibrated = bool(calibrated)
        self.upstream_tracking = bool(tracking)
        self.upstream_receipt = monotonic_now

    def accept(self, stamp_ns, xyz, quaternion, frame, ros_now_ns, monotonic_now):
        age = (ros_now_ns-stamp_ns)*1e-9
        reason = ''
        if not frame:
            reason = 'input_frame_mismatch'
        elif stamp_ns <= 0 or age > self.timeout or age < -self.future_tolerance:
            reason = 'input_stamp_invalid_or_stale'
        elif self.sample is not None and stamp_ns <= self.sample['stamp_ns']:
            reason = 'duplicate_or_out_of_order_input'
        try:
            p = vector3(xyz, 'position')
            q = np.asarray(quaternion, dtype=float)
            if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
                raise ValueError('invalid quaternion')
            R = quat_matrix(q)
        except ValueError:
            reason = 'invalid_pose'
        if reason:
            self.rejected += 1
            self.last_rejection = reason
            return False
        self.sample = {'stamp_ns': int(stamp_ns), 'frame_id': frame,
                       'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]),
                       'yaw': math.atan2(R[1, 0], R[0, 0]), 'receipt': monotonic_now}
        return True

    def snapshot(self, ros_now_ns, monotonic_now):
        sample = self.sample
        if sample is None:
            reason = 'waiting_for_input'
        elif ((ros_now_ns-sample['stamp_ns'])*1e-9 > self.timeout
              or monotonic_now-sample['receipt'] > self.timeout
              or (sample['stamp_ns']-ros_now_ns)*1e-9 > self.future_tolerance):
            reason = 'input_timed_out'
        elif self.upstream_receipt is None or monotonic_now-self.upstream_receipt > self.timeout:
            reason = 'upstream_status_timed_out'
        elif not self.calibrated and not self.allow_unverified:
            reason = 'extrinsics_unconfirmed'
        elif not self.upstream_tracking:
            reason = 'upstream_not_tracking'
        else:
            reason = 'tracking' if self.calibrated else 'tracking_unverified_extrinsics'
        valid = reason in ('tracking', 'tracking_unverified_extrinsics')
        age = None if sample is None else (ros_now_ns-sample['stamp_ns'])*1e-9
        # Invalid snapshots deliberately carry no usable coordinates.
        data = None if not valid else {k: sample[k] for k in
                                      ('stamp_ns', 'frame_id', 'x', 'y', 'z', 'yaw')}
        return {'schema_version': 1, 'valid': valid, 'reason': reason,
                'reference_point': 'corner', 'position_unit': 'm', 'yaw_unit': 'rad',
                'yaw_convention': 'right-handed about +Z; atan2(R_yx, R_xx)',
                'calibration_confirmed': self.calibrated, 'source_age_sec': age,
                'reference_id': self.reference_id, 'data_source': self.data_source,
                'output_components': ['x', 'y', 'yaw'], 'z_valid': False,
                'physical_accuracy_verified': False,
                'rejected': self.rejected, 'last_rejection': self.last_rejection,
                'data': data}
