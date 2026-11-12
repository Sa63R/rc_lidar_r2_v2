"""Four-state EKF with wheel increments and fixed-lag delayed updates.

State: x, y, z, yaw. Wheel input is constant body twist over each measured
interval; interpolation splits a real measured increment, never invents motion.
Accepted LiDAR observations are replayed along with wheel controls. This matters
when a late observation arrives before another already accepted observation.
"""
from dataclasses import dataclass
import math
import numpy as np
from .math_utils import se2_exp, wrap


@dataclass
class Config:
    history_sec: float = 2.0
    max_wheel_gap_sec: float = 0.10
    wheel_noise_per_sqrt_m: float = 0.01
    yaw_noise_per_sqrt_m: float = 0.02
    moving_xy_noise: float = 0.001
    moving_z_noise: float = 0.002
    stationary_xy_noise: float = 0.00015
    stationary_z_noise: float = 0.00010
    stationary_yaw_noise: float = 0.0002
    lidar_std: tuple = (0.010, 0.010, 0.012, 0.015)
    stationary_lidar_std_scale: float = 1.5
    innovation_gate: float = 18.47  # chi-square 4 dof, approximately 99.9%
    max_position_innovation_m: float = 0.50
    max_yaw_innovation_rad: float = 0.50


@dataclass
class Motion:
    start: float
    end: float
    velocity: np.ndarray
    stationary: bool = False


@dataclass
class Observation:
    stamp: float
    value: np.ndarray
    covariance: np.ndarray


class FixedLagEKF:
    def __init__(self, config=None):
        self.cfg = config or Config()
        if self.cfg.history_sec <= self.cfg.max_wheel_gap_sec:
            raise ValueError("history must exceed maximum wheel gap")
        if any(s <= 0 for s in self.cfg.lidar_std):
            raise ValueError("LiDAR standard deviations must be positive")
        self.motions, self.observations, self.snapshots = [], [], []
        self.anchor = self.state = self.P = None
        self.anchor_t = self.latest_t = self.last_lidar_t = None
        self.accepted = self.rejected = 0
        self.last_reason = "waiting_for_lidar_and_wheels"
        self.last_nis = math.nan
        self.last_innovation = np.full(4, np.nan)

    @property
    def initialized(self):
        return self.anchor is not None

    def predict(self, x, P, motion, dt):
        d = se2_exp(*motion.velocity, dt)
        c, s = math.cos(x[3]), math.sin(x[3])
        gx, gy = c*d[0]-s*d[1], s*d[0]+c*d[1]
        out = x.copy()
        out[0] += gx
        out[1] += gy
        out[3] = wrap(out[3] + d[2])
        F = np.eye(4)
        F[0, 3], F[1, 3] = -gy, gx
        distance = math.hypot(motion.velocity[0], motion.velocity[1]) * dt
        cfg = self.cfg
        xy = cfg.stationary_xy_noise if motion.stationary else cfg.moving_xy_noise
        z = cfg.stationary_z_noise if motion.stationary else cfg.moving_z_noise
        yaw = cfg.stationary_yaw_noise if motion.stationary else cfg.moving_xy_noise
        Q = np.diag([xy*xy*dt + cfg.wheel_noise_per_sqrt_m**2*distance,
                     xy*xy*dt + cfg.wheel_noise_per_sqrt_m**2*distance,
                     z*z*dt,
                     yaw*yaw*dt + cfg.yaw_noise_per_sqrt_m**2*(distance+abs(d[2]))])
        return out, F @ P @ F.T + Q

    @staticmethod
    def correct(x, P, obs):
        residual = obs.value - x
        residual[3] = wrap(residual[3])
        S = P + obs.covariance
        K = np.linalg.solve(S.T, P.T).T
        out = x + K @ residual
        out[3] = wrap(out[3])
        A = np.eye(4) - K
        C = A @ P @ A.T + K @ obs.covariance @ K.T  # Joseph form
        return out, (C + C.T) * 0.5

    def add_motion(self, motion):
        dt = motion.end - motion.start
        if (not math.isfinite(motion.start) or not math.isfinite(motion.end)
                or not np.isfinite(motion.velocity).all() or dt <= 1e-8):
            raise ValueError("invalid wheel interval")
        if dt > self.cfg.max_wheel_gap_sec + 1e-8:
            raise ValueError("wheel_gap_requires_reinitialization")
        if self.latest_t is not None and abs(motion.start-self.latest_t) > 1e-6:
            raise ValueError("wheel timestamps must be contiguous and increasing")
        self.motions.append(motion)
        self.latest_t = motion.end
        if self.initialized:
            self.state, self.P = self.predict(self.state, self.P, motion, dt)
            self.snapshots.append((motion.end, self.state.copy(), self.P.copy()))
        self._prune()

    def _replay(self, stop=None, save=False):
        stop = self.latest_t if stop is None else stop
        x, P = self.anchor[0].copy(), self.anchor[1].copy()
        obs = [o for o in self.observations if self.anchor_t < o.stamp <= stop]
        i = 0
        snapshots = []
        for motion in self.motions:
            t, end = max(motion.start, self.anchor_t), min(motion.end, stop)
            if end <= t:
                continue
            while i < len(obs) and obs[i].stamp <= end + 1e-9:
                at = min(end, max(t, obs[i].stamp))
                x, P = self.predict(x, P, motion, at-t)
                x, P = self.correct(x, P, obs[i])
                t, i = at, i+1
            x, P = self.predict(x, P, motion, end-t)
            if abs(end-motion.end) < 1e-9:
                snapshots.append((end, x.copy(), P.copy()))
            if end >= stop:
                break
        if save:
            self.snapshots = snapshots
        return x, P

    def add_lidar(self, stamp, value):
        value = np.asarray(value, dtype=float)
        if value.shape != (4,) or not np.isfinite(value).all() or not math.isfinite(stamp):
            return self._reject("invalid_lidar")
        if not self.motions:
            return self._reject("no_wheel_history")
        oldest = self.anchor_t if self.initialized else self.motions[0].start
        if stamp < oldest - 1e-8:
            return self._reject("lidar_older_than_history")
        if stamp > self.latest_t + 1e-8:
            return self._reject("lidar_ahead_of_wheels")
        if self.initialized and (stamp <= self.anchor_t + 1e-8 or
                any(abs(o.stamp-stamp) < 1e-8 for o in self.observations)):
            return self._reject("duplicate_or_folded_lidar")
        stationary = next((m.stationary for m in self.motions
                           if m.start <= stamp <= m.end), False)
        scale = self.cfg.stationary_lidar_std_scale if stationary else 1.0
        R = np.diag((np.asarray(self.cfg.lidar_std) * scale)**2)
        observation = Observation(stamp, value.copy(), R)
        if not self.initialized:
            self.anchor_t = stamp
            self.anchor = (value.copy(), R.copy())
            self.observations = []
            self.state, self.P = self._replay(save=True)
        else:
            x, P = self._replay(stop=stamp)
            innovation = value - x
            innovation[3] = wrap(innovation[3])
            nis = float(innovation @ np.linalg.solve(P + R, innovation))
            self.last_nis, self.last_innovation = nis, innovation
            if (np.linalg.norm(innovation[:3]) > self.cfg.max_position_innovation_m
                    or abs(innovation[3]) > self.cfg.max_yaw_innovation_rad
                    or nis > self.cfg.innovation_gate):
                return self._reject("innovation_rejected")
            self.observations.append(observation)
            self.observations.sort(key=lambda o: o.stamp)
            self.state, self.P = self._replay(save=True)
        self.last_lidar_t = stamp if self.last_lidar_t is None else max(stamp, self.last_lidar_t)
        self.accepted += 1
        self.last_reason = "accepted"
        return True

    def _reject(self, reason):
        self.rejected += 1
        self.last_reason = reason
        return False

    def _prune(self):
        cutoff = self.latest_t - self.cfg.history_sec
        if self.initialized:
            candidates = [s for s in self.snapshots if s[0] <= cutoff]
            if candidates:
                t, x, P = candidates[-1]
                self.anchor_t, self.anchor = t, (x.copy(), P.copy())
                self.motions = [m for m in self.motions if m.end > t + 1e-8]
                self.observations = [o for o in self.observations if o.stamp > t + 1e-8]
                self.snapshots = [s for s in self.snapshots if s[0] > t + 1e-8]
        else:
            self.motions = [m for m in self.motions if m.end > cutoff]
