import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def se2_exp(vx, vy, w, dt):
    a = w * dt
    if abs(a) < 1e-7:
        A = dt * (1 - a * a / 6)
        B = dt * (a / 2 - a ** 3 / 24)
    else:
        A, B = math.sin(a) / w, (1 - math.cos(a)) / w
    return np.array([A * vx - B * vy, B * vx + A * vy, a])


def se2_log(delta, dt):
    dx, dy, a = delta
    if dt <= 0:
        raise ValueError("positive dt required")
    A = 1 - a * a / 6 if abs(a) < 1e-7 else math.sin(a) / a
    B = a / 2 - a ** 3 / 24 if abs(a) < 1e-7 else (1 - math.cos(a)) / a
    return np.array([(A * dx + B * dy) / (A*A + B*B) / dt,
                     (-B * dx + A * dy) / (A*A + B*B) / dt, a / dt])


def relative_se2(previous, current):
    dx, dy = np.asarray(current[:2]) - previous[:2]
    c, s = math.cos(previous[2]), math.sin(previous[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy,
                     wrap(current[2] - previous[2])])


def quat_matrix(q):
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(xyz, rotation):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rotation, xyz
    return t


def matrix_state(t):
    R = t[:3, :3]
    yaw = math.atan2(R[1, 0], R[0, 0])
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    roll = math.atan2(R[2, 1], R[2, 2])
    return np.array([*t[:3, 3], yaw]), (roll, pitch)


def rpy_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
