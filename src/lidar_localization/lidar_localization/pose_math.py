import math
from typing import Iterable

from geometry_msgs.msg import Point, Pose, Quaternion


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def quat_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 0.0:
        return Quaternion(w=1.0)
    return Quaternion(x=q.x / norm, y=q.y / norm, z=q.z / norm, w=q.w / norm)


def quat_conjugate(q: Quaternion) -> Quaternion:
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def quat_inverse(q: Quaternion) -> Quaternion:
    norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if norm <= 0.0:
        return Quaternion(w=1.0)
    conj = quat_conjugate(q)
    return Quaternion(
        x=conj.x / norm,
        y=conj.y / norm,
        z=conj.z / norm,
        w=conj.w / norm,
    )


def rotate_point(q: Quaternion, p: Point) -> Point:
    pq = Quaternion(x=p.x, y=p.y, z=p.z, w=0.0)
    rotated = quat_multiply(quat_multiply(q, pq), quat_conjugate(q))
    return Point(x=rotated.x, y=rotated.y, z=rotated.z)


def point_add(a: Point, b: Point) -> Point:
    return Point(x=a.x + b.x, y=a.y + b.y, z=a.z + b.z)


def point_sub(a: Point, b: Point) -> Point:
    return Point(x=a.x - b.x, y=a.y - b.y, z=a.z - b.z)


def point_lerp(a: Point, b: Point, alpha: float) -> Point:
    alpha = max(0.0, min(1.0, float(alpha)))
    return Point(
        x=a.x + (b.x - a.x) * alpha,
        y=a.y + (b.y - a.y) * alpha,
        z=a.z + (b.z - a.z) * alpha,
    )


def point_from_iter(values: Iterable[float]) -> Point:
    x, y, z = list(values)
    return Point(x=float(x), y=float(y), z=float(z))


def copy_pose(pose: Pose) -> Pose:
    copied = Pose()
    copied.position.x = pose.position.x
    copied.position.y = pose.position.y
    copied.position.z = pose.position.z
    copied.orientation.x = pose.orientation.x
    copied.orientation.y = pose.orientation.y
    copied.orientation.z = pose.orientation.z
    copied.orientation.w = pose.orientation.w
    return copied


def compose_pose(a: Pose, b: Pose) -> Pose:
    """Return a * b, treating Pose as a rigid transform."""
    out = Pose()
    out.position = point_add(a.position, rotate_point(a.orientation, b.position))
    out.orientation = quat_normalize(quat_multiply(a.orientation, b.orientation))
    return out


def inverse_pose(pose: Pose) -> Pose:
    inv = Pose()
    inv.orientation = quat_inverse(pose.orientation)
    neg = Point(x=-pose.position.x, y=-pose.position.y, z=-pose.position.z)
    inv.position = rotate_point(inv.orientation, neg)
    return inv


def quat_slerp(a: Quaternion, b: Quaternion, alpha: float) -> Quaternion:
    alpha = max(0.0, min(1.0, float(alpha)))
    qa = quat_normalize(a)
    qb = quat_normalize(b)
    dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w

    if dot < 0.0:
        qb = Quaternion(x=-qb.x, y=-qb.y, z=-qb.z, w=-qb.w)
        dot = -dot

    if dot > 0.9995:
        return quat_normalize(
            Quaternion(
                x=qa.x + alpha * (qb.x - qa.x),
                y=qa.y + alpha * (qb.y - qa.y),
                z=qa.z + alpha * (qb.z - qa.z),
                w=qa.w + alpha * (qb.w - qa.w),
            )
        )

    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return quat_normalize(
        Quaternion(
            x=s0 * qa.x + s1 * qb.x,
            y=s0 * qa.y + s1 * qb.y,
            z=s0 * qa.z + s1 * qb.z,
            w=s0 * qa.w + s1 * qb.w,
        )
    )


def interpolate_pose(a: Pose, b: Pose, alpha: float) -> Pose:
    out = Pose()
    out.position = point_lerp(a.position, b.position, alpha)
    out.orientation = quat_slerp(a.orientation, b.orientation, alpha)
    return out


def rpy_from_quat(q: Quaternion) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
