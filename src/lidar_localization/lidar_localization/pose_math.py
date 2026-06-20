import math
from typing import Iterable

from geometry_msgs.msg import Point, Quaternion


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


def point_from_iter(values: Iterable[float]) -> Point:
    x, y, z = list(values)
    return Point(x=float(x), y=float(y), z=float(z))


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
