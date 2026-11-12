"""Rigid-body corner projection; independent of ROS.

T_A_B is the pose of B in A. Pose covariance uses ROS fixed-axis small
rotations in the parent frame; twist/covariance are in the child frame.
"""
from dataclasses import dataclass
import numpy as np
from .math_utils import quat_matrix, rpy_matrix, transform


def vector3(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain three finite numbers')
    return result


def xyz_rpy(xyz, rpy):
    return transform(vector3(xyz, 'xyz'), rpy_matrix(vector3(rpy, 'rpy')))


def rigid_matrix(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError('transform must be a finite 4x4 matrix')
    R = value[:3, :3]
    if (not np.allclose(value[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-10)
            or not np.isclose(np.linalg.det(R), 1.0, atol=1e-10)):
        raise ValueError('transform must be a proper rigid transform')
    return value.copy()


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation_quaternion(R):
    """Quaternion xyzw, including rotations at Euler-angle singularities."""
    trace = np.trace(R)
    if trace > 0:
        s = 2 * np.sqrt(1 + trace)
        q = np.array([(R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s,
                      (R[1, 0]-R[0, 1])/s, s/4])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2 * np.sqrt(max(0., 1 + R[i, i]-R[j, j]-R[k, k]))
        q = np.zeros(4)
        q[i], q[j], q[k] = s/4, (R[j, i]+R[i, j])/s, (R[k, i]+R[i, k])/s
        q[3] = (R[k, j]-R[j, k])/s
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def covariance6(value):
    result = np.asarray(value, dtype=float)
    if result.size != 36 or not np.isfinite(result).all():
        raise ValueError('covariance must contain 36 finite numbers')
    result = result.reshape(6, 6)
    if not np.allclose(result, result.T, atol=1e-8):
        raise ValueError('covariance must be symmetric')
    if np.linalg.eigvalsh(result).min() < -1e-8:
        raise ValueError('covariance must be positive semidefinite')
    return (result + result.T) / 2


def corner_extrinsic(source_to_body_xyz, source_to_body_rpy,
                     body_to_lidar_xyz, body_to_lidar_rpy,
                     lidar_to_diagram_rpy, lidar_to_corner_diagram_xyz):
    """Compose source->body->LiDAR->diagram-aligned corner, all lengths in m."""
    return (xyz_rpy(source_to_body_xyz, source_to_body_rpy)
            @ xyz_rpy(body_to_lidar_xyz, body_to_lidar_rpy)
            @ xyz_rpy([0., 0., 0.], lidar_to_diagram_rpy)
            @ xyz_rpy(lidar_to_corner_diagram_xyz, [0., 0., 0.]))


@dataclass
class CornerPose:
    xyz: np.ndarray
    quaternion: np.ndarray
    covariance: np.ndarray


class CornerProjector:
    def __init__(self, source_to_corner, output_from_input=None):
        self.source_T_corner = rigid_matrix(source_to_corner)
        self.output_T_input = rigid_matrix(
            np.eye(4) if output_from_input is None else output_from_input)

    def pose(self, xyz, quaternion, covariance):
        position = vector3(xyz, 'position')
        q = np.asarray(quaternion, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
            raise ValueError('orientation must be a finite nonzero xyzw quaternion')
        R = quat_matrix(q)
        result = self.output_T_input @ transform(position, R) @ self.source_T_corner
        # A small parent-frame rotation moves the translated origin too.
        J = np.eye(6)
        J[:3, 3:] = -skew(R @ self.source_T_corner[:3, 3])
        A = np.zeros((6, 6))
        A[:3, :3] = A[3:, 3:] = self.output_T_input[:3, :3]
        J = A @ J
        C = J @ covariance6(covariance) @ J.T
        return CornerPose(result[:3, 3], rotation_quaternion(result[:3, :3]), (C+C.T)/2)

    def twist(self, velocity, covariance):
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape != (6,) or not np.isfinite(velocity).all():
            raise ValueError('twist must contain six finite numbers')
        # v_corner = v_source + omega x lever, then express in corner axes.
        Rt = self.source_T_corner[:3, :3].T
        J = np.zeros((6, 6))
        J[:3, :3] = J[3:, 3:] = Rt
        J[:3, 3:] = -Rt @ skew(self.source_T_corner[:3, 3])
        C = J @ covariance6(covariance) @ J.T
        return J @ velocity, (C+C.T)/2
