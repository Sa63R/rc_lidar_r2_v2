#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException


def normalize_quat(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_conjugate(q):
    q = normalize_quat(q)
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ])


def rotate_vector(q, v):
    q = normalize_quat(q)
    vq = np.array([v[0], v[1], v[2], 0.0])
    rq = quat_multiply(quat_multiply(q, vq), quat_conjugate(q))
    return rq[:3]


def quat_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    ])

    return normalize_quat(q)


def euler_from_quat(q):
    x, y, z, w = normalize_quat(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def average_quaternions(quats):
    """
    Quaternion average using eigenvector method.
    quats format: [x, y, z, w]
    """
    ref = normalize_quat(quats[0])
    A = np.zeros((4, 4), dtype=float)

    for q in quats:
        q = normalize_quat(q)
        if np.dot(q, ref) < 0.0:
            q = -q
        A += np.outer(q, q)

    A /= len(quats)

    eigvals, eigvecs = np.linalg.eigh(A)
    q_avg = eigvecs[:, np.argmax(eigvals)]

    if q_avg[3] < 0.0:
        q_avg = -q_avg

    return normalize_quat(q_avg)


class FastlioWorldInitializer(Node):
    def __init__(self):
        super().__init__('fastlio_world_initializer')

        self.source_parent_frame = self.declare_parameter(
            'source_parent_frame', 'camera_init'
        ).value

        self.source_child_frame = self.declare_parameter(
            'source_child_frame', 'body'
        ).value

        self.world_frame = self.declare_parameter(
            'world_frame', 'world'
        ).value

        self.capture_duration = float(self.declare_parameter(
            'capture_duration', 3.0
        ).value)

        self.settle_time = float(self.declare_parameter(
            'settle_time', 0.0
        ).value)

        self.sample_period = float(self.declare_parameter(
            'sample_period', 0.02
        ).value)

        self.min_samples = int(self.declare_parameter(
            'min_samples', 20
        ).value)

        self.orientation_mode = self.declare_parameter(
            'orientation_mode', 'full'
        ).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.first_seen_time = None
        self.capture_start_time = None
        self.trans_samples = []
        self.quat_samples = []

        self.finished = False
        self.final_tf = None

        self.timer = self.create_timer(self.sample_period, self.timer_callback)

        self.get_logger().info(
            f'Waiting for TF {self.source_parent_frame} -> {self.source_child_frame}'
        )

    def timer_callback(self):
        if self.finished:
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.source_parent_frame,
                self.source_child_frame,
                Time()
            )
        except TransformException:
            return

        now = self.get_clock().now()

        if self.first_seen_time is None:
            self.first_seen_time = now
            self.get_logger().info(
                f'Detected TF {self.source_parent_frame} -> {self.source_child_frame}'
            )

        since_first_seen = (now - self.first_seen_time).nanoseconds * 1e-9

        if since_first_seen < self.settle_time:
            return

        if self.capture_start_time is None:
            self.capture_start_time = now
            self.get_logger().info(
                f'Start capturing for {self.capture_duration:.2f} seconds...'
            )

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        self.trans_samples.append(np.array([t.x, t.y, t.z], dtype=float))
        self.quat_samples.append(np.array([q.x, q.y, q.z, q.w], dtype=float))

        elapsed = (now - self.capture_start_time).nanoseconds * 1e-9

        if elapsed >= self.capture_duration and len(self.trans_samples) >= self.min_samples:
            self.finish_initialization()

    def finish_initialization(self):
        trans_avg = np.mean(np.array(self.trans_samples), axis=0)

        if self.orientation_mode == 'full':
            quat_avg = average_quaternions(self.quat_samples)

        elif self.orientation_mode == 'yaw_only':
            yaws = []
            for q in self.quat_samples:
                _, _, yaw = euler_from_quat(q)
                yaws.append(yaw)

            yaw_avg = math.atan2(
                np.mean(np.sin(yaws)),
                np.mean(np.cos(yaws))
            )

            quat_avg = quat_from_euler(0.0, 0.0, yaw_avg)

        elif self.orientation_mode == 'none':
            quat_avg = np.array([0.0, 0.0, 0.0, 1.0])

        else:
            self.get_logger().warn(
                f'Unknown orientation_mode={self.orientation_mode}, using full'
            )
            quat_avg = average_quaternions(self.quat_samples)

        # Put the new world frame at the averaged body pose in camera_init.
        # Publish:
        #   camera_init -> world = average(camera_init -> body)
        #
        # Then:
        #   world -> body = inverse(camera_init -> world) * camera_init -> body
        # so body will be close to zero at initialization.

        quat_out = quat_avg
        trans_out = trans_avg

        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()

        # 注意：这里反过来！
        static_tf.header.frame_id = self.source_parent_frame   # camera_init
        static_tf.child_frame_id = self.world_frame            # world

        static_tf.transform.translation.x = float(trans_out[0])
        static_tf.transform.translation.y = float(trans_out[1])
        static_tf.transform.translation.z = float(trans_out[2])

        static_tf.transform.rotation.x = float(quat_out[0])
        static_tf.transform.rotation.y = float(quat_out[1])
        static_tf.transform.rotation.z = float(quat_out[2])
        static_tf.transform.rotation.w = float(quat_out[3])

        self.static_broadcaster.sendTransform(static_tf)

        self.final_tf = static_tf
        self.finished = True

        roll, pitch, yaw = euler_from_quat(quat_avg)

        self.get_logger().info('Initialization finished.')
        self.get_logger().info(
            f'Average {self.source_parent_frame}->{self.source_child_frame}: '
            f'x={trans_avg[0]:.4f}, y={trans_avg[1]:.4f}, z={trans_avg[2]:.4f}, '
            f'roll={math.degrees(roll):.2f} deg, '
            f'pitch={math.degrees(pitch):.2f} deg, '
            f'yaw={math.degrees(yaw):.2f} deg'
        )

        self.get_logger().info(
            f'Published static TF {self.source_parent_frame} -> {self.world_frame}'
        )

        # Re-publish occasionally for robustness.
        self.republish_timer = self.create_timer(5.0, self.republish_static_tf)

    def republish_static_tf(self):
        if self.final_tf is None:
            return

        self.final_tf.header.stamp = self.get_clock().now().to_msg()
        self.static_broadcaster.sendTransform(self.final_tf)


def main(args=None):
    rclpy.init(args=args)
    node = FastlioWorldInitializer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
