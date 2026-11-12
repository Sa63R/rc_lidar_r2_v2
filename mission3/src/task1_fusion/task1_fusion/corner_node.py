"""Task 3: project acquisition-stamped odometry onto the requested corner."""
import math
import time
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster
from .task3_config import DEFAULTS, prepare
from .task3_quality import xy_filter


class CornerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('task3_corner', **kwargs)
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.v, self.projector = prepare({k: self.get_parameter(k).value for k in DEFAULTS})
        self.parent = self.v['output_frame'] or self.v['input_parent_frame']
        self.child = self.v['output_child_frame']
        self.smoother = xy_filter(self.v)
        prefix = self.v['output_prefix'].rstrip('/')
        self.pose_pub = self.create_publisher(PoseStamped, prefix+'/pose', 20)
        self.point_pub = self.create_publisher(PointStamped, prefix+'/point', 20)
        self.odom_pub = self.create_publisher(Odometry, prefix+'/odom', 20)
        self.raw_pub = (self.create_publisher(Odometry, prefix+'/raw_odom', 20)
                        if self.v['xy_smoothing_enabled'] else None)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/task3/diagnostics', 10)
        self.tf_pub = TransformBroadcaster(self) if self.v['publish_tf'] else None
        self.accepted = self.rejected = 0
        self.last_stamp_ns = None
        self.last_receipt = None
        self.last_clock_ns = None
        self.last_rejection = ''
        self.reason = 'waiting_for_input'
        self.create_subscription(Odometry, self.v['input_topic'], self.callback, qos_profile_sensor_data)
        self.create_timer(.1, self.diagnostics)
        if not self.v['calibration_confirmed']:
            self.get_logger().warning(
                'Corner output uses CAD/nominal extrinsics; physical accuracy is not verified. '
                'The vision interface uses its explicit availability policy; Z remains deferred.')
        self.get_logger().info(f'{self.v["input_topic"]} -> {prefix}/pose, /point, /odom; '
                               f'frames {self.parent} -> {self.child}; acquisition stamps preserved.')

    def clock_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now < self.last_clock_ns:
            self.last_stamp_ns = self.last_receipt = None
            self.smoother.reset()
            self.reason = 'clock_reset_waiting_for_input'
        self.last_clock_ns = now
        return now

    def reject(self, reason):
        self.rejected += 1
        self.last_rejection = self.reason = reason

    def callback(self, msg):
        now = self.clock_ns()
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if (msg.header.frame_id != self.v['input_parent_frame']
                or msg.child_frame_id != self.v['input_child_frame']):
            return self.reject('input_frame_mismatch')
        age = (now-stamp)*1e-9
        if stamp <= 0 or age > self.v['input_timeout_sec'] or age < -self.v['future_tolerance_sec']:
            return self.reject('input_stamp_invalid_or_stale')
        if self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            return self.reject('duplicate_or_out_of_order_input')
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        try:
            pose = self.projector.pose([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], msg.pose.covariance)
            velocity, twist_cov = self.projector.twist(
                [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z],
                msg.twist.covariance)
        except (ValueError, np.linalg.LinAlgError):
            return self.reject('invalid_pose_twist_or_covariance')
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id, out.child_frame_id = self.parent, self.child
        op, oq = out.pose.pose.position, out.pose.pose.orientation
        op.x, op.y, op.z = map(float, pose.xyz)
        oq.x, oq.y, oq.z, oq.w = map(float, pose.quaternion)
        out.pose.covariance = pose.covariance.ravel().tolist()
        ot = out.twist.twist
        ot.linear.x, ot.linear.y, ot.linear.z = map(float, velocity[:3])
        ot.angular.x, ot.angular.y, ot.angular.z = map(float, velocity[3:])
        out.twist.covariance = twist_cov.ravel().tolist()
        if self.raw_pub:
            self.raw_pub.publish(deepcopy(out))
        # Only accepted, fresh input updates the causal state; no timer outputs.
        pose = self.smoother.apply(stamp, pose)
        op.x, op.y, op.z = map(float, pose.xyz)
        out.pose.covariance = pose.covariance.ravel().tolist()
        stamped = PoseStamped(header=out.header, pose=out.pose.pose)
        point = PointStamped(header=out.header, point=op)
        self.pose_pub.publish(stamped)
        self.point_pub.publish(point)
        self.odom_pub.publish(out)
        if self.tf_pub:
            tf = TransformStamped(header=out.header, child_frame_id=self.child)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pose.xyz)
            tf.transform.rotation = oq
            self.tf_pub.sendTransform(tf)
        self.accepted += 1
        self.last_stamp_ns, self.last_receipt = stamp, time.monotonic()
        self.reason = 'tracking' if self.v['calibration_confirmed'] else 'tracking_unverified_extrinsics'

    def diagnostics(self):
        now = self.clock_ns()
        age = math.inf if self.last_stamp_ns is None else (now-self.last_stamp_ns)*1e-9
        receipt_age = math.inf if self.last_receipt is None else time.monotonic()-self.last_receipt
        stale = max(age, receipt_age) > self.v['input_timeout_sec']
        reason = ('input_timed_out' if self.accepted else 'waiting_for_input') if stale else self.reason
        status = DiagnosticStatus(name='task3_corner', hardware_id='corner_projection', message=reason)
        level = 1 if stale or not self.v['calibration_confirmed'] or self.reason != 'tracking' else 0
        status.level = bytes([level]) if isinstance(status.level, bytes) else level
        status.values = [KeyValue(key=k, value=str(v)) for k, v in {
            'accepted': self.accepted, 'rejected': self.rejected,
            'last_rejection': self.last_rejection, 'source_age_sec': age,
            'receipt_age_sec': receipt_age, 'calibration_confirmed': self.v['calibration_confirmed'],
            'output_parent': self.parent, 'output_child': self.child,
            'xy_smoothing_enabled': self.v['xy_smoothing_enabled'],
            'xy_smoothing_correction_m': self.smoother.correction_m,
        }.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CornerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
