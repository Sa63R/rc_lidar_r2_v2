import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from .core import Config, FixedLagEKF, Motion
from .math_utils import (matrix_state, quat_matrix, relative_se2, rpy_matrix,
                         rpy_quat, se2_exp, se2_log, transform, wrap)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_matrix(pose):
    return transform([pose.position.x, pose.position.y, pose.position.z],
                     quat_matrix([pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w]))


class FusionNode(Node):
    def __init__(self):
        super().__init__('task1_fusion')
        defaults = {
            'wheel_topic': '/wheel/odom', 'lidar_topic': '/map_lidar/odom',
            'lidar_parent_frame': 'map', 'lidar_child_frame': 'body',
            'wheel_parent_frame': 'wheel_odom', 'wheel_child_frame': 'robot_base',
            'output_frame': 'robot_origin', 'output_child_frame': 'robot_base_fused',
            'body_to_base_xyz': [0.237488, -0.272804, -0.483],
            'body_to_base_rpy': [0.0, 0.0, -math.pi/2],
            'map_to_origin_xyz': [0.237488, -0.272804, -0.483],
            'map_to_origin_rpy': [0.0, 0.0, -math.pi/2],
            'wheel_timeout_sec': 0.10, 'lidar_timeout_sec': 1.0,
            'stationary_speed_mps': 0.003, 'stationary_yaw_rate': 0.01,
            'stationary_dwell_sec': 0.30, 'max_speed_mps': 5.0,
            'max_yaw_rate': 10.0, 'publish_tf': False,
        }
        cfg = Config()
        for k, v in vars(cfg).items():
            defaults[k] = list(v) if isinstance(v, tuple) else v
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.v = {k: self.get_parameter(k).value for k in defaults}
        self.cfg = Config(**{k: self.v[k] for k in vars(cfg)})
        self.origin_T_map = np.linalg.inv(transform(self.v['map_to_origin_xyz'],
                                                      rpy_matrix(self.v['map_to_origin_rpy'])))
        self.body_T_base = transform(self.v['body_to_base_xyz'], rpy_matrix(self.v['body_to_base_rpy']))
        self.pose_pub = self.create_publisher(PoseStamped, '/task1/fused/pose', 20)
        self.odom_pub = self.create_publisher(Odometry, '/task1/fused/odom', 20)
        self.raw_pub = self.create_publisher(PoseStamped, '/task1/baseline/lidar', 20)
        self.naive_pub = self.create_publisher(PoseStamped, '/task1/baseline/naive', 20)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/task1/diagnostics', 10)
        self.tf_pub = TransformBroadcaster(self) if self.v['publish_tf'] else None
        self.create_subscription(Odometry, self.v['wheel_topic'], self.wheel_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.v['lidar_topic'], self.lidar_callback, qos_profile_sensor_data)
        self.create_service(Trigger, '/task1/reset', self.reset_service)
        self.create_timer(0.1, self.diagnostics)
        self.clock_last = self.get_clock().now().nanoseconds * 1e-9
        self.resets = 0
        self.input_rejections = 0
        self.reset('startup')
        self.get_logger().info('Waiting for measured wheel odometry and acquisition-stamped map LiDAR odometry.')

    def reset(self, reason):
        self.core = FixedLagEKF(self.cfg)
        self.previous = None
        self.pending = []
        self.naive = None
        self.roll_pitch = (0.0, 0.0)
        self.attitude_stamp = -math.inf
        self.stationary_since = None
        self.stationary = False
        self.last_wheel_wall = None
        self.last_wheel_stamp = None
        self.reason = reason
        self.resets += 1

    def reset_service(self, request, response):
        self.reset('operator_reset')
        response.success, response.message = True, 'Reset; waiting for new wheel history and a valid LiDAR observation.'
        return response

    def now_sec(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self.clock_last - 0.05:
            self.reset('ros_clock_moved_backwards')
        self.clock_last = now
        return now

    def reject(self, reason):
        self.reason = reason
        self.input_rejections += 1

    def wheel_callback(self, msg):
        now, stamp = self.now_sec(), seconds(msg.header.stamp)
        if msg.header.frame_id != self.v['wheel_parent_frame'] or msg.child_frame_id != self.v['wheel_child_frame']:
            return self.reject('wheel_frame_mismatch')
        if stamp <= 0 or stamp > now + 0.05 or now-stamp > self.v['wheel_timeout_sec']:
            return self.reject('wheel_acquisition_stamp_invalid_or_stale')
        try:
            x, _ = matrix_state(pose_matrix(msg.pose.pose))
        except ValueError:
            return self.reject('invalid_wheel_quaternion')
        current = np.array([x[0], x[1], x[3]])
        if not np.isfinite(current).all():
            return self.reject('invalid_wheel_position')
        if self.previous is None:
            self.previous = (stamp, current)
            self.last_wheel_stamp, self.last_wheel_wall = stamp, time.monotonic()
            return
        previous_stamp, previous_pose = self.previous
        dt = stamp - previous_stamp
        if dt <= 1e-8:
            return self.reject('duplicate_or_out_of_order_wheel')
        if dt > self.cfg.max_wheel_gap_sec:
            self.reset('wheel_gap_requires_reinitialization')
            self.previous = (stamp, current)
            return
        velocity = se2_log(relative_se2(previous_pose, current), dt)
        if math.hypot(*velocity[:2]) > self.v['max_speed_mps'] or abs(velocity[2]) > self.v['max_yaw_rate']:
            self.reset('wheel_jump_requires_reinitialization')
            self.previous = (stamp, current)
            return
        slow = (math.hypot(*velocity[:2]) < self.v['stationary_speed_mps']
                and abs(velocity[2]) < self.v['stationary_yaw_rate'])
        if not slow:
            self.stationary_since = None
        elif self.stationary_since is None:
            self.stationary_since = previous_stamp
        self.stationary = slow and stamp-self.stationary_since >= self.v['stationary_dwell_sec']
        motion = Motion(previous_stamp, stamp, velocity, self.stationary)
        self.core.add_motion(motion)
        if self.naive is not None:
            self.naive, _ = self.core.predict(self.naive, np.zeros((4, 4)), motion, dt)
        self.previous = (stamp, current)
        self.last_wheel_stamp, self.last_wheel_wall = stamp, time.monotonic()
        self.process_pending()
        if self.core.initialized and stamp-self.core.last_lidar_t <= self.v['lidar_timeout_sec']:
            self.reason = 'tracking_stationary' if self.stationary else 'tracking_moving'
            self.publish(stamp, velocity)
        else:
            self.reason = 'waiting_for_lidar' if not self.core.initialized else 'accepted_lidar_timed_out'

    def lidar_callback(self, msg):
        now, stamp = self.now_sec(), seconds(msg.header.stamp)
        if msg.header.frame_id != self.v['lidar_parent_frame'] or msg.child_frame_id != self.v['lidar_child_frame']:
            return self.reject('lidar_frame_mismatch')
        if stamp <= 0 or stamp > now+0.05 or now-stamp > self.cfg.history_sec:
            return self.reject('lidar_acquisition_stamp_invalid_or_stale')
        try:
            value, rp = matrix_state(self.origin_T_map @ pose_matrix(msg.pose.pose) @ self.body_T_base)
        except ValueError:
            return self.reject('invalid_lidar_quaternion')
        if not np.isfinite(value).all():
            return self.reject('invalid_lidar_position')
        self.raw_pub.publish(self.make_pose(value, stamp, rp))
        self.pending.append((stamp, value, rp, time.monotonic()))
        self.pending.sort(key=lambda p: p[0])
        self.pending = self.pending[-100:]
        self.process_pending()

    def process_pending(self):
        keep = []
        for stamp, value, rp, received in self.pending:
            if self.core.latest_t is None or stamp > self.core.latest_t + 1e-8:
                if time.monotonic()-received < self.v['wheel_timeout_sec']:
                    keep.append((stamp, value, rp, received))
                else:
                    self.reject('lidar_wait_for_wheel_expired')
                continue
            was_ready = self.core.initialized
            accepted = self.core.add_lidar(stamp, value)
            # Deliberately naive benchmark: reset the CURRENT pose to a delayed
            # observation. This is an illustrative comparator, not repo 2 code.
            self.naive = value.copy() if was_ready else (self.core.state.copy() if accepted else self.naive)
            if accepted and stamp >= self.attitude_stamp:
                self.roll_pitch, self.attitude_stamp = rp, stamp
            if not accepted:
                self.reason = self.core.last_reason
        self.pending = keep

    def make_pose(self, value, stamp, rp):
        msg = PoseStamped()
        msg.header.frame_id = self.v['output_frame']
        nanoseconds = int(round(stamp * 1e9))
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(nanoseconds, 10**9)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [float(v) for v in value[:3]]
        q = rpy_quat(*rp, value[3])
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        return msg

    def publish(self, stamp, velocity):
        pose = self.make_pose(self.core.state, stamp, self.roll_pitch)
        self.pose_pub.publish(pose)
        msg = Odometry()
        msg.header, msg.pose.pose = pose.header, pose.pose
        msg.child_frame_id = self.v['output_child_frame']
        C = np.zeros((6, 6))
        for i, ii in enumerate((0, 1, 2, 5)):
            for j, jj in enumerate((0, 1, 2, 5)):
                C[ii, jj] = self.core.P[i, j]
        C[3, 3] = C[4, 4] = 0.02**2
        msg.pose.covariance = C.ravel().tolist()
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = map(float, velocity[:2])
        msg.twist.twist.angular.z = float(velocity[2])
        msg.twist.covariance = np.diag([0.02**2, 0.02**2, 1e6, 1e6, 1e6, 0.03**2]).ravel().tolist()
        self.odom_pub.publish(msg)
        if self.naive is not None:
            self.naive_pub.publish(self.make_pose(self.naive, stamp, self.roll_pitch))
        if self.tf_pub:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = pose.header, self.v['output_child_frame']
            p = pose.pose.position
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
            tf.transform.rotation = pose.pose.orientation
            self.tf_pub.sendTransform(tf)

    def diagnostics(self):
        now = self.now_sec()
        wheel_age = math.inf if self.last_wheel_stamp is None else now-self.last_wheel_stamp
        lidar_age = math.inf if self.core.last_lidar_t is None else now-self.core.last_lidar_t
        wall_age = math.inf if self.last_wheel_wall is None else time.monotonic()-self.last_wheel_wall
        healthy = (self.core.initialized and wheel_age <= self.v['wheel_timeout_sec']
                   and wall_age <= self.v['wheel_timeout_sec'] and lidar_age <= self.v['lidar_timeout_sec'])
        if wall_age > self.v['wheel_timeout_sec']:
            self.reason = 'wheel_timed_out'
        status = DiagnosticStatus(name='task1_fusion', hardware_id='wheel_lidar', message=self.reason)
        # diagnostic_msgs uses IDL byte; Python representation differs across
        # ROS releases. Follow the installed message type instead of assuming int.
        level = 0 if healthy else 1
        status.level = bytes([level]) if isinstance(status.level, bytes) else level
        fields = {'wheel_age_sec': wheel_age, 'lidar_age_sec': lidar_age,
                  'stationary': self.stationary, 'accepted_lidar': self.core.accepted,
                  'rejected_lidar': self.core.rejected, 'input_rejections': self.input_rejections,
                  'last_lidar_result': self.core.last_reason, 'last_nis': self.core.last_nis,
                  'history_intervals': len(self.core.motions), 'reset_count': self.resets}
        status.values = [KeyValue(key=k, value=str(v)) for k, v in fields.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
