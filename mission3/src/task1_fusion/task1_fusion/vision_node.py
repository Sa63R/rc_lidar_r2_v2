"""Vision-facing stamped corner interface and explicit validity heartbeat."""
import json
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from .vision import VisionFeed


class VisionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('task3_vision', **kwargs)
        defaults = {
            'input_topic': '/task3/corner/odom', 'input_parent_frame': 'map',
            'input_child_frame': 'task3_corner', 'diagnostics_topic': '/task3/diagnostics',
            'input_timeout_sec': .5, 'future_tolerance_sec': .05,
            'allow_unverified_extrinsics': False, 'reference_id': '', 'data_source': 'live',
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.v = {k: self.get_parameter(k).value for k in defaults}
        if self.v['input_topic'] in ('/vision/pose', '/vision/odom', '/vision/state'):
            raise ValueError('vision input must differ from output topics')
        self.feed = VisionFeed(self.v['input_timeout_sec'], self.v['future_tolerance_sec'],
                               allow_unverified_extrinsics=self.v['allow_unverified_extrinsics'],
                               reference_id=self.v['reference_id'], data_source=self.v['data_source'])
        self.last_clock = None
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/pose', 20)
        self.odom_pub = self.create_publisher(Odometry, '/vision/odom', 20)
        self.state_pub = self.create_publisher(String, '/vision/state', 20)
        self.create_subscription(Odometry, self.v['input_topic'], self.pose, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, self.v['diagnostics_topic'], self.diagnostics,
                                 qos_profile_sensor_data)
        self.create_timer(.1, self.state)
        self.get_logger().info('Vision interface: /vision/state contains x/y/yaw, source time, '
                               'frame and validity. Reference point is CORNER, not chassis center.')

    def now(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.feed.reset()
        self.last_clock = now
        return now

    def diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'task3_corner':
                values = {entry.key: entry.value for entry in status.values}
                self.feed.upstream(values.get('calibration_confirmed', '').lower() == 'true',
                                   status.message in ('tracking', 'tracking_unverified_extrinsics'),
                                   time.monotonic())

    def pose(self, msg):
        if (msg.header.frame_id != self.v['input_parent_frame']
                or msg.child_frame_id != self.v['input_child_frame']):
            self.feed.rejected += 1
            self.feed.last_rejection = 'input_frame_mismatch'
            return
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        now, receipt = self.now(), time.monotonic()
        if not self.feed.accept(stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                                msg.header.frame_id, now, receipt):
            return
        if self.feed.snapshot(now, receipt)['valid']:
            self.pose_pub.publish(PoseStamped(header=msg.header, pose=msg.pose.pose))
            self.odom_pub.publish(msg)
        self.state()

    def state(self):
        snapshot = self.feed.snapshot(self.now(), time.monotonic())
        self.state_pub.publish(String(data=json.dumps(snapshot, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
