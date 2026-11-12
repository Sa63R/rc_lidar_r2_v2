import socket
import struct
import numpy as np
import yaml
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from .can_protocol import Decoder
from .math_utils import rpy_quat


class CanBridge(Node):
    def __init__(self):
        super().__init__('task1_can_bridge')
        self.declare_parameter('protocol_file', '')
        self.declare_parameter('can_interface', 'can0')
        with open(self.get_parameter('protocol_file').value, encoding='utf-8') as f:
            self.protocol = yaml.safe_load(f)
        self.decoder = Decoder(self.protocol)
        self.pub = self.create_publisher(Odometry, '/wheel/odom', 50)
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        can_id = int(self.protocol['can_id'])
        if self.protocol.get('extended_id', False):
            can_id |= 0x80000000
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                             struct.pack('=II', can_id, 0xDFFFFFFF))
        self.sock.bind((self.get_parameter('can_interface').value,))
        self.sock.setblocking(False)
        self.failed = False
        self.last_error_log = -1e9
        self.create_timer(0.001, self.poll)
        self.get_logger().info('CAN receive only; publish /wheel/odom using confirmed protocol.')

    def poll(self):
        if self.failed:
            return
        for _ in range(200):
            try:
                frame = self.sock.recv(72)
            except BlockingIOError:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if len(frame) not in (16, 72):
                continue
            can_id, length = struct.unpack_from('=IB', frame)
            if can_id & 0x60000000 or length > len(frame)-8:
                continue
            try:
                stamp, pose, velocity = self.decoder.decode(frame[8:8+length], now)
            except (ValueError, KeyError, struct.error) as e:
                # Stop instead of integrating across an unknown reset or gap.
                self.failed = True
                self.get_logger().error(f'CAN decode stopped: {e}. Correct the input and restart bridge; fusion will reinitialize.')
                return
            msg = Odometry()
            ns = int(round(stamp*1e9))
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
            msg.header.frame_id = 'wheel_odom'
            msg.child_frame_id = self.protocol.get('base_frame', 'robot_base')
            msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, pose[:2])
            q = rpy_quat(0, 0, pose[2])
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
            msg.twist.twist.linear.x, msg.twist.twist.linear.y = map(float, velocity[:2])
            msg.twist.twist.angular.z = float(velocity[2])
            msg.pose.covariance = np.diag([0.01**2, 0.01**2, 1e6, 1e6, 1e6, 0.02**2]).ravel().tolist()
            msg.twist.covariance = np.diag([0.02**2, 0.02**2, 1e6, 1e6, 1e6, 0.03**2]).ravel().tolist()
            self.pub.publish(msg)

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
